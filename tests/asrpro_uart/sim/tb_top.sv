`timescale 1ns/1ps
module tb_top;
 reg clk=0;always #10 clk=~clk;
 reg rst_n=0,rx=1,pc_rx=1,key_step=1,key_fault=1;
 wire tx,pc_tx,ready,busy,error_led,rx_led;wire [6:0] motors;
 tri fscl,fsda,fx,lscl,lsda,lx,rscl,rsda,rxshut;
 asrpro_uart_top #(.KEY_CYCLES(50)) dut(.clk_50m(clk),.reset_n(rst_n),
  .asr_uart_rx(rx),.asr_uart_tx(tx),.uart_rx(pc_rx),.uart_tx(pc_tx),
  .key_step_n(key_step),.key_fault_n(key_fault),.led_ready(ready),.led_busy(busy),
  .led_error(error_led),.led_rx(rx_led),.PWMA(motors[6]),.AIN1(motors[5]),
  .AIN2(motors[4]),.PWMB(motors[3]),.BIN1(motors[2]),.BIN2(motors[1]),.STBY(motors[0]),
  .imu_gnd_reserved(1'b0),.imu_spi_sck(1'b0),.imu_spi_mosi(1'b0),.imu_spi_miso(1'b0),
  .imu_cs_n(1'b1),.imu_int1(1'b0),.imu_int2(1'b0),
  .tof_front_gpio1(1'b1),.tof_left_gpio1(1'b1),.tof_right_gpio1(1'b1),
  .tof_front_scl(fscl),.tof_front_sda(fsda),.tof_front_xshut_n(fx),
  .tof_left_scl(lscl),.tof_left_sda(lsda),.tof_left_xshut_n(lx),
  .tof_right_scl(rscl),.tof_right_sda(rsda),.tof_right_xshut_n(rxshut));
 task check(input bit ok,input string msg);if(!ok)$fatal(1,"FAIL %s t=%0t",msg,$time);endtask
 always @(posedge clk) begin
  check(motors===7'b0,"all motor outputs permanently disabled");
  check({fscl,fsda,fx,lscl,lsda,lx,rscl,rsda,rxshut}===9'bz,"ToF outputs high impedance");
 end
 reg [7:0] out_bytes[0:8191];integer received=0;reg [7:0] byte_value;
 initial forever begin
  @(negedge tx);repeat(651) @(posedge clk);#1;
  for(integer i=0;i<8;i++) begin byte_value[i]=tx;repeat(434) @(posedge clk);#1;end
  check(tx==1,"ASR feedback stop");out_bytes[received]=byte_value;received++;
 end
 string log_line="";integer log_lines=0;reg [7:0] pc_byte;
 integer lc,la,ls,lg,lr,lb,lt,lo,ld,ll,fields;bit log_active;
 initial forever begin
  @(negedge pc_tx);log_active=1;repeat(651) @(posedge clk);#1;
  for(integer i=0;i<8;i++) begin
   pc_byte[i]=pc_tx;if(!rst_n)log_active=0;
   repeat(434) @(posedge clk);#1;
  end
  if(log_active && rst_n) begin
   check(pc_tx==1,"debug stop");
   log_line={log_line,pc_byte};
   if(pc_byte==10) begin
    fields=$sscanf(log_line,"TEST CMD=%h ACK=%h STATE=%h GEAR=%h RX=%h BAD=%h TMO=%h OVF=%h DBG=%h LOG=%h",
     lc,la,ls,lg,lr,lb,lt,lo,ld,ll);
    // A reset can truncate the previous line; otherwise all ten fields are required.
    check(fields==10,"diagnostic ASCII snapshot format");
    check(lg>=0 && lg<=2,"diagnostic gear");
    log_lines++;log_line="";
   end
  end else log_line="";
 end
 task automatic send_byte(input [7:0] b,input bit pc=0);
  @(negedge clk);if(pc)pc_rx=0;else rx=0;repeat(434) @(negedge clk);
  for(integer i=0;i<8;i++) begin if(pc)pc_rx=b[i];else rx=b[i];repeat(434) @(negedge clk);end
  if(pc)pc_rx=1;else rx=1;repeat(434) @(negedge clk);
 endtask
 task expect_frame(input integer offset,input [7:0] t,input [7:0] c);
  check(out_bytes[offset]==8'hAA && out_bytes[offset+1]==8'h55 && out_bytes[offset+2]==t &&
   out_bytes[offset+3]==c && out_bytes[offset+4]==(t+c) && out_bytes[offset+5]==13,"feedback frame");
 endtask
 task wait_bytes(input integer n);
  wait(received>=n);repeat(250) @(negedge clk);check(received==n,"no extra feedback");
 endtask
 task command(input [7:0] c,input [7:0] a,input [7:0] s);
  integer offset;offset=received;
  $display("TEST command %02h expected ack=%02h state=%02h offset=%0d",c,a,s,offset);
  send_byte('hAA);send_byte('h55);send_byte(1);send_byte(c);send_byte(8'h01+c);send_byte(13);
  wait_bytes(offset+12);expect_frame(offset,2,a);expect_frame(offset+6,3,s);
 endtask
 function [7:0] digit(input [3:0] v);digit=(v<10)?48+v:55+v;endfunction
 task state_line(input [7:0] s,input bit expect_change=1);
  string text_line;integer offset;offset=received;
  $display("TEST injected state %02h offset=%0d",s,offset);
  text_line={"STATE ",digit(s[7:4]),digit(s[3:0]),8'h0D,8'h0A};
  for(integer i=0;i<text_line.len();i++) send_byte(text_line[i],1);
  if(expect_change) begin wait_bytes(offset+6);expect_frame(offset,3,s);end
  else begin repeat(10000) @(negedge clk);check(received==offset,"invalid/same state no feedback");end
 endtask
 integer offset,old_good;
 reg completed=0;
 initial begin
  repeat(5) @(negedge clk);rst_n=1;repeat(100) @(negedge clk);
  check(ready && !busy && received==0,"reset idle, no unsolicited ASR transmission");
  command(3,0,0);command(1,0,1);command(1,1,1);command('h10,1,1);
  // Bounce below debounce threshold cannot advance the state.
  key_step=0;repeat(20) @(negedge clk);key_step=1;repeat(80) @(negedge clk);
  check(dut.status==1,"key bounce rejected");
  offset=received;key_step=0;repeat(80) @(negedge clk);key_step=1;
  wait_bytes(offset+6);expect_frame(offset,3,2);
  check(dut.status==2,"key advances search to approach");
  offset=received;key_step=0;repeat(80) @(negedge clk);key_step=1;
  wait_bytes(offset+6);expect_frame(offset,3,3);
  command('h14,0,3);command('h14,0,3);check(dut.gear==2,"speed high saturation");
  command('h15,0,3);command('h15,0,3);command('h15,0,3);check(dut.gear==0,"speed low saturation");
  command('h10,0,'h10);command(3,0,'h10);command(2,0,4);
  command('h11,0,'h10);command(2,0,4);command('h12,0,'h10);command(2,0,4);
  command('h13,0,'h10);command(2,0,4);
  state_line(0);state_line(1);state_line(2);state_line(3);state_line(4);state_line(5);
  state_line('h10);state_line('h11);state_line('h11,0);
  state_line('hFF,0);check(dut.debug_control_errors==1,"unknown injected status rejected");
  // Invalid and overlong ASCII lines never execute their suffix.
  begin
   string invalid_line;invalid_line="XSTATE 00\n";
   for(integer i=0;i<invalid_line.len();i++) send_byte(invalid_line[i],1);
  end
  repeat(10) @(negedge clk);check(dut.debug_parse_errors==1,"overlength line rejected");
  // Concurrent physical debug reception and ASR command reception.
  fork
   begin
    string concurrent_line;concurrent_line="STATE 11\n";
    for(integer i=0;i<concurrent_line.len();i++) send_byte(concurrent_line[i],1);
   end
   begin command(3,0,'h11);end
  join
  state_line(6);state_line(0,0);command(1,2,6);command(2,0,6);
  check(!ready && error_led,"fault latched, stop cannot clear");
  // Drain diagnostics then reset and test the actual fault key.
  wait(!dut.diagnostics.active && dut.diagnostics.used==0);repeat(4500) @(negedge clk);
  check(lc==2 && la==0 && ls==6 && lo==0,"decoded diagnostic command/ack/state/overflow snapshot");
  rst_n=0;repeat(5) @(negedge clk);rst_n=1;repeat(100) @(negedge clk);
  check(dut.status==0 && dut.gear==1,"reset clears state and speed");
  offset=received;key_fault=0;repeat(80) @(negedge clk);key_fault=1;
  wait_bytes(offset+6);expect_frame(offset,3,6);
  // Silence has no heartbeat semantics and cannot change state.
  old_good=dut.good;repeat(1050000) @(negedge clk);
  check(dut.status==6 && dut.good==old_good,"21ms silence does not create commands");
  check(dut.overflows==0,"normal interactive ASR traffic never overflows");
  check(log_lines>0,"physical debug UART decoded");
  check(lc==0 && la==255 && ls==6 && lo==0,"unsolicited fault diagnostic after reset");
  // Full-rate physical RX can overrun feedback bandwidth (one 6-byte command
  // requests 12 response bytes). Even under this load STOP must execute.
  wait(!dut.diagnostics.active && dut.diagnostics.used==0);repeat(4500) @(negedge clk);
  rst_n=0;repeat(5) @(negedge clk);rst_n=1;repeat(100) @(negedge clk);
  offset=received;
  for(integer n=0;n<40;n++) begin
   send_byte('hAA);send_byte('h55);send_byte(1);send_byte(1);send_byte(2);send_byte(13);
  end
  send_byte('hAA);send_byte('h55);send_byte(1);send_byte(2);send_byte(3);send_byte(13);
  repeat(20) @(negedge clk);
  check(dut.good==41 && dut.status==4 && dut.overflows>0,"STOP executes despite physical RX/TX FIFO overload");
  wait(!dut.feedback.active && dut.feedback.used==0 && dut.feedback.ready);
  repeat(500) @(negedge clk);
  check((received-offset)%12==0,"overload cannot split response/state pairs");
  for(integer n=offset;n<received;n+=6) begin
   check(out_bytes[n]=='hAA && out_bytes[n+1]=='h55 &&
    out_bytes[n+4]==(out_bytes[n+2]+out_bytes[n+3]) && out_bytes[n+5]==13,"all overload output frames valid");
  end
  $display("PASS top: real clock/baud, every command/state, debug ASCII, concurrent RX, keys, silence, safety");
  completed=1;$finish;
 end
 initial begin #600000000;$fatal(1,"top watchdog state=%h rx=%0d good=%0d dbg=%0d step=%b",dut.status,received,dut.good,dut.all_debug,dut.step);end
endmodule
