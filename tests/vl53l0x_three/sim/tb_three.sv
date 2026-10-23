`timescale 1ns/1ps
module tb_three;
 reg clk=0; always #10 clk=~clk;
 reg reset_n=0; tri1 [2:0] scl,sda,xshut;
 wire uart,oscl; tri1 osda; wire [6:0] motor;
 reg [2:0] present=7,never_ready=0,nack_data=0;
 reg [15:0] distance_value[0:2];
 wire [3:0] raw_status[0:2];
 assign raw_status[0]=3; assign raw_status[1]=5; assign raw_status[2]=7;
 integer shots[0:2],polls[0:2],blocks[0:2],resets[0:2];
 reg completed=0;
 vl53l0x_three_top #(.CLK_HZ(100000),.I2C_HZ(5000),.BAUD(10000),
  .RESET_CYCLES(10),.BOOT_CYCLES(30),.POLL_CYCLES(30),.MAX_POLLS(100),
  .TIMEOUT_CYCLES(300),.ROUND_CYCLES(50000),
  .OLED_I2C_HZ(5000),.OLED_POWERUP_CYCLES(10),.OLED_WAIT_CYCLES(100)) dut(
  .clk_50m(clk),.reset_n(reset_n),.imu_gnd_reserved(1'b0),
  .tof_front_gpio1(1'b1),.tof_left_gpio1(1'b0),.tof_right_gpio1(1'b1),
  .tof_front_scl(scl[0]),.tof_front_sda(sda[0]),.tof_front_xshut_n(xshut[0]),
  .tof_left_scl(scl[1]),.tof_left_sda(sda[1]),.tof_left_xshut_n(xshut[1]),
  .tof_right_scl(scl[2]),.tof_right_sda(sda[2]),.tof_right_xshut_n(xshut[2]),
  .uart_tx(uart),.oled_scl(oscl),.oled_sda(osda),
  .PWMA(motor[0]),.AIN1(motor[1]),.AIN2(motor[2]),.PWMB(motor[3]),
  .BIN1(motor[4]),.BIN2(motor[5]),.STBY(motor[6]));
 genvar g;
 generate for(g=0;g<3;g++) begin:m
  vl53l0x_model model(.scl(scl[g]),.sda(sda[g]),.xshut_n(xshut[g]),
   .present(present[g]),.nack_data(nack_data[g]),.never_ready(never_ready[g]),
   .distance_mm(distance_value[g]),.status(raw_status[g]),.ready_after(3),
   .shots(shots[g]),.polls(polls[g]),.read_blocks(blocks[g]),.reset_count(resets[g]));
 end endgenerate
 always @(negedge clk) if(reset_n && motor!==0) $fatal(1,"Motor outputs not held low");
 integer expected_channel=0;
 always @(negedge clk) if(dut.event_valid) begin
  if(dut.event_channel!=expected_channel) $fatal(1,"Round-robin order incorrect");
  expected_channel=(expected_channel+1)%3;
 end
 function automatic string dec5(input integer v);
  string s; begin s=$sformatf("%0d",v); while(s.len()<5) s={"0",s}; return s; end
 endfunction
 integer report_count[0:2]; integer startup_count=0;
 string line="",expected,label_text; reg [7:0] ch; integer k,side,seq,mm,er,ok,st;
 // Decode the physical UART pin at 10 clocks per bit, including stop bit.
 initial begin
  for(k=0;k<3;k++) report_count[k]=0;
  forever begin
   @(negedge uart); repeat(5) @(posedge clk);
   if(uart!==0) $fatal(1,"UART start bit");
   for(k=0;k<8;k++) begin repeat(10) @(posedge clk); ch[k]=uart; end
   repeat(10) @(posedge clk); if(uart!==1) $fatal(1,"UART stop bit");
   if(ch==10) begin
    if(startup_count==0) begin
     expected="VL53L0X DIAG: supplied example only; full initialization/calibration missing.\015";
     if(line!=expected) $fatal(1,"Missing startup disclaimer: %s length=%0d expected=%0d last=%h/%h",line,line.len(),expected.len(),line.getc(line.len()-1),expected.getc(expected.len()-1));
     startup_count++;
    end else begin
     case(line.substr(0,4)) "FRONT":side=0; "LEFT ":side=1; "RIGHT":side=2;
      default:$fatal(1,"UART label %s",line); endcase
     seq=report_count[side]+1;
     label_text=(side==0)?"FRONT":((side==1)?"LEFT ":"RIGHT");
     mm=1234+1111*side; er=0; ok=1; st=3+2*side;
     if(seq==2 && side==0) begin er=2; ok=0; end
     if(seq==2 && side==1) begin er=1; ok=0; end
     expected=$sformatf("%s seq=%08X mm=%s id=%02X rev=%02X pre=%02X final=%02X status=0%1X ambient=%s signal=%s gpio=%1d valid=%1d err=%02X dbg=%04X pad=7 rd=%02X raw=%02X000000000001230456%04X\015",
      label_text,seq,dec5(mm),(seq==2 && side==1)?0:238,(seq==2 && side==1)?0:16,
      (seq==2 && side==1)?0:6,(seq==2 && side==1)?0:4,st,dec5(291),dec5(1110),side==1?0:1,ok,er,(seq==2 && side==1)?16'h1027:0,
      (seq==2 && side==1)?0:31,(seq==2 && side==1)?0:((st<<3)|((er==2)?0:1)),(seq==2 && side==1)?0:mm);
     if(seq==2 && side==1) begin
      expected=$sformatf("%s seq=%08X mm=----- id=-- rev=-- pre=-- final=-- status=-- ambient=----- signal=----- gpio=0 valid=0 err=01 dbg=1027 pad=7 rd=00 raw=------------------------\015",
       label_text,seq);
     end
     if(line.toupper()!=expected.toupper()) $fatal(1,"UART mismatch\nactual   %s\nexpected %s",line,expected);
     report_count[side]++;
    end
    line="";
   end else line={line,ch};
  end
 end
 integer j;
 initial begin
  distance_value[0]=1234; distance_value[1]=2345; distance_value[2]=3456;
  repeat(5) @(negedge clk); reset_n=1;
  wait(dut.seen==7); @(negedge clk);
  if(dut.valid!=7 || dut.distances!={16'd3456,16'd2345,16'd1234}) $fatal(1,"Three distances / byte order");
  if(dut.models!=24'heeeeee || dut.ambients!={3{16'h0123}} || dut.signals!={3{16'h0456}})
   $fatal(1,"Diagnostic register parsing");
  // Verify source register pointer order, including delayed ready polling.
  for(j=0;j<3;j++) begin
   case(j)
    0: if(m[0].model.register_log[0]!='hc2 || m[0].model.register_log[1]!='hc0 ||
          m[0].model.register_log[2]!='h50 || m[0].model.register_log[3]!='h70 ||
          m[0].model.register_log[4]!=0 || m[0].model.register_log[5]!='h14)
         $fatal(1,"Source transaction order");
   endcase
   if(polls[j]!=4 || blocks[j]!=1) $fatal(1,"Delayed readiness or block read failed");
  end
  wait(report_count[0]==1 && report_count[1]==1 && report_count[2]==1);
  $display("PASS normal three-channel measurements, diagnostics, UART, order");
  @(negedge clk); never_ready[0]=1; present[1]=0;
  wait(dut.sequences[64+:32]>=2); @(negedge clk);
  if(dut.errors!=24'h000102 || dut.valid!=4) $fatal(1,"Fault isolation %h %b",dut.errors,dut.valid);
  if(polls[0]!=101) $fatal(1,"Must poll exactly 100 times then read diagnostic block: %0d",polls[0]);
  wait(report_count[0]==2 && report_count[1]==2 && report_count[2]==2);
  $display("PASS 100-poll timeout, missing device, right-channel progress, UART errors");
  @(negedge clk); never_ready=0; present=7;
  wait(dut.sequences[64+:32]>=3); @(negedge clk);
  if(dut.errors!=0 || dut.valid!=7 || resets[0]<1 || resets[1]<1) $fatal(1,"Fault recovery");
  wait(report_count[0]==3 && report_count[1]==3 && report_count[2]==3);
  $display("PASS per-channel XSHUT recovery");
  // Reset during a transaction; all outputs and the scheduler restart.
  wait(scl[0]===0); @(negedge clk); reset_n=0; expected_channel=0;
  repeat(5) @(negedge clk);
  if(xshut!==0 || scl!==7 || sda!==7) $fatal(1,"Reset must shut sensors down and release I2C");
  // UART verification above is complete; do not parse the deliberately cut frame.
  completed=1; $display("PASS reset during transfer and safe outputs"); $finish;
 end
 initial begin #100000000; $fatal(1,"Integration watchdog timeout seen=%b seq=%h",dut.seen,dut.sequences); end
endmodule
