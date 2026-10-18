`timescale 1ns/1ps
module tb_robot_system;
 reg clk=0,rst=0,key=1,estop=1,rx=1,miso=0;
 always #10 clk=~clk;
 reg [1:0] abl=0,abr=0;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n;
 wire ready_led,run_led,fault_led,hb_led;
 reg completed=0,init_bad=0,model_on=0;
 reg [7:0] model_id=8'h24,model_error=0;
 robot_system_top #(.CLK_HZ(1000),.UART_BAUD(50),.UART_BIT_CYCLES(20),
 .WATCHDOG_CYCLES(5000),.WINDOW_CYCLES(10000),.DEBOUNCE_CYCLES(8),
 .RUN_CYCLES(500000),.STARTUP_CYCLES(10),.IMU_POWERUP_CYCLES(2),
 .IMU_CONFIG_WAIT_CYCLES(2),.IMU_RETRY_CYCLES(20),.IMU_ID_RETRY_CYCLES(2),
 .IMU_SAMPLE_CYCLES(1000),.IMU_SPI_HALF_PERIOD(1),.IMU_HEALTH_CYCLES(10000),
 .IMU_TIMEOUT_CYCLES(5000)) dut(
 .clk_50m(clk),.reset_n(rst),.key_enable_n(key),.key_estop_n(estop),.uart_rx(rx),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .imu_spi_miso(miso),.imu_gnd_reserved(1'b0),.imu_int1(1'b0),.imu_int2(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.uart_tx(tx),
 .led_ready(ready_led),.led_run(run_led),.led_fault(fault_led),.led_heartbeat(hb_led));
 integer log_count=0,boot_count=0;
 task tick;begin @(posedge clk);#1;end endtask
 task cycles(input integer n);repeat(n)tick;endtask
 task off;begin #1;if({a,a1,a2,b,b1,b2,st}!==0)$fatal(1,"unsafe motor outputs");end endtask
 task byte_send(input [7:0] data,input stop);integer i;begin
 @(negedge clk);rx=0;cycles(20);
 for(i=0;i<8;i=i+1)begin @(negedge clk);rx=data[i];cycles(20);end
 @(negedge clk);rx=stop;cycles(20);@(negedge clk);rx=1;cycles(30);
 end endtask
 reg hb_on=0,hb_busy=0;
 initial forever begin
 wait(hb_on);hb_busy=1;byte_send(8'hA5,1);hb_busy=0;cycles(1000);
 end
 task stop_hb;begin hb_on=0;wait(!hb_busy);end endtask
 task reset_system;begin
 stop_hb;model_on=0;@(negedge clk);rst=0;key=1;estop=1;abl=0;abr=0;
 model_id=8'h24;model_error=0;init_bad=0;cycles(50);rst=1;
 end endtask
 task boot;begin
 wait(ready_led);cycles(2500);off;
 if(boot_count!=4 || log_count!=4)$fatal(1,"boot log count %0d/%0d",boot_count,log_count);
 if(dut.imu_sample!==96'h7fff800056784000fffe1234)$fatal(1,"retained six-axis sample");
 end endtask
 task start;begin
 hb_on=1;wait(hb_led);cycles(40);@(negedge clk);key=0;
 wait(dut.motion.run_request);key=1;wait(st);
 end endtask
 // Bit-accurate UART decoder; rejects malformed or repeated event lines.
 reg [7:0] got,line[0:7];integer bitno,idx=0;
 reg [15:0] seen_log=0;integer event_index;
 always @(negedge rst) begin idx=0;log_count=0;boot_count=0;seen_log=0;end
 always @(negedge tx) begin: capture
 if(rst) begin
  #600;
  for(bitno=0;bitno<8;bitno=bitno+1)begin
   if(!rst)disable capture;got[bitno]=tx;#400;
  end
  if(!rst)disable capture;
  if(tx!==1)$fatal(1,"UART stop");
  line[idx]=got;
  if(idx==7) begin
   if(line[1]!=32 || line[3]!=32 || line[6]!=13 || line[7]!=10)$fatal(1,"UART line format");
   event_index=-1;
   if(line[0]=="S")begin
    boot_count=boot_count+1;
    case(line[2]) "M":event_index=0;"N":event_index=1;"W":event_index=2;"I":event_index=3;endcase
   end else if(line[0]=="E")begin
    case(line[2])
     "W":event_index=12;"L":event_index=13;"R":event_index=14;"X":event_index=15;
     "I":case({line[4],line[5]})
      "04":event_index=4;"05":event_index=5;"06":event_index=6;"07":event_index=7;
      "0A":event_index=8;"08":event_index=9;"09":event_index=10;"FF":event_index=11;
     endcase
    endcase
   end
   if(event_index<0 || seen_log[event_index])$fatal(1,"unknown/duplicate log %c %c %c%c",line[0],line[2],line[4],line[5]);
   seen_log[event_index]=1;log_count=log_count+1;idx=0;
  end else idx=idx+1;
 end
 end
 // Both independently loaded wheel models, reusing the validated ground test plant.
    reg [7:0] tx_shift, rx_shift;
    // The C driver clocks one dummy byte for the SPI turnaround and then
    // clocks the first requested register byte.  Keep both bytes in the
    // model so the DUT's read_buffer indexing is checked against the real
    // transaction.
    reg [55:0] response_shift;
    reg [3:0] bit_count;
    reg [7:0] command;
    integer config_bytes;
    integer write_count;
    reg [7:0] config_first;
    reg [7:0] config_last;
    integer config_uploads=0;
    reg [7:0] expected_config [0:8191];
    initial $readmemh("src/imu/imu660ra_config.hex", expected_config);
    initial begin
        #300000000;
        $fatal(1, "timeout waiting for initialization");
    end

    always @(posedge cs_n) if(rst && command==8'h5e && config_bytes!=8192)
        $fatal(1,"bad config stream count=%0d",config_bytes);
    always @(negedge cs_n) begin
        bit_count = 0;
        tx_shift = 0;
        rx_shift = 0;
        response_shift = 0;
        command = 0;
        config_bytes = 0;
        write_count = 0;
        config_first = 0;
        config_last = 0;
        miso = 0;
    end

    always @(posedge sck) begin
        if (!cs_n) begin
            #1;
            tx_shift = {tx_shift[6:0], mosi};
            if (bit_count == 7) begin
                if (command == 0) begin
                    command = tx_shift;
                    if (tx_shift == 8'h5E)
                        config_uploads = config_uploads + 1;
                    if (tx_shift == 8'h80)
                        response_shift = {8'h00,model_id,40'd0};
                    else if (tx_shift == 8'h82)
                        response_shift = {8'h00,model_error,40'd0};
                    else if (tx_shift == 8'hA1)
                        response_shift = (init_bad) ? 56'h00020000000000 : 56'h00010000000000;
                    else if (tx_shift == 8'h8C)
                        response_shift = 56'h003412feff0040;
                    else if (tx_shift == 8'h92)
                        response_shift = 56'h0078560080ff7f;
                    else
                        response_shift = 0;
                end else if (command == 8'h5E) begin
                    if (config_bytes >= 8192)
                        $fatal(1, "too many configuration bytes");
                    if (tx_shift !== expected_config[config_bytes])
                        $fatal(1, "configuration mismatch index=%0d actual=%h expected=%h", config_bytes, tx_shift, expected_config[config_bytes]);
                    if (config_bytes == 0)
                        config_first = tx_shift;
                    config_last = tx_shift;
                    config_bytes = config_bytes + 1;
                end else if ((command == 8'h7C) || (command == 8'h59) ||
                             (command == 8'h7D) || (command == 8'h40) ||
                             (command == 8'h42) || (command == 8'h41) ||
                             (command == 8'h43)) begin
                    write_count = write_count + 1;
                end
                bit_count = 0;
            end else begin
                bit_count = bit_count + 1;
            end
        end
    end

    always @(negedge sck) begin
        if (!cs_n) begin
            miso = response_shift[55];
            response_shift = {response_shift[54:0], 1'b0};
        end else begin
            miso = 0;
        end
    end

 integer mc=0,vl=0,vr=0,el=0,er=0;
 always @(negedge clk) if(model_on) begin
  if(mc==0) begin
   vl=(vl+$signed(dut.motion.ramp_l)/16)/2;vr=(vr+$signed(dut.motion.ramp_r)/20)/2;
   el=vl<0 ? -vl:vl;er=vr<0 ? -vr:vr;
  end
  if(mc%500==0) begin
   if(el>0) begin
    if(vl>=0) case(abl) 0:abl=1;1:abl=3;3:abl=2;2:abl=0;endcase
    else case(abl) 0:abl=2;2:abl=3;3:abl=1;1:abl=0;endcase
    el=el-1;
   end
   if(er>0) begin
    if(vr>=0) case(abr) 0:abr=2;2:abr=3;3:abr=1;1:abr=0;endcase
    else case(abr) 0:abr=1;1:abr=3;3:abr=2;2:abr=0;endcase
    er=er-1;
   end
  end
  mc=(mc+1)%10000;
 end
 integer hl=0,hr=0,dl=0,dr=0,periods=0;reg full=0,drive_l=0,drive_r=0;
 always @(negedge clk) begin
  if(dut.motion.ramp_l>625 || dut.motion.ramp_l< -625 || dut.motion.ramp_r>625 || dut.motion.ramp_r< -625) $fatal(1,"dual limit");
  if(!st) begin full=0;hl=0;hr=0;end
  else begin
   if(dut.motion.driver.phase==0) begin
    if(full) begin
     if(hl>625 || hr>625 || (drive_l && hl!=dl) || (drive_r && hr!=dr)) $fatal(1,"dual physical PWM width");
     periods=periods+1;
    end
    full=1;hl=0;hr=0;dl=$signed(dut.motion.driver.command_l);dr=$signed(dut.motion.driver.command_r);
    if(dl<0)dl=-dl;if(dr<0)dr=-dr;drive_l=1;drive_r=1;
   end
   if(dut.motion.driver.left_channel.state!=1)drive_l=0;
   if(dut.motion.driver.right_channel.state!=1)drive_r=0;
   if(a)begin hl=hl+1;if(a1!==!dut.motion.driver.command_l[15] || a2!==dut.motion.driver.command_l[15])$fatal(1,"left direction");end
   if(b)begin hr=hr+1;if(b1!==!dut.motion.driver.command_r[15] || b2!==dut.motion.driver.command_r[15])$fatal(1,"right direction");end
  end
 end

 integer n,base_logs;
 initial begin
 cycles(5);rst=1;cycles(30);key=0;cycles(20);off;key=1;
 boot;
 byte_send(8'h00,1);byte_send(8'hA5,0);cycles(30);
 if(hb_led)$fatal(1,"invalid heartbeat accepted");off;
 start;model_on=1;cycles(450000);
 if(!st || !dut.motion.run_request || dut.motion.pos_l<=0 || dut.motion.pos_r<=0 ||
    $signed(dut.motion.speed_l)<7 || $signed(dut.motion.speed_l)>13 ||
    $signed(dut.motion.speed_r)<7 || $signed(dut.motion.speed_r)>13)
 $fatal(1,"closed-loop feedback %d %d",dut.motion.speed_l,dut.motion.speed_r);
 if(dut.motion.control_l==dut.motion.control_r)$fatal(1,"plants not independent");
 wait(!dut.motion.run_request);model_on=0;off;
 if(log_count!=4)$fatal(1,"periodic telemetry still present");
 cycles(30);start;cycles(4000);if(!st)$fatal(1,"rearm failed");
 stop_hb;wait(dut.wd);cycles(10);off;
 if(dut.motion.wd.watchdog.age!=4999)$fatal(1,"watchdog boundary");
 byte_send(8'hA5,1);key=0;cycles(30);off;key=1;cycles(3000);
 if(!seen_log[12])$fatal(1,"missing watchdog event");
 $display("PASS integrated IMU/config/axes, independent plants, PWM, timed stop/rearm, heartbeat/framing/boundary/latch, quiet UART");
 reset_system;boot;start;model_on=0;
 @(negedge clk);estop=0;#1;off;#2;estop=1;cycles(50);off;
 cycles(3000);if(!seen_log[15])$fatal(1,"missing estop event");
 reset_system;boot;start;@(negedge clk);abl=3;abr=3;cycles(120);off;
 cycles(5000);if(!seen_log[13]||!seen_log[14])$fatal(1,"simultaneous encoder events lost");
 $display("PASS estop and simultaneous left/right encoder faults latch and report once");
 reset_system;model_id=8'hff;wait(dut.imu_error==4);cycles(8000);off;
 if(!seen_log[4]||boot_count!=4)$fatal(1,"ID initialization error logging");
 model_id=8'h24;wait(dut.imu_ready);wait(dut.first_sample);start_blocked;
 reset_system;init_bad=1;wait(dut.imu_error==5);cycles(5000);off;
 if(!seen_log[5]||boot_count!=4)$fatal(1,"initialization error logging");
 init_bad=0;wait(dut.imu_ready);wait(dut.first_sample);cycles(100);start_blocked;
 $display("PASS initialization error locks out motion after automatic recovery");
 reset_system;boot;start;model_id=8'hff;wait(dut.imu_error==6);cycles(20);off;
 model_id=8'h24;cycles(5000);
 if(!seen_log[6]||!seen_log[9])$fatal(1,"runtime ID/ready-loss event");
 reset_system;boot;start;model_error=8'h01;wait(dut.imu_error==7);cycles(20);off;
 cycles(7000);if(!seen_log[7]||!seen_log[8])$fatal(1,"sensor register error event");
 reset_system;boot;start;
 force dut.sample_valid=0;cycles(5100);off;release dut.sample_valid;
 cycles(4000);if(!seen_log[10])$fatal(1,"sample timeout log");
 cycles(6000);off;
 reset_system;boot;start;force dut.imu_ready=0;cycles(10);off;release dut.imu_ready;
 cycles(4000);if(!seen_log[9])$fatal(1,"ready loss log");
 $display("PASS runtime ID, sensor error, sample timeout and ready loss inhibit and latch");
 if(periods<100)$fatal(1,"PWM coverage");
 completed=1;$display("PASS robot_system integration suite");$finish;
 end
 task start_blocked;begin
 hb_on=1;wait(hb_led);cycles(30);key=0;cycles(30);off;key=1;
 end endtask
endmodule
