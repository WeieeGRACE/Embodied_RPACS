`timescale 1ns/1ps
// Minimal reproducer: panel start, pause, voice-noise window, resume.
// Prints every dispatch/fault state change so the auto_run drop source is visible.
module tb_probe_resume;
 reg clk=0;always #10 clk=~clk;
 reg k0=1,k1=1,k2=1,rx=1,miso=0;
 wire rst=dut.rst_n;
 reg [1:0] abl=0,abr=0;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n,ready_led,run_led,fault_led,voice_led;
 wire usb,oscl;tri1 osda;tri1 [2:0] scl,sda,xshut;
 reg model_on=0;
 reg [7:0] model_id=8'h24,model_error=0;
 reg signed [15:0] gyro_model=17;
 reg [47:0] ranges={16'd2000,16'd2000,16'd2000};
 robot_system_top #(.CLK_HZ(1000000),.UART_BAUD(100000),.PWM_HZ(1000),
 .ENC_FILTER(2),.DIR_DEADTIME(10),.POR_CYCLES(8),.DEBOUNCE_CYCLES(4),
 .IMU_POWERUP_CYCLES(2),.IMU_CONFIG_WAIT_CYCLES(2),.IMU_RETRY_CYCLES(20000),
 .IMU_SPI_HALF_PERIOD(1),.IMU_HEALTH_CYCLES(20000),.IMU_CALIBRATION_SAMPLES(16),
 .OLED_I2C_HZ(100000),.OLED_POWERUP_CYCLES(3),.OLED_FRAME_WAIT_CYCLES(100),
 .TOF_I2C_HZ(50000),.TOF_RESET_CYCLES(10),.TOF_BOOT_CYCLES(30),
 .TOF_POLL_CYCLES(30),.TOF_MAX_POLLS(3),.TOF_TIMEOUT_CYCLES(300),.TOF_ROUND_CYCLES(50000)) dut(
 .clk_50m(clk),.reset_n(k0),.key_enable_n(k1),.key_estop_n(k2),.uart_rx(1'b1),.asr_rx(rx),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .imu_spi_miso(miso),.imu_int1(1'b0),.imu_int2(1'b0),.imu_gnd_reserved(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.asr_tx(tx),.uart_tx(usb),
 .oled_scl(oscl),.oled_sda(osda),
 .tof_front_gpio1(1'b1),.tof_left_gpio1(1'b0),.tof_right_gpio1(1'b1),
 .tof_front_scl(scl[0]),.tof_front_sda(sda[0]),.tof_front_xshut_n(xshut[0]),
 .tof_left_scl(scl[1]),.tof_left_sda(sda[1]),.tof_left_xshut_n(xshut[1]),
 .tof_right_scl(scl[2]),.tof_right_sda(sda[2]),.tof_right_xshut_n(xshut[2]),
 .led_ready(ready_led),.led_run(run_led),.led_fault(fault_led),.led_heartbeat(voice_led));
 genvar g;
 generate for(g=0;g<3;g++)begin:tof_models
 vl53l0x_model model(.scl(scl[g]),.sda(sda[g]),.xshut_n(xshut[g]),
 .present(3'b111),.nack_data(1'b0),.never_ready(1'b0),
 .distance_mm(ranges[g*16+:16]),.status(4'd3),.ready_after(1),
 .shots(),.polls(),.read_blocks(),.reset_count());
 end endgenerate
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task keypress(input integer key_index);begin
 @(negedge clk);case(key_index)0:k0=0;1:k1=0;2:k2=0;endcase
 cycles(12);@(negedge clk);k0=1;k1=1;k2=1;cycles(12);
 end endtask
 task send(input [7:0] data);begin
 @(negedge clk);rx=0;cycles(10);
 for(integer i=0;i<8;i++)begin @(negedge clk);rx=data[i];cycles(10);end
 @(negedge clk);rx=1;cycles(15);
 end endtask
 task frame(input [7:0] code);begin
 send('haa);send('h55);send(1);send(code);send(code+1);send(13);cycles(5);
 end endtask
 // Physical plant identical in shape to the suite model.
 integer pwm_l=0,pwm_r=0,plant_tick=0,velocity_l=0,velocity_r=0;
 integer accum_l=0,accum_r=0;
 real physical_yaw=0.0;
 always @(negedge clk)begin
 if(!rst || !model_on)begin
 pwm_l=0;pwm_r=0;plant_tick=0;velocity_l=0;velocity_r=0;accum_l=0;accum_r=0;gyro_model=17;
 if(!rst)physical_yaw=0;
 end else begin
 if(a && st)pwm_l=pwm_l+(a1 ? 1:-1);
 if(b && st)pwm_r=pwm_r+(b1 ? 1:-1);
 if(plant_tick==999)begin
 velocity_l=(3*velocity_l+pwm_l*8)/4;
 velocity_r=(3*velocity_r+pwm_r*7)/4;
 pwm_l=0;pwm_r=0;plant_tick=0;
 gyro_model=$rtoi((velocity_l-velocity_r)*0.557)+17;
 physical_yaw=physical_yaw+(velocity_r-velocity_l)*0.000034;
 end else plant_tick++;
 if(accum_l>=1000000)begin
 case(abl)0:abl=1;1:abl=3;3:abl=2;2:abl=0;endcase accum_l-=1000000;
 end
 if(accum_l<=-1000000)begin
 case(abl)0:abl=2;2:abl=3;3:abl=1;1:abl=0;endcase accum_l+=1000000;
 end
 accum_l=accum_l+velocity_l;
 if(accum_r>=1000000)begin
 case(abr)0:abr=2;2:abr=3;3:abr=1;1:abr=0;endcase accum_r-=1000000;
 end
 if(accum_r<=-1000000)begin
 case(abr)0:abr=1;1:abr=3;3:abr=2;2:abr=0;endcase accum_r+=1000000;
 end
 accum_r=accum_r+velocity_r;
 end
 end
 // IMU SPI slave model (same contract as the suite testbench).
 reg init_bad=0;
 reg [7:0] tx_shift, rx_shift;
 reg [55:0] response_shift;
 reg [3:0] bit_count;
 reg [7:0] command;
 integer config_bytes;
 integer write_count;
 reg [7:0] expected_config [0:8191];
 initial $readmemh("src/imu/imu660ra_config.hex", expected_config);
 always @(posedge cs_n) if(rst && command==8'h5e && config_bytes!=8192)
  $fatal(1,"bad config stream count=%0d",config_bytes);
 always @(negedge cs_n) begin
  bit_count = 0;tx_shift = 0;rx_shift = 0;response_shift = 0;
  command = 0;config_bytes = 0;write_count = 0;miso = 0;
 end
 always @(posedge sck) begin
  if (!cs_n) begin
   #1;
   tx_shift = {tx_shift[6:0], mosi};
   if (bit_count == 7) begin
    if (command == 0) begin
     command = tx_shift;
     if (tx_shift == 8'h80) response_shift = {8'h00,model_id,40'd0};
     else if (tx_shift == 8'h82) response_shift = {8'h00,model_error,40'd0};
     else if (tx_shift == 8'hA1) response_shift = (init_bad) ? 56'h00020000000000 : 56'h00010000000000;
     else if (tx_shift == 8'h8C) response_shift = 56'h00000000000010;
     else if (tx_shift == 8'h92) response_shift = {8'd0,16'd0,16'd0,gyro_model[7:0],gyro_model[15:8]};
     else response_shift = 0;
    end else if (command == 8'h5E) begin
     if (config_bytes >= 8192) $fatal(1, "too many configuration bytes");
     if (tx_shift !== expected_config[config_bytes])
      $fatal(1, "configuration mismatch index=%0d", config_bytes);
     config_bytes = config_bytes + 1;
    end
    bit_count = 0;
   end else bit_count = bit_count + 1;
  end
 end
 always @(negedge sck) begin
  if (!cs_n) begin miso = response_shift[55];response_shift = {response_shift[54:0], 1'b0};end
  else miso = 0;
 end
 // State-change monitors: print every interesting transition.
 reg [4:0] route_q=0;reg auto_q=0;reg fault_q=0;reg wait_q=0;reg arm_q=0;
 always @(posedge clk) begin
 route_q<=dut.control.route_state;
 auto_q<=dut.control.auto_run;
 fault_q<=dut.control.fault;
 wait_q<=dut.control.waiting;
 arm_q<=dut.control.dispatch.key_arming;
 if(route_q!=dut.control.route_state)
  $display("[%0t] ROUTE %0d->%0d auto=%b wait=%b arm=%b diag=%0d fault=%b mflt=%h sflt=%h rflt=%b imu_bad=%b enc=%b/%b txovf=%b stop_req=%b cancel=%b",
   $time,route_q,dut.control.route_state,dut.control.auto_run,dut.control.waiting,dut.control.dispatch.key_arming,dut.control.start_diag,
   dut.control.fault,dut.control.motion_fault,dut.control.sensor_fault,dut.control.route.route_fault,
   dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow,
   dut.control.stopping,dut.control.route_cancel);
 if(auto_q!=dut.control.auto_run)
  $display("[%0t] AUTO %b->%b wait=%b arm=%b diag=%0d fault=%b mflt=%h sflt=%h rflt=%b imu_bad=%b enc=%b/%b txovf=%b",
   $time,auto_q,dut.control.auto_run,dut.control.waiting,dut.control.dispatch.key_arming,dut.control.start_diag,
   dut.control.fault,dut.control.motion_fault,dut.control.sensor_fault,dut.control.route.route_fault,
   dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow);
 if(fault_q!=dut.control.fault)
  $display("[%0t] FAULT %b->%b mflt=%h sflt=%h rflt=%b imu_bad=%b enc=%b/%b txovf=%b imu_err=%h",
   $time,fault_q,dut.control.fault,dut.control.motion_fault,dut.control.sensor_fault,dut.control.route.route_fault,
   dut.control.imu_bad,dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow,
   dut.control.imu_error);
 if(wait_q!=dut.control.waiting)$display("[%0t] WAITING %b",$time,dut.control.waiting);
 if(arm_q!=dut.control.dispatch.key_arming)$display("[%0t] ARMING %b",$time,dut.control.dispatch.key_arming);
 end
 integer done=0;
 initial begin
 wait(rst);cycles(100);
 wait(ready_led);cycles(1000);model_on=1;
 $display("BOOT ready");
 // Navigate: K1 x2 -> SETTINGS (item 2), K2, K1, K2 (voice page), K1, K2 (commit ON).
 repeat(2)begin keypress(1);end
 keypress(2); // open settings group 4 page, item 0
 keypress(2); // voice page
 keypress(1); // ON candidate
 keypress(2); // commit
 if(!dut.voice_enable)$fatal(1,"voice enable failed");
 $display("VOICE ON");
 // Navigate to obstacle panel: K0 leaves voice page, K0 returns to root with
 // the cursor resting on item 3 (OBSTACLE AVOID); K2 opens the panel.
 keypress(0);keypress(0);
 if(dut.group_id!=0 || dut.item!=3)$fatal(1,"root cursor %0d/%0d",dut.group_id,dut.item);
 keypress(2); // select obstacle entry -> panel
 if(dut.group_id!=5 || !dut.page)$fatal(1,"panel open failed");
 keypress(2); // panel start
 wait(dut.auto_run);wait(run_led);
 $display("STARTED route=%0d",dut.control.route_state);
 cycles(40000); // cruise a while
 $display("PAUSING");
 keypress(2); // pause
 wait(!run_led);cycles(2000);
 $display("PAUSED auto=%b",dut.control.auto_run);
 // Voice noise while paused (as the suite's assert does).
 frame(1);frame(2);frame(3);frame('h10);frame('h11);frame('h12);frame('h13);
 frame('h14);frame('h15);frame('h16);frame('h17);
 cycles(5000);
 $display("RESUMING");
 keypress(2); // resume
 cycles(60000);
 $display("FINAL auto=%b route=%0d diag=%0d fault=%b waiting=%b arm=%b",
  dut.control.auto_run,dut.control.route_state,dut.control.start_diag,dut.control.fault,dut.control.waiting,dut.control.dispatch.key_arming);
 if(dut.control.auto_run && run_led)begin $display("PROBE PASS resume works");done=1;end
 else $display("PROBE FAIL resume blocked");
 $finish;
 end
 initial begin #20000000;$display("PROBE TIMEOUT");$finish;end
endmodule
