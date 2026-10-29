`timescale 1ns/1ps
// ===========================================================================
// tb_fault_turns.sv - fault-injection harness for the reported symptom of the
// autonomous obstacle-avoidance demo on GW5A-25:
//   "STOP with FAULT prompt (OLED: FAULT RESET REQUIRED) right after a
//    90 degree turn completes".
//
// One scenario per simulation; select with +SCENARIO=<0..8> on the vsim
// command line (see sim/run_fault_turns*.do).
//
//   0 A_FRONT : exact 20 mm on FRONT  ToF right after turn 4->8, held 1.2 s
//   1 A_SIDE  : exact 20 mm on RIGHT  ToF right after turn 4->8, held 1.2 s
//   2 B       : exact 20 mm on FRONT  ToF right after turn 4->8, permanent
//   3 C       : transient I2C NACK (present=0) on RIGHT ToF, 20 ms after turn
//   4 D       : IMU health-error burst (model_error=1) 150 ms during rotation
//   5 E       : encoder-count injection during post-turn WAIT_STILL, 1.2 s
//   6 CTRL    : negative control, exact 20 mm on FRONT for only 100 ms
//   7 G_NEAR  : valid 60 mm on RIGHT side ToF right after the turn (baseline
//               latches faults[4]; fixed RTL keeps it valid and takes the
//               fixed-mileage fallback because 60 mm <= AVOID_MM)
//   8 H_TURN2 : exact 20 mm on LEFT ToF right after the SECOND turn (7->13)
//   9 I_HOLD  : exact 20 mm on FRONT ToF, comm normal, held 2 s after the turn
//               (hold20 reuses the old value but post_turn never goes fresh)
//  10 K_CLIP  : obstacle-side lane permanently clamped at 20 mm from the turn
//  11 PASS_CLIP: obstacle-side lane stays healthy through the barrier and only
//               clamps at 20 mm once PASS has started (observed on the board
//               as BLOCKED reason 0x04 after the edge detector stopped firing)
//
// The DUT runs the accelerated parameter set of tb_system_obstacle.sv
// (CLK_HZ=1 MHz), so 1 DUT millisecond = 1000 clock cycles = 20 us of
// simulation time. 10 DUT ms = 0.2 us -> the plant/route timing matches the
// existing passing suite.
// ===========================================================================
module tb_fault_turns;

 localparam integer DUT_MS = 1000; // clock cycles per DUT millisecond (CLK_HZ=1e6)

 localparam integer SC_A_FRONT = 0;
 localparam integer SC_A_SIDE  = 1;
 localparam integer SC_B       = 2;
 localparam integer SC_C       = 3;
 localparam integer SC_D       = 4;
 localparam integer SC_E       = 5;
 localparam integer SC_CTRL    = 6;
 localparam integer SC_G_NEAR  = 7;
 localparam integer SC_H_TURN2 = 8;
 localparam integer SC_I_HOLD  = 9;
 localparam integer SC_K_CLIP  = 10;
 localparam integer SC_PASS_CLIP = 11;
 localparam integer SC_PASS_FRONT = 12;

 reg clk=0; always #10 clk=~clk;
 reg k0=1,k1=1,k2=1,rx=1,miso=0;
 wire rst=dut.rst_n;
 reg [1:0] abl=0,abr=0;
 wire a,a1,a2,b,b1,b2,st,tx,sck,mosi,cs_n,ready_led,run_led,fault_led,voice_led;
 wire oscl;tri1 osda;tri1 [2:0] scl,sda,xshut;
 reg completed=0,model_on=0,freeze_l=0,freeze_r=0;
 reg route_started=0,pause_seen=0,blocked_seen=0;
 reg inject_enc_l=0,inject_enc_r=0;
 reg [7:0] model_id=8'h24,model_error=0;
 reg signed [15:0] gyro_model=17;
 reg [2:0] present=7,never_ready=0;reg [47:0] ranges={16'd2000,16'd2000,16'd2000};
 wire [31:0] shots[0:2],polls[0:2],blocks[0:2],resets[0:2];

 integer scenario=0;
 integer fixed_mode=0;           // +FIXED=1: observation-only run of the fixed RTL
 integer menu_new=0;            // +NEWMENU=1: post-fix menu (DISPLAY group removed):
                                //   root item0=MANUAL(2), item1=SETTINGS(4), item2=OBSTACLE(5)
 integer cyc=0;                 // free-running clock-cycle counter
 integer mark_cyc=0;            // cycle of the last injection marker
 integer reach_shift=0;         // scenario K: fallback detour entered
 integer fallback_done=0;       // scenario K: detour completed back to CRUISE
 integer near_shift=0;          // scenario G_NEAR: near-side fallback entered
 integer pass_reached=0;        // scenario PASS_CLIP: PASS state entered
 integer pass_clip_done=0;      // scenario PASS_CLIP: fallback pass finished, CRUISE resumed
 integer pass_front_done=0;     // scenario PASS_FRONT: transient front graze tolerated, CRUISE resumed
 reg mark_valid=0;

 // first-fault snapshot (filled by the monitor below)
 integer f_cyc=-1;
 reg [4:0] f_ch0=0,f_ch1=0,f_ch2=0;
 reg [14:0] f_bits=0;
 reg [7:0] f_motion=0,f_imuerr=0,f_senserr=0,f_reason=0;
 reg f_rfault=0,f_imubad=0,f_sfault=0,f_flatch=0,f_auto=0,f_latch_seen=0;
 reg [4:0] f_rstate=0;
 reg f_enc_l=0,f_enc_r=0,f_txovf=0;

 // Fixed-RTL only probes (compiled with +define+FIXED_RTL in the fixed runner):
 // count channel glitch-recovery requests vs the driver actually accepting
 // them, to check the intended "4 consecutive exact-20 mm -> XSHUT re-init".
 // Count confirmed minimum-range clips per channel (fixed RTL) and bus-error
 // re-inits (driver recover), to prove the clip path no longer churns XSHUT.
 integer req_cnt0=0,req_cnt1=0,req_cnt2=0,rec_cnt0=0,rec_cnt1=0,rec_cnt2=0;
`ifdef FIXED_RTL
 reg req_q0=0,req_q1=0,req_q2=0,rec_q0=0,rec_q1=0,rec_q2=0;
 always @(posedge clk)begin
  req_q0<=dut.sensors.channels[0].confirmed;
  req_q1<=dut.sensors.channels[1].confirmed;
  req_q2<=dut.sensors.channels[2].confirmed;
  rec_q0<=dut.sensors.channels[0].driver.recover;
  rec_q1<=dut.sensors.channels[1].driver.recover;
  rec_q2<=dut.sensors.channels[2].driver.recover;
  if(dut.sensors.channels[0].confirmed && !req_q0)req_cnt0<=req_cnt0+1;
  if(dut.sensors.channels[1].confirmed && !req_q1)req_cnt1<=req_cnt1+1;
  if(dut.sensors.channels[2].confirmed && !req_q2)req_cnt2<=req_cnt2+1;
  if(dut.sensors.channels[0].driver.recover && !rec_q0)rec_cnt0<=rec_cnt0+1;
  if(dut.sensors.channels[1].driver.recover && !rec_q1)rec_cnt1<=rec_cnt1+1;
  if(dut.sensors.channels[2].driver.recover && !rec_q2)rec_cnt2<=rec_cnt2+1;
 end
`endif

 robot_system_top #(.CLK_HZ(1000000),.UART_BAUD(100000),.PWM_HZ(1000),
 .ENC_FILTER(2),.DIR_DEADTIME(10),.POR_CYCLES(8),.DEBOUNCE_CYCLES(4),
 .IMU_POWERUP_CYCLES(2),.IMU_CONFIG_WAIT_CYCLES(2),.IMU_RETRY_CYCLES(20000),
 .IMU_SPI_HALF_PERIOD(1),.IMU_HEALTH_CYCLES(20000),.IMU_CALIBRATION_SAMPLES(16),
 .OLED_I2C_HZ(100000),.OLED_POWERUP_CYCLES(3),.OLED_FRAME_WAIT_CYCLES(100),
 .TOF_I2C_HZ(50000),.TOF_RESET_CYCLES(10),.TOF_BOOT_CYCLES(30),
 .TOF_POLL_CYCLES(30),.TOF_MAX_POLLS(3),.TOF_TIMEOUT_CYCLES(300),.TOF_ROUND_CYCLES(50000)) dut(
 .clk_50m(clk),.reset_n(k0),.key_enable_n(k1),.key_estop_n(k2),.asr_rx(rx),
 .encoder_a_l(abl[1]),.encoder_b_l(abl[0]),.encoder_a_r(abr[1]),.encoder_b_r(abr[0]),
 .imu_spi_miso(miso),.imu_int1(1'b0),.imu_int2(1'b0),.imu_gnd_reserved(1'b0),
 .imu_spi_sck(sck),.imu_spi_mosi(mosi),.imu_cs_n(cs_n),
 .PWMA(a),.AIN1(a1),.AIN2(a2),.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(st),.asr_tx(tx),
 .oled_scl(oscl),.oled_sda(osda),
 .tof_front_gpio1(1'b1),.tof_left_gpio1(1'b0),.tof_right_gpio1(1'b1),
 .tof_front_scl(scl[0]),.tof_front_sda(sda[0]),.tof_front_xshut_n(xshut[0]),
 .tof_left_scl(scl[1]),.tof_left_sda(sda[1]),.tof_left_xshut_n(xshut[1]),
 .tof_right_scl(scl[2]),.tof_right_sda(sda[2]),.tof_right_xshut_n(xshut[2]),
 .led_ready(ready_led),.led_run(run_led),.led_fault(fault_led),.led_heartbeat(voice_led));

 genvar g;
 generate for(g=0;g<3;g++)begin:tof_models
 vl53l0x_model model(.scl(scl[g]),.sda(sda[g]),.xshut_n(xshut[g]),
 .present(present[g]),.nack_data(1'b0),.never_ready(never_ready[g]),
 .distance_mm(ranges[g*16+:16]),.status(4'(3+g*2)),.ready_after(1),
 .shots(shots[g]),.polls(polls[g]),.read_blocks(blocks[g]),.reset_count(resets[g]));
 end endgenerate

 // --------------------------------------------------------------------------
 // Helpers (same style as the passing tb_system_obstacle suite)
 // --------------------------------------------------------------------------
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task off;begin #1;if({a,a1,a2,b,b1,b2,st}!==0)$fatal(1,"unsafe motor outputs");end endtask
 task keypress(input integer key_index);begin
 @(negedge clk);case(key_index)0:k0=0;1:k1=0;2:k2=0;endcase
 cycles(12);@(negedge clk);k0=1;k1=1;k2=1;cycles(12);
 end endtask
 task root;begin
 if(dut.page)keypress(0);
 if(dut.group_id!=0)keypress(0);
 if(dut.group_id!=0 || dut.page)$fatal(1,"root navigation");
 end endtask
 function automatic integer menu_index(input integer gr);
  // Baseline menu has the DISPLAY group: root item1=MANUAL(2), item2=SETTINGS(4),
  // item3=OBSTACLE(5). The frozen final menu removed DISPLAY, so the same groups
  // sit at root items 0/1/2 (select with +NEWMENU=1).
  if(menu_new) begin
   case(gr)2:menu_index=0;4:menu_index=1;5:menu_index=2;default:menu_index=0;endcase
  end else begin
   case(gr)2:menu_index=1;4:menu_index=2;5:menu_index=3;default:menu_index=0;endcase
  end
 endfunction
 task choose(input integer gr,input integer it);begin
 root();$display("CHOOSE group=%0d item=%0d page=%b target=%0d",dut.group_id,dut.item,dut.page,gr);
 while(dut.item!=menu_index(gr))keypress(1);keypress(2);
 while(dut.item!=it)begin keypress(1);$display("CHOOSE page item=%0d",dut.item);end
 if(gr!=5)keypress(2);
 if(!dut.page || dut.group_id!=gr || dut.item!=it)$fatal(1,"page selection");
 end endtask
 task send(input [7:0] data);begin
 @(negedge clk);rx=0;cycles(10);
 for(integer i=0;i<8;i++)begin @(negedge clk);rx=data[i];cycles(10);end
 @(negedge clk);rx=1;cycles(15);
 end endtask
 task frame(input [7:0] code);begin
 send('haa);send('h55);send(1);send(code);send(code+1);send(13);cycles(5);
 end endtask

 // Wait for a route state, bounded by a clock-cycle budget. ok=1 on success.
 task wait_route(input integer target_state,input integer max_cycles,output integer ok);
 integer n;begin
  n=0;ok=0;
  while(n<max_cycles)begin
   if(dut.route_state==target_state)begin ok=1;n=max_cycles;end
   else begin @(posedge clk);n=n+1;end
  end
  $display("EVENT wait_route target=%0d ok=%0d now=%0d dut_ms=%0d",target_state,ok,dut.route_state,cyc/DUT_MS);
 end endtask

 // Wait until dut.control.fault rises or the budget expires. ok=1 on fault.
 task wait_fault(input integer max_cycles,output integer ok);
 integer n;begin
  n=0;ok=0;
  while(n<max_cycles)begin
   if(dut.control.fault)begin ok=1;n=max_cycles;end
   else begin @(posedge clk);n=n+1;end
  end
 end endtask

 // --------------------------------------------------------------------------
 // Independent physical plants consume output pin duty/direction, never RTL
 // targets. Copied from tb_system_obstacle.sv, plus an encoder-count injection
 // hook used only by scenario E.
 // --------------------------------------------------------------------------
 integer pwm_l=0,pwm_r=0,plant_tick=0,velocity_l=0,velocity_r=0;
 integer accum_l=0,accum_r=0;
 integer inj_l=0,inj_r=0;
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
   // Measured installation: sensor Z is positive for a physical right turn.
   gyro_model=$rtoi((velocity_l-velocity_r)*0.557)+17;
   physical_yaw=physical_yaw+(velocity_r-velocity_l)*0.000034;
  end else plant_tick++;
  if(inject_enc_l)begin
   // Sustained forward quadrature on the left wheel: one edge every 200
   // cycles -> 50 counts per 10 ms speed window, never an illegal state.
   inj_l=inj_l+1;
   if(inj_l>=200)begin inj_l=0;case(abl)0:abl=1;1:abl=3;3:abl=2;2:abl=0;endcase end
  end else if(!freeze_l)begin
   accum_l=accum_l+velocity_l;
   if(accum_l>=1000000)begin
    case(abl)0:abl=1;1:abl=3;3:abl=2;2:abl=0;endcase accum_l-=1000000;
   end
   if(accum_l<=-1000000)begin
    case(abl)0:abl=2;2:abl=3;3:abl=1;1:abl=0;endcase accum_l+=1000000;
   end
  end
  if(inject_enc_r)begin
   inj_r=inj_r+1;
   if(inj_r>=200)begin inj_r=0;case(abr)0:abr=2;2:abr=3;3:abr=1;1:abr=0;endcase end
  end else if(!freeze_r)begin
   accum_r=accum_r+velocity_r;
   if(accum_r>=1000000)begin
    case(abr)0:abr=2;2:abr=3;3:abr=1;1:abr=0;endcase accum_r-=1000000;
   end
   if(accum_r<=-1000000)begin
    case(abr)0:abr=1;1:abr=3;3:abr=2;2:abr=0;endcase accum_r+=1000000;
   end
  end
 end
 end
 integer duty_l=0,duty_r=0,phase_count=0;
 always @(negedge clk)begin
 if(!rst || !st)begin duty_l=0;duty_r=0;phase_count=0;end
 else begin
  if(a)duty_l++;if(b)duty_r++;
  if(phase_count==999)begin
   if(duty_l>250 || duty_r>250)$fatal(1,"PWM exceeds 25 percent %0d %0d",duty_l,duty_r);
   duty_l=0;duty_r=0;phase_count=0;
  end else phase_count++;
 end
 end

 // --------------------------------------------------------------------------
 // ASR feedback capture: also counts the LATCHED-FAULT status event (type 3
 // code 6) that drives the spoken/OLED fault prompt.
 // --------------------------------------------------------------------------
 integer done_events=0,stop_events=0,fault_events=0,busy_replies=0,reject_replies=0,accepted=0;
 integer audio_requests=0,obstacle_events=0,clear_events=0,exit_events=0;reg [7:0] audio_token=0;
 integer idx=0;reg [7:0] got,line[0:5];
 // asr_rx is held idle at 1: this harness starts the route with K2, but still
 // captures the outgoing feedback frames to count the latched-fault event.
 always @(negedge tx)begin:capture
 if(rst)begin
  repeat(15)@(negedge clk);
  for(integer i=0;i<8;i++)begin if(!rst)disable capture;got[i]=tx;repeat(10)@(negedge clk);end
  if(!rst)disable capture;
  if(tx!==1)$fatal(1,"UART stop");line[idx]=got;
  if(idx==5)begin
   if(line[0]!='haa || line[1]!='h55 || (line[2]!=2 && line[2]!=3 && line[2]!=4 && line[2]!=6 && line[2]!=7) || line[4]!=((line[2]+line[3])&255) || got!=13)$fatal(1,"bad feedback frame");
   if(line[2]==4)begin audio_token=line[3];audio_requests++;end
   if(line[2]==7)case(line[3])1:obstacle_events++;2:clear_events++;3:exit_events++;endcase
   if(line[2]==2)case(line[3])0:accepted++;1:busy_replies++;2:reject_replies++;default:$fatal(1,"bad response");endcase
   if(line[2]==3)case(line[3])'h11:done_events++;4:stop_events++;6:fault_events++;endcase
   idx=0;
  end else idx++;
 end
 end

 // --------------------------------------------------------------------------
 // Inline IMU660RA SPI slave model (same as the passing suite).
 // --------------------------------------------------------------------------
 reg [7:0] tx_shift, rx_shift;
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
      response_shift = 56'h00010000000000;
     else if (tx_shift == 8'h8C)
      response_shift = 56'h00000000000010;
     else if (tx_shift == 8'h92)
      response_shift = {8'd0,16'd0,16'd0,gyro_model[7:0],gyro_model[15:8]};
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

 // --------------------------------------------------------------------------
 // Timeline trace of the obstacle route (same diagnostic style as the suite).
 // --------------------------------------------------------------------------
 always @(dut.route_state)$display("MAIN route=%0d auto=%b waiting=%b reason=%02h dut_ms=%0d time_ns=%0t",
  dut.route_state,dut.auto_run,dut.waiting,dut.control.route_reason,cyc/DUT_MS,$time);

 // --------------------------------------------------------------------------
 // First-fault evidence monitor: captures every latch source at the first
 // clock where dut.control.fault is high, then the route FAULT reaction.
 // --------------------------------------------------------------------------
 reg auto_q=0,rf_q=0;
 integer dt_ms=0;
 always @(posedge clk)begin
  cyc<=cyc+1;
  if(mark_valid)dt_ms=(cyc-mark_cyc)/DUT_MS;else dt_ms=-1;
  if(f_cyc<0 && dut.control.fault)begin
   f_cyc<=cyc;
   f_ch0<=dut.sensors.channels[0].faults;
   f_ch1<=dut.sensors.channels[1].faults;
   f_ch2<=dut.sensors.channels[2].faults;
   f_bits<=dut.sensors.fault_bits;
   f_motion<=dut.control.motion_fault;
   f_imuerr<=dut.control.imu_error;
   f_senserr<=dut.control.sensor_error;
   f_rstate<=dut.control.route.state;
   f_reason<=dut.control.route.reason;
   f_rfault<=dut.control.route.route_fault;
   f_imubad<=dut.control.imu_bad;
   f_sfault<=dut.control.sensor_fault;
   f_flatch<=dut.control.fault_latched;
   f_enc_l<=dut.control.enc_fault_l;
   f_enc_r<=dut.control.enc_fault_r;
   f_txovf<=dut.control.tx_overflow;
   f_auto<=dut.control.auto_run;
   $display("EVIDENCE FAULT_T0 dt_from_mark_ms=%0d dut_ms=%0d time_ns=%0t ctrl.fault=%b fault_latched=%b sensor_fault=%b fault_bits=%04h ch0_faults=%02h ch1_faults=%02h ch2_faults=%02h route_state=%0d route_fault=%b route_reason=%02h route_elapsed_ms=%0d motion_fault=%02h motion_phase=%0d motion_fault_code=%02h motion_stop_elapsed=%0d imu_bad=%b imu_error=%02h sensor_error=%02h enc_fault=%b/%b tx_overflow=%b post_turn=%b fresh=%b%b%b auto_run=%b avoid_mode=%b fault_led=%b status=%02h distances=%04h/%04h/%04h tof_raw_age=%0d/%0d/%0d tof_comm_age=%0d/%0d/%0d tof_error=%02h/%02h/%02h fault_events=%0d",
    dt_ms,cyc/DUT_MS,$time,dut.control.fault,dut.control.fault_latched,dut.control.sensor_fault,
    dut.sensors.fault_bits,dut.sensors.channels[0].faults,dut.sensors.channels[1].faults,dut.sensors.channels[2].faults,
    dut.control.route.state,dut.control.route.route_fault,dut.control.route.reason,dut.control.route.elapsed,
    dut.control.motion_fault,dut.control.motion.phase,dut.control.motion.fault_code,dut.control.motion.stop_elapsed,
    dut.control.imu_bad,dut.control.imu_error,dut.control.sensor_error,dut.control.enc_fault_l,dut.control.enc_fault_r,
    dut.control.tx_overflow,dut.post_turn,dut.sensors.channels[0].fresh,dut.sensors.channels[1].fresh,dut.sensors.channels[2].fresh,
    dut.control.auto_run,dut.avoid_mode,dut.led_fault,dut.control.status,dut.distances[15:0],dut.distances[31:16],dut.distances[47:32],
    dut.sensors.channels[0].age,dut.sensors.channels[1].age,dut.sensors.channels[2].age,
    dut.sensors.channels[0].comm_age,dut.sensors.channels[1].comm_age,dut.sensors.channels[2].comm_age,
    dut.sensors.channels[0].error,dut.sensors.channels[1].error,dut.sensors.channels[2].error,fault_events);
  end
  // Route's own FAULT reaction (best effort: the route is reset as soon as
  // the dispatch drops auto_run, so this window can be a single cycle).
  if(!rf_q && (dut.control.route.state==19 || dut.control.route.route_fault))begin
   rf_q<=1;
   $display("EVIDENCE ROUTE_FAULT state=%0d reason=%02h route_fault=%b barrier_state_elapsed_ms=%0d dut_ms=%0d",
    dut.control.route.state,dut.control.route.reason,dut.control.route.route_fault,dut.control.route.elapsed,cyc/DUT_MS);
  end
  if(route_started && dut.avoid_mode && !dut.auto_run && !dut.control.fault)pause_seen<=1;
  if(dut.route_state==17)blocked_seen<=1;
  if(auto_q && !dut.control.auto_run)
   $display("EVIDENCE AUTO_DROP dut_ms=%0d ctrl.fault=%b fault_latched=%b sensor_fault=%b route_fault=%b motion_fault=%02h imu_bad=%b fault_led=%b",
    cyc/DUT_MS,dut.control.fault,dut.control.fault_latched,dut.control.sensor_fault,dut.control.route.route_fault,
    dut.control.motion_fault,dut.control.imu_bad,dut.led_fault);
  auto_q<=dut.control.auto_run;
  if(!f_latch_seen && dut.control.fault_latched)begin
   f_latch_seen<=1;
   $display("EVIDENCE LATCH ctrl.fault_latched=1 dut_ms=%0d source_summary sensor_fault=%b route_fault=%b motion_fault=%02h imu_bad=%b enc=%b/%b txovf=%b",
    cyc/DUT_MS,dut.control.sensor_fault,dut.control.route.route_fault,dut.control.motion_fault,dut.control.imu_bad,
    dut.control.enc_fault_l,dut.control.enc_fault_r,dut.control.tx_overflow);
  end
 end

 // Print the full latch-source snapshot at the end of a scenario.
 task print_snapshot;begin
  $display("SNAPSHOT ctrl.fault=%b fault_latched=%b sensor_fault=%b fault_bits=%04h ch0_faults=%02h ch1_faults=%02h ch2_faults=%02h route_state=%0d route_fault=%b route_reason=%02h motion_fault=%02h imu_bad=%b imu_error=%02h sensor_error=%02h enc_fault=%b/%b tx_overflow=%b post_turn=%b fresh=%b%b%b auto_run=%b avoid_mode=%b fault_led=%b status=%02h fault_events=%0d dut_ms=%0d",
   dut.control.fault,dut.control.fault_latched,dut.control.sensor_fault,dut.sensors.fault_bits,
   dut.sensors.channels[0].faults,dut.sensors.channels[1].faults,dut.sensors.channels[2].faults,
   dut.route_state,dut.control.route.route_fault,dut.control.route.reason,dut.control.motion_fault,
   dut.control.imu_bad,dut.control.imu_error,dut.control.sensor_error,dut.control.enc_fault_l,dut.control.enc_fault_r,
   dut.control.tx_overflow,dut.post_turn,
   dut.sensors.channels[0].fresh,dut.sensors.channels[1].fresh,dut.sensors.channels[2].fresh,
   dut.control.auto_run,dut.avoid_mode,dut.led_fault,dut.control.status,fault_events,cyc/DUT_MS);
 end endtask

 // Motor output safety snapshot after a latched fault.
 task print_motor_off;begin
  $display("EVIDENCE MOTOR_AFTER_FAULT pwm_a=%b a1=%b a2=%b pwm_b=%b b1=%b b2=%b stby=%b off=%b dut_ms=%0d",
   a,a1,a2,b,b1,b2,st,({a,a1,a2,b,b1,b2,st}===7'b0000000),cyc/DUT_MS); end endtask

 // ==========================================================================
 // Scenario bodies
 // ==========================================================================

 // Drive from the route start to the first 90 degree turn (TURN_OUT=4) and
 // its completion (WAIT_PARALLEL=8). Returns 0 on harness error.
 task drive_to_first_turn(output integer ok);
 integer w_ok;begin
  ok=0;
  ranges={16'd2000,16'd2000,16'd2000};
  wait_route(2,4000000,w_ok);           // CRUISE
  if(!w_ok)begin $display("FAIL harness: never reached CRUISE");disable drive_to_first_turn;end
  ranges[15:0]=16'd130;                 // brake-select point
  wait_route(3,4000000,w_ok);           // BRAKE_SELECT
  if(!w_ok)begin $display("FAIL harness: never reached BRAKE_SELECT");disable drive_to_first_turn;end
  // left detour around a box held on the right side
  ranges={16'd2000,16'd2000,16'd2000};ranges[47:32]=16'd400;
  wait_route(4,8000000,w_ok);           // TURN_OUT (rotation running)
  if(!w_ok)begin $display("FAIL harness: never reached TURN_OUT");disable drive_to_first_turn;end
  $display("EVENT TURN_OUT_ENTERED dut_ms=%0d motion_mode=%0d time_ns=%0t",cyc/DUT_MS,dut.control.motion.mode,$time);
  if(dut.control.fault)begin $display("FAIL harness: fault before any injection");disable drive_to_first_turn;end
  ok=1;
 end endtask

 // Wait for the end of the first rotation (WAIT_PARALLEL=8).
 task wait_turn_done(output integer ok);
 integer w_ok;begin
  wait_route(8,12000000,w_ok);
  ok=w_ok;
  if(!w_ok)$display("FAIL harness: first rotation never completed (route_state=%0d)",dut.route_state);
  else $display("EVENT TURN_DONE state=8 dut_ms=%0d time_ns=%0t",cyc/DUT_MS,$time);
 end endtask

 // A: exact-20 mm injection after the rotation; hold_ms in DUT milliseconds.
 // monitor_extra_ms > 0 keeps the fault under observation after the latch.
 task inject_20mm(input integer ch,input integer hold_ms,input integer expect_fault,input integer monitor_extra_ms);
 integer n;integer limit;begin
  mark_valid=1;mark_cyc=cyc;
  $display("EVENT INJECT_EXACT20 ch=%0d mm=20 hold_ms=%0d expect_fault=%0d route_state=%0d dut_ms=%0d time_ns=%0t",
   ch,hold_ms,expect_fault,dut.route_state,cyc/DUT_MS,$time);
  ranges[ch*16+:16]=16'd20;
  limit=hold_ms*DUT_MS;n=0;
  while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
  if(f_cyc<0)$display("EVENT NO_FAULT_WHILE_HELD ch=%0d held_ms=%0d route_state=%0d",ch,hold_ms,dut.route_state);
  else $display("EVENT FAULT_WHILE_HELD ch=%0d after_ms=%0d dut_ms=%0d",ch,(f_cyc-mark_cyc)/DUT_MS,cyc/DUT_MS);
  cycles(3*DUT_MS);
  if(monitor_extra_ms>0)cycles(monitor_extra_ms*DUT_MS);
  ranges[ch*16+:16]=16'd2000;
  $display("EVENT RESTORE ch=%0d mm=2000 dut_ms=%0d time_ns=%0t",ch,cyc/DUT_MS,$time);
  cycles(2*DUT_MS);
 end endtask

 // C: transient I2C NACK on the RIGHT ToF channel for hold_ms after the turn.
 // Channel index is fixed (2) because a variable generate-block index is not a
 // legal hierarchical reference in Verilog.
 task inject_nack(input integer hold_ms);
 integer n;integer limit;integer r;begin
  mark_valid=1;mark_cyc=cyc;
  $display("EVENT INJECT_NACK ch=2 present=0 hold_ms=%0d route_state=%0d dut_ms=%0d time_ns=%0t",
   hold_ms,dut.route_state,cyc/DUT_MS,$time);
  present[2]=1'b0;
  limit=hold_ms*DUT_MS;n=0;
  while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
  cycles(3*DUT_MS);
  present=3'b111;
  $display("EVENT NACK_RESTORE present=111 dut_ms=%0d time_ns=%0t",cyc/DUT_MS,$time);
  // observe whether the channel recovers communication (fault itself is latched)
  n=0;r=0;
  while(n<400*DUT_MS)begin
   @(posedge clk);n=n+1;
   if(dut.sensors.channels[2].error==0 && dut.sensors.channels[2].fresh==1)begin r=1;n=400*DUT_MS;end
  end
  $display("EVIDENCE CHANNEL_RECOVERY ch=2 recovered=%0d error=%02h comm_age=%0d fresh=%b good=%b faults=%02h dut_ms=%0d",
   r,dut.sensors.channels[2].error,dut.sensors.channels[2].comm_age,dut.sensors.channels[2].fresh,
   dut.sensors.channels[2].good,dut.sensors.channels[2].faults,cyc/DUT_MS);
 end endtask

 // D: IMU health-error burst during the rotation.
 task inject_imu(input integer hold_ms,input integer monitor_extra_ms);
 integer n;integer limit;integer ok_mode;begin
  wait_route(4,8000000,ok_mode);
  if(!ok_mode)begin $display("FAIL harness: TURN_OUT not reached before IMU injection");disable inject_imu;end
  n=0;
  while(n<5000*DUT_MS && dut.control.motion.mode!=4'd6)begin @(posedge clk);n=n+1;end
  mark_valid=1;mark_cyc=cyc;
  $display("EVENT INJECT_IMU model_error=1 hold_ms=%0d motion_mode=%0d motion_phase=%0d route_state=%0d dut_ms=%0d time_ns=%0t",
   hold_ms,dut.control.motion.mode,dut.control.motion.phase,dut.route_state,cyc/DUT_MS,$time);
  model_error=8'h01;
  limit=hold_ms*DUT_MS;n=0;
  while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
  cycles(3*DUT_MS);
  model_error=8'h00;
  $display("EVENT IMU_RESTORE model_error=0 dut_ms=%0d time_ns=%0t",cyc/DUT_MS,$time);
  if(monitor_extra_ms>0)cycles(monitor_extra_ms*DUT_MS);
  $display("EVIDENCE IMU_STATE imu_ready=%b imu_error=%02h calibrating=%b attitude_valid=%b dut_ms=%0d",
   dut.control.imu_ready,dut.control.imu_error,dut.control.calibrating,dut.control.attitude_valid,cyc/DUT_MS);
 end endtask

 // E: encoder-count injection during the post-turn WAIT_STILL settle window.
 task inject_encoder(input integer hold_ms);
 integer n;integer limit;integer ok_still;begin
  wait_route(4,8000000,ok_still);
  if(!ok_still)begin $display("FAIL harness: TURN_OUT not reached before encoder injection");disable inject_encoder;end
  n=0;
  while(n<10000*DUT_MS && dut.control.motion.phase!=3'd4)begin @(posedge clk);n=n+1;end
  mark_valid=1;mark_cyc=cyc;
  $display("EVENT INJECT_ENCODER phase=%0d mode=%0d stop_elapsed=%0d route_state=%0d dut_ms=%0d time_ns=%0t",
   dut.control.motion.phase,dut.control.motion.mode,dut.control.motion.stop_elapsed,dut.route_state,cyc/DUT_MS,$time);
  inject_enc_l=1;
  limit=hold_ms*DUT_MS;n=0;
  while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
  cycles(3*DUT_MS);
  inject_enc_l=0;
  $display("EVENT ENCODER_RESTORE motion_fault=%02h motion_stop_elapsed=%0d motion_phase=%0d dut_ms=%0d",
   dut.control.motion_fault,dut.control.motion.stop_elapsed,dut.control.motion.phase,cyc/DUT_MS);
 end endtask

 // I: exact-20 mm on the FRONT ToF with normal communication held for hold_ms
 // after the turn. hold20 keeps reusing the last good distance, so the channel
 // looks "alive" but post_turn (fresh) never asserts again.
 task inject_hold20(input integer hold_ms,input integer post_ms);
 integer n;integer limit;begin
  mark_valid=1;mark_cyc=cyc;
  $display("EVENT INJECT_HOLD20 ch=0 mm=20 hold_ms=%0d route_state=%0d dut_ms=%0d time_ns=%0t",
   hold_ms,dut.route_state,cyc/DUT_MS,$time);
  ranges[15:0]=16'd20;
  limit=hold_ms*DUT_MS;n=0;
  while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
  cycles(3*DUT_MS);
  $display("EVIDENCE CLIP_AT_FAULT ch=0 fresh=%b post_turn=%b accepted=%0d age=%0d comm_age=%0d error=%02h faults=%02h dt_ms=%0d",
   dut.sensors.channels[0].fresh,dut.post_turn[0],
   dut.sensors.channels[0].accepted,dut.sensors.channels[0].age,
   dut.sensors.channels[0].comm_age,dut.sensors.channels[0].error,dut.sensors.channels[0].faults,
   (f_cyc<0)?-1:(f_cyc-mark_cyc)/DUT_MS);
  cycles(post_ms*DUT_MS);
  $display("EVIDENCE CLIP_TOTAL ch=0 total_hold_ms=%0d fresh=%b post_turn=%b accepted=%0d age=%0d comm_age=%0d error=%02h faults=%02h fault=%b latch=%b dut_ms=%0d",
   hold_ms+post_ms,dut.sensors.channels[0].fresh,dut.post_turn[0],
   dut.sensors.channels[0].accepted,dut.sensors.channels[0].age,
   dut.sensors.channels[0].comm_age,dut.sensors.channels[0].error,dut.sensors.channels[0].faults,
   dut.control.fault,dut.control.fault_latched,cyc/DUT_MS);
  ranges[15:0]=16'd2000;
  cycles(2*DUT_MS);
 end endtask

 // G: valid near-distance (non-20, <=85 mm) reading on a side sensor after
 // the rotation - the designed "near-distance protection" latch.
 task inject_near(input integer ch,input [15:0] mm,input integer hold_ms);
 integer n;integer limit;begin
  mark_valid=1;mark_cyc=cyc;
  $display("EVENT INJECT_NEAR ch=%0d mm=%0d hold_ms=%0d route_state=%0d dut_ms=%0d time_ns=%0t",
   ch,mm,hold_ms,dut.route_state,cyc/DUT_MS,$time);
  ranges[ch*16+:16]=mm;
  limit=hold_ms*DUT_MS;n=0;
  while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
  cycles(3*DUT_MS);
  $display("EVENT NEAR_HOLD_END fault=%b after_ms=%0d dut_ms=%0d",dut.control.fault,(f_cyc<0)?-1:(f_cyc-mark_cyc)/DUT_MS,cyc/DUT_MS);
 end endtask

 // H: drive through the first detour (turn-left, sideways shift, turn-right)
 // and reach the WAIT_BACK barrier (state 13) before injecting exact-20 mm.
 task drive_to_second_turn(output integer ok);
 integer n;integer w_ok;integer shift_l,shift_r;integer tr;begin
  ok=0;
  wait_route(8,12000000,w_ok);
  if(!w_ok)begin $display("FAIL harness: first rotation never completed");disable drive_to_second_turn;end
  wait_route(23,4000000,w_ok);
  if(!w_ok)begin $display("FAIL harness: SAMPLE_SHIFT not reached");disable drive_to_second_turn;end
  wait_route(6,4000000,w_ok);
  if(!w_ok)begin $display("FAIL harness: SHIFT not reached");disable drive_to_second_turn;end
  shift_l=dut.control.um_l;shift_r=dut.control.um_r;
  n=0;
  while(n<20000*DUT_MS && (((dut.control.um_l-shift_l)>>>1)+((dut.control.um_r-shift_r)>>>1))<350000)begin
   @(posedge clk);n=n+1;
  end
  ranges[47:32]=16'd900;
  wait_route(22,4000000,w_ok);
  if(!w_ok)begin $display("FAIL harness: SHIFT_EXTRA not reached");disable drive_to_second_turn;end
  wait_route(7,8000000,w_ok);
  if(!w_ok)begin $display("FAIL harness: TURN_PARALLEL not reached");disable drive_to_second_turn;end
  ranges[47:32]=16'd2000;
  wait_route(13,12000000,w_ok);
  if(!w_ok)begin $display("FAIL harness: WAIT_BACK not reached");disable drive_to_second_turn;end
  $display("EVENT SECOND_TURN_DONE state=13 dut_ms=%0d actual_shift=%0d time_ns=%0t",
   cyc/DUT_MS,dut.control.route.actual_shift,$time);
  if(dut.control.fault)begin $display("FAIL harness: fault before H injection");disable drive_to_second_turn;end
  ok=1;
 end endtask

 // ==========================================================================
 // Main sequence
 // ==========================================================================
 integer n,limit,ok_turn,ok_done,w_ok,ok_drv;
 integer check_ok;
 integer ev_start;
 initial begin
  if(!$value$plusargs("SCENARIO=%d",scenario))scenario=SC_A_FRONT;
  if(!$value$plusargs("FIXED=%d",fixed_mode))fixed_mode=0;
  if(!$value$plusargs("NEWMENU=%d",menu_new))menu_new=0;
  $display("SCENARIO id=%0d",scenario);
  $display("FIXED_MODE=%0d",fixed_mode);
  $display("NEWMENU=%0d",menu_new);
  if(fixed_mode)
   $display("NOTE: FIXED_MODE is observation-only. Scenario PASS/FAIL lines still state the BASELINE expectation (PASS = the old design would latch a fault); the authoritative result is the final FIXED_RUN_OBSERVED line.");

  // boot: wait for chassis ready and fresh ToF data
  wait(rst);wait(ready_led);wait(dut.tof_valid==7);cycles(2000);off();model_on=1;
  $display("EVENT BOOT_READY dut_ms=%0d tof_valid=%b voice=%b time_ns=%0t",cyc/DUT_MS,dut.tof_valid,dut.voice_enable,$time);

  // enable voice in SETTINGS (same path as the passing suite)
  choose(4,0);keypress(1);keypress(2);root();
  if(!dut.voice_enable)$fatal(1,"voice enable setup failed");
  $display("EVENT VOICE_ENABLED dut_ms=%0d",cyc/DUT_MS);

  // OBSTACLE AVOID panel, then K2 starts the autonomous route
  choose(5,0);keypress(2);cycles(100);wait(dut.auto_run);wait(run_led);
  $display("EVENT ROUTE_STARTED auto_run=%b avoid_mode=%b route_state=%0d dut_ms=%0d time_ns=%0t",
   dut.auto_run,dut.avoid_mode,dut.route_state,cyc/DUT_MS,$time);
  route_started=1;
  if(dut.control.fault)$fatal(1,"fault present before injection");

  drive_to_first_turn(ok_drv);
  if(!ok_drv)begin $display("FAIL scenario %0d: harness could not reach the first turn",scenario);$finish;end

  check_ok=0;
  case(scenario)
   SC_A_FRONT:begin
    wait_turn_done(ok_turn);
    if(ok_turn)inject_20mm(0,1200,1,0);
    print_snapshot();
    check_ok=(f_cyc>=0)&&(f_rfault==1)&&(f_reason==8'h84)&&(f_bits==15'h0000)&&(f_sfault==0)&&(f_motion==8'h00)&&(f_imubad==0);
    // barrier timeout is 1000 DUT ms after WAIT_PARALLEL is entered
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)<950)check_ok=0;
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)>1200)check_ok=0;
    if(check_ok)
     $display("PASS scenario A_FRONT: exact-20 mm on FRONT ToF after turn -> route FAULT reason=0x84 barrier timeout, sensor faults 0, latch at +%0d ms",(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario A_FRONT: expected route 0x84 barrier-timeout latch, observed route_reason=%02h route_fault=%b sensor_fault=%b ch_faults=%02h/%02h/%02h",f_reason,f_rfault,f_sfault,f_ch0,f_ch1,f_ch2);
   end
   SC_A_SIDE:begin
    wait_turn_done(ok_turn);
    if(ok_turn)inject_20mm(2,1200,1,0);
    print_snapshot();
    check_ok=(f_cyc>=0)&&(f_rfault==1)&&(f_reason==8'h84)&&(f_bits==15'h0000)&&(f_sfault==0)&&(f_motion==8'h00)&&(f_imubad==0);
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)<950)check_ok=0;
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)>1200)check_ok=0;
    if(check_ok)
     $display("PASS scenario A_SIDE: exact-20 mm on RIGHT side ToF after turn -> route FAULT reason=0x84 barrier timeout, sensor faults 0, latch at +%0d ms",(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario A_SIDE: expected route 0x84 barrier-timeout latch, observed route_reason=%02h route_fault=%b sensor_fault=%b ch_faults=%02h/%02h/%02h",f_reason,f_rfault,f_sfault,f_ch0,f_ch1,f_ch2);
   end
   SC_B:begin
    wait_turn_done(ok_turn);
    // baseline expects a 0x84 latch; the fixed design must NOT fault, so bound
    // the observation window at ~4.5 DUT-s (3 barrier retries then BLOCKED).
    if(ok_turn)begin
     if(fixed_mode)inject_20mm(0,4500,0,0);
     else inject_20mm(0,1000000,1,500);
    end
    print_snapshot();
    check_ok=(f_cyc>=0)&&(f_rfault==1)&&(f_reason==8'h84)&&(f_bits==15'h0000)&&(f_sfault==0)&&(dut.control.fault_latched==1);
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)<950)check_ok=0;
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)>1200)check_ok=0;
    if(check_ok)
     $display("PASS scenario B: permanent exact-20 mm after turn -> single route FAULT 0x84, fault stays latched (no auto-clear), sensor faults 0");
    else
     $display("FAIL scenario B: expected permanent route 0x84 latch, observed route_reason=%02h route_fault=%b sensor_fault=%b latch=%b",f_reason,f_rfault,f_sfault,dut.control.fault_latched);
   end
   SC_C:begin
    wait_turn_done(ok_turn);
    if(ok_turn)inject_nack(20);
    print_snapshot();print_motor_off();
    check_ok=(f_cyc>=0)&&(f_sfault==1)&&(f_ch2[0]==1)&&(dut.control.fault_latched==1)&&(dut.control.auto_run==0);
    if(check_ok)
     $display("PASS scenario C: transient I2C NACK (present[2]=0, 20 ms) after turn -> sensor_fault latched via faults[0] (ch2 faults=%02h) at +%0d ms; channel communication self-recovered but fault is sticky",f_ch2,(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario C: expected sensor faults[0] latch, observed sensor_fault=%b ch2_faults=%02h latch=%b",f_sfault,f_ch2,dut.control.fault_latched);
   end
   SC_D:begin
    inject_imu(150,100);
    print_snapshot();print_motor_off();
    check_ok=(f_cyc>=0)&&(f_imubad==1)&&(dut.control.fault_latched==1)&&(f_sfault==0);
    if(check_ok)
     $display("PASS scenario D: IMU health-error burst 150 ms during rotation -> imu_bad latched (imu_error=%02h sensor_error=%02h) at +%0d ms",f_imuerr,f_senserr,(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario D: expected imu_bad latch, observed imu_bad=%b imu_error=%02h sensor_error=%02h latch=%b",f_imubad,f_imuerr,f_senserr,dut.control.fault_latched);
   end
   SC_E:begin
    // fixed design uses STOP_TIMEOUT_MS=3000 and finishes with reason=7, so the
    // fixed run holds the encoder injection long enough to exercise it.
    if(fixed_mode)inject_encoder(3500);
    else inject_encoder(1200);
    print_snapshot();print_motor_off();
    check_ok=(f_cyc>=0)&&(f_motion==8'h05)&&(dut.control.fault_latched==1)&&(f_sfault==0);
    if(check_ok)
     $display("PASS scenario E: sustained encoder counts during post-turn WAIT_STILL -> motion fault 5 (stop timeout) latched at +%0d ms",(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario E: expected motion_fault=5 latch, observed motion_fault=%02h latch=%b",f_motion,dut.control.fault_latched);
   end
   SC_I_HOLD:begin
    wait_turn_done(ok_turn);
    if(ok_turn)inject_hold20(2000,1000);
    print_snapshot();
    check_ok=(f_cyc>=0)&&(f_rfault==1)&&(f_reason==8'h84)&&(f_bits==15'h0000)&&(f_sfault==0)&&(dut.control.fault_latched==1);
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)<950)check_ok=0;
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)>1200)check_ok=0;
    if(check_ok)
     $display("PASS scenario I_HOLD: exact-20 mm with normal communication held 2 s after turn -> single route FAULT 0x84 at +%0d ms; hold20 reuses the last good distance but fresh/post_turn never assert again, sensor faults 0",(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario I_HOLD: expected route 0x84 latch, observed route_reason=%02h route_fault=%b sensor_fault=%b latch=%b",f_reason,f_rfault,f_sfault,dut.control.fault_latched);
   end
   SC_CTRL:begin
    wait_turn_done(ok_turn);
    if(ok_turn)inject_20mm(0,100,0,0);
    // the barrier must now complete and the route must move on
    wait_route(23,600000,w_ok);
    if(!w_ok)wait_route(6,600000,w_ok);
    print_snapshot();
    check_ok=(f_cyc<0)&&(w_ok==1)&&(dut.control.fault==0);
    if(check_ok)
     $display("PASS scenario CTRL: 100 ms exact-20 mm pulse after turn -> NO fault, route continued (state=%0d), proving the 1000 ms barrier timeout threshold",dut.route_state);
    else
     $display("FAIL scenario CTRL: expected no fault and route progress, observed fault=%b route_state=%0d reached=%0d",dut.control.fault,dut.route_state,w_ok);
   end
   SC_G_NEAR:begin
    wait_turn_done(ok_turn);
    if(ok_turn)inject_near(2,16'd60,500);
    // Fixed RTL: a real 60 mm obstacle-side reading cannot seed the box profile,
    // so the route must take the fixed-mileage backstop instead of looping in
    // SAMPLE_SHIFT until the barrier retries turn into PAUSED.
    if(fixed_mode)begin
     wait_route(6,4000*DUT_MS,w_ok);
     if(!w_ok)wait_route(21,2000*DUT_MS,w_ok);
     if(!w_ok)wait_route(22,2000*DUT_MS,w_ok);
     near_shift=w_ok;
    end
    print_snapshot();print_motor_off();
    if(fixed_mode)begin
     // Fixed RTL: a real 60 mm reading is trustworthy data (readiness no longer
     // masks anything <=85 mm); it is closer than AVOID_MM, so the route must
     // open the fixed-mileage backstop instead of waiting for a face profile.
     check_ok=(near_shift==1)&&(dut.control.fault==0)&&(f_cyc<0);
     if(check_ok)
      $display("PASS scenario G_NEAR: valid 60 mm (non-20 <=85 mm) on RIGHT side ToF after turn -> no fault, fixed-mileage SHIFT entered (route_state=%0d)",dut.route_state);
     else
      $display("FAIL scenario G_NEAR(fixed): expected no fault and fallback SHIFT, observed near_shift=%0d fault=%b route_state=%0d",near_shift,dut.control.fault,dut.route_state);
    end else begin
     check_ok=(f_cyc>=0)&&(f_sfault==1)&&(f_ch2[4]==1)&&(dut.control.fault_latched==1);
     if(check_ok)
      $display("PASS scenario G_NEAR: valid 60 mm (non-20 <=85 mm) on RIGHT side ToF after turn -> sensor_fault latched via faults[4] (ch2 faults=%02h) at +%0d ms",f_ch2,(f_cyc-mark_cyc)/DUT_MS);
     else
      $display("FAIL scenario G_NEAR: expected faults[4] near-distance latch, observed sensor_fault=%b ch2_faults=%02h latch=%b",f_sfault,f_ch2,dut.control.fault_latched);
    end
   end
   SC_H_TURN2:begin
    drive_to_second_turn(ok_done);
    if(!ok_done)begin $display("FAIL scenario H_TURN2: harness could not reach the second turn");$finish;end
    mark_valid=1;mark_cyc=cyc;
    $display("EVENT INJECT_EXACT20_TURN2 ch=1 mm=20 hold_ms=1200 route_state=%0d dut_ms=%0d time_ns=%0t",dut.route_state,cyc/DUT_MS,$time);
    ranges[31:16]=16'd20;
    limit=1200*DUT_MS;n=0;
    while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
    cycles(3*DUT_MS);
    ranges[31:16]=16'd2000;
    print_snapshot();
    check_ok=(f_cyc>=0)&&(f_rfault==1)&&(f_reason==8'h84)&&(f_bits==15'h0000)&&(f_sfault==0);
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)<950)check_ok=0;
    if(f_cyc>=0 && ((f_cyc-mark_cyc)/DUT_MS)>1200)check_ok=0;
    if(check_ok)
     $display("PASS scenario H_TURN2: exact-20 mm on LEFT ToF after second turn (7->13) -> route FAULT reason=0x84 barrier timeout at +%0d ms",(f_cyc-mark_cyc)/DUT_MS);
    else
     $display("FAIL scenario H_TURN2: expected route 0x84 latch at second barrier, observed route_reason=%02h route_fault=%b sensor_fault=%b",f_reason,f_rfault,f_sfault);
   end
   SC_K_CLIP:begin
    // Obstacle-side (RIGHT) lane permanently clamped at 20 mm right after the
    // first rotation. Old RTL: 4x1000 ms barrier timeouts -> BLOCKED/PAUSED.
    // Fixed RTL: the confirmed clip is a fresh shot, so the route opens the
    // fixed-mileage detour immediately.
    wait_turn_done(ok_turn);
    mark_valid=1;mark_cyc=cyc;
    $display("EVENT INJECT_CLIP_FALLBACK ch=2 mm=20 route_state=%0d dut_ms=%0d time_ns=%0t",
     dut.route_state,cyc/DUT_MS,$time);
    ranges[47:32]=16'd20;
    if(fixed_mode)begin
     wait_route(6,2000*DUT_MS,w_ok);
     if(!w_ok)wait_route(21,2000*DUT_MS,w_ok);
     reach_shift=w_ok;
     // Keep the obstacle-side lane clamped for the whole detour: SHIFT and PASS
     // must both use the fixed-mileage backstop, then the final barrier must let
     // the route resume CRUISE.
     if(w_ok)begin
      wait_route(2,80000*DUT_MS,w_ok);
      fallback_done=w_ok;
     end
    end else begin
     n=0;limit=5000*DUT_MS;
     while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
    end
    cycles(5*DUT_MS);
    ranges[47:32]=16'd2000;
    cycles(5*DUT_MS);
    print_snapshot();
    $display("EVIDENCE CLIP_FALLBACK reached_shift=%0d fallback_done=%0d fixed=%0d route_state=%0d paused=%b blocked_seen=%b dut_ms=%0d",
     reach_shift,fallback_done,fixed_mode,dut.route_state,pause_seen,blocked_seen,cyc/DUT_MS);
    if(fixed_mode)begin
     check_ok=(reach_shift==1)&&(fallback_done==1)&&(pause_seen==0)&&(blocked_seen==0)&&(dut.control.fault==0);
     if(check_ok)
      $display("PASS scenario K_CLIP_FALLBACK(fixed): permanent obstacle-side 20 mm clip -> fixed-mileage detour completed and CRUISE resumed (route_state=%0d)",dut.route_state);
     else
      $display("FAIL scenario K_CLIP_FALLBACK(fixed): reached_shift=%0d fallback_done=%0d paused=%b blocked=%b fault=%b",reach_shift,fallback_done,pause_seen,blocked_seen,dut.control.fault);
    end
    check_ok=(!fixed_mode)&&(f_cyc>=0)&&(f_rfault==1)&&(f_reason==8'h84)&&(f_bits==15'h0000)&&(f_sfault==0);
    if(check_ok)
     $display("PASS scenario K: baseline latched 0x84 on a permanent side clip after turn");
    else if(!fixed_mode)
     $display("FAIL scenario K: expected baseline 0x84 latch, observed route_reason=%02h route_fault=%b",f_reason,f_rfault);
   end
   SC_PASS_CLIP:begin
    // Board evidence: the obstacle-side lane was healthy through WAIT_BACK and
    // only clamped at 20 mm after PASS started. The edge detector then never
    // fired again and the route drove on until front_block
    // (BLOCKED reason 0x04, 3 of 5 runs on 2026-10-29).
    // Drive a healthy box-edge detour up to WAIT_BACK first (same helper as
    // H_TURN2), then let PASS start and clamp the obstacle-side lane.
    drive_to_second_turn(ok_done);
    if(!ok_done)begin $display("FAIL scenario PASS_CLIP: harness could not reach WAIT_BACK");$finish;end
    wait_route(9,10000*DUT_MS,w_ok);
    pass_reached=w_ok;
    if(fixed_mode && w_ok)begin
     $display("EVENT INJECT_PASS_CLIP ch=2 mm=20 route_state=9 dut_ms=%0d",cyc/DUT_MS);
     ranges[47:32]=16'd20;
     wait_route(2,80000*DUT_MS,w_ok);
     pass_clip_done=w_ok;
    end else begin
     n=0;limit=5000*DUT_MS;
     while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
    end
    cycles(5*DUT_MS);
    ranges[47:32]=16'd2000;
    cycles(5*DUT_MS);
    print_snapshot();
    $display("EVIDENCE PASS_CLIP pass_reached=%0d pass_clip_done=%0d fixed=%0d route_state=%0d paused=%b blocked_seen=%b fault=%b dut_ms=%0d",
     pass_reached,pass_clip_done,fixed_mode,dut.route_state,pause_seen,blocked_seen,dut.control.fault,cyc/DUT_MS);
    if(fixed_mode)begin
     // Fixed RTL: a clip that appears during PASS must finish the pass on fixed
     // mileage (fallback_pass becomes active via side_unusable) and resume CRUISE.
     check_ok=(pass_reached==1)&&(pass_clip_done==1)&&(pause_seen==0)&&(blocked_seen==0)&&(dut.control.fault==0);
     if(check_ok)
      $display("PASS scenario PASS_CLIP(fixed): obstacle-side 20 mm clip starting in PASS -> fixed-mileage pass finished and CRUISE resumed (route_state=%0d)",dut.route_state);
     else
      $display("FAIL scenario PASS_CLIP(fixed): pass_reached=%0d done=%0d paused=%b blocked=%b fault=%b",pass_reached,pass_clip_done,pause_seen,blocked_seen,dut.control.fault);
    end
   end
   SC_PASS_FRONT:begin
    // Board evidence (2026-10-29 22:52-22:59): while PASS drove past the box the
    // front lane saw the box corner for a few hundred ms at a time; the old RTL
    // turned that graze into BLOCKED reason 0x04. The fixed RTL only blocks when
    // the front lane stays below AVOID_MM for FRONT_LEG_MS, so the pass finishes.
    drive_to_second_turn(ok_done);
    if(!ok_done)begin $display("FAIL scenario PASS_FRONT: harness could not reach WAIT_BACK");$finish;end
    wait_route(9,10000*DUT_MS,w_ok);
    pass_reached=w_ok;
    if(fixed_mode && w_ok)begin
     $display("EVENT INJECT_PASS_FRONT_GRAZE route_state=9 dut_ms=%0d",cyc/DUT_MS);
     // Same blind-pass condition as scenario 11 (side lane clips once PASS is
     // running) so the pass finishes on fixed mileage, plus short front grazes
     // that must be filtered by FRONT_LEG_MS instead of blocking the leg.
     ranges[47:32]=16'd20;
     ranges[15:0]=16'd60;cycles(200*DUT_MS);
     ranges[15:0]=16'd2000;cycles(200*DUT_MS);
     ranges[15:0]=16'd60;cycles(200*DUT_MS);
     ranges[15:0]=16'd2000;cycles(200*DUT_MS);
     ranges[15:0]=16'd60;cycles(200*DUT_MS);
     ranges[15:0]=16'd2000;cycles(200*DUT_MS);
     wait_route(2,80000*DUT_MS,w_ok);
     pass_front_done=w_ok;
    end else begin
     n=0;limit=5000*DUT_MS;
     while(n<limit && f_cyc<0)begin @(posedge clk);n=n+1;end
    end
    cycles(5*DUT_MS);
    ranges[15:0]=16'd2000;
    ranges[47:32]=16'd2000;
    cycles(5*DUT_MS);
    print_snapshot();
    $display("EVIDENCE PASS_FRONT pass_reached=%0d pass_front_done=%0d fixed=%0d route_state=%0d paused=%b blocked_seen=%b fault=%b dut_ms=%0d",
     pass_reached,pass_front_done,fixed_mode,dut.route_state,pause_seen,blocked_seen,dut.control.fault,cyc/DUT_MS);
    if(fixed_mode)begin
     check_ok=(pass_reached==1)&&(pass_front_done==1)&&(pause_seen==0)&&(blocked_seen==0)&&(dut.control.fault==0);
     if(check_ok)
      $display("PASS scenario PASS_FRONT_GRAZE(fixed): 3x200ms front-lane graze during PASS -> pass finished and CRUISE resumed (route_state=%0d)",dut.route_state);
     else
      $display("FAIL scenario PASS_FRONT_GRAZE(fixed): pass_reached=%0d done=%0d paused=%b blocked=%b fault=%b",pass_reached,pass_front_done,pause_seen,blocked_seen,dut.control.fault);
    end
   end
   default:$fatal(1,"unknown scenario id %0d",scenario);
  endcase

  if(fixed_mode)begin
   // Observation-only run of the fixed RTL: allow the route to continue or the
   // fault to self-clear, then publish the authoritative fixed-version result.
   cycles(300*DUT_MS);
   $display("FIXED_RUN_OBSERVED scenario=%0d fault_seen=%b fault_latched=%b fault_dt_ms=%0d route_state=%0d auto_run=%b avoid_mode=%b sensor_fault=%b fault_bits=%04h ch0=%02h ch1=%02h ch2=%02h motion_fault=%02h motion_fault_code=%02h motion_reason=%0d motion_done=%b motion_aborted=%b imu_bad=%b imu_error=%02h route_fault=%b route_reason=%02h fault_events=%0d status=%02h fault_led=%b post_turn=%b distances=%04h/%04h/%04h tof_resets=%0d/%0d/%0d tof_shots=%0d/%0d/%0d clip_confirm=%0d/%0d/%0d driver_recover=%0d/%0d/%0d paused=%b blocked_seen=%b reached_shift=%0d fallback_done=%0d near_shift=%0d pass_reached=%0d pass_clip_done=%0d pass_front_done=%0d dut_ms=%0d",
    scenario,(f_cyc>=0),dut.control.fault_latched,(f_cyc<0)?-1:(f_cyc-mark_cyc)/DUT_MS,dut.route_state,
    dut.control.auto_run,dut.avoid_mode,dut.control.sensor_fault,dut.sensors.fault_bits,
    dut.sensors.channels[0].faults,dut.sensors.channels[1].faults,dut.sensors.channels[2].faults,
    dut.control.motion_fault,dut.control.motion.fault_code,dut.control.motion.reason,
    dut.control.motion.done,dut.control.motion.aborted,
    dut.control.imu_bad,dut.control.imu_error,
    dut.control.route.route_fault,dut.control.route.reason,fault_events,dut.control.status,dut.led_fault,
    dut.post_turn,dut.distances[15:0],dut.distances[31:16],dut.distances[47:32],
    resets[0],resets[1],resets[2],shots[0],shots[1],shots[2],
    req_cnt0,req_cnt1,req_cnt2,rec_cnt0,rec_cnt1,rec_cnt2,
    pause_seen,blocked_seen,reach_shift,fallback_done,near_shift,pass_reached,pass_clip_done,pass_front_done,cyc/DUT_MS);
   completed=1;
  end else if(check_ok)begin
   $display("RESULT scenario=%0d PASS fault_seen=%b fault_dt_ms=%0d",scenario,(f_cyc>=0),(f_cyc<0)?-1:(f_cyc-mark_cyc)/DUT_MS);
   completed=1;
  end else begin
   $display("RESULT scenario=%0d FAIL",scenario);
  end
  cycles(1000);
  $finish;
 end

 // Absolute watchdog: the longest scenario needs ~400 ms of simulation time.
 initial begin #(64'd2500000000);$fatal(1,"fault-turn suite watchdog state=%0d reason=%02h",dut.route_state,dut.control.route_reason);end
endmodule
