`timescale 1ns/1ps
// Single wheel hold-to-run bench test; time limit is not a watchdog.
module robot_closed_loop_test_top #(
 parameter signed [15:0] TARGET=10,KP=32,KI=1,KD=0,
 parameter signed [15:0] TARGET_L=TARGET,TARGET_R=TARGET,
 parameter integer WATCHDOG_ENABLE=0,WATCHDOG_CYCLES=25000000,UART_BIT_CYCLES=434,
 parameter [4:0] GAIN_SHIFT=0,
 parameter integer TEST_RIGHT=0,TEST_BOTH=0,WINDOW_CYCLES=500000,FILTER_LENGTH=4,
 DEBOUNCE_CYCLES=500000,RUN_CYCLES=100000000,STARTUP_CYCLES=250000,REPORT_CYCLES=5000000
)(input wire clk_50m,reset_n,key_enable_n,key_estop_n,
 input wire uart_rx,
 input wire encoder_a_l,encoder_b_l,encoder_a_r,encoder_b_r,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx,
 output wire led_enable,led_estop,led_fault,led_saturated);
 localparam [15:0] PWM_LIMIT=625;
 localparam LEFT_ACTIVE=(TEST_BOTH!=0 || TEST_RIGHT==0), RIGHT_ACTIVE=(TEST_BOTH!=0 || TEST_RIGHT!=0);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe,enable_pipe,estop_pipe;
 always @(posedge clk_50m or negedge reset_n)
  if(!reset_n) begin reset_pipe<=0;enable_pipe<=0;estop_pipe<=0;end
  else begin reset_pipe<={reset_pipe[0],1'b1};enable_pipe<={enable_pipe[0],key_enable_n};estop_pipe<={estop_pipe[0],key_estop_n};end
 wire rst_n=reset_pipe[1];
 wire heartbeat_seen,watchdog_expired;
 generate if(WATCHDOG_ENABLE) begin: wd
 uart_heartbeat_watchdog #(.BIT_CYCLES(UART_BIT_CYCLES),.TIMEOUT_CYCLES(WATCHDOG_CYCLES)) watchdog(
 .clk(clk_50m),.rst_n(rst_n),.rx(uart_rx),.seen(heartbeat_seen),.expired(watchdog_expired));
 end else begin: no_wd
 assign heartbeat_seen=1'b1;assign watchdog_expired=1'b0;
 end endgenerate
 wire enc_fault_l,enc_fault_r;
 wire encoder_fault=enc_fault_l|enc_fault_r;
 wire fault_event=!key_estop_n|encoder_fault;
 reg local_fault;
 // Capture sub-clock estop pulses. Only KEY0 can clear the latch.
 always @(posedge clk_50m or posedge fault_event)
  if(fault_event) local_fault<=1;else if(!rst_n) local_fault<=0;
 reg [31:0] press_count,release_count,run_count;
 reg armed,run_request;
 wire immediate_stop=!reset_n|key_enable_n|!key_estop_n|encoder_fault|local_fault|watchdog_expired;
 always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin press_count<=0;release_count<=0;run_count<=0;armed<=0;run_request<=0;end
  else if(local_fault || encoder_fault || watchdog_expired || !heartbeat_seen || !estop_pipe[1]) begin press_count<=0;release_count<=0;run_count<=0;armed<=0;run_request<=0;end
  else if(key_enable_n || enable_pipe[1]) begin
   run_request<=0;run_count<=0;press_count<=0;
   if(key_enable_n && enable_pipe[1]) begin
    if(release_count<DEBOUNCE_CYCLES) release_count<=release_count+1;
    if(release_count>=DEBOUNCE_CYCLES-1) armed<=1;
   end else release_count<=0;
  end else begin
   release_count<=0;
   if(run_request) begin
    if(run_count>=RUN_CYCLES-1) begin run_request<=0;armed<=0;press_count<=0;end
    else run_count<=run_count+1;
   end else if(armed) begin
    if(press_count>=DEBOUNCE_CYCLES-1) begin run_request<=1;armed<=0;run_count<=0;press_count<=0;end
    else press_count<=press_count+1;
   end
  end
 end
 wire signed [31:0] pos_l,pos_r;
 wire signed [1:0] delta_l,delta_r;
 wire [31:0] illegal_l,illegal_r;
 dual_ab_quadrature_encoder #(.FILTER_CYCLES(50),.INVERT_L(1),.INVERT_R(0)) enc(
 .clk(clk_50m),.rst_n(rst_n),.encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),.encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),
 .clear_fault_l(1'b0),.clear_fault_r(1'b0),.position_l(pos_l),.position_r(pos_r),.delta_count_l(delta_l),.delta_count_r(delta_r),
 .direction_l(),.direction_r(),.illegal_transition_l(illegal_l),.illegal_transition_r(illegal_r),.encoder_fault_l(enc_fault_l),.encoder_fault_r(enc_fault_r));
 wire signed [15:0] delta16_l={{14{delta_l[1]}},delta_l},delta16_r={{14{delta_r[1]}},delta_r};
 wire signed [31:0] speed_raw_l,speed_raw_r,speed_l,speed_r;
 wire valid_l,valid_r;
 speed_measurement_and_velocity_filter #(.WINDOW_CYCLES(WINDOW_CYCLES),.FILTER_LENGTH(FILTER_LENGTH),.ZERO_PAD_WARMUP(1)) sm_l(
 .clk(clk_50m),.rst_n(rst_n),.delta_count(delta16_l),.speed_raw(speed_raw_l),.speed_filtered(speed_l),.position(),.speed_valid(valid_l),.direction(),.zero_speed());
 speed_measurement_and_velocity_filter #(.WINDOW_CYCLES(WINDOW_CYCLES),.FILTER_LENGTH(FILTER_LENGTH),.ZERO_PAD_WARMUP(1)) sm_r(
 .clk(clk_50m),.rst_n(rst_n),.delta_count(delta16_r),.speed_raw(speed_raw_r),.speed_filtered(speed_r),.position(),.speed_valid(valid_r),.direction(),.zero_speed());
 function signed [15:0] saturate16;
 input signed [31:0] value;
 begin if(value>32767) saturate16=16'sh7fff;else if(value< -32768) saturate16=16'sh8000;else saturate16=value[15:0];end
 endfunction
 wire signed [15:0] feedback_l=saturate16(speed_l),feedback_r=saturate16(speed_r);
 wire signed [15:0] target_l=LEFT_ACTIVE ? TARGET_L:16'sd0,target_r=RIGHT_ACTIVE ? TARGET_R:16'sd0;
 wire signed [15:0] control_l,control_r;
 wire sat_l,sat_r,fault_latched,safety_active;
 wire control_enable=run_request && !immediate_stop && !fault_latched;
 // Registered speed_valid is consumed at the next edge, with updated feedback.
 dual_speed_pi_pid_controller pi(.clk(clk_50m),.rst_n(rst_n),.control_tick(valid_l&valid_r),.enable(control_enable),
 .target_l(target_l),.target_r(target_r),.feedback_l(feedback_l),.feedback_r(feedback_r),
 .kp_l(LEFT_ACTIVE ? KP:16'sd0),.ki_l(LEFT_ACTIVE ? KI:16'sd0),.kd_l(LEFT_ACTIVE ? KD:16'sd0),
 .kp_r(RIGHT_ACTIVE ? KP:16'sd0),.ki_r(RIGHT_ACTIVE ? KI:16'sd0),.kd_r(RIGHT_ACTIVE ? KD:16'sd0),
 .gain_shift_l(GAIN_SHIFT),.gain_shift_r(GAIN_SHIFT),.output_limit_l(LEFT_ACTIVE ? PWM_LIMIT:16'd0),.output_limit_r(RIGHT_ACTIVE ? PWM_LIMIT:16'd0),
 .control_l(control_l),.control_r(control_r),.output_valid(),.sat_l(sat_l),.sat_r(sat_r),.active_l(),.active_r());
 wire signed [15:0] ramp_l,ramp_r,safe_l,safe_r;
 pwm_and_startup_ramp ramp(.clk(clk_50m),.rst_n(rst_n),.enable(control_enable),
 .pwm_cmd_l(LEFT_ACTIVE ? control_l:16'sd0),.pwm_cmd_r(RIGHT_ACTIVE ? control_r:16'sd0),
 .pwm_limit(PWM_LIMIT),.pwm_min(16'd0),.ramp_step(16'd1),.pwm_out_l(),.pwm_out_r(),.pwm_actual_l(ramp_l),.pwm_actual_r(ramp_r));
 wire [3:0] fault_code;
 safety_protection_and_fault_arbitration #(.STARTUP_STABLE_CYCLES(STARTUP_CYCLES),.PWM_LIMIT(625),.RANGE_CHECK_ENABLE(0)) safety(
 .clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.system_ready(1'b1),.clear_fault(1'b0),.emergency_stop(!key_estop_n),
 .over_current(1'b0),.under_voltage(1'b0),.over_temperature(1'b0),.watchdog_timeout(watchdog_expired),.external_fault(encoder_fault|local_fault),
 .range_valid(1'b1),.front_distance(16'hffff),.pwm_cmd_l(ramp_l),.pwm_cmd_r(ramp_r),.pwm_safe_l(safe_l),.pwm_safe_r(safe_r),
 .stby(),.safety_active(safety_active),.fault_latched(fault_latched),.fault_code(fault_code),.obstacle_state(),.startup_done());
 tb6612_driver_interface driver(.clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.emergency_stop(!key_estop_n),
 .fault(encoder_fault|local_fault|fault_latched),.watchdog_timeout(watchdog_expired),
 .pwm_signed_l(LEFT_ACTIVE ? safe_l:16'sd0),.pwm_signed_r(RIGHT_ACTIVE ? safe_r:16'sd0),
 .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY));
 wire signed [15:0] actual_l=STBY ? safe_l:16'sd0,actual_r=STBY ? safe_r:16'sd0;
 assign led_enable=control_enable;assign led_estop=!key_estop_n;assign led_fault=local_fault|encoder_fault|fault_latched;
 assign led_saturated=(LEFT_ACTIVE && sat_l)|(RIGHT_ACTIVE && sat_r);
 // Low word first: L,R,TL,TR,VL,VR,SFL,SFR,PL,PR,SL,SR,EN,ST,IL,IR,FL,FR,FAULT,FC.
 wire [639:0] telemetry_values={28'd0,fault_code,31'd0,led_fault,31'd0,enc_fault_r,31'd0,enc_fault_l,
 illegal_r,illegal_l,31'd0,STBY,31'd0,control_enable,31'd0,(RIGHT_ACTIVE ? sat_r:1'b0),31'd0,(LEFT_ACTIVE ? sat_l:1'b0),
 {{16{actual_r[15]}},actual_r},{{16{actual_l[15]}},actual_l},speed_r,speed_l,speed_raw_r,speed_raw_l,
 {{16{target_r[15]}},target_r},{{16{target_l[15]}},target_l},pos_r,pos_l};
 wire legacy_tx,extended_tx;
 generate if(WATCHDOG_ENABLE) begin: extended_report
 closed_loop_uart_telemetry #(.REPORT_CYCLES(REPORT_CYCLES),.FIELD_COUNT(22)) telemetry(
 .clk(clk_50m),.rst_n(rst_n),.values({31'd0,watchdog_expired,31'd0,heartbeat_seen,telemetry_values}),.tx(extended_tx));
 end endgenerate
 closed_loop_uart_telemetry #(.REPORT_CYCLES(REPORT_CYCLES)) telemetry(.clk(clk_50m),.rst_n(rst_n),.values(telemetry_values),.tx(legacy_tx));
 assign uart_tx=WATCHDOG_ENABLE ? extended_tx:legacy_tx;
endmodule


