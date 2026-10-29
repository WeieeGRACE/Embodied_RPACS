`timescale 1ns/1ps
module system_motion #(
 parameter integer CLK_HZ=50000000,BAUD=115200,WINDOW_CYCLES=CLK_HZ/100,
 PWM_HZ=20000,ENC_FILTER=50,DIR_DEADTIME=2500,
 COUNTS_L=1456,COUNTS_R=1456,UM_PER_REV_L=207345,UM_PER_REV_R=207345,
 YAW_SIGN=-1,GOAL_UM=200000,GOAL_MDEG=90000,ACTION_MS=10000,
 IMU_POWERUP=CLK_HZ/50,IMU_WAIT=CLK_HZ/50,IMU_RETRY=CLK_HZ,
 IMU_SAMPLE=CLK_HZ/200,IMU_SPI_HALF=25,IMU_HEALTH=CLK_HZ,
 IMU_TIMEOUT=CLK_HZ/10,CALIBRATION_SAMPLES=200,
 parameter signed [15:0] KP=32,KI=1,KD=0,
 parameter CONFIG_HEX_FILE="src/imu/imu660ra_config.hex"
)(
 input wire clk_50m,reset_n,stop_gate,stop_request,uart_rx,voice_enable,
 input wire menu_valid,input wire [7:0] menu_command,
 input wire gear_commit,input wire [1:0] gear_value,
 input wire [47:0] distances,input wire [2:0] tof_seen,tof_valid,post_turn,sample_mask,
 input wire sensor_fault,input wire [2:0] sensor_degraded,
 output wire sensor_refresh,sensor_monitor,
 output wire avoid_mode,auto_run,waiting,show_avoid,show_root,output wire [4:0] route_state,
 output wire [7:0] route_reason,
 input wire encoder_a_l,encoder_b_l,encoder_a_r,encoder_b_r,
 input wire imu_spi_miso,imu_int1,imu_int2,imu_gnd_reserved,
 output wire imu_spi_sck,imu_spi_mosi,imu_cs_n,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx,
 output wire led_ready,led_run,led_fault,
 output wire imu_ready,calibrating,attitude_valid,
 output wire signed [31:0] roll_mdeg,pitch_mdeg,yaw_wrapped,
 output wire signed [31:0] pos_l,pos_r,speed_l,speed_r,
 output wire [7:0] imu_error,sensor_error,
 output reg [1:0] gear,output reg [7:0] status,
 output wire [7:0] motion_fault,
 output wire command_valid,output wire [7:0] command,
 output wire [7:0] response,output wire [11:0] event_errors,
 output wire [7:0] start_diag,output wire [63:0] debug_state,output wire debug_valid
);
 wire clk=clk_50m;
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe;
 always @(posedge clk or negedge reset_n)
 if(!reset_n)reset_pipe<=0;else reset_pipe<={reset_pipe[0],1'b1};
 wire rst_n=reset_pipe[1];
 wire signed [15:0] ax,ay,az,gx,gy,gz;
 wire sample_valid;

 imu660ra_driver #(.CLK_HZ(CLK_HZ),.SPI_HALF_PERIOD(IMU_SPI_HALF),
 .POWERUP_CYCLES(IMU_POWERUP),.CONFIG_WAIT_CYCLES(IMU_WAIT),.RETRY_CYCLES(IMU_RETRY),
 .ID_RETRY_CYCLES(CLK_HZ/1000),.SAMPLE_CYCLES(IMU_SAMPLE),
 .HEALTH_CHECK_CYCLES(IMU_HEALTH),.CONFIG_HEX_FILE(CONFIG_HEX_FILE)) imu(
 .clk(clk),.rst_n(rst_n),.spi_miso(imu_spi_miso),.imu_int1(imu_int1),.imu_int2(imu_int2),
 .spi_sclk(imu_spi_sck),.spi_mosi(imu_spi_mosi),.spi_cs_n(imu_cs_n),
 .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
 .sample_valid(sample_valid),.sample_sequence(),.chip_id(),.init_status(),
 .power_conf_readback(),.init_ctrl_readback(),.sensor_error(sensor_error),
 .ready(imu_ready),.error_code(imu_error),.int1_sync(),.int2_sync());
 wire attitude_update;
 wire signed [31:0] yaw;
 wire yaw_valid;
 imu_attitude_fixed #(.CALIBRATION_SAMPLES(CALIBRATION_SAMPLES)) attitude(
 .clk(clk),.rst_n(rst_n),.ready_in(imu_ready),.sample_valid(sample_valid),
 .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
 .calibrating(calibrating),.attitude_valid(attitude_valid),.update_valid(attitude_update),
 .roll_mdeg(roll_mdeg),.pitch_mdeg(pitch_mdeg),.yaw_mdeg(yaw_wrapped),.gyro_bias_x(),.gyro_bias_y(),.gyro_bias_z());
 asr_yaw_unwrap #(.YAW_SIGN(YAW_SIGN)) unwrap(
 .clk(clk),.rst_n(rst_n),.update_valid(attitude_update),.wrapped(yaw_wrapped),.unwrapped(yaw),.valid(yaw_valid));
 reg ready_seen,yaw_seen,speed_seen;
 reg [31:0] sample_age,yaw_age;
 wire imu_bad=imu_error!=0 || sensor_error!=0 || (ready_seen && !imu_ready) ||
 (imu_ready && !sample_valid && sample_age>=IMU_TIMEOUT-1) ||
 (yaw_seen && !yaw_valid && yaw_age>=IMU_TIMEOUT-1);
 // A brief IMU/SPI glitch must not latch the whole demo. The raw condition
 // still shows in the event log, but the system fault requires it to persist
 // for IMU_FAULT_MS and the motion FSM has its own feedback-abort path.
 localparam integer IMU_FAULT_MS=2000;
 reg [15:0] ms_ticks,imu_bad_ms;
 wire ms_tick=ms_ticks==CLK_HZ/1000-1;
 wire imu_bad_q=imu_bad && imu_bad_ms>=IMU_FAULT_MS-1;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin ms_ticks<=0;imu_bad_ms<=0;end
 else begin
 if(ms_tick)ms_ticks<=0;else ms_ticks<=ms_ticks+1'b1;
 if(!imu_bad)imu_bad_ms<=0;
 else if(ms_tick && imu_bad_ms!=16'hffff)imu_bad_ms<=imu_bad_ms+1'b1;
 end
 wire signed [31:0] um_l,um_r,raw_l,raw_r;
 wire signed [1:0] delta_l,delta_r;
 wire valid_l,valid_r,enc_fault_l,enc_fault_r;
 wire [31:0] illegal_l,illegal_r;
 dual_ab_quadrature_encoder #(.FILTER_CYCLES(ENC_FILTER),.INVERT_L(1),.INVERT_R(0),
 .ILLEGAL_TOLERANCE(5),.CLEAN_DECAY(256)) enc(
 .clk(clk),.rst_n(rst_n),.encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),
 .encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),.clear_fault_l(1'b0),.clear_fault_r(1'b0),
 .position_l(pos_l),.position_r(pos_r),.delta_count_l(delta_l),.delta_count_r(delta_r),
 .direction_l(),.direction_r(),.illegal_transition_l(illegal_l),.illegal_transition_r(illegal_r),
 .encoder_fault_l(enc_fault_l),.encoder_fault_r(enc_fault_r));
 asr_distance #(.UM_PER_REV(UM_PER_REV_L),.COUNTS_PER_REV(COUNTS_L)) distance_l(
 .clk(clk),.rst_n(rst_n),.delta(delta_l),.position_um(um_l));
 asr_distance #(.UM_PER_REV(UM_PER_REV_R),.COUNTS_PER_REV(COUNTS_R)) distance_r(
 .clk(clk),.rst_n(rst_n),.delta(delta_r),.position_um(um_r));
 speed_measurement_and_velocity_filter #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES),
 .FILTER_LENGTH(4),.ZERO_PAD_WARMUP(1)) measure_l(
 .clk(clk),.rst_n(rst_n),.delta_count({{14{delta_l[1]}},delta_l}),
 .speed_raw(raw_l),.speed_filtered(speed_l),.position(),.speed_valid(valid_l),.direction(),.zero_speed());
 speed_measurement_and_velocity_filter #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES),
 .FILTER_LENGTH(4),.ZERO_PAD_WARMUP(1)) measure_r(
 .clk(clk),.rst_n(rst_n),.delta_count({{14{delta_r[1]}},delta_r}),
 .speed_raw(raw_r),.speed_filtered(speed_r),.position(),.speed_valid(valid_r),.direction(),.zero_speed());
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin ready_seen<=0;yaw_seen<=0;speed_seen<=0;sample_age<=0;yaw_age<=0;end
 else begin
 if(imu_ready)ready_seen<=1;
 if(sample_valid)sample_age<=0;else if(imu_ready && sample_age<IMU_TIMEOUT)sample_age<=sample_age+1'b1;
 if(yaw_valid)begin yaw_age<=0;yaw_seen<=1;end else if(yaw_seen && yaw_age<IMU_TIMEOUT)yaw_age<=yaw_age+1'b1;
 if(valid_l && valid_r)speed_seen<=1;
 end
 reg fault_latched;
 wire tx_overflow;
 wire route_fault;
 wire fault_now=sensor_fault || route_fault || imu_bad_q || enc_fault_l || enc_fault_r || tx_overflow || motion_fault!=0;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)fault_latched<=0;else if(fault_now)fault_latched<=1;
 wire fault=fault_now || fault_latched;
 wire modules_ready=imu_ready && attitude_valid && yaw_seen && speed_seen && !calibrating && !fault;
 wire ack_valid;wire [7:0] ack;
 asr_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(
 .clk(clk),.rst_n(rst_n),.rx(uart_rx),.command_valid(command_valid),.command(command),.ack_valid(ack_valid),.ack(ack));
 wire cv=command_valid;
 wire voice_command_valid;
 wire busy,cmd_ready,done,aborted,rejected,drive_enable,controller_clear,command_pending;
 wire stopping,route_cancel,route_press,request_refresh,manual_active;
 wire [7:0] action,mode_status;wire aux_valid;wire [7:0] aux_type,aux_code;
 wire start;
 reg issue_valid;
 reg [230:0] issue_payload;
 system_dispatch #(.CLK_HZ(CLK_HZ)) dispatch(
 .clk(clk),.rst_n(rst_n),.voice_enable(voice_enable),.cv(cv),.command(command),
 .voice_command_valid(voice_command_valid),
 .ack_valid(ack_valid),.ack(ack),.menu_valid(menu_valid),.menu_command(menu_command),
 .stop_gate(stop_gate),.stop_request(stop_request),.ready(modules_ready),.fault(fault),
 .busy(busy || command_pending || issue_valid),.cmd_ready(cmd_ready),.done(done),
 .tof_ready(&tof_seen && &tof_valid),.post_fresh(&post_turn),.route_state(route_state),
 .avoid_mode(avoid_mode),.auto_run(auto_run),.waiting(waiting),.manual_active(manual_active),
 .manual_action(action),.manual_submit(start),.stopping(stopping),.route_cancel(route_cancel),
 .route_press(route_press),.refresh(request_refresh),.show_avoid(show_avoid),.show_root(show_root),
 .response(response),.aux_valid(aux_valid),.aux_type(aux_type),.aux_code(aux_code),.mode_status(mode_status),.start_diag(start_diag));
 // Registered by the USB logger; independent from ASR control traffic.
 assign debug_state={start_diag,command,menu_command,ack,aux_type,aux_code,
  3'd0,route_state,voice_enable,modules_ready,(&tof_seen && &tof_valid),fault,
  busy,waiting,auto_run,avoid_mode};
 assign debug_valid=menu_valid || command_valid || ack_valid || aux_valid;
 wire rotate=action==8'h12 || action==8'h13;
 wire reverse=action==8'h11 || action==8'h13;
 wire [2:0] phase;
 wire [3:0] reason;
 wire signed [31:0] target_um_l,target_um_r;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)gear<=1;
 else if(gear_commit && !stopping && !fault)gear<=gear_value;
 else if(voice_command_valid && voice_enable && !fault && !stopping)begin
 if(command==8'h14 && gear<2)gear<=gear+1'b1;
 if(command==8'h15 && gear>0)gear<=gear-1'b1;
 end
 localparam integer LOW_UM=(64'd50000*WINDOW_CYCLES)/CLK_HZ;
 reg [31:0] gear_speed;
 reg [31:0] requested_l,requested_r,goal;
 reg [3:0] requested_mode;
 reg [1:0] requested_end;
 always @* begin
 requested_l=rotate ? LOW_UM:gear_speed;
 requested_r=requested_l;
 requested_mode=rotate ? 6:7;requested_end=rotate ? 3:2;
 goal=rotate ? GOAL_MDEG:GOAL_UM;
 end
 wire route_start,route_stop,route_rotate,route_reverse,route_refresh;
 wire [1:0] route_end;wire [31:0] route_goal,route_speed,route_cap;
 wire signed [31:0] route_heading;
 wire sensor_hold=|sensor_degraded;
 wire command_submit,buffer_busy,buffer_ready;
 wire [230:0] payload;wire [132:0] context_out;
 obstacle_route #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES)) route(
 .clk(clk),.rst_n(rst_n && auto_run),.press(route_press),
 .ready(modules_ready && &tof_seen && &tof_valid),.fault(fault),.distances(distances),
 .post_turn(post_turn),.sample_mask(sample_mask),.sensor_hold(sensor_hold),.position_l(um_l),.position_r(um_r),.yaw(yaw),
 .busy(buffer_busy),.cmd_ready(buffer_ready),.done(done),.aborted(aborted),.rejected(rejected),
 .start(route_start),.stop(route_stop),.rotate(route_rotate),.reverse(route_reverse),.end_kind(route_end),
 .goal(route_goal),.speed(route_speed),.speed_cap(route_cap),.refresh(route_refresh),.state(route_state),
 .reason(route_reason),.route_fault(route_fault),.target_heading(route_heading));
 assign sensor_refresh=request_refresh || route_refresh;
 assign sensor_monitor=auto_run && !route_cancel;
 wire cruise=route_state==1 || route_state==2;

 always @(posedge clk or negedge rst_n)
 if(!rst_n)gear_speed<=2*LOW_UM;else case(gear)
 0:gear_speed<=LOW_UM;1:gear_speed<=2*LOW_UM;default:gear_speed<=3*LOW_UM;endcase
 wire [31:0] auto_speed=cruise && route_cap>LOW_UM ? gear_speed:LOW_UM;
 obstacle_command_buffer buffer(
 .clk(clk),.rst_n(rst_n),.cancel(!auto_run || route_cancel || route_stop || fault),
 .request(route_start && auto_run && !stopping),.motion_ready(cmd_ready),.motion_busy(busy || issue_valid),
 .proposal({(route_rotate ? 4'd6:(route_end==2 ? 4'd7:4'd2)),route_end,route_reverse,
 (cruise ? gear_speed:route_speed),(cruise ? gear_speed:route_speed),route_goal,
 (route_end==0 ? 32'd0:32'd10000),(3*LOW_UM),(LOW_UM/5),(LOW_UM/2)}),
 .context_in({route_state,yaw,route_heading,um_l,um_r}),.ready(buffer_ready),.submit(command_submit),
 .busy(buffer_busy),.pending(command_pending),.command(payload),.context_out(context_out));
 wire manual_live=manual_active && !rotate;
 localparam [31:0] MANUAL_TIMEOUT=ACTION_MS,GEAR_LIMIT=3*LOW_UM,
 ACCEL_STEP=LOW_UM/5,DECEL_STEP=LOW_UM/2;
 wire [31:0] manual_limit=GEAR_LIMIT;
 // Capture the entire selected transaction before controller validation.
 // Stop/fault still reach the controller and bridge directly and cancel a
 // captured transaction before it can execute; no cancelled command is replayed.
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin issue_valid<=0;issue_payload<=0;end
 else begin
 issue_valid<=!stopping && !fault && (start || command_submit);
 if(start || command_submit)issue_payload<=command_submit ? payload:
 {requested_mode,requested_end,reverse,requested_l,requested_r,goal,
 MANUAL_TIMEOUT,manual_limit,ACCEL_STEP,DECEL_STEP};
 end
 motion_control_and_mode_fsm #(.CLK_HZ(CLK_HZ),.STILL_THRESHOLD(0),.ALLOW_SINGLE_TIMED(1),.STOP_TIMEOUT_MS(3000)) motion(
 .clk(clk),.rst_n(rst_n),.enable(modules_ready),.stop_request(stopping || (auto_run && route_stop)),
 .external_fault(fault_now || fault_latched),.clear_fault(1'b0),.cmd_valid(issue_valid),.cmd_ready(cmd_ready),
 .runtime_speed_cap(auto_run ? auto_speed:32'h7fffffff),
 .runtime_speed_valid((auto_run && cruise) || manual_live),.runtime_speed(auto_run ? auto_speed:gear_speed),
 .cmd_mode(issue_payload[230:227]),
 .cmd_end(issue_payload[226:225]),.cmd_reverse(issue_payload[224]),
 .cmd_speed_l(issue_payload[223:192]),.cmd_speed_r(issue_payload[191:160]),
 .cmd_goal(issue_payload[159:128]),.cmd_timeout_ms(issue_payload[127:96]),
 .cmd_speed_limit(issue_payload[95:64]),
 .cmd_accel_step(issue_payload[63:32]),.cmd_decel_step(issue_payload[31:0]),
 .position_l(um_l),.position_r(um_r),.speed_l(raw_l),.speed_r(raw_r),.yaw_mdeg(yaw),
 .speed_valid_l(valid_l),.speed_valid_r(valid_r),.yaw_valid(yaw_valid),
 .target_l(target_um_l),.target_r(target_um_r),.drive_enable(drive_enable),.controller_clear(controller_clear),
 .busy(busy),.mode(),.phase(phase),.done(done),.aborted(aborted),.rejected(rejected),.reason(reason),.fault_code(motion_fault),.accepted(),.command_errors(),.feedback_ages());
 // Constant reciprocal converts physical speed to signed encoder counts/window.
 localparam [31:0] RECIP_L=(64'd16777216*COUNTS_L+UM_PER_REV_L/2)/UM_PER_REV_L;
 localparam [31:0] RECIP_R=(64'd16777216*COUNTS_R+UM_PER_REV_R/2)/UM_PER_REV_R;
 function signed [15:0] convert_speed;
 input signed [31:0] value;
 input [31:0] reciprocal;
 reg signed [63:0] product;
 reg [31:0] magnitude;
 begin
 magnitude=value<0 ? -value:value;
 product=({32'd0,magnitude}*{32'd0,reciprocal}+64'd8388608)>>>24;
 if(product>32767)product=32767;
 convert_speed=value<0 ? -$signed(product[15:0]):$signed(product[15:0]);
 end
 endfunction
 // Pipeline physical-unit targets before reciprocal conversion. This keeps
 // fault/enable qualification out of the multiply path; the bridge enable is
 // still gated directly by stop/fault in the same cycle.
 reg signed [31:0] target_um_q_l,target_um_q_r;
 reg signed [15:0] target_l,target_r;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin target_um_q_l<=0;target_um_q_r<=0;target_l<=0;target_r<=0;end
 else begin
 target_um_q_l<=target_um_l;target_um_q_r<=target_um_r;
 target_l<=convert_speed(target_um_q_l,RECIP_L);target_r<=convert_speed(target_um_q_r,RECIP_R);
 end
 function signed [15:0] sat16;
 input signed [31:0] value;
 begin sat16=value>32767 ? 16'sh7fff:value< -32768 ? 16'sh8000:value[15:0];end
 endfunction
 wire control_enable=drive_enable && !controller_clear && !fault && !stopping;
 wire signed [15:0] control_l,control_r,ramp_l,ramp_r;
 localparam [15:0] PWM_LIMIT=CLK_HZ/PWM_HZ/4;
 dual_speed_pi_pid_controller pi(.clk(clk),.rst_n(rst_n),.control_tick(valid_l && valid_r),.enable(control_enable),
 .target_l(target_l),.target_r(target_r),.feedback_l(sat16(speed_l)),.feedback_r(sat16(speed_r)),
 .kp_l(KP),.ki_l(KI),.kd_l(KD),.kp_r(KP),.ki_r(KI),.kd_r(KD),
 .gain_shift_l(5'd0),.gain_shift_r(5'd0),.output_limit_l(PWM_LIMIT),.output_limit_r(PWM_LIMIT),
 .control_l(control_l),.control_r(control_r),.output_valid(),.sat_l(),.sat_r(),.active_l(),.active_r());
 pwm_and_startup_ramp #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) ramp(
 .clk(clk),.rst_n(rst_n),.enable(control_enable),.pwm_cmd_l(control_l),.pwm_cmd_r(control_r),
 .pwm_limit(PWM_LIMIT),.pwm_min(16'd0),.ramp_step(16'd1),.pwm_out_l(),.pwm_out_r(),.pwm_actual_l(ramp_l),.pwm_actual_r(ramp_r));
 tb6612_driver_interface #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),.DIR_DEADTIME_CYCLES(DIR_DEADTIME)) driver(
 .clk(clk),.rst_n(rst_n && reset_n),.enable(control_enable),.emergency_stop(stop_gate),.fault(fault),
 .watchdog_timeout(1'b0),.pwm_signed_l(ramp_l),.pwm_signed_r(ramp_r),
 .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY));
 reg [7:0] next_status;
 always @* begin
 next_status=status;
 if(done && !auto_run)next_status=8'h11;
 if(aborted && reason==2)next_status=8'h04;
 if((stopping) && !busy)next_status=8'h04;
 if(start)next_status=8'h10;
 if(avoid_mode && !manual_active)next_status=mode_status;
 if(!avoid_mode && status>=8'h20)next_status=8'h04;
 if(fault)next_status=8'h06;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)status<=0;else status<=next_status;
 // Register complete protocol events before the FIFO admission arithmetic.
 // This removes the fault/ready/arbitration chain from the RAM write path.
 reg pair_q,event_q,aux_q;
 reg [7:0] response_q,status_q,event_status_q,aux_type_q,aux_code_q;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin pair_q<=0;event_q<=0;aux_q<=0;response_q<=0;status_q<=0;event_status_q<=0;aux_type_q<=0;aux_code_q<=0;end
 else begin
 pair_q<=voice_command_valid;response_q<=response;status_q<=next_status;
 event_q<=next_status!=status && !voice_command_valid;event_status_q<=next_status;
 aux_q<=aux_valid;aux_type_q<=aux_type;aux_code_q<=aux_code;
 end
 asr_packet_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) packets(
 .clk(clk),.rst_n(rst_n),.pair_valid(pair_q),.response(response_q),.pair_status(status_q),
 .event_valid(event_q),.event_status(event_status_q),.aux_valid(aux_q),.aux_type(aux_type_q),.aux_code(aux_code_q),.tx(uart_tx),.overflow(tx_overflow));
 assign event_errors={fault,enc_fault_r,enc_fault_l,1'b0,
 (imu_error!=0 && imu_error!=4 && imu_error!=5 && imu_error!=6 && imu_error!=7),
 (imu_ready && sample_age>=IMU_TIMEOUT-1),(ready_seen && !imu_ready),
 (sensor_error!=0),(imu_error==7),(imu_error==6),(imu_error==5),(imu_error==4)};
 assign led_ready=modules_ready;
 assign led_run=busy;
 assign led_fault=fault;
endmodule
