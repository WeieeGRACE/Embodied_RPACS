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
 output wire [7:0] response,output wire [11:0] event_errors
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
 wire signed [31:0] um_l,um_r,raw_l,raw_r;
 wire signed [1:0] delta_l,delta_r;
 wire valid_l,valid_r,enc_fault_l,enc_fault_r;
 wire [31:0] illegal_l,illegal_r;
 dual_ab_quadrature_encoder #(.FILTER_CYCLES(ENC_FILTER),.INVERT_L(1),.INVERT_R(0)) enc(
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
 wire fault_now=imu_bad || enc_fault_l || enc_fault_r || tx_overflow || motion_fault!=0;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)fault_latched<=0;else if(fault_now)fault_latched<=1;
 wire fault=fault_now || fault_latched;
 wire modules_ready=imu_ready && attitude_valid && yaw_seen && speed_seen && !calibrating && !fault;
 asr_uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(
 .clk(clk),.rst_n(rst_n),.rx(uart_rx),.command_valid(command_valid),.command(command));
 wire cv=command_valid;
 wire voice_allowed=voice_enable || command==8'h02 || command==8'h03;
 wire voice_stop=cv && command==8'h02;
 wire menu_stop=menu_valid && menu_command==8'h02;
 wire stopping=stop_gate || stop_request || voice_stop || menu_stop;
 wire busy,cmd_ready,done,aborted,rejected,drive_enable,controller_clear;
 reg stop_hold;
 always @(posedge clk_50m or posedge stop_gate)
 if(stop_gate)stop_hold<=1;
 else if(!rst_n)stop_hold<=0;
 else if(stopping)stop_hold<=1;
 else if(!busy)stop_hold<=0;
 wire menu_move=menu_valid && menu_command!=8'h02;
 wire [7:0] action=menu_move ? menu_command:command;
 wire move=command>=8'h10 && command<=8'h13;
 wire requested=menu_move || (cv && voice_enable && move);
 wire debug_action=action>=8'h20 && action<=8'h27;
 wire rotate=action==8'h12 || action==8'h13;
 wire reverse=action==8'h11 || action==8'h13;
 wire [2:0] phase;
 wire [3:0] reason;
 wire signed [31:0] target_um_l,target_um_r;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)gear<=1;
 else if(gear_commit)gear<=gear_value;
 else if(cv && voice_enable && !fault && !stopping)begin
 if(command==8'h14 && gear<2)gear<=gear+1'b1;
 if(command==8'h15 && gear>0)gear<=gear-1'b1;
 end
 localparam integer LOW_UM=(64'd50000*WINDOW_CYCLES)/CLK_HZ;
 // The old bench targets are counts/window: 10 for the outer wheel,
 // 5 for the inner wheel. Round upwards to preserve integer conversion.
 localparam integer DEBUG_L=(64'd10*UM_PER_REV_L+COUNTS_L-1)/COUNTS_L;
 localparam integer DEBUG_R=(64'd10*UM_PER_REV_R+COUNTS_R-1)/COUNTS_R;
 reg [31:0] requested_l,requested_r,goal;
 reg [3:0] requested_mode;
 reg [1:0] requested_end;
 always @* begin
 requested_l=rotate ? LOW_UM:(gear+32'd1)*LOW_UM;
 requested_r=requested_l;
 requested_mode=rotate ? 6:7;requested_end=rotate ? 3:2;
 goal=rotate ? GOAL_MDEG:GOAL_UM;
 if(debug_action)begin
 requested_end=1;goal=8000;
 requested_l=DEBUG_L;requested_r=DEBUG_R;
 requested_mode=(action==8'h21 || action==8'h23 || action==8'h25) ? 3:2;
 case(action)
 8'h20,8'h21:requested_r=0;
 8'h22,8'h23:requested_l=0;
 8'h26:begin requested_mode=4;requested_l=DEBUG_L/2;end
 8'h27:begin requested_mode=5;requested_r=DEBUG_R/2;end
 default:begin end
 endcase
 end
 end
 wire start=requested && modules_ready && cmd_ready && !stopping && !stop_hold;
 motion_control_and_mode_fsm #(.CLK_HZ(CLK_HZ),.STILL_THRESHOLD(0),.ALLOW_SINGLE_TIMED(1)) motion(
 .clk(clk),.rst_n(rst_n),.enable(modules_ready),.stop_request(stopping || stop_hold),
 .external_fault(fault_now || fault_latched),.clear_fault(1'b0),.cmd_valid(start),.cmd_ready(cmd_ready),
 .cmd_mode(requested_mode),.cmd_end(requested_end),.cmd_reverse(reverse),
 .cmd_speed_l(requested_l),.cmd_speed_r(requested_r),.cmd_goal(goal),
 .cmd_timeout_ms(ACTION_MS),.cmd_speed_limit(debug_action ? ((DEBUG_L>DEBUG_R)?DEBUG_L:DEBUG_R):3*LOW_UM),.cmd_accel_step(LOW_UM/5),.cmd_decel_step(LOW_UM/2),
 .position_l(um_l),.position_r(um_r),.speed_l(raw_l),.speed_r(raw_r),.yaw_mdeg(yaw),
 .speed_valid_l(valid_l),.speed_valid_r(valid_r),.yaw_valid(yaw_valid),
 .target_l(target_um_l),.target_r(target_um_r),.drive_enable(drive_enable),.controller_clear(controller_clear),
 .busy(busy),.mode(),.phase(phase),.done(done),.aborted(aborted),.rejected(rejected),.reason(reason),.fault_code(motion_fault));
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
 // A register separates physical-unit conversion and fault gating from PI arithmetic.
 reg signed [15:0] target_l,target_r;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin target_l<=0;target_r<=0;end
 else begin target_l<=convert_speed(target_um_l,RECIP_L);target_r<=convert_speed(target_um_r,RECIP_R);end
 function signed [15:0] sat16;
 input signed [31:0] value;
 begin sat16=value>32767 ? 16'sh7fff:value< -32768 ? 16'sh8000:value[15:0];end
 endfunction
 wire control_enable=drive_enable && !controller_clear && !fault && !stopping && !stop_hold;
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
 assign response=(!voice_allowed || command==8'h01 ||
 ((move || command==8'h14 || command==8'h15) && fault) ||
 (move && !modules_ready)) ? 8'h02:
 (move && (busy || !cmd_ready || menu_move || stopping || stop_hold)) ? 8'h01:8'h00;
 always @* begin
 next_status=status;
 if(done)next_status=8'h11;
 if(aborted && reason==2)next_status=8'h04;
 if((stopping || stop_hold) && !busy)next_status=8'h04;
 if(start)next_status=8'h10;
 if(fault)next_status=8'h06;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)status<=0;else status<=next_status;
 asr_packet_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) packets(
 .clk(clk),.rst_n(rst_n),.pair_valid(cv),.response(response),.pair_status(next_status),
 .event_valid(next_status!=status && !cv),.event_status(next_status),.tx(uart_tx),.overflow(tx_overflow));
 assign event_errors={fault,enc_fault_r,enc_fault_l,1'b0,
 (imu_error!=0 && imu_error!=4 && imu_error!=5 && imu_error!=6 && imu_error!=7),
 (imu_ready && sample_age>=IMU_TIMEOUT-1),(ready_seen && !imu_ready),
 (sensor_error!=0),(imu_error==7),(imu_error==6),(imu_error==5),(imu_error==4)};
 assign led_ready=modules_ready;
 assign led_run=busy;
 assign led_fault=fault;
endmodule
