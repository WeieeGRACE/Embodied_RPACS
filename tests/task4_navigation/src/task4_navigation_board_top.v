`timescale 1ns/1ps
// Board adapter for independent task-4 sensor/actuator verification.
// Route 6 is the only selectable baseline here. ENABLE_SINGLE_LEG defaults 0;
// route motion remains inhibited until dynamic safety measurements are signed off.
module task4_navigation_board_top #(
    parameter integer CLK_HZ=50000000,BAUD=115200,WINDOW_CYCLES=CLK_HZ/100,
    parameter integer PWM_HZ=20000,ENC_FILTER=50,DIR_DEADTIME=2500,
    parameter integer COUNTS_L=1456,COUNTS_R=1456,
    parameter integer UM_PER_REV_L=207345,UM_PER_REV_R=207345,
    parameter integer YAW_SIGN=-1,KEY_CYCLES=CLK_HZ/50,
    parameter integer TOF_RESET=CLK_HZ/1000,TOF_BOOT=CLK_HZ/50,
    parameter integer TOF_POLL=CLK_HZ/100,TOF_MAX_POLLS=10,TOF_TIMEOUT=CLK_HZ/100,
    parameter integer IMU_POWERUP=CLK_HZ/50,IMU_WAIT=CLK_HZ/50,IMU_RETRY=CLK_HZ,
    parameter integer IMU_SAMPLE=CLK_HZ/200,IMU_SPI_HALF=25,
    parameter integer CALIBRATION_SAMPLES=200,
    parameter integer ENABLE_SINGLE_LEG=0,
    parameter integer DIAGNOSTIC_ONLY=0,
    parameter [31:0] BUILD_ID=32'h26102702,
    parameter CONFIG_HEX_FILE="src/imu660ra_config.hex",
    parameter signed [15:0] KP=32,KI=1,KD=0
) (
    input wire clk_50m,reset_n,key_enable_n,key_estop_n,
    inout wire tof_front_scl,tof_front_sda,tof_front_xshut_n,
    inout wire tof_left_scl,tof_left_sda,tof_left_xshut_n,
    inout wire tof_right_scl,tof_right_sda,tof_right_xshut_n,
    output wire oled_scl,inout wire oled_sda,
    input wire encoder_a_l,encoder_b_l,encoder_a_r,encoder_b_r,
    input wire imu_spi_miso,
    output wire imu_spi_sck,imu_spi_mosi,imu_cs_n,
    output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx,
    output wire led_ready,led_run,led_fault,led_heartbeat
);
    (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe=0;
    always @(posedge clk_50m or negedge reset_n)
        if(!reset_n) reset_pipe<=0; else reset_pipe<={reset_pipe[0],1'b1};
    wire rst_n=reset_pipe[1];

    wire signed [15:0] ax,ay,az,gx,gy,gz;
    wire sample_valid,imu_ready;
    wire [7:0] imu_error,sensor_error;
    imu660ra_driver #(.CLK_HZ(CLK_HZ),.SPI_HALF_PERIOD(IMU_SPI_HALF),
      .POWERUP_CYCLES(IMU_POWERUP),.CONFIG_WAIT_CYCLES(IMU_WAIT),.RETRY_CYCLES(IMU_RETRY),
      .ID_RETRY_CYCLES(CLK_HZ/1000),.SAMPLE_CYCLES(IMU_SAMPLE),
      .HEALTH_CHECK_CYCLES(CLK_HZ),.CONFIG_HEX_FILE(CONFIG_HEX_FILE)) imu(
      .clk(clk_50m),.rst_n(rst_n),.spi_miso(imu_spi_miso),.imu_int1(1'b0),.imu_int2(1'b0),
      .spi_sclk(imu_spi_sck),.spi_mosi(imu_spi_mosi),.spi_cs_n(imu_cs_n),
      .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
      .sample_valid(sample_valid),.sample_sequence(),.chip_id(),.init_status(),
      .power_conf_readback(),.init_ctrl_readback(),.sensor_error(sensor_error),
      .ready(imu_ready),.error_code(imu_error),.int1_sync(),.int2_sync());
    wire calibrating,attitude_valid,attitude_update;
    wire signed [31:0] yaw_wrapped,yaw_unwrapped;
    wire yaw_valid;
    wire signed [15:0] gyro_bias_x,gyro_bias_y,gyro_bias_z;
    imu_attitude_fixed #(.CALIBRATION_SAMPLES(CALIBRATION_SAMPLES)) attitude(
      .clk(clk_50m),.rst_n(rst_n),.ready_in(imu_ready),.sample_valid(sample_valid),
      .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
      .calibrating(calibrating),.attitude_valid(attitude_valid),.update_valid(attitude_update),
      .roll_mdeg(),.pitch_mdeg(),.yaw_mdeg(yaw_wrapped),
      .gyro_bias_x(gyro_bias_x),.gyro_bias_y(gyro_bias_y),.gyro_bias_z(gyro_bias_z));
    asr_yaw_unwrap #(.YAW_SIGN(YAW_SIGN)) yaw_unwrap(
      .clk(clk_50m),.rst_n(rst_n),.update_valid(attitude_update),.wrapped(yaw_wrapped),
      .unwrapped(yaw_unwrapped),.valid(yaw_valid));

    wire signed [31:0] pos_l,pos_r,um_l,um_r,speed_count_l,speed_count_r;
    wire signed [31:0] raw_l,raw_r;
    wire signed [1:0] delta_l,delta_r;
    wire valid_l,valid_r,enc_fault_l,enc_fault_r;
    dual_ab_quadrature_encoder #(.FILTER_CYCLES(ENC_FILTER),.INVERT_L(1),.INVERT_R(0)) enc(
      .clk(clk_50m),.rst_n(rst_n),.encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),
      .encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),.clear_fault_l(1'b0),.clear_fault_r(1'b0),
      .position_l(pos_l),.position_r(pos_r),.delta_count_l(delta_l),.delta_count_r(delta_r),
      .direction_l(),.direction_r(),.illegal_transition_l(),.illegal_transition_r(),
      .encoder_fault_l(enc_fault_l),.encoder_fault_r(enc_fault_r));
    asr_distance #(.UM_PER_REV(UM_PER_REV_L),.COUNTS_PER_REV(COUNTS_L)) distance_l(
      .clk(clk_50m),.rst_n(rst_n),.delta(delta_l),.position_um(um_l));
    asr_distance #(.UM_PER_REV(UM_PER_REV_R),.COUNTS_PER_REV(COUNTS_R)) distance_r(
      .clk(clk_50m),.rst_n(rst_n),.delta(delta_r),.position_um(um_r));
    speed_measurement_and_velocity_filter #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES),
      .FILTER_LENGTH(4),.ZERO_PAD_WARMUP(1)) speed_measure_l(
      .clk(clk_50m),.rst_n(rst_n),.delta_count({{14{delta_l[1]}},delta_l}),
      .speed_raw(raw_l),.speed_filtered(speed_count_l),.position(),.speed_valid(valid_l),.direction(),.zero_speed());
    speed_measurement_and_velocity_filter #(.CLK_HZ(CLK_HZ),.WINDOW_CYCLES(WINDOW_CYCLES),
      .FILTER_LENGTH(4),.ZERO_PAD_WARMUP(1)) speed_measure_r(
      .clk(clk_50m),.rst_n(rst_n),.delta_count({{14{delta_r[1]}},delta_r}),
      .speed_raw(raw_r),.speed_filtered(speed_count_r),.position(),.speed_valid(valid_r),.direction(),.zero_speed());

    // Navigation only needs a settled/not-settled indication. A filtered
    // encoder count represents motion over the 10 ms speed window; with this
    // encoder scale one count exceeds the 5 mm/s stop threshold, so zero on
    // both channels is conservative and avoids a wide speed conversion here.
    reg [31:0] nav_div;
    reg nav_tick;
    reg signed [31:0] last_um_l,last_um_r,last_imu_yaw;
    reg signed [23:0] encoder_left_um,encoder_right_um,imu_yaw_delta;
    function signed [23:0] clamp24;
        input signed [63:0] value;
        begin
            if(value>64'sd8388607) clamp24=24'sh7fffff;
            else if(value< -64'sd8388608) clamp24=-24'sd8388608;
            else clamp24=value[23:0];
        end
    endfunction
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            nav_div<=0;nav_tick<=0;last_um_l<=0;last_um_r<=0;last_imu_yaw<=0;
            encoder_left_um<=0;encoder_right_um<=0;imu_yaw_delta<=0;
        end else if(nav_div>=CLK_HZ/100-1) begin
            nav_div<=0;nav_tick<=1;
            encoder_left_um<=clamp24($signed(um_l)-$signed(last_um_l));
            encoder_right_um<=clamp24($signed(um_r)-$signed(last_um_r));
            imu_yaw_delta<=clamp24($signed(yaw_unwrapped)-$signed(last_imu_yaw));
            last_um_l<=um_l;last_um_r<=um_r;last_imu_yaw<=yaw_unwrapped;
        end else begin nav_div<=nav_div+1'b1;nav_tick<=0;end
    end

    wire press;
    obstacle_key #(.DEBOUNCE_CYCLES(KEY_CYCLES)) key_filter(
      .clk(clk_50m),.rst_n(rst_n),.key_n(key_enable_n),.press(press));
    wire [47:0] distances,ages,tof_raw,comm_ages,confirm_ages;
    wire [2:0] tof_seen,tof_valid,post_turn,pending,raw_events,sample_mask,reused_distance;
    wire [23:0] tof_errors;
    wire [11:0] tof_status;
    wire [14:0] sensor_causes;
    wire sensor_fault;
    wire [4:0] nav_state;
    obstacle_sensors #(.CLK_HZ(CLK_HZ),.RESET_CYCLES(TOF_RESET),.BOOT_CYCLES(TOF_BOOT),
      .POLL_CYCLES(TOF_POLL),.MAX_POLLS(TOF_MAX_POLLS),.TIMEOUT_CYCLES(TOF_TIMEOUT),
      // This is a sensor-validity floor only. Route-specific collision
      // protection uses the separate forward_stop_mm table entry.
      .NEAR_MM(25),.SIDE_NEAR_MM(25)) sensors(
      .clk(clk_50m),.rst_n(rst_n),.refresh(1'b0),.monitor_enable(nav_state!=0 && nav_state!=5),
      .front_near_mm(16'd85),.gpio1(3'b000),
      .scl({tof_right_scl,tof_left_scl,tof_front_scl}),.sda({tof_right_sda,tof_left_sda,tof_front_sda}),
      .xshut_n({tof_right_xshut_n,tof_left_xshut_n,tof_front_xshut_n}),
      .distances(distances),.ages(ages),.seen(tof_seen),.valid(tof_valid),.post_turn(post_turn),
      .raw_distances(tof_raw),.comm_ages(comm_ages),.confirm_ages(confirm_ages),.pending(pending),
      .reused(reused_distance),.errors(tof_errors),.statuses(tof_status),.event_mask(sample_mask),
      .raw_event_mask(raw_events),.fault_bits(sensor_causes),.fault(sensor_fault));

    wire [2:0] tof_nav_valid=tof_valid & ~reused_distance &
      { (ages[47:32]<16'd250 && comm_ages[47:32]<16'd250),
        (ages[31:16]<16'd250 && comm_ages[31:16]<16'd250),
        (ages[15:0]<16'd250 && comm_ages[15:0]<16'd250) };
    reg estop_latched,ready_seen,yaw_seen,speed_seen;
    reg [31:0] sample_age,yaw_age,speed_age,ms_div;
    reg ms_tick;
    reg speed_recent;
    wire wheels_stationary=speed_recent && speed_count_l==0 && speed_count_r==0;
    wire estop_event=!key_estop_n;
    always @(posedge clk_50m or posedge estop_event) begin
        if(estop_event) estop_latched<=1;
        else if(!rst_n) estop_latched<=0;
    end
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            ready_seen<=0;yaw_seen<=0;speed_seen<=0;sample_age<=0;yaw_age<=0;
            speed_age<=0;speed_recent<=0;ms_div<=0;ms_tick<=0;
        end
        else begin
            if(ms_div>=CLK_HZ/1000-1) begin ms_div<=0;ms_tick<=1;end
            else begin ms_div<=ms_div+1'b1;ms_tick<=0;end
            if(imu_ready) ready_seen<=1;
            if(sample_valid) sample_age<=0;else if(ms_tick && sample_age<32'hffffffff) sample_age<=sample_age+1'b1;
            if(yaw_valid) begin yaw_seen<=1;yaw_age<=0;end
            else if(ms_tick && yaw_age<32'hffffffff) yaw_age<=yaw_age+1'b1;
            if(valid_l && valid_r) begin speed_seen<=1;speed_age<=0;speed_recent<=1;end
            else if(ms_tick && speed_age<32'hffffffff) begin
                speed_age<=speed_age+1'b1;
                if(speed_age>=100) speed_recent<=0;
            end
        end
    end
    wire imu_bad=imu_error!=0 || sensor_error!=0 ||
                 (ready_seen && sample_age>=100) || (yaw_seen && yaw_age>=100);
    wire imu_signal_valid=attitude_valid && yaw_seen && !imu_bad;
    wire physical_fault=sensor_fault || enc_fault_l || enc_fault_r ||
                        imu_bad || estop_latched || !key_estop_n;
    wire [3:0] segment_id;
    wire [7:0] nav_fault_code,wall_event_id;
    wire [2:0] accepted_tof,tof_fresh;
    wire [15:0] tof_age_front,tof_age_left,tof_age_right;
    wire record_valid;
    wire [3:0] record_segment_id;
    wire [31:0] record_distance_mm;
    wire signed [31:0] record_start_yaw_mdeg,record_end_yaw_mdeg;
    wire [7:0] record_wall_event_id,record_fault_code;
    wire nav_done,nav_blocked,nav_range_fault,nav_input_fault,nav_drive_enable;
    wire signed [31:0] nav_velocity_l_mm_s,nav_velocity_r_mm_s;
    wire nav_pose_fault,nav_odometry_fault;
    wire nav_estop_n=key_estop_n && !estop_latched;
    wire nav_start=press && nav_state==0 && ENABLE_SINGLE_LEG!=0 && DIAGNOSTIC_ONLY==0;
    wire nav_stop=press && (nav_state==1 || nav_state==2 || nav_state==3 || nav_state==4);
    wire calibration_done=imu_ready && imu_signal_valid && speed_seen &&
                          (&tof_seen) && (&tof_nav_valid) && !calibrating && !physical_fault;
    task4_event_navigation_top #(.CLK_HZ(CLK_HZ)) nav(
      .clk_50m(clk_50m),.reset_n(rst_n),.key_start_n(!nav_start),.key_stop_n(!nav_stop),
      .key_estop_n(nav_estop_n),.calibration_done(calibration_done),
      .geometry_ok(ENABLE_SINGLE_LEG!=0),
      .range_stale_external(sensor_fault || ages[15:0]>=16'd250 || comm_ages[15:0]>=16'd250),
      .imu_valid(imu_signal_valid),
      .encoder_valid(speed_seen && !enc_fault_l && !enc_fault_r),
      .encoder_left_um(encoder_left_um),.encoder_right_um(encoder_right_um),
      .imu_yaw_delta_mdeg(imu_yaw_delta),.wheels_stationary(wheels_stationary),
      .tof_valid(tof_nav_valid),.tof_update(sample_mask),
      .tof_front_mm(distances[15:0]),.tof_left_mm(distances[31:16]),.tof_right_mm(distances[47:32]),
      .route_id(3'd6),.pose_x_um(),.pose_y_um(),.pose_yaw_mdeg(),
      .velocity_l_mm_s(nav_velocity_l_mm_s),.velocity_r_mm_s(nav_velocity_r_mm_s),
      .pose_fault_out(nav_pose_fault),.odometry_fault_out(nav_odometry_fault),.nav_state(nav_state),.segment_id(segment_id),.fault_code(nav_fault_code),
      .wall_event_id(wall_event_id),.accepted_tof(accepted_tof),.tof_fresh(tof_fresh),
      .tof_age_front(tof_age_front),.tof_age_left(tof_age_left),.tof_age_right(tof_age_right),
      .record_valid(record_valid),.record_segment_id(record_segment_id),
      .record_distance_mm(record_distance_mm),.record_start_yaw_mdeg(record_start_yaw_mdeg),
      .record_end_yaw_mdeg(record_end_yaw_mdeg),.record_wall_event_id(record_wall_event_id),
      .record_fault_code(record_fault_code),.nav_done(nav_done),.nav_blocked(nav_blocked),
      .nav_range_fault(nav_range_fault),.nav_input_fault(nav_input_fault),.drive_enable(nav_drive_enable));

    // A 10 ms speed window turns mm/s into micrometres by multiplying by 10.
    // The encoder-count scale is about 0.07024 count per mm/s for the current
    // wheel calibration. 9/128 (0.0703125) is within 0.11% and avoids a wide
    // three-factor reciprocal multiply on the 50 MHz motor-control path.
    function signed [15:0] convert_mm_s;
        input signed [31:0] value;
        reg signed [35:0] extended_value;
        reg signed [35:0] scaled_numerator;
        reg signed [35:0] scaled_value;
        begin
            extended_value={{4{value[31]}},value};
            scaled_numerator=(extended_value <<< 3)+extended_value;
            // Truncate toward zero so integer encoder-count quantization can
            // never raise the requested speed above its configured ceiling.
            if(scaled_numerator<0)
                scaled_value=(scaled_numerator+36'sd127)>>>7;
            else scaled_value=scaled_numerator>>>7;
            if(scaled_value>36'sd32767) scaled_value=36'sd32767;
            else if(scaled_value< -36'sd32768) scaled_value=-36'sd32768;
            convert_mm_s=scaled_value[15:0];
        end
    endfunction
    wire signed [15:0] target_l=convert_mm_s(nav_velocity_l_mm_s);
    wire signed [15:0] target_r=convert_mm_s(nav_velocity_r_mm_s);
    wire control_enable=nav_drive_enable && !physical_fault && ENABLE_SINGLE_LEG!=0 && DIAGNOSTIC_ONLY==0;
    wire signed [15:0] control_l,control_r,ramp_l,ramp_r;
    localparam [15:0] PWM_LIMIT=CLK_HZ/PWM_HZ/4;
    dual_speed_pi_pid_controller pi(.clk(clk_50m),.rst_n(rst_n),.control_tick(valid_l && valid_r),
      .enable(control_enable),.target_l(target_l),.feedback_l(speed_count_l[15:0]),
      .target_r(target_r),.feedback_r(speed_count_r[15:0]),.kp_l(KP),.ki_l(KI),.kd_l(KD),
      .kp_r(KP),.ki_r(KI),.kd_r(KD),.gain_shift_l(5'd0),.gain_shift_r(5'd0),
      .output_limit_l(PWM_LIMIT),.output_limit_r(PWM_LIMIT),.control_l(control_l),.control_r(control_r),
      .output_valid(),.sat_l(),.sat_r(),.active_l(),.active_r());
    pwm_and_startup_ramp #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ)) ramp(
      .clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.pwm_cmd_l(control_l),.pwm_cmd_r(control_r),
      .pwm_limit(PWM_LIMIT),.pwm_min(16'd0),.ramp_step(16'd1),
      .pwm_out_l(),.pwm_out_r(),.pwm_actual_l(ramp_l),.pwm_actual_r(ramp_r));
    tb6612_driver_interface #(.CLK_HZ(CLK_HZ),.PWM_HZ(PWM_HZ),.DIR_DEADTIME_CYCLES(DIR_DEADTIME)) driver(
      .clk(clk_50m),.rst_n(rst_n),.enable(control_enable),.emergency_stop(!nav_estop_n),
      .fault(physical_fault),.watchdog_timeout(nav_range_fault || nav_input_fault),
      .pwm_signed_l(ramp_l),.pwm_signed_r(ramp_r),.PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),
      .PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY));

    wire [7:0] first_fault_code=physical_fault ? 8'h01:nav_fault_code;
    wire [671:0] oled_text,normal_oled_text;
    wire frame_done;
    reg [31:0] frame_count;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) frame_count<=0; else if(frame_done) frame_count<=frame_count+1'b1;
    obstacle_page oled_page(.state(nav_state),.reason(first_fault_code),
      .fault(physical_fault || nav_fault_code!=0),.distances(distances),.valid(tof_valid),
      .reused(reused_distance),.text(normal_oled_text));
    tof_oled_text #(.CLK_HZ(CLK_HZ)) oled(.clk(clk_50m),.rst_n(rst_n),.text(oled_text),
      .frame_done(frame_done),.scl(oled_scl),.sda(oled_sda));
    generate if(DIAGNOSTIC_ONLY!=0) begin: diagnostics
        nav_board_diagnostics #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.BUILD_ID(BUILD_ID)) diag(
          .clk(clk_50m),.rst_n(rst_n),.key_enable_n(key_enable_n),.key_estop_n(key_estop_n),
          .estop_latched(estop_latched),.frame_count(frame_count),.raw_mm(tof_raw),
          .accepted_mm(distances),.ages(ages),.errors(tof_errors),.statuses(tof_status),
          .valid(tof_nav_valid),.seen(tof_seen),.nav_fault(nav_fault_code),
          .pose_fault(nav_pose_fault),.odometry_fault(nav_odometry_fault),
          .imu_valid(imu_signal_valid),.encoder_valid(speed_seen),
          .oled_text(oled_text),.tx(uart_tx));
    end else begin: normal_logging
        assign oled_text=normal_oled_text;
        nav_segment_uart_logger #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) log_segments(
          .clk(clk_50m),.rst_n(rst_n),.record_valid(record_valid),.segment_id(record_segment_id),
          .distance_mm(record_distance_mm),.yaw_mdeg(record_end_yaw_mdeg),.wall_id(record_wall_event_id),
          .fault_code(record_fault_code),.tx(uart_tx),.dropped_records());
    end endgenerate
    assign led_ready=calibration_done && !physical_fault;
    assign led_run=nav_state==1 || nav_state==3;
    assign led_fault=physical_fault || nav_fault_code!=0 || nav_blocked || nav_range_fault || nav_input_fault;
    assign led_heartbeat=nav_done;
endmodule
