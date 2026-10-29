`timescale 1ns/1ps
// Main project: one motion owner; UI navigation never resets sensor consumers.
module robot_system_top #(
 parameter integer CLK_HZ=50000000,UART_BAUD=115200,PWM_HZ=20000,
 WINDOW_CYCLES=CLK_HZ/100,DEBOUNCE_CYCLES=CLK_HZ/100,
 POR_CYCLES=1024,ENC_FILTER=50,DIR_DEADTIME=2500,
 IMU_POWERUP_CYCLES=CLK_HZ/50,IMU_CONFIG_WAIT_CYCLES=CLK_HZ/50,
 IMU_RETRY_CYCLES=CLK_HZ,IMU_SAMPLE_CYCLES=CLK_HZ/200,
 IMU_SPI_HALF_PERIOD=25,IMU_HEALTH_CYCLES=CLK_HZ,
 IMU_TIMEOUT_CYCLES=CLK_HZ/10,IMU_CALIBRATION_SAMPLES=200,
 OLED_I2C_HZ=250000,OLED_POWERUP_CYCLES=CLK_HZ/10,
 OLED_FRAME_WAIT_CYCLES=CLK_HZ*7/50,
 TOF_I2C_HZ=100000,TOF_RESET_CYCLES=CLK_HZ/1000,
 TOF_BOOT_CYCLES=CLK_HZ/50,TOF_POLL_CYCLES=CLK_HZ/100,
 TOF_MAX_POLLS=100,TOF_TIMEOUT_CYCLES=CLK_HZ/100,TOF_ROUND_CYCLES=CLK_HZ,
 parameter CONFIG_HEX_FILE="src/imu/imu660ra_config.hex"
)(
 input wire clk_50m,reset_n,key_enable_n,key_estop_n,asr_rx,
 input wire encoder_a_l,encoder_b_l,encoder_a_r,encoder_b_r,
 input wire imu_spi_miso,imu_int1,imu_int2,imu_gnd_reserved,
 input wire tof_front_gpio1,tof_left_gpio1,tof_right_gpio1,
 inout wire tof_front_scl,tof_front_sda,tof_front_xshut_n,
 inout wire tof_left_scl,tof_left_sda,tof_left_xshut_n,
 inout wire tof_right_scl,tof_right_sda,tof_right_xshut_n,
 output wire imu_spi_sck,imu_spi_mosi,imu_cs_n,
 output wire oled_scl,inout wire oled_sda,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,asr_tx,
 output wire led_ready,led_run,led_fault,led_heartbeat
);
 // Gowin synthesizes register initialization into configuration INIT values.
 // KEY0 is an ordinary key, so startup reset must not depend on its level.
 reg [31:0] por_count=0;
 wire por_n=por_count>=POR_CYCLES;
 always @(posedge clk_50m)if(!por_n)por_count<=por_count+1'b1;
 wire [2:0] pressed,released_n;
 system_keys #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) keys(
 .clk(clk_50m),.por_n(por_n),.keys_n({key_estop_n,key_enable_n,reset_n}),
 .pressed(pressed),.released_n(released_n));
 wire [2:0] group_id;wire [3:0] item;wire page;
 reg [5:0] reset_count=0;
 always @(posedge clk_50m)
 if(!por_n)reset_count<=32;
 else if(pressed[0] && group_id==0 && !page)reset_count<=32;
 else if(reset_count!=0)reset_count<=reset_count-1'b1;
 wire rst_n=por_n && reset_count==0;
 wire avoid_mode,auto_run,waiting,show_avoid,show_root,sensor_refresh,sensor_monitor,sensor_fault;
 wire [4:0] route_state;wire [7:0] route_reason;
 wire [2:0] tof_seen,tof_valid,post_turn,sample_mask,tof_degraded,reused;wire [47:0] distances;
 wire voice_enable,voice_candidate,menu_valid,gear_commit,menu_stop;
 wire [7:0] menu_command;wire [1:0] gear,gear_candidate,gear_value;
 system_menu menu(.clk(clk_50m),.rst_n(rst_n),.pressed(pressed),.gear(gear),.show_avoid(show_avoid),.show_root(show_root),.avoid_mode(avoid_mode),
 .group_id(group_id),.item(item),.page(page),.voice_enable(voice_enable),
 .voice_candidate(voice_candidate),.gear_candidate(gear_candidate),
 .command_valid(menu_valid),.command(menu_command),.gear_commit(gear_commit),
 .gear_value(gear_value),.stop_request(menu_stop));
 wire imu_ready,calibrating,attitude_valid;
 wire signed [31:0] roll_mdeg,pitch_mdeg,yaw_wrapped,pos_l,pos_r,speed_l,speed_r;
 wire [7:0] status,imu_error,sensor_error,motion_fault,command,response;
 wire command_valid;
 wire [7:0] start_diag;
 system_motion #(.CLK_HZ(CLK_HZ),.BAUD(UART_BAUD),.PWM_HZ(PWM_HZ),
 .WINDOW_CYCLES(WINDOW_CYCLES),.ENC_FILTER(ENC_FILTER),.DIR_DEADTIME(DIR_DEADTIME),
 .IMU_POWERUP(IMU_POWERUP_CYCLES),.IMU_WAIT(IMU_CONFIG_WAIT_CYCLES),
 .IMU_RETRY(IMU_RETRY_CYCLES),.IMU_SAMPLE(IMU_SAMPLE_CYCLES),
 .IMU_SPI_HALF(IMU_SPI_HALF_PERIOD),.IMU_HEALTH(IMU_HEALTH_CYCLES),
 .IMU_TIMEOUT(IMU_TIMEOUT_CYCLES),.CALIBRATION_SAMPLES(IMU_CALIBRATION_SAMPLES),
 .CONFIG_HEX_FILE(CONFIG_HEX_FILE)) control(
 .clk_50m(clk_50m),.reset_n(rst_n),.stop_gate(!reset_n),.stop_request(menu_stop),
 .uart_rx(asr_rx),.voice_enable(voice_enable),.menu_valid(menu_valid),.menu_command(menu_command),
 .gear_commit(gear_commit),.gear_value(gear_value),
 .distances(distances),.tof_seen(tof_seen),.tof_valid(tof_valid),.post_turn(post_turn),.sample_mask(sample_mask),.tof_reused(reused),
 .sensor_fault(sensor_fault),.sensor_degraded(tof_degraded),.sensor_refresh(sensor_refresh),.sensor_monitor(sensor_monitor),
 .avoid_mode(avoid_mode),.auto_run(auto_run),.waiting(waiting),.show_avoid(show_avoid),.show_root(show_root),.route_state(route_state),
 .route_reason(route_reason),
 .encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),.encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),
 .imu_spi_miso(imu_spi_miso),.imu_int1(imu_int1),.imu_int2(imu_int2),.imu_gnd_reserved(imu_gnd_reserved),
 .imu_spi_sck(imu_spi_sck),.imu_spi_mosi(imu_spi_mosi),.imu_cs_n(imu_cs_n),
 .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY),.uart_tx(asr_tx),
 .led_ready(led_ready),.led_run(led_run),.led_fault(led_fault),
 .imu_ready(imu_ready),.calibrating(calibrating),.attitude_valid(attitude_valid),
 .roll_mdeg(roll_mdeg),.pitch_mdeg(pitch_mdeg),.yaw_wrapped(yaw_wrapped),
 .pos_l(pos_l),.pos_r(pos_r),.speed_l(speed_l),.speed_r(speed_r),
 .imu_error(imu_error),.sensor_error(sensor_error),.gear(gear),.status(status),
 .motion_fault(motion_fault),.command_valid(command_valid),.command(command),.response(response),
 .start_diag(start_diag));
 assign led_heartbeat=voice_enable;
 wire [23:0] tof_errors;wire [11:0] tof_statuses;
 obstacle_sensors #(.CLK_HZ(CLK_HZ),.I2C_HZ(TOF_I2C_HZ),.RESET_CYCLES(TOF_RESET_CYCLES),
 .BOOT_CYCLES(TOF_BOOT_CYCLES),.POLL_CYCLES(TOF_POLL_CYCLES),.MAX_POLLS(TOF_MAX_POLLS),
 .TIMEOUT_CYCLES(TOF_TIMEOUT_CYCLES)) sensors(
 .clk(clk_50m),.rst_n(rst_n),.refresh(sensor_refresh),.monitor_enable(sensor_monitor),
 .gpio1({tof_right_gpio1,tof_left_gpio1,tof_front_gpio1}),
 .scl({tof_right_scl,tof_left_scl,tof_front_scl}),.sda({tof_right_sda,tof_left_sda,tof_front_sda}),
 .xshut_n({tof_right_xshut_n,tof_left_xshut_n,tof_front_xshut_n}),
 .seen(tof_seen),.valid(tof_valid),.distances(distances),.post_turn(post_turn),.event_mask(sample_mask),
 .degraded(tof_degraded),.errors(tof_errors),.statuses(tof_statuses),.reused(reused),.fault(sensor_fault),.ages(),.raw_distances(),.comm_ages(),.raw_event_mask(),.fault_bits());
 // Minimal demo UI: the DISPLAY diagnostic group and its ToF page were removed,
 // so the text pipeline only serves the root menu, MANUAL, SETTINGS and the
 // OBSTACLE AVOID panel.
 wire [671:0] text;
 system_pages pages(.clk(clk_50m),.rst_n(rst_n),.group_id(group_id),.item(item),.page(page),
 .start_diag(start_diag),.route_reason(route_reason),
 .avoid_mode(avoid_mode),.auto_run(auto_run),.waiting(waiting),.route_state(route_state),.distances(distances),.tof_valid(tof_valid),
 .tof_reused(reused),
 .attitude_valid(attitude_valid),.roll_mdeg(roll_mdeg),.pitch_mdeg(pitch_mdeg),.yaw_wrapped(yaw_wrapped),
 .voice_enable(voice_enable),.voice_candidate(voice_candidate),.gear(gear),.gear_candidate(gear_candidate),
 .ready(led_ready),.busy(led_run),.fault(led_fault),
 .imu_error(imu_error),.sensor_error(sensor_error),.motion_fault(motion_fault),.command(command),.response(response),
 .text(text));
 system_oled_text #(.CLK_HZ(CLK_HZ),.I2C_HZ(OLED_I2C_HZ),.POWERUP_CYCLES(OLED_POWERUP_CYCLES),
 .FRAME_WAIT_CYCLES(OLED_FRAME_WAIT_CYCLES)) display(
 .clk(clk_50m),.rst_n(rst_n),.text(text),.frame_done(),.scl(oled_scl),.sda(oled_sda));
endmodule
