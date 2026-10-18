`timescale 1ns/1ps
// Production integration: one clock, one reset release, one event-only UART.
module robot_system_top #(
 parameter integer CLK_HZ=50000000, UART_BAUD=115200, UART_BIT_CYCLES=434,
 WATCHDOG_CYCLES=25000000, WINDOW_CYCLES=500000, DEBOUNCE_CYCLES=500000,
 RUN_CYCLES=400000000, STARTUP_CYCLES=250000,
 IMU_POWERUP_CYCLES=1000000, IMU_CONFIG_WAIT_CYCLES=1000000,
 IMU_RETRY_CYCLES=50000000, IMU_ID_RETRY_CYCLES=50000,
 IMU_SAMPLE_CYCLES=250000, IMU_SPI_HALF_PERIOD=25,
 IMU_HEALTH_CYCLES=50000000, IMU_TIMEOUT_CYCLES=5000000,
 parameter signed [15:0] TARGET_L=10,TARGET_R=10
)(
 input wire clk_50m,reset_n,key_enable_n,key_estop_n,uart_rx,
 input wire encoder_a_l,encoder_b_l,encoder_a_r,encoder_b_r,
 input wire imu_spi_miso,imu_int1,imu_int2,imu_gnd_reserved,
 output wire imu_spi_sck,imu_spi_mosi,imu_cs_n,
 output wire PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx,
 output wire led_ready,led_run,led_fault,led_heartbeat
);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_pipe;
 always @(posedge clk_50m or negedge reset_n)
 if(!reset_n) reset_pipe<=0; else reset_pipe<={reset_pipe[0],1'b1};
 wire rst_n=reset_pipe[1];
 (* syn_keep=1 *) wire signed [15:0] ax,ay,az,gx,gy,gz;
 (* syn_keep=1 *) wire sample_valid;
 (* syn_keep=1 *) wire imu_int1_sync,imu_int2_sync;
 wire imu_ready; wire [7:0] imu_error,sensor_error;
 imu660ra_driver #(.CLK_HZ(CLK_HZ),.SPI_HALF_PERIOD(IMU_SPI_HALF_PERIOD),
 .POWERUP_CYCLES(IMU_POWERUP_CYCLES),.CONFIG_WAIT_CYCLES(IMU_CONFIG_WAIT_CYCLES),
 .ID_RETRY_CYCLES(IMU_ID_RETRY_CYCLES),.RETRY_CYCLES(IMU_RETRY_CYCLES),
 .SAMPLE_CYCLES(IMU_SAMPLE_CYCLES),.HEALTH_CHECK_CYCLES(IMU_HEALTH_CYCLES)) imu(
 .clk(clk_50m),.rst_n(rst_n),.spi_miso(imu_spi_miso),.imu_int1(imu_int1),.imu_int2(imu_int2),
 .spi_sclk(imu_spi_sck),.spi_mosi(imu_spi_mosi),.spi_cs_n(imu_cs_n),
 .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
 .sample_valid(sample_valid),.sample_sequence(),.chip_id(),.init_status(),
 .power_conf_readback(),.init_ctrl_readback(),.sensor_error(sensor_error),
 .ready(imu_ready),.error_code(imu_error),.int1_sync(imu_int1_sync),.int2_sync(imu_int2_sync));
 // Retained internal sample bank for later control consumers; no periodic serial output.
 (* syn_preserve=1, syn_keep=1 *) reg [98:0] imu_sample;
 always @(posedge clk_50m or negedge rst_n)
 if(!rst_n) imu_sample<=0;
 else if(sample_valid) imu_sample<={imu_gnd_reserved,imu_int2_sync,imu_int1_sync,gz,gy,gx,az,ay,ax};
 reg first_sample,ready_seen;
 reg [31:0] sample_age;
 reg [7:0] imu_faults;
 wire sample_timeout=imu_ready && !sample_valid && sample_age>=IMU_TIMEOUT_CYCLES-1;
 wire ready_lost=ready_seen && !imu_ready;
 wire imu_bad=(imu_error!=0)||(sensor_error!=0)||ready_lost||sample_timeout;
 always @(posedge clk_50m or negedge rst_n) begin
 if(!rst_n) begin first_sample<=0;ready_seen<=0;sample_age<=0;imu_faults<=0;end
 else begin
  if(imu_ready) ready_seen<=1;
  if(sample_valid) begin first_sample<=1;sample_age<=0;end
  else if(imu_ready && sample_age<IMU_TIMEOUT_CYCLES) sample_age<=sample_age+1'b1;
  else if(!imu_ready) sample_age<=0;
  if(imu_error==4) imu_faults[0]<=1;
  if(imu_error==5) imu_faults[1]<=1;
  if(imu_error==6) imu_faults[2]<=1;
  if(imu_error==7) imu_faults[3]<=1;
  if(sensor_error!=0) imu_faults[4]<=1;
  if(ready_lost) imu_faults[5]<=1;
  if(sample_timeout) imu_faults[6]<=1;
  if(imu_error!=0 && imu_error!=4 && imu_error!=5 && imu_error!=6 && imu_error!=7) imu_faults[7]<=1;
 end
 end
 wire imu_stop=(|imu_faults)||imu_bad;
 wire modules_ready=imu_ready && first_sample && !imu_stop;
 wire hb,wd,enc_l,enc_r,motion_fault,motion_run;
 reg estop_seen;
 wire estop_event=!key_estop_n;
 always @(posedge clk_50m or posedge estop_event)
 if(estop_event) estop_seen<=1;else if(!rst_n) estop_seen<=0;
 robot_closed_loop_test_top #(.SYSTEM_INTEGRATION(1),.TELEMETRY_ENABLE(0),
 .TEST_BOTH(1),.WATCHDOG_ENABLE(1),.WATCHDOG_CYCLES(WATCHDOG_CYCLES),
 .UART_BIT_CYCLES(UART_BIT_CYCLES),.TARGET_L(TARGET_L),.TARGET_R(TARGET_R),
 .WINDOW_CYCLES(WINDOW_CYCLES),.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),
 .RUN_CYCLES(RUN_CYCLES),.STARTUP_CYCLES(STARTUP_CYCLES)) motion(
 .clk_50m(clk_50m),.reset_n(rst_n),.key_enable_n(key_enable_n),.key_estop_n(key_estop_n),
 .uart_rx(uart_rx),.system_ready(modules_ready),.system_fault(imu_stop),
 .status_heartbeat(hb),.status_watchdog(wd),.status_enc_l(enc_l),.status_enc_r(enc_r),
 .status_fault_code(),
 .encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),.encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),
 .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY),
 .uart_tx(),.led_enable(motion_run),.led_estop(),.led_fault(motion_fault),.led_saturated());
 assign led_ready=modules_ready && !motion_fault && !wd;
 assign led_run=motion_run; assign led_fault=imu_stop||motion_fault||wd;
 assign led_heartbeat=hb;
 // Bit per distinct source/code, retained until KEY0. Simultaneous faults cannot overwrite.
 wire [11:0] errors={estop_seen,enc_r,enc_l,wd,imu_faults};
 system_event_logger #(.CLK_HZ(CLK_HZ),.BAUD(UART_BAUD)) logger(
 .clk(clk_50m),.rst_n(rst_n),.imu_started(imu_ready && first_sample),
 .imu_failed(imu_stop),.imu_error(imu_error),.errors(errors),.tx(uart_tx));
endmodule
