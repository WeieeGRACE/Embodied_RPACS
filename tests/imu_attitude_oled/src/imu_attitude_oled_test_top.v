`timescale 1ns/1ps
// Isolated OLED/IMU validation top. Motor and UART interfaces are not used.
module imu_attitude_oled_test_top #(
    parameter integer POWERUP_CYCLES = 1000000,
    parameter integer CONFIG_WAIT_CYCLES = 1000000,
    parameter integer ID_RETRY_CYCLES = 50000,
    parameter integer RETRY_CYCLES = 50000000,
    parameter integer SAMPLE_CYCLES = 250000,
    parameter integer SPI_HALF_PERIOD = 25,
    parameter integer CALIBRATION_SAMPLES = 200
)(
    input wire clk_50m,
    input wire reset_n,
    input wire imu_spi_miso,
    input wire imu_int1,
    input wire imu_int2,
    output wire imu_spi_sck,
    output wire imu_spi_mosi,
    output wire imu_cs_n,
    output wire oled_scl,
    inout wire oled_sda
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_pipe;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n)
            reset_pipe <= 0;
        else
            reset_pipe <= {reset_pipe[0], 1'b1};
    end
    wire rst_n = reset_pipe[1];

    wire signed [15:0] accel_x, accel_y, accel_z;
    wire signed [15:0] gyro_x, gyro_y, gyro_z;
    wire sample_valid;
    wire [31:0] sample_sequence;
    wire [7:0] chip_id;
    wire [7:0] init_status;
    wire [7:0] power_conf_readback, init_ctrl_readback, sensor_error;
    wire ready;
    wire [7:0] error_code;
    wire int1_sync, int2_sync;

    imu660ra_driver #(
        .CLK_HZ(50000000),
        .SPI_HALF_PERIOD(SPI_HALF_PERIOD),
        .POWERUP_CYCLES(POWERUP_CYCLES),
        .CONFIG_WAIT_CYCLES(CONFIG_WAIT_CYCLES),
        .ID_RETRY_CYCLES(ID_RETRY_CYCLES),
        .RETRY_CYCLES(RETRY_CYCLES),
        .SAMPLE_CYCLES(SAMPLE_CYCLES),
        .CONFIG_HEX_FILE("../../src/imu/imu660ra_config.hex")
    ) driver (
        .clk(clk_50m), .rst_n(rst_n),
        .spi_miso(imu_spi_miso), .imu_int1(imu_int1), .imu_int2(imu_int2),
        .spi_sclk(imu_spi_sck), .spi_mosi(imu_spi_mosi), .spi_cs_n(imu_cs_n),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .sample_valid(sample_valid), .sample_sequence(sample_sequence),
        .chip_id(chip_id), .init_status(init_status), .ready(ready),
        .error_code(error_code), .power_conf_readback(power_conf_readback),
        .init_ctrl_readback(init_ctrl_readback), .sensor_error(sensor_error),
        .int1_sync(int1_sync), .int2_sync(int2_sync)
    );

    wire calibrating;
    wire attitude_valid;
    wire signed [31:0] roll_mdeg, pitch_mdeg, yaw_mdeg;
    wire signed [15:0] gyro_bias_x, gyro_bias_y, gyro_bias_z;

    imu_attitude_fixed #(
        .CALIBRATION_SAMPLES(CALIBRATION_SAMPLES),
        .GYRO_WEIGHT(98), .FILTER_DENOMINATOR(100)
    ) attitude (
        .clk(clk_50m), .rst_n(rst_n), .ready_in(ready),
        .sample_valid(sample_valid),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .calibrating(calibrating), .attitude_valid(attitude_valid),
        .roll_mdeg(roll_mdeg), .pitch_mdeg(pitch_mdeg), .yaw_mdeg(yaw_mdeg),
        .gyro_bias_x(gyro_bias_x), .gyro_bias_y(gyro_bias_y), .gyro_bias_z(gyro_bias_z)
    );

    imu_attitude_oled #(
        .CLK_HZ(50000000), .I2C_HZ(250000),
        .FRAME_WAIT_CYCLES(7000000)
    ) display (
        .clk(clk_50m), .rst_n(rst_n),
        .imu_ready(ready), .imu_error(error_code != 0),
        .calibrating(calibrating), .attitude_valid(attitude_valid),
        .roll_mdeg(roll_mdeg), .pitch_mdeg(pitch_mdeg), .yaw_mdeg(yaw_mdeg),
        .scl(oled_scl), .sda(oled_sda)
    );
endmodule
