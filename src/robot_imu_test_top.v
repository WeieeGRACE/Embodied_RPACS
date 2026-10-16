`timescale 1ns/1ps
module robot_imu_test_top #(
    parameter integer POWERUP_CYCLES = 1000000,
    parameter integer CONFIG_WAIT_CYCLES = 1000000,
    parameter integer ID_RETRY_CYCLES = 50000,
    parameter integer RETRY_CYCLES = 50000000,
    parameter integer SAMPLE_CYCLES = 250000,
    parameter integer REPORT_CYCLES = 2500000,
    // 50 MHz / (2 * 25) = 1 MHz SPI clock.
    parameter integer SPI_HALF_PERIOD = 25
)(
    input wire clk_50m,
    input wire reset_n,
    input wire imu_spi_miso,
    input wire imu_int1,
    input wire imu_int2,
    output wire imu_spi_sck,
    output wire imu_spi_mosi,
    output wire imu_cs_n,
    output wire uart_tx,
    output wire led_ready,
    output wire led_error,
    output wire led_int1,
    output wire led_int2
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
        .SAMPLE_CYCLES(SAMPLE_CYCLES)
    ) driver (
        .clk(clk_50m), .rst_n(rst_n),
        .spi_miso(imu_spi_miso), .imu_int1(imu_int1), .imu_int2(imu_int2),
        .spi_sclk(imu_spi_sck), .spi_mosi(imu_spi_mosi), .spi_cs_n(imu_cs_n),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .sample_valid(sample_valid), .sample_sequence(sample_sequence),
        .chip_id(chip_id), .init_status(init_status), .ready(ready), .error_code(error_code),
        .power_conf_readback(power_conf_readback), .init_ctrl_readback(init_ctrl_readback), .sensor_error(sensor_error),
        .int1_sync(int1_sync), .int2_sync(int2_sync)
    );

    imu_uart_telemetry #(
        .CLK_HZ(50000000), .BAUD(115200), .REPORT_CYCLES(REPORT_CYCLES)
    ) telemetry (
        .clk(clk_50m), .rst_n(rst_n), .chip_id(chip_id), .init_status(init_status), .ready_in(ready),
        .power_conf_readback(power_conf_readback), .init_ctrl_readback(init_ctrl_readback), .sensor_error(sensor_error),
        .error_code(error_code), .sample_sequence(sample_sequence),
        .int1(int1_sync), .int2(int2_sync),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z), .tx(uart_tx)
    );

    assign led_ready = ready;
    assign led_error = (error_code != 0);
    assign led_int1 = int1_sync;
    assign led_int2 = int2_sync;
endmodule
