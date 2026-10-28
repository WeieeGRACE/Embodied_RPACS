`timescale 1ns/1ps
// Resource-lean variant of the UART bridge board top.
//
// Same protocol, same safety behaviour, but built with the low-resource knobs:
//   * diagnostics counters 8-bit instead of 16-bit
//   * no ACK frames (the host relies on the 50 Hz telemetry + cmd_seq echo instead)
//
// Measured on GW5A-25: full build 1154 Logic / 858 CLS -> this variant 1110 / 830.
// See records/2026-10-29_uart_bridge_validation.md section 7 for the table.
//
// SAFETY: identical to uart_bridge_top - every motor driver output is tied to 0.
module uart_bridge_lite_top #(
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer BAUD        = 115200,
    parameter integer COUNTS_PER_M = 1116
)(
    input  wire clk_50m,
    input  wire reset_n,

    input  wire uart_rx,
    output wire uart_tx,

    output wire led_ready,
    output wire led_run,
    output wire led_fault,
    output wire led_heartbeat,

    input  wire encoder_a_l,
    input  wire encoder_b_l,
    input  wire encoder_a_r,
    input  wire encoder_b_r,

    output wire PWMA, AIN1, AIN2,
    output wire PWMB, BIN1, BIN2,
    output wire STBY,

    input  wire imu_gnd_reserved,
    input  wire imu_spi_sck,
    input  wire imu_spi_mosi,
    input  wire imu_spi_miso,
    input  wire imu_cs_n,
    input  wire imu_int1,
    input  wire imu_int2,

    input  wire tof_front_gpio1, tof_left_gpio1, tof_right_gpio1,
    inout  wire tof_front_scl, tof_front_sda, tof_front_xshut_n,
    inout  wire tof_left_scl,  tof_left_sda,  tof_left_xshut_n,
    inout  wire tof_right_scl, tof_right_sda, tof_right_xshut_n,

    input  wire asr_rx,
    output wire asr_tx
);
    uart_bridge_top #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD),
        .COUNTS_PER_M(COUNTS_PER_M),
        .CNT_W(8),
        .ENABLE_ACK(0)
    ) u_full (
        .clk_50m(clk_50m), .reset_n(reset_n),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .led_ready(led_ready), .led_run(led_run),
        .led_fault(led_fault), .led_heartbeat(led_heartbeat),
        .encoder_a_l(encoder_a_l), .encoder_b_l(encoder_b_l),
        .encoder_a_r(encoder_a_r), .encoder_b_r(encoder_b_r),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2),
        .PWMB(PWMB), .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY),
        .imu_gnd_reserved(imu_gnd_reserved),
        .imu_spi_sck(imu_spi_sck), .imu_spi_mosi(imu_spi_mosi),
        .imu_spi_miso(imu_spi_miso), .imu_cs_n(imu_cs_n),
        .imu_int1(imu_int1), .imu_int2(imu_int2),
        .tof_front_gpio1(tof_front_gpio1), .tof_left_gpio1(tof_left_gpio1),
        .tof_right_gpio1(tof_right_gpio1),
        .tof_front_scl(tof_front_scl), .tof_front_sda(tof_front_sda),
        .tof_front_xshut_n(tof_front_xshut_n),
        .tof_left_scl(tof_left_scl), .tof_left_sda(tof_left_sda),
        .tof_left_xshut_n(tof_left_xshut_n),
        .tof_right_scl(tof_right_scl), .tof_right_sda(tof_right_sda),
        .tof_right_xshut_n(tof_right_xshut_n),
        .asr_rx(asr_rx), .asr_tx(asr_tx)
    );
endmodule
