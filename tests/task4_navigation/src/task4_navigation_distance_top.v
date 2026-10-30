`timescale 1ns/1ps
// 距离到达即成功版单段探针：不依赖 ToF 地图关联，ToF 只做停车边界（120 mm）。
module task4_navigation_distance_top (
    input  wire clk_50m, reset_n, key_enable_n, key_estop_n,
    inout  wire tof_front_scl, tof_front_sda, tof_front_xshut_n,
    inout  wire tof_left_scl, tof_left_sda, tof_left_xshut_n,
    inout  wire tof_right_scl, tof_right_sda, tof_right_xshut_n,
    output wire oled_scl, inout wire oled_sda,
    input  wire encoder_a_l, encoder_b_l, encoder_a_r, encoder_b_r,
    input  wire imu_spi_miso,
    output wire imu_spi_sck, imu_spi_mosi, imu_cs_n,
    output wire PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY, uart_tx,
    output wire led_ready, led_run, led_fault, led_heartbeat
);
    task4_navigation_single_leg_probe_top #(
        .REQUIRE_WALL_EVENT(0)
    ) probe (
        .clk_50m(clk_50m), .reset_n(reset_n),
        .key_enable_n(key_enable_n), .key_estop_n(key_estop_n),
        .tof_front_scl(tof_front_scl), .tof_front_sda(tof_front_sda),
        .tof_front_xshut_n(tof_front_xshut_n),
        .tof_left_scl(tof_left_scl), .tof_left_sda(tof_left_sda),
        .tof_left_xshut_n(tof_left_xshut_n),
        .tof_right_scl(tof_right_scl), .tof_right_sda(tof_right_sda),
        .tof_right_xshut_n(tof_right_xshut_n),
        .oled_scl(oled_scl), .oled_sda(oled_sda),
        .encoder_a_l(encoder_a_l), .encoder_b_l(encoder_b_l),
        .encoder_a_r(encoder_a_r), .encoder_b_r(encoder_b_r),
        .imu_spi_miso(imu_spi_miso), .imu_spi_sck(imu_spi_sck),
        .imu_spi_mosi(imu_spi_mosi), .imu_cs_n(imu_cs_n),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2), .PWMB(PWMB),
        .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY), .uart_tx(uart_tx),
        .led_ready(led_ready), .led_run(led_run),
        .led_fault(led_fault), .led_heartbeat(led_heartbeat)
    );
endmodule
