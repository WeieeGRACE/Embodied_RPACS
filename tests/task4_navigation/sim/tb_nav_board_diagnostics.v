`timescale 1ns/1ps
module tb_nav_board_diagnostics;
  reg clk=0, reset_n=0, key_enable_n=1, key_estop_n=1;
  tri tof_front_scl,tof_front_sda,tof_front_xshut_n,tof_left_scl,tof_left_sda,tof_left_xshut_n,tof_right_scl,tof_right_sda,tof_right_xshut_n;
  tri1 oled_sda; wire oled_scl,PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx;
  reg encoder_a_l=0,encoder_b_l=0,encoder_a_r=0,encoder_b_r=0,imu_spi_miso=0;
  wire imu_spi_sck,imu_spi_mosi,imu_cs_n,led_ready,led_run,led_fault,led_heartbeat;
  task4_navigation_diagnostic_top #(.KEY_CYCLES(4)) dut(
    .clk_50m(clk),.reset_n(reset_n),.key_enable_n(key_enable_n),.key_estop_n(key_estop_n),
    .tof_front_scl(tof_front_scl),.tof_front_sda(tof_front_sda),.tof_front_xshut_n(tof_front_xshut_n),
    .tof_left_scl(tof_left_scl),.tof_left_sda(tof_left_sda),.tof_left_xshut_n(tof_left_xshut_n),
    .tof_right_scl(tof_right_scl),.tof_right_sda(tof_right_sda),.tof_right_xshut_n(tof_right_xshut_n),
    .oled_scl(oled_scl),.oled_sda(oled_sda),.encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),
    .encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),.imu_spi_miso(imu_spi_miso),
    .imu_spi_sck(imu_spi_sck),.imu_spi_mosi(imu_spi_mosi),.imu_cs_n(imu_cs_n),.PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),
    .PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY),.uart_tx(uart_tx),.led_ready(led_ready),.led_run(led_run),.led_fault(led_fault),.led_heartbeat(led_heartbeat));
  always #10 clk=~clk;
  always @(posedge clk) if(reset_n && (STBY!==0 || PWMA!==0 || PWMB!==0 || AIN1!==0 || AIN2!==0 || BIN1!==0 || BIN2!==0)) $fatal(1,"diagnostic motor enabled");
  initial begin
    repeat(5) @(negedge clk); reset_n=1;
    repeat(3000) @(negedge clk);
    if (dut.probe.control_enable!==0) $fatal(1,"diagnostic drive was enabled");
    key_estop_n=0; repeat(10) @(negedge clk); key_estop_n=1;
    repeat(10) @(negedge clk); reset_n=0; repeat(5) @(negedge clk); reset_n=1;
    $display("PASS: diagnostic startup, motor inhibit, estop and reset path"); $finish;
  end
endmodule
