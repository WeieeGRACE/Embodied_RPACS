`timescale 1ns/1ps
module tb_nav_board_safe_default;
    reg clk=0,reset_n=0,key_enable_n=1,key_estop_n=1;
    tri tof_front_scl,tof_front_sda,tof_front_xshut_n;
    tri tof_left_scl,tof_left_sda,tof_left_xshut_n;
    tri tof_right_scl,tof_right_sda,tof_right_xshut_n;
    tri oled_sda;
    wire oled_scl,PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,uart_tx;
    wire led_ready,led_run,led_fault,led_heartbeat;
    reg encoder_a_l=0,encoder_b_l=0,encoder_a_r=0,encoder_b_r=0;
    reg imu_spi_miso=0,imu_int1=0,imu_int2=0,imu_gnd_reserved=0;
    wire imu_spi_sck,imu_spi_mosi,imu_cs_n;

    task4_navigation_single_leg_probe_top #(.KEY_CYCLES(4)) dut(
      .clk_50m(clk),.reset_n(reset_n),.key_enable_n(key_enable_n),.key_estop_n(key_estop_n),
      .tof_front_scl(tof_front_scl),.tof_front_sda(tof_front_sda),.tof_front_xshut_n(tof_front_xshut_n),
      .tof_left_scl(tof_left_scl),.tof_left_sda(tof_left_sda),.tof_left_xshut_n(tof_left_xshut_n),
      .tof_right_scl(tof_right_scl),.tof_right_sda(tof_right_sda),.tof_right_xshut_n(tof_right_xshut_n),
      .oled_scl(oled_scl),.oled_sda(oled_sda),
      .encoder_a_l(encoder_a_l),.encoder_b_l(encoder_b_l),
      .encoder_a_r(encoder_a_r),.encoder_b_r(encoder_b_r),
      .imu_spi_miso(imu_spi_miso),
      .imu_spi_sck(imu_spi_sck),.imu_spi_mosi(imu_spi_mosi),.imu_cs_n(imu_cs_n),
      .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY),
      .uart_tx(uart_tx),.led_ready(led_ready),.led_run(led_run),.led_fault(led_fault),
      .led_heartbeat(led_heartbeat));

    always #10 clk=~clk;

    initial begin
        repeat(5) @(negedge clk);reset_n=1;
        repeat(20) @(negedge clk);
        force dut.probe.nav_velocity_l_mm_s=32'sd50;
        force dut.probe.nav_velocity_r_mm_s=-32'sd50;
        #1;
        if(dut.probe.target_l!==16'sd3 || dut.probe.target_r!==-16'sd3)
            $fatal(1,"50 mm/s encoder scale exceeded the requested speed ceiling: L=%0d R=%0d",
                dut.probe.target_l,dut.probe.target_r);
        force dut.probe.nav_velocity_l_mm_s=32'sd8;
        force dut.probe.nav_velocity_r_mm_s=-32'sd8;
        #1;
        if(dut.probe.target_l!==16'sd0 || dut.probe.target_r!==16'sd0)
            $fatal(1,"sub-count speed quantization is not symmetric toward zero");
        release dut.probe.nav_velocity_l_mm_s;
        release dut.probe.nav_velocity_r_mm_s;
        key_enable_n=0;repeat(5) @(negedge clk);key_enable_n=1;
        repeat(20) @(negedge clk);
        if(STBY!==1'b0 || PWMA!==1'b0 || PWMB!==1'b0 || AIN1!==1'b0 || AIN2!==1'b0 ||
           BIN1!==1'b0 || BIN2!==1'b0)
            $fatal(1,"default build enabled motor outputs: STBY=%b pwm=%b%b",STBY,PWMA,PWMB);
        if(!led_fault) $fatal(1,"probe start without calibrated sensors did not latch a fault");

        // Exercise the board-level asynchronous E-stop gate while injecting a
        // run request at the driver boundary. This avoids requiring real
        // sensors or changing route/calibration gates in the production RTL.
        force dut.probe.physical_fault=1'b0;
        force dut.probe.control_enable=1'b1;
        force dut.probe.ramp_l=16'sd100;
        force dut.probe.ramp_r=16'sd100;
        repeat(5) @(negedge clk);
        if(STBY!==1'b1) $fatal(1,"driver gate did not enable under injected run request");
        key_estop_n=0;#1;
        if(STBY!==1'b0 || PWMA!==1'b0 || PWMB!==1'b0)
            $fatal(1,"E-stop did not asynchronously disable driver outputs");
        key_estop_n=1;repeat(5) @(negedge clk);
        if(STBY!==1'b0 || dut.probe.estop_latched!==1'b1)
            $fatal(1,"E-stop release resumed motion without reset");

        reset_n=0;repeat(3) @(negedge clk);
        release dut.probe.control_enable;
        release dut.probe.physical_fault;
        release dut.probe.ramp_l;
        release dut.probe.ramp_r;
        if(STBY!==1'b0) $fatal(1,"reset did not keep TB6612 disabled");
        reset_n=1;repeat(30) @(negedge clk);
        if(STBY!==1'b0 || PWMA!==1'b0 || PWMB!==1'b0)
            $fatal(1,"reset or sensor-fault recovery unexpectedly resumed motion");
        $display("PASS: sensor-calibration fault, injected driver E-stop cutoff/latch, reset-safe re-entry");
        $finish;
    end
endmodule
