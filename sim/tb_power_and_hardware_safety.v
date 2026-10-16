`timescale 1ns/1ps
module tb_power_and_hardware_safety;
    reg clk = 0, rst_n = 0, enable = 0, clear_fault = 0, system_ready = 0;
    reg emergency_stop = 0, over_current = 0, under_voltage = 0;
    reg over_temperature = 0, external_fault = 0;
    reg signed [15:0] pwm_cmd_l = 0, pwm_cmd_r = 0;
    wire signed [15:0] pwm_safe_l, pwm_safe_r;
    wire stby, fault_latched, startup_done, safety_active;
    wire [2:0] fault_code;
    reg completed = 0;

    always #5 clk = ~clk;

    power_and_hardware_safety #(.STABLE_CYCLES(4), .PWM_LIMIT(1000)) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable), .clear_fault(clear_fault),
        .system_ready(system_ready), .emergency_stop(emergency_stop),
        .over_current(over_current), .under_voltage(under_voltage),
        .over_temperature(over_temperature), .external_fault(external_fault),
        .pwm_cmd_l(pwm_cmd_l), .pwm_cmd_r(pwm_cmd_r),
        .pwm_safe_l(pwm_safe_l), .pwm_safe_r(pwm_safe_r), .stby(stby),
        .fault_latched(fault_latched), .fault_code(fault_code),
        .startup_done(startup_done), .safety_active(safety_active));

    task check;
        input condition;
        input [8*80-1:0] message;
        begin if (!condition) begin $display("FAIL: %0s", message); $fatal; end end
    endtask

    task tick; begin @(posedge clk); #1; end endtask

    initial begin
        #1;
        check(!stby && pwm_safe_l == 0 && pwm_safe_r == 0, "reset output off");
        rst_n = 1; enable = 1; system_ready = 1;
        repeat (3) tick;
        check(!startup_done && !stby, "stabilizing");
        tick;
        check(startup_done && stby, "startup complete");
        pwm_cmd_l = 16'sd1500; pwm_cmd_r = -16'sd1200; tick;
        check(pwm_safe_l == 1000 && pwm_safe_r == -1000, "pwm clamp");

        emergency_stop = 1; #1;
        check(!stby && pwm_safe_l == 0 && pwm_safe_r == 0, "emergency stop gate");
        tick;
        check(fault_latched && fault_code == 1, "estop latch");
        emergency_stop = 0; clear_fault = 1; tick; clear_fault = 0;
        check(!fault_latched && !startup_done && !stby, "clear restarts startup");

        repeat (4) tick; check(stby, "restart after clear");
        over_current = 1; tick; over_current = 0;
        check(fault_latched && fault_code == 2 && !stby, "overcurrent latch");
        tick; check(fault_latched, "fault remains latched");
        clear_fault = 1; tick; clear_fault = 0;
        repeat (4) tick; check(stby, "overcurrent recovery");

        under_voltage = 1; over_temperature = 1; external_fault = 1; tick;
        check(fault_code == 3, "fault priority");
        under_voltage = 0; over_temperature = 0; external_fault = 0;
        clear_fault = 1; tick; clear_fault = 0;
        enable = 0; tick; check(!stby && !startup_done, "disable output");
        completed = 1; $display("PASS: power_and_hardware_safety checks complete");
        #10 $finish;
    end
endmodule
