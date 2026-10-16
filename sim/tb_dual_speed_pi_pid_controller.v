`timescale 1ns/1ps
module tb_dual_speed_pi_pid_controller;
    reg clk=0, rst_n=0, tick=0, enable=0;
    reg signed [15:0] target_l=0, feedback_l=0, target_r=0, feedback_r=0;
    reg signed [15:0] kp_l=0,ki_l=0,kd_l=0,kp_r=0,ki_r=0,kd_r=0;
    reg [4:0] shift_l=0, shift_r=0; reg [15:0] lim_l=1000,lim_r=1000;
    wire signed [15:0] control_l,control_r; wire valid,sat_l,sat_r,active_l,active_r;
    integer checks=0; reg completed=0;
    dual_speed_pi_pid_controller dut(clk,rst_n,tick,enable,target_l,feedback_l,target_r,feedback_r,
        kp_l,ki_l,kd_l,kp_r,ki_r,kd_r,shift_l,shift_r,lim_l,lim_r,
        control_l,control_r,valid,sat_l,sat_r,active_l,active_r);
    always #5 clk=~clk;
    task step; begin @(negedge clk); tick=1; @(negedge clk); tick=0; end endtask
    task check; input signed [15:0] a,b; begin checks=checks+1; if(a!==b) $fatal(1,"check %0d got %0d expected %0d",checks,a,b); end endtask
    initial begin
        repeat(2) @(negedge clk); rst_n=1; enable=1; kp_l=2; ki_l=1; kp_r=1; ki_r=0; lim_l=100; lim_r=40;
        target_l=20; target_r=100; step; check(control_l,60); check(control_r,40); if(!sat_r) $fatal(1,"right saturation missing");
        step; check(control_l,80); check(control_r,40);
        target_l=-20; feedback_l=0; step; if(control_l>=0) $fatal(1,"negative response missing");
        enable=0; @(negedge clk); check(control_l,0); check(control_r,0); if(active_l||active_r) $fatal(1,"active not cleared");
        enable=1; target_l=10; target_r=10; kp_l=1;ki_l=0;kd_l=1; step; if(!valid) $fatal(1,"valid missing");
        lim_l=32767; target_l=32767; kp_l=32767; ki_l=32767; kd_l=32767; step; if(!sat_l) $fatal(1,"large value not saturated");
        $display("PASS: %0d checks",checks); completed=1; #10 $finish;
    end
endmodule
