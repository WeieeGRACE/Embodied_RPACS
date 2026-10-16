`timescale 1ns/1ps
module tb_safety_protection_and_fault_arbitration;
  reg clk=0,rst_n=0,enable=0,system_ready=0,clear_fault=0;
  reg emergency_stop=0,over_current=0,under_voltage=0,over_temperature=0,watchdog_timeout=0,external_fault=0;
  reg range_valid=1; reg [15:0] front_distance=1000; reg signed [15:0] pwm_cmd_l=0,pwm_cmd_r=0;
  wire signed [15:0] pwm_safe_l,pwm_safe_r; wire stby,safety_active,fault_latched,startup_done;
  wire [3:0] fault_code; wire [1:0] obstacle_state; integer checks=0,errors=0; reg completed=0;
  always #5 clk=~clk;
  safety_protection_and_fault_arbitration #(.STARTUP_STABLE_CYCLES(3),.PWM_LIMIT(1000),
    .OBSTACLE_STOP_DISTANCE(200),.OBSTACLE_SLOW_DISTANCE(500),.OBSTACLE_SLOW_LIMIT(100)) dut(.*);
  task check; input condition; input [255:0] name; begin checks=checks+1; if(!condition) begin errors=errors+1; $display("FAIL: %s t=%0t",name,$time); end end endtask
  task tick; input integer n; integer i; begin for(i=0;i<n;i=i+1) @(posedge clk); #1; end endtask
  initial begin
    tick(2); check(!stby && pwm_safe_l==0,"reset off"); rst_n=1; enable=1; system_ready=1; pwm_cmd_l=1200; pwm_cmd_r=-1200;
    tick(1); check(!startup_done,"startup wait"); tick(3); check(startup_done && stby,"startup complete");
    check(pwm_safe_l==1000 && pwm_safe_r==-1000,"pwm clamp");
    emergency_stop=1; tick(1); check(fault_latched && fault_code==1 && !stby,"estop latch"); emergency_stop=0; clear_fault=1; tick(1); clear_fault=0;
    check(!fault_latched && !startup_done,"clear restarts startup"); tick(4); check(stby,"restart");
    over_current=1; under_voltage=1; tick(1); check(fault_code==2,"fault priority"); over_current=0; under_voltage=0; clear_fault=1; tick(1); clear_fault=0; tick(4);
    front_distance=400; tick(1); check(obstacle_state==1 && pwm_safe_l==100,"slow obstacle"); front_distance=100; tick(1); check(obstacle_state==2 && pwm_safe_l==0,"stop obstacle");
    front_distance=1000; tick(1); check(obstacle_state==0 && stby,"obstacle recovery"); range_valid=0; tick(1); check(fault_latched && fault_code==6 && !stby,"sensor fault");
    $display("CHECKS=%0d ERRORS=%0d",checks,errors); completed=(errors==0); #1; $finish;
  end
endmodule
