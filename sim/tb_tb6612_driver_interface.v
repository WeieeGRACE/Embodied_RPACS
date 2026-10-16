`timescale 1ns/1ps
module tb_tb6612_driver_interface;
reg clk=0;
always #10 clk=~clk;
reg rst_n=0, enable=0, emergency_stop=0, fault=0, watchdog_timeout=0;
reg signed [15:0] l=0,r=0;
wire a,a1,a2,b,b1,b2,stby;
reg completed=0;
integer checks=0,i,j,ca,cb,blank;
tb6612_driver_interface dut(.clk(clk),.rst_n(rst_n),.enable(enable),
.emergency_stop(emergency_stop),.fault(fault),.watchdog_timeout(watchdog_timeout),
.pwm_signed_l(l),.pwm_signed_r(r),.PWMA(a),.AIN1(a1),.AIN2(a2),
.PWMB(b),.BIN1(b1),.BIN2(b2),.STBY(stby));
task check;
input condition;
input [511:0] description;
begin checks=checks+1; if(condition !== 1'b1) begin
$display("FAIL %0t: %0s",$time,description); $fatal(1); end end
endtask
task tick; begin @(posedge clk); #1; end endtask
task settle; begin repeat(10000) tick; end endtask
task off; begin #1; check({a,a1,a2,b,b1,b2,stby}===7'b0,"safety outputs"); end endtask
task measure;
input integer expected_a,expected_b;
begin
while(dut.phase!=0) tick;
ca=0;cb=0;
for(j=0;j<2500;j=j+1) begin
if(a) ca=ca+1; if(b) cb=cb+1;
check(!(a1&&a2) && !(b1&&b2),"direction invariant");
tick;
end
check(ca==expected_a && cb==expected_b,"2500-clock duty measurement");
end endtask
initial begin
#1; off;
@(negedge clk); rst_n=1;enable=1;l=625;r=-1000;
tick;check(!stby,"first release cycle");
settle;check(a1&&!a2&&!b1&&b2,"independent directions");
repeat(3) measure(625,1000);
@(negedge clk);l=2500;r=0;settle;measure(2500,0);
check(!b1&&!b2,"zero coast");
@(negedge clk);l=-2500;
while(dut.command_l!=-2500) tick;
check(!a,"sign mismatch immediately masks PWM");
tick; check(!a&&a1,"deadtime entry retains old direction");
repeat(2499) begin tick;check(!a&&a1,"exact 2500 clock blanking");end
tick;check(!a&&a2,"direction switch with PWM off");
tick;check(a&&a2,"direction setup clock");
measure(2500,0);
@(negedge clk);l=-32768;r=32767;settle;measure(2500,2500);
check(a2&&b1,"signed extremes saturate correctly");
// Midperiod command changes leave the current pulse intact.
@(negedge clk);l=625;r=-1000;settle;
while(dut.phase!=100) tick;
@(negedge clk);l=0;
check(dut.command_l==625,"command held midperiod");
while(dut.phase!=0) tick;
check(!a,"zero at next boundary");
// Latest target replaces an earlier reversal while deadtime is active.
@(negedge clk);l=2500;r=2500;settle;
@(negedge clk);l=-2500;
while(dut.command_l!=-2500) tick;
tick;
@(negedge clk);l=2500;
settle;check(a&&a1,"cancelled reversal uses latest target");
for(i=0;i<5;i=i+1) begin
@(negedge clk);#2;
case(i) 0:emergency_stop=1;1:fault=1;2:watchdog_timeout=1;3:enable=0;4:rst_n=0;endcase
off;#2;
emergency_stop=0;fault=0;watchdog_timeout=0;enable=1;rst_n=1;
#1;check(!a&&!b&&!stby,"short safety pulse clears state");
tick;check(!stby,"synchronous release");
settle;measure(2500,2500);
end
@(negedge clk);enable=0;off;
repeat(10) tick;off;
$display("PASS: %0d checks; standalone 50 MHz / 20 kHz; duties 0/625/1000/2500, deadtime 2500 clocks",checks);
completed=1;$finish;
end
initial begin #5000000;$fatal(1,"timeout");end
endmodule
