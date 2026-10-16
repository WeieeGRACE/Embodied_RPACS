`timescale 1ns/1ps
module tb_pwm_and_startup_ramp;
localparam P=2500;
reg clk=0;
always #10 clk=~clk;
reg rst_n=0, enable=0;
reg signed [15:0] pwm_cmd_l=0,pwm_cmd_r=0;
reg [15:0] pwm_limit=P,pwm_min=0,ramp_step=P;
wire pwm_out_l,pwm_out_r;
wire signed [15:0] pwm_actual_l,pwm_actual_r;
integer checks=0;
reg completed=0;
reg [511:0] case_name;
pwm_and_startup_ramp dut(.clk(clk),.rst_n(rst_n),.enable(enable),.pwm_cmd_l(pwm_cmd_l),.pwm_cmd_r(pwm_cmd_r),.pwm_limit(pwm_limit),.pwm_min(pwm_min),.ramp_step(ramp_step),.pwm_out_l(pwm_out_l),.pwm_out_r(pwm_out_r),.pwm_actual_l(pwm_actual_l),.pwm_actual_r(pwm_actual_r));
task check;
input integer l,r;
begin
checks=checks+1;
if(pwm_actual_l !== l || pwm_actual_r !== r) begin
$display("FAIL %0s t=%0t expected=(%0d,%0d) actual=(%0d,%0d)",case_name,$time,l,r,pwm_actual_l,pwm_actual_r); $fatal(1);
end
end
endtask
task advance;
input integer l,r;
begin repeat(P) begin @(posedge clk); #1; end check(l,r); end
endtask
task measure;
input integer l,r;
integer i,hl,hr;
time start_time;
begin
hl=0;hr=0;start_time=$time;
for(i=0;i<P;i=i+1) begin
@(negedge clk); check(l,r);
if(pwm_out_l !== (i<((l<0)?-l:l)) || pwm_out_r !== (i<((r<0)?-r:r))) begin
$display("FAIL %0s pulse shape at tick %0d",case_name,i);$fatal(1);end
hl=hl+pwm_out_l;hr=hr+pwm_out_r;
@(posedge clk);#1;
end
if($time-start_time != 50000 || hl != ((l<0)?-l:l) || hr != ((r<0)?-r:r)) $fatal(1);
$display("PASS %0s period=50000 ns high_ticks=(%0d,%0d)",case_name,hl,hr);
end
endtask
task restart;
begin
@(negedge clk);enable=0;#1;check(0,0);
if(pwm_out_l !== 0 || pwm_out_r !== 0) $fatal(1);
@(posedge clk);#1;
@(negedge clk);enable=1;
repeat(P) begin @(posedge clk);#1;end
end
endtask
initial begin
case_name="reset and disabled";
#1;rst_n=1;#1;rst_n=0;#1;check(0,0);
repeat(3) @(posedge clk);
@(negedge clk);rst_n=1;
@(posedge clk);#1;check(0,0);
case_name="0 percent";restart;measure(0,0);
case_name="25/40 percent independent channels";
pwm_cmd_l=625;pwm_cmd_r=-1000;advance(625,-1000);measure(625,-1000);
case_name="100 percent, saturation and -32768";
pwm_cmd_l=32767;pwm_cmd_r=-32768;pwm_limit=65535;
advance(2500,-2500);measure(2500,-2500);
case_name="startup ramp exact endpoint";
pwm_limit=P;pwm_cmd_l=350;pwm_cmd_r=-250;ramp_step=100;
restart;check(100,-100);advance(200,-200);advance(300,-250);advance(350,-250);
case_name="stop ramp";
pwm_cmd_l=0;pwm_cmd_r=0;
advance(250,-150);advance(150,-50);advance(50,0);advance(0,0);measure(0,0);
case_name="reverse complete zero period";
pwm_cmd_l=250;pwm_cmd_r=-250;
advance(100,-100);advance(200,-200);advance(250,-250);
pwm_cmd_l=-250;pwm_cmd_r=250;
advance(150,-150);advance(50,-50);advance(0,0);
measure(0,0);check(-100,100);advance(-200,200);advance(-250,250);
case_name="target changes during reverse deceleration";
pwm_cmd_l=250;pwm_cmd_r=-250;advance(-150,150);
pwm_cmd_l=-250;pwm_cmd_r=250;advance(-250,250);
case_name="minimum target preserves startup ramp";
pwm_cmd_l=1;pwm_cmd_r=-1;pwm_min=220;
restart;check(100,-100);advance(200,-200);advance(220,-220);
case_name="minimum exceeds limit";
pwm_limit=120;pwm_min=500;advance(120,-120);
case_name="zero limit while reversing";
pwm_cmd_l=-1000;pwm_cmd_r=1000;pwm_limit=0;ramp_step=1;
advance(0,0);measure(0,0);
case_name="zero command with minimum";
pwm_limit=P;pwm_cmd_l=0;pwm_cmd_r=0;advance(0,0);
case_name="zero step becomes one";
pwm_min=0;ramp_step=0;pwm_cmd_l=3;pwm_cmd_r=-2;
advance(1,-1);advance(2,-2);advance(3,-2);advance(3,-2);
case_name="large step and lowered limit during reversal";
ramp_step=65535;pwm_cmd_l=2000;pwm_cmd_r=-2000;advance(2000,-2000);
pwm_cmd_l=-2000;pwm_cmd_r=2000;pwm_limit=100;ramp_step=1;
advance(100,-100);advance(99,-99);
case_name="mid-period updates preserve pulse shape";
pwm_min=0;pwm_limit=P;ramp_step=P;pwm_cmd_l=1000;pwm_cmd_r=625;
restart;check(1000,625);
fork
begin repeat(100) @(negedge clk);#2;pwm_cmd_l=100;pwm_cmd_r=200;pwm_limit=500;end
measure(1000,625);
join
check(500,500);advance(100,200);
case_name="disable during high pulse and restart";
pwm_limit=P;ramp_step=100;pwm_cmd_l=300;pwm_cmd_r=-300;
restart;check(100,-100);
#2;enable=0;#1;check(0,0);
if(pwm_out_l !== 0 || pwm_out_r !== 0) $fatal(1);
@(posedge clk);#1;restart;check(100,-100);
case_name="async reset during high pulse";
#2;rst_n=0;#1;check(0,0);
if(pwm_out_l !== 0 || pwm_out_r !== 0) $fatal(1);
repeat(2) @(posedge clk);
@(negedge clk);rst_n=1;
repeat(P) begin @(posedge clk);#1;end
check(100,-100);
completed=1; $display("PASS ALL: %0d checks; default PWM 20 kHz",checks);$finish;
end
initial begin #10000000;$display("FAIL timeout %0s",case_name);$fatal(1);end
endmodule

