`timescale 1ns/1ps
module tb_motion_control_and_mode_fsm;
parameter integer TEST_CLK_HZ=10000;
localparam integer P=TEST_CLK_HZ/1000;
reg clk=0;always #10 clk=~clk;
reg rst_n=0,enable=0,stop_request=0,external_fault=0,clear_fault=0,cmd_valid=0;
wire cmd_ready;
reg [3:0] cmd_mode=2;
reg [1:0] cmd_end=0;
reg cmd_reverse=0;
reg [31:0] cmd_speed_l=7,cmd_speed_r=7,cmd_goal=10,cmd_timeout_ms=0;
reg [31:0] cmd_speed_limit=100,cmd_accel_step=3,cmd_decel_step=2;
reg signed [31:0] position_l=0,position_r=0,speed_l=0,speed_r=0,yaw_mdeg=0;
reg speed_valid_l=1,speed_valid_r=1,yaw_valid=1;
wire signed [31:0] target_l,target_r;
wire drive_enable,controller_clear,busy,done,aborted,rejected;
wire [3:0] mode,reason;wire [2:0] phase;wire [7:0] fault_code;
reg completed=0;
reg terminal_seen=0; integer decel_expected_l,decel_expected_r; reg check_decel;
integer checks=0,done_count=0,abort_count=0,reject_count=0;
integer i,j,k,seed=71429,rnd,model_l,model_r,expect_l,expect_r,timeout_count;
motion_control_and_mode_fsm #(.CLK_HZ(TEST_CLK_HZ),.FEEDBACK_TIMEOUT_MS(8),.STOP_TIMEOUT_MS(7)) dut(.*);
task check;input condition;input [511:0] msg;
begin checks=checks+1;if(condition!==1'b1)begin $display("FAIL %0s t=%0t mode=%0d phase=%0d fault=%0d L=%0d R=%0d",msg,$time,mode,phase,fault_code,target_l,target_r);$fatal(1);end end endtask
task cycle;begin @(posedge clk);#1;end endtask
task cycles;input integer n;integer q;begin for(q=0;q<n;q=q+1)cycle;end endtask
task reset_all;begin
@(negedge clk);rst_n=0;enable=0;external_fault=0;clear_fault=0;stop_request=0;cmd_valid=0;
speed_valid_l=1;speed_valid_r=1;yaw_valid=1;speed_l=0;speed_r=0;position_l=0;position_r=0;yaw_mdeg=0;
cmd_mode=2;cmd_end=0;cmd_reverse=0;cmd_speed_l=7;cmd_speed_r=7;cmd_goal=10;cmd_timeout_ms=0;
cmd_speed_limit=100;cmd_accel_step=3;cmd_decel_step=2;
cycles(2);@(negedge clk);rst_n=1;cycles(2);check(mode==0 && !drive_enable,"reset");
@(negedge clk);enable=1;cycles(2);check(cmd_ready,"idle ready");end endtask
task send;begin @(negedge clk);cmd_valid=1;cycle;@(negedge clk);cmd_valid=0;end endtask
task wait_idle;begin timeout_count=0;while(busy && timeout_count<100*P)begin cycle;timeout_count=timeout_count+1;end check(mode==1 && !busy,"idle finish");end endtask
task stop_wait;begin @(negedge clk);stop_request=1;cycle;@(negedge clk);stop_request=0;wait_idle;end endtask
task wait_fault;input [7:0] code;begin timeout_count=0;while(fault_code==0 && timeout_count<30*P)begin cycle;timeout_count=timeout_count+1;end check(fault_code==code && !drive_enable && mode==9,"fault code");end endtask
always @(posedge clk) begin
check_decel=rst_n && enable && !external_fault && fault_code==0 && dut.detected_fault==0 && dut.tick && phase==3 && !(stop_request && !dut.cancelled);
if(check_decel) begin
 decel_expected_l=(target_l<0)?-target_l:target_l;
 decel_expected_r=(target_r<0)?-target_r:target_r;
 if(decel_expected_l<=dut.decel)decel_expected_l=0;else decel_expected_l=decel_expected_l-dut.decel;
 if(decel_expected_r<=dut.decel)decel_expected_r=0;else decel_expected_r=decel_expected_r-dut.decel;
 if(dut.neg_l)decel_expected_l=-decel_expected_l;
 if(dut.neg_r)decel_expected_r=-decel_expected_r;
end
if(!rst_n || (cmd_valid && cmd_ready && dut.valid_command)) terminal_seen=0;
#2;
if(check_decel)check(target_l==decel_expected_l && target_r==decel_expected_r,"deceleration reference");
if(rst_n)begin
if(done || aborted)begin check(!terminal_seen,"one terminal notification per action");terminal_seen=1;end
if(done)done_count=done_count+1;if(aborted)abort_count=abort_count+1;if(rejected)reject_count=reject_count+1;
if(dut.tick_count==0 || done || aborted || rejected) begin check(!(done && aborted),"exclusive events");check(!done || (!busy && mode==1),"done idle");check(drive_enable || (target_l==0 && target_r==0),"safe outputs");end end end
initial begin
for(i=2;i<=7;i=i+1)for(j=0;j<2;j=j+1)begin
reset_all;cmd_mode=i;cmd_reverse=j;if(i==4)cmd_speed_r=11;if(i==5)cmd_speed_l=11;
if(i==7)begin cmd_end=2;cmd_goal=1000;cmd_timeout_ms=200;end
send;check(busy,"mode accepted");model_l=0;model_r=0;
repeat(5)begin cycles(P);model_l=model_l+3;if(model_l>cmd_speed_l)model_l=cmd_speed_l;model_r=model_r+3;if(model_r>cmd_speed_r)model_r=cmd_speed_r;
expect_l=(i==3 || (i==7 && j==1) || (i==6 && j==0))?-model_l:model_l;
expect_r=(i==3 || (i==7 && j==1) || (i==6 && j==1))?-model_r:model_r;
check(target_l==expect_l && target_r==expect_r,"reference slope signs");end
send;check(rejected,"busy rejection");stop_wait;check(aborted && !done,"stop abort");end
for(i=0;i<24;i=i+1)begin
reset_all;rnd=$random(seed)&32'h7fffffff;cmd_speed_l=1+rnd%99;cmd_speed_r=cmd_speed_l;
rnd=$random(seed)&32'h7fffffff;cmd_accel_step=1+rnd%43;if(i==23)cmd_accel_step=32'hffffffff;
send;model_l=0;repeat(5)begin cycles(P);if(cmd_accel_step>=cmd_speed_l-model_l)model_l=cmd_speed_l;else model_l=model_l+cmd_accel_step;check(target_l==model_l,"random reference");end
model_l=cmd_speed_l;k=(cmd_accel_step>=cmd_speed_l)?1:((cmd_speed_l+cmd_accel_step-1)/cmd_accel_step);cmd_speed_l=1;cmd_speed_r=1;cmd_accel_step=0;cmd_decel_step=0;cycles(k*P);check(target_l==model_l,"config snapshot");stop_wait;end
reset_all;cmd_end=1;cmd_goal=4;cmd_timeout_ms=50;send;cycles(3*P);check(mode!=8,"before time");cycles(P+2);check(mode==8,"time boundary");wait_idle;check(done,"time done");
reset_all;cmd_mode=7;cmd_end=2;cmd_goal=20;cmd_timeout_ms=80;position_l=32'h7ffffff8;position_r=32'h7ffffff8;send;cycles(4*P);
@(negedge clk);position_l=32'h8000000c;position_r=32'h7fffffe0;cycles(5*P);check(target_l==0 && target_r==7,"independent distance wrong direction");
@(negedge clk);position_l=32'h7ffffff8;position_r=32'h8000000d;wait_idle;check(done,"wrap distance latched hit");
reset_all;cmd_mode=7;cmd_end=2;cmd_reverse=1;cmd_goal=20;cmd_timeout_ms=80;position_l=32'h80000008;position_r=32'h80000008;send;cycles(3*P);
@(negedge clk);position_l=32'h7ffffff4;position_r=32'h7ffffff3;wait_idle;check(done,"negative position wrap");
for(i=0;i<2;i=i+1)begin reset_all;cmd_mode=6;cmd_end=3;cmd_reverse=i;cmd_goal=30;cmd_timeout_ms=80;yaw_mdeg=i?32'h80000008:32'h7ffffff8;send;cycles(3*P);
@(negedge clk);yaw_mdeg=i?32'h80000020:32'h7fffffe0;cycles(P);check(mode==6,"wrong yaw direction");
@(negedge clk);yaw_mdeg=i?32'h7fffffea:32'h80000016;wait_idle;check(done,"yaw modulo wrap");end
reset_all;cmd_mode=7;cmd_end=2;cmd_goal=10;cmd_timeout_ms=80;send;@(negedge clk);position_l=10;position_r=10;stop_request=1;cycle;@(negedge clk);stop_request=0;wait_idle;check(aborted && !done,"stop beats completion");
reset_all;send;cycles(3*P);speed_l=5;speed_r=5;@(negedge clk);stop_request=1;cycle;@(negedge clk);stop_request=0;while(phase!=4)cycle;
@(negedge clk);speed_valid_l=0;speed_valid_r=0;cycles(P);check(busy,"fresh samples required");
@(negedge clk);speed_valid_l=1;speed_valid_r=1;speed_l=0;speed_r=0;cycles(2);@(negedge clk);speed_l=5;cycle;@(negedge clk);speed_l=0;cycles(2);check(busy,"still consecutive");wait_idle;
for(i=0;i<12;i=i+1)begin reset_all;case(i)
0:cmd_mode=15;1:cmd_speed_l=0;2:cmd_speed_limit=0;3:cmd_speed_limit=32'h80000000;4:cmd_accel_step=0;5:cmd_decel_step=0;6:cmd_speed_l=101;
7:cmd_mode=4;8:begin cmd_end=3;cmd_timeout_ms=10;end
9:cmd_mode=7;10:cmd_end=1;11:begin cmd_end=1;cmd_timeout_ms=10;cmd_goal=32'h80000000;end endcase
send;check(rejected && !busy && mode==1,"invalid command");end
reset_all;speed_valid_l=0;speed_valid_r=0;cycles(9*P);send;check(rejected,"stale start");
reset_all;send;speed_valid_l=0;wait_fault(2);
reset_all;cmd_mode=6;cmd_end=3;cmd_timeout_ms=50;send;yaw_valid=0;wait_fault(3);
reset_all;cmd_end=1;cmd_goal=20;cmd_timeout_ms=2;send;wait_fault(4);
reset_all;send;speed_l=5;speed_r=5;@(negedge clk);stop_request=1;cycle;@(negedge clk);stop_request=0;wait_fault(5);
reset_all;send;cycles(3*P);@(negedge clk);external_fault=1;#1;check(target_l==0 && !drive_enable,"immediate fault gate");cycle;check(fault_code==1,"fault latch");
@(negedge clk);external_fault=0;clear_fault=1;cycle;check(mode==9,"clear enabled denied");@(negedge clk);enable=0;cycle;check(mode==0 && fault_code==0,"clear disabled");
@(negedge clk);clear_fault=0;enable=1;cycles(2);check(mode==1 && !busy,"clear no restart");send;cycles(2*P);@(negedge clk);enable=0;#1;check(target_l==0,"immediate disable");cycle;check(aborted,"disable event");
@(negedge clk);enable=1;cycles(2);check(!busy,"enable idle");send;cycles(P);@(negedge clk);rst_n=0;#1;check(mode==0 && target_l==0,"running async reset");
reset_all;@(negedge clk);external_fault=1;cmd_valid=1;cycle;check(fault_code==1 && !busy,"fault command priority");
reset_all;send;cycles(3*P);speed_l=5;speed_r=5;
@(negedge clk);stop_request=1;cycle;@(negedge clk);stop_request=0;while(phase!=4)cycle;
@(negedge clk);speed_l=0;speed_r=0;cycles(3);
@(negedge clk);speed_l=5;cycle;check(busy,"new movement cancels pending still exit");
@(negedge clk);speed_l=0;wait_idle;
reset_all;cmd_mode=6;cmd_end=3;cmd_timeout_ms=30;yaw_valid=0;cycles(9*P);send;check(rejected,"stale yaw command rejected");
reset_all;cmd_end=1;cmd_goal=0;cmd_timeout_ms=20;send;check(rejected,"zero duration rejected");
reset_all;send;cycles(P);@(negedge clk);external_fault=1;clear_fault=1;enable=0;cycle;check(fault_code==1,"fault beats disable and clear");
reset_all;send;force dut.phase=3'd7;cycle;release dut.phase;cycle;check(fault_code==6,"illegal phase");
$display("PASS motion checks=%0d done=%0d aborted=%0d rejected=%0d CLK_HZ=%0d",checks,done_count,abort_count,reject_count,TEST_CLK_HZ);completed=1;$finish;
end
initial begin #2000000000;$display("FAIL global timeout");$fatal(1);end
endmodule
module tb_motion_invalid_parameter;
motion_control_and_mode_fsm #(.CLK_HZ(999)) dut(
.clk(1'b0),.rst_n(1'b0),.enable(1'b0),.stop_request(1'b0),.external_fault(1'b0),.clear_fault(1'b0),
.cmd_valid(1'b0),.cmd_ready(),.cmd_mode(4'd2),.cmd_end(2'd0),.cmd_reverse(1'b0),
.cmd_speed_l(32'd1),.cmd_speed_r(32'd1),.cmd_goal(32'd1),.cmd_timeout_ms(32'd1),
.cmd_speed_limit(32'd1),.cmd_accel_step(32'd1),.cmd_decel_step(32'd1),
.position_l(32'sd0),.position_r(32'sd0),.speed_l(32'sd0),.speed_r(32'sd0),.yaw_mdeg(32'sd0),
.speed_valid_l(1'b0),.speed_valid_r(1'b0),.yaw_valid(1'b0),.target_l(),.target_r(),
.drive_enable(),.controller_clear(),.busy(),.mode(),.phase(),.done(),.aborted(),.rejected(),.reason(),.fault_code());
initial begin #100;$display("FAIL invalid parameter accepted");$finish;end
endmodule




