`timescale 1ns/1ps
module tb_live_speed;
 reg clk=0,rst_n=0,cmd_valid=0,stop_request=0,runtime_speed_valid=1;always #5 clk=~clk;
 reg [31:0] runtime_speed=1000,runtime_speed_cap=1500;reg signed [31:0] position_l=0,position_r=0;
 wire cmd_ready,busy,done,aborted,rejected;wire signed [31:0] target_l,target_r;wire [7:0] fault_code;
 reg completed=0;integer last_l=0,checks=0;
 motion_control_and_mode_fsm #(.CLK_HZ(10000),.STILL_THRESHOLD(0)) dut(
 .clk(clk),.rst_n(rst_n),.enable(1'b1),.stop_request(stop_request),.external_fault(1'b0),.clear_fault(1'b0),
 .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.runtime_speed_cap(runtime_speed_cap),.runtime_speed_valid(runtime_speed_valid),.runtime_speed(runtime_speed),
 .cmd_mode(4'd7),.cmd_end(2'd2),.cmd_reverse(1'b0),.cmd_speed_l(32'd1000),.cmd_speed_r(32'd1000),
 .cmd_goal(32'd200000),.cmd_timeout_ms(32'd10000),.cmd_speed_limit(32'd1500),.cmd_accel_step(32'd100),.cmd_decel_step(32'd250),
 .position_l(position_l),.position_r(position_r),.speed_l(32'sd0),.speed_r(32'sd0),.yaw_mdeg(32'sd0),
 .speed_valid_l(1'b1),.speed_valid_r(1'b1),.yaw_valid(1'b1),.target_l(target_l),.target_r(target_r),
 .busy(busy),.done(done),.aborted(aborted),.rejected(rejected),.fault_code(fault_code),.drive_enable(),.controller_clear(),.mode(),.phase(),.accepted(),.reason(),.command_errors(),.feedback_ages());
 always @(negedge clk)if(rst_n)begin
 if(target_l-last_l>100 || last_l-target_l>250)$fatal(1,"speed target jumped %0d -> %0d",last_l,target_l);
 if(target_l!=target_r || rejected || fault_code)$fatal(1,"invalid live update");last_l=target_l;
 end
 task cycles(input integer n);repeat(n)begin @(posedge clk);#1;end endtask
 task check(input bit v,input string msg);begin checks++;if(!v)$fatal(1,"%s",msg);end endtask
 initial begin
 cycles(3);rst_n=1;wait(cmd_ready);cmd_valid=1;cycles(1);cmd_valid=0;cycles(150);check(target_l==1000,"initial mid");
 position_l=70000;position_r=70000;runtime_speed=1500;cycles(80);check(target_l==1500,"live high");
 runtime_speed=500;cycles(80);check(target_l==500,"live low");
 runtime_speed=1500;runtime_speed_cap=500;cycles(80);check(target_l==500,"safety cap priority");
 runtime_speed_cap=1500;cycles(150);check(target_l==1500,"cap release resumes gear");
 check(dut.origin_l==0 && dut.origin_r==0 && dut.goal==200000,"update changed goal/origin");
 check(busy && !done,"speed update completed prematurely");
 position_l=200000;position_r=200000;wait(done);check(!busy,"completion after distance");
 cycles(20);runtime_speed=500;cycles(100);check(!busy && target_l==0,"idle speed command started movement");
 $display("PASS live_speed %0d checks, ramp, cap, distance continuity, no idle start",checks);completed=1;$finish;
 end
 initial begin #100000;$fatal(1,"watchdog");end
endmodule
