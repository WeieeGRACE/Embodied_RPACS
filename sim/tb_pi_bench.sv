`timescale 1ns/1ps
module tb_pi_bench;
 reg clk=0,rst=0,tick=0,enable=0,completed=0;
 always #10 clk=~clk;
 reg signed [15:0] feedback=0;
 wire signed [15:0] control;
 wire sat;
 speed_pi_pid_channel dut(.clk(clk),.rst_n(rst),.tick(tick),.enable(enable),
 .target(16'sd10),.feedback(feedback),.kp(16'sd32),.ki(16'sd1),.kd(16'sd0),
 .gain_shift(5'd0),.limit(16'd625),.control(control),.sat(sat),.active());
 task update(input signed [15:0] f,input integer expected);
 begin
  @(negedge clk);feedback=f;tick=1;
  @(posedge clk);#1;if(control!=expected) $fatal(1,"PI control=%0d expected=%0d",control,expected);
  @(negedge clk);tick=0;
 end endtask
 initial begin
  repeat(3) @(negedge clk);rst=1;enable=1;
  update(5,165);update(5,170);update(5,175);
  update(10,15);update(10,15); // Integral retained at zero error.
  update(0,345);update(0,355);
  repeat(27) update(0,control<615 ? control+10:625);
  update(0,625);if(!sat) $fatal(1,"positive clamp");
  repeat(10) update(0,625); // No hidden accumulating integral.
  update(11,272); // Proportional -352, integral -1: promptly leaves saturation.
  @(negedge clk);enable=0;@(posedge clk);#1;
  if(control!=0 || sat || dut.e1!=0 || dut.e2!=0) $fatal(1,"disable clear");
  @(negedge clk);enable=1;update(20,-330);
  repeat(30) update(20,control> -615 ? control-10:-625);
  repeat(10) update(20,-625);if(!sat) $fatal(1,"negative clamp");
  update(9,-272);
  completed=1;$display("PASS PI accumulation, zero error hold, both clamps, antiwindup and disable clear");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
