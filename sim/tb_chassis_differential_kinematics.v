`timescale 1ns/1ps
module tb_chassis_differential_kinematics;
 reg clk=0,rst=1,update=0,enable=0,inv_l=0,inv_r=0;
 reg signed [31:0] v=0,w=0; reg [31:0] base=240,maxv=1000,step=0;
 wire signed [31:0] l,r; wire valid;
 chassis_differential_kinematics dut(clk,rst,update,enable,v,w,base,maxv,step,inv_l,inv_r,l,r,valid);
 always #5 clk=~clk;
 task apply; input signed [31:0] tv,tw; input signed [31:0] el,er;
   begin v=tv; w=tw; update=1; @(posedge clk); #1; update=0;
     if (!valid || l!==el || r!==er) $fatal(1,"v=%0d w=%0d got %0d/%0d expected %0d/%0d",tv,tw,l,r,el,er);
   end
 endtask
 initial begin
   #12; @(posedge clk); rst=0; enable=1;
   apply(500,0,500,500);
   apply(0,1000,-120,120);
   maxv=100; apply(500,0,100,100);
   step=30; maxv=1000; apply(500,0,130,130); apply(500,0,160,160);
   enable=0; apply(0,0,0,0);
   inv_l=1; enable=1; step=0; apply(200,0,-200,200);
   $display("KINEMATICS TEST PASS"); $finish;
 end
endmodule
