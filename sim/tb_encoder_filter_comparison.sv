`timescale 1ns/1ps
// Identical legal AB input; isolate acceptance filtering from motor control.
module tb_encoder_filter_comparison;
 reg clk=0, rst_n=0;
 reg [1:0] ab=0;
 always #10 clk=~clk;
 wire [31:0] errors_fast, errors_filtered;
 dual_ab_encoder_channel #(.FILTER_CYCLES(1)) fast (
  .clk(clk),.rst_n(rst_n),.a(ab[1]),.b(ab[0]),.clear_fault(1'b0),
  .illegal_transition(errors_fast));
 dual_ab_encoder_channel #(.FILTER_CYCLES(50)) filtered (
  .clk(clk),.rst_n(rst_n),.a(ab[1]),.b(ab[0]),.clear_fault(1'b0),
  .illegal_transition(errors_filtered));
 task phase(input [1:0] value, input integer cycles);
  begin @(negedge clk); ab=value; repeat(cycles) @(negedge clk); end
 endtask
 initial begin
  repeat(5) @(negedge clk); rst_n=1;
  phase(0,100); phase(2,100); phase(3,100); phase(1,100); phase(0,100);
  if(errors_fast!==0 || errors_filtered!==0) $fatal(1,"Slow legal sequence failed");
  // A legal but brief intermediate phase is rejected only by FILTER=50.
  phase(2,10); phase(3,100);
  if(errors_fast!==0 || errors_filtered!==1) $fatal(1,"Filter comparison failed");
  $display("PASS: legal slow phases: both zero; 200ns intermediate phase: FILTER1=0 FILTER50=1");
  $finish;
 end
endmodule
