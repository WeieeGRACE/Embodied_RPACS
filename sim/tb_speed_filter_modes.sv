`timescale 1ns/1ps
// Standalone speed-filter mode test. The filter now registers a completed
// window one cycle after the boundary (prefetch history pipeline, 2026-10-26),
// so each 4-cycle constant-delta window is driven for four samples and its
// result is checked on the following (5th) clock edge.
module tb_speed_filter_modes;
 reg clk=0,rst=0,completed=0;
 always #10 clk=~clk;
 reg signed [15:0] delta=0;
 wire signed [31:0] raw,avg,padded;
 wire valid;
 speed_measurement_and_velocity_filter #(.WINDOW_CYCLES(4),.FILTER_LENGTH(4)) normal(
 .clk(clk),.rst_n(rst),.delta_count(delta),.speed_raw(raw),.speed_filtered(avg),.speed_valid(valid),.position(),.direction(),.zero_speed());
 speed_measurement_and_velocity_filter #(.WINDOW_CYCLES(4),.FILTER_LENGTH(4),.ZERO_PAD_WARMUP(1)) pad(
 .clk(clk),.rst_n(rst),.delta_count(delta),.speed_filtered(padded),.speed_raw(),.speed_valid(),.position(),.direction(),.zero_speed());
 // One constant-delta window of four samples. `first_window` waits the four
 // sample edges; later calls only need three because the previous check edge
 // already delivered the first sample of this window. `next_delta` is applied
 // before the boundary+1 edge so the next window stays pure.
 task automatic window(input signed [15:0] d,input signed [15:0] next_delta,
  input integer expected_raw,expected_avg,expected_pad,input bit first_window);
 begin
  delta=d;
  if(first_window) repeat(4) @(posedge clk);
  else repeat(3) @(posedge clk);
  delta=next_delta;
  @(posedge clk);#1;
  if(!valid || raw!=expected_raw || avg!=expected_avg || padded!=expected_pad)
   $fatal(1,"filter raw=%0d avg=%0d pad=%0d",raw,avg,padded);
  @(negedge clk);
 end endtask
 initial begin
  repeat(5) @(negedge clk);rst=1;
  window(2,-1,8,8,2,1);
  window(-1,0,-4,2,1,0);
  window(0,-2,0,1,1,0);
  window(-2,1,-8,-1,-1,0);
  window(1,2,4,-2,-2,0);
  rst=0;repeat(3) @(negedge clk);rst=1;
  window(-2,1,-8,-8,-2,1);
  window(1,0,4,-2,-1,0);
  window(0,2,0,-1,-1,0);
  window(2,2,8,1,1,0);
  completed=1;$display("PASS signed filter warm-up, zero padding, truncation and history replacement");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
