`timescale 1ns/1ps
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
 task window(input signed [15:0] d,input integer expected_raw,expected_avg,expected_pad);
 begin
  delta=d;repeat(4) begin @(posedge clk);#1;end
  if(!valid || raw!=expected_raw || avg!=expected_avg || padded!=expected_pad)
   $fatal(1,"filter raw=%0d avg=%0d pad=%0d",raw,avg,padded);
  @(negedge clk);
 end endtask
 initial begin
  repeat(5) @(negedge clk);rst=1;
  window(2,8,8,2);window(-1,-4,2,1);window(0,0,1,1);
  window(-2,-8,-1,-1);window(1,4,-2,-2);
  rst=0;repeat(3) @(negedge clk);rst=1;
  window(-2,-8,-8,-2);window(1,4,-2,-1);window(0,0,-1,-1);
  window(2,8,1,1);
  completed=1;$display("PASS signed filter warm-up, zero padding, truncation and history replacement");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
