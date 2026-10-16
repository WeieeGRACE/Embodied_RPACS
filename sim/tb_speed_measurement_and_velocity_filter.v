`timescale 1ns/1ps
module tb_speed_measurement_and_velocity_filter;
  reg clk=0, rst_n=0; reg signed [15:0] delta_count;
  wire signed [31:0] speed_raw, speed_filtered, position;
  wire speed_valid, direction, zero_speed;
  speed_measurement_and_velocity_filter #(.WINDOW_CYCLES(4),.FILTER_LENGTH(4)) dut(
    .clk(clk),.rst_n(rst_n),.delta_count(delta_count),.speed_raw(speed_raw),
    .speed_filtered(speed_filtered),.position(position),.speed_valid(speed_valid),
    .direction(direction),.zero_speed(zero_speed));
  always #5 clk=~clk;
  integer checks=0;
  task check; input cond; input [255:0] msg; begin checks=checks+1; if(!cond) begin $display("FAIL: %s t=%0t raw=%0d filt=%0d pos=%0d",msg,$time,speed_raw,speed_filtered,position); $fatal(1); end end endtask
  task cycle; input signed [15:0] d; begin @(negedge clk); delta_count=d; @(posedge clk); #1; end endtask
  initial begin
    delta_count=0; repeat(2) @(posedge clk); rst_n=1;
    repeat(4) cycle(2); check(speed_valid && speed_raw==8 && speed_filtered==8 && position==8 && !direction,"positive window");
    repeat(4) cycle(-1); check(speed_valid && speed_raw==-4 && speed_filtered==2 && position==4 && direction,"negative window and average");
    repeat(4) cycle(0); check(speed_valid && speed_raw==0 && speed_filtered==1 && position==4 && zero_speed,"zero window");
    repeat(4) cycle(-2); check(speed_valid && speed_raw==-8 && speed_filtered==-1 && position==-4,"reverse window");
    @(posedge clk); #1; check(!speed_valid,"valid pulse width");
    $display("PASS: %0d checks",checks); $finish;
  end
endmodule
