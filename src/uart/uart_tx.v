`timescale 1ns/1ps
// One byte is accepted on valid && ready. No generated clocks.
module uart_tx #(parameter integer CLK_HZ=50000000, BAUD=115200)(
 input wire clk,rst_n,input wire [7:0] data,input wire valid,
 output wire ready,output wire tx,output reg done);
 localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
 reg busy; reg [9:0] frame; reg [3:0] bit_index;
 reg [31:0] cycles;
 assign ready=!busy;
 assign tx=busy ? frame[bit_index] : 1'b1;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin busy<=0; frame<=10'h3ff; bit_index<=0; cycles<=0; done<=0; end
  else begin
   done<=0;
   if(!busy) begin
    if(valid) begin frame<={1'b1,data,1'b0}; busy<=1; bit_index<=0; cycles<=0; end
   end else if(cycles==BIT_CYCLES-1) begin
    cycles<=0;
    if(bit_index==9) begin busy<=0; done<=1; end
    else bit_index<=bit_index+1'b1;
   end else cycles<=cycles+1'b1;
  end
 end
endmodule
