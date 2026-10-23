`timescale 1ns/1ps
module asr_key #(parameter integer CYCLES=1000000)(
 input wire clk,rst_n,key_n,output reg press);
 (* ASYNC_REG="TRUE" *) reg [1:0] sync_key;
 reg stable;reg [31:0] count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sync_key<=3;stable<=1;count<=0;press<=0;end
  else begin
   sync_key<={sync_key[0],key_n};press<=0;
   if(sync_key[1]==stable) count<=0;
   else if(count==CYCLES-1) begin
    stable<=sync_key[1];count<=0;if(!sync_key[1]) press<=1;
   end else count<=count+1'b1;
  end
 end
endmodule
