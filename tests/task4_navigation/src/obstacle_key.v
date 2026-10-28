`timescale 1ns/1ps
module obstacle_key #(parameter integer DEBOUNCE_CYCLES=1000000)(
 input wire clk,rst_n,key_n,output reg press);
 (* ASYNC_REG="TRUE" *) reg [1:0] sync;
 reg stable; reg [31:0] count;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin sync<=3;stable<=1;count<=0;press<=0;end
 else begin
 sync<={sync[0],key_n};press<=0;
 if(sync[1]==stable)count<=0;
 else if(count>=DEBOUNCE_CYCLES-1)begin
 stable<=sync[1];count<=0;if(!sync[1])press<=1;
 end else count<=count+1'b1;
 end
endmodule
