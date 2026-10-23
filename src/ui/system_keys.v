`timescale 1ns/1ps
// Reset only from power-on: a held KEY0 must not become a second press
// when a software reset returns the UI to the root directory.
module system_keys #(parameter integer DEBOUNCE_CYCLES=500000)(
 input wire clk,por_n,input wire [2:0] keys_n,
 output reg [2:0] pressed,output reg [2:0] released_n);
 (* ASYNC_REG="TRUE" *) reg [2:0] meta,sync;
 reg [31:0] count[0:2];
 integer i;
 always @(posedge clk or negedge por_n)
 if(!por_n)begin meta<=7;sync<=7;released_n<=7;pressed<=0;
 for(i=0;i<3;i=i+1)count[i]<=0;end
 else begin
 meta<=keys_n;sync<=meta;pressed<=0;
 for(i=0;i<3;i=i+1)begin
 if(sync[i]==released_n[i])count[i]<=0;
 else if(count[i]>=DEBOUNCE_CYCLES-1)begin
 count[i]<=0;released_n[i]<=sync[i];pressed[i]<=!sync[i];
 end else count[i]<=count[i]+1'b1;
 end
 end
endmodule
