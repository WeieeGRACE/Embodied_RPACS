`timescale 1ns/1ps
// Q16 micrometres: low 16 fractional bits survive every encoder transition.
module asr_distance #(parameter integer UM_PER_REV=207345,COUNTS_PER_REV=1456)(
 input wire clk,rst_n,input wire signed [1:0] delta,
 output wire signed [31:0] position_um);
 localparam [47:0] STEP=(64'd65536*UM_PER_REV+COUNTS_PER_REV/2)/COUNTS_PER_REV;
 reg [47:0] position_q16;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)position_q16<=0;
 else if(delta==1)position_q16<=position_q16+STEP;
 else if(delta== -1)position_q16<=position_q16-STEP;
 assign position_um=position_q16[47:16];
endmodule

module asr_yaw_unwrap #(parameter integer YAW_SIGN=1)(
 input wire clk,rst_n,update_valid,input wire signed [31:0] wrapped,
 output reg signed [31:0] unwrapped,output reg valid);
 reg seen;
 reg signed [31:0] previous;
 wire signed [31:0] difference=wrapped-previous;
 wire signed [31:0] step=(difference>180000) ? difference-360000:
 (difference< -180000) ? difference+360000:difference;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin previous<=0;unwrapped<=0;seen<=0;valid<=0;end
 else begin
 valid<=0;
 if(update_valid)begin
 previous<=wrapped;seen<=1;valid<=1;
 if(seen)unwrapped<=unwrapped+(YAW_SIGN<0 ? -step:step);
 end
 end
endmodule
