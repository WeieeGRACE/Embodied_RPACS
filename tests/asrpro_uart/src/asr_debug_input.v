`timescale 1ns/1ps
// Strict ASCII STATE XX + LF or CRLF, bounded line length, 100 ms line timeout.
module asr_debug_input #(parameter integer TIMEOUT_CYCLES=5000000)(
 input wire clk,rst_n,input wire [31:0] tick,input wire [7:0] data,
 input wire valid,frame_error,output reg inject_valid,
 output reg [7:0] inject_status,output reg [31:0] error_count);
 reg [63:0] line; reg [3:0] length; reg cr; reg [31:0] first_tick;
 function hex_ok;input [7:0] c;
  begin hex_ok=(c>=48 && c<=57)||(c>=65 && c<=70)||(c>=97 && c<=102);end
 endfunction
 function [3:0] hex_value;input [7:0] c;reg [7:0] offset;
  begin
   if(c<=57) offset=c-8'd48;else if(c<=70) offset=c-8'd55;else offset=c-8'd87;
   hex_value=offset[3:0];
  end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin line<=0;length<=0;cr<=0;first_tick<=0;
   inject_valid<=0;inject_status<=0;error_count<=0;end
  else begin
   inject_valid<=0;
   if(frame_error || (length!=0 && tick-first_tick>=TIMEOUT_CYCLES)) begin
    length<=0;cr<=0;error_count<=error_count+1'b1;
   end else if(valid) begin
    if(data==10) begin
     if(length==8 && line[63:16]==48'h535441544520 &&
        hex_ok(line[15:8]) && hex_ok(line[7:0])) begin
      inject_status<={hex_value(line[15:8]),hex_value(line[7:0])};inject_valid<=1;
     end else error_count<=error_count+1'b1;
     length<=0;cr<=0;
    end else if(data==13 && length==8 && !cr) cr<=1;
    else begin
     if(length==0) first_tick<=tick;
     if(length<8 && !cr) begin line<={line[55:0],data};length<=length+1'b1;end
     else length<=9; // discard the whole overlength line, never accept a suffix
    end
   end
  end
 end
endmodule
