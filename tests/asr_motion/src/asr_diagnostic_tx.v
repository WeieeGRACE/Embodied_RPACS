`timescale 1ns/1ps
// Passive diagnostic mode only: atomic signed hexadecimal count/yaw snapshot.
module asr_diagnostic_tx #(parameter integer CLK_HZ=50000000,BAUD=115200)(
 input wire clk,rst_n,input wire [31:0] position_l,position_r,yaw,
 output wire tx);
 reg [95:0] snapshot;
 reg [31:0] timer;
 reg [5:0] index;
 reg active;
 reg [7:0] data;
 reg [3:0] nibble;
 wire ready;
 always @* begin
 nibble=0;data=32;
 if(index>=2 && index<=9)nibble=snapshot[95-((index-2)*4) -:4];
 if(index>=13 && index<=20)nibble=snapshot[63-((index-13)*4) -:4];
 if(index>=24 && index<=31)nibble=snapshot[31-((index-24)*4) -:4];
 case(index)
 0:data="L";11:data="R";22:data="Y";
 1,12,23:data="=";
 10,21:data=" ";32:data=13;33:data=10;
 default:data=nibble<10 ? 8'd48+{4'd0,nibble}:8'd55+{4'd0,nibble};
 endcase
 end
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(
 .clk(clk),.rst_n(rst_n),.data(data),.valid(active),.ready(ready),.tx(tx),.done());
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin snapshot<=0;timer<=0;index<=0;active<=0;end
 else begin
 if(timer==CLK_HZ/5-1)begin
 timer<=0;
 if(!active)begin snapshot<={position_l,position_r,yaw};index<=0;active<=1;end
 end else timer<=timer+1'b1;
 if(active && ready)begin
 if(index==33)begin active<=0;index<=0;end else index<=index+1'b1;
 end
 end
endmodule
