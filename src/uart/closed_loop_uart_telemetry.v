`timescale 1ns/1ps
// 20-word atomic snapshot. Fields 0..9 signed, others unsigned.
module closed_loop_uart_telemetry #(parameter integer REPORT_CYCLES=5000000,FIELD_COUNT=20)(
 input wire clk,rst_n,input wire [FIELD_COUNT*32-1:0] values,output wire tx);
 localparam IDLE=0,LABEL=1,NUMBER=2,DIGITS=3,SEP=4,CR=5,LF=6,SEND=7;
 reg [3:0] state;reg [31:0] interval_count;
 reg [FIELD_COUNT*32-1:0] snapshot;reg [4:0] field;reg [2:0] label_index;
 reg [7:0] line[0:511];reg [8:0] length,index;
 reg [7:0] digits[0:9];reg [3:0] digit_count;reg [31:0] magnitude;
 wire [31:0] value=snapshot[field*32 +:32];
 wire ready,done;
 uart_tx sender(.clk(clk),.rst_n(rst_n),.data(line[index]),.valid(state==SEND),.ready(ready),.tx(tx),.done(done));
 function [47:0] label_word;
 input [4:0] f;
 begin case(f)
   0: label_word="L=";
   1: label_word="R=";
   2: label_word="TL=";
   3: label_word="TR=";
   4: label_word="VL=";
   5: label_word="VR=";
   6: label_word="SFL=";
   7: label_word="SFR=";
   8: label_word="PL=";
   9: label_word="PR=";
   10: label_word="SL=";
   11: label_word="SR=";
   12: label_word="EN=";
   13: label_word="ST=";
   14: label_word="IL=";
   15: label_word="IR=";
   16: label_word="FL=";
   17: label_word="FR=";
   18: label_word="FAULT=";
   19: label_word="FC=";
   20: label_word="HB=";
   21: label_word="WD=";
   default:label_word="?=";
 endcase end endfunction
 reg [47:0] label_shift;
 reg [2:0] label_length;
 always @* begin
  label_length=(field==18) ? 6 : (field==6 || field==7) ? 4 : (field<2) ? 2 : 3;
  label_shift=label_word(field) >> (8*(label_length-1-label_index));
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=IDLE;interval_count<=0;snapshot<=0;field<=0;label_index<=0;length<=0;index<=0;digit_count<=0;magnitude<=0;end
  else begin
   if(interval_count==REPORT_CYCLES-1) interval_count<=0;else interval_count<=interval_count+1;
   case(state)
    IDLE:if(interval_count==REPORT_CYCLES-1 && ready) begin snapshot<=values;field<=0;label_index<=0;length<=0;state<=LABEL;end
    LABEL:begin
     line[length]<=label_shift[7:0];length<=length+1;
     if(label_index==label_length-1) begin
      label_index<=0;magnitude<=(field<10 && value[31]) ? (~value+32'd1):value;digit_count<=0;state<=NUMBER;
     end else label_index<=label_index+1;
    end
    NUMBER:begin
     if(digit_count==0 && field<10 && value[31]) begin line[length]<= "-";length<=length+1;end
     digits[digit_count]<=8'd48+(magnitude%10);digit_count<=digit_count+1;magnitude<=magnitude/10;
     if(magnitude<10) state<=DIGITS;
    end
    DIGITS:begin line[length]<=digits[digit_count-1];length<=length+1;digit_count<=digit_count-1;if(digit_count==1) state<=SEP;end
    SEP:if(field==FIELD_COUNT-1) state<=CR;else begin line[length]<= " ";length<=length+1;field<=field+1;state<=LABEL;end
    CR:begin line[length]<=13;length<=length+1;state<=LF;end
    LF:begin line[length]<=10;length<=length+1;index<=0;state<=SEND;end
    SEND:if(ready) begin if(index==length-1) state<=IDLE;else index<=index+1;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
