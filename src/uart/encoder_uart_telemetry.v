`timescale 1ns/1ps
// Snapshot -> decimal conversion (one digit/clock) -> ready/valid byte stream.
module encoder_uart_telemetry #(parameter integer CLK_HZ=50000000,BAUD=115200,
 REPORT_CYCLES=5000000)(input wire clk,rst_n,
 input wire signed [31:0] position_l,position_r,
 input wire direction_l,direction_r,input wire [31:0] illegal_l,illegal_r,
 input wire fault_l,fault_r,output wire tx);
 localparam IDLE=0,LABEL=1,NUMBER=2,DIGITS=3,FLAG=4,CR=5,LF=6,SEND=7;
 reg [3:0] state; reg [31:0] interval_count;
 reg [31:0] values[0:3]; reg [3:0] flags;
 reg [2:0] field; reg [1:0] label_index;
 reg [7:0] line[0:127]; reg [7:0] length,index;
 reg [7:0] digits[0:9]; reg [3:0] digit_count;
 reg [31:0] magnitude;
 wire ready,done;
 wire valid=(state==SEND);
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(
 .clk(clk),.rst_n(rst_n),.data(line[index]),.valid(valid),.ready(ready),.tx(tx),.done(done));
 // Field order: L R DL DR IL IR FL FR.
 function [7:0] label_char;
 input [2:0] f; input [1:0] n;
 begin
  case(f)
   0: label_char=n==0 ? "L" : "=";
   1: label_char=n==0 ? "R" : "=";
   2,3: label_char=n==0 ? "D" : n==1 ? (f==2 ? "L":"R") : "=";
   4,5: label_char=n==0 ? "I" : n==1 ? (f==4 ? "L":"R") : "=";
   default: label_char=n==0 ? "F" : n==1 ? (f==6 ? "L":"R") : "=";
  endcase
 end endfunction
 wire [31:0] value=(field==0) ? values[0] : (field==1) ? values[1] : (field==4) ? values[2] : values[3];
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=IDLE; interval_count<=0; field<=0; label_index<=0;
   length<=0; index<=0; magnitude<=0; digit_count<=0; flags<=0;
   values[0]<=0; values[1]<=0; values[2]<=0; values[3]<=0;
  end else begin
   if(interval_count==REPORT_CYCLES-1) interval_count<=0;
   else interval_count<=interval_count+1'b1;
   case(state)
    IDLE: if(interval_count==REPORT_CYCLES-1) begin
     values[0]<=position_l; values[1]<=position_r; values[2]<=illegal_l; values[3]<=illegal_r;
     flags<={fault_r,fault_l,direction_r,direction_l};
     length<=0; index<=0; field<=0; label_index<=0; state<=LABEL;
    end
    LABEL: begin
     line[length]<=label_char(field,label_index); length<=length+1'b1;
     if(label_index==(field<2 ? 1:2)) begin
      label_index<=0;
      if(field==0 || field==1 || field==4 || field==5) begin
       magnitude<=(field<2 && value[31]) ? (~value+32'd1):value;
       digit_count<=0; state<=NUMBER;
      end else state<=FLAG;
     end else label_index<=label_index+1'b1;
    end
    NUMBER: begin
     if(digit_count==0 && field<2 && value[31]) begin line[length]<= "-"; length<=length+1'b1; end
     digits[digit_count]<=8'd48+(magnitude%10); digit_count<=digit_count+1'b1;
     magnitude<=magnitude/10;
     if(magnitude<10) state<=DIGITS;
    end
    DIGITS: begin
     line[length]<=digits[digit_count-1]; length<=length+1'b1;
     digit_count<=digit_count-1'b1;
     if(digit_count==1) begin line[length+1]<= " "; length<=length+2;
      field<=field+1'b1; state<=LABEL; end
    end
    FLAG: begin
     line[length]<=8'd48+((field==2) ? flags[0] : (field==3) ? flags[1] : (field==6) ? flags[2]:flags[3]);
     if(field==7) begin length<=length+1'b1; state<=CR; end
     else begin line[length+1]<= " "; length<=length+2; field<=field+1'b1; state<=LABEL; end
    end
    CR: begin line[length]<=13; length<=length+1'b1; state<=LF; end
    LF: begin line[length]<=10; length<=length+1'b1; index<=0; state<=SEND; end
    SEND: if(ready) begin
     if(index==length-1) state<=IDLE;
     else index<=index+1'b1;
    end
    default: state<=IDLE;
   endcase
  end
 end
endmodule
