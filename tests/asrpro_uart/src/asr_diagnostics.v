`timescale 1ns/1ps
// Snapshot FIFO: debug UART congestion cannot stall the ASR command path.
module asr_diagnostics #(parameter integer CLK_HZ=50000000,BAUD=115200,DEPTH=32)(
 input wire clk,rst_n,event_valid,input wire [7:0] command,response,status,
 input wire [1:0] gear,input wire [31:0] good,bad,timeouts,overflows,debug_errors,
 output wire tx,output reg [31:0] log_overflows);
 localparam integer LENGTH=113;
 reg [223:0] fifo[0:DEPTH-1]; integer wr,rd,used;
 reg [LENGTH*8-1:0] line; reg active;integer index;
 wire ready;wire pop=!active && used!=0;
 wire push=event_valid && (used<DEPTH || pop);
 wire [223:0] item=fifo[rd];
 function [7:0] digit;input [3:0] v;
  begin digit=(v<10)?8'd48+v:8'd55+v;end
 endfunction
 function [15:0] hex8;input [7:0] v;
  begin hex8={digit(v[7:4]),digit(v[3:0])};end
 endfunction
 function [63:0] hex32;input [31:0] v;integer i;
  begin for(i=0;i<8;i=i+1) hex32[(7-i)*8+:8]=digit(v[(7-i)*4+:4]);end
 endfunction
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(
  .clk(clk),.rst_n(rst_n),.data(line[LENGTH*8-1-:8]),.valid(active),
  .ready(ready),.tx(tx),.done());
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin wr<=0;rd<=0;used<=0;line<=0;active<=0;index<=0;log_overflows<=0;end
  else begin
   case({push,pop}) 2'b10:used<=used+1;2'b01:used<=used-1;default:;endcase
   if(event_valid && !push) log_overflows<=log_overflows+1'b1;
   if(push) begin
    fifo[wr]<={command,response,status,{6'b0,gear},good,bad,timeouts,overflows,debug_errors,log_overflows};
    wr<=(wr==DEPTH-1)?0:wr+1;
   end
   if(pop) begin
    line<={"TEST CMD=",hex8(item[223:216])," ACK=",hex8(item[215:208]),
     " STATE=",hex8(item[207:200])," GEAR=",digit(item[195:192]),
     " RX=",hex32(item[191:160])," BAD=",hex32(item[159:128]),
     " TMO=",hex32(item[127:96])," OVF=",hex32(item[95:64]),
     " DBG=",hex32(item[63:32])," LOG=",hex32(item[31:0]),8'h0D,8'h0A};
    rd<=(rd==DEPTH-1)?0:rd+1;active<=1;index<=0;
   end
   if(active && ready) begin
    line<=line<<8;
    if(index==LENGTH-1) active<=0;else index<=index+1;
   end
  end
 end
endmodule
