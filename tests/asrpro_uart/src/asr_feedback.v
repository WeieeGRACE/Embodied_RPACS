`timescale 1ns/1ps
// One FIFO entry is an atomic response+state pair or an unsolicited state.
// Drop NEW events on full; command handling never waits for this transmitter.
module asr_feedback #(parameter integer CLK_HZ=50000000,BAUD=115200,
 parameter integer DEPTH=16)(
 input wire clk,rst_n,event_valid,event_pair,
 input wire [7:0] response,status,output wire tx,
 output reg [31:0] overflow_count);
 reg [16:0] fifo[0:DEPTH-1];
 integer wr,rd,used; reg active,is_state; reg [16:0] item;
 reg [2:0] index; wire ready; reg [7:0] byte_data;
 wire pop=(!active && used!=0);
 wire push=event_valid && (used<DEPTH || pop);
 wire [7:0] kind=is_state?8'h03:8'h02;
 wire [7:0] code=is_state?item[7:0]:item[15:8];
 always @* case(index)
  0:byte_data=8'hAA; 1:byte_data=8'h55; 2:byte_data=kind;
  3:byte_data=code; 4:byte_data=kind+code; default:byte_data=8'h0D;
 endcase
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(
  .clk(clk),.rst_n(rst_n),.data(byte_data),.valid(active),.ready(ready),.tx(tx),.done());
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin wr<=0;rd<=0;used<=0;active<=0;is_state<=0;item<=0;index<=0;overflow_count<=0;end
  else begin
   case({push,pop}) 2'b10:used<=used+1; 2'b01:used<=used-1; default:; endcase
   if(event_valid && !push) overflow_count<=overflow_count+1'b1;
   if(push) begin fifo[wr]<={event_pair,response,status};wr<=(wr==DEPTH-1)?0:wr+1;end
   if(pop) begin item<=fifo[rd];rd<=(rd==DEPTH-1)?0:rd+1;
    active<=1;index<=0;is_state<=!fifo[rd][16];end
   if(active && ready) begin
    if(index==5) begin
     index<=0;
     if(!is_state && item[16]) is_state<=1; else active<=0;
    end else index<=index+1'b1;
   end
  end
 end
endmodule
