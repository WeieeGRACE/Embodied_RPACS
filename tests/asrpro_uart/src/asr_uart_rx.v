`timescale 1ns/1ps
// Synchronised 8N1 receiver. A framing error waits for a high line before rearming.
module asr_uart_rx #(parameter integer CLK_HZ=50000000, BAUD=115200)(
 input wire clk,rst_n,rx,input wire [31:0] tick,
 output reg [7:0] data,output reg valid,frame_error,
 output reg [31:0] start_tick);
 localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
 (* ASYNC_REG="TRUE" *) reg [1:0] rx_sync;
 reg [2:0] state; reg [2:0] bit_index; reg [31:0] timer;
 reg [7:0] shift;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) rx_sync<=2'b11; else rx_sync<={rx_sync[0],rx};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=0; timer<=0; bit_index<=0; shift<=0;
   data<=0; valid<=0; frame_error<=0; start_tick<=0; end
  else begin
   valid<=0; frame_error<=0;
   case(state)
    0: if(!rx_sync[1]) begin state<=1; timer<=0; start_tick<=tick; end
    1: if(timer==BIT_CYCLES/2-1) begin
        timer<=0;
        if(!rx_sync[1]) begin state<=2; bit_index<=0; end else state<=0;
       end else timer<=timer+1'b1;
    2: if(timer==BIT_CYCLES-1) begin
        timer<=0; shift[bit_index]<=rx_sync[1];
        if(bit_index==7) state<=3; else bit_index<=bit_index+1'b1;
       end else timer<=timer+1'b1;
    3: if(timer==BIT_CYCLES-1) begin
        timer<=0;
        if(rx_sync[1]) begin data<=shift; valid<=1; state<=0; end
        else begin frame_error<=1; state<=4; end
       end else timer<=timer+1'b1;
    4: if(rx_sync[1]) state<=0;
    default: state<=0;
   endcase
  end
 end
endmodule
