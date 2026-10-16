`timescale 1ns/1ps
// Valid 8N1 A5 frames refresh a reset-only latched watchdog.
module uart_heartbeat_watchdog #(parameter integer BIT_CYCLES=434,TIMEOUT_CYCLES=25000000)(
 input wire clk,rst_n,rx,output reg seen,expired);
 (* ASYNC_REG="TRUE" *) reg [1:0] sync_rx;
 reg [1:0] state;reg [31:0] bit_timer,age;reg [2:0] bit_index;reg [7:0] data;
 wire heartbeat=(state==3 && bit_timer==BIT_CYCLES-1 && sync_rx[1] && data==8'hA5);
 always @(posedge clk or negedge rst_n)
 if(!rst_n) sync_rx<=3;else sync_rx<={sync_rx[0],rx};
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin state<=0;bit_timer<=0;bit_index<=0;data<=0;end
 else case(state)
 0:if(!sync_rx[1]) begin state<=1;bit_timer<=0;end
 1:if(bit_timer==BIT_CYCLES/2-1) begin bit_timer<=0;if(!sync_rx[1]) begin state<=2;bit_index<=0;end else state<=0;end else bit_timer<=bit_timer+1;
 2:if(bit_timer==BIT_CYCLES-1) begin data[bit_index]<=sync_rx[1];bit_timer<=0;if(bit_index==7) state<=3;else bit_index<=bit_index+1;end else bit_timer<=bit_timer+1;
 3:if(bit_timer==BIT_CYCLES-1) begin state<=0;bit_timer<=0;end else bit_timer<=bit_timer+1;
 endcase
 end
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin seen<=0;expired<=0;age<=0;end
 else if(!expired) begin
 if(heartbeat) begin seen<=1;age<=0;end
 else if(seen) begin if(age==TIMEOUT_CYCLES-1) expired<=1;else age<=age+1;end
 end
 end
endmodule
