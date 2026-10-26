`timescale 1ns/1ps
// 8N1 receiver followed by a sliding six-byte protocol window.
module asr_uart_rx #(
 parameter integer CLK_HZ=50000000, BAUD=115200,
 parameter integer FRAME_CYCLES=CLK_HZ/50
)(input wire clk,rst_n,rx,output reg command_valid,output reg [7:0] command,output reg ack_valid,output reg [7:0] ack);
 localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
 (* ASYNC_REG="TRUE" *) reg [1:0] sync_rx;
 reg [1:0] state;
 reg [31:0] timer,age;
 reg [2:0] bit_index,count;
 reg [7:0] data;
 reg byte_valid;
 reg [47:0] window_data;
 wire [47:0] candidate={window_data[39:0],data};
 wire known=(candidate[23:16]==8'h02 || candidate[23:16]==8'h03 ||
 candidate[23:16]==8'h01 || (candidate[23:16]>=8'h10 && candidate[23:16]<=8'h17));
 always @(posedge clk or negedge rst_n)
 if(!rst_n) sync_rx<=3;else sync_rx<={sync_rx[0],rx};
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin state<=0;timer<=0;bit_index<=0;data<=0;byte_valid<=0;end
 else begin
 byte_valid<=0;
 case(state)
 0:if(!sync_rx[1])begin state<=1;timer<=0;end
 1:if(timer==BIT_CYCLES/2-1)begin timer<=0;state<=sync_rx[1] ? 0:2;bit_index<=0;end else timer<=timer+1'b1;
 2:if(timer==BIT_CYCLES-1)begin
 timer<=0;data[bit_index]<=sync_rx[1];
 if(bit_index==7)state<=3;else bit_index<=bit_index+1'b1;
 end else timer<=timer+1'b1;
 3:if(timer==BIT_CYCLES-1)begin timer<=0;state<=0;byte_valid<=sync_rx[1];end else timer<=timer+1'b1;
 endcase
 end
 end
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n)begin window_data<=0;age<=0;count<=0;command_valid<=0;command<=0;ack_valid<=0;ack<=0;end
 else begin
 command_valid<=0;ack_valid<=0;
 if(count!=0)begin
 if(age>=FRAME_CYCLES-1)begin count<=0;window_data<=0;age<=0;end
 else age<=age+1'b1;
 end
 if(byte_valid)begin
 if(count==0 || age>=FRAME_CYCLES-1)begin
 window_data<={40'd0,data};count<=(data==8'haa)?1:0;age<=0;
 end else begin
 window_data<=candidate;
 if(count<6)count<=count+1'b1;
 if(count>=5 && candidate[47:32]==16'haa55 && ((candidate[31:24]==1 && known) || candidate[31:24]==5) &&
 candidate[15:8]==((candidate[31:24]+candidate[23:16])&8'hff) && candidate[7:0]==8'h0d)begin
 if(candidate[31:24]==5)begin ack<=candidate[23:16];ack_valid<=1;end
 else begin command<=candidate[23:16];command_valid<=1;end count<=0;window_data<=0;age<=0;
 end
 end
 end
 end
 end
endmodule
