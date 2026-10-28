`timescale 1ns/1ps
// Atomic command response/status pair plus independent state transitions.
// Saturation latches a fault; stop parsing never depends on this queue.
 module asr_packet_tx #(parameter integer CLK_HZ=50000000,BAUD=115200)(
 input wire clk,rst_n,pair_valid,input wire [7:0] response,pair_status,
 input wire event_valid,input wire [7:0] event_status,
 input wire aux_valid,input wire [7:0] aux_type,aux_code,
 output wire tx,output reg overflow);
 // Depth 16 covers the worst single-cycle burst of four entries many times
 // over; the UART drains one entry per six transmitted bytes.
 reg [15:0] fifo[0:15];
 reg [3:0] wr,rd;
 reg [4:0] used;
 reg [2:0] index;
 reg active,fault_pending;
 reg [15:0] packet;
 wire ready;
 reg [7:0] data;
 wire pop=!active && used!=0;
 wire av=aux_valid===1'b1;
 wire [2:0] requested=(pair_valid ? 3'd2:3'd0)+(event_valid ? 3'd1:3'd0)+(av ? 3'd1:3'd0);
 wire room=({1'b0,used}+requested)<=16;
 wire inject=fault_pending && !pair_valid && !event_valid && !av && used<16;
 wire [2:0] added=room ? requested:3'd0;
 always @* case(index)
 0:data=8'haa;1:data=8'h55;2:data=packet[15:8];3:data=packet[7:0];
 4:data=packet[15:8]+packet[7:0];default:data=8'h0d;
 endcase
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) sender(
 .clk(clk),.rst_n(rst_n),.data(data),.valid(active),.ready(ready),.tx(tx),.done());
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n)begin wr<=0;rd<=0;used<=0;index<=0;active<=0;packet<=0;overflow<=0;fault_pending<=0;end
 else begin
 if(requested!=0 && !room)begin overflow<=1;fault_pending<=1;end
 if(room)begin
 if(pair_valid)begin fifo[wr]<={8'h02,response};fifo[(wr+4'd1)&4'hf]<={8'h03,pair_status};end
 if(event_valid)fifo[(wr+(pair_valid ? 4'd2:4'd0))&4'hf]<={8'h03,event_status};
 if(av)fifo[(wr+(pair_valid ? 4'd2:4'd0)+(event_valid ? 4'd1:4'd0))&4'hf]<={aux_type,aux_code};
 wr<=wr+{1'b0,added};
 end
 if(inject)begin fifo[wr]<=16'h0306;wr<=wr+4'd1;fault_pending<=0;end
 used<=used+added+(inject ? 5'd1:5'd0)-(pop ? 5'd1:5'd0);
 if(pop)begin packet<=fifo[rd];rd<=rd+4'd1;active<=1;index<=0;end
 if(active && ready)begin
 if(index==5)begin active<=0;index<=0;end else index<=index+1'b1;
 end
 end
 end
endmodule
