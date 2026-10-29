`timescale 1ns/1ps
// Minimal obstacle-demo menu: MANUAL, SETTINGS, OBSTACLE AVOID.
// The DISPLAY diagnostics group and the next-item preview row were removed to
// free FPGA logic for later features; the obstacle panel itself still shows
// state, gear, three ranges and the fault diagnostic bytes.
module system_menu(
 input wire clk,rst_n,input wire [2:0] pressed,
 input wire [1:0] gear,input wire show_avoid,show_root,avoid_mode,
 output reg [2:0] group_id,output reg [3:0] item,
 output reg page,output reg voice_enable,
 output reg voice_candidate,output reg [1:0] gear_candidate,
 output reg command_valid,output reg [7:0] command,
 output reg gear_commit,output reg [1:0] gear_value,
 output reg stop_request);
 reg [3:0] last_item;
 always @* case(group_id)
 0:last_item=2;2:last_item=4;4:last_item=1;
 5:last_item=0;default:last_item=0;
 endcase
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 group_id<=0;item<=0;page<=0;voice_enable<=0;
 voice_candidate<=0;gear_candidate<=1;command_valid<=0;
 command<=0;gear_commit<=0;gear_value<=1;stop_request<=0;
 end else begin
 command_valid<=0;gear_commit<=0;stop_request<=0;
 if(pressed[0])begin
 stop_request<=1;
 if(avoid_mode || group_id==5)begin group_id<=0;item<=2;page<=0;end
 else if(page)page<=0;
 else if(group_id!=0)begin item<=group_id==2 ? 4'd0:4'd1;group_id<=0;end
 end else if(pressed[1])begin
 if(!page)item<=item==last_item ? 0:item+4'd1;
 else if(group_id==4 && item==0)voice_candidate<=!voice_candidate;
 else if(group_id==4 && item==1)gear_candidate<=gear_candidate==2 ? 0:gear_candidate+2'd1;
 end else if(pressed[2])begin
 if(!page)begin
 if(group_id==0)begin
 group_id<=item==0 ? 4'd2:(item==1 ? 4'd4:4'd5);
 item<=0;if(item==2)page<=1;end
 else begin page<=1;voice_candidate<=voice_enable;gear_candidate<=gear;end
 end else case(group_id)
 5:begin command_valid<=1;command<=8'h16;end
 2:begin command_valid<=1;command<=item==4 ? 8'h02:8'h10+item;end
 4:if(item==0)begin
 voice_enable<=voice_candidate;
 if(voice_enable && !voice_candidate)stop_request<=1;
 end else begin gear_commit<=1;gear_value<=gear_candidate;end
 default:begin end
 endcase
 end
 if(show_avoid && !pressed[0])begin group_id<=5;item<=0;page<=1;end
 if(show_root)begin group_id<=0;item<=2;page<=0;end
 end
endmodule
