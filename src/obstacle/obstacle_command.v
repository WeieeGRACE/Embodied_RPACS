`timescale 1ns/1ps
// One atomic command; cancellation always wins over capture and submission.
// Payload: mode[4], end[2], reverse[1], then seven32-bit parameters.
module obstacle_command_buffer(
 input wire clk,rst_n,cancel,request,motion_ready,motion_busy,
 input wire [230:0] proposal,input wire [132:0] context_in,
 output wire ready,submit,busy,output reg pending,
 output reg [230:0] command,output reg [132:0] context_out);
 reg occupied,settling;
 assign ready=!occupied && !motion_busy && !cancel;
 assign submit=pending && motion_ready && !motion_busy && !cancel;
 assign busy=occupied || motion_busy;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin pending<=0;occupied<=0;settling<=0;command<=0;context_out<=0;end
 else if(cancel)begin pending<=0;occupied<=0;settling<=0;end
 else if(request && ready)begin
 command<=proposal;context_out<=context_in;pending<=1;occupied<=1;
 end else if(submit)begin pending<=0;settling<=1;end
 else if(settling)settling<=0;
 else if(occupied && !pending && !motion_busy)occupied<=0;
endmodule

// One completion per rotation. Targets are captured with the atomic command.
module obstacle_turn_trace #(parameter [31:0] BUILD_ID=32'h26102509)(
 input wire clk,rst_n,submit,done,cancel,input wire [230:0] command,
 input wire [132:0] context_data,input wire signed [31:0] yaw,
 input wire [31:0] milliseconds,output reg event_valid,output reg [223:0] event_data);
 reg [31:0] sequence_number,saved_sequence,saved_state;
 reg signed [31:0] saved_target;
 reg active;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin sequence_number<=0;saved_sequence<=0;saved_state<=0;saved_target<=0;
 active<=0;event_valid<=0;event_data<=0;end
 else begin
 event_valid<=0;
 if(submit)begin
 sequence_number<=sequence_number+1'b1;
 if(command[230:227]==6)begin
 active<=1;saved_sequence<=sequence_number+1'b1;
 saved_state<={27'd0,context_data[132:128]};saved_target<=context_data[95:64];
 end
 end
 if(active && done && !cancel)begin
 active<=0;event_valid<=1;
 event_data<={saved_sequence,saved_state,saved_target,yaw,(yaw-saved_target),milliseconds,BUILD_ID};
 end
 if(cancel)active<=0;
 end
endmodule

// Snapshot the actual validation edge, rather than the following FAULT state.
module obstacle_command_trace #(parameter [31:0] BUILD_ID=32'h26102509)(
 input wire clk,rst_n,submit,accepted,rejected,input wire [3:0] reason,input wire [230:0] command,
 input wire [132:0] context_data,input wire [95:0] feedback_ages,
 input wire [15:0] errors,input wire [31:0] milliseconds,
 output reg event_valid,output reg [735:0] event_data,
 output reg reject_valid,output reg [735:0] reject_data);
 reg [31:0] sequence_number;
 reg awaiting_ack;reg [735:0] pending_record;
 wire [735:0] record_data={sequence_number+32'd1,
 27'd0,context_data[132:128],28'd0,command[230:227],30'd0,command[226:225],31'd0,command[224],
 command[223:0],context_data[127:96],context_data[95:64],
 feedback_ages[31:0],feedback_ages[63:32],feedback_ages[95:64],
 16'd0,errors,(errors==0 ? 32'd1:32'd2),BUILD_ID,context_data[63:0],milliseconds};
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin sequence_number<=0;event_valid<=0;event_data<=0;reject_valid<=0;reject_data<=0;awaiting_ack<=0;pending_record<=0;end
 else begin
 event_valid<=0;
 if(awaiting_ack)begin
 awaiting_ack<=0;
 if(accepted && !rejected)begin event_valid<=1;event_data<=pending_record;event_data[159-:32]<={20'd0,reason,8'd1};end
 else if(!reject_valid)begin
 reject_valid<=1;reject_data<=pending_record;
 reject_data[159-:32]<={20'd0,reason,(rejected ? 8'd2:8'd3)};
 end
 end
 if(submit)begin
 sequence_number<=sequence_number+1'b1;
 pending_record<=record_data;awaiting_ack<=1;
 end
 end
endmodule
