`timescale 1ns/1ps
// Owns mode lifetime and command arbitration. The route never owns manual moves.
module system_dispatch #(parameter integer CLK_HZ=50000000,AUDIO_TIMEOUT_MS=10000)(
 input wire clk,rst_n,voice_enable,cv,input wire [7:0] command,
 input wire ack_valid,input wire [7:0] ack,
 input wire menu_valid,input wire [7:0] menu_command,
 input wire stop_gate,stop_request,ready,fault,busy,cmd_ready,done,
 input wire tof_ready,post_fresh,input wire [4:0] route_state,
 output reg avoid_mode,auto_run,waiting,manual_active,
 output reg [7:0] manual_action,output wire manual_submit,
 output wire stopping,route_cancel,output reg route_press,refresh,
 output reg show_avoid,show_root,output reg [7:0] response,
 output reg aux_valid,output reg [7:0] aux_type,aux_code,
 output wire [7:0] mode_status);
 reg pending_manual,ack_seen,exit_pending,stop_hold;
 reg [31:0] ticks,wait_ms,heartbeat;
 reg [4:0] old_route;
 reg [5:0] events;
 // Configuration-initialized sequence survives soft reset, preventing reuse on K0 reset.
 reg [7:0] sequence_id=0;
 reg [7:0] token;
 wire ms=ticks==CLK_HZ/1000-1;
 wire menu_move=menu_valid && ((menu_command>=8'h10 && menu_command<=8'h13) || (menu_command>=8'h20 && menu_command<=8'h27));
 wire voice_move=cv && command>=8'h10 && command<=8'h13;
 wire enter_cmd=cv && command==8'h16;
 wire resume_cmd=cv && avoid_mode && command==8'h10;
 wire toggle_cmd=menu_valid && menu_command==8'h16;
 wire exit_cmd=(cv && command==8'h17) || stop_gate || stop_request;
 wire pause_cmd=(cv && command==2) || (menu_valid && menu_command==2) || (toggle_cmd && auto_run);
 wire stop_event=exit_cmd || pause_cmd || (avoid_mode && !voice_enable) || fault;
 wire manual_owned=manual_active || pending_manual;
 wire accepted_voice_move=voice_move && voice_enable && ready && !fault && !manual_owned && !menu_valid && !exit_cmd && !pause_cmd;
 wire accepted_manual=accepted_voice_move && !(avoid_mode && command==8'h10);
 wire takeover=accepted_manual && (auto_run || waiting);
 wire start_request=(enter_cmd || resume_cmd || toggle_cmd) && !auto_run && !waiting && !manual_owned && !busy &&
                    voice_enable && ready && tof_ready && !fault && !exit_cmd && !pause_cmd;
 wire manual_request=(menu_move && !busy && !manual_owned && !avoid_mode && ready && !fault) || accepted_manual;
 assign route_cancel=stop_event || takeover;
 assign stopping=stop_event || takeover || stop_hold;
 assign manual_submit=pending_manual && cmd_ready && !busy && !stopping && ready && !fault;
 assign mode_status=waiting ? 8'h21:auto_run ? ((route_state==1 || route_state==2) ? 8'h22:8'h23):8'h20;
 always @* begin
 response=0;
 if(!(voice_enable || command==2 || command==3 || command==8'h17) || command==1)response=2;
 else if((voice_move || enter_cmd || command==8'h14 || command==8'h15) && (!ready || fault))response=2;
 else if((enter_cmd || resume_cmd) && !tof_ready)response=2;
 else if((voice_move || enter_cmd) && (manual_owned || (busy && !auto_run) || menu_valid || exit_cmd || pause_cmd))response=1;
 end
 always @(posedge clk)if(start_request)sequence_id<=sequence_id+1'b1;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 avoid_mode<=0;auto_run<=0;waiting<=0;manual_active<=0;manual_action<=0;
 pending_manual<=0;ack_seen<=0;exit_pending<=0;stop_hold<=0;token<=0;
 ticks<=0;wait_ms<=0;heartbeat<=0;old_route<=0;events<=0;
 route_press<=0;refresh<=0;show_avoid<=0;show_root<=0;aux_valid<=0;aux_type<=0;aux_code<=0;
 end else begin
 route_press<=0;refresh<=0;show_avoid<=0;show_root<=0;aux_valid<=0;
 if(ms)ticks<=0;else ticks<=ticks+1'b1;
 old_route<=route_state;
 if(stopping && busy)stop_hold<=1;else if(!busy)stop_hold<=0;
 if(done || (!busy && !pending_manual))manual_active<=0;
 // One serialized auxiliary frame per clock; capture wins over consumption.
 if(events[0])begin aux_valid<=1;aux_type<=4;aux_code<=token;events[0]<=0;end
 else if(events[1])begin aux_valid<=1;aux_type<=6;aux_code<={7'd0,avoid_mode};events[1]<=0;end
 else if(events[2])begin aux_valid<=1;aux_type<=7;aux_code<=1;events[2]<=0;end
 else if(events[3])begin aux_valid<=1;aux_type<=7;aux_code<=2;events[3]<=0;end
 else if(events[4])begin aux_valid<=1;aux_type<=7;aux_code<=3;events[4]<=0;end
 if(avoid_mode && ms)begin
 if(heartbeat==999)begin heartbeat<=0;events[1]<=1;end else heartbeat<=heartbeat+1'b1;
 end else if(!avoid_mode)heartbeat<=0;
 if(auto_run && old_route!=3 && route_state==3)events[2]<=1;
 if(auto_run && old_route==20 && route_state==1)events[3]<=1;
 if(auto_run && (route_state==17 || route_state==18))auto_run<=0;
 // The spoken mode name is also the direct panel-navigation shortcut. Entry
 // may still be refused below (voice off, fault, stale ToF, or busy motion),
 // but the panel must remain visible so its readiness prompt is actionable.
 if(cv && command==8'h16)show_avoid<=1;
 if(start_request)begin
 avoid_mode<=1;waiting<=1;wait_ms<=0;ack_seen<=0;token<=sequence_id+1'b1;
 events[0]<=1;events[1]<=1;show_avoid<=1;refresh<=1;
 end
 if(waiting)begin
 if(ms && wait_ms<AUDIO_TIMEOUT_MS)wait_ms<=wait_ms+1'b1;
 if(ack_valid && ack==token)ack_seen<=1;
 if((ack_seen || (ack_valid && ack==token)) && ready && tof_ready && post_fresh && !busy)begin
 waiting<=0;ack_seen<=0;auto_run<=1;route_press<=1;
 end else if(wait_ms>=AUDIO_TIMEOUT_MS || !tof_ready)begin waiting<=0;ack_seen<=0;end
 end
 if(manual_request)begin
 auto_run<=0;waiting<=0;ack_seen<=0;events[0]<=0;
 pending_manual<=1;manual_action<=menu_move ? menu_command:command;
 end
 if(manual_submit)begin pending_manual<=0;manual_active<=1;end
 if(exit_pending && !busy)begin exit_pending<=0;show_root<=1;events[4]<=1;end
 if(stop_event)begin
 auto_run<=0;waiting<=0;ack_seen<=0;pending_manual<=0;events[0]<=0;
 if(exit_cmd && avoid_mode)begin
 avoid_mode<=0;events[1]<=1;exit_pending<=1;
 end
 end
 end
endmodule
