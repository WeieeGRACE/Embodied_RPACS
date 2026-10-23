`timescale 1ns/1ps
// Independent motion sequencer. All inputs are synchronous to clk.
// Position/yaw must be free-running modulo-2^32 counters, never reset mid-action.
module motion_control_and_mode_fsm #(
 parameter integer CLK_HZ=50000000, TICK_HZ=1000,
 parameter integer FEEDBACK_TIMEOUT_MS=100, STOP_TIMEOUT_MS=1000,
 parameter integer STILL_SAMPLES=3, STILL_THRESHOLD=1,
 parameter integer ALLOW_SINGLE_TIMED=0
)(
 input wire clk,rst_n,enable,stop_request,external_fault,clear_fault,
 input wire cmd_valid,
 output wire cmd_ready,
 input wire [3:0] cmd_mode,
 input wire [1:0] cmd_end, // 0 continuous, 1 time, 2 distance, 3 angle
 input wire cmd_reverse, // distance: backward; rotate: clockwise
 input wire [31:0] cmd_speed_l,cmd_speed_r,cmd_goal,cmd_timeout_ms,
 input wire [31:0] cmd_speed_limit,cmd_accel_step,cmd_decel_step,
 input wire signed [31:0] position_l,position_r,speed_l,speed_r,yaw_mdeg,
 input wire speed_valid_l,speed_valid_r,yaw_valid,
 output wire signed [31:0] target_l,target_r,
 output wire drive_enable,controller_clear,busy,
 output reg [3:0] mode,
 output reg [2:0] phase,
 output reg done,aborted,rejected,
 output reg [3:0] reason,
 output reg [7:0] fault_code
);
 localparam DISABLED=0,IDLE=1,FORWARD=2,BACKWARD=3,TURN_LEFT=4,
 TURN_RIGHT=5,ROTATE=6,DISTANCE_MOVE=7,BRAKE=8,FAULT=9;
 localparam NONE=0,ACCEL=1,RUNNING=2,DECEL=3,WAIT_STILL=4;
 localparam integer PERIOD=CLK_HZ/TICK_HZ;
 reg [31:0] tick_count;
 wire tick=(tick_count==PERIOD-1);
 reg [31:0] mag_l,mag_r,vel_l,vel_r,accel,decel,goal,timeout_ms;
 reg neg_l,neg_r,reverse;
 reg [1:0] end_kind;
 reg [31:0] origin_l,origin_r,origin_yaw;
 reg [31:0] elapsed,stop_elapsed,age_l,age_r,age_y;
 reg seen_l,seen_r,seen_y,hit_l,hit_r,cancelled;
 reg [31:0] still_l,still_r;
 wire active=(phase!=NONE);
 wire safe=rst_n && enable && !external_fault && fault_code==0 && mode!=FAULT;
 assign drive_enable=safe && active;
 assign target_l=drive_enable ? (neg_l ? -$signed(mag_l):$signed(mag_l)):32'sd0;
 assign target_r=drive_enable ? (neg_r ? -$signed(mag_r):$signed(mag_r)):32'sd0;
 assign busy=active;
 assign controller_clear=!drive_enable || phase==WAIT_STILL;
 assign cmd_ready=safe && mode==IDLE && !stop_request;
 wire fresh_l=speed_valid_l || (seen_l && age_l<FEEDBACK_TIMEOUT_MS);
 wire fresh_r=speed_valid_r || (seen_r && age_r<FEEDBACK_TIMEOUT_MS);
 wire fresh_y=yaw_valid || (seen_y && age_y<FEEDBACK_TIMEOUT_MS);
 wire [31:0] dl=reverse ? origin_l-position_l:position_l-origin_l;
 wire [31:0] dr=reverse ? origin_r-position_r:position_r-origin_r;
 wire [31:0] dy=reverse ? origin_yaw-yaw_mdeg:yaw_mdeg-origin_yaw;
 wire reach_l=hit_l || (!dl[31] && dl>=goal);
 wire reach_r=hit_r || (!dr[31] && dr>=goal);
 wire reached=(end_kind==1 && elapsed>=goal) ||
              (end_kind==2 && reach_l && reach_r) ||
              (end_kind==3 && !dy[31] && dy>=goal);
 wire valid_mode=(cmd_mode>=FORWARD && cmd_mode<=DISTANCE_MOVE);
 wire single_timed=ALLOW_SINGLE_TIMED!=0 && cmd_end==1 &&
 (cmd_mode==FORWARD || cmd_mode==BACKWARD) &&
 ((cmd_speed_l==0) != (cmd_speed_r==0));
 wire valid_command=valid_mode && ((cmd_speed_l!=0 && cmd_speed_r!=0) || single_timed) &&
 !cmd_speed_limit[31] && cmd_speed_limit!=0 &&
 cmd_speed_l<=cmd_speed_limit && cmd_speed_r<=cmd_speed_limit &&
 cmd_accel_step!=0 && cmd_decel_step!=0 &&
 ((cmd_mode==FORWARD || cmd_mode==BACKWARD || cmd_mode==ROTATE || cmd_mode==DISTANCE_MOVE) ? (cmd_speed_l==cmd_speed_r || single_timed):1'b1) &&
 (cmd_mode!=TURN_LEFT || cmd_speed_r>cmd_speed_l) &&
 (cmd_mode!=TURN_RIGHT || cmd_speed_l>cmd_speed_r) &&
 (cmd_mode!=DISTANCE_MOVE || cmd_end==2) &&
 (cmd_end!=2 || cmd_mode==DISTANCE_MOVE) &&
 (cmd_end!=3 || cmd_mode==ROTATE) &&
 (cmd_end==0 || (cmd_goal!=0 && !cmd_goal[31] && cmd_timeout_ms!=0)) &&
 fresh_l && fresh_r && (cmd_end!=3 || fresh_y);
 function [31:0] approach;
 input [31:0] current,target,step;
 reg [32:0] sum;
 begin
 if(current<target) begin sum={1'b0,current}+{1'b0,step}; approach=(sum>={1'b0,target})?target:sum[31:0]; end
 else approach=(current-target<=step)?target:current-step;
 end
 endfunction
 function is_still;
 input signed [31:0] speed;
 begin is_still=(speed>=-STILL_THRESHOLD && speed<=STILL_THRESHOLD); end
 endfunction
 // synthesis translate_off
 initial begin
 if(CLK_HZ<1 || TICK_HZ!=1000 || CLK_HZ%TICK_HZ!=0 || PERIOD<1 ||
 FEEDBACK_TIMEOUT_MS<1 || STOP_TIMEOUT_MS<1 || STILL_SAMPLES<1 || STILL_THRESHOLD<0) begin
 $display("ERROR: invalid motion FSM parameters"); $fatal(1); end
 end
 // synthesis translate_on
 wire bad_state=(mode>FAULT || phase>WAIT_STILL ||
 (active && !(mode>=FORWARD && mode<=BRAKE)) ||
 (!active && !(mode==DISABLED || mode==IDLE || mode==FAULT)));
 reg [7:0] detected_fault;
 always @* begin
 detected_fault=0;
 if(external_fault) detected_fault=1;
 else if(bad_state) detected_fault=6;
 else if(active && (!fresh_l || !fresh_r)) detected_fault=2;
 else if(active && end_kind==3 && !fresh_y) detected_fault=3;
 else if(active && timeout_ms!=0 && elapsed>=timeout_ms) detected_fault=4;
 else if(phase==WAIT_STILL && stop_elapsed>=STOP_TIMEOUT_MS) detected_fault=5;
 end
 always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
 tick_count<=0; mode<=DISABLED; phase<=NONE;
 mag_l<=0; mag_r<=0; vel_l<=0; vel_r<=0; accel<=0; decel<=0;
 goal<=0; timeout_ms<=0; neg_l<=0; neg_r<=0; reverse<=0; end_kind<=0;
 origin_l<=0; origin_r<=0; origin_yaw<=0; elapsed<=0; stop_elapsed<=0;
 age_l<=0; age_r<=0; age_y<=0; seen_l<=0; seen_r<=0; seen_y<=0;
 still_l<=0; still_r<=0; hit_l<=0; hit_r<=0; cancelled<=0;
 done<=0; aborted<=0; rejected<=0; reason<=0; fault_code<=0;
 end else begin
 done<=0; aborted<=0; rejected<=0;
 if(tick) tick_count<=0; else tick_count<=tick_count+1'b1;
 if(speed_valid_l) begin age_l<=0; seen_l<=1; end
 else if(tick && age_l<FEEDBACK_TIMEOUT_MS) age_l<=age_l+1'b1;
 if(speed_valid_r) begin age_r<=0; seen_r<=1; end
 else if(tick && age_r<FEEDBACK_TIMEOUT_MS) age_r<=age_r+1'b1;
 if(yaw_valid) begin age_y<=0; seen_y<=1; end
 else if(tick && age_y<FEEDBACK_TIMEOUT_MS) age_y<=age_y+1'b1;
 if(fault_code!=0 || mode==FAULT || detected_fault!=0) begin
 mode<=FAULT; phase<=NONE; mag_l<=0; mag_r<=0;
 if(fault_code==0) fault_code<=detected_fault;
 if(active) begin aborted<=1; reason<=4; end
 if(clear_fault && !enable && !external_fault && fault_code!=0) begin
 fault_code<=0; mode<=DISABLED; reason<=0;
 end
 end else if(!enable) begin
 if(active) begin aborted<=1; reason<=3; end
 mode<=DISABLED; phase<=NONE; mag_l<=0; mag_r<=0;
 end else begin
 if(mode==DISABLED) mode<=IDLE;
 if(cmd_valid && !cmd_ready) begin rejected<=1; reason<=5; end
 if(active) begin
 if(tick && elapsed!=32'hffffffff) elapsed<=elapsed+1'b1;
 if(end_kind==2) begin
 if(reach_l) hit_l<=1;
 if(reach_r) hit_r<=1;
 end
 if(stop_request && !cancelled) begin
 cancelled<=1; mode<=BRAKE; phase<=DECEL;
 end else if(phase==ACCEL || phase==RUNNING) begin
 if(reached) begin phase<=DECEL; mode<=BRAKE; end
 else if(tick) begin
 mag_l<=approach(mag_l,(end_kind==2 && reach_l)?0:vel_l,(end_kind==2 && reach_l)?decel:accel);
 mag_r<=approach(mag_r,(end_kind==2 && reach_r)?0:vel_r,(end_kind==2 && reach_r)?decel:accel);
 if(mag_l==vel_l && mag_r==vel_r) phase<=RUNNING;
 end
 end else if(phase==DECEL) begin
 if(tick) begin mag_l<=approach(mag_l,0,decel); mag_r<=approach(mag_r,0,decel); end
 if(mag_l==0 && mag_r==0) begin
 phase<=WAIT_STILL; still_l<=0; still_r<=0; stop_elapsed<=0;
 end
 end else if(phase==WAIT_STILL) begin
 if(tick) stop_elapsed<=stop_elapsed+1'b1;
 if(speed_valid_l) begin
 if(!is_still(speed_l)) still_l<=0;
 else if(still_l<STILL_SAMPLES) still_l<=still_l+1'b1;
 end
 if(speed_valid_r) begin
 if(!is_still(speed_r)) still_r<=0;
 else if(still_r<STILL_SAMPLES) still_r<=still_r+1'b1;
 end
 if(still_l>=STILL_SAMPLES && still_r>=STILL_SAMPLES &&
 (!speed_valid_l || is_still(speed_l)) && (!speed_valid_r || is_still(speed_r))) begin
 phase<=NONE; mode<=IDLE;
 if(cancelled) begin aborted<=1; reason<=2; end
 else begin done<=1; reason<=1; end
 end
 end
 end else if(cmd_valid && cmd_ready) begin
 if(!valid_command) begin rejected<=1; reason<=6; end
 else begin
 mode<=cmd_mode; phase<=ACCEL; reason<=0;
 vel_l<=cmd_speed_l; vel_r<=cmd_speed_r; mag_l<=0; mag_r<=0;
 accel<=cmd_accel_step; decel<=cmd_decel_step;
 end_kind<=cmd_end; reverse<=cmd_reverse; goal<=cmd_goal; timeout_ms<=cmd_timeout_ms;
 neg_l<=(cmd_mode==BACKWARD || (cmd_mode==DISTANCE_MOVE && cmd_reverse) || (cmd_mode==ROTATE && !cmd_reverse));
 neg_r<=(cmd_mode==BACKWARD || (cmd_mode==DISTANCE_MOVE && cmd_reverse) || (cmd_mode==ROTATE && cmd_reverse));
 origin_l<=position_l; origin_r<=position_r; origin_yaw<=yaw_mdeg;
 elapsed<=0; stop_elapsed<=0; hit_l<=0; hit_r<=0; cancelled<=0;
 tick_count<=0;
 end
 end
 end
 end
 end
endmodule

