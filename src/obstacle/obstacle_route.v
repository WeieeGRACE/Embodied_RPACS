`timescale 1ns/1ps
// Geometry is deliberately bounded to one static box. Distances are micrometres,
// yaw is unwrapped left-positive millidegrees. Only post-stop shots authorize moves.
module obstacle_route #(
 parameter integer CLK_HZ=50000000,WINDOW_CYCLES=CLK_HZ/100,
 EXTRA_UM=300000,PASS_LIMIT_UM=2000000,
 SLOW_MM=600,STOP_MM=130,AVOID_MM=85,SIDE_MM=300,HYST_MM=50,EDGE_MM=600,EDGE_RISE_MM=300,STABLE_MM=50,
 STEP_TIMEOUT_MS=10000,BARRIER_TIMEOUT_MS=1000,
 // Blind-detour backstop used when the obstacle-side lane keeps reporting the
 // VL53L0X minimum-range clamp instead of a usable face distance.
 FALLBACK_SHIFT_UM=500000,FALLBACK_PASS_UM=600000
)(input wire clk,rst_n,press,ready,fault,input wire [47:0] distances,
 input wire [2:0] post_turn,input wire [2:0] sample_mask,input wire [2:0] clip_mask,input wire sensor_hold,
 input wire signed [31:0] position_l,position_r,yaw,
 input wire busy,cmd_ready,done,aborted,rejected,
 output wire start,stop,rotate,reverse,output wire [1:0] end_kind,
 output reg [31:0] goal,output wire [31:0] speed,speed_cap,
 output reg refresh,output reg [4:0] state,output reg [7:0] reason,output reg route_fault,
 output wire signed [31:0] target_heading);
 localparam IDLE=0,CRUISE_START=1,CRUISE=2,BRAKE_SELECT=3,TURN_OUT=4,WAIT_OUT=5,
 SHIFT=6,TURN_PARALLEL=7,WAIT_PARALLEL=8,PASS=9,BRAKE_EDGE=10,EXTRA=11,
 TURN_BACK=12,WAIT_BACK=13,RETURN=14,TURN_HOME=15,WAIT_HOME=16,BLOCKED=17,USER_STOP=18,FAULT=19,
 BRAKE_SHIFT=21,SHIFT_EXTRA=22,SAMPLE_SHIFT=23;
 localparam integer LOW=(64'd50000*WINDOW_CYCLES)/CLK_HZ;
 reg issued,left_side,slow_mode,box_seen,edge_seen,pass_clear_seen;
 reg [2:0] retry_count;
 reg fallback_shift,fallback_pass;
 reg [1:0] clear_count,front_count,stable_count;
 reg [15:0] box_distance;
 reg signed [31:0] home,shift_l,shift_r,pass_l,pass_r,leg_l,leg_r;
 reg [31:0] actual_shift;
 reg [31:0] ticks,elapsed;
 wire ms=ticks==CLK_HZ/1000-1;
 wire [15:0] front=distances[15:0],left_mm=distances[31:16],right_mm=distances[47:32];
 wire [15:0] side_mm=left_side ? right_mm:left_mm;
 wire side_sample=sample_mask[left_side ? 2:1];
 wire turn_state=state==TURN_OUT || state==TURN_PARALLEL || state==TURN_BACK || state==TURN_HOME;
 wire drive_state=state==CRUISE_START || state==SHIFT || state==SHIFT_EXTRA || state==PASS || state==EXTRA || state==RETURN || turn_state;
 wire barrier=state==WAIT_OUT || state==WAIT_PARALLEL || state==WAIT_BACK || state==WAIT_HOME || state==20 || state==SAMPLE_SHIFT;
 wire translation=state==SHIFT_EXTRA || state==EXTRA || state==RETURN;
 wire signed [31:0] leg_traveled=((position_l-leg_l)>>>1)+((position_r-leg_r)>>>1);
 wire [31:0] leg_goal=(state==SHIFT_EXTRA || state==EXTRA) ? EXTRA_UM:actual_shift;
 wire [31:0] remaining=leg_traveled>=leg_goal ? 1:leg_goal-leg_traveled;
 wire braking=state==BRAKE_SELECT || state==BRAKE_SHIFT || state==BRAKE_EDGE || state==BLOCKED || state==USER_STOP;
 // Select an explicit signed offset before a single addition. Nested ternary
 // additions produced a different target in Gowin's synthesized replay.
 reg signed [31:0] heading_offset;
 always @* begin
 heading_offset=32'sd0;
 case(state)
 TURN_OUT:if(left_side)heading_offset=32'sd90000;else heading_offset=-32'sd90000;
 TURN_BACK:if(left_side)heading_offset=-32'sd90000;else heading_offset=32'sd90000;
 default:heading_offset=32'sd0;
 endcase
 end
 wire signed [31:0] target_yaw=home+heading_offset;
 wire signed [31:0] yaw_error=target_yaw-yaw;
 assign target_heading=target_yaw;
 wire [31:0] abs_yaw=yaw_error<0 ? -yaw_error:yaw_error;
 wire turn_space_ok=(yaw_error<0 ? right_mm:left_mm)>SIDE_MM;
 wire signed [31:0] traveled_l=position_l-pass_l,traveled_r=position_r-pass_r;
 wire pass_limit=traveled_l>=PASS_LIMIT_UM || traveled_r>=PASS_LIMIT_UM;
 wire signed [31:0] shifted_l=position_l-shift_l,shifted_r=position_r-shift_r;
 wire shift_limit=shifted_l>=PASS_LIMIT_UM || shifted_r>=PASS_LIMIT_UM;
 wire [2:0] clip_lane=left_side ? 2:1;
 wire side_clip=clip_mask[clip_lane];
 wire signed [31:0] shift_traveled=((position_l-shift_l)>>>1)+((position_r-shift_r)>>>1);
 wire signed [31:0] pass_traveled=((position_l-pass_l)>>>1)+((position_r-pass_r)>>>1);
 wire stable_side={1'b0,side_mm}+STABLE_MM>={1'b0,box_distance} &&
                  {1'b0,side_mm}<={1'b0,box_distance}+STABLE_MM;
 wire cleared_side={1'b0,side_mm}>={1'b0,box_distance}+EDGE_RISE_MM;
 wire cruise_state=state==CRUISE || state==CRUISE_START;
 wire front_block=front<=(cruise_state ? STOP_MM:AVOID_MM);
 // A degraded ToF channel stops the current drive and the same state is
 // retried once the channel is healthy again. Turning in place does not need
 // ToF data, so a sweep glitch never aborts a rotation.
 wire drive_hold=sensor_hold && !turn_state && (drive_state || state==CRUISE);
 // Only the lanes a state actually reads must be fresh after a rotation. The
 // unused opposite-side lane may legitimately keep reporting exact-20 while a
 // close wall fills its field of view; waiting for it stalled every turn.
 wire front_fresh=post_turn[0];
 wire side_fresh=post_turn[left_side ? 2:1];
 wire barrier_fresh=(state==WAIT_OUT) ? side_fresh:(front_fresh && side_fresh);
 assign rotate=turn_state;
 assign reverse=turn_state && yaw_error<0;
 assign end_kind=turn_state ? 3:(translation ? 2:0);
 assign speed=state==CRUISE_START ? 2*LOW:LOW;
 assign speed_cap=(state==CRUISE || state==CRUISE_START) && !slow_mode ? 2*LOW:LOW;
 assign start=((drive_state || state==CRUISE) && !issued && cmd_ready && ready && !fault &&
              (!turn_state || (abs_yaw>1000 && turn_space_ok)) && !front_block && (!press) && !drive_hold);
 assign stop=braking || (press && state!=IDLE && state!=BLOCKED && state!=USER_STOP) ||
 ((state==CRUISE || (drive_state && !turn_state)) && front_block && (state!=CRUISE_START || issued)) ||
 (drive_hold && (state==CRUISE || (drive_state && !turn_state)));
 always @* begin
 goal=0;
 if(turn_state)goal=abs_yaw;
 // Each edge clearance is one complete EXTRA_UM command. Only the return
 // leg retains the existing250mm command chunks.
 else if(state==SHIFT_EXTRA || state==EXTRA)goal=remaining;
 else if(translation)goal=remaining>250000 ? 250000:remaining;
 end
 task advance;
 input [4:0] next_state;
 begin state<=next_state;issued<=0;elapsed<=0;retry_count<=0;end
 endtask
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
 state<=IDLE;reason<=0;route_fault<=0;refresh<=0;issued<=0;left_side<=1;home<=0;pass_clear_seen<=0;
 shift_l<=0;shift_r<=0;pass_l<=0;pass_r<=0;actual_shift<=0;slow_mode<=0;retry_count<=0;
 fallback_shift<=0;fallback_pass<=0;
 ticks<=0;elapsed<=0;box_seen<=0;edge_seen<=0;clear_count<=0;front_count<=0;stable_count<=0;box_distance<=0;leg_l<=0;leg_r<=0;
 end else begin
 refresh<=0;
 if(ms)begin ticks<=0;if(elapsed<32'hffffffff)elapsed<=elapsed+1'b1;end else ticks<=ticks+1'b1;
 if(sample_mask[0])begin
 if(front<=SLOW_MM)begin slow_mode<=1;front_count<=0;end
 else if(front>=SLOW_MM+HYST_MM && (state==CRUISE || state==IDLE))begin
 if(front_count==2)begin slow_mode<=0;front_count<=0;end else front_count<=front_count+1'b1;
 end else front_count<=0;
 end
 if(start)issued<=1;
 if(fault)begin advance(FAULT);reason<=8'h80;end
 else if(rejected)begin
  // A command validation miss is retried from the same route state.
  if(retry_count==3'd3)begin advance(BLOCKED);reason<=8'h83;end
  else begin retry_count<=retry_count+1'b1;issued<=0;elapsed<=0;refresh<=1;end
 end else if(aborted && (drive_state || state==CRUISE))begin
  // Lost feedback or a safety stop cancelled the action; retry it. Only
  // repeated failures pause the mode - a transient never latches a fault.
  issued<=0;elapsed<=0;
  if(retry_count==3'd3)begin advance(BLOCKED);reason<=8'h86;end
  else retry_count<=retry_count+1'b1;
 end else if(drive_state && elapsed>=(translation ? 30000:(state==PASS || state==SHIFT) ? 60000:STEP_TIMEOUT_MS))begin
  if(retry_count==3'd3)begin advance(BLOCKED);reason<=8'h81;end
  else begin retry_count<=retry_count+1'b1;issued<=0;elapsed<=0;end
 end else if(barrier && elapsed>=BARRIER_TIMEOUT_MS)begin
  // A post-rotation fresh sample did not arrive in time. Refresh and wait
  // again; a persistent problem pauses the mode instead of latching a fault.
  if(retry_count==3'd3)begin advance(BLOCKED);reason<=8'h85;end
  else begin retry_count<=retry_count+1'b1;elapsed<=0;refresh<=1;end
 end else if(press && state!=IDLE && state!=BLOCKED && state!=USER_STOP)begin
 advance(USER_STOP);reason<=1;
 end else if(turn_state && !issued && abs_yaw>1000 && !turn_space_ok)begin
 advance(BLOCKED);reason<=3;
 end else case(state)
 IDLE,BLOCKED,USER_STOP:if(press && ready && !busy)begin
 home<=yaw;box_seen<=0;edge_seen<=0;pass_clear_seen<=0;clear_count<=0;reason<=0;
 fallback_shift<=0;fallback_pass<=0;
 if(front>STOP_MM)begin slow_mode<=front<=SLOW_MM;advance(CRUISE_START);end
 else if(front>AVOID_MM)begin reason<=2;advance(BRAKE_SELECT);end
 else begin reason<=2;advance(BLOCKED);end
 end
 CRUISE_START:if(front_block)begin advance(BRAKE_SELECT);reason<=2;end else if(start)advance(CRUISE);
 CRUISE:if(front_block)begin advance(BRAKE_SELECT);reason<=2;end
 BRAKE_SELECT:if(!busy)begin
 if((left_mm>SIDE_MM || right_mm>SIDE_MM) && front>AVOID_MM)begin
 left_side<=left_mm>SIDE_MM && left_mm>=right_mm;refresh<=1;advance(WAIT_OUT);
 end else begin advance(BLOCKED);reason<=3;end
 end
 // WAIT_OUT first refreshes the stationary side choice before the first rotation.
 WAIT_OUT:if(!refresh && barrier_fresh)begin
 if((left_side ? left_mm:right_mm)<=SIDE_MM)begin advance(BLOCKED);reason<=3;end
 else advance(TURN_OUT);
 end
 TURN_OUT:if((issued && done) || (!issued && abs_yaw<=1000 && !busy))begin
 refresh<=1;advance(WAIT_PARALLEL);
 end
 WAIT_PARALLEL:if(!refresh && barrier_fresh)begin
 if(front<=AVOID_MM)begin advance(BLOCKED);reason<=4;end
 else if(side_clip)begin
 // The obstacle-side lane only reports the minimum-range clamp, so there is no
 // usable face distance to learn. Keep the obstacle distance conservative and
 // open the detour on fixed mileage instead of waiting for the barrier forever.
 box_distance<=AVOID_MM;fallback_shift<=1;shift_l<=position_l;shift_r<=position_r;
 advance(SHIFT);
 end else begin box_distance<=side_mm;stable_count<=0;box_seen<=0;clear_count<=0;
 fallback_shift<=0;shift_l<=position_l;shift_r<=position_r;advance(SAMPLE_SHIFT);end
 end
 // Learn the obstacle face while stationary. Only accepted new samples from
 // the near-box lane count; no event is manufactured for discarded20.
 SAMPLE_SHIFT:if(front<=AVOID_MM)begin advance(BLOCKED);reason<=4;end
 else if(side_clip)begin
 box_distance<=AVOID_MM;fallback_shift<=1;shift_l<=position_l;shift_r<=position_r;advance(SHIFT);
 end
 else if(side_sample)begin
 // A narrow box may already be clear after the pivot. Require three normal
 // clear results; the pre-turn front observation established the obstacle.
 if(side_mm>=EDGE_MM)begin
 stable_count<=0;if(clear_count==2)advance(BRAKE_SHIFT);else clear_count<=clear_count+1'b1;
 end else if(side_mm>AVOID_MM && stable_side)begin
 clear_count<=0;
 if(stable_count==2)begin
 box_seen<=1;clear_count<=0;shift_l<=position_l;shift_r<=position_r;advance(SHIFT);
 end else stable_count<=stable_count+1'b1;
 end else begin box_distance<=side_mm;stable_count<=0;clear_count<=0;end
 end
 SHIFT:begin
 if(front_block || shift_limit)begin advance(BLOCKED);reason<=front_block ? 4:5;end
 else if(fallback_shift && shift_traveled>=FALLBACK_SHIFT_UM)advance(BRAKE_SHIFT);
 else if(side_sample)begin
 if(cleared_side)begin
 if(clear_count==2)advance(BRAKE_SHIFT);else clear_count<=clear_count+1'b1;
 end else clear_count<=0;
 end
 end
 BRAKE_SHIFT:if(!busy)begin leg_l<=position_l;leg_r<=position_r;advance(SHIFT_EXTRA);end
 SHIFT_EXTRA:if(front_block)begin advance(BLOCKED);reason<=4;end
 else if(issued && done && leg_traveled<EXTRA_UM)issued<=0;
 else if(issued && done)begin
 actual_shift<=((position_l-shift_l)>>>1)+((position_r-shift_r)>>>1);
 advance(TURN_PARALLEL);
 end
 TURN_PARALLEL:if((issued && done) || (!issued && abs_yaw<=1000 && !busy))begin
 refresh<=1;advance(WAIT_BACK);
 end
 WAIT_BACK:if(!refresh && barrier_fresh)begin
 if(front<=AVOID_MM)begin advance(BLOCKED);reason<=4;end
 else begin pass_l<=position_l;pass_r<=position_r;box_seen<=0;stable_count<=0;clear_count<=0;
 if(side_clip)begin box_distance<=AVOID_MM;fallback_pass<=1;pass_clear_seen<=1;end
 else begin fallback_pass<=0;pass_clear_seen<=side_mm>=EDGE_MM;end
 advance(PASS);end
 end
 PASS:begin
 if(front_block || pass_limit)begin advance(BLOCKED);reason<=front_block ? 4:5;end
 else if(fallback_pass && pass_traveled>=FALLBACK_PASS_UM)advance(BRAKE_EDGE);
 else if(side_sample)begin
 if(!pass_clear_seen)begin
 if(side_mm>=EDGE_MM)pass_clear_seen<=1;
 stable_count<=0;clear_count<=0;
 end else if(!box_seen)begin
 if(side_mm<EDGE_MM && side_mm>AVOID_MM)begin
 if(stable_count!=0 && stable_side)begin
 if(stable_count==2)begin box_seen<=1;stable_count<=0;end else stable_count<=stable_count+1'b1;
 end else begin box_distance<=side_mm;stable_count<=1;end
 end else stable_count<=0;
 clear_count<=0;
 end
 else if(cleared_side)begin
 if(clear_count==2)begin edge_seen<=1;advance(BRAKE_EDGE);end else clear_count<=clear_count+1'b1;
 end else clear_count<=0;
 end
 end
 BRAKE_EDGE:if(!busy)begin leg_l<=position_l;leg_r<=position_r;advance(EXTRA);end
 EXTRA:if(front_block)begin advance(BLOCKED);reason<=4;end
 else if(issued && done)begin if(leg_traveled<EXTRA_UM)issued<=0;else advance(TURN_BACK);end
 TURN_BACK:if((issued && done) || (!issued && abs_yaw<=1000 && !busy))begin refresh<=1;advance(WAIT_HOME);end
 WAIT_HOME:if(!refresh && barrier_fresh)begin
 if(front<=AVOID_MM)begin advance(BLOCKED);reason<=4;end else begin leg_l<=position_l;leg_r<=position_r;advance(RETURN);end
 end
 RETURN:if(front_block)begin advance(BLOCKED);reason<=4;end
 else if(issued && done)begin if(leg_traveled<actual_shift)issued<=0;else advance(TURN_HOME);end
 TURN_HOME:if((issued && done) || (!issued && abs_yaw<=1000 && !busy))begin refresh<=1;advance(20);end
 20:if(!refresh && barrier_fresh)begin
 if(front<=STOP_MM)begin advance(BLOCKED);reason<=4;end else advance(CRUISE_START);
 end
 FAULT:begin end
 default:begin advance(FAULT);route_fault<=1;reason<=8'h82;end
 endcase
 end
endmodule

