`timescale 1ns/1ps
// Independent task-4 route supervisor. It does not drive PWM directly: the
// existing closed-loop motion block consumes signed wheel velocity requests.
// A failure is latched until reset; no failure can transition to DONE.
module nav_route_supervisor #(
    parameter integer SPEED_MM_S = 50,
    parameter integer STOP_MM = 120,
    parameter integer RANGE_AGE_LIMIT = 250,
    parameter integer STATIONARY_TICKS = 5,
    parameter integer ACTION_TIMEOUT = 30000
) (
    input wire clk, input wire rst_n,
    input wire start, input wire stop, input wire estop,
    input wire [2:0] route_id,
    input wire calibration_done, input wire geometry_ok,
    input wire imu_valid, input wire encoder_valid,
    input wire [2:0] tof_valid, input wire [2:0] tof_update,
    input wire [15:0] tof_front_mm, tof_left_mm, tof_right_mm,
    input wire signed [31:0] x_mm, y_mm, yaw_mdeg,
    output reg signed [31:0] velocity_l_mm_s, velocity_r_mm_s,
    output reg [4:0] state,
    output reg [7:0] fault_code,
    output reg [2:0] accepted_tof,
    output reg [15:0] tof_age_front, tof_age_left, tof_age_right,
    output reg done, blocked, range_fault, input_fault,
    output wire drive_enable
);
    localparam S_IDLE=0, S_CALIB=1, S_DRIVE=2, S_BRAKE=3,
               S_TURN=4, S_WAIT_RANGE=5, S_DONE=6, S_BLOCKED=7,
               S_RANGE_FAULT=8, S_INPUT_FAULT=9, S_ESTOP=10;
    localparam F_NONE=0, F_STOP=1, F_EST0P=2, F_RANGE=3, F_INPUT=4,
               F_GEOMETRY=5, F_TIMEOUT=6, F_NO_PROGRESS=7;
    reg [31:0] action_ticks, still_ticks;
    reg signed [31:0] last_x, last_y;
    reg progress_seen;
    reg [2:0] active_route;
    wire moving = state==S_DRIVE || state==S_TURN;
    assign drive_enable = moving && fault_code==F_NONE && !estop && !stop;

    function automatic [15:0] age_inc(input [15:0] v);
        begin age_inc = (v==16'hffff) ? v : v+1'b1; end
    endfunction
    function automatic [15:0] valid_age(input [15:0] age, input [2:0] upd, input [15:0] value);
        begin valid_age = age; if (upd && value!=16'd20 && value>=25 && value<=2500) valid_age=0; end
    endfunction
    wire accepted_front = tof_update[0] && tof_front_mm!=16'd20 && tof_front_mm>=25 && tof_front_mm<=2500;
    wire accepted_left  = tof_update[1] && tof_left_mm !=16'd20 && tof_left_mm >=25 && tof_left_mm <=2500;
    wire accepted_right = tof_update[2] && tof_right_mm!=16'd20 && tof_right_mm>=25 && tof_right_mm<=2500;
    wire range_expired = (tof_age_front>=RANGE_AGE_LIMIT) ||
                         ((active_route!=0) && ((tof_age_left>=RANGE_AGE_LIMIT) || (tof_age_right>=RANGE_AGE_LIMIT)));
    wire no_progress = action_ticks>ACTION_TIMEOUT;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_IDLE; fault_code<=F_NONE; active_route<=0; action_ticks<=0;
            still_ticks<=0; last_x<=0; last_y<=0; progress_seen<=0;
            velocity_l_mm_s<=0; velocity_r_mm_s<=0; done<=0; blocked<=0;
            range_fault<=0; input_fault<=0; accepted_tof<=0;
            tof_age_front<=16'hffff; tof_age_left<=16'hffff; tof_age_right<=16'hffff;
        end else begin
            done<=0; blocked<=0; range_fault<=0; input_fault<=0; accepted_tof<=0;
            tof_age_front<=valid_age(age_inc(tof_age_front),tof_update,tof_front_mm);
            tof_age_left <=valid_age(age_inc(tof_age_left),tof_update,tof_left_mm);
            tof_age_right<=valid_age(age_inc(tof_age_right),tof_update,tof_right_mm);
            if (accepted_front) accepted_tof[0]<=1;
            if (accepted_left)  accepted_tof[1]<=1;
            if (accepted_right) accepted_tof[2]<=1;
            if (state==S_DRIVE || state==S_TURN) begin
                if (action_ticks!=32'hffffffff) action_ticks<=action_ticks+1'b1;
                if (x_mm!=last_x || y_mm!=last_y) begin progress_seen<=1;last_x<=x_mm;last_y<=y_mm; end
            end
            if (estop) begin
                state<=S_ESTOP; fault_code<=F_EST0P; velocity_l_mm_s<=0; velocity_r_mm_s<=0;
            end else if (fault_code!=F_NONE) begin
                velocity_l_mm_s<=0; velocity_r_mm_s<=0;
            end else if (stop) begin
                state<=S_BRAKE; velocity_l_mm_s<=0; velocity_r_mm_s<=0; fault_code<=F_STOP;
            end else begin
                case (state)
                    S_IDLE: begin
                        velocity_l_mm_s<=0; velocity_r_mm_s<=0;
                        if (start) begin
                            if (!calibration_done || !geometry_ok) begin state<=S_BLOCKED; fault_code<=F_GEOMETRY; end
                            else if (!imu_valid || !encoder_valid) begin state<=S_INPUT_FAULT; fault_code<=F_INPUT; end
                            else begin state<=S_CALIB; active_route<=route_id; action_ticks<=0; progress_seen<=0; end
                        end
                    end
                    S_CALIB: begin
                        if (!imu_valid || !encoder_valid) begin state<=S_INPUT_FAULT; fault_code<=F_INPUT; end
                        else begin state<=S_DRIVE; action_ticks<=0; last_x<=x_mm; last_y<=y_mm; end
                    end
                    S_DRIVE: begin
                        if (!imu_valid || !encoder_valid) begin state<=S_INPUT_FAULT; fault_code<=F_INPUT; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else if (range_expired) begin state<=S_RANGE_FAULT; fault_code<=F_RANGE; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else if (no_progress) begin state<=S_BLOCKED; fault_code<=F_NO_PROGRESS; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else if (tof_front_mm<=STOP_MM && tof_valid[0] && tof_front_mm!=16'd20) begin state<=S_BRAKE; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else begin velocity_l_mm_s<=SPEED_MM_S; velocity_r_mm_s<=SPEED_MM_S; end
                    end
                    S_BRAKE: begin
                        velocity_l_mm_s<=0; velocity_r_mm_s<=0;
                        if (still_ticks<STATIONARY_TICKS) still_ticks<=still_ticks+1'b1;
                        else begin state<=S_TURN; action_ticks<=0; still_ticks<=0; end
                    end
                    S_TURN: begin
                        if (!imu_valid || !encoder_valid) begin state<=S_INPUT_FAULT; fault_code<=F_INPUT; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else if (range_expired) begin state<=S_WAIT_RANGE; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else if (action_ticks>1000) begin state<=S_WAIT_RANGE; velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                        else begin velocity_l_mm_s<=SPEED_MM_S/2; velocity_r_mm_s<=-(SPEED_MM_S/2); end
                    end
                    S_WAIT_RANGE: begin
                        velocity_l_mm_s<=0; velocity_r_mm_s<=0;
                        if (!range_expired && tof_valid!=0) begin state<=S_DRIVE; action_ticks<=0; end
                    end
                    default: begin velocity_l_mm_s<=0; velocity_r_mm_s<=0; end
                endcase
            end
            // A route completion signal is intentionally not inferred from a
            // position tolerance in this safety primitive. The route planner
            // must assert a separately verified waypoint completion in a later
            // integration layer; this prevents false DONE in test hardware.
            if (state==S_DONE) done<=1;
            if (state==S_BLOCKED) blocked<=1;
            if (state==S_RANGE_FAULT) range_fault<=1;
            if (state==S_INPUT_FAULT || state==S_ESTOP) input_fault<=1;
            if (fault_code==F_EST0P || fault_code==F_INPUT) input_fault<=1;
            if (fault_code==F_RANGE) range_fault<=1;
            if (fault_code==F_GEOMETRY || fault_code==F_NO_PROGRESS) blocked<=1;
        end
    end
endmodule
