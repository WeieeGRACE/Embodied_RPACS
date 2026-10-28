`timescale 1ns/1ps
// Fixed-table event navigation controller. Only the current segment and one
// segment record are retained; no sampled trajectory is stored.
module nav_event_segment_controller #(
    parameter integer STATIONARY_TICKS=5,
    parameter integer TURN_TIMEOUT_TICKS=10000,
    parameter integer ACTION_TIMEOUT_TICKS=30000,
    parameter integer RANGE_WAIT_TIMEOUT_TICKS=500,
    parameter integer NO_PROGRESS_TICKS=500,
    parameter integer YAW_TOLERANCE_MDEG=5000,
    parameter integer GOAL_TOLERANCE_MM=100
) (
    input wire clk, input wire rst_n,
    input wire control_tick,
    input wire start, input wire stop, input wire estop,
    input wire [2:0] route_id,
    input wire calibration_done, input wire geometry_ok,
    input wire imu_valid, input wire encoder_valid,
    input wire range_ready,
    input wire signed [31:0] pose_x_mm, pose_y_mm,
    input wire [31:0] odometry_distance_mm,
    input wire signed [31:0] yaw_mdeg,
    input wire wheels_stationary,
    input wire front_valid, input wire [15:0] front_mm,
    input wire range_stale_fault,
    input wire wall_event_pulse, input wire [7:0] wall_event_id,
    input wire range_fresh_after_turn,
    output wire clear_wall_event,
    output reg signed [31:0] velocity_l_mm_s, velocity_r_mm_s,
    output reg [4:0] state,
    output reg [3:0] segment_id,
    output reg [7:0] accepted_wall_event,
    output reg [31:0] segment_distance_mm,
    output wire [15:0] expected_min_mm, expected_max_mm, hard_stop_mm,
    output wire [15:0] forward_stop_mm,
    output wire [7:0] expected_wall_id,
    output wire [1:0] expected_tof_dir, turn_kind,
    output wire [15:0] segment_speed_mm_s,
    output wire signed [31:0] target_yaw_mdeg,
    output wire signed [31:0] target_x_mm, target_y_mm,
    output reg done, blocked, range_fault, input_fault,
    output reg [7:0] fault_code,
    output reg record_valid,
    output reg [3:0] record_segment_id,
    output reg [31:0] record_distance_mm,
    output reg signed [31:0] record_start_yaw_mdeg,
    output reg signed [31:0] record_end_yaw_mdeg,
    output reg [7:0] record_wall_event_id,
    output reg [7:0] record_fault_code,
    output wire drive_enable
);
    localparam S_IDLE=0,S_DRIVE=1,S_BRAKE=2,S_TURN=3,S_WAIT_RANGE=4,
               S_DONE=5,S_BLOCKED=6,S_RANGE_FAULT=7,S_INPUT_FAULT=8,S_ESTOP=9;
    localparam F_NONE=0,F_STOP=1,F_ESTOP=2,F_RANGE=3,F_INPUT=4,F_GEOMETRY=5,
               F_TIMEOUT=6,F_EVENT_MISMATCH=7,F_NO_PROGRESS=8,F_BLOCKED=9,F_GOAL=10;
    localparam B_NONE=0,B_EVENT=1,B_FRONT=2,B_DISTANCE=3,B_TIMEOUT=4,B_STALL=5,B_MISMATCH=6;

    wire table_valid, final_segment;
    wire [15:0] event_range_min_mm,event_range_max_mm;
    nav_segment_table table_i(
      .route_id(route_id),.segment_id(segment_id),.valid(table_valid),
      .final_segment(final_segment),.distance_min_mm(expected_min_mm),
      .distance_max_mm(expected_max_mm),.hard_stop_mm(hard_stop_mm),
      .forward_stop_mm(forward_stop_mm),.wall_id(expected_wall_id),.tof_dir(expected_tof_dir),
      .event_range_min_mm(event_range_min_mm),.event_range_max_mm(event_range_max_mm),
      .turn_kind(turn_kind),.speed_mm_s(segment_speed_mm_s),
      .target_yaw_mdeg(target_yaw_mdeg),.target_x_mm(target_x_mm),.target_y_mm(target_y_mm));

    reg [31:0] segment_start_distance;
    reg signed [31:0] segment_start_yaw;
    reg [31:0] action_ticks,elapsed_ticks,still_ticks;
    reg [31:0] progress_distance;
    reg signed [31:0] progress_yaw;
    reg event_seen;
    reg [2:0] brake_reason;
    wire distance_underflow=odometry_distance_mm<segment_start_distance;
    wire event_matches=wall_event_pulse && wall_event_id==expected_wall_id && expected_wall_id!=0;
    wire event_in_window=segment_distance_mm>=expected_min_mm && segment_distance_mm<=expected_max_mm;
    wire hard_stop_reached=segment_distance_mm>=hard_stop_mm;
    wire front_obstacle=front_valid && front_mm!=16'd20 && front_mm<=forward_stop_mm;
    wire signed [31:0] x_error=pose_x_mm-target_x_mm;
    wire signed [31:0] y_error=pose_y_mm-target_y_mm;
    wire [31:0] abs_x_error=x_error<0 ? -x_error:x_error;
    wire [31:0] abs_y_error=y_error<0 ? -y_error:y_error;

    always @* begin
        segment_distance_mm=distance_underflow ? 0 : odometry_distance_mm-segment_start_distance;
    end

    assign clear_wall_event=(state==S_IDLE) || (state==S_WAIT_RANGE) ||
                            (state==S_BRAKE && still_ticks>=STATIONARY_TICKS);
    assign drive_enable=(state==S_DRIVE || state==S_TURN) && fault_code==F_NONE && !stop && !estop;

    function automatic signed [31:0] angle_error(input signed [31:0] target,input signed [31:0] actual);
        reg signed [31:0] d;
        begin
            d=target-actual;
            if (d>=180000) d=d-360000;
            if (d< -180000) d=d+360000;
            angle_error=d;
        end
    endfunction
    wire signed [31:0] yaw_error=angle_error(target_yaw_mdeg,yaw_mdeg);
    // Q24 reciprocal for 1/500 keeps the heading correction out of a wide
    // combinational divider on the 50 MHz route-control path. Bias negative
    // products before shifting to preserve division's truncation-to-zero rule.
    // Pipeline the multiply and scale over two 50 MHz clocks; the navigation
    // command updates at 100 Hz, so this latency does not change route timing.
    // 33554 needs a signed width of at least 17 bits.
    localparam signed [16:0] YAW_RECIP_Q24 = 17'sd33554;
    reg signed [31:0] yaw_error_pipe;
    reg signed [48:0] yaw_correction_product_pipe;
    reg signed [31:0] yaw_correction_raw_pipe;
    wire signed [48:0] yaw_correction_product_next = yaw_error_pipe * YAW_RECIP_Q24;
    wire signed [48:0] yaw_correction_adjusted = yaw_correction_product_pipe<0 ?
        yaw_correction_product_pipe + 49'sd16777215 : yaw_correction_product_pipe;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            yaw_error_pipe<=0;
            yaw_correction_product_pipe<=0;
            yaw_correction_raw_pipe<=0;
        end else begin
            yaw_error_pipe<=yaw_error;
            yaw_correction_product_pipe<=yaw_correction_product_next;
            yaw_correction_raw_pipe<=yaw_correction_adjusted >>> 24;
        end
    end
    wire signed [31:0] yaw_correction_raw=yaw_correction_raw_pipe;
    wire signed [31:0] yaw_correction_limit=$signed({16'd0,segment_speed_mm_s})/3;
    wire signed [31:0] yaw_correction=yaw_correction_raw>yaw_correction_limit ? yaw_correction_limit:
                                yaw_correction_raw< -yaw_correction_limit ? -yaw_correction_limit:
                                yaw_correction_raw;
    wire yaw_at_target=(yaw_error<=YAW_TOLERANCE_MDEG && yaw_error>=-YAW_TOLERANCE_MDEG);
    wire target_position_ok=(abs_x_error<=GOAL_TOLERANCE_MM && abs_y_error<=GOAL_TOLERANCE_MM);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_IDLE;segment_id<=0;segment_start_distance<=0;action_ticks<=0;
            elapsed_ticks<=0;still_ticks<=0;progress_distance<=0;progress_yaw<=0;event_seen<=0;
            brake_reason<=B_NONE;accepted_wall_event<=0;velocity_l_mm_s<=0;velocity_r_mm_s<=0;
            done<=0;blocked<=0;range_fault<=0;input_fault<=0;fault_code<=F_NONE;
            record_valid<=0;record_segment_id<=0;record_distance_mm<=0;
            record_start_yaw_mdeg<=0;record_end_yaw_mdeg<=0;
            record_wall_event_id<=0;record_fault_code<=0;segment_start_yaw<=0;
        end else begin
            done<=0;blocked<=0;range_fault<=0;input_fault<=0;record_valid<=0;

            if (control_tick && (state==S_DRIVE || state==S_TURN)) begin
                if (elapsed_ticks!=32'hffffffff) elapsed_ticks<=elapsed_ticks+1'b1;
                if (odometry_distance_mm!=progress_distance || yaw_mdeg!=progress_yaw) begin
                    progress_distance<=odometry_distance_mm;progress_yaw<=yaw_mdeg;action_ticks<=0;
                end else if (action_ticks!=32'hffffffff) action_ticks<=action_ticks+1'b1;
            end
            if (control_tick && state==S_BRAKE) begin
                if (wheels_stationary) begin
                    if (still_ticks<STATIONARY_TICKS) still_ticks<=still_ticks+1'b1;
                end else still_ticks<=0;
            end

            if (estop) begin
                state<=S_ESTOP;fault_code<=F_ESTOP;velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                if (state!=S_ESTOP && fault_code==F_NONE) begin
                    record_valid<=1;record_segment_id<=segment_id;record_distance_mm<=segment_distance_mm;
                    record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                    record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;record_fault_code<=F_ESTOP;
                end
            end else if (fault_code!=F_NONE) begin
                velocity_l_mm_s<=0;velocity_r_mm_s<=0;
            end else if (stop) begin
                state<=S_BLOCKED;fault_code<=F_STOP;velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                record_valid<=1;record_segment_id<=segment_id;record_distance_mm<=segment_distance_mm;
                record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;record_fault_code<=F_STOP;
            end else begin
                case (state)
                    S_IDLE: begin
                        velocity_l_mm_s<=0;velocity_r_mm_s<=0;segment_id<=0;event_seen<=0;
                        if (start) begin
                            if (!calibration_done || !geometry_ok || !table_valid) begin
                                state<=S_BLOCKED;fault_code<=F_GEOMETRY;
                                record_valid<=1;record_segment_id<=0;record_distance_mm<=0;
                                record_start_yaw_mdeg<=yaw_mdeg;record_end_yaw_mdeg<=yaw_mdeg;
                                record_wall_event_id<=0;record_fault_code<=F_GEOMETRY;
                            end else if (!imu_valid || !encoder_valid) begin
                                state<=S_INPUT_FAULT;fault_code<=F_INPUT;
                                record_valid<=1;record_segment_id<=0;record_distance_mm<=0;
                                record_start_yaw_mdeg<=yaw_mdeg;record_end_yaw_mdeg<=yaw_mdeg;
                                record_wall_event_id<=0;record_fault_code<=F_INPUT;
                            end else if (range_stale_fault) begin
                                state<=S_RANGE_FAULT;fault_code<=F_RANGE;
                                record_valid<=1;record_segment_id<=0;record_distance_mm<=0;
                                record_start_yaw_mdeg<=yaw_mdeg;record_end_yaw_mdeg<=yaw_mdeg;
                                record_wall_event_id<=0;record_fault_code<=F_RANGE;
                            end else if (!range_ready) begin
                                state<=S_RANGE_FAULT;fault_code<=F_RANGE;
                                record_valid<=1;record_segment_id<=0;record_distance_mm<=0;
                                record_start_yaw_mdeg<=yaw_mdeg;record_end_yaw_mdeg<=yaw_mdeg;
                                record_wall_event_id<=0;record_fault_code<=F_RANGE;
                            end else begin
                                state<=S_DRIVE;segment_start_distance<=odometry_distance_mm;
                                segment_start_yaw<=yaw_mdeg;progress_distance<=odometry_distance_mm;
                                progress_yaw<=yaw_mdeg;action_ticks<=0;elapsed_ticks<=0;still_ticks<=0;
                                brake_reason<=B_NONE;
                            end
                        end
                    end
                    S_DRIVE: begin
                        velocity_l_mm_s<=segment_speed_mm_s-yaw_correction;
                        velocity_r_mm_s<=segment_speed_mm_s+yaw_correction;
                        if (!imu_valid || !encoder_valid) begin
                            state<=S_INPUT_FAULT;fault_code<=F_INPUT;velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                            record_valid<=1;record_segment_id<=segment_id;record_distance_mm<=segment_distance_mm;
                            record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                            record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;record_fault_code<=F_INPUT;
                        end else if (range_stale_fault) begin
                            state<=S_RANGE_FAULT;fault_code<=F_RANGE;velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                            record_valid<=1;record_segment_id<=segment_id;record_distance_mm<=segment_distance_mm;
                            record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                            record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;record_fault_code<=F_RANGE;
                        end else if (front_obstacle) begin
                            state<=S_BRAKE;brake_reason<=B_FRONT;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (hard_stop_reached) begin
                            state<=S_BRAKE;brake_reason<=B_DISTANCE;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (action_ticks>NO_PROGRESS_TICKS) begin
                            state<=S_BRAKE;brake_reason<=B_STALL;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (elapsed_ticks>ACTION_TIMEOUT_TICKS) begin
                            state<=S_BRAKE;brake_reason<=B_TIMEOUT;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (wall_event_pulse && !event_matches) begin
                            state<=S_BRAKE;brake_reason<=B_MISMATCH;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (event_matches && event_in_window && !event_seen) begin
                            event_seen<=1;accepted_wall_event<=wall_event_id;
                            state<=S_BRAKE;brake_reason<=B_EVENT;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end
                    end
                    S_BRAKE: begin
                        velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        if (still_ticks>=STATIONARY_TICKS) begin
                            record_valid<=1;record_segment_id<=segment_id;
                            record_distance_mm<=segment_distance_mm;
                            record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                            record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;
                            record_fault_code<=0;
                            if (brake_reason!=B_EVENT || !event_seen) begin
                                state<=S_BLOCKED;fault_code<=brake_reason==B_DISTANCE ? F_BLOCKED:
                                    (brake_reason==B_TIMEOUT ? F_TIMEOUT:
                                    (brake_reason==B_STALL ? F_NO_PROGRESS:
                                    (brake_reason==B_MISMATCH ? F_EVENT_MISMATCH:F_BLOCKED)));
                                record_fault_code<=brake_reason==B_DISTANCE ? F_BLOCKED:
                                    (brake_reason==B_TIMEOUT ? F_TIMEOUT:
                                    (brake_reason==B_STALL ? F_NO_PROGRESS:
                                    (brake_reason==B_MISMATCH ? F_EVENT_MISMATCH:F_BLOCKED)));
                            end else if (final_segment) begin
                                if (target_position_ok && yaw_at_target) begin state<=S_DONE;done<=1;end
                                else begin state<=S_BLOCKED;fault_code<=F_GOAL;record_fault_code<=F_GOAL;end
                            end else begin
                                segment_id<=segment_id+1'b1;
                                segment_start_distance<=odometry_distance_mm;segment_start_yaw<=yaw_mdeg;
                                event_seen<=0;action_ticks<=0;elapsed_ticks<=0;still_ticks<=0;brake_reason<=B_NONE;
                                progress_distance<=odometry_distance_mm;progress_yaw<=yaw_mdeg;
                                // The next row owns the next segment's heading.
                                state<=S_TURN;
                            end
                        end
                    end
                    S_TURN: begin
                        if (!imu_valid || !encoder_valid) begin
                            state<=S_INPUT_FAULT;fault_code<=F_INPUT;velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                            record_valid<=1;record_segment_id<=segment_id;record_distance_mm<=segment_distance_mm;
                            record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                            record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;record_fault_code<=F_INPUT;
                        end else if (yaw_at_target) begin
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;state<=S_WAIT_RANGE;action_ticks<=0;
                        end else if (elapsed_ticks>TURN_TIMEOUT_TICKS) begin
                            state<=S_BRAKE;brake_reason<=B_TIMEOUT;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (action_ticks>NO_PROGRESS_TICKS) begin
                            state<=S_BRAKE;brake_reason<=B_STALL;still_ticks<=0;
                            velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        end else if (turn_kind==0) begin
                            if (yaw_error>0) begin
                                velocity_l_mm_s<=-(segment_speed_mm_s/2);velocity_r_mm_s<=segment_speed_mm_s/2;
                            end else begin
                                velocity_l_mm_s<=segment_speed_mm_s/2;velocity_r_mm_s<=-(segment_speed_mm_s/2);
                            end
                        end else if (yaw_error>0) begin
                            velocity_l_mm_s<=segment_speed_mm_s/2;velocity_r_mm_s<=segment_speed_mm_s;
                        end else begin
                            velocity_l_mm_s<=segment_speed_mm_s;velocity_r_mm_s<=segment_speed_mm_s/2;
                        end
                    end
                    S_WAIT_RANGE: begin
                        velocity_l_mm_s<=0;velocity_r_mm_s<=0;
                        if (!range_fresh_after_turn) begin
                            if (control_tick && action_ticks<RANGE_WAIT_TIMEOUT_TICKS)
                                action_ticks<=action_ticks+1'b1;
                            else if (control_tick) begin
                                state<=S_RANGE_FAULT;fault_code<=F_RANGE;
                                record_valid<=1;record_segment_id<=segment_id;record_distance_mm<=segment_distance_mm;
                                record_start_yaw_mdeg<=segment_start_yaw;record_end_yaw_mdeg<=yaw_mdeg;
                                record_wall_event_id<=event_seen ? accepted_wall_event:8'd0;record_fault_code<=F_RANGE;
                            end
                        end
                        else begin
                            state<=S_DRIVE;action_ticks<=0;segment_start_distance<=odometry_distance_mm;
                            segment_start_yaw<=yaw_mdeg;progress_distance<=odometry_distance_mm;progress_yaw<=yaw_mdeg;
                        end
                    end
                    S_DONE: begin velocity_l_mm_s<=0;velocity_r_mm_s<=0;done<=1; end
                    S_BLOCKED: begin velocity_l_mm_s<=0;velocity_r_mm_s<=0;blocked<=1; end
                    S_RANGE_FAULT: begin velocity_l_mm_s<=0;velocity_r_mm_s<=0;range_fault<=1; end
                    S_INPUT_FAULT,S_ESTOP: begin velocity_l_mm_s<=0;velocity_r_mm_s<=0;input_fault<=1; end
                    default: state<=S_IDLE;
                endcase
            end
            if (fault_code==F_RANGE) range_fault<=1;
            if (fault_code==F_INPUT || fault_code==F_ESTOP) input_fault<=1;
            if (fault_code==F_GEOMETRY || fault_code==F_TIMEOUT || fault_code==F_EVENT_MISMATCH ||
                fault_code==F_NO_PROGRESS || fault_code==F_BLOCKED || fault_code==F_GOAL || fault_code==F_STOP)
                blocked<=1;
        end
    end
endmodule
