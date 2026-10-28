`timescale 1ns/1ps
// Normalized simulation shell. Wall IDs come only from the fixed route/map
// association; the shell exposes sensor values but no caller-supplied wall ID.
module task4_event_navigation_top #(
    parameter integer CLK_HZ=50000000
) (
    input wire clk_50m, input wire reset_n,
    input wire key_start_n, input wire key_stop_n, input wire key_estop_n,
    input wire calibration_done, input wire geometry_ok,
    input wire range_stale_external,
    input wire imu_valid, input wire encoder_valid,
    input wire signed [23:0] encoder_left_um, encoder_right_um,
    input wire signed [23:0] imu_yaw_delta_mdeg,
    input wire wheels_stationary,
    input wire [2:0] tof_valid, tof_update,
    input wire [15:0] tof_front_mm, tof_left_mm, tof_right_mm,
    input wire [2:0] route_id,
    output wire signed [31:0] pose_x_um, pose_y_um, pose_yaw_mdeg,
    output wire signed [31:0] velocity_l_mm_s, velocity_r_mm_s,
    output wire [4:0] nav_state, output wire [3:0] segment_id,
    output wire [7:0] fault_code, output wire [7:0] wall_event_id,
    output wire [2:0] accepted_tof, output wire [2:0] tof_fresh,
    output wire [15:0] tof_age_front, tof_age_left, tof_age_right,
    output wire record_valid, output wire [3:0] record_segment_id,
    output wire [31:0] record_distance_mm,
    output wire signed [31:0] record_start_yaw_mdeg, record_end_yaw_mdeg,
    output wire [7:0] record_wall_event_id, record_fault_code,
    output wire nav_done, nav_blocked, nav_range_fault, nav_input_fault,
    output wire pose_fault_out,odometry_fault_out,
    output wire drive_enable
);
    reg [31:0] div_count;
    reg tick_100hz;
    reg [31:0] odometry_distance_mm;
    reg [9:0] odometry_fraction_um;
    reg odometry_fault;
    wire signed [31:0] x_um_i,y_um_i,yaw_i;
    wire pose_fault;
    assign pose_fault_out=pose_fault;
    assign odometry_fault_out=odometry_fault;
    wire [15:0] accepted_front,accepted_left,accepted_right,age_front,age_left,age_right;
    wire [2:0] pose_fresh;
    // At the configured 50 mm/s route speed a 100 Hz wheel delta is about
    // 500 um. Reject >2 mm/sample, then update integer millimetres and the
    // sub-mm remainder with small add/compare logic instead of wide / and %.
    wire delta_over_limit=(encoder_left_um>24'sd2000 || encoder_left_um< -24'sd2000 ||
                           encoder_right_um>24'sd2000 || encoder_right_um< -24'sd2000);
    wire signed [13:0] center_sum_small =
        $signed(encoder_left_um[12:0]) + $signed(encoder_right_um[12:0]);
    wire [13:0] center_sum_abs_um = center_sum_small<0 ? -center_sum_small:center_sum_small;
    wire [12:0] center_travel_um = center_sum_abs_um[13:1];
    wire [11:0] center_with_fraction_um = {2'b0,odometry_fraction_um}+center_travel_um[11:0];
    wire signed [25:0] wheel_difference_um = $signed(encoder_right_um)-$signed(encoder_left_um);
    // mdeg_per_um = 57296/240000. A reciprocal multiply removes the large
    // variable divider from the 50 MHz path while retaining sub-mdeg scale.
    localparam signed [31:0] ENC_YAW_RECIP_Q24 = 32'sd4005281;
    wire signed [57:0] encoder_yaw_product = $signed(wheel_difference_um) * ENC_YAW_RECIP_Q24;
    wire signed [57:0] encoder_yaw_scaled = encoder_yaw_product >>> 24;
    wire signed [23:0] encoder_yaw_delta_mdeg = encoder_yaw_scaled[23:0];
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin div_count<=0;tick_100hz<=0;odometry_distance_mm<=0;odometry_fraction_um<=0;odometry_fault<=0;end
        else if (div_count>=CLK_HZ/100-1) begin
            div_count<=0;tick_100hz<=1;
            // Opposite signed wheel motion cancels during a pure turn. Reverse
            // translation contributes positive segment travel by magnitude.
            if (delta_over_limit) odometry_fault<=1'b1;
            else if (center_with_fraction_um>=12'd2000) begin
                if (odometry_distance_mm>=32'hfffffffd) odometry_distance_mm<=32'hffffffff;
                else odometry_distance_mm<=odometry_distance_mm+2'd2;
                odometry_fraction_um<=center_with_fraction_um-12'd2000;
            end else if (center_with_fraction_um>=12'd1000) begin
                if (odometry_distance_mm!=32'hffffffff) odometry_distance_mm<=odometry_distance_mm+1'b1;
                odometry_fraction_um<=center_with_fraction_um-12'd1000;
            end else odometry_fraction_um<=center_with_fraction_um[9:0];
        end else begin div_count<=div_count+1'b1;tick_100hz<=0;end
    end

    reg pose_started;
    always @(posedge clk_50m or negedge reset_n)
        if (!reset_n) pose_started<=0;
        else if (imu_valid && encoder_valid) pose_started<=1;

    nav_pose_range_guard #(.START_X_UM(32'sd2700000),.START_Y_UM(32'sd300000)) pose(
      .clk(clk_50m),.rst_n(reset_n),.tick_100hz(tick_100hz && (pose_started || (imu_valid && encoder_valid))),.pose_valid(imu_valid && encoder_valid),
      .left_um(encoder_left_um),.right_um(encoder_right_um),.gyro_yaw_delta_mdeg(imu_yaw_delta_mdeg),
      .encoder_yaw_delta_mdeg(encoder_yaw_delta_mdeg),.range_update(tof_update),
      .range_front_mm(tof_front_mm),.range_left_mm(tof_left_mm),.range_right_mm(tof_right_mm),
      .x_um(x_um_i),.y_um(y_um_i),.yaw_mdeg(yaw_i),.accepted_front_mm(accepted_front),
      .accepted_left_mm(accepted_left),.accepted_right_mm(accepted_right),.age_front(age_front),
      .age_left(age_left),.age_right(age_right),.range_fresh(pose_fresh),.pose_fault(pose_fault));

    wire [7:0] expected_wall;
    wire [15:0] expected_min,expected_max,hard_stop,forward_stop,segment_speed;
    wire [15:0] event_range_min,event_range_max;
    wire [1:0] expected_dir,turn_kind;
    wire signed [31:0] target_yaw,target_x,target_y;
    wire table_valid,final_segment;
    nav_segment_table config_i(
      .route_id(route_id),.segment_id(segment_id),.valid(table_valid),.final_segment(final_segment),
      .distance_min_mm(expected_min),.distance_max_mm(expected_max),.hard_stop_mm(hard_stop),
      .forward_stop_mm(forward_stop),.wall_id(expected_wall),.tof_dir(expected_dir),
      .event_range_min_mm(event_range_min),.event_range_max_mm(event_range_max),.turn_kind(turn_kind),
      .speed_mm_s(segment_speed),.target_yaw_mdeg(target_yaw),.target_x_mm(target_x),.target_y_mm(target_y));

    reg selected_valid,selected_update;
    reg [15:0] selected_mm;
    always @* begin
        case (expected_dir)
            0: begin selected_valid=tof_valid[0];selected_update=tof_update[0];selected_mm=tof_front_mm;end
            1: begin selected_valid=tof_valid[1];selected_update=tof_update[1];selected_mm=tof_left_mm;end
            default: begin selected_valid=tof_valid[2];selected_update=tof_update[2];selected_mm=tof_right_mm;end
        endcase
    end
    wire [7:0] ray_wall_id;
    wire [15:0] predicted_wall_range;
    wire map_match;
    nav_map_wall_association #(.RANGE_TOLERANCE_MM(50)) map_assoc(
      .pose_x_mm(x_um_i/1000),.pose_y_mm(y_um_i/1000),.yaw_mdeg(yaw_i),.tof_dir(expected_dir),
      .expected_wall_id(expected_wall),.range_valid(selected_valid),.measured_range_mm(selected_mm),
      .ray_wall_id(ray_wall_id),.predicted_range_mm(predicted_wall_range),.associated(map_match));
    wire wall_event_pulse,wall_event_latched,wall_stale,wall_discard20,wall_accepted_valid,wall_accepted_pulse;
    wire [15:0] wall_age,wall_accepted;
    wire [31:0] segment_distance_i;
    wire clear_wall_event;
    nav_tof_wall_event wall_event(
      .clk(clk_50m),.rst_n(reset_n),.clear_event(clear_wall_event),.enable(nav_state==1),.age_tick(tick_100hz),
      .expected_wall_id(expected_wall),.map_match(map_match),.sample_update(selected_update),
      .sample_valid(selected_valid),.sample_mm(selected_mm),.event_range_min_mm(event_range_min),
      .event_range_max_mm(event_range_max),.segment_distance_mm(segment_distance_i),
      .distance_min_mm(expected_min),.distance_max_mm(expected_max),.age(wall_age),
      .accepted_mm(wall_accepted),.accepted_valid(wall_accepted_valid),.accepted_pulse(wall_accepted_pulse),
      .wall_event_id(wall_event_id),.event_pulse(wall_event_pulse),.event_latched(wall_event_latched),
      .stale_fault(wall_stale),.sample_discarded_20(wall_discard20));

    nav_event_segment_controller controller(
      .clk(clk_50m),.rst_n(reset_n),.control_tick(tick_100hz),
      .start(!key_start_n),.stop(!key_stop_n),.estop(!key_estop_n),
      .route_id(route_id),.calibration_done(calibration_done),.geometry_ok(geometry_ok),
      .imu_valid(imu_valid && !pose_fault),.encoder_valid(encoder_valid && !odometry_fault),
      .range_ready(wall_accepted_valid && wall_age<16'd250),
      .pose_x_mm(x_um_i/1000),.pose_y_mm(y_um_i/1000),
      .odometry_distance_mm(odometry_distance_mm),.yaw_mdeg(yaw_i),
      .wheels_stationary(wheels_stationary),
      .front_valid(tof_valid[0]),.front_mm(tof_front_mm),
      .range_stale_fault(wall_stale || range_stale_external),
      .wall_event_pulse(wall_event_pulse),.wall_event_id(wall_event_id),.range_fresh_after_turn(wall_accepted_pulse),
      .clear_wall_event(clear_wall_event),.velocity_l_mm_s(velocity_l_mm_s),.velocity_r_mm_s(velocity_r_mm_s),
      .state(nav_state),.segment_id(segment_id),.accepted_wall_event(),.segment_distance_mm(segment_distance_i),
      .expected_min_mm(),.expected_max_mm(),.hard_stop_mm(),.forward_stop_mm(),.expected_wall_id(),
      .expected_tof_dir(),.turn_kind(),.segment_speed_mm_s(),.target_yaw_mdeg(),.target_x_mm(),.target_y_mm(),
      .done(nav_done),.blocked(nav_blocked),.range_fault(nav_range_fault),.input_fault(nav_input_fault),
      .fault_code(fault_code),.record_valid(record_valid),.record_segment_id(record_segment_id),
      .record_distance_mm(record_distance_mm),.record_start_yaw_mdeg(record_start_yaw_mdeg),
      .record_end_yaw_mdeg(record_end_yaw_mdeg),.record_wall_event_id(record_wall_event_id),
      .record_fault_code(record_fault_code),.drive_enable(drive_enable));

    assign pose_x_um=x_um_i;assign pose_y_um=y_um_i;assign pose_yaw_mdeg=yaw_i;
    assign accepted_tof=wall_accepted_pulse ? (3'b001<<expected_dir):3'b000;
    assign tof_fresh=pose_fresh;
    assign tof_age_front=age_front;assign tof_age_left=age_left;assign tof_age_right=age_right;
endmodule
