`timescale 1ns/1ps
// Integration shell for the independent navigation test. Existing board
// drivers feed the normalized sensor signals here; this shell deliberately
// keeps route enable behind calibration and geometry inputs.
module task4_navigation_top #(
    parameter integer CLK_HZ=50000000
) (
    input wire clk_50m, input wire reset_n,
    input wire key_start_n, input wire key_stop_n, input wire key_estop_n,
    input wire calibration_done, input wire geometry_ok,
    input wire imu_valid, input wire encoder_valid,
    input wire signed [23:0] encoder_left_mm, encoder_right_mm,
    input wire signed [23:0] imu_yaw_delta_mdeg, encoder_yaw_delta_mdeg,
    input wire [2:0] tof_valid, tof_update,
    input wire [15:0] tof_front_mm, tof_left_mm, tof_right_mm,
    input wire [2:0] route_id,
    output wire signed [31:0] pose_x_um, pose_y_um, pose_yaw_mdeg,
    output wire signed [31:0] velocity_l_mm_s, velocity_r_mm_s,
    output wire [4:0] nav_state, output wire [7:0] fault_code,
    output wire [2:0] accepted_tof, output wire [2:0] tof_fresh,
    output wire [15:0] tof_age_front, tof_age_left, tof_age_right,
    output wire nav_done, nav_blocked, nav_range_fault, nav_input_fault,
    output wire drive_enable
);
    reg [31:0] div_count;
    reg tick_100hz;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) begin div_count<=0; tick_100hz<=0; end
        else if (div_count >= CLK_HZ/100-1) begin div_count<=0; tick_100hz<=1; end
        else begin div_count<=div_count+1'b1; tick_100hz<=0; end
    end
    wire signed [31:0] x_um_i,y_um_i,yaw_i;
    wire signed [23:0] encoder_left_um = $signed(encoder_left_mm) * 24'sd1000;
    wire signed [23:0] encoder_right_um = $signed(encoder_right_mm) * 24'sd1000;
    wire pose_fault;
    nav_pose_range_guard pose(
      .clk(clk_50m),.rst_n(reset_n),.tick_100hz(tick_100hz),.pose_valid(imu_valid && encoder_valid),
      .left_um(encoder_left_um),.right_um(encoder_right_um),.gyro_yaw_delta_mdeg(imu_yaw_delta_mdeg),
      .encoder_yaw_delta_mdeg(encoder_yaw_delta_mdeg),.range_update(tof_update),
      .range_front_mm(tof_front_mm),.range_left_mm(tof_left_mm),.range_right_mm(tof_right_mm),
      .x_um(x_um_i),.y_um(y_um_i),.yaw_mdeg(yaw_i),.accepted_front_mm(),.accepted_left_mm(),
      .accepted_right_mm(),.age_front(),.age_left(),.age_right(),.range_fresh(tof_fresh),.pose_fault(pose_fault));
    wire supervisor_estop = !key_estop_n || pose_fault;
    nav_route_supervisor supervisor(
      .clk(clk_50m),.rst_n(reset_n),.start(!key_start_n),.stop(!key_stop_n),.estop(supervisor_estop),.route_id(route_id),
      .calibration_done(calibration_done),.geometry_ok(geometry_ok),.imu_valid(imu_valid),.encoder_valid(encoder_valid),
      .tof_valid(tof_valid),.tof_update(tof_update),.tof_front_mm(tof_front_mm),.tof_left_mm(tof_left_mm),.tof_right_mm(tof_right_mm),
      .x_mm(x_um_i/1000),.y_mm(y_um_i/1000),.yaw_mdeg(yaw_i),.velocity_l_mm_s(velocity_l_mm_s),.velocity_r_mm_s(velocity_r_mm_s),
      .state(nav_state),.fault_code(fault_code),.accepted_tof(accepted_tof),.tof_age_front(tof_age_front),.tof_age_left(tof_age_left),.tof_age_right(tof_age_right),
      .done(nav_done),.blocked(nav_blocked),.range_fault(nav_range_fault),.input_fault(nav_input_fault),.drive_enable(drive_enable));
    assign pose_x_um=x_um_i; assign pose_y_um=y_um_i; assign pose_yaw_mdeg=yaw_i;
endmodule
