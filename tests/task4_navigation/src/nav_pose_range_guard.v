// Task 4 navigation primitive. Coordinates: +X right, +Y forward at yaw=0,
// positive yaw is left. Distances are mm and angles are millidegrees.
// This module is an independent RTL building block, not a board top-level.
module nav_pose_range_guard #(
    parameter integer AGE_LIMIT_TICKS = 100,
    parameter integer RANGE_MIN_MM = 25,
    parameter integer RANGE_MAX_MM = 2500,
    parameter signed [31:0] START_X_UM = 0,
    parameter signed [31:0] START_Y_UM = 0,
    parameter signed [31:0] START_YAW_MDEG = 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_100hz,
    input  wire               pose_valid,
    input  wire signed [23:0] left_um,
    input  wire signed [23:0] right_um,
    input  wire signed [23:0] gyro_yaw_delta_mdeg,
    input  wire signed [23:0] encoder_yaw_delta_mdeg,
    input  wire [2:0]         range_update,
    input  wire [15:0]        range_front_mm,
    input  wire [15:0]        range_left_mm,
    input  wire [15:0]        range_right_mm,
    output reg signed [31:0]   x_um,
    output reg signed [31:0]   y_um,
    output reg signed [31:0]   yaw_mdeg,
    output reg [15:0]          accepted_front_mm,
    output reg [15:0]          accepted_left_mm,
    output reg [15:0]          accepted_right_mm,
    output reg [15:0]          age_front,
    output reg [15:0]          age_left,
    output reg [15:0]          age_right,
    output reg [2:0]           range_fresh,
    output reg                 pose_fault
);
    // 100 Hz pose tick. The small piecewise-linear sine table is synthesizable
    // and intentionally bounded; route-level wall association remains outside
    // this primitive.
    wire signed [24:0] ds2_um = $signed(left_um) + $signed(right_um);
    wire signed [24:0] ds_um = ds2_um / 2; // center displacement, micrometres
    wire signed [24:0] dyaw = $signed(gyro_yaw_delta_mdeg);
    wire signed [24:0] encoder_dyaw = $signed(encoder_yaw_delta_mdeg);
    wire signed [25:0] fused_yaw = (dyaw * 3 + encoder_dyaw) / 4;

    function automatic signed [15:0] sin_q15(input integer angle);
        integer a, q, v;
        begin
            a = angle % 360000; if (a < 0) a = a + 360000;
            q = a / 90000;
            case (q)
                0: v = (a * 32767) / 90000;
                1: v = 32767 - ((a-90000) * 32767) / 90000;
                2: v = -((a-180000) * 32767) / 90000;
                default: v = -32767 + ((a-270000) * 32767) / 90000;
            endcase
            sin_q15 = v;
        end
    endfunction
    function automatic signed [15:0] cos_q15(input integer angle);
        begin cos_q15 = sin_q15(angle + 90000); end
    endfunction
    wire signed [40:0] dx_product = $signed(ds_um) * $signed(sin_q15(yaw_mdeg));
    wire signed [40:0] dy_product = $signed(ds_um) * $signed(cos_q15(yaw_mdeg));
    // sin_q15/cos_q15 use 32767, so an arithmetic 15-bit shift is a close,
    // constant-time divide by the Q15 scale (error under 0.004%).
    wire signed [31:0] dx_step_um = -($signed(dx_product) >>> 15);
    wire signed [31:0] dy_step_um =  ($signed(dy_product) >>> 15);

    function automatic [15:0] next_age(input [15:0] age);
        begin if (age != 16'hffff) next_age = age + 1'b1; else next_age = age; end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_um <= START_X_UM; y_um <= START_Y_UM; yaw_mdeg <= START_YAW_MDEG; pose_fault <= 0;
            accepted_front_mm <= 0; accepted_left_mm <= 0; accepted_right_mm <= 0;
            age_front <= 16'hffff; age_left <= 16'hffff; age_right <= 16'hffff;
            range_fresh <= 0;
        end else begin
            range_fresh <= 0;
            if (tick_100hz) begin
                age_front <= next_age(age_front);
                age_left <= next_age(age_left);
                age_right <= next_age(age_right);
                if (pose_valid) begin
                    // This estimator primitive uses straight-line odometry.
                    // Full map-constrained sin/cos integration belongs in the
                    // higher level and is not represented as validated here.
                    if ((fused_yaw > 26'sd18000000) || (fused_yaw < -26'sd18000000))
                        pose_fault <= 1'b1;
                    else begin
                        x_um <= x_um + dx_step_um;
                        y_um <= y_um + dy_step_um;
                        if (yaw_mdeg + fused_yaw >= 32'sd18000000)
                            yaw_mdeg <= yaw_mdeg + fused_yaw - 32'sd36000000;
                        else if (yaw_mdeg + fused_yaw < -32'sd18000000)
                            yaw_mdeg <= yaw_mdeg + fused_yaw + 32'sd36000000;
                        else yaw_mdeg <= yaw_mdeg + fused_yaw;
                    end
                end else pose_fault <= 1'b1;
            end
            if (range_update[0] && range_front_mm != 16'd20 && range_front_mm >= RANGE_MIN_MM && range_front_mm <= RANGE_MAX_MM) begin
                accepted_front_mm <= range_front_mm; age_front <= 0; range_fresh[0] <= 1'b1;
            end
            if (range_update[1] && range_left_mm != 16'd20 && range_left_mm >= RANGE_MIN_MM && range_left_mm <= RANGE_MAX_MM) begin
                accepted_left_mm <= range_left_mm; age_left <= 0; range_fresh[1] <= 1'b1;
            end
            if (range_update[2] && range_right_mm != 16'd20 && range_right_mm >= RANGE_MIN_MM && range_right_mm <= RANGE_MAX_MM) begin
                accepted_right_mm <= range_right_mm; age_right <= 0; range_fresh[2] <= 1'b1;
            end
        end
    end
endmodule
