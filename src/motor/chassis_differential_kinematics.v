// Differential-drive kinematics, integer fixed-point interface.
// Units: velocity = mm/s, angular velocity = mrad/s, wheelbase = mm.
// Therefore omega * wheelbase / 2000 is the tangential wheel velocity.
module chassis_differential_kinematics #(
    parameter integer DATA_WIDTH = 32,
    parameter integer ACC_WIDTH  = 32
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         update,
    input  wire                         enable,
    input  wire signed [DATA_WIDTH-1:0] body_velocity,
    input  wire signed [DATA_WIDTH-1:0] angular_velocity,
    input  wire [DATA_WIDTH-1:0]        wheelbase_mm,
    input  wire [DATA_WIDTH-1:0]        max_wheel_speed,
    input  wire [ACC_WIDTH-1:0]         accel_step,
    input  wire                         invert_left,
    input  wire                         invert_right,
    output reg signed [DATA_WIDTH-1:0]  left_velocity,
    output reg signed [DATA_WIDTH-1:0]  right_velocity,
    output reg                          valid
);
    localparam integer MW = 2*DATA_WIDTH + 2;
    reg signed [MW-1:0] left_raw, right_raw;
    reg signed [MW-1:0] turn_term;
    reg signed [MW-1:0] max_v;
    reg signed [MW-1:0] left_target, right_target;
    reg signed [MW-1:0] left_next, right_next;
    reg signed [MW-1:0] cur_l, cur_r;

    function signed [MW-1:0] abs_sat;
        input signed [MW-1:0] x;
        input signed [MW-1:0] lim;
        reg signed [MW-1:0] a;
        begin
            a = (x < 0) ? -x : x;
            abs_sat = (a > lim) ? ((x < 0) ? -lim : lim) : x;
        end
    endfunction

    always @* begin
        // Divide by 2000: mrad/s * mm -> mm/s (mrad to rad and half track).
        turn_term  = ($signed(angular_velocity) * $signed({1'b0,wheelbase_mm})) / 2000;
        left_raw   = $signed(body_velocity) - turn_term;
        right_raw  = $signed(body_velocity) + turn_term;
        max_v      = $signed({1'b0,max_wheel_speed});
        left_target  = abs_sat(left_raw, max_v);
        right_target = abs_sat(right_raw, max_v);
        if (invert_left)  left_target  = -left_target;
        if (invert_right) right_target = -right_target;

        left_next = left_target;
        right_next = right_target;
        if (!enable) begin
            left_next = 0;
            right_next = 0;
        end else if (accel_step != 0) begin
            if (left_target > cur_l + $signed({1'b0,accel_step}))
                left_next = cur_l + $signed({1'b0,accel_step});
            else if (left_target < cur_l - $signed({1'b0,accel_step}))
                left_next = cur_l - $signed({1'b0,accel_step});
            if (right_target > cur_r + $signed({1'b0,accel_step}))
                right_next = cur_r + $signed({1'b0,accel_step});
            else if (right_target < cur_r - $signed({1'b0,accel_step}))
                right_next = cur_r - $signed({1'b0,accel_step});
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            left_velocity  <= 0;
            right_velocity <= 0;
            cur_l <= 0;
            cur_r <= 0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;
            if (update) begin
                left_velocity  <= left_next[DATA_WIDTH-1:0];
                right_velocity <= right_next[DATA_WIDTH-1:0];
                cur_l <= left_next;
                cur_r <= right_next;
                valid <= 1'b1;
            end
        end
    end
endmodule
