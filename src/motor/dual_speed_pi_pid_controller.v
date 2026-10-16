`timescale 1ns/1ps
// Standalone dual-channel signed fixed-point incremental PI/PID controller.
module dual_speed_pi_pid_controller (
    input  wire clk, input wire rst_n, input wire control_tick, input wire enable,
    input  wire signed [15:0] target_l, feedback_l,
    input  wire signed [15:0] target_r, feedback_r,
    input  wire signed [15:0] kp_l, ki_l, kd_l,
    input  wire signed [15:0] kp_r, ki_r, kd_r,
    input  wire [4:0] gain_shift_l, gain_shift_r,
    input  wire [15:0] output_limit_l, output_limit_r,
    output wire signed [15:0] control_l, control_r,
    output wire output_valid, sat_l, sat_r, active_l, active_r
);
    assign output_valid = control_tick && enable;
    speed_pi_pid_channel left_channel (
        .clk(clk), .rst_n(rst_n), .tick(control_tick), .enable(enable),
        .target(target_l), .feedback(feedback_l), .kp(kp_l), .ki(ki_l), .kd(kd_l),
        .gain_shift(gain_shift_l), .limit(output_limit_l),
        .control(control_l), .sat(sat_l), .active(active_l));
    speed_pi_pid_channel right_channel (
        .clk(clk), .rst_n(rst_n), .tick(control_tick), .enable(enable),
        .target(target_r), .feedback(feedback_r), .kp(kp_r), .ki(ki_r), .kd(kd_r),
        .gain_shift(gain_shift_r), .limit(output_limit_r),
        .control(control_r), .sat(sat_r), .active(active_r));
endmodule

module speed_pi_pid_channel (
    input wire clk, rst_n, tick, enable,
    input wire signed [15:0] target, feedback,
    input wire signed [15:0] kp, ki, kd,
    input wire [4:0] gain_shift, input wire [15:0] limit,
    output reg signed [15:0] control, output reg sat, active
);
    reg signed [31:0] e1, e2;
    reg signed [63:0] p_term, i_term, d_term, delta_term, shifted_term;
    reg signed [63:0] error_now, de, derivative, candidate;
    reg signed [63:0] max_limit, min_limit;
    reg signed [63:0] control_ext;
    reg signed [63:0] limited;
    reg outward;

    always @* begin
        error_now = $signed(target) - $signed(feedback);
        de = error_now - $signed(e1);
        derivative = error_now - ($signed(e1) <<< 1) + $signed(e2);
        p_term = $signed(kp) * de;
        i_term = $signed(ki) * error_now;
        d_term = $signed(kd) * derivative;
        delta_term = p_term + i_term + d_term;
        shifted_term = (gain_shift >= 32) ? 64'sd0 : (delta_term >>> gain_shift);
        max_limit = {48'd0, limit};
        min_limit = -max_limit;
        control_ext = $signed(control);
        candidate = control_ext + shifted_term;
        outward = ((control_ext >= max_limit) && (error_now > 0)) ||
                  ((control_ext <= min_limit) && (error_now < 0));
        // Conditional integration: remove the integral contribution while
        // saturated and the current error would drive farther outward.
        if (outward) begin
            delta_term = p_term + d_term;
            shifted_term = (gain_shift >= 32) ? 64'sd0 : (delta_term >>> gain_shift);
            candidate = control_ext + shifted_term;
        end
        if (candidate > max_limit) begin limited = max_limit; end
        else if (candidate < min_limit) begin limited = min_limit; end
        else limited = candidate;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control <= 16'sd0; e1 <= 32'sd0; e2 <= 32'sd0;
            sat <= 1'b0; active <= 1'b0;
        end else if (!enable) begin
            control <= 16'sd0; e1 <= 32'sd0; e2 <= 32'sd0;
            sat <= 1'b0; active <= 1'b0;
        end else begin
            active <= 1'b1;
            if (tick) begin
                control <= limited[15:0];
                e2 <= e1[31:0];
                e1 <= error_now[31:0];
                sat <= (limited >= max_limit) || (limited <= min_limit);
            end
        end
    end
endmodule
