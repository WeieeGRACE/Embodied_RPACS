`timescale 1ns/1ps
// 轮速硬限幅：把导航控制器给出的左右轮目标速度钳在 ±LIMIT_MM_S 之内。
//
// 为什么放在顶层：导航段表、航向修正、后续调参都可能把目标速度算大，
// 这里做最后一道硬件闸门——**任何软件/表格错误都不可能让轮子超过这个速度**。
module nav_speed_limit #(
    parameter integer LIMIT_MM_S = 60
)(
    input  wire signed [31:0] velocity_l_mm_s,
    input  wire signed [31:0] velocity_r_mm_s,
    output wire signed [31:0] limited_l_mm_s,
    output wire signed [31:0] limited_r_mm_s,
    output wire               clamped_l,
    output wire               clamped_r
);
    localparam signed [31:0] LIMIT = LIMIT_MM_S;

    function automatic signed [31:0] clamp(input signed [31:0] v);
        begin
            if (v > LIMIT)       clamp = LIMIT;
            else if (v < -LIMIT) clamp = -LIMIT;
            else                 clamp = v;
        end
    endfunction
    localparam signed [31:0] NEG_LIMIT = -LIMIT_MM_S;

    assign limited_l_mm_s = clamp(velocity_l_mm_s);
    assign limited_r_mm_s = clamp(velocity_r_mm_s);
    assign clamped_l = (velocity_l_mm_s > LIMIT) || (velocity_l_mm_s < NEG_LIMIT);
    assign clamped_r = (velocity_r_mm_s > LIMIT) || (velocity_r_mm_s < NEG_LIMIT);

    // synthesis translate_off
    initial begin
        if (LIMIT_MM_S <= 0) $fatal(1, "nav_speed_limit needs a positive limit");
    end
    // synthesis translate_on
endmodule
