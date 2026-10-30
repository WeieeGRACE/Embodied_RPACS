`timescale 1ns/1ps
// 速度目标 → TB6612 占空比（有符号，单位=时钟 tick），并做统一使能门控。
//
// 供 eth_motion_top 使用：链路 SET_VEL 解出的 target_v_left/right（mm/s，已被
// SET_LIMIT 限幅）在这里线性映射成 tb6612_driver_interface 需要的 signed duty，
// 并把「链路/故障/急停/停车」合成一个 motors_enabled。
module eth_motion_bridge #(
    parameter integer MAX_V_MM_S = 300,     // 满占空比对应的轮速
    parameter integer PWM_TICKS  = 6250,    // tb6612 的 PWM 周期（tick）
    parameter integer SCALE_NUM  = 20       // duty = v * SCALE_NUM（300*20=6000 < 6250）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] target_l,     // mm/s
    input  wire signed [15:0] target_r,     // mm/s
    input  wire [7:0]         fault_bits,   // 链路故障位
    input  wire               link_ok,      // 心跳存活
    input  wire               stop_latched, // STOP 锁存
    input  wire               key_estop_n,  // 板上急停键（低有效）
    output reg signed [15:0]  pwm_l,
    output reg signed [15:0]  pwm_r,
    output wire               motors_enabled,
    output wire signed [15:0] v_limited_l,
    output wire signed [15:0] v_limited_r
);
    // ---------------------------------------------------------- 速度限幅
    localparam signed [15:0] V_MAX = MAX_V_MM_S;
    localparam signed [15:0] V_MIN = -MAX_V_MM_S;

    function signed [15:0] clamp_v(input signed [15:0] v);
        begin
            if (v > V_MAX)      clamp_v = V_MAX;
            else if (v < V_MIN) clamp_v = V_MIN;
            else                clamp_v = v;
        end
    endfunction

    wire signed [15:0] vl = clamp_v(target_l);
    wire signed [15:0] vr = clamp_v(target_r);
    assign v_limited_l = vl;
    assign v_limited_r = vr;

    // ------------------------------------------------------ 速度 → 占空比
    // duty = v * SCALE_NUM，再限到 ±(PWM_TICKS-1)
    function signed [15:0] to_duty(input signed [15:0] v);
        reg signed [31:0] p;
        begin
            p = v * SCALE_NUM;
            if (p > (PWM_TICKS - 1))       to_duty = PWM_TICKS - 1;
            else if (p < -(PWM_TICKS - 1)) to_duty = -(PWM_TICKS - 1);
            else                           to_duty = p[15:0];
        end
    endfunction

    // 一级流水寄存器：把「限幅」和「×SCALE_NUM + 饱和」拆成两个 125 MHz 周期
    // 内的短路径。原实现是一条 "target_v -> clamp -> *SCALE_NUM -> 饱和 -> pwm"
    // 的 ~11 ns 组合链（PnR SETUP slack -0.8 ns），在 125 MHz(8 ns) 域下无法收敛。
    // 这里只增加 1 个时钟周期（8 ns）的目标更新延迟，占空比映射关系与使能门控
    // 完全不变（安全逻辑未改动）。
    reg signed [15:0] vl_q, vr_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vl_q <= 16'sd0;
            vr_q <= 16'sd0;
        end else begin
            vl_q <= vl;
            vr_q <= vr;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_l <= 16'sd0;
            pwm_r <= 16'sd0;
        end else begin
            pwm_l <= to_duty(vl_q);
            pwm_r <= to_duty(vr_q);
        end
    end

    // ------------------------------------------------------------ 使能门控
    // 只有「链路活着 + 无任何故障位 + 未停车 + 急停键未按下」才允许驱动电机
    assign motors_enabled = rst_n && link_ok && !stop_latched &&
                            (fault_bits == 8'h00) && key_estop_n;
endmodule
