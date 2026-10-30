`timescale 1ns/1ps
// 双通道有符号定点增量式 PI/PID（t4_lite_drive 本地副本）。
//
// 来源：主工程 src/motor/dual_speed_pi_pid_controller.v（50 MHz 版本）。
// 本副本保持**模块名/端口名/参数名/控制方程完全不变**，只做了两处 125 MHz 适配：
//   1) 组合逻辑拆成 3 级流水（误差/差分 -> P/I/D 项 -> 累加限幅），
//      控制量在 control_tick 之后 2 个 clk 更新（原版同拍更新）；
//      要求 control_tick 间隔 > 3 clk —— 本工程 20 ms 一拍，无影响；
//   2) 内部位宽按输入范围裁剪（17/19/40 位），不再使用 64 位加法链。
// 方程（与主工程一致）：
//   control[n+1] = clamp( control[n] + (KP*de + KI*e + KD*dd) >> gain_shift, ±limit )
//   e = target - feedback；de = e[n]-e[n-1]；dd = e[n]-2e[n-1]+e[n-2]
//   （饱和且误差继续向外时，去掉 I 项：条件积分）
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
    localparam integer EW = 17;    // e = target-feedback：-65535..65535
    localparam integer DW = 19;    // de / dd
    localparam integer TW = 40;    // P/I/D 项与增量（KP*de 等最坏 ~35 位）
    localparam [4:0]  SHIFT_MAX = 5'd31;

    reg signed [EW-1:0] e1, e2;    // 上一拍/上上拍误差
    reg signed [EW-1:0] e_a;       // 流水 1：e
    reg signed [EW-1:0] de_a;      // 流水 1：de
    reg signed [DW-1:0] dd_a;      // 流水 1：dd
    reg                outward_a;  // 流水 1：条件积分标志
    reg signed [TW-1:0] delta_b;   // 流水 2：本拍增量
    reg v1, v2, v3;                // 流水有效位

    // ---------------- 流水 1（输入全部是寄存器）：误差/差分/条件积分标志
    wire signed [EW-1:0] error_now = $signed(target) - $signed(feedback);
    wire signed [EW-1:0] de_now    = error_now - e1;
    wire signed [DW-1:0] dd_now    = $signed(error_now) - (e1 <<< 1) + $signed(e2);
    wire signed [TW-1:0] max_limit = {40'd0, limit};
    wire signed [TW-1:0] min_limit = -max_limit;
    wire signed [TW-1:0] control_ext = $signed(control);
    wire outward_now = ((control_ext >= max_limit) && (error_now > 0)) ||
                       ((control_ext <= min_limit) && (error_now < 0));

    // ---------------- 流水 2：P/I/D 三项与增量（乘常数在综合时折叠为移位/加法）
    wire signed [TW-1:0] p_term = $signed(kp) * $signed(de_a);
    wire signed [TW-1:0] i_term = $signed(ki) * $signed(e_a);
    wire signed [TW-1:0] d_term = $signed(kd) * $signed(dd_a);
    wire signed [TW-1:0] delta_full = p_term + i_term + d_term;
    wire signed [TW-1:0] delta_no_i = p_term + d_term;
    wire signed [TW-1:0] delta_sel  = outward_a ? delta_no_i : delta_full;
    wire signed [TW-1:0] delta_shift = (gain_shift >= SHIFT_MAX) ? {TW{1'b0}}
                                                                  : (delta_sel >>> gain_shift);

    // ---------------- 流水 3：累加、限幅、状态更新
    wire signed [TW-1:0] candidate = control_ext + delta_b;
    wire signed [TW-1:0] limited = (candidate > max_limit) ? max_limit :
                                   (candidate < min_limit) ? min_limit : candidate;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control <= 16'sd0; e1 <= {EW{1'b0}}; e2 <= {EW{1'b0}};
            sat <= 1'b0; active <= 1'b0;
            e_a <= {EW{1'b0}}; de_a <= {EW{1'b0}}; dd_a <= {DW{1'b0}};
            outward_a <= 1'b0; delta_b <= {TW{1'b0}};
            v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
        end else if (!enable) begin
            control <= 16'sd0; e1 <= {EW{1'b0}}; e2 <= {EW{1'b0}};
            sat <= 1'b0; active <= 1'b0;
            e_a <= {EW{1'b0}}; de_a <= {EW{1'b0}}; dd_a <= {DW{1'b0}};
            outward_a <= 1'b0; delta_b <= {TW{1'b0}};
            v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
        end else begin
            active <= 1'b1;
            v1 <= tick;
            v2 <= v1;
            v3 <= v2;
            if (tick) begin
                e_a       <= error_now;
                de_a      <= de_now;
                dd_a      <= dd_now;
                outward_a <= outward_now;
            end
            if (v1) delta_b <= delta_shift;
            if (v2) begin
                control <= limited[15:0];
                sat     <= (limited >= max_limit) || (limited <= min_limit);
                e2      <= e1;          // 本次采样前的 e[n-2]
                e1      <= e_a;         // 本次采样误差
            end
        end
    end
endmodule
