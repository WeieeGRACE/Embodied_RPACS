`timescale 1ns/1ps
// tests/t4_lite_drive —— 双轮速度闭环：SET_VEL(mm/s) -> 编码器测速 -> PI -> TB6612 duty。
//
// 这是对旧 `eth_motion_bridge::to_duty`（duty = v*20，开环）的替换。数据流：
//
//   delta_count(+/-1 每 clk)
//        -> speed_measurement_and_velocity_filter   （主工程原件：窗口 counts 测速）
//        -> mm/s -> counts/窗口 换算（Q24 常数倒数，四舍五入）
//        -> dual_speed_pi_pid_controller            （主工程原件：增量式 PI/PID）
//        -> pwm_and_startup_ramp                    （主工程原件：duty 斜率限制）
//        -> signed duty -> tb6612_driver_interface  （顶层例化，未改动）
//
// 三个子模块都是从主工程 src/motor/ 原样复制的文件（文件名/模块名/参数名不变），
// 本文件只负责参数换算与互联，不改变任何安全门控（motors_enabled / STOP / KEY2 /
// ToF / 链路故障全部在 t4_lite_top 里原样保留，且本模块的 enable 就是那些门的合成，
// 关断时 PI 清零、ramp 回到 0 duty）。
//
// 控制律（与主工程 system_motion.v 的接法一致）：
//   control[n+1] = clamp( control[n] + (KP*de + KI*e + KD*dd) >> GAIN_SHIFT, +/-DUTY_LIMIT )
//   e    = target_counts_per_window - filtered_counts_per_window
//   de   = e[n] - e[n-1]；dd = e[n] - 2e[n-1] + e[n-2]
//
// 参数标定（默认值，125 MHz / 20 kHz PWM / 65 mm 轮 / 1456 counts/rev）：
//   * 速度窗口 20 ms：主工程用 10 ms；50 mm/s 在 20 ms 窗口 = 7.13 counts，
//     四舍五入到 7 counts = 49.1 mm/s（-1.8%）。若用 10 ms 窗口只有 3.56 counts，
//     取整后是 4 counts = 56.1 mm/s（+12%）——20 ms 窗口让 SET_VEL(50) 更接近 50。
//   * KP=32 / KI=1 / KD=0：主工程 system_motion.v 的出厂参数（KI 以“每窗口”计）。
//   * DUTY_DIV=2：PI/ramp 的 duty 上限 = PWM_PERIOD/2 = 3125 拍（50%）。
//     主工程是 PWM_PERIOD/4（25%）；本工程 MAX_V_MM_S=300，按实测开环增益
//     （50 mm/s 对应约 455 拍 ≈ 9.1 拍/(mm/s)）50% 对应约 343 mm/s，仍覆盖满量程，
//     同时给“编码器失效 -> 积分灌满”留了 50% 的限幅余量（不会 100% 油门）。
//   * RAMP_STEP=1：duty 每个 PWM 周期最多变化 1 拍（主工程同值），启动/换向无冲击。
module t4_wheel_speed_loop #(
    parameter integer CLK_HZ        = 125_000_000,
    parameter integer PWM_HZ        = 20000,
    // 速度窗口长度 = WINDOW_CYCLES / CLK_HZ 秒。要求 WINDOW_CYCLES*1000 能被 CLK_HZ 整除
    // （默认 2_500_000 @125 MHz = 20 ms）。FILTER_LENGTH=4 为主工程取值。
    parameter integer WINDOW_CYCLES = CLK_HZ / 50,
    parameter integer FILTER_LENGTH = 4,
    parameter integer COUNTS_PER_REV = 1456,
    parameter integer UM_PER_REV     = 204203,
    // PI 参数（主工程 system_motion.v 出厂值）
    parameter signed [15:0] KP = 32,
    parameter signed [15:0] KI = 1,
    parameter signed [15:0] KD = 0,
    parameter integer GAIN_SHIFT = 0,
    // duty 上限 = PWM_PERIOD / DUTY_DIV；ramp 斜率 = 每 PWM 周期多少 tick
    parameter integer DUTY_DIV  = 2,
    parameter integer RAMP_STEP = 1
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,          // 安全门合成（motors_enable_gated）
    input  wire signed [15:0] target_l_mm_s,   // SET_VEL 目标轮速（已限幅）
    input  wire signed [15:0] target_r_mm_s,
    input  wire signed [1:0]  delta_count_l,   // 左编码器增量（每 clk）
    input  wire signed [1:0]  delta_count_r,   // 右编码器增量（每 clk）
    output wire signed [15:0] pwm_l,           // 有符号 duty（tick，给 tb6612）
    output wire signed [15:0] pwm_r,
    output wire signed [15:0] control_l,       // PI 原始输出（调试）
    output wire signed [15:0] control_r,
    output wire signed [31:0] speed_count_l,   // 滤波后实测（counts/窗口，调试）
    output wire signed [31:0] speed_count_r,
    output wire               speed_valid      // 每速度窗口一个单拍脉冲
);
    // ------------------------------------------------------- 常数换算（综合时常量折叠）
    localparam integer PWM_PERIOD = CLK_HZ / ((PWM_HZ > 0) ? PWM_HZ : 1);
    localparam integer DUTY_LIMIT = PWM_PERIOD / ((DUTY_DIV > 0) ? DUTY_DIV : 1);
    // 1 mm/s 在一个窗口内走过的距离（um）：WINDOW_CYCLES/CLK_HZ * 1e6 um/s * 1e-3 s/ms
    // 注意用 64 位中间量：20 ms @125 MHz 的 WINDOW_CYCLES*1000 = 2.5e9 会溢出 32 位。
    localparam integer UM_PER_WINDOW_PER_MMPS = (64'd1000 * WINDOW_CYCLES) / CLK_HZ;
    // counts/窗口 = v_mm_s * UM_PER_WINDOW_PER_MMPS * COUNTS_PER_REV / UM_PER_REV
    // 用 Q10 定点比例（146/1024 ≈ 0.1426 counts/(mm/s)/窗口，误差 -0.02%）：
    // 3 个移位加法即可完成换算，避免 64 位乘法/除法关键路径（125 MHz 单拍）。
    localparam [31:0] TARGET_SCALE_Q10 =
        (64'd1024 * COUNTS_PER_REV * UM_PER_WINDOW_PER_MMPS + (UM_PER_REV / 2)) / UM_PER_REV;
    localparam [4:0]  GAIN_SHIFT_5    = GAIN_SHIFT;
    localparam [15:0] DUTY_LIMIT_16   = DUTY_LIMIT;
    localparam [15:0] RAMP_STEP_16    = RAMP_STEP;

    // synthesis translate_off
    initial begin
        if (CLK_HZ <= 0 || PWM_HZ <= 0 || WINDOW_CYCLES < 2 || FILTER_LENGTH < 1 ||
            FILTER_LENGTH > 16 || COUNTS_PER_REV < 1 || UM_PER_REV < 1) begin
            $display("ERROR: t4_wheel_speed_loop parameter error");
            $fatal(1);
        end
        if (((64'd1000 * WINDOW_CYCLES) % CLK_HZ) != 0) begin
            $display("ERROR: WINDOW_CYCLES/CLK_HZ must give an integer us-per-mm/s scale");
            $fatal(1);
        end
    end
    // synthesis translate_on

    // ------------------------------------------------------------- 编码器测速
    // 主工程接法：2 位有符号增量符号扩展到 16 位。
    wire signed [31:0] speed_raw_l, speed_raw_r;
    wire signed [31:0] speed_filt_l, speed_filt_r;
    wire               valid_l, valid_r;

    speed_measurement_and_velocity_filter #(
        .CLK_HZ(CLK_HZ),
        .WINDOW_CYCLES(WINDOW_CYCLES),
        .FILTER_LENGTH(FILTER_LENGTH),
        .ZERO_PAD_WARMUP(1)
    ) u_speed_l (
        .clk(clk), .rst_n(rst_n),
        .delta_count({{14{delta_count_l[1]}}, delta_count_l}),
        .speed_raw(speed_raw_l), .speed_filtered(speed_filt_l),
        .position(), .speed_valid(valid_l), .direction(), .zero_speed()
    );
    speed_measurement_and_velocity_filter #(
        .CLK_HZ(CLK_HZ),
        .WINDOW_CYCLES(WINDOW_CYCLES),
        .FILTER_LENGTH(FILTER_LENGTH),
        .ZERO_PAD_WARMUP(1)
    ) u_speed_r (
        .clk(clk), .rst_n(rst_n),
        .delta_count({{14{delta_count_r[1]}}, delta_count_r}),
        .speed_raw(speed_raw_r), .speed_filtered(speed_filt_r),
        .position(), .speed_valid(valid_r), .direction(), .zero_speed()
    );

    assign speed_count_l = speed_filt_l;
    assign speed_count_r = speed_filt_r;
    // 两个滤波器完全同构、同窗口，valid 同拍；anding 与主工程 system_motion 一致。
    wire control_tick = valid_l && valid_r;
    assign speed_valid = control_tick;

    // feedback 饱和到 16 位（主工程 sat16(speed_filtered) 同义）后再打一拍，
    // 让 PI 的输入全部来自寄存器，缩短 125 MHz 关键路径。
    function signed [15:0] sat16_counts;
        input signed [31:0] value;
        begin
            if (value > 32'sd32767)       sat16_counts = 16'sh7fff;
            else if (value < -32'sd32768) sat16_counts = -16'sd32768;
            else                          sat16_counts = value[15:0];
        end
    endfunction
    reg signed [15:0] feedback_l, feedback_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            feedback_l <= 16'sd0;
            feedback_r <= 16'sd0;
        end else begin
            feedback_l <= sat16_counts(speed_filt_l);
            feedback_r <= sat16_counts(speed_filt_r);
        end
    end

    // ------------------------------------------------- 目标 mm/s -> counts/窗口
    // 有符号 16 位输入 -> 计数（实际只用 ±MAX_V_MM_S=±300）；结果饱和到 int16。
    // 拆成 3 级流水（输入寄存 -> 绝对值寄存 -> 乘常数寄存 -> 舍入/饱和输出），
    // 每级只有 1 个短组合逻辑，满足 125 MHz（原主工程在 50 MHz 下单拍完成）。
    // 乘法比例 TARGET_SCALE_Q10 = 146（Q10），综合时按 17 位×32 位常数乘法实现。
    reg signed [15:0] target_l_q, target_r_q;                  // 1: 输入寄存
    reg [16:0]        tgt_mag_l, tgt_mag_r;                    // 2: |v|（-32768 -> 32768）
    reg               tgt_neg_l, tgt_neg_r;                    // 2: 符号
    reg [31:0]        tgt_prod_l, tgt_prod_r;                  // 3: |v| * SCALE_Q10
    reg signed [15:0] target_l_counts, target_r_counts;        // 4: counts/窗口
    // 显式符号扩展到 17 位（signed），供绝对值计算
    wire signed [16:0] tgt_l_ext = {{1{target_l_q[15]}}, target_l_q};
    wire signed [16:0] tgt_r_ext = {{1{target_r_q[15]}}, target_r_q};

    // round(x/1024) + int16 饱和（乘法结果寄存器 -> 计数寄存器之间的短组合）
    function [15:0] sat_cnt16;
        input [31:0] prod;
        reg [31:0] q;
        begin
            q = (prod + 32'd512) >> 10;
            if (q > 32'd32767) q = 32'd32767;
            sat_cnt16 = q[15:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_l_q  <= 16'sd0;  target_r_q  <= 16'sd0;
            tgt_mag_l   <= 17'd0;   tgt_mag_r   <= 17'd0;
            tgt_neg_l   <= 1'b0;    tgt_neg_r   <= 1'b0;
            tgt_prod_l  <= 32'd0;   tgt_prod_r  <= 32'd0;
            target_l_counts <= 16'sd0; target_r_counts <= 16'sd0;
        end else begin
            target_l_q <= target_l_mm_s;
            target_r_q <= target_r_mm_s;
            // 绝对值：先显式符号扩展到 17 位（signed），再取反。
            // 不能用 `~target_q + 17'd1`：17'd1 是 unsigned，会把 ~ 操作数按
            // 无符号零扩展，-50 会算成 65586（仿真已实测该坑）。
            tgt_mag_l  <= tgt_l_ext[16] ? (17'sd0 - tgt_l_ext) : tgt_l_ext;
            tgt_mag_r  <= tgt_r_ext[16] ? (17'sd0 - tgt_r_ext) : tgt_r_ext;
            tgt_neg_l  <= target_l_q[15];
            tgt_neg_r  <= target_r_q[15];
            // 常数乘法单独一拍（乘法器前后都有寄存器）
            tgt_prod_l <= tgt_mag_l * TARGET_SCALE_Q10;
            tgt_prod_r <= tgt_mag_r * TARGET_SCALE_Q10;
            // counts = round(|v|*SCALE/1024)，饱和到 int16，再按符号还原
            target_l_counts <= tgt_neg_l ? -$signed({1'b0, sat_cnt16(tgt_prod_l)})
                                         :  $signed({1'b0, sat_cnt16(tgt_prod_l)});
            target_r_counts <= tgt_neg_r ? -$signed({1'b0, sat_cnt16(tgt_prod_r)})
                                         :  $signed({1'b0, sat_cnt16(tgt_prod_r)});
        end
    end

    // ------------------------------------------------ 使能：安全门 + 零目标清零
    // 与主工程 motion FSM 的 controller_clear 同义：SET_VEL(0,0) 视为“停车”，
    // PI 清零、ramp 目标归零（避免积分残留让停住的轮子继续爬行）。
    // 使能再打一拍：`target_limited -> 比较 -> enable` 从 125 MHz 关键路径里移出去，
    // 代价是停/启环路晚 1 个 clk（8 ns）；tb6612 的 STBY 仍由 motors_enable_gated
    // 直接控制，安全门本身没有变慢。
    reg control_enable;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) control_enable <= 1'b0;
        else        control_enable <= enable &&
                                      ((target_l_mm_s != 16'sd0) || (target_r_mm_s != 16'sd0));
    end

    // ---------------------------------------------------------------- PI/PID
    dual_speed_pi_pid_controller u_pi (
        .clk(clk), .rst_n(rst_n),
        .control_tick(control_tick),
        .enable(control_enable),
        .target_l(target_l_counts), .feedback_l(feedback_l),
        .target_r(target_r_counts), .feedback_r(feedback_r),
        .kp_l(KP), .ki_l(KI), .kd_l(KD),
        .kp_r(KP), .ki_r(KI), .kd_r(KD),
        .gain_shift_l(GAIN_SHIFT_5), .gain_shift_r(GAIN_SHIFT_5),
        .output_limit_l(DUTY_LIMIT_16), .output_limit_r(DUTY_LIMIT_16),
        .control_l(control_l), .control_r(control_r),
        .output_valid(), .sat_l(), .sat_r(), .active_l(), .active_r()
    );

    // ------------------------------------------------------- duty 启动/换向斜坡
    pwm_and_startup_ramp #(.CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ)) u_ramp (
        .clk(clk), .rst_n(rst_n),
        .enable(control_enable),
        .pwm_cmd_l(control_l), .pwm_cmd_r(control_r),
        .pwm_limit(DUTY_LIMIT_16), .pwm_min(16'd0), .ramp_step(RAMP_STEP_16),
        .pwm_out_l(), .pwm_out_r(),
        .pwm_actual_l(pwm_l), .pwm_actual_r(pwm_r)
    );
endmodule
