`timescale 1ns/1ps
// t4_lite_drive 速度闭环物理仿真：RTL 控制链 + 一阶电机/编码器被控对象模型。
//
// 覆盖：SET_VEL(mm/s) -> t4_wheel_speed_loop -> tb6612_driver_interface -> 电机对象
//       -> 编码器增量回灌 -> 测速 -> PI，验证：
//   V1 使能前 STBY=0（不动车）
//   V2 50 mm/s 前进：左右实测轮速 ~50 mm/s，方向正确，duty 在限幅内
//   V3 100 mm/s 前进：~100 mm/s
//   V4 原地转 +/-50：左 +50 / 右 -50（差速 100 mm/s）
//   V5 后退 -50：方向翻转
//   V6 SET_VEL(0,0)：duty 归零、轮速衰减
//   V7 enable=0：立即断 duty / STBY=0（安全门关断）
//   V8 300 mm/s 满量程：~300 mm/s 且 duty 未顶到 50% 限幅
//
// 时间基准：真实 125 MHz / 20 kHz PWM / 20 ms 速度窗口 —— 不做时间缩放，
// 保证 duty 分辨率（1/6250）与 counts/窗口 量化（50 mm/s = 7.13 counts）与上板一致。
// 被控对象一阶模型由旧开环实测标定：duty≈1000 tick 时稳态 ~110 mm/s
// -> 增益 0.1222 mm/s/tick，并含 100 tick（1.6% duty）静摩擦死区、50 ms 时间常数。
// 这是“控制律行为”仿真，不是电机参数的最终标定；上板后可用遥测 speed_l/r 复核。
module tb_wheel_speed_loop;
    localparam integer CLK_HZ        = 125_000_000;
    localparam integer PWM_HZ        = 20_000;
    localparam integer PWM_PERIOD    = CLK_HZ / PWM_HZ;      // 6250
    localparam integer WINDOW_CYCLES = 2_500_000;            // 20 ms
    localparam integer COUNTS_PER_REV = 1456;
    localparam integer UM_PER_REV     = 204203;
    localparam real    UM_PER_COUNT   = 204203.0 / 1456.0;   // 140.25 um/count
    localparam integer DUTY_LIMIT     = PWM_PERIOD / 2;      // 3125（与顶层 SPEED_DUTY_DIV=2 一致）
    // 被控对象（旧开环实测标定 + 典型小直流减速电机参数）
    localparam real    PLANT_GAIN = 0.1222;   // mm/s per duty tick（扣死区）
    localparam integer PLANT_DEAD = 100;      // duty tick 静摩擦死区
    localparam real    PLANT_TAU  = 0.05;     // s
    localparam real    PERIOD_S   = 1.0 / PWM_HZ;

    reg clk = 0;
    always #4 clk = ~clk;                     // 125 MHz
    reg rst_n = 0;
    reg enable = 0;
    reg signed [15:0] target_l = 0, target_r = 0;
    reg signed [1:0]  enc_l = 0, enc_r = 0;

    wire signed [15:0] pwm_l, pwm_r, ctrl_l, ctrl_r;
    wire signed [31:0] spd_l, spd_r;
    wire spd_valid;

    t4_wheel_speed_loop #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ),
        .WINDOW_CYCLES(WINDOW_CYCLES), .FILTER_LENGTH(4),
        .COUNTS_PER_REV(COUNTS_PER_REV), .UM_PER_REV(UM_PER_REV),
        .KP(16), .KI(4), .KD(0), .GAIN_SHIFT(0),
        .DUTY_DIV(2), .RAMP_STEP(1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .target_l_mm_s(target_l), .target_r_mm_s(target_r),
        .delta_count_l(enc_l), .delta_count_r(enc_r),
        .pwm_l(pwm_l), .pwm_r(pwm_r),
        .control_l(ctrl_l), .control_r(ctrl_r),
        .speed_count_l(spd_l), .speed_count_r(spd_r),
        .speed_valid(spd_valid)
    );

    wire PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY;
    tb6612_driver_interface #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ), .DIR_DEADTIME_CYCLES(2500)
    ) drv (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .emergency_stop(1'b0), .fault(1'b0), .watchdog_timeout(1'b0),
        .pwm_signed_l(pwm_l), .pwm_signed_r(pwm_r),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2),
        .PWMB(PWMB), .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY)
    );

    // ------------------------------------------------------- 电机 + 编码器对象
    real    v_l = 0.0, v_r = 0.0;          // 真实轮速 mm/s
    real    dist_um_l = 0.0, dist_um_r = 0.0;
    integer cnt_l = 0, cnt_r = 0;          // 已发出的累计编码器 count

    task automatic plant_step;
        real duty_l, duty_r, vcmd_l, vcmd_r;
        integer new_l, new_r;
        begin
            duty_l = pwm_l;
            duty_r = pwm_r;
            if (duty_l > DUTY_LIMIT) duty_l = DUTY_LIMIT;
            if (duty_l < -DUTY_LIMIT) duty_l = -DUTY_LIMIT;
            if (duty_r > DUTY_LIMIT) duty_r = DUTY_LIMIT;
            if (duty_r < -DUTY_LIMIT) duty_r = -DUTY_LIMIT;
            // 一阶 + 静摩擦死区
            if (duty_l > PLANT_DEAD)       vcmd_l =  PLANT_GAIN * (duty_l - PLANT_DEAD);
            else if (duty_l < -PLANT_DEAD) vcmd_l = -PLANT_GAIN * (-duty_l - PLANT_DEAD);
            else                           vcmd_l = 0.0;
            if (duty_r > PLANT_DEAD)       vcmd_r =  PLANT_GAIN * (duty_r - PLANT_DEAD);
            else if (duty_r < -PLANT_DEAD) vcmd_r = -PLANT_GAIN * (-duty_r - PLANT_DEAD);
            else                           vcmd_r = 0.0;
            v_l = v_l + (vcmd_l - v_l) * (PERIOD_S / PLANT_TAU);
            v_r = v_r + (vcmd_r - v_r) * (PERIOD_S / PLANT_TAU);
            dist_um_l = dist_um_l + v_l * PERIOD_S * 1000.0;
            dist_um_r = dist_um_r + v_r * PERIOD_S * 1000.0;
            // 位置量化成编码器 count，只在变化时发一拍 ±1 增量
            new_l = $floor(dist_um_l / UM_PER_COUNT);
            new_r = $floor(dist_um_r / UM_PER_COUNT);
            if (new_l > cnt_l) enc_l <= 2'sd1;
            else if (new_l < cnt_l) enc_l <= -2'sd1;
            else enc_l <= 2'sd0;
            if (new_r > cnt_r) enc_r <= 2'sd1;
            else if (new_r < cnt_r) enc_r <= -2'sd1;
            else enc_r <= 2'sd0;
            cnt_l = new_l;
            cnt_r = new_r;
        end
    endtask

    integer pwm_div = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_div <= 0;
        end else begin
            if (pwm_div == PWM_PERIOD - 1) pwm_div <= 0;
            else pwm_div <= pwm_div + 1;
        end
    end
    // 注意：编码器增量必须是**单拍脉冲**。plant_step 里用非阻塞赋值给出 ±1，
    // 这里每拍默认清零，否则增量会一直保持 6250 拍，测速会严重过计数。
    always @(posedge clk) begin
        enc_l <= 2'sd0;
        enc_r <= 2'sd0;
        if (rst_n && pwm_div == 0) plant_step;
    end

    // ------------------------------------------------------------- 窗口记录
    localparam integer MAXW = 512;
    real    vlog_l [0:MAXW-1];
    real    vlog_r [0:MAXW-1];
    integer flog_l [0:MAXW-1];
    integer flog_r [0:MAXW-1];
    integer clog_l [0:MAXW-1];
    integer clog_r [0:MAXW-1];
    integer plog_l [0:MAXW-1];
    integer plog_r [0:MAXW-1];
    integer widx = 0;
    always @(posedge clk) begin
        if (rst_n && spd_valid && widx < MAXW) begin
            vlog_l[widx] = v_l;  vlog_r[widx] = v_r;
            flog_l[widx] = spd_l; flog_r[widx] = spd_r;
            clog_l[widx] = ctrl_l; clog_r[widx] = ctrl_r;
            plog_l[widx] = pwm_l;  plog_r[widx] = pwm_r;
            widx = widx + 1;
`ifdef SIM_VERBOSE
            $display("win %0d: filt=%0d/%0d ctrl=%0d/%0d pwm=%0d/%0d v=%.2f/%.2f tgt=%0d/%0d",
                     widx, spd_l, spd_r, ctrl_l, ctrl_r, pwm_l, pwm_r, v_l, v_r,
                     target_l, target_r);
`endif
        end
    end

    function automatic real avg_real(input integer a, input integer b, input integer which_lr);
        integer k; real s;
        begin
            s = 0.0;
            for (k = a; k < b; k = k + 1) s = s + (which_lr == 0 ? vlog_l[k] : vlog_r[k]);
            avg_real = s / (b - a);
        end
    endfunction
    function automatic real avg_int_l(input integer a, input integer b);
        integer k; real s;
        begin
            s = 0.0;
            for (k = a; k < b; k = k + 1) s = s + clog_l[k];
            avg_int_l = s / (b - a);
        end
    endfunction

    integer errors = 0, checks = 0;
    real vl, vr;
    task automatic check(input bit cond, input string msg);
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s", msg);
        end else $display("PASS %s", msg);
    endtask

    task automatic wait_windows(input integer n);
        integer guard;
        begin
            guard = 0;
            while (widx < n && guard < 500_000_000) begin
                @(posedge clk);
                guard = guard + 1;
            end
        end
    endtask

    integer w;
`ifdef SIM_SHORT
    reg short_mode = 1'b1;
`else
    reg short_mode = 1'b0;
`endif
    initial begin
        repeat (20) @(posedge clk);
        rst_n = 1;
        wait_windows(2);                      // 滤波器预热（enable=0）

        // ---------------- V1 使能前
        check(STBY == 1'b0, "V1_disabled_stby_low");
        check(pwm_l == 0 && pwm_r == 0, "V1_disabled_duty_zero");

`ifdef SIM_V8ONLY
        // 诊断模式：直接 300 mm/s，观察满量程跟踪是否收敛
        enable = 1;
        target_l = 16'sd300; target_r = 16'sd300;
        w = widx + 70; wait_windows(w);
        $display("V8ONLY: v=(%.2f, %.2f) mm/s pwm=(%0d, %0d) filt=(%0d, %0d)",
                 v_l, v_r, pwm_l, pwm_r, spd_l, spd_r);
        $finish;
`endif

`ifdef SIM_NEGONLY
        // 诊断模式：直接 -50 mm/s，验证负目标/负测速/反向 duty 通路
        enable = 1;
        target_l = -16'sd50; target_r = -16'sd50;
        w = widx + 45; wait_windows(w);
        $display("NEGONLY: v=(%.2f, %.2f) mm/s pwm=(%0d, %0d) filt=(%0d, %0d) ctrl=(%0d, %0d)",
                 v_l, v_r, pwm_l, pwm_r, spd_l, spd_r, ctrl_l, ctrl_r);
        $finish;
`endif

        // ---------------- V2 50 mm/s 前进
        enable = 1;
        target_l = 16'sd50; target_r = 16'sd50;
        w = widx + (short_mode ? 8 : 45);
        wait_windows(w);
        if (short_mode) begin
            $display("SHORT: window=%0d filt=%0d/%0d ctrl=%0d/%0d pwm=%0d/%0d v=%.2f/%.2f",
                     widx, spd_l, spd_r, ctrl_l, ctrl_r, pwm_l, pwm_r, v_l, v_r);
            $finish;
        end
        vl = avg_real(widx-8, widx, 0);
        vr = avg_real(widx-8, widx, 1);
        $display("V2 50mm/s: true v=(%.2f, %.2f) mm/s  filt=(%0d, %0d) counts/win  ctrl=(%0d, %0d)  pwm=(%0d, %0d)",
                 vl, vr, spd_l, spd_r, ctrl_l, ctrl_r, pwm_l, pwm_r);
        check(vl > 46.0 && vl < 53.0, $sformatf("V2_left_speed_50 (%.2f)", vl));
        check(vr > 46.0 && vr < 53.0, $sformatf("V2_right_speed_50 (%.2f)", vr));
        check(AIN1 == 1'b1 && AIN2 == 1'b0, "V2_left_direction_forward");
        check(BIN1 == 1'b1 && BIN2 == 1'b0, "V2_right_direction_forward");
        check(spd_l >= 6 && spd_l <= 8, $sformatf("V2_left_counts_per_window=%0d", spd_l));
        check(pwm_l > 0 && pwm_l < DUTY_LIMIT && pwm_r > 0 && pwm_r < DUTY_LIMIT,
              $sformatf("V2_duty_inside_ceiling (pwm=%0d/%0d, limit=%0d)", pwm_l, pwm_r, DUTY_LIMIT));

        // ---------------- V3 100 mm/s
        target_l = 16'sd100; target_r = 16'sd100;
        w = widx + 35; wait_windows(w);
        vl = avg_real(widx-8, widx, 0); vr = avg_real(widx-8, widx, 1);
        $display("V3 100mm/s: true v=(%.2f, %.2f) mm/s  pwm=(%0d, %0d)", vl, vr, pwm_l, pwm_r);
        check(vl > 90.0 && vl < 108.0, $sformatf("V3_left_speed_100 (%.2f)", vl));
        check(vr > 90.0 && vr < 108.0, $sformatf("V3_right_speed_100 (%.2f)", vr));

        // ---------------- V4 原地转 +50 / -50
        // 从 100 mm/s 直接切到差速 ±50：右轮要跨零换向（100 -> -50），给足 90 个窗口。
        target_l = 16'sd50; target_r = -16'sd50;
        w = widx + 90; wait_windows(w);
        vl = avg_real(widx-8, widx, 0); vr = avg_real(widx-8, widx, 1);
        $display("V4 turn +/-50: true v=(%.2f, %.2f) mm/s  diff=%.2f  -> yaw %.1f deg/s @240mm track",
                 vl, vr, vl - vr, (vr - vl) / 240.0 * 57.2957795);
        check(vl > 46.0 && vl < 56.0, $sformatf("V4_left_speed_+50 (%.2f)", vl));
        check(vr < -46.0 && vr > -56.0, $sformatf("V4_right_speed_-50 (%.2f)", vr));
        check(AIN1 == 1'b1 && AIN2 == 1'b0, "V4_left_direction_forward");
        check(BIN1 == 1'b0 && BIN2 == 1'b1, "V4_right_direction_reverse");

        // ---------------- V5 后退 -50
        // 从 (+50,-50) 切到 (-50,-50)：左轮要跨零换向，给足 90 个窗口。
        target_l = -16'sd50; target_r = -16'sd50;
        w = widx + 90; wait_windows(w);
        vl = avg_real(widx-8, widx, 0); vr = avg_real(widx-8, widx, 1);
        $display("V5 reverse: true v=(%.2f, %.2f) mm/s", vl, vr);
        check(vl < -46.0 && vl > -56.0, $sformatf("V5_left_speed_-50 (%.2f)", vl));
        check(vr < -46.0 && vr > -56.0, $sformatf("V5_right_speed_-50 (%.2f)", vr));
        check(AIN1 == 1'b0 && AIN2 == 1'b1, "V5_reverse_direction");

        // ---------------- V6 SET_VEL(0,0)：duty 归零
        target_l = 0; target_r = 0;
        w = widx + 8; wait_windows(w);
        $display("V6 stop: pwm=(%0d, %0d) v=(%.2f, %.2f)", pwm_l, pwm_r, v_l, v_r);
        check(pwm_l == 0 && pwm_r == 0, "V6_zero_target_duty_zero");
        check(v_l > -5.0 && v_l < 5.0, $sformatf("V6_wheel_stopped (%.2f)", v_l));

        // ---------------- V7 enable=0（安全门关断）
        enable = 1; target_l = 16'sd50; target_r = 16'sd50;
        w = widx + 10; wait_windows(w);
        enable = 0;
        repeat (10) @(posedge clk);
        check(pwm_l == 0 && pwm_r == 0, "V7_disable_duty_zero");
        check(STBY == 1'b0, "V7_disable_stby_low");

        // ---------------- V8 300 mm/s 满量程（MAX_V_MM_S）
        enable = 1;
        target_l = 16'sd300; target_r = 16'sd300;
        w = widx + 50; wait_windows(w);
        vl = avg_real(widx-8, widx, 0); vr = avg_real(widx-8, widx, 1);
        $display("V8 300mm/s: true v=(%.2f, %.2f) mm/s  pwm=(%0d, %0d)  limit=%0d",
                 vl, vr, pwm_l, pwm_r, DUTY_LIMIT);
        check(vl > 285.0 && vl < 320.0, $sformatf("V8_left_speed_300 (%.2f)", vl));
        check(vr > 285.0 && vr < 320.0, $sformatf("V8_right_speed_300 (%.2f)", vr));
        check(pwm_l < DUTY_LIMIT && pwm_r < DUTY_LIMIT, "V8_duty_not_clipped_at_ceiling");

        $display("");
        if (errors == 0) $display("PASS tb_wheel_speed_loop (%0d checks)", checks);
        else             $display("FAIL tb_wheel_speed_loop (%0d errors, %0d checks)", errors, checks);
        $finish;
    end

    initial begin
        #(64'd9_000_000_000);   // 9 s 仿真时间保护（真实时间基准，全部场景约 6.6 s）
        $display("FAIL tb_wheel_speed_loop timeout at window %0d", widx);
        $finish;
    end
endmodule
