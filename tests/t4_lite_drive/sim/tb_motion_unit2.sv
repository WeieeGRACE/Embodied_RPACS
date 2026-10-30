`timescale 1ns/1ps
// eth_motion_bridge + tb6612_driver_interface 单元级自检（不涉及链路、不动车）。
//
// 覆盖：
//   U1 占空比-速度映射扫描（-300..+300，step 50，容差 ±2 拍）
//   U2 PWM 周期准确（PWM_TICKS 拍）
//   U3 方向位正确，且任何时刻不会 in1&in2 同时为 1（防直通）
//   U4 零速度：占空比 0、方向位全 0、STBY 仍为 1
//   U5 换向死区：PWM 立即屏蔽、死区后方向翻转、死区内不直通
//   U6 超速限幅（±300 以上饱和）
//   U7 差速/原地转（左右独立）
//   U8 motors_enabled 真值表（link_ok / stop / fault / estop 16 组合）
//   U9 四条安全路径都能拉低 STBY，恢复后可重新使能
module tb_eth_motion_unit;
    localparam integer CLK_HZ    = 125_000;
    localparam integer PWM_HZ    = 20;
    localparam integer PWM_TICKS = CLK_HZ / PWM_HZ;     // 6250
    localparam integer MAX_V     = 300;
    localparam integer SCALE     = PWM_TICKS / MAX_V;   // 20
    localparam integer DEADTIME  = 20;

    reg clk = 0;
    always #4 clk = ~clk;
    reg rst_n = 0;

    integer errors = 0, checks = 0;
    task automatic check(input bit cond, input string msg);
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s (t=%0t)", msg, $time);
        end else $display("PASS %s", msg);
    endtask

    // ---------------------------------------------------------------- DUT
    reg signed [15:0] target_l = 0, target_r = 0;
    reg  [7:0] fault_bits = 0;
    reg        link_ok = 1, stop_latched = 0, key_estop_n = 1;
    wire signed [15:0] pwm_l, pwm_r;
    wire motors_enabled;
    wire signed [15:0] vlim_l, vlim_r;

    eth_motion_bridge #(
        .MAX_V_MM_S(MAX_V), .PWM_TICKS(PWM_TICKS), .SCALE_NUM(SCALE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .target_l(target_l), .target_r(target_r),
        .fault_bits(fault_bits), .link_ok(link_ok),
        .stop_latched(stop_latched), .key_estop_n(key_estop_n),
        .pwm_l(pwm_l), .pwm_r(pwm_r), .motors_enabled(motors_enabled),
        .v_limited_l(vlim_l), .v_limited_r(vlim_r)
    );

    wire PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY;
    tb6612_driver_interface #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ), .DIR_DEADTIME_CYCLES(DEADTIME)
    ) drv (
        .clk(clk), .rst_n(rst_n),
        .enable(motors_enabled),
        .emergency_stop(!key_estop_n),
        .fault(fault_bits != 8'h00),
        .watchdog_timeout(!link_ok),
        .pwm_signed_l(pwm_l), .pwm_signed_r(pwm_r),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2),
        .PWMB(PWMB), .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY)
    );

    integer shoot_through = 0;
    always @(posedge clk) begin
        if (rst_n && AIN1 && AIN2) shoot_through = shoot_through + 1;
        if (rst_n && BIN1 && BIN2) shoot_through = shoot_through + 1;
    end

    // ---------------------------------------------- 在 PWM_TICKS 拍窗口里数高电平
    task automatic measure_full(output integer dla, output integer dlb, output integer period);
        integer i;
        reg prev;
        begin
            dla = 0; dlb = 0; period = 0;
            while (!PWMA) @(negedge clk);                 // 对齐到高电平起点
            for (i = 0; i < PWM_TICKS; i = i + 1) begin
                @(negedge clk);
                if (PWMA) dla = dla + 1;
                if (PWMB) dlb = dlb + 1;
            end
            // 周期 = 相邻两个上升沿之间的拍数
            begin : count_period_loop
            while (PWMA) @(negedge clk);                  // 先到低电平
            prev = 1'b0;
            while (!(PWMA && !prev)) begin                // 等第一个上升沿
                prev = PWMA;
                @(negedge clk);
            end
            period = 0;
            prev = 1'b1;
            while (1) begin
                @(negedge clk);
                period = period + 1;
                if (PWMA && !prev) disable count_period_loop;
                prev = PWMA;
            end
            end
        end
    endtask

    task automatic settle(input integer n);
        begin repeat (n) @(negedge clk); end
    endtask

    // 不对齐窗口：直接数 PWM_TICKS 拍内的高电平（v=0 时没有上升沿可用）
    task automatic count_span(output integer dla, output integer dlb);
        integer i;
        begin
            dla = 0; dlb = 0;
            for (i = 0; i < PWM_TICKS; i = i + 1) begin
                @(negedge clk);
                if (PWMA) dla = dla + 1;
                if (PWMB) dlb = dlb + 1;
            end
        end
    endtask

    integer da, db, period, expect_duty, v;

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        settle(20);

        // ---------------- U1 占空比扫描（含方向 U3）
        begin : sweep
            integer sweep_errors;
            integer repeat_guard;
            sweep_errors = 0;
            v = -300;
            while (v <= 300) begin
                target_l = v; target_r = v;
                settle(PWM_TICKS * 2);
                if (v == 0) begin
                    for (repeat_guard = 0; repeat_guard < 3; repeat_guard = repeat_guard + 1)
                        count_span(da, db);
                    if (da > 1 || db > 1) begin
                        $display("FAIL U4 zero_duty_in_sweep da=%0d db=%0d", da, db);
                        sweep_errors = sweep_errors + 1;
                    end
                end else begin
                    measure_full(da, db, period);
                end
                expect_duty = (v < 0 ? -v : v) * SCALE;
                if (expect_duty > PWM_TICKS - 1) expect_duty = PWM_TICKS - 1;
                if (v != 0) begin
                    if (da > expect_duty + 2 || da < expect_duty - 2) begin
                        $display("FAIL U1 duty_l v=%0d measured=%0d expect=%0d", v, da, expect_duty);
                        sweep_errors = sweep_errors + 1;
                    end
                    if (db > expect_duty + 2 || db < expect_duty - 2) begin
                        $display("FAIL U1 duty_r v=%0d measured=%0d expect=%0d", v, db, expect_duty);
                        sweep_errors = sweep_errors + 1;
                    end
                    if (v > 0) begin
                        if (!(AIN1 && !AIN2)) begin
                            $display("FAIL U3 forward v=%0d AIN1=%b AIN2=%b", v, AIN1, AIN2);
                            sweep_errors = sweep_errors + 1;
                        end
                    end else begin
                        if (!(!AIN1 && AIN2)) begin
                            $display("FAIL U3 reverse v=%0d AIN1=%b AIN2=%b", v, AIN1, AIN2);
                            sweep_errors = sweep_errors + 1;
                        end
                    end
                end
                v = v + 50;
            end
            errors = errors + sweep_errors;
            checks = checks + 1;
            if (sweep_errors == 0) $display("PASS U1/U3 duty sweep + direction (-300..300 step 50, tol +-2)");
            else                   $display("FAIL U1/U3 duty sweep (%0d errors)", sweep_errors);
        end

        // ---------------- U2 周期
        target_l = 150; target_r = 150;
        settle(PWM_TICKS * 2);
        measure_full(da, db, period);
        check(period >= PWM_TICKS - 2 && period <= PWM_TICKS + 2,
              $sformatf("U2 pwm_period measured=%0d expect=%0d", period, PWM_TICKS));

        // ---------------- U4 零速度
        target_l = 0; target_r = 0;
        settle(PWM_TICKS * 2);
        count_span(da, db);
        count_span(da, db);
        check(da <= 1 && db <= 1, "U4 zero_duty");
        check(!AIN1 && !AIN2 && !BIN1 && !BIN2, "U4 zero_command_direction_pins_low");
        check(STBY == 1'b1, "U4 zero_command_stby_still_on");

        // ---------------- U5 换向死区
        target_l = 150; target_r = 150;
        settle(PWM_TICKS);
        begin : deadtime_check
            integer guard;
            integer pwm_ok = 0, dead_ok = 1, low_run = 0, max_low_run = 0;
            target_l = -150; target_r = -150;
            // 命令在 PWM 周期边界才被驱动采样（原子更新），所以最多等 2 个周期
            for (guard = 0; guard < PWM_TICKS * 2 + 10; guard = guard + 1) begin
                @(negedge clk);
                if (!PWMA) begin
                    pwm_ok = 1; low_run = low_run + 1;
                    if (low_run > max_low_run) max_low_run = low_run;
                end else low_run = 0;
                if (AIN1 && AIN2) dead_ok = 0;
            end
            // 反向前应有一段 PWM=0 的死区（>= DEADTIME 拍量级）
            if (max_low_run < DEADTIME) dead_ok = 0;
            check(pwm_ok && dead_ok, "U5 deadtime_pwm_masked_no_shoot_through");
            settle(PWM_TICKS);
            measure_full(da, db, period);
            check(!AIN1 && AIN2, "U5 direction_after_deadtime");
            expect_duty = 150 * SCALE;
            check(da <= expect_duty + 2 && da >= expect_duty - 2, "U5 duty_after_reverse");
        end

        // ---------------- U6 限幅
        target_l = 2000; target_r = -2000;
        settle(PWM_TICKS * 2);
        measure_full(da, db, period);
        expect_duty = MAX_V * SCALE;   // 300 mm/s -> 6000 拍（约 96%），超出按此限幅
        check(da <= expect_duty + 2 && da >= expect_duty - 2, "U6 clamp_positive");
        check(db <= expect_duty + 2 && db >= expect_duty - 2, "U6 clamp_negative");
        check(vlim_l == MAX_V && vlim_r == -MAX_V, "U6 velocity_clamped");

        // ---------------- U7 差速
        target_l = -120; target_r = 120;
        settle(PWM_TICKS * 2);
        measure_full(da, db, period);
        check(!AIN1 && AIN2, "U7 differential_left_reverse");
        check(BIN1 && !BIN2, "U7 differential_right_forward");
        expect_duty = 120 * SCALE;
        check(da <= expect_duty + 2 && da >= expect_duty - 2, "U7 differential_left_duty");
        check(db <= expect_duty + 2 && db >= expect_duty - 2, "U7 differential_right_duty");

        // ---------------- U8 真值表
        begin : truth_table
            integer li, st, fp, es, bad;
            bad = 0;
            target_l = 0; target_r = 0;
            for (li = 0; li < 2; li = li + 1)
                for (st = 0; st < 2; st = st + 1)
                    for (fp = 0; fp < 2; fp = fp + 1)
                        for (es = 0; es < 2; es = es + 1) begin
                            link_ok      = li[0];
                            stop_latched = st[0];
                            fault_bits   = fp[0] ? 8'h20 : 8'h00;
                            key_estop_n  = !es[0];
                            @(negedge clk); @(negedge clk);
                            if (motors_enabled !== (li[0] && !st[0] && !fp[0] && !es[0])) bad = bad + 1;
                            settle(6);
                            if (STBY !== (li[0] && !st[0] && !fp[0] && !es[0])) bad = bad + 1;
                        end
            check(bad == 0, $sformatf("U8 motors_enabled_truth_table (%0d mismatches)", bad));
            link_ok = 1; stop_latched = 0; fault_bits = 0; key_estop_n = 1;
            settle(6);
        end

        // ---------------- U9 安全路径 + 恢复
        target_l = 150; target_r = 150;
        settle(PWM_TICKS);
        check(STBY == 1'b1, "U9 baseline_stby_on");
        link_ok = 0; settle(6);
        check(STBY == 1'b0 && !PWMA, "U9 link_lost_stops");
        link_ok = 1; settle(6);
        check(STBY == 1'b1, "U9 link_ok_recovers");
        fault_bits = 8'h20; settle(6);
        check(STBY == 1'b0 && !PWMA, "U9 fault_stops");
        fault_bits = 0; settle(6);
        stop_latched = 1; settle(6);
        check(STBY == 1'b0 && !PWMA, "U9 stop_latched_stops");
        stop_latched = 0; settle(6);
        key_estop_n = 0; settle(6);
        check(STBY == 1'b0 && !PWMA, "U9 estop_key_stops");
        key_estop_n = 1; settle(6);
        check(STBY == 1'b1, "U9 reenable_after_all_clear");
        check(shoot_through == 0, "U3 no_shoot_through_ever");

        $display("");
        if (errors == 0) $display("PASS tb_eth_motion_unit (%0d checks)", checks);
        else             $display("FAIL tb_eth_motion_unit (%0d errors, %0d checks)", errors, checks);
        $finish;
    end

    initial begin
        #400_000_000;
        $display("FAIL tb_eth_motion_unit timeout");
        $finish;
    end
endmodule
