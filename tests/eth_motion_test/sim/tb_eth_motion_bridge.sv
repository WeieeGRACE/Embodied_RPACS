`timescale 1ns/1ps
// 电机控制路径自检（不动车、不需要以太网）：
//   eth_motion_bridge + 主工程 tb6612_driver_interface
//
// 用 125 kHz 时钟 / 20 Hz PWM（周期 6250 拍）替代 125 MHz / 20 kHz，
// 这样可以在有限仿真时间里数清占空比，逻辑等价。
module tb_eth_motion_bridge;
    localparam integer CLK_HZ    = 125_000;
    localparam integer PWM_HZ    = 20;
    localparam integer PWM_TICKS = CLK_HZ / PWM_HZ;    // 6250
    localparam integer MAX_V     = 300;

    reg clk = 0;
    always #4 clk = ~clk;              // 125 MHz 形式的占空比：这里 4ns 半周期，仅用于节拍
    reg rst_n = 0;

    integer errors = 0, checks = 0;
    task automatic check(input bit cond, input string msg);
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s (t=%0t)", msg, $time);
        end else $display("PASS %s", msg);
    endtask

    // ------------------------------------------------------------- DUT
    reg signed [15:0] target_l = 0, target_r = 0;
    reg  [7:0] fault_bits = 0;
    reg        link_ok = 1, stop_latched = 0, key_estop_n = 1;
    wire signed [15:0] pwm_l, pwm_r;
    wire motors_enabled;

    eth_motion_bridge #(
        .MAX_V_MM_S(MAX_V), .PWM_TICKS(PWM_TICKS), .SCALE_NUM(PWM_TICKS / MAX_V)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .target_l(target_l), .target_r(target_r),
        .fault_bits(fault_bits), .link_ok(link_ok),
        .stop_latched(stop_latched), .key_estop_n(key_estop_n),
        .pwm_l(pwm_l), .pwm_r(pwm_r), .motors_enabled(motors_enabled),
        .v_limited_l(), .v_limited_r()
    );

    wire PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY;

    tb6612_driver_interface #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ), .DIR_DEADTIME_CYCLES(20)
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

    // ---------------------------------------------- 数一个 PWM 周期的高电平拍数
    task automatic measure_duty(output integer duty_a, output integer duty_b);
        integer i;
        begin
            duty_a = 0; duty_b = 0;
            // 等一个周期边界，然后数 PWM_TICKS 拍
            repeat (PWM_TICKS * 2) @(negedge clk);
            for (i = 0; i < PWM_TICKS; i = i + 1) begin
                @(negedge clk);
                if (PWMA) duty_a = duty_a + 1;
                if (PWMB) duty_b = duty_b + 1;
            end
        end
    endtask

    integer da, db;
    integer wait_cnt;

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        // 1) 静止：占空比 0，STBY 应使能（无故障时）
        measure_duty(da, db);
        check(da <= 2 && db <= 2, "idle_duty_zero");
        check(motors_enabled == 1'b1, "idle_motors_enabled");

        // 2) 前进 150 mm/s：占空比 ≈ 150*20 = 3000 拍（±5%）
        target_l = 16'sd150; target_r = 16'sd150;
        measure_duty(da, db);
        check(da > PWM_TICKS*0.40 && da < PWM_TICKS*0.56, "forward_duty_150mmps_left");
        check(db > PWM_TICKS*0.40 && db < PWM_TICKS*0.56, "forward_duty_150mmps_right");
        check(AIN1 == 1'b1 && AIN2 == 1'b0, "forward_direction_left");
        check(BIN1 == 1'b1 && BIN2 == 1'b0, "forward_direction_right");

        // 3) 后退：方向反转
        target_l = -16'sd150; target_r = -16'sd150;
        measure_duty(da, db);
        check(AIN1 == 1'b0 && AIN2 == 1'b1, "reverse_direction_left");
        check(da > PWM_TICKS*0.40 && da < PWM_TICKS*0.56, "reverse_duty_left");

        // 4) 超速限幅：给 2000 mm/s，占空比不超过满量程
        target_l = 16'sd2000; target_r = 16'sd2000;
        measure_duty(da, db);
        check(da <= PWM_TICKS - 1 && da >= PWM_TICKS*0.93, "clamp_at_full_scale");

        // 5) 链路丢失 -> 立即 STBY=0
        link_ok = 0;
        repeat (10) @(negedge clk);
        check(STBY == 1'b0, "link_lost_disables_stby");
        check(motors_enabled == 1'b0, "link_lost_clears_enable");
        link_ok = 1;
        repeat (10) @(negedge clk);

        // 6) 任何故障位 -> STBY=0
        fault_bits = 8'h20;
        repeat (10) @(negedge clk);
        check(STBY == 1'b0, "fault_disables_stby");
        fault_bits = 8'h00;
        repeat (10) @(negedge clk);

        // 7) STOP 锁存 -> STBY=0
        stop_latched = 1;
        repeat (10) @(negedge clk);
        check(STBY == 1'b0, "stop_latched_disables_stby");
        stop_latched = 0;
        repeat (10) @(negedge clk);

        // 8) 急停键按下（低）-> STBY=0
        key_estop_n = 0;
        repeat (10) @(negedge clk);
        check(STBY == 1'b0, "estop_key_disables_stby");
        key_estop_n = 1;
        repeat (10) @(negedge clk);

        // 9) 恢复后能重新使能
        target_l = 0; target_r = 0;
        repeat (PWM_TICKS * 2) @(negedge clk);
        check(STBY == 1'b1, "recover_reenables_stby");

        $display("");
        if (errors == 0) $display("PASS tb_eth_motion_bridge (%0d checks)", checks);
        else             $display("FAIL tb_eth_motion_bridge (%0d errors)", errors);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("FAIL tb_eth_motion_bridge timeout");
        $finish;
    end
endmodule
