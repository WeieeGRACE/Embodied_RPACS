`timescale 1ns/1ps
// Synchronous commands/configuration. Duty unit: one clk tick.
//
// t4_lite_drive 本地副本（来源：主工程 src/motor/pwm_and_startup_ramp.v）。
// 模块名/端口名/参数名/功能不变；为 125 MHz 适配，把「命令 -> 绝对值/限幅」的
// 组合逻辑单独打一拍（原版在 50 MHz 下与 tick 更新同一拍完成）。ramp 更新仍然
// 只在 PWM 周期末（tick）发生一次，命令最多 20 ms 变一次，多一拍延迟无影响。
module pwm_and_startup_ramp #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_HZ = 20000
)(
    input wire clk, rst_n, enable,
    input wire signed [15:0] pwm_cmd_l, pwm_cmd_r,
    input wire [15:0] pwm_limit, pwm_min, ramp_step,
    output wire pwm_out_l, pwm_out_r,
    output wire signed [15:0] pwm_actual_l, pwm_actual_r
);
    localparam integer PWM_PERIOD = CLK_HZ / ((PWM_HZ > 0) ? PWM_HZ : 1);
    reg [15:0] phase;
    wire run = rst_n && enable;
    wire tick = (phase == PWM_PERIOD-1);
    // synthesis translate_off
    initial begin
        if (CLK_HZ <= 0 || PWM_HZ <= 0 || PWM_PERIOD < 2 ||
            PWM_PERIOD > 32767 || CLK_HZ % ((PWM_HZ > 0) ? PWM_HZ : 1) != 0) begin
            $display("ERROR: clock/PWM parameters require an integer period in [2,32767]");
            $fatal(1);
        end
    end
    // synthesis translate_on
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) phase <= 0;
        else if (!enable || tick) phase <= 0;
        else phase <= phase + 1'b1;
    end
    pwm_startup_ramp_channel #(.PWM_PERIOD(PWM_PERIOD)) left_channel (
        .clk(clk), .rst_n(rst_n), .run(run), .tick(tick), .phase(phase),
        .command(pwm_cmd_l), .limit(pwm_limit), .minimum(pwm_min),
        .step(ramp_step), .pwm(pwm_out_l), .actual(pwm_actual_l));
    pwm_startup_ramp_channel #(.PWM_PERIOD(PWM_PERIOD)) right_channel (
        .clk(clk), .rst_n(rst_n), .run(run), .tick(tick), .phase(phase),
        .command(pwm_cmd_r), .limit(pwm_limit), .minimum(pwm_min),
        .step(ramp_step), .pwm(pwm_out_r), .actual(pwm_actual_r));
endmodule

// Private implementation helper; direction pins/deadtime belong to the driver.
module pwm_startup_ramp_channel #(parameter integer PWM_PERIOD = 2500)(
    input wire clk, rst_n, run, tick,
    input wire [15:0] phase,
    input wire signed [15:0] command,
    input wire [15:0] limit, minimum, step,
    output wire pwm,
    output wire signed [15:0] actual
);
    reg [15:0] duty;
    reg direction;
    // 命令预处理（独立一拍）：cap/floor/|command|/target/increment/方向/非零
    wire [15:0] cap_now       = (limit > PWM_PERIOD) ? PWM_PERIOD : limit;
    wire [15:0] floor_now     = (minimum > cap_now) ? cap_now : minimum;
    wire [15:0] magnitude_now = command[15] ? (~command + 16'd1) : command;
    wire [15:0] clamped_now   = (magnitude_now > cap_now) ? cap_now : magnitude_now;
    wire [15:0] target_now    = (magnitude_now != 16'd0 && clamped_now < floor_now)
                                ? floor_now : clamped_now;
    wire [15:0] increment_now = (step == 16'd0) ? 16'd1 : step;
    reg [15:0] target, cap, increment;
    reg        dir_cmd, nonzero;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target <= 16'd0; cap <= 16'd0; increment <= 16'd1;
            dir_cmd <= 1'b0; nonzero <= 1'b0;
        end else begin
            target    <= target_now;
            cap       <= cap_now;
            increment <= increment_now;
            dir_cmd   <= command[15];
            nonzero   <= (target_now != 16'd0);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin duty <= 0; direction <= 0; end
        else if (!run) begin duty <= 0; direction <= 0; end
        else if (tick) begin
            // Safety limit has priority, including while reversing.
            if (duty > cap) duty <= cap;
            else if (duty != 0 && nonzero && dir_cmd != direction) begin
                if (duty <= increment) duty <= 0;
                else duty <= duty - increment;
            end else begin
                // A zero duty state has already occupied a complete PWM period.
                if (duty == 0 && nonzero) direction <= dir_cmd;
                if (duty < target) begin
                    if (target - duty <= increment) duty <= target;
                    else duty <= duty + increment;
                end else if (duty > target) begin
                    if (duty - target <= increment) duty <= target;
                    else duty <= duty - increment;
                end
            end
        end
    end
    assign pwm = run && (phase < duty);
    assign actual = !run ? 16'sd0 : direction ? -$signed(duty) : $signed(duty);
endmodule
