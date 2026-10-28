`timescale 1ns/1ps
// Synchronous commands/configuration. Duty unit: one clk tick.
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
    reg [15:0] target, magnitude, cap, floor_duty;
    wire [15:0] increment = (step == 0) ? 16'd1 : step;
    always @* begin
        cap = (limit > PWM_PERIOD) ? PWM_PERIOD : limit;
        floor_duty = (minimum > cap) ? cap : minimum;
        magnitude = command[15] ? (~command + 16'd1) : command;
        target = (magnitude > cap) ? cap : magnitude;
        if (magnitude != 0 && target < floor_duty) target = floor_duty;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin duty <= 0; direction <= 0; end
        else if (!run) begin duty <= 0; direction <= 0; end
        else if (tick) begin
            // Safety limit has priority, including while reversing.
            if (duty > cap) duty <= cap;
            else if (duty != 0 && target != 0 && command[15] != direction) begin
                if (duty <= increment) duty <= 0;
                else duty <= duty - increment;
            end else begin
                // A zero duty state has already occupied a complete PWM period.
                if (duty == 0 && target != 0) direction <= command[15];
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
