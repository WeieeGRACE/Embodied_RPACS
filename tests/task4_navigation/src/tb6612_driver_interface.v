`timescale 1ns/1ps
// Standalone Verilog-2001 driver. Signed duty unit: one clk tick.
// Commands must share clk; no dependency on other project modules.
module tb6612_driver_interface #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_HZ = 20000,
    parameter integer DIR_DEADTIME_CYCLES = 2500
)(
    input wire clk, rst_n, enable,
    input wire emergency_stop, fault, watchdog_timeout,
    input wire signed [15:0] pwm_signed_l, pwm_signed_r,
    output wire PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY
);
    // Asynchronous assertion, two-clock synchronous release. Even a short
    // safety pulse clears state; recovery cannot expose stale PWM/direction.
    wire inhibit = !rst_n || !enable || emergency_stop || fault || watchdog_timeout;
    reg [1:0] release_pipe;
    wire run = release_pipe[1] && !inhibit;
    localparam integer PERIOD = CLK_HZ / ((PWM_HZ > 0) ? PWM_HZ : 1);
    reg [15:0] phase;
    reg signed [15:0] command_l, command_r;
    function [15:0] magnitude;
        input signed [15:0] value;
        reg [15:0] absolute_value;
        begin
            absolute_value = value[15] ? (~value + 16'd1) : value;
            magnitude = absolute_value > PERIOD ? PERIOD : absolute_value;
        end
    endfunction
    // Atomic command updates at PWM boundaries prevent partial-duty pulses.
    always @(posedge clk or negedge run) begin
        if (!run) begin phase <= 0; command_l <= 0; command_r <= 0; end
        else if (phase == PERIOD-1) begin
            phase <= 0; command_l <= pwm_signed_l; command_r <= pwm_signed_r;
        end else phase <= phase + 1'b1;
    end
    always @(posedge clk or posedge inhibit) begin
        if (inhibit) release_pipe <= 2'b00;
        else release_pipe <= {release_pipe[0], 1'b1};
    end
    assign STBY = run;
    tb6612_driver_channel #(.DEADTIME(DIR_DEADTIME_CYCLES)) left_channel (
        .clk(clk), .run(run), .command(command_l), .pwm_in(phase < magnitude(command_l)),
        .pwm(PWMA), .in1(AIN1), .in2(AIN2));
    tb6612_driver_channel #(.DEADTIME(DIR_DEADTIME_CYCLES)) right_channel (
        .clk(clk), .run(run), .command(command_r), .pwm_in(phase < magnitude(command_r)),
        .pwm(PWMB), .in1(BIN1), .in2(BIN2));
    // synthesis translate_off
    initial begin
        if (DIR_DEADTIME_CYCLES < 1) $fatal(1, "Deadtime must be at least one clock");
        if (CLK_HZ <= 0 || PWM_HZ <= 0 || PERIOD < 2 || PERIOD > 32767 ||
            CLK_HZ % ((PWM_HZ > 0) ? PWM_HZ : 1) != 0)
            $fatal(1, "PWM period must be an integer in [2,32767]");
    end
    // synthesis translate_on
endmodule

module tb6612_driver_channel #(parameter integer DEADTIME = 2500)(
    input wire clk, run,
    input wire signed [15:0] command,
    input wire pwm_in,
    output wire pwm, in1, in2
);
    function integer counter_width;
        input integer n;
        begin
            counter_width = 1;
            while (n > 1) begin n = n >> 1; counter_width = counter_width + 1; end
        end
    endfunction
    localparam integer CW = counter_width(DEADTIME);
    localparam [1:0] IDLE=0, DRIVE=1, WAIT_OFF=2, SETUP=3;
    reg [1:0] state;
    reg direction;
    reg [CW-1:0] remaining;
    wire nonzero = (command != 0);
    always @(posedge clk or negedge run) begin
        if (!run) begin
            state <= IDLE; direction <= 0; remaining <= 0;
        end else begin
            case (state)
                IDLE: if (nonzero) begin
                    direction <= command[15]; state <= SETUP;
                end
                DRIVE: if (nonzero && command[15] != direction) begin
                    state <= WAIT_OFF; remaining <= DEADTIME;
                end
                WAIT_OFF: begin
                    if (remaining > 1) remaining <= remaining - 1'b1;
                    else begin
                        remaining <= 0;
                        // Sample the latest command, never a queued stale target.
                        if (nonzero) begin direction <= command[15]; state <= SETUP; end
                        else state <= DRIVE;
                    end
                end
                SETUP: begin
                    if (nonzero && command[15] != direction) begin
                        state <= WAIT_OFF; remaining <= DEADTIME;
                    end else state <= DRIVE;
                end
                default: state <= IDLE;
            endcase
        end
    end
    // Zero command: IN1=IN2=0, PWM=0 => TB6612 stop (high impedance).
    // PWM-low during nonzero drive uses the chip's short-brake PWM behavior.
    // Sign mismatch masks PWM immediately, before the FSM observes reversal.
    assign pwm = run && state == DRIVE && nonzero &&
                 (command[15] == direction) && pwm_in;
    assign in1 = run && state != IDLE && nonzero && !direction;
    assign in2 = run && state != IDLE && nonzero && direction;
endmodule
