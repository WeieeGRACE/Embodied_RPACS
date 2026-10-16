`timescale 1ns/1ps

module power_and_hardware_safety #(
    parameter integer STABLE_CYCLES = 250000,
    parameter integer PWM_LIMIT = 32767
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire               clear_fault,
    input  wire               system_ready,
    input  wire               emergency_stop,
    input  wire               over_current,
    input  wire               under_voltage,
    input  wire               over_temperature,
    input  wire               external_fault,
    input  wire signed [15:0] pwm_cmd_l,
    input  wire signed [15:0] pwm_cmd_r,
    output reg  signed [15:0] pwm_safe_l,
    output reg  signed [15:0] pwm_safe_r,
    output wire               stby,
    output reg                fault_latched,
    output reg  [2:0]         fault_code,
    output wire               startup_done,
    output wire               safety_active
);

    localparam [2:0] FAULT_NONE = 3'd0;
    localparam [2:0] FAULT_ESTOP = 3'd1;
    localparam [2:0] FAULT_OVERCURRENT = 3'd2;
    localparam [2:0] FAULT_UNDERVOLTAGE = 3'd3;
    localparam [2:0] FAULT_OVERTEMP = 3'd4;
    localparam [2:0] FAULT_EXTERNAL = 3'd5;

    reg [31:0] stable_count;
    reg startup_done_reg;
    wire live_fault = emergency_stop | over_current | under_voltage |
                       over_temperature | external_fault;

    assign startup_done = startup_done_reg;
    assign safety_active = startup_done_reg && enable && system_ready &&
                           !fault_latched && !emergency_stop && rst_n;
    assign stby = safety_active;

    function [2:0] highest_fault;
        input estop;
        input oc;
        input uv;
        input ot;
        input ext;
        begin
            if (estop) highest_fault = FAULT_ESTOP;
            else if (oc) highest_fault = FAULT_OVERCURRENT;
            else if (uv) highest_fault = FAULT_UNDERVOLTAGE;
            else if (ot) highest_fault = FAULT_OVERTEMP;
            else if (ext) highest_fault = FAULT_EXTERNAL;
            else highest_fault = FAULT_NONE;
        end
    endfunction

    function signed [15:0] clamp_pwm;
        input signed [15:0] value;
        integer signed tmp;
        begin
            tmp = value;
            if (tmp > PWM_LIMIT) tmp = PWM_LIMIT;
            if (tmp < -PWM_LIMIT) tmp = -PWM_LIMIT;
            clamp_pwm = tmp[15:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stable_count <= 32'd0;
            startup_done_reg <= 1'b0;
            fault_latched <= 1'b0;
            fault_code <= FAULT_NONE;
        end else begin
            if (live_fault) begin
                fault_latched <= 1'b1;
                fault_code <= highest_fault(emergency_stop, over_current,
                                            under_voltage, over_temperature,
                                            external_fault);
            end else if (fault_latched) begin
                if (clear_fault && !emergency_stop) begin
                    fault_latched <= 1'b0;
                    fault_code <= FAULT_NONE;
                    stable_count <= 32'd0;
                    startup_done_reg <= 1'b0;
                end
            end else if (!enable || !system_ready) begin
                stable_count <= 32'd0;
                startup_done_reg <= 1'b0;
            end else if (!startup_done_reg) begin
                if (STABLE_CYCLES <= 0) begin
                    startup_done_reg <= 1'b1;
                end else if (stable_count >= STABLE_CYCLES - 1) begin
                    startup_done_reg <= 1'b1;
                    stable_count <= stable_count;
                end else begin
                    stable_count <= stable_count + 1'b1;
                end
            end
        end
    end

    always @* begin
        pwm_safe_l = 16'sd0;
        pwm_safe_r = 16'sd0;
        if (safety_active) begin
            pwm_safe_l = clamp_pwm(pwm_cmd_l);
            pwm_safe_r = clamp_pwm(pwm_cmd_r);
        end
    end

endmodule
