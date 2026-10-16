`timescale 1ns/1ps

module safety_protection_and_fault_arbitration #(
    parameter integer STARTUP_STABLE_CYCLES = 250000,
    parameter integer PWM_LIMIT = 32767,
    parameter integer OBSTACLE_STOP_DISTANCE = 200,
    parameter integer OBSTACLE_SLOW_DISTANCE = 500,
    parameter integer OBSTACLE_SLOW_LIMIT = 50,
    parameter integer RANGE_CHECK_ENABLE = 1
) (
    input wire clk, input wire rst_n,
    input wire enable, input wire system_ready, input wire clear_fault,
    input wire emergency_stop, input wire over_current, input wire under_voltage,
    input wire over_temperature, input wire watchdog_timeout, input wire external_fault,
    input wire range_valid, input wire [15:0] front_distance,
    input wire signed [15:0] pwm_cmd_l, input wire signed [15:0] pwm_cmd_r,
    output reg signed [15:0] pwm_safe_l, output reg signed [15:0] pwm_safe_r,
    output wire stby, output reg safety_active, output reg fault_latched,
    output reg [3:0] fault_code, output reg [1:0] obstacle_state,
    output wire startup_done
);
    localparam [3:0] FAULT_NONE=4'd0, FAULT_ESTOP=4'd1, FAULT_OVERCURRENT=4'd2,
                     FAULT_UNDERVOLTAGE=4'd3, FAULT_OVERTEMP=4'd4,
                     FAULT_WATCHDOG=4'd5, FAULT_SENSOR=4'd6, FAULT_EXTERNAL=4'd7;
    localparam [1:0] OBS_CLEAR=2'd0, OBS_SLOW=2'd1, OBS_STOP=2'd2, OBS_SENSOR_FAULT=2'd3;
    reg [31:0] stable_count;
    reg startup_done_reg;
    wire sensor_fault = (RANGE_CHECK_ENABLE != 0) && !range_valid;
    wire hard_fault = emergency_stop | over_current | under_voltage | over_temperature |
                      watchdog_timeout | sensor_fault | external_fault;
    wire forward_command = (pwm_cmd_l > 0) || (pwm_cmd_r > 0);
    wire range_stop = (RANGE_CHECK_ENABLE != 0) && range_valid &&
                      (front_distance <= OBSTACLE_STOP_DISTANCE);
    wire range_slow = (RANGE_CHECK_ENABLE != 0) && range_valid &&
                      (front_distance <= OBSTACLE_SLOW_DISTANCE) && !range_stop;

    assign startup_done = startup_done_reg;
    assign stby = safety_active;

    function [3:0] select_fault;
      input estop, oc, uv, ot, wd, sf, ext;
      begin
        if (estop) select_fault=FAULT_ESTOP; else if (oc) select_fault=FAULT_OVERCURRENT;
        else if (uv) select_fault=FAULT_UNDERVOLTAGE; else if (ot) select_fault=FAULT_OVERTEMP;
        else if (wd) select_fault=FAULT_WATCHDOG; else if (sf) select_fault=FAULT_SENSOR;
        else if (ext) select_fault=FAULT_EXTERNAL; else select_fault=FAULT_NONE;
      end
    endfunction

    function signed [15:0] clamp_value;
      input signed [15:0] value; input integer limit_value;
      integer signed temp;
      begin temp=value; if (temp > limit_value) temp=limit_value;
        if (temp < -limit_value) temp=-limit_value; clamp_value=temp[15:0]; end
    endfunction

    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin stable_count<=0; startup_done_reg<=0; fault_latched<=0;
        fault_code<=FAULT_NONE; obstacle_state<=OBS_CLEAR; safety_active<=0; end
      else begin
        if (sensor_fault) obstacle_state<=OBS_SENSOR_FAULT;
        else if (range_stop && forward_command) obstacle_state<=OBS_STOP;
        else if (range_slow && forward_command) obstacle_state<=OBS_SLOW;
        else obstacle_state<=OBS_CLEAR;
        if (hard_fault) begin fault_latched<=1'b1;
          fault_code<=select_fault(emergency_stop,over_current,under_voltage,
                                    over_temperature,watchdog_timeout,sensor_fault,external_fault);
          stable_count<=0; startup_done_reg<=0;
        end else if (fault_latched) begin
          if (clear_fault) begin fault_latched<=0; fault_code<=FAULT_NONE;
            stable_count<=0; startup_done_reg<=0; end
        end else if (!enable || !system_ready) begin stable_count<=0; startup_done_reg<=0; end
        else if (!startup_done_reg) begin
          if (STARTUP_STABLE_CYCLES <= 0) startup_done_reg<=1;
          else if (stable_count >= STARTUP_STABLE_CYCLES-1) startup_done_reg<=1;
          else stable_count<=stable_count+1'b1;
        end
        safety_active <= startup_done_reg && enable && system_ready && !fault_latched &&
                         !emergency_stop && !over_current && !under_voltage &&
                         !over_temperature && !watchdog_timeout && !sensor_fault &&
                         !external_fault && !(range_stop && forward_command);
      end
    end

    always @* begin
      pwm_safe_l=0; pwm_safe_r=0;
      if (safety_active) begin
        if (range_slow && forward_command) begin
          pwm_safe_l=clamp_value(pwm_cmd_l,OBSTACLE_SLOW_LIMIT);
          pwm_safe_r=clamp_value(pwm_cmd_r,OBSTACLE_SLOW_LIMIT);
        end else begin
          pwm_safe_l=clamp_value(pwm_cmd_l,PWM_LIMIT);
          pwm_safe_r=clamp_value(pwm_cmd_r,PWM_LIMIT);
        end
      end
    end
endmodule
