`timescale 1ns/1ps
// Standalone Verilog-2001 register peripheral. All inputs synchronous to clk.
module chassis_calibration_telemetry_and_register_interface #(
    parameter integer CLK_HZ = 50000000,
    parameter integer PWM_PERIOD_CYCLES = 2500,
    parameter integer RUNTIME_WIDTH = 32
) (
    input wire clk, input wire rst_n,
    input wire req, input wire write,
    input wire [11:0] addr, input wire [31:0] wdata,
    input wire [3:0] wstrb,
    output reg ack, output reg error, output reg [31:0] rdata,
    input wire signed [31:0] speed_l, speed_r, position_l, position_r,
    input wire signed [15:0] pwm_l, pwm_r,
    input wire [31:0] illegal_count_l, illegal_count_r,
    input wire [31:0] status_in, fault_in,
    input wire running_l, running_r,
    input wire [31:0] voltage_mv,
    input wire signed [31:0] temperature_mc,
    input wire voltage_valid, temperature_valid,
    output reg control_enable,
    output reg [2:0] mode,
    output reg signed [31:0] target_l, target_r,
    output reg config_updated, target_updated, soft_reset_req,
    output reg fault_clear_req, output reg [31:0] fault_clear_mask,
    output reg [31:0] fault_latched,
    output wire [3:0] direction_cfg,
    output wire [31:0] counts_per_rev_l,
    output wire [31:0] counts_per_rev_r,
    output wire [31:0] wheel_diameter_l_mm,
    output wire [31:0] wheel_diameter_r_mm,
    output wire [31:0] wheelbase_mm,
    output wire [31:0] max_speed_counts,
    output wire signed [31:0] kp_l,
    output wire signed [31:0] ki_l,
    output wire signed [31:0] kd_l,
    output wire signed [31:0] kp_r,
    output wire signed [31:0] ki_r,
    output wire signed [31:0] kd_r,
    output wire [5:0] gain_shift,
    output wire [15:0] pwm_limit,
    output wire [15:0] pwm_min,
    output wire [15:0] ramp_step,
    output wire [15:0] stall_pwm_threshold,
    output wire [31:0] stall_speed_threshold,
    output wire [31:0] stall_timeout_ms,
    output wire [31:0] integral_limit
);
    localparam integer MS_CYCLES = CLK_HZ / 1000;
    localparam integer DEFAULT_PWM_LIMIT = (PWM_PERIOD_CYCLES < 1000) ? PWM_PERIOD_CYCLES : 1000;
    localparam integer CFG_COUNT = 21;
    reg [31:0] shadow [0:CFG_COUNT-1];
    reg [31:0] active [0:CFG_COUNT-1];
    reg signed [31:0] target_shadow_l, target_shadow_r;
    reg [31:0] snapshot [0:15];
    reg snapshot_hold, snapshot_valid;
    reg [31:0] snapshot_period, period_count, snapshot_sequence, missed_samples;
    reg [31:0] timestamp_ms, ms_count;
    reg [RUNTIME_WIDTH-1:0] runtime_l, runtime_r;
    integer i;
    reg [31:0] read_value;
    reg mapped, writable;
    wire [31:0] byte_mask = {{8{wstrb[3]}},{8{wstrb[2]}},{8{wstrb[1]}},{8{wstrb[0]}}};
    wire [31:0] command = wdata & byte_mask;
    wire aligned = (addr[1:0] == 0);
    wire config_address = addr >= 12'h100 && addr <= 12'h150 && aligned;
    wire active_address = addr >= 12'h180 && addr <= 12'h1d0 && aligned;
    wire telemetry_address = addr >= 12'h200 && addr <= 12'h23c && aligned;
    wire periodic_due = snapshot_period != 0 && period_count >= snapshot_period-1;
    wire manual_due = req && write && addr == 12'h014 && command[0];
    wire hold_write = req && write && addr == 12'h018 && wstrb[0];
    wire hold_next = hold_write ? wdata[0] : snapshot_hold;
    wire [31:0] clear_bits = (req && write && addr == 12'h300) ? command : 32'd0;
    wire config_valid =
        shadow[0] <= 15 && shadow[1] != 0 && shadow[2] != 0 &&
        shadow[3] != 0 && shadow[4] != 0 && shadow[5] != 0 &&
        shadow[6] != 0 && !shadow[6][31] && shadow[13] <= 31 &&
        shadow[14] <= PWM_PERIOD_CYCLES && shadow[15] <= shadow[14] &&
        shadow[16] > 0 && shadow[16] <= 65535 &&
        shadow[17] <= shadow[14] && !shadow[18][31] &&
        shadow[19] != 0 && !shadow[20][31];

    function [31:0] merge_bytes;
        input [31:0] old_value, new_value, mask;
        begin merge_bytes = (old_value & ~mask) | (new_value & mask); end
    endfunction
    function [31:0] increment_sat;
        input [31:0] value;
        begin increment_sat = (&value) ? value : value + 1'b1; end
    endfunction
    function target_ok;
        input signed [31:0] value;
        input [31:0] limit_value;
        reg signed [32:0] extended;
        begin
            extended = {value[31],value};
            target_ok = extended <= $signed({1'b0,limit_value}) &&
                        extended >= -$signed({1'b0,limit_value});
        end
    endfunction
    assign direction_cfg = active[0][3:0];
    assign counts_per_rev_l = active[1][31:0];
    assign counts_per_rev_r = active[2][31:0];
    assign wheel_diameter_l_mm = active[3][31:0];
    assign wheel_diameter_r_mm = active[4][31:0];
    assign wheelbase_mm = active[5][31:0];
    assign max_speed_counts = active[6][31:0];
    assign kp_l = active[7][31:0];
    assign ki_l = active[8][31:0];
    assign kd_l = active[9][31:0];
    assign kp_r = active[10][31:0];
    assign ki_r = active[11][31:0];
    assign kd_r = active[12][31:0];
    assign gain_shift = active[13][5:0];
    assign pwm_limit = active[14][15:0];
    assign pwm_min = active[15][15:0];
    assign ramp_step = active[16][15:0];
    assign stall_pwm_threshold = active[17][15:0];
    assign stall_speed_threshold = active[18][31:0];
    assign stall_timeout_ms = active[19][31:0];
    assign integral_limit = active[20][31:0];

    // synthesis translate_off
    initial begin
        if (CLK_HZ < 1000 || CLK_HZ % 1000 != 0 ||
            PWM_PERIOD_CYCLES < 1 || PWM_PERIOD_CYCLES > 32767 ||
            RUNTIME_WIDTH < 1 || RUNTIME_WIDTH > 32) begin
            $display("ERROR: invalid chassis register parameters");
            $fatal(1);
        end
    end
    // synthesis translate_on

    always @* begin
        read_value = 0; mapped = aligned; writable = 0;
        case (addr)
            12'h000: read_value = 32'h00010000;
            12'h004: begin read_value = {28'd0,mode,control_enable}; writable = 1; end
            12'h008: begin read_value = target_shadow_l; writable = 1; end
            12'h00c: begin read_value = target_shadow_r; writable = 1; end
            12'h010,12'h014,12'h01c,12'h020: writable = 1;
            12'h018: begin read_value = {31'd0,snapshot_hold}; writable = 1; end
            12'h024: begin read_value = snapshot_period; writable = 1; end
            12'h028: read_value = target_l;
            12'h02c: read_value = target_r;
            12'h030: read_value = {31'd0,snapshot_valid};
            12'h034: read_value = snapshot_sequence;
            12'h038: read_value = timestamp_ms;
            12'h300: begin read_value = fault_latched; writable = 1; end
            12'h304: read_value = fault_in;
            12'h308: read_value = missed_samples;
            12'h30c: read_value = runtime_l;
            12'h310: read_value = runtime_r;
            default: begin
                if (config_address) begin
                    read_value = shadow[(addr-12'h100)>>2]; writable = 1;
                end else if (active_address) read_value = active[(addr-12'h180)>>2];
                else if (telemetry_address) read_value = snapshot[(addr-12'h200)>>2];
                else mapped = 0;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack <= 0; error <= 0; rdata <= 0;
            control_enable <= 0; mode <= 0;
            target_l <= 0; target_r <= 0; target_shadow_l <= 0; target_shadow_r <= 0;
            config_updated <= 0; target_updated <= 0; soft_reset_req <= 0;
            fault_clear_req <= 0; fault_clear_mask <= 0; fault_latched <= 0;
            snapshot_hold <= 0; snapshot_valid <= 0; snapshot_period <= 0;
            period_count <= 0; snapshot_sequence <= 0; missed_samples <= 0;
            timestamp_ms <= 0; ms_count <= 0; runtime_l <= 0; runtime_r <= 0;
            for (i=0; i<16; i=i+1) snapshot[i] <= 0;
            shadow[0] <= 0; active[0] <= 0;
            shadow[1] <= 1456; active[1] <= 1456;
            shadow[2] <= 1456; active[2] <= 1456;
            shadow[3] <= 66; active[3] <= 66;
            shadow[4] <= 66; active[4] <= 66;
            shadow[5] <= 240; active[5] <= 240;
            shadow[6] <= 1000; active[6] <= 1000;
            shadow[7] <= 0; active[7] <= 0;
            shadow[8] <= 0; active[8] <= 0;
            shadow[9] <= 0; active[9] <= 0;
            shadow[10] <= 0; active[10] <= 0;
            shadow[11] <= 0; active[11] <= 0;
            shadow[12] <= 0; active[12] <= 0;
            shadow[13] <= 16; active[13] <= 16;
            shadow[14] <= DEFAULT_PWM_LIMIT; active[14] <= DEFAULT_PWM_LIMIT;
            shadow[15] <= 0; active[15] <= 0;
            shadow[16] <= 10; active[16] <= 10;
            shadow[17] <= ((DEFAULT_PWM_LIMIT < 500) ? DEFAULT_PWM_LIMIT : 500); active[17] <= ((DEFAULT_PWM_LIMIT < 500) ? DEFAULT_PWM_LIMIT : 500);
            shadow[18] <= 1; active[18] <= 1;
            shadow[19] <= 500; active[19] <= 500;
            shadow[20] <= 100000; active[20] <= 100000;
        end else begin
            ack <= req; error <= 0; rdata <= 0;
            config_updated <= 0; target_updated <= 0; soft_reset_req <= 0;
            fault_clear_req <= 0; fault_clear_mask <= 0;
            fault_latched <= (fault_latched & ~clear_bits) | fault_in;
            if (ms_count == MS_CYCLES-1) begin
                ms_count <= 0; timestamp_ms <= timestamp_ms + 1'b1;
                if (running_l && !(&runtime_l)) runtime_l <= runtime_l + 1'b1;
                if (running_r && !(&runtime_r)) runtime_r <= runtime_r + 1'b1;
            end else ms_count <= ms_count + 1'b1;

            if (snapshot_period == 0 || periodic_due) period_count <= 0;
            else period_count <= period_count + 1'b1;
            if (manual_due || periodic_due) begin
                if (hold_next) missed_samples <= increment_sat(missed_samples);
                else begin
                    snapshot_valid <= 1;
                    snapshot_sequence <= snapshot_sequence + 1'b1;
                    snapshot[0] <= speed_l; snapshot[1] <= speed_r;
                    snapshot[2] <= position_l; snapshot[3] <= position_r;
                    snapshot[4] <= {{16{pwm_l[15]}},pwm_l};
                    snapshot[5] <= {{16{pwm_r[15]}},pwm_r};
                    snapshot[6] <= illegal_count_l; snapshot[7] <= illegal_count_r;
                    snapshot[8] <= status_in;
                    snapshot[9] <= (fault_latched & ~clear_bits) | fault_in;
                    snapshot[10] <= voltage_mv; snapshot[11] <= temperature_mc;
                    snapshot[12] <= {30'd0,temperature_valid,voltage_valid};
                    snapshot[13] <= timestamp_ms;
                    snapshot[14] <= runtime_l; snapshot[15] <= runtime_r;
                end
            end
            if (req) begin
                if (!mapped || (write && !writable)) error <= 1;
                else if (!write) rdata <= read_value;
                else begin
                    case (addr)
                        12'h004: if (wstrb[0]) begin
                            if (wdata[0] && (fault_in != 0 || fault_latched != 0)) error <= 1;
                            else begin control_enable <= wdata[0]; mode <= wdata[3:1]; end
                        end
                        12'h008: target_shadow_l <= merge_bytes(target_shadow_l,wdata,byte_mask);
                        12'h00c: target_shadow_r <= merge_bytes(target_shadow_r,wdata,byte_mask);
                        12'h010: if (command[0]) begin
                            if (target_ok(target_shadow_l,active[6]) && target_ok(target_shadow_r,active[6])) begin
                                target_l <= target_shadow_l; target_r <= target_shadow_r; target_updated <= 1;
                            end else error <= 1;
                        end
                        12'h014: begin end // snapshot handled above
                        12'h018: if (wstrb[0]) snapshot_hold <= wdata[0];
                        12'h01c: if (command[0]) begin
                            if (control_enable || !config_valid ||
                                !target_ok(target_l,shadow[6]) || !target_ok(target_r,shadow[6])) error <= 1;
                            else begin
                                for (i=0;i<CFG_COUNT;i=i+1) active[i] <= shadow[i];
                                config_updated <= 1;
                            end
                        end
                        12'h020: soft_reset_req <= command[0];
                        12'h024: begin
                            snapshot_period <= merge_bytes(snapshot_period,wdata,byte_mask);
                            period_count <= 0;
                        end
                        12'h300: begin fault_clear_req <= |command; fault_clear_mask <= command; end
                        default: if (config_address)
                            shadow[(addr-12'h100)>>2] <= merge_bytes(shadow[(addr-12'h100)>>2],wdata,byte_mask);
                    endcase
                end
            end
            // A live fault always wins over software, including a simultaneous enable.
            if (fault_in != 0) control_enable <= 0;
        end
    end
endmodule
