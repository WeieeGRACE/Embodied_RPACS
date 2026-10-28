`timescale 1ns/1ps
// Adapter: host link command interface  ->  existing motion sequencer command interface.
//
// The link core exposes differential wheel speeds (req_v_left / req_v_right in mm/s,
// signed) plus latched stop and link state. The main-project motion layer
// (src/motor/motion_control_and_mode_fsm.v) is *action* based: it accepts
// (mode, end kind, reverse, magnitude speeds) and produces target_l/target_r itself.
//
// Mapping (USE_VELOCITY_MODE = 0, works with the FSM as it exists today):
//     v_l == v_r > 0        -> FORWARD  (mode 2)
//     v_l == v_r < 0        -> BACKWARD (mode 3)
//     v_l == -v_r           -> ROTATE   (mode 6), reverse = direction
//     v_l == v_r == 0       -> BRAKE    (mode 8)
//     other (differential)  -> approximated by the dominant component and counted in
//                              differential_approx_cnt (legacy FSM has no arc mode)
//
// USE_VELOCITY_MODE = 1 emits the signed speeds as mode 10 VELOCITY; that mode has to
// be added to the sequencer first (see docs/INTEGRATION_GUIDE.md section 3).
//
// Command discipline:
//   * one command is issued per SET_VEL, and only when the requested motion actually
//     changed (deduplicated against the last issued command);
//   * the sequencer must assert cmd_ready; otherwise the request is dropped and the
//     next SET_VEL (20 Hz) retries;
//   * a latched stop / lost link issues exactly one BRAKE and then blocks until the
//     latch is cleared.
module host_link_motion_adapter #(
    parameter integer CLK_HZ = 50_000_000,
    parameter         USE_VELOCITY_MODE = 0,
    parameter integer MM_S_TO_UM_S = 1000,
    parameter integer ACCEL_STEP_UM_S2 = 100_000,
    parameter integer DECEL_STEP_UM_S2 = 200_000,
    parameter integer SPEED_LIMIT_MM_S = 300,
    parameter integer CMD_TIMEOUT_MS   = 3000
)(
    input  wire               clk,
    input  wire               rst_n,
    // ---- from uart_bridge_core ----
    input  wire signed [15:0] req_v_left,        // mm/s (already clamped by the link)
    input  wire signed [15:0] req_v_right,
    input  wire               setvel_strobe,     // one pulse per accepted SET_VEL
    input  wire               stop_latched,
    input  wire               link_ok,
    input  wire               link_lost,
    // ---- to motion_control_and_mode_fsm ----
    output reg                cmd_valid,
    output reg  [3:0]         cmd_mode,
    output reg  [1:0]         cmd_end,
    output reg                cmd_reverse,
    output reg  [31:0]        cmd_speed_l,
    output reg  [31:0]        cmd_speed_r,
    output reg  [31:0]        cmd_goal,
    output reg  [31:0]        cmd_timeout_ms,
    output reg  [31:0]        cmd_speed_limit,
    output reg  [31:0]        cmd_accel_step,
    output reg  [31:0]        cmd_decel_step,
    input  wire               cmd_ready,
    // ---- diagnostics ----
    output reg  [15:0]        cmds_issued,
    output reg  [15:0]        cmds_rejected,     // requested while not ready
    output reg  [15:0]        differential_approx_cnt
);
    localparam [3:0] M_FORWARD  = 4'd2;
    localparam [3:0] M_BACKWARD = 4'd3;
    localparam [3:0] M_ROTATE   = 4'd6;
    localparam [3:0] M_BRAKE    = 4'd8;
    localparam [3:0] M_VELOCITY = 4'd10;   // requires the sequencer to support it
    localparam [1:0] E_CONT     = 2'd0;

    localparam [2:0] S_IDLE = 3'd0, S_EVAL = 3'd1, S_ISSUE = 3'd2, S_BRAKE = 3'd3;

    // ---------------------------------------------------------------- helpers
    function [31:0] mag_um;
        input signed [15:0] v;
        reg   signed [31:0] m;
        begin
            m = (v < 0) ? -v : v;
            mag_um = m * MM_S_TO_UM_S;
        end
    endfunction

    // latest request
    reg signed [15:0] want_l, want_r;
    reg               req_pending;
    reg [2:0]         state;
    reg               brake_done;

    // last issued command (for dedup)
    reg [3:0]  last_mode;
    reg        last_rev;
    reg [31:0] last_spd;

    // ------------------------------------------------------- desired command
    wire signed [15:0] v_l = want_l;
    wire signed [15:0] v_r = want_r;
    wire both_zero = (v_l == 16'sd0) && (v_r == 16'sd0);
    wire same_sign = ((v_l >= 0 && v_r >= 0) || (v_l <= 0 && v_r <= 0));
    wire anti_phase = !same_sign;
    wire signed [15:0] avg_v  = (v_l + v_r) >>> 1;   // linear component
    wire signed [15:0] diff_v = (v_r - v_l) >>> 1;   // rotational component
    wire straight_ok = same_sign && (v_l == v_r);
    wire rotate_ok   = anti_phase && (v_l == -v_r);
    wire mag_avg_ge  = ((avg_v  >= 0 ? avg_v  : -avg_v) >=
                        (diff_v >= 0 ? diff_v : -diff_v));

    reg [3:0]  d_mode;
    reg        d_rev;
    reg [31:0] d_spd;
    reg        d_approx;

    always @(*) begin
        d_mode   = M_FORWARD;
        d_rev    = 1'b0;
        d_spd    = 32'd0;
        d_approx = 1'b0;
        if (both_zero) begin
            d_mode = M_BRAKE;
        end else if (USE_VELOCITY_MODE != 0) begin
            d_mode = M_VELOCITY;
            d_spd  = mag_um(avg_v);
        end else if (straight_ok) begin
            d_mode = (v_l >= 0) ? M_FORWARD : M_BACKWARD;
            d_rev  = (v_l < 0);
            d_spd  = mag_um(v_l);
        end else if (rotate_ok) begin
            d_mode = M_ROTATE;
            d_rev  = (v_l < 0);
            d_spd  = mag_um(v_l);
        end else begin
            d_approx = 1'b1;
            if (mag_avg_ge) begin
                d_mode = (avg_v >= 0) ? M_FORWARD : M_BACKWARD;
                d_rev  = (avg_v < 0);
                d_spd  = mag_um(avg_v);
            end else begin
                d_mode = M_ROTATE;
                d_rev  = (diff_v < 0);
                d_spd  = mag_um(diff_v);
            end
        end
    end

    wire desired_changed = (d_mode != last_mode) || (d_rev != last_rev) ||
                           (d_spd != last_spd) || (cmds_issued == 16'd0);

    // ---------------------------------------------------------------- sequencer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            cmd_valid      <= 1'b0;
            cmd_mode       <= 4'd0;
            cmd_end        <= E_CONT;
            cmd_reverse    <= 1'b0;
            cmd_speed_l    <= 32'd0;
            cmd_speed_r    <= 32'd0;
            cmd_goal       <= 32'd0;
            cmd_timeout_ms <= CMD_TIMEOUT_MS[31:0];
            cmd_speed_limit<= SPEED_LIMIT_MM_S * MM_S_TO_UM_S;
            cmd_accel_step <= ACCEL_STEP_UM_S2[31:0];
            cmd_decel_step <= DECEL_STEP_UM_S2[31:0];
            want_l         <= 16'sd0;
            want_r         <= 16'sd0;
            req_pending    <= 1'b0;
            last_mode      <= 4'd0;
            last_rev       <= 1'b0;
            last_spd       <= 32'd0;
            brake_done     <= 1'b0;
            cmds_issued    <= 16'd0;
            cmds_rejected  <= 16'd0;
            differential_approx_cnt <= 16'd0;
        end else begin
            cmd_valid <= 1'b0;

            // 1) latch a new request
            if (setvel_strobe) begin
                want_l      <= req_v_left;
                want_r      <= req_v_right;
                req_pending <= 1'b1;
                state       <= S_EVAL;
            end

            // 2) safety overrides everything: one BRAKE then block
            if (stop_latched || link_lost || !link_ok) begin
                want_l      <= 16'sd0;
                want_r      <= 16'sd0;
                req_pending <= 1'b0;
                if (!brake_done) state <= S_BRAKE;
            end else begin
                brake_done <= 1'b0;
            end

            case (state)
                S_EVAL: begin
                    if (!req_pending || !desired_changed || d_mode == M_BRAKE) begin
                        // nothing new to send, or a brake is handled elsewhere
                        if (d_mode == M_BRAKE && req_pending && cmd_ready) begin
                            state <= S_ISSUE;
                        end else begin
                            req_pending <= 1'b0;
                            state       <= S_IDLE;
                        end
                    end else if (cmd_ready) begin
                        state <= S_ISSUE;
                    end else begin
                        if (cmds_rejected != 16'hFFFF) cmds_rejected <= cmds_rejected + 16'd1;
                        state <= S_IDLE;     // retried by the next SET_VEL (20 Hz)
                    end
                end
                S_ISSUE: begin
                    cmd_valid      <= 1'b1;
                    cmd_mode       <= d_mode;
                    cmd_end        <= E_CONT;
                    cmd_reverse    <= d_rev;
                    cmd_speed_l    <= d_spd;
                    cmd_speed_r    <= d_spd;
                    cmd_goal       <= 32'd0;
                    cmd_timeout_ms <= CMD_TIMEOUT_MS[31:0];
                    last_mode      <= d_mode;
                    last_rev       <= d_rev;
                    last_spd       <= d_spd;
                    if (cmds_issued != 16'hFFFF) cmds_issued <= cmds_issued + 16'd1;
                    if (d_approx && differential_approx_cnt != 16'hFFFF)
                        differential_approx_cnt <= differential_approx_cnt + 16'd1;
                    req_pending    <= 1'b0;
                    state          <= S_IDLE;
                end
                S_BRAKE: begin
                    if (cmd_ready) begin
                        cmd_valid      <= 1'b1;
                        cmd_mode       <= M_BRAKE;
                        cmd_end        <= E_CONT;
                        cmd_reverse    <= 1'b0;
                        cmd_speed_l    <= 32'd0;
                        cmd_speed_r    <= 32'd0;
                        cmd_goal       <= 32'd0;
                        cmd_timeout_ms <= CMD_TIMEOUT_MS[31:0];
                        last_mode      <= M_BRAKE;
                        last_rev       <= 1'b0;
                        last_spd       <= 32'd0;
                        brake_done     <= 1'b1;
                        if (cmds_issued != 16'hFFFF) cmds_issued <= cmds_issued + 16'd1;
                        state          <= S_IDLE;
                    end
                end
                S_IDLE: begin
                    // idle: state may already be steered by the setvel/safety
                    // branches above, so this case item must not assign state
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
