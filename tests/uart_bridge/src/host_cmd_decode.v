`timescale 1ns/1ps
// Command decoder for the host link.
//
// Pure decoding + latching: it never touches PWM or motor pins. The effective speed
// is computed in uart_bridge_core after the safety arbitration.
//
// Payload layout (little endian, see docs/PROTOCOL_v1.md section 3.1):
//   SET_VEL     : int16 v_left | int16 v_right | uint8 seq
//   SET_LIMIT   : uint16 limit_mm_s
//   HEARTBEAT   : uint8 seq
//   STOP        : -
//   CLEAR_FAULT : -
module host_cmd_decode #(
    parameter integer DEFAULT_LIMIT_MM_S = 300
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              frame_valid,
    input  wire  [7:0]       frame_type,
    input  wire  [7:0]       frame_len,
    input  wire  [63:0]      payload_head,
    input  wire              link_ok,        // heartbeat fresh
    input  wire              link_lost,      // latched link loss
    // command outputs (registered)
    output reg   signed [15:0] req_v_left,
    output reg   signed [15:0] req_v_right,
    output reg   [7:0]       cmd_seq,        // echo of the last accepted SET_VEL seq
    output reg               stop_latched,
    output reg   [15:0]      limit_mm_s,
    output reg               hb_strobe,
    output reg   [7:0]       hb_seq,
    output reg               setvel_strobe,
    output reg               clear_fault_strobe,
    output reg               over_limit,
    // ack output (one-cycle strobe, core queues it)
    output reg               ack_strobe,
    output reg   [7:0]       ack_type,
    output reg   [7:0]       ack_seq,
    output reg   [7:0]       ack_status,
    // unsupported command counter
    output reg   [15:0]      unsupported_cnt
);
    localparam [7:0] T_SET_VEL     = 8'h01;
    localparam [7:0] T_STOP        = 8'h02;
    localparam [7:0] T_HEARTBEAT   = 8'h03;
    localparam [7:0] T_SET_LIMIT   = 8'h04;
    localparam [7:0] T_CLEAR_FAULT = 8'h05;

    localparam [7:0] ACK_OK          = 8'h00;
    localparam [7:0] ACK_REJ_LINK    = 8'h01;
    localparam [7:0] ACK_REJ_STOP    = 8'h02;
    localparam [7:0] ACK_REJ_LIMIT   = 8'h03;
    localparam [7:0] ACK_UNSUPPORTED = 8'h04;

    wire signed [15:0] pl_v_left  = payload_head[15:0];
    wire signed [15:0] pl_v_right = payload_head[31:16];
    wire        [7:0]  pl_seq     = payload_head[39:32];
    wire        [15:0] pl_limit   = payload_head[15:0];

    // clamp magnitude of a signed speed to the configured limit
    function signed [15:0] clamp_speed;
        input signed [15:0] v;
        input        [15:0] lim;
        integer mag;
        begin
            mag = (v < 0) ? -v : v;
            if (mag > lim) clamp_speed = (v < 0) ? -$signed({1'b0, lim}) : $signed({1'b0, lim});
            else           clamp_speed = v;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_v_left <= 16'sd0; req_v_right <= 16'sd0;
            cmd_seq <= 8'd0; stop_latched <= 1'b0;
            limit_mm_s <= DEFAULT_LIMIT_MM_S[15:0];
            hb_strobe <= 1'b0; hb_seq <= 8'd0;
            setvel_strobe <= 1'b0; clear_fault_strobe <= 1'b0;
            over_limit <= 1'b0;
            ack_strobe <= 1'b0; ack_type <= 8'd0; ack_seq <= 8'd0; ack_status <= 8'd0;
            unsupported_cnt <= 16'd0;
        end else begin
            hb_strobe          <= 1'b0;
            setvel_strobe      <= 1'b0;
            clear_fault_strobe <= 1'b0;
            over_limit         <= 1'b0;
            ack_strobe         <= 1'b0;

            if (frame_valid) begin
                case (frame_type)
                    T_SET_VEL: begin
                        if (frame_len != 8'd5) begin
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= 8'd0; ack_status <= ACK_UNSUPPORTED;
                            unsupported_cnt <= unsupported_cnt + 16'd1;
                        end else if (link_lost) begin
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= pl_seq; ack_status <= ACK_REJ_LINK;
                        end else if (stop_latched) begin
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= pl_seq; ack_status <= ACK_REJ_STOP;
                        end else begin
                            req_v_left  <= clamp_speed(pl_v_left,  limit_mm_s);
                            req_v_right <= clamp_speed(pl_v_right, limit_mm_s);
                            cmd_seq     <= pl_seq;
                            setvel_strobe <= 1'b1;
                            if ((pl_v_left > $signed({1'b0, limit_mm_s})) ||
                                (pl_v_left < -$signed({1'b0, limit_mm_s})) ||
                                (pl_v_right > $signed({1'b0, limit_mm_s})) ||
                                (pl_v_right < -$signed({1'b0, limit_mm_s}))) begin
                                over_limit <= 1'b1;
                                ack_strobe <= 1'b1; ack_type <= frame_type;
                                ack_seq <= pl_seq; ack_status <= ACK_REJ_LIMIT;
                            end else begin
                                ack_strobe <= 1'b1; ack_type <= frame_type;
                                ack_seq <= pl_seq; ack_status <= ACK_OK;
                            end
                        end
                    end
                    T_STOP: begin
                        stop_latched <= 1'b1;
                        req_v_left   <= 16'sd0;
                        req_v_right  <= 16'sd0;
                        ack_strobe   <= 1'b1; ack_type <= frame_type;
                        ack_seq      <= 8'd0; ack_status <= ACK_OK;
                    end
                    T_HEARTBEAT: begin
                        if (frame_len == 8'd1) begin
                            hb_strobe <= 1'b1;
                            hb_seq    <= payload_head[7:0];
                        end
                    end
                    T_SET_LIMIT: begin
                        if (frame_len == 8'd2) begin
                            if (pl_limit > 16'd0) limit_mm_s <= pl_limit;
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= 8'd0; ack_status <= ACK_OK;
                        end else begin
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= 8'd0; ack_status <= ACK_UNSUPPORTED;
                            unsupported_cnt <= unsupported_cnt + 16'd1;
                        end
                    end
                    T_CLEAR_FAULT: begin
                        if (link_ok) begin
                            stop_latched <= 1'b0;
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= 8'd0; ack_status <= ACK_OK;
                        end else begin
                            ack_strobe <= 1'b1; ack_type <= frame_type;
                            ack_seq <= 8'd0; ack_status <= ACK_REJ_LINK;
                        end
                        clear_fault_strobe <= 1'b1;
                    end
                    default: begin
                        ack_strobe <= 1'b1; ack_type <= frame_type;
                        ack_seq <= 8'd0; ack_status <= ACK_UNSUPPORTED;
                        unsupported_cnt <= unsupported_cnt + 16'd1;
                    end
                endcase
            end
        end
    end
endmodule
