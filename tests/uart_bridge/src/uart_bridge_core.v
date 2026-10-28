`timescale 1ns/1ps
// Host link core: transport-agnostic byte in/out + framing + decoding + safety +
// telemetry. The transport adapter (UART today, UDP-over-1GbE later) only has to
// provide rx_byte/rx_valid and consume tx_byte/tx_valid/tx_ready.
//
// Safety (required by PLAN 9.4):
//   * target speeds are forced to 0 while STOP is latched, the link is lost, the
//     heartbeat is stale, or no SET_VEL arrived for CMD_TIMEOUT_MS
//   * CLEAR_FAULT only releases the latches when the link is alive again
//   * no PWM is generated here: target_v_* are plain registers for the motor side
module uart_bridge_core #(
    parameter integer CLK_HZ             = 50_000_000,
    parameter integer TELEM_PERIOD_MS    = 20,
    parameter integer CMD_TIMEOUT_MS     = 500,
    parameter integer HB_TIMEOUT_MS      = 300,   // 1.5x the 200 ms heartbeat period
    parameter integer BYTE_TIMEOUT_CYCLES= 250_000,   // 5 ms @ 50 MHz
    parameter integer DEFAULT_LIMIT_MM_S = 300,
    parameter integer MAX_LEN            = 32,
    // ---- resource knobs (defaults keep the verified full-feature behaviour) ----
    parameter integer CNT_W              = 16,  // diagnostics counter width
    parameter integer ENABLE_ACK         = 1    // 0: no ACK frames (saves ~120 CLS)
)(
    input  wire              clk,
    input  wire              rst_n,
    // transport-agnostic byte stream
    input  wire  [7:0]       rx_byte,
    input  wire              rx_valid,
    output wire  [7:0]       tx_byte,
    output wire              tx_valid,
    input  wire              tx_ready,
    // sensor snapshot (wired to real encoders / IMU by the board top)
    input  wire signed [31:0] enc_l,
    input  wire signed [31:0] enc_r,
    input  wire signed [15:0] yaw_cdeg,
    input  wire signed [15:0] pitch_cdeg,
    input  wire signed [15:0] roll_cdeg,
    input  wire signed [15:0] speed_l_mm_s,
    input  wire signed [15:0] speed_r_mm_s,
    input  wire        [7:0]  ext_fault,      // bit2 = hardware estop, others reserved
    // command interface (consumed by the motor side later; unused in the test top)
    output wire signed [15:0] target_v_left,
    output wire signed [15:0] target_v_right,
    output wire               stop_latched_out,
    output wire               link_ok_out,
    output wire               link_lost_out,
    output wire        [7:0]  fault_out,
    // diagnostics
    output wire [15:0]        good_cnt,
    output wire [15:0]        crc_err_cnt,
    output wire [15:0]        len_err_cnt,
    output wire [15:0]        tail_err_cnt,
    output wire [15:0]        sync_err_cnt,
    output wire [15:0]        timeout_cnt,
    output wire [15:0]        unsupported_cnt,
    output wire [15:0]        telem_sent_cnt,
    output wire [15:0]        ack_sent_cnt,
    output wire [15:0]        ack_drop_cnt,
    output wire [7:0]         hb_seq_out,
    output wire [7:0]         cmd_seq_out,
    output wire [15:0]        hb_age_ms_out
);
    localparam [7:0] T_TELEM = 8'h81;
    localparam [7:0] T_ACK   = 8'h82;

    // ------------------------------------------------------------------ ticks
    reg [15:0] ms_div;
    reg        ms_tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ms_div <= 16'd0; ms_tick <= 1'b0; end
        else begin
            if (ms_div >= (CLK_HZ / 1000) - 1) begin ms_div <= 16'd0; ms_tick <= 1'b1; end
            else begin ms_div <= ms_div + 16'd1; ms_tick <= 1'b0; end
        end
    end

    // -------------------------------------------------------------- receiver
    wire        frame_valid;
    wire [7:0]  frame_type, frame_len;
    wire [63:0] payload_head;

    host_frame_rx #(
        .MAX_LEN(MAX_LEN),
        .TIMEOUT_CYCLES(BYTE_TIMEOUT_CYCLES),
        .CNT_W(CNT_W)
    ) u_rx (
        .clk(clk), .rst_n(rst_n),
        .byte_in(rx_byte), .byte_valid(rx_valid),
        .frame_valid(frame_valid), .frame_type(frame_type),
        .frame_len(frame_len), .payload_head(payload_head),
        .good_cnt(good_cnt), .crc_err_cnt(crc_err_cnt), .len_err_cnt(len_err_cnt),
        .tail_err_cnt(tail_err_cnt), .sync_err_cnt(sync_err_cnt),
        .timeout_cnt(timeout_cnt)
    );

    // --------------------------------------------------------------- decoder
    wire signed [15:0] req_v_left, req_v_right;
    wire        [7:0]  cmd_seq, hb_seq, ack_type, ack_seq, ack_status;
    wire               stop_latched, setvel_strobe;
    wire               hb_strobe, clear_fault_strobe, over_limit;
    wire               ack_strobe;
    wire        [15:0] unsupported_cnt_i;
    wire        [15:0] limit_echo_unused;
    wire               link_ok, link_lost;
    wire        [15:0] hb_age_ms;

    host_cmd_decode #(
        .DEFAULT_LIMIT_MM_S(DEFAULT_LIMIT_MM_S)
    ) u_decode (
        .clk(clk), .rst_n(rst_n),
        .frame_valid(frame_valid), .frame_type(frame_type),
        .frame_len(frame_len), .payload_head(payload_head),
        .link_ok(link_ok), .link_lost(link_lost),
        .req_v_left(req_v_left), .req_v_right(req_v_right),
        .cmd_seq(cmd_seq), .stop_latched(stop_latched), .limit_mm_s(limit_echo_unused),
        .hb_strobe(hb_strobe), .hb_seq(hb_seq),
        .setvel_strobe(setvel_strobe), .clear_fault_strobe(clear_fault_strobe),
        .over_limit(over_limit),
        .ack_strobe(ack_strobe), .ack_type(ack_type),
        .ack_seq(ack_seq), .ack_status(ack_status),
        .unsupported_cnt(unsupported_cnt_i)
    );

    // -------------------------------------------------------------- watchdog
    host_link_watchdog #(
        .TIMEOUT_MS(HB_TIMEOUT_MS)
    ) u_wd (
        .clk(clk), .rst_n(rst_n), .ms_tick(ms_tick),
        .hb_strobe(hb_strobe), .clear_latched(clear_fault_strobe),
        .link_ok(link_ok), .link_lost(link_lost), .hb_age_ms(hb_age_ms)
    );

    // --------------------------------------------------- command time-out
    reg [15:0] cmd_age_ms;
    reg        cmd_timeout;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_age_ms  <= 16'd0;
            cmd_timeout <= 1'b1;
        end else if (setvel_strobe) begin
            cmd_age_ms  <= 16'd0;
            cmd_timeout <= 1'b0;
        end else if (ms_tick) begin
            if (cmd_age_ms < 16'hFFFF) cmd_age_ms <= cmd_age_ms + 16'd1;
            if (cmd_age_ms >= CMD_TIMEOUT_MS) cmd_timeout <= 1'b1;
        end
    end

    // ------------------------------------------------------- safety arbiter
    wire safety_stop = stop_latched | link_lost | ~link_ok | cmd_timeout;
    assign target_v_left  = safety_stop ? 16'sd0 : req_v_left;
    assign target_v_right = safety_stop ? 16'sd0 : req_v_right;
    assign stop_latched_out = stop_latched;
    assign link_ok_out      = link_ok;
    assign link_lost_out    = link_lost;

    // ---------------------------------------------------------- telemetry
    reg  [15:0] telem_div;
    wire        telem_tick = ms_tick && (telem_div >= TELEM_PERIOD_MS - 1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) telem_div <= 16'd0;
        else if (ms_tick) begin
            if (telem_div >= TELEM_PERIOD_MS - 1) telem_div <= 16'd0;
            else telem_div <= telem_div + 16'd1;
        end
    end

    reg  telem_snapshot;
    wire [4:0] tx_rd_addr;
    wire [7:0] tx_rd_data;
    wire [7:0] telem_data;

    // sticky diagnostics for the telemetry fault bitmap
    reg        crc_err_sticky, over_limit_sticky;
    reg [15:0] crc_err_cnt_d;
    wire       crc_err_new = (crc_err_cnt != crc_err_cnt_d);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_err_cnt_d     <= 16'd0;
            crc_err_sticky    <= 1'b0;
            over_limit_sticky <= 1'b0;
        end else begin
            crc_err_cnt_d <= crc_err_cnt;
            if (crc_err_new)          crc_err_sticky    <= 1'b1;
            else if (frame_valid)     crc_err_sticky    <= 1'b0;
            if (over_limit)           over_limit_sticky <= 1'b1;
            else if (telem_snapshot)  over_limit_sticky <= 1'b0;
        end
    end

    assign fault_out = {2'b00,
                        cmd_timeout,          // bit5
                        crc_err_sticky,       // bit4
                        over_limit_sticky,    // bit3
                        ext_fault[2],         // bit2 estop
                        link_lost,            // bit1
                        stop_latched};        // bit0

    telem_builder u_telem (
        .clk(clk), .rst_n(rst_n), .snapshot(telem_snapshot),
        .enc_l(enc_l), .enc_r(enc_r),
        .yaw_cdeg(yaw_cdeg), .pitch_cdeg(pitch_cdeg), .roll_cdeg(roll_cdeg),
        .speed_l_mm_s(speed_l_mm_s), .speed_r_mm_s(speed_r_mm_s),
        .fault(fault_out), .link_seq(hb_seq), .cmd_seq(cmd_seq),
        .hb_age_ms(hb_age_ms),
        .addr(tx_rd_addr), .data(telem_data)
    );

    // ------------------------------------------- single TX / ACK arbiter block
    reg        ack_pending;
    reg [7:0]  ack_type_q, ack_seq_q, ack_status_q;
    reg [15:0] ack_drop_cnt_i, ack_sent_cnt_i, telem_sent_cnt_i;
    reg        telem_pending;
    reg        tx_start;
    reg        tx_is_ack;
    reg [7:0]  tx_type_i, tx_len_i;
    wire       tx_busy, tx_done;

    reg [7:0] ack_byte;
    always @(*) begin
        case (tx_rd_addr[1:0])
            2'd0:    ack_byte = ack_type_q;
            2'd1:    ack_byte = ack_seq_q;
            default: ack_byte = ack_status_q;
        endcase
    end
    assign tx_rd_data = tx_is_ack ? ack_byte : telem_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_pending     <= 1'b0;
            ack_type_q      <= 8'd0;
            ack_seq_q       <= 8'd0;
            ack_status_q    <= 8'd0;
            ack_drop_cnt_i  <= 16'd0;
            ack_sent_cnt_i  <= 16'd0;
            telem_sent_cnt_i<= 16'd0;
            telem_pending   <= 1'b0;
            tx_start        <= 1'b0;
            tx_is_ack       <= 1'b0;
            tx_type_i       <= 8'd0;
            tx_len_i        <= 8'd0;
            telem_snapshot  <= 1'b0;
        end else begin
            tx_start       <= 1'b0;
            telem_snapshot <= 1'b0;

            // 1) new ACK request from the decoder (disabled in the lite build)
            if (ENABLE_ACK != 0 && ack_strobe) begin
                if (!ack_pending) begin
                    ack_pending  <= 1'b1;
                    ack_type_q   <= ack_type;
                    ack_seq_q    <= ack_seq;
                    ack_status_q <= ack_status;
                end else if (ack_drop_cnt_i != 16'hFFFF) begin
                    ack_drop_cnt_i <= ack_drop_cnt_i + 16'd1;
                end
            end

            // 2) periodic telemetry request
            if (telem_tick) telem_pending <= 1'b1;

            // 3) arbitration: ACK has priority over TELEM
            if (!tx_busy && !tx_start) begin
                if (ENABLE_ACK != 0 && ack_pending) begin
                    ack_pending  <= 1'b0;
                    tx_is_ack    <= 1'b1;
                    tx_type_i    <= T_ACK;
                    tx_len_i     <= 8'd3;
                    tx_start     <= 1'b1;
                    if (ack_sent_cnt_i != 16'hFFFF) ack_sent_cnt_i <= ack_sent_cnt_i + 16'd1;
                end else if (telem_pending) begin
                    telem_pending  <= 1'b0;
                    telem_snapshot <= 1'b1;
                    tx_is_ack      <= 1'b0;
                    tx_type_i      <= T_TELEM;
                    tx_len_i       <= 8'd23;
                    tx_start       <= 1'b1;
                    if (telem_sent_cnt_i != 16'hFFFF) telem_sent_cnt_i <= telem_sent_cnt_i + 16'd1;
                end
            end
        end
    end

    frame_tx_engine u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tx_start), .frame_type(tx_type_i), .len(tx_len_i),
        .ready(tx_ready), .rd_data(tx_rd_data), .rd_addr(tx_rd_addr),
        .out_byte(tx_byte), .out_valid(tx_valid), .busy(tx_busy), .done(tx_done)
    );

    assign unsupported_cnt = unsupported_cnt_i;
    assign telem_sent_cnt  = telem_sent_cnt_i;
    assign ack_sent_cnt    = ack_sent_cnt_i;
    assign ack_drop_cnt    = ack_drop_cnt_i;
    assign hb_seq_out      = hb_seq;
    assign cmd_seq_out     = cmd_seq;
    assign hb_age_ms_out   = hb_age_ms;
endmodule
