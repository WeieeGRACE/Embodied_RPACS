`timescale 1ns/1ps
// Frame parser for the host link: AA 55 | TYPE | LEN | PAYLOAD | CRC16 | 0D.
//
// Transport agnostic: it only consumes (byte_in, byte_valid). A UART receiver or a
// future Ethernet/UDP byte source both drive the same port, see docs/PROTOCOL_v1.md.
//
// Safety rule: any parse error discards the frame and never updates the command
// interface; the first 8 payload bytes are exposed on payload_head (enough for every
// command in v1; LEN is still validated against MAX_LEN).
module host_frame_rx #(
    parameter integer MAX_LEN        = 32,
    parameter integer TIMEOUT_CYCLES = 250_000,  // 5 ms @ 50 MHz
    parameter integer CNT_W          = 16
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire [7:0]       byte_in,
    input  wire             byte_valid,
    // decoded frame (frame_valid is a one-cycle pulse)
    output reg              frame_valid,
    output reg  [7:0]       frame_type,
    output reg  [7:0]       frame_len,
    output reg  [63:0]      payload_head,
    // diagnostics (saturating)
    output reg  [CNT_W-1:0] good_cnt,
    output reg  [CNT_W-1:0] crc_err_cnt,
    output reg  [CNT_W-1:0] len_err_cnt,
    output reg  [CNT_W-1:0] tail_err_cnt,
    output reg  [CNT_W-1:0] sync_err_cnt,
    output reg  [CNT_W-1:0] timeout_cnt
);
    localparam [3:0]
        S_SYNC0  = 4'd0,
        S_SYNC1  = 4'd1,
        S_TYPE   = 4'd2,
        S_FEED   = 4'd3,   // feed the type byte into the CRC (no byte consumed)
        S_LEN    = 4'd4,
        S_PAY    = 4'd5,
        S_CRC0   = 4'd6,
        S_CRC1   = 4'd7,
        S_TAIL   = 4'd8;

    reg [3:0]  state;
    reg [7:0]  type_r, len_r, idx;
    reg [7:0]  feed_r;              // byte waiting to be pushed into the CRC
    reg [15:0] crc_rx;              // CRC taken from the frame
    reg [31:0] idle_cnt;            // cycles since the last received byte

    // ---- CRC engine ------------------------------------------------------
    reg        crc_init, crc_valid;
    reg  [7:0] crc_data;
    wire [15:0] crc_val;

    crc16_ccitt u_crc (
        .clk(clk), .rst_n(rst_n),
        .init(crc_init), .data(crc_data), .valid(crc_valid), .crc(crc_val)
    );

    wire byte_now = byte_valid;
    wire [7:0] b  = byte_in;

    function [CNT_W-1:0] sat_inc;
        input [CNT_W-1:0] v;
        begin sat_inc = (v == {CNT_W{1'b1}}) ? v : (v + 1'b1); end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_SYNC0;
            type_r <= 8'd0; len_r <= 8'd0; idx <= 8'd0; feed_r <= 8'd0;
            crc_rx <= 16'd0; idle_cnt <= 32'd0;
            frame_valid <= 1'b0; frame_type <= 8'd0; frame_len <= 8'd0;
            payload_head <= 64'd0;
            good_cnt <= {CNT_W{1'b0}}; crc_err_cnt <= {CNT_W{1'b0}};
            len_err_cnt <= {CNT_W{1'b0}}; tail_err_cnt <= {CNT_W{1'b0}};
            sync_err_cnt <= {CNT_W{1'b0}}; timeout_cnt <= {CNT_W{1'b0}};
            crc_init <= 1'b0; crc_valid <= 1'b0; crc_data <= 8'd0;
        end else begin
            frame_valid <= 1'b0;
            crc_init    <= 1'b0;
            crc_valid   <= 1'b0;

            // ---- inter-byte timeout -------------------------------------
            if (state == S_SYNC0) begin
                idle_cnt <= 32'd0;
            end else if (byte_now) begin
                idle_cnt <= 32'd0;
            end else if (idle_cnt >= TIMEOUT_CYCLES) begin
                idle_cnt  <= 32'd0;
                state     <= S_SYNC0;
                timeout_cnt <= sat_inc(timeout_cnt);
            end else begin
                idle_cnt <= idle_cnt + 32'd1;
            end

            // ---- byte level state machine --------------------------------
            case (state)
                S_SYNC0: begin
                    if (byte_now) begin
                        if (b == 8'hAA) state <= S_SYNC1;
                        else            sync_err_cnt <= sat_inc(sync_err_cnt);
                    end
                end
                S_SYNC1: begin
                    if (byte_now) begin
                        if (b == 8'h55) begin
                            state <= S_TYPE;
                        end else begin
                            sync_err_cnt <= sat_inc(sync_err_cnt);
                            state <= (b == 8'hAA) ? S_SYNC1 : S_SYNC0;
                        end
                    end
                end
                S_TYPE: begin
                    if (byte_now) begin
                        type_r    <= b;
                        feed_r    <= b;
                        crc_init  <= 1'b1;       // restart CRC, 0xFFFF
                        state     <= S_FEED;
                    end
                end
                S_FEED: begin
                    crc_valid <= 1'b1;           // push the type byte
                    crc_data  <= feed_r;
                    idx       <= 8'd0;
                    payload_head <= 64'd0;
                    state     <= S_LEN;
                end
                S_LEN: begin
                    if (byte_now) begin
                        if (b > MAX_LEN) begin
                            len_err_cnt <= sat_inc(len_err_cnt);
                            state <= S_SYNC0;
                        end else begin
                            len_r     <= b;
                            crc_valid <= 1'b1;   // the LEN byte itself is covered
                            crc_data  <= b;
                            state     <= (b == 8'd0) ? S_CRC0 : S_PAY;
                        end
                    end
                end
                S_PAY: begin
                    if (idx >= len_r) begin
                        state <= S_CRC0;         // no byte needed here
                    end else if (byte_now) begin
                        crc_valid <= 1'b1;
                        crc_data  <= b;
                        if (idx < 8'd8) payload_head[{idx[2:0], 3'b000} +: 8] <= b;
                        idx   <= idx + 8'd1;
                        if (idx + 8'd1 >= len_r) state <= S_CRC0;
                    end
                end
                S_CRC0: begin
                    if (byte_now) begin
                        crc_rx[7:0] <= b;
                        state <= S_CRC1;
                    end
                end
                S_CRC1: begin
                    if (byte_now) begin
                        crc_rx[15:8] <= b;
                        state <= S_TAIL;
                    end
                end
                S_TAIL: begin
                    if (byte_now) begin
                        if (b != 8'h0D) begin
                            tail_err_cnt <= sat_inc(tail_err_cnt);
                        end else if (crc_rx != crc_val) begin
                            crc_err_cnt <= sat_inc(crc_err_cnt);
                        end else begin
                            good_cnt      <= sat_inc(good_cnt);
                            frame_valid   <= 1'b1;
                            frame_type    <= type_r;
                            frame_len     <= len_r;
                        end
                        state <= S_SYNC0;
                    end
                end
                default: state <= S_SYNC0;
            endcase
        end
    end
endmodule
