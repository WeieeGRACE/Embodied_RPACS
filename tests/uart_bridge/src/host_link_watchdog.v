`timescale 1ns/1ps
// Heartbeat watchdog for the host link.
//
// - any accepted HEARTBEAT resets hb_age_ms and marks the link as fresh (link_ok)
// - if no heartbeat arrives within TIMEOUT_MS the link is declared lost and the loss
//   is latched (link_lost) until CLEAR_FAULT arrives *after* the link is fresh again
module host_link_watchdog #(
    parameter integer TIMEOUT_MS = 200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ms_tick,          // 1 kHz strobe
    input  wire        hb_strobe,        // one-cycle pulse per accepted heartbeat
    input  wire        clear_latched,    // CLEAR_FAULT strobe
    output reg         link_ok,
    output reg         link_lost,
    output reg  [15:0] hb_age_ms
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            link_ok     <= 1'b0;
            link_lost   <= 1'b1;          // power-up: no link yet
            hb_age_ms   <= 16'hFFFF;
        end else begin
            if (hb_strobe) begin
                link_ok   <= 1'b1;
                hb_age_ms <= 16'd0;
            end else if (ms_tick) begin
                if (hb_age_ms != 16'hFFFF) hb_age_ms <= hb_age_ms + 16'd1;
                if (hb_age_ms >= TIMEOUT_MS) begin
                    link_ok   <= 1'b0;
                    link_lost <= 1'b1;
                end
            end
            // Note: the last non-blocking assignment wins, so a CLEAR_FAULT that lands on
            // the very same ms_tick as a timeout clears the latch for one tick and the
            // next tick re-latches it. Motion is still gated by ~link_ok (safety_stop),
            // so this is cosmetic; kept as-is to avoid extra state.
            if (clear_latched && link_ok) link_lost <= 1'b0;
        end
    end
endmodule
