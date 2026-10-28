`timescale 1ns/1ps
// Oversampled 8N1 UART receiver with mid-bit sampling and frame-error detection.
// Transport adapter for the host link: converts a serial line into (byte, valid).
module uart_rx_8n1 #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        data_valid,   // one-cycle pulse per received byte
    output reg        frame_error   // one-cycle pulse if the stop bit was low
);
    localparam integer BIT_CYCLES = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer HALF       = BIT_CYCLES / 2;

    (* ASYNC_REG = "TRUE" *) reg [2:0] sync;
    wire rx_s = sync[2];

    localparam [1:0] S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;
    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) sync <= 3'b111;
        else        sync <= {sync[1:0], rx};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; cnt <= 16'd0; bit_idx <= 3'd0; shreg <= 8'd0;
            data <= 8'd0; data_valid <= 1'b0; frame_error <= 1'b0;
        end else begin
            data_valid  <= 1'b0;
            frame_error <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (!rx_s) begin cnt <= 16'd0; state <= S_START; end
                end
                S_START: begin
                    if (cnt == HALF - 1) begin
                        cnt <= 16'd0;
                        if (!rx_s) begin bit_idx <= 3'd0; state <= S_DATA; end
                        else             state <= S_IDLE;      // glitch, abort
                    end else cnt <= cnt + 16'd1;
                end
                S_DATA: begin
                    if (cnt == BIT_CYCLES - 1) begin
                        cnt   <= 16'd0;
                        shreg <= {rx_s, shreg[7:1]};             // LSB first
                        if (bit_idx == 3'd7) state <= S_STOP;
                        else                 bit_idx <= bit_idx + 3'd1;
                    end else cnt <= cnt + 16'd1;
                end
                S_STOP: begin
                    if (cnt == BIT_CYCLES - 1) begin
                        cnt         <= 16'd0;
                        state       <= S_IDLE;
                        data        <= shreg;
                        data_valid  <= 1'b1;
                        frame_error <= ~rx_s;
                    end else cnt <= cnt + 16'd1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
