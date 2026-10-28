`timescale 1ns/1ps
// Frame transmitter: streams AA 55 | TYPE | LEN | PAYLOAD | CRC16 | 0D
// one byte at a time into a ready/valid byte sink (the UART transmitter).
//
// The payload is read through rd_addr / rd_data so any buffer (telemetry snapshot,
// ACK registers) can be used without extra copies.
module frame_tx_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,        // one-cycle pulse
    input  wire  [7:0] frame_type,
    input  wire  [7:0] len,
    input  wire        ready,        // byte sink can accept a byte
    input  wire  [7:0] rd_data,      // payload[rd_addr]
    output reg   [4:0] rd_addr,
    output reg   [7:0] out_byte,
    output reg         out_valid,
    output reg         busy,
    output reg         done
);
    localparam [3:0]
        S_IDLE   = 4'd0,
        S_SYNC0  = 4'd1,
        S_SYNC1  = 4'd2,
        S_TYPE   = 4'd3,
        S_LEN    = 4'd4,
        S_PAY    = 4'd5,
        S_CRCLO  = 4'd6,
        S_CRCHI  = 4'd7,
        S_TAIL   = 4'd8;

    reg [3:0]  state;
    reg [7:0]  type_r, len_r, idx;

    reg        crc_init, crc_valid;
    reg  [7:0] crc_data;
    wire [15:0] crc_val;

    crc16_ccitt u_crc (
        .clk(clk), .rst_n(rst_n),
        .init(crc_init), .data(crc_data), .valid(crc_valid), .crc(crc_val)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; type_r <= 8'd0; len_r <= 8'd0; idx <= 8'd0;
            rd_addr <= 5'd0; out_byte <= 8'd0; out_valid <= 1'b0;
            busy <= 1'b0; done <= 1'b0;
            crc_init <= 1'b0; crc_valid <= 1'b0; crc_data <= 8'd0;
        end else begin
            out_valid <= 1'b0;
            done      <= 1'b0;
            crc_init  <= 1'b0;
            crc_valid <= 1'b0;

            if (state == S_IDLE) begin
                busy <= 1'b0;
                if (start) begin
                    type_r     <= frame_type;
                    len_r      <= len;
                    idx        <= 8'd0;
                    rd_addr    <= 5'd0;
                    crc_init   <= 1'b1;
                    busy       <= 1'b1;
                    state      <= S_SYNC0;
                end
            end else if (ready && !out_valid) begin
                case (state)
                    S_SYNC0: begin
                        out_byte <= 8'hAA;
                        out_valid <= 1'b1;
                        state <= S_SYNC1;
                    end
                    S_SYNC1: begin
                        out_byte <= 8'h55;
                        out_valid <= 1'b1;
                        state <= S_TYPE;
                    end
                    S_TYPE: begin
                        out_byte  <= type_r;
                        out_valid <= 1'b1;
                        crc_valid <= 1'b1;
                        crc_data  <= type_r;
                        state     <= S_LEN;
                    end
                    S_LEN: begin
                        out_byte  <= len_r;
                        out_valid <= 1'b1;
                        crc_valid <= 1'b1;
                        crc_data  <= len_r;
                        idx       <= 8'd0;
                        rd_addr   <= 5'd0;
                        state     <= (len_r == 8'd0) ? S_CRCLO : S_PAY;
                    end
                    S_PAY: begin
                        out_byte  <= rd_data;
                        out_valid <= 1'b1;
                        crc_valid <= 1'b1;
                        crc_data  <= rd_data;
                        idx       <= idx + 8'd1;
                        rd_addr   <= rd_addr + 5'd1;
                        if (idx + 8'd1 >= len_r) state <= S_CRCLO;
                    end
                    S_CRCLO: begin
                        out_byte  <= crc_val[7:0];
                        out_valid <= 1'b1;
                        state     <= S_CRCHI;
                    end
                    S_CRCHI: begin
                        out_byte  <= crc_val[15:8];
                        out_valid <= 1'b1;
                        state     <= S_TAIL;
                    end
                    S_TAIL: begin
                        out_byte  <= 8'h0D;
                        out_valid <= 1'b1;
                        state     <= S_IDLE;
                        busy      <= 1'b0;
                        done      <= 1'b1;
                    end
                    default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule
