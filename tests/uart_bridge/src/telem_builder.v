`timescale 1ns/1ps
// Telemetry payload snapshot: 23 bytes, little endian (docs/PROTOCOL_v1.md 3.2).
//
// The snapshot is taken by the core only when the transmitter is idle, so the buffer
// is never modified while a frame is being streamed out.
module telem_builder (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               snapshot,
    input  wire signed [31:0] enc_l,
    input  wire signed [31:0] enc_r,
    input  wire signed [15:0] yaw_cdeg,
    input  wire signed [15:0] pitch_cdeg,
    input  wire signed [15:0] roll_cdeg,
    input  wire signed [15:0] speed_l_mm_s,
    input  wire signed [15:0] speed_r_mm_s,
    input  wire        [7:0]  fault,
    input  wire        [7:0]  link_seq,
    input  wire        [7:0]  cmd_seq,
    input  wire        [15:0] hb_age_ms,
    input  wire        [4:0]  addr,
    output wire        [7:0]  data
);
    reg [7:0] pay_buf [0:22];

    assign data = pay_buf[addr];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pay_buf[0]  <= 8'd0;  pay_buf[1]  <= 8'd0;  pay_buf[2]  <= 8'd0;  pay_buf[3]  <= 8'd0;
            pay_buf[4]  <= 8'd0;  pay_buf[5]  <= 8'd0;  pay_buf[6]  <= 8'd0;  pay_buf[7]  <= 8'd0;
            pay_buf[8]  <= 8'd0;  pay_buf[9]  <= 8'd0;  pay_buf[10] <= 8'd0;  pay_buf[11] <= 8'd0;
            pay_buf[12] <= 8'd0;  pay_buf[13] <= 8'd0;  pay_buf[14] <= 8'd0;  pay_buf[15] <= 8'd0;
            pay_buf[16] <= 8'd0;  pay_buf[17] <= 8'd0;  pay_buf[18] <= 8'd0;  pay_buf[19] <= 8'd0;
            pay_buf[20] <= 8'd0;  pay_buf[21] <= 8'd0;  pay_buf[22] <= 8'd0;
        end else if (snapshot) begin
            pay_buf[0]  <= enc_l[7:0];
            pay_buf[1]  <= enc_l[15:8];
            pay_buf[2]  <= enc_l[23:16];
            pay_buf[3]  <= enc_l[31:24];
            pay_buf[4]  <= enc_r[7:0];
            pay_buf[5]  <= enc_r[15:8];
            pay_buf[6]  <= enc_r[23:16];
            pay_buf[7]  <= enc_r[31:24];
            pay_buf[8]  <= yaw_cdeg[7:0];
            pay_buf[9]  <= yaw_cdeg[15:8];
            pay_buf[10] <= pitch_cdeg[7:0];
            pay_buf[11] <= pitch_cdeg[15:8];
            pay_buf[12] <= roll_cdeg[7:0];
            pay_buf[13] <= roll_cdeg[15:8];
            pay_buf[14] <= speed_l_mm_s[7:0];
            pay_buf[15] <= speed_l_mm_s[15:8];
            pay_buf[16] <= speed_r_mm_s[7:0];
            pay_buf[17] <= speed_r_mm_s[15:8];
            pay_buf[18] <= fault;
            pay_buf[19] <= link_seq;
            pay_buf[20] <= cmd_seq;
            pay_buf[21] <= hb_age_ms[7:0];
            pay_buf[22] <= hb_age_ms[15:8];
        end
    end
endmodule
