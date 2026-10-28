`timescale 1ns/1ps
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
// Bit-serial LFSR unrolled to 8 steps per byte -> ~40 LUT, no ROM/table.
// Must match docs/PROTOCOL_v1.md section 2.2 and tools/protocol.py: crc16_ccitt().
module crc16_ccitt (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init,        // synchronous reload with 0xFFFF (start of frame)
    input  wire  [7:0] data,
    input  wire        valid,       // consume one byte when high
    output reg  [15:0] crc
);
    function [15:0] crc16_next;
        input [15:0] c;
        input [7:0]  d;
        integer i;
        reg [15:0] x;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc16_next = x;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      crc <= 16'hFFFF;
        else if (init)   crc <= 16'hFFFF;
        else if (valid)  crc <= crc16_next(crc, data);
    end
endmodule
