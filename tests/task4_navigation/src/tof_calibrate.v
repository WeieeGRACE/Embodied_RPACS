`timescale 1ns/1ps
// 单路 ToF 固定偏置标定：把实测偏置减掉（front +60 / left +15 / right +70 mm，
// 2026-10-30 实测）。0 和 20 是传感器/驱动的无效哨兵值，原样透传。
module tof_calibrate #(
    parameter integer OFFSET_MM = 0
)(
    input  wire [15:0] din,
    output wire [15:0] dout
);
    wire sentinel = (din == 16'd0) || (din == 16'd20);
    wire [16:0] diff = {1'b0, din} - OFFSET_MM;
    assign dout = sentinel ? din :
                  (diff[16] ? 16'd0 : diff[15:0]);   // 减到负数按 0 处理
endmodule
