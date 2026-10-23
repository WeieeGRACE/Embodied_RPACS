`timescale 1ns/1ps
// SeekFree IMU660RA initialization image converted byte-for-byte from the
// project's GPL-3.0 zf_device_imu660ra.c configuration array.
module imu660ra_config_rom #(
    parameter INIT_FILE = "src/imu/imu660ra_config.hex"
)(
    input wire clk,
    input wire [12:0] address,
    output reg [7:0] data
);
    reg [7:0] rom [0:8191];
    initial begin
        $readmemh(INIT_FILE, rom);
    end
    always @(posedge clk) begin
        data <= rom[address];
    end
endmodule
