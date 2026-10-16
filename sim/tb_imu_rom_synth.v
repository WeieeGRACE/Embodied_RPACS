`timescale 1ns/1ps
module tb_imu_rom_synth;
    GSR GSR(.GSRI(1'b1));
    reg clk=0;
    reg [12:0] address=0;
    wire [7:0] data;
    reg [7:0] expected [0:8191];
    integer i;
    always #10 clk=~clk;
    imu660ra_config_rom dut(.clk_50m_d(clk),.config_index(address),.config_rom_byte(data));
    initial begin
        $readmemh("src/imu/imu660ra_config.hex",expected);
        for(i=0;i<8192;i=i+1) begin
            @(negedge clk); address=i;
            @(posedge clk); #1;
            if(data !== expected[i]) $fatal(1,"synth ROM mismatch address=%0d actual=%h expected=%h",i,data,expected[i]);
        end
        $display("PASS synthesized ROM: all 8192 bytes match HEX");
        $finish;
    end
endmodule
