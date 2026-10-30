`timescale 1ns/1ps
// SPI mode 0, MSB first. One byte is accepted while ready is high.
module imu660ra_spi_master #(
    parameter integer HALF_PERIOD = 25
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] tx_data,
    output wire ready,
    output reg busy,
    output reg done,
    output reg [7:0] rx_data,
    input wire spi_miso,
    output reg spi_sclk,
    output reg spi_mosi
);
    reg [31:0] half_count;
    reg [2:0] bit_index;
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    localparam integer HALF_LAST = (HALF_PERIOD > 0) ? HALF_PERIOD - 1 : 0;

    assign ready = !busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            half_count <= 0;
            bit_index <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            rx_data <= 0;
            busy <= 0;
            done <= 0;
            spi_sclk <= 0;
            spi_mosi <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                spi_sclk <= 0;
                if (start) begin
                    busy <= 1;
                    half_count <= 0;
                    bit_index <= 7;
                    tx_shift <= tx_data;
                    rx_shift <= 0;
                    spi_mosi <= tx_data[7];
                end
            end else if (half_count == HALF_LAST) begin
                half_count <= 0;
                if (!spi_sclk) begin
                    // Mode 0 samples MISO on the rising edge.
                    spi_sclk <= 1;
                    rx_shift <= {rx_shift[6:0], spi_miso};
                end else begin
                    spi_sclk <= 0;
                    if (bit_index == 0) begin
                        busy <= 0;
                        done <= 1;
                        // The final MISO bit was sampled on the preceding
                        // rising edge and is already present in rx_shift.
                        rx_data <= rx_shift;
                        spi_mosi <= 0;
                    end else begin
                        bit_index <= bit_index - 1'b1;
                        spi_mosi <= tx_shift[bit_index - 1'b1];
                    end
                end
            end else begin
                half_count <= half_count + 1'b1;
            end
        end
    end
endmodule
