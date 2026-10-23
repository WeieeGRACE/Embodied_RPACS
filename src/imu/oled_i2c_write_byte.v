`timescale 1ns/1ps
// One SSD1306-compatible I2C write: START, 0x78, control, payload, STOP.
// SDA is open drain; the board's OLED module supplies the pull-up resistors.
module oled_i2c_write_byte #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 250000
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire data_mode,
    input wire [7:0] data_in,
    output reg busy,
    output reg done,
    output reg scl,
    inout wire sda
);
    localparam integer HALF_TICKS = (CLK_HZ / (2 * I2C_HZ) > 0) ?
                                    CLK_HZ / (2 * I2C_HZ) : 1;

    localparam [3:0] ST_IDLE      = 4'd0,
                     ST_START_HI  = 4'd1,
                     ST_START_LOW = 4'd2,
                     ST_BIT_LOW   = 4'd3,
                     ST_BIT_HIGH  = 4'd4,
                     ST_ACK_LOW   = 4'd5,
                     ST_ACK_HIGH  = 4'd6,
                     ST_STOP_LOW  = 4'd7,
                     ST_STOP_HIGH = 4'd8,
                     ST_STOP_END  = 4'd9;

    reg [3:0] state;
    reg [31:0] divider;
    reg [1:0] byte_index;
    reg [2:0] bit_index;
    reg [7:0] shift_byte;
    reg sda_drive_low;
    reg [7:0] payload_latched;
    reg data_mode_latched;

    assign sda = sda_drive_low ? 1'b0 : 1'bz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            divider <= 0;
            byte_index <= 0;
            bit_index <= 0;
            shift_byte <= 0;
            payload_latched <= 0;
            data_mode_latched <= 0;
            sda_drive_low <= 0;
            scl <= 1'b1;
            busy <= 1'b0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (state == ST_IDLE) begin
                scl <= 1'b1;
                sda_drive_low <= 1'b0;
                divider <= 0;
                if (start) begin
                    payload_latched <= data_in;
                    data_mode_latched <= data_mode;
                    busy <= 1'b1;
                    state <= ST_START_HI;
                end
            end else if (divider >= HALF_TICKS - 1) begin
                divider <= 0;
                case (state)
                    ST_START_HI: begin
                        scl <= 1'b1;
                        sda_drive_low <= 1'b0;
                        state <= ST_START_LOW;
                    end
                    ST_START_LOW: begin
                        scl <= 1'b1;
                        sda_drive_low <= 1'b1;
                        byte_index <= 0;
                        bit_index <= 7;
                        shift_byte <= 8'h78;
                        state <= ST_BIT_LOW;
                    end
                    ST_BIT_LOW: begin
                        scl <= 1'b0;
                        sda_drive_low <= ~shift_byte[bit_index];
                        state <= ST_BIT_HIGH;
                    end
                    ST_BIT_HIGH: begin
                        scl <= 1'b1;
                        if (bit_index == 0) begin
                            state <= ST_ACK_LOW;
                        end else begin
                            bit_index <= bit_index - 1'b1;
                            state <= ST_BIT_LOW;
                        end
                    end
                    ST_ACK_LOW: begin
                        scl <= 1'b0;
                        sda_drive_low <= 1'b0;
                        state <= ST_ACK_HIGH;
                    end
                    ST_ACK_HIGH: begin
                        scl <= 1'b1;
                        if (byte_index == 2) begin
                            state <= ST_STOP_LOW;
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            bit_index <= 7;
                            if (byte_index == 0)
                                shift_byte <= data_mode_latched ? 8'h40 : 8'h00;
                            else
                                shift_byte <= payload_latched;
                            state <= ST_BIT_LOW;
                        end
                    end
                    ST_STOP_LOW: begin
                        scl <= 1'b0;
                        sda_drive_low <= 1'b1;
                        state <= ST_STOP_HIGH;
                    end
                    ST_STOP_HIGH: begin
                        scl <= 1'b1;
                        sda_drive_low <= 1'b1;
                        state <= ST_STOP_END;
                    end
                    ST_STOP_END: begin
                        scl <= 1'b1;
                        sda_drive_low <= 1'b0;
                        busy <= 1'b0;
                        done <= 1'b1;
                        state <= ST_IDLE;
                    end
                    default: begin
                        state <= ST_IDLE;
                        scl <= 1'b1;
                        sda_drive_low <= 1'b0;
                        busy <= 1'b0;
                    end
                endcase
            end else begin
                divider <= divider + 1'b1;
            end
        end
    end
endmodule
