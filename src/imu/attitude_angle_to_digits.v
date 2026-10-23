`timescale 1ns/1ps
// Convert signed millidegrees to a sign and four decimal digits (DDD.d).
module attitude_angle_to_digits(
    input wire signed [31:0] angle_mdeg,
    output wire negative,
    output reg [3:0] hundreds,
    output reg [3:0] tens,
    output reg [3:0] ones,
    output reg [3:0] tenths
);
    // Attitude outputs are wrapped to +/-180000 millidegrees.  Keeping the
    // formatter at that range avoids synthesizing several 32-bit dividers.
    wire [17:0] angle_low = angle_mdeg[17:0];
    wire [17:0] magnitude = angle_mdeg[31] ? (~angle_low + 18'd1) : angle_low;
    wire [17:0] rounded_deci = (magnitude + 18'd50) / 18'd100;
    wire [10:0] limited = (rounded_deci > 18'd1800) ? 11'd1800 : rounded_deci[10:0];
    wire [15:0] bcd = binary_to_bcd(limited);

    assign negative = angle_mdeg[31];

    always @* begin
        hundreds = bcd[15:12];
        tens = bcd[11:8];
        ones = bcd[7:4];
        tenths = bcd[3:0];
    end

    function [15:0] binary_to_bcd;
        input [10:0] value;
        reg [15:0] digits;
        integer bit_index;
        begin
            digits = 0;
            for (bit_index = 10; bit_index >= 0; bit_index = bit_index - 1) begin
                if (digits[3:0] >= 5) digits[3:0] = digits[3:0] + 4'd3;
                if (digits[7:4] >= 5) digits[7:4] = digits[7:4] + 4'd3;
                if (digits[11:8] >= 5) digits[11:8] = digits[11:8] + 4'd3;
                if (digits[15:12] >= 5) digits[15:12] = digits[15:12] + 4'd3;
                digits = {digits[14:0], value[bit_index]};
            end
            binary_to_bcd = digits;
        end
    endfunction
endmodule
