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

// Clocked formatter for the main system page. It performs one decimal
// double-dabble iteration per cycle so the OLED data path does not place an
// eleven-stage combinational BCD network between attitude samples and text.
module attitude_angle_to_digits_pipeline(
    input wire clk,
    input wire rst_n,
    input wire signed [31:0] angle_mdeg,
    output reg negative,
    output reg [3:0] hundreds,
    output reg [3:0] tens,
    output reg [3:0] ones,
    output reg [3:0] tenths
);
    wire [17:0] magnitude = angle_mdeg[31]
        ? (~angle_mdeg[17:0] + 18'd1) : angle_mdeg[17:0];
    reg [17:0] magnitude_q;
    reg negative_q;
    wire [17:0] rounded_deci = (magnitude_q + 18'd50) / 18'd100;
    wire [10:0] limited = (rounded_deci > 18'd1800)
        ? 11'd1800 : rounded_deci[10:0];

    reg active;
    reg [3:0] bit_index;
    reg [10:0] binary_work;
    reg [15:0] bcd_work;
    reg negative_work;
    wire [15:0] bcd_next;

    function [15:0] double_dabble_step;
        input [15:0] digits;
        input serial_bit;
        reg [15:0] adjusted;
        begin
            adjusted = digits;
            if (adjusted[3:0] >= 5) adjusted[3:0] = adjusted[3:0] + 4'd3;
            if (adjusted[7:4] >= 5) adjusted[7:4] = adjusted[7:4] + 4'd3;
            if (adjusted[11:8] >= 5) adjusted[11:8] = adjusted[11:8] + 4'd3;
            if (adjusted[15:12] >= 5) adjusted[15:12] = adjusted[15:12] + 4'd3;
            double_dabble_step = {adjusted[14:0], serial_bit};
        end
    endfunction

    assign bcd_next = double_dabble_step(bcd_work, binary_work[10]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            magnitude_q <= 0;
            negative_q <= 0;
            active <= 0;
            bit_index <= 0;
            binary_work <= 0;
            bcd_work <= 0;
            negative_work <= 0;
            negative <= 0;
            hundreds <= 0;
            tens <= 0;
            ones <= 0;
            tenths <= 0;
        end else begin
            magnitude_q <= magnitude;
            negative_q <= angle_mdeg[31];
            if (!active) begin
                binary_work <= limited;
                bcd_work <= 0;
                bit_index <= 10;
                negative_work <= negative_q;
                active <= 1;
            end else begin
                binary_work <= {binary_work[9:0], 1'b0};
                bcd_work <= bcd_next;
                if (bit_index == 0) begin
                    hundreds <= bcd_next[15:12];
                    tens <= bcd_next[11:8];
                    ones <= bcd_next[7:4];
                    tenths <= bcd_next[3:0];
                    negative <= negative_work;
                    active <= 0;
                end else begin
                    bit_index <= bit_index - 1'b1;
                end
            end
        end
    end
endmodule
