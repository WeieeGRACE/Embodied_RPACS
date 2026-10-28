`timescale 1ns/1ps
module tb_attitude_pipeline;
    reg clk = 0;
    reg rst_n = 0;
    reg signed [31:0] angle_mdeg = 0;
    wire negative;
    wire [3:0] hundreds, tens, ones, tenths;
    reg completed = 0;

    always #10 clk = ~clk;

    attitude_angle_to_digits_pipeline dut (
        .clk(clk), .rst_n(rst_n), .angle_mdeg(angle_mdeg),
        .negative(negative), .hundreds(hundreds), .tens(tens),
        .ones(ones), .tenths(tenths), .result_valid()
    );

    task check_value;
        input signed [31:0] value;
        input expected_negative;
        input [3:0] expected_hundreds, expected_tens, expected_ones, expected_tenths;
        begin
            @(negedge clk) angle_mdeg = value;
            repeat (24) @(posedge clk);
            #1;
            if (negative !== expected_negative || hundreds !== expected_hundreds ||
                tens !== expected_tens || ones !== expected_ones || tenths !== expected_tenths)
                $fatal(1, "angle=%0d got %b %0d%0d%0d.%0d expected %b %0d%0d%0d.%0d",
                    value, negative, hundreds, tens, ones, tenths,
                    expected_negative, expected_hundreds, expected_tens, expected_ones, expected_tenths);
            $display("PASS pipeline angle %0d -> %b %0d%0d%0d.%0d", value,
                negative, hundreds, tens, ones, tenths);
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        check_value(32'sd12345, 1'b0, 4'd0, 4'd1, 4'd2, 4'd3);
        check_value(-32'sd12345, 1'b1, 4'd0, 4'd1, 4'd2, 4'd3);
        check_value(32'sd12450, 1'b0, 4'd0, 4'd1, 4'd2, 4'd5);
        check_value(32'sd180000, 1'b0, 4'd1, 4'd8, 4'd0, 4'd0);
        completed = 1;
        $display("PASS attitude display pipeline rounding, sign and limits");
        $finish;
    end
endmodule
