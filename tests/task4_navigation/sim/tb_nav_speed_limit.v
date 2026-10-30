`timescale 1ns/1ps
// nav_speed_limit 单元自检：无论输入多大，输出必须钳在 ±LIMIT 内。
module tb_nav_speed_limit;
    localparam integer LIMIT = 60;

    reg  signed [31:0] in_l, in_r;
    wire signed [31:0] out_l, out_r;
    wire clamped_l, clamped_r;

    nav_speed_limit #(.LIMIT_MM_S(LIMIT)) dut (
        .velocity_l_mm_s(in_l), .velocity_r_mm_s(in_r),
        .limited_l_mm_s(out_l), .limited_r_mm_s(out_r),
        .clamped_l(clamped_l), .clamped_r(clamped_r)
    );

    integer errors = 0, checks = 0;

    task automatic chk(input integer il, input integer ir,
                          input integer el, input integer er,
                          input bit ecl, input bit ecr, input string msg);
        begin
            in_l = il; in_r = ir;
            #1;
            checks = checks + 1;
            if (out_l !== el || out_r !== er || clamped_l !== ecl || clamped_r !== ecr) begin
                errors = errors + 1;
                $display("FAIL %s: in=(%0d,%0d) out=(%0d,%0d) clamped=(%b,%b) 期望=(%0d,%0d,%b,%b)",
                         msg, il, ir, out_l, out_r, clamped_l, clamped_r, el, er, ecl, ecr);
            end else begin
                $display("PASS %s", msg);
            end
        end
    endtask

    initial begin
        chk(0, 0, 0, 0, 0, 0, "zero_passthrough");
        chk(42, -42, 42, -42, 0, 0, "inside_limit_passthrough");
        chk(60, -60, 60, -60, 0, 0, "exactly_limit_no_clamp");
        chk(61, -61, 60, -60, 1, 1, "just_over_limit_clamped");
        chk(500, -500, 60, -60, 1, 1, "absurd_speed_clamped");
        chk(100, 30, 60, 30, 1, 0, "only_one_wheel_clamped");
        chk(2147483647, -2147483648, 60, -60, 1, 1, "int32_extremes_clamped");
        $display("");
        if (errors == 0) $display("PASS tb_nav_speed_limit (%0d checks)", checks);
        else             $display("FAIL tb_nav_speed_limit (%0d errors)", errors);
        $finish;
    end
endmodule
