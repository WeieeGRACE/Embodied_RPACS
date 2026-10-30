`timescale 1ns/1ps
// Unit test for t4_div10_sat (the iterative /10 + int16 saturation used to turn
// imu_attitude_fixed yaw_mdeg into telemetry yaw_cdeg in the 50 MHz domain).
// Checks that the result equals sat16(din / 10) with truncation toward zero,
// i.e. exactly the semantics of the original "yaw_cdeg <= yaw_cdeg_full" logic.
module tb_t4_div10_sat;
    reg               clk = 1'b0;
    reg               rst_n = 1'b0;
    reg               start = 1'b0;
    reg signed [31:0] din = 32'sd0;
    wire              busy, done;
    wire signed [15:0] cdeg;

    integer errors = 0;
    integer checks = 0;

    t4_div10_sat dut (
        .clk(clk), .rst_n(rst_n), .start(start), .din(din),
        .busy(busy), .done(done), .cdeg(cdeg)
    );

    always #10 clk = ~clk;   // 50 MHz

    function signed [15:0] expected;
        input signed [31:0] v;
        reg signed [31:0] q;
        begin
            q = v / 32'sd10;   // Verilog signed division truncates toward zero
            if (q > 32'sd32767)       expected = 16'sh7fff;
            else if (q < -32'sd32768) expected = -16'sd32768;
            else                      expected = q[15:0];
        end
    endfunction

    task run_case;
        input signed [31:0] v;
        reg signed [15:0] exp;
        integer timeout;
        begin
            exp = expected(v);
            @(negedge clk);
            din   = v;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            timeout = 0;
            while (!done && timeout < 200) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            checks = checks + 1;
            if (!done) begin
                $display("FAIL: no done for %0d", v);
                errors = errors + 1;
            end else if (cdeg !== exp) begin
                $display("FAIL: din=%0d cdeg=%0d expected=%0d", v, cdeg, exp);
                errors = errors + 1;
            end else begin
                $display("ok: din=%0d cdeg=%0d", v, cdeg);
            end
            // let done fall before next start
            @(negedge clk);
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        run_case(32'sd0);
        run_case(32'sd1);
        run_case(32'sd9);
        run_case(32'sd10);
        run_case(32'sd11);
        run_case(32'sd12345);
        run_case(32'sd32767);
        run_case(32'sd32768);
        run_case(32'sd32769);
        run_case(32'sd180000);
        run_case(32'sd327674);
        run_case(32'sd999999);
        run_case(-32'sd1);
        run_case(-32'sd9);
        run_case(-32'sd10);
        run_case(-32'sd11);
        run_case(-32'sd12345);
        run_case(-32'sd180000);
        run_case(-32'sd327670);
        run_case(-32'sd327671);
        run_case(-32'sd327680);
        run_case(-32'sd999999);
        run_case(32'sh7fffffff);
        run_case(-32'sh80000000);

        if (errors == 0)
            $display("PASS tb_t4_div10_sat (%0d checks)", checks);
        else
            $display("FAIL tb_t4_div10_sat (%0d errors / %0d checks)", errors, checks);
        $finish;
    end
endmodule
