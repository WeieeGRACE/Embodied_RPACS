`timescale 1ns/1ps
// Stub-based testbench for host_link_motion_adapter.
//
// The real sequencer lives in the main project, so this TB models its *command
// contract* (cmd_valid / cmd_ready / one latched command) and verifies that the
// adapter produces the right mode / direction / magnitude for each request, and
// that a latched stop only produces one BRAKE and blocks further commands.
module tb_motion_adapter;
    reg clk = 0;
    always #10 clk = ~clk;

    reg rst_n = 0;
    integer errors = 0;

    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s (t=%0t)", msg, $time);
        end else begin
            $display("PASS %s", msg);
        end
    endtask

    // ------------------------------------------------------------- adapter DUT
    reg signed [15:0] req_l = 0, req_r = 0;
    reg               setvel = 0;
    reg               stop_latched = 0;
    reg               link_ok = 0;
    reg               link_lost = 1;
    wire              cmd_valid;
    wire [3:0]        cmd_mode;
    wire [1:0]        cmd_end;
    wire              cmd_reverse;
    wire [31:0]       cmd_speed_l, cmd_speed_r, cmd_goal, cmd_timeout_ms;
    wire [31:0]       cmd_speed_limit, cmd_accel_step, cmd_decel_step;
    wire [15:0]       cmds_issued, cmds_rejected, diff_approx;
    reg               cmd_ready = 1;
    integer           busy_cnt = 0;

    host_link_motion_adapter #(
        .USE_VELOCITY_MODE(0),
        .SPEED_LIMIT_MM_S(300),
        .CMD_TIMEOUT_MS(3000)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .req_v_left(req_l), .req_v_right(req_r),
        .setvel_strobe(setvel), .stop_latched(stop_latched),
        .link_ok(link_ok), .link_lost(link_lost),
        .cmd_valid(cmd_valid), .cmd_mode(cmd_mode), .cmd_end(cmd_end),
        .cmd_reverse(cmd_reverse), .cmd_speed_l(cmd_speed_l),
        .cmd_speed_r(cmd_speed_r), .cmd_goal(cmd_goal),
        .cmd_timeout_ms(cmd_timeout_ms), .cmd_speed_limit(cmd_speed_limit),
        .cmd_accel_step(cmd_accel_step), .cmd_decel_step(cmd_decel_step),
        .cmd_ready(cmd_ready),
        .cmds_issued(cmds_issued), .cmds_rejected(cmds_rejected),
        .differential_approx_cnt(diff_approx)
    );

    // ------------------------------------------------------- sequencer stub
    reg [3:0]  cap_mode;
    reg        cap_reverse;
    reg [31:0] cap_speed_l, cap_speed_r;
    integer    cap_count = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_ready <= 1'b1;
            busy_cnt  <= 0;
            cap_mode  <= 4'd0;
            cap_reverse <= 1'b0;
            cap_speed_l <= 32'd0;
            cap_speed_r <= 32'd0;
            cap_count <= 0;
        end else begin
            if (cmd_valid && cmd_ready) begin
                cap_mode    <= cmd_mode;
                cap_reverse <= cmd_reverse;
                cap_speed_l <= cmd_speed_l;
                cap_speed_r <= cmd_speed_r;
                cap_count   <= cap_count + 1;
                $display("      [stub] captured mode=%0d rev=%b spd=(%0d,%0d)",
                         cmd_mode, cmd_reverse, cmd_speed_l, cmd_speed_r);
                busy_cnt    <= 4;          // emulate the sequencer latching
            end
            if (busy_cnt > 0) begin
                cmd_ready <= 1'b0;
                busy_cnt  <= busy_cnt - 1;
            end else begin
                cmd_ready <= 1'b1;
            end
        end
    end

    task automatic request(input integer l, input integer r);        @(negedge clk);
        req_l = l[15:0];
        req_r = r[15:0];
        setvel = 1'b1;
        @(negedge clk);
        setvel = 1'b0;
        repeat (60) @(negedge clk);        // let the sequencer latch
    endtask

    initial begin
        repeat (10) @(negedge clk);
        rst_n = 1;
        link_ok = 1'b1;
        link_lost = 1'b0;
        repeat (10) @(negedge clk);

        // 1) straight forward
        request(100, 100);
        check(cap_mode == 4'd2, "fwd_mode_is_FORWARD");
        check(cap_reverse == 1'b0, "fwd_direction");
        check(cap_speed_l == 32'd100000, "fwd_speed_l_um_s");

        // 2) straight backward
        request(-80, -80);
        check(cap_mode == 4'd3, "rev_mode_is_BACKWARD");
        check(cap_reverse == 1'b1, "rev_direction");
        check(cap_speed_l == 32'd80000, "rev_speed_um_s");

        // 3) in-place rotation (left positive, right negative)
        request(100, -100);
        check(cap_mode == 4'd6, "rotate_mode");
        check(cap_reverse == 1'b0, "rotate_ccw");
        check(cap_speed_l == 32'd100000, "rotate_speed");

        // 4) rotation the other way
        request(-70, 70);
        check(cap_mode == 4'd6, "rotate_mode_cw");
        check(cap_reverse == 1'b1, "rotate_cw_direction");

        // 5) differential -> legacy approximation uses the dominant component
        request(120, 60);
        check(cap_mode == 4'd2, "diff_approx_uses_forward");
        check(cap_speed_l == 32'd90000, "diff_approx_speed_is_average_90");
        check(diff_approx >= 16'd1, "diff_approx_counter");

        // 6) stop latch -> exactly one BRAKE
        cap_count = 0;
        @(negedge clk); stop_latched = 1'b1;
        repeat (200) @(negedge clk);
        check(cap_count == 1, "stop_issues_exactly_one_command");
        check(cap_mode == 4'd8, "stop_mode_is_BRAKE");

        // 7) while stopped, speed requests must not reach the sequencer
        cap_count = 0;
        req_l = 100; req_r = 100; setvel = 1'b1;
        @(negedge clk); setvel = 1'b0;
        repeat (120) @(negedge clk);
        check(cap_count == 0, "no_command_while_stopped");

        // 8) release the latch, new command is accepted again
        @(negedge clk); stop_latched = 1'b0;
        repeat (30) @(negedge clk);
        request(50, 50);
        check(cap_mode == 4'd2 && cap_speed_l == 32'd50000, "resume_after_stop_clear");

        // 9) zero speed -> BRAKE
        cap_count = 0;
        request(0, 0);
        check(cap_count == 1 && cap_mode == 4'd8, "zero_speed_brakes");

        $display("");
        if (errors == 0) $display("PASS tb_motion_adapter");
        else             $display("FAIL tb_motion_adapter (%0d errors)", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("FAIL tb_motion_adapter timeout");
        $finish;
    end
endmodule
