`timescale 1ns/1ps
// 端到端系统自检：UDP 负载字节（协议帧） -> eth_link_bridge -> eth_motion_bridge
//                        -> tb6612_driver_interface -> PWMA/AIN1/AIN2/.../STBY
//
// 这是"控制电机移动"的完整数据通路（除了 PHY/RGMII 之外全部覆盖），不需要板子。
//
// 覆盖场景：
//   S1 上电未建链：STBY=0，电机不动
//   S2 心跳 + CLEAR_FAULT：建链、STBY=1
//   S3 SET_VEL(150,150)：前进，占空比 ≈ 150*SCALE，ACK OK
//   S4 SET_VEL(-100,-100)：后退，方向翻转
//   S5 SET_VEL(-150,150)：原地转（左右反向）
//   S6 STOP：占空比 0、STBY=0、ACK OK
//   S7 停止状态下 SET_VEL：ACK REJECT_STOP，电机保持不动
//   S8 CLEAR_FAULT 后恢复驱动
//   S9 心跳停发：心跳超时 -> LINK_LOST -> STBY=0，命令被拒
//   S10 急停键按下：STBY=0
//   S11 解除急停 + 恢复心跳 + CLEAR：重新可驱动
module tb_motion_system;
    localparam integer CLK_HZ    = 1_000_000;          // 1 MHz（仿真加速）
    localparam integer PWM_HZ    = 100;                // -> 周期 10000 拍
    localparam integer PWM_TICKS = CLK_HZ / PWM_HZ;
    localparam integer MAX_V     = 300;
    localparam integer SCALE     = PWM_TICKS / MAX_V;  // 33
    localparam integer TELEM_MS  = 5;
    localparam integer HB_MS     = 50;
    localparam integer CMD_MS    = 100;

    reg clk = 0;
    always #4 clk = ~clk;
    reg rst_n = 0;

    integer errors = 0, checks = 0;
    task automatic check(input bit cond, input string msg);
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s (t=%0t)", msg, $time);
        end else $display("PASS %s", msg);
    endtask

    // ---------------------------------------------------------------- DUT
    reg         rx_valid = 0;
    reg  [7:0]  rx_data = 0;
    wire        tx_start;
    wire [15:0] tx_len;
    reg         eth_req = 0;
    reg         eth_done = 0;
    wire [7:0]  tx_data;
    wire [7:0]  fault_bits;
    wire [15:0] frames_in, frames_out;
    wire        rx_overflow, tx_overflow, link_ok, stop_latched;
    wire signed [15:0] target_l, target_r;

    eth_link_bridge #(
        .CLK_HZ(CLK_HZ), .TELEM_PERIOD_MS(TELEM_MS),
        .CMD_TIMEOUT_MS(CMD_MS), .HB_TIMEOUT_MS(HB_MS),
        .DEFAULT_LIMIT_MM_S(MAX_V)
    ) u_bridge (
        .clk(clk), .rst_n(rst_n),
        .rx_valid(rx_valid), .rx_data(rx_data),
        .tx_start(tx_start), .tx_len(tx_len),
        .eth_req(eth_req), .eth_done(eth_done), .tx_data(tx_data),
        .fault_bits(fault_bits), .frames_in(frames_in), .frames_out(frames_out),
        .rx_overflow(rx_overflow), .tx_overflow(tx_overflow),
        .link_ok(link_ok), .stop_latched(stop_latched),
        .target_v_left(target_l), .target_v_right(target_r)
    );

    reg key_estop_n = 1;
    wire signed [15:0] pwm_l, pwm_r;
    wire motors_enabled;

    eth_motion_bridge #(
        .MAX_V_MM_S(MAX_V), .PWM_TICKS(PWM_TICKS), .SCALE_NUM(SCALE)
    ) u_motion (
        .clk(clk), .rst_n(rst_n),
        .target_l(target_l), .target_r(target_r),
        .fault_bits(fault_bits), .link_ok(link_ok), .stop_latched(stop_latched),
        .key_estop_n(key_estop_n),
        .pwm_l(pwm_l), .pwm_r(pwm_r), .motors_enabled(motors_enabled),
        .v_limited_l(), .v_limited_r()
    );

    wire PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY;
    tb6612_driver_interface #(
        .CLK_HZ(CLK_HZ), .PWM_HZ(PWM_HZ), .DIR_DEADTIME_CYCLES(50)
    ) u_drv (
        .clk(clk), .rst_n(rst_n),
        .enable(motors_enabled),
        .emergency_stop(!key_estop_n),
        .fault(fault_bits != 8'h00),
        .watchdog_timeout(!link_ok),
        .pwm_signed_l(pwm_l), .pwm_signed_r(pwm_r),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2),
        .PWMB(PWMB), .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY)
    );

    // ------------------------------------------------------------- CRC / 注入
    function automatic [15:0] crc16(input [15:0] c, input [7:0] d);
        reg [15:0] x;
        integer i;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc16 = x;
        end
    endfunction

    task automatic inject_frame(input [7:0] t, input integer n, input [63:0] head);
        integer i;
        reg [15:0] c;
        reg [7:0] b;
        begin
            c = 16'hFFFF;
            @(negedge clk);
            rx_valid = 1; rx_data = 8'hAA; @(negedge clk);
            rx_data = 8'h55; @(negedge clk);
            rx_data = t;  c = crc16(c, t); @(negedge clk);
            rx_data = n[7:0]; c = crc16(c, n[7:0]); @(negedge clk);
            for (i = 0; i < n; i = i + 1) begin
                b = head[i*8 +: 8];
                c = crc16(c, b);
                rx_data = b; @(negedge clk);
            end
            rx_data = c[7:0];  @(negedge clk);
            rx_data = c[15:8]; @(negedge clk);
            rx_data = 8'h0D;   @(negedge clk);
            rx_valid = 0; rx_data = 8'h00;
            @(negedge clk);
        end
    endtask

    task automatic send_heartbeat(input [7:0] seq);
        begin inject_frame(8'h03, 1, {56'd0, seq}); end
    endtask

    task automatic send_clear;
        begin inject_frame(8'h05, 0, 64'd0); end
    endtask

    task automatic send_stop;
        begin inject_frame(8'h02, 0, 64'd0); end
    endtask

    task automatic send_setvel(input integer l, input integer r, input [7:0] seq);
        begin
            inject_frame(8'h01, 5, {8'd0, seq, r[15:0], l[15:0]});
        end
    endtask

    // ------------------------------------------------------- MAC 侧收帧模型
    localparam [2:0] M_IDLE = 3'd0, M_FETCH = 3'd1, M_WAIT = 3'd2,
                     M_CAP = 3'd5, M_DONE = 3'd3, M_IDLE2 = 3'd4;
    reg [2:0]  mac_state;
    reg [15:0] cur_len;
    reg [7:0]  cap [0:127];
    reg [15:0] k;
    integer telem_cnt, ack_cnt;
    reg [7:0] last_ack_type, last_ack_seq, last_ack_status;
    integer bad_frames;
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mac_state <= M_IDLE; eth_req <= 0; eth_done <= 0;
            cur_len <= 0; k <= 0; telem_cnt <= 0; ack_cnt <= 0;
            bad_frames <= 0; last_ack_type <= 8'hFF; last_ack_seq <= 8'hFF;
            last_ack_status <= 8'hFF;
        end else begin
            case (mac_state)
                M_IDLE: begin
                    eth_req <= 0; eth_done <= 0;
                    if (tx_start) begin cur_len <= tx_len; k <= 0; mac_state <= M_FETCH; end
                end
                M_FETCH: begin eth_req <= 1; mac_state <= M_WAIT; end
                M_WAIT:  begin eth_req <= 0; mac_state <= M_CAP; end
                M_CAP:   begin
                    cap[k] <= tx_data;
                    if (k + 1 >= cur_len) mac_state <= M_DONE;
                    else begin k <= k + 1; mac_state <= M_FETCH; end
                end
                M_DONE:  begin eth_done <= 1; mac_state <= M_IDLE2; end
                M_IDLE2: begin
                    eth_done <= 0;
                    // 校验帧格式（AA 55 / 尾 0D / 长度自洽）
                    if (cap[0] !== 8'hAA || cap[1] !== 8'h55 || cap[cur_len-1] !== 8'h0D ||
                        cur_len !== cap[3] + 7) bad_frames <= bad_frames + 1;
                    if (cap[2] == 8'h81) telem_cnt <= telem_cnt + 1;
                    if (cap[2] == 8'h82) begin
                        ack_cnt <= ack_cnt + 1;
                        last_ack_type   <= cap[4];
                        last_ack_seq    <= cap[5];
                        last_ack_status <= cap[6];
                    end
                    mac_state <= M_IDLE;
                end
                default: mac_state <= M_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------- 占空比/周期测量
    task automatic count_span(output integer dla, output integer dlb);
        integer j;
        begin
            dla = 0; dlb = 0;
            for (j = 0; j < PWM_TICKS; j = j + 1) begin
                @(negedge clk);
                if (PWMA) dla = dla + 1;
                if (PWMB) dlb = dlb + 1;
            end
        end
    endtask

    task automatic measure_duty(output integer dla, output integer dlb);
        begin
            // 对齐到高电平起点再数一个周期（无 PWM 时退化为直接数）
            if (PWMA) begin
                while (PWMA) @(negedge clk);
            end
            count_span(dla, dlb);
        end
    endtask

    task automatic settle(input integer n);
        begin repeat (n) @(negedge clk); end
    endtask

    task automatic cruise(input integer l, input integer r, input [7:0] seq0, input integer rounds);
        integer q;
        begin
            for (q = 0; q < rounds; q = q + 1) begin
                send_heartbeat(seq0 + q[7:0]);
                send_setvel(l, r, seq0 + q[7:0]);
                settle(PWM_TICKS * 2);    // 20 ms 一轮：心跳 <50 ms、命令 <100 ms
            end
        end
    endtask

    integer da, db, expect_duty;

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        settle(50);

        // ---------------- S1 上电：未建链
        settle(CLK_HZ / 1000 * (HB_MS + 10));      // 超过心跳超时
        check(!link_ok, "S1_powerup_no_link");
        check(STBY == 1'b0, "S1_powerup_stby_off");

        // ---------------- S2 心跳 + CLEAR -> 建链
        send_heartbeat(8'd1);
        settle(CLK_HZ / 1000 * 2);
        check(link_ok, "S2_link_ok_after_heartbeat");
        send_clear();
        settle(CLK_HZ / 1000 * 2);
        check(fault_bits == 8'h00 || fault_bits == 8'h20,   // 允许 CMD_TIMEOUT（还没发命令）
              $sformatf("S2_fault_clear (0x%02X)", fault_bits));
        settle(10);
        check(STBY == 1'b1 || fault_bits == 8'h20, "S2_motors_ready_or_cmd_timeout");

        // ---------------- S3 前进 150
        cruise(150, 150, 8'd10, 4);
        send_heartbeat(8'd20);
        settle(PWM_TICKS);
        measure_duty(da, db);
        expect_duty = 150 * SCALE;
        check(STBY == 1'b1, "S3_forward_motors_enabled");
        check(AIN1 && !AIN2 && BIN1 && !BIN2, "S3_forward_direction");
        check(da <= expect_duty + 3 && da >= expect_duty - 3,
              $sformatf("S3_forward_duty_l (%0d vs %0d)", da, expect_duty));
        check(last_ack_type == 8'h01 && last_ack_status == 8'h00, "S3_setvel_ack_ok");

        // ---------------- S4 后退 100
        cruise(-100, -100, 8'd40, 4);
        send_heartbeat(8'd45);
        settle(PWM_TICKS);
        measure_duty(da, db);
        expect_duty = 100 * SCALE;
        check(!AIN1 && AIN2 && !BIN1 && BIN2, "S4_reverse_direction");
        check(da <= expect_duty + 3 && da >= expect_duty - 3,
              $sformatf("S4_reverse_duty_l (%0d vs %0d)", da, expect_duty));

        // ---------------- S5 原地转
        cruise(-150, 150, 8'd60, 3);
        send_heartbeat(8'd70);
        settle(PWM_TICKS);
        measure_duty(da, db);
        check(!AIN1 && AIN2, "S5_rotate_left_reverse");
        check(BIN1 && !BIN2, "S5_rotate_right_forward");

        // ---------------- S6 STOP
        send_heartbeat(8'd80);
        send_clear();                    // 先清 CMD_TIMEOUT，保证 STOP 被接受
        settle(CLK_HZ / 1000 * 2);
        send_stop();
        settle(CLK_HZ / 1000 * 2);
        check(last_ack_type == 8'h02 && last_ack_status == 8'h00, "S6_stop_ack_ok");
        check(STBY == 1'b0, "S6_stop_stby_off");
        count_span(da, db);
        check(da <= 1 && db <= 1, "S6_stop_duty_zero");

        // ---------------- S7 停止中 SET_VEL 被拒
        send_setvel(120, 120, 8'd90);
        settle(CLK_HZ / 1000 * 3);
        check(last_ack_type == 8'h01 && last_ack_status == 8'h02,
              $sformatf("S7_setvel_rejected_when_stopped (status=0x%02X)", last_ack_status));
        check(STBY == 1'b0, "S7_still_stopped");

        // ---------------- S8 CLEAR 后恢复
        send_clear();
        settle(CLK_HZ / 1000 * 2);
        cruise(100, 100, 8'd100, 3);
        settle(PWM_TICKS);
        check(STBY == 1'b1, "S8_resume_after_clear");
        measure_duty(da, db);
        check(da > 10, "S8_pwm_present_after_resume");

        // ---------------- S9 心跳停发 -> LINK_LOST
        settle(CLK_HZ / 1000 * (HB_MS + 20));
        check(!link_ok, "S9_link_lost_after_no_heartbeat");
        check(STBY == 1'b0, "S9_stby_off_on_link_lost");
        send_setvel(100, 100, 8'd120);
        settle(CLK_HZ / 1000 * 3);
        check(last_ack_status == 8'h01,
              $sformatf("S9_setvel_rejected_link_lost (status=0x%02X)", last_ack_status));

        // ---------------- S10 急停键
        send_heartbeat(8'd2);
        settle(CLK_HZ / 1000 * 2);
        send_clear();
        settle(CLK_HZ / 1000 * 2);
        cruise(100, 100, 8'd130, 2);
        key_estop_n = 0;
        settle(20);
        check(STBY == 1'b0 && !PWMA, "S10_estop_key_stops");

        // ---------------- S11 恢复
        key_estop_n = 1;
        send_heartbeat(8'd3);
        settle(CLK_HZ / 1000 * 2);
        send_clear();
        settle(CLK_HZ / 1000 * 2);
        cruise(100, 100, 8'd150, 2);
        settle(PWM_TICKS);
        check(STBY == 1'b1, "S11_recovered_after_estop_release");
        check(bad_frames == 0, "T1_output_frames_well_formed");
        check(telem_cnt > 0, "T2_telemetry_seen");

        $display("");
        $display("telem=%0d ack=%0d bad_frames=%0d", telem_cnt, ack_cnt, bad_frames);
        if (errors == 0) $display("PASS tb_eth_motion_system (%0d checks)", checks);
        else             $display("FAIL tb_eth_motion_system (%0d errors, %0d checks)", errors, checks);
        $finish;
    end

    initial begin
        #500_000_000;    // 500 ms 仿真保护
        $display("FAIL tb_eth_motion_system timeout");
        $finish;
    end
endmodule
