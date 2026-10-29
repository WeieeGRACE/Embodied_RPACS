`timescale 1ns/1ps
// 以太网链路桥接层自检 testbench（不需要 PHY/板子）。
//
// 模拟 MAC 侧：每拍 1 字节注入 UDP 负载（= 一个协议帧）；对输出的 tx_start/tx_len
// 做“逐字节取数 -> 发送完成”握手，然后校验帧格式/CRC/长度，并统计 TELEM/ACK。
// 为缩短仿真时间，遥测周期参数化为 2 ms（真实 20 ms），速率判定按比例换算。
module tb_eth_link_bridge;
    localparam integer CLK_HZ = 125_000_000;
    localparam integer TELEM_MS = 2;

    reg clk = 0;
    always #4 clk = ~clk;              // 125 MHz
    reg rst_n = 0;

    integer errors = 0;
    integer checks = 0;

    task automatic check(input bit cond, input string msg);
        checks = checks + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL %s (t=%0t)", msg, $time);
        end else begin
            $display("PASS %s", msg);
        end
    endtask

    // ------------------------------------------------------------ DUT
    reg        rx_valid = 0;
    reg  [7:0] rx_data = 0;
    wire       tx_start;
    wire [15:0] tx_len;
    reg        eth_req = 0;
    reg        eth_done = 0;
    wire [7:0] tx_data;
    wire [7:0] fault_bits;
    wire [15:0] frames_in, frames_out;
    wire        rx_overflow, tx_overflow, link_ok, stop_latched;

    eth_link_bridge #(
        .CLK_HZ(CLK_HZ),
        .TELEM_PERIOD_MS(TELEM_MS),
        .CMD_TIMEOUT_MS(50),
        .HB_TIMEOUT_MS(30)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .rx_valid(rx_valid), .rx_data(rx_data),
        .tx_start(tx_start), .tx_len(tx_len),
        .eth_req(eth_req), .eth_done(eth_done), .tx_data(tx_data),
        .fault_bits(fault_bits), .frames_in(frames_in), .frames_out(frames_out),
        .rx_overflow(rx_overflow), .tx_overflow(tx_overflow),
        .link_ok(link_ok), .stop_latched(stop_latched)
    );

    // ------------------------------------------------------------ CRC
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

    // -------------------------------------------------- 注入一帧（MAC RX 侧）
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

    // ------------------------------------------------- MAC TX 侧状态机（取数）
    localparam [2:0] M_IDLE = 3'd0, M_FETCH = 3'd1, M_WAIT = 3'd2,
                     M_CAP = 3'd5, M_DONE = 3'd3, M_IDLE2 = 3'd4;
    reg [2:0]  mac_state;
    reg [15:0] cur_len;
    reg [7:0]  cap_buf [0:127];
    reg [15:0] k;
    integer    out_frames;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mac_state <= M_IDLE;
            eth_req   <= 1'b0;
            eth_done  <= 1'b0;
            cur_len   <= 16'd0;
            k         <= 16'd0;
            out_frames <= 0;
        end else begin
            case (mac_state)
                M_IDLE: begin
                    eth_req <= 1'b0; eth_done <= 1'b0;
                    if (tx_start) begin
                        cur_len <= tx_len; k <= 16'd0;
                        mac_state <= M_FETCH;
                    end
                end
                M_FETCH: begin eth_req <= 1'b1; mac_state <= M_WAIT; end
                M_WAIT: begin eth_req <= 1'b0; mac_state <= M_CAP; end
                M_CAP: begin
                    cap_buf[k] <= tx_data;
                    if (k + 1 >= cur_len) mac_state <= M_DONE;
                    else begin k <= k + 1'b1; mac_state <= M_FETCH; end
                end
                M_DONE: begin eth_done <= 1'b1; mac_state <= M_IDLE2; end
                M_IDLE2: begin
                    eth_done <= 1'b0;
                    out_frames <= out_frames + 1;
                    mac_state <= M_IDLE;
                end
                default: mac_state <= M_IDLE;
            endcase
        end
    end

    // ------------------------------------------------- 已收帧的校验与统计
    integer seen = 0;
    integer telem_frames = 0;
    integer ack_frames = 0;
    integer bad_frames = 0;
    reg [7:0] last_ack_type = 8'hFF;
    reg [7:0] last_ack_seq  = 8'hFF;
    reg [7:0] last_ack_status = 8'hFF;

    always @(posedge clk) begin
        if (out_frames != seen) begin : analyze
            integer j;
            reg [15:0] c, rx_c;
            reg ok;
            reg [7:0] typ, len;
            seen = out_frames;
            typ = cap_buf[2];
            len = cap_buf[3];
            ok = (cur_len >= 7) && (cap_buf[0] == 8'hAA) && (cap_buf[1] == 8'h55) &&
                 (cap_buf[cur_len-1] == 8'h0D) && (cur_len == len + 7);
            c = 16'hFFFF;
            c = crc16(c, typ);
            c = crc16(c, len);
            for (j = 0; j < len; j = j + 1) c = crc16(c, cap_buf[4+j]);
            rx_c = {cap_buf[cur_len-2], cap_buf[cur_len-3]};
            if (!(ok && (rx_c == c))) begin
                bad_frames = bad_frames + 1;
            end
            if (typ == 8'h81) telem_frames = telem_frames + 1;
            if (typ == 8'h82) begin
                ack_frames = ack_frames + 1;
                last_ack_type   = cap_buf[4];
                last_ack_seq    = cap_buf[5];
                last_ack_status = cap_buf[6];
            end
        end
    end

    // ------------------------------------------------------------ 主流程
    integer t0;
    initial begin
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (20) @(negedge clk);

        // 1) 上电后应自动出 TELEM（2 ms 周期）
        repeat (CLK_HZ / 1000 * 10) @(negedge clk);        // 10 ms
        check(telem_frames >= 3, "telem_after_powerup");
        check(bad_frames == 0, "telem_frames_well_formed");
        check(!link_ok, "powerup_link_lost_until_clear");

        // 2) HEARTBEAT -> link_ok；CLEAR_FAULT 解锁
        inject_frame(8'h03, 1, 64'h05);
        repeat (CLK_HZ / 1000 * 2) @(negedge clk);
        check(link_ok, "link_ok_after_heartbeat");
        inject_frame(8'h05, 0, 64'h0);
        repeat (CLK_HZ / 1000 * 2) @(negedge clk);

        // 3) SET_VEL(120,120,seq=0x37) -> ACK OK + seq 回显
        inject_frame(8'h01, 5, {8'h37, 16'sd120, 16'sd120});
        repeat (CLK_HZ / 1000 * 2) @(negedge clk);
        check(ack_frames >= 1, "ack_received");
        check(last_ack_type == 8'h01 && last_ack_status == 8'h00 && last_ack_seq == 8'h37,
              "ack_setvel_ok_seq_echo");

        // 4) 遥测周期 ≈ 2 ms（20 ms 内 8~12 帧）
        begin : rate_check
            integer n0;
            n0 = telem_frames;
            repeat (CLK_HZ / 1000 * 20) @(negedge clk);    // 20 ms
            check((telem_frames - n0) >= 8 && (telem_frames - n0) <= 12,
                  "telem_period_about_2ms");
        end

        // 5) 诊断计数与无溢出
        check(bad_frames == 0, "all_output_frames_valid");
        check(!rx_overflow && !tx_overflow, "no_overflow");
        check(frames_in >= 3, "frames_in_counted");

        $display("");
        if (errors == 0) $display("PASS tb_eth_link_bridge (%0d checks)", checks);
        else             $display("FAIL tb_eth_link_bridge (%0d errors)", errors);
        $finish;
    end

    initial begin
        #200_000_000;                    // 200 ms 仿真保护
        $display("FAIL tb_eth_link_bridge timeout (out_frames=%0d)", out_frames);
        $finish;
    end
endmodule
