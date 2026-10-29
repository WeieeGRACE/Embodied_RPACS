`timescale 1ns/1ps
// Ethernet 链路的「UDP 负载 <-> 协议帧」桥接层（与 PHY/RGMII 无关，可独立仿真）。
//
// 一个 UDP 报文 = 一个完整协议帧（docs/PROTOCOL_v1.md §7 的 v2 约定）：
//   rx_*  ：收到的 UDP 负载字节流 -> 降速(每 2 拍 1 字节) -> host_frame_rx -> 命令解码
//   tx_*  ：telem_builder/ACK 组帧 -> 整帧缓存 -> 一次 UDP 发送（tx_start + tx_len）
//
// 与 MAC 侧握手：本模块发 tx_start(脉冲)+tx_len 后，由 eth_udp_tx_gmii 逐字节
// 用 eth_req 取数，发完给 eth_done 脉冲，本模块才允许下一帧。
module eth_link_bridge #(
    parameter integer CLK_HZ             = 125_000_000,
    parameter integer TELEM_PERIOD_MS    = 20,
    parameter integer CMD_TIMEOUT_MS     = 500,
    parameter integer HB_TIMEOUT_MS      = 300,
    parameter integer DEFAULT_LIMIT_MM_S = 300,
    parameter integer MAX_LEN            = 32,
    parameter integer RXFIFO_AW          = 9,           // 512 字节接收缓冲
    parameter integer TB_AW              = 7            // 128 字节整帧缓存
)(
    input  wire        clk,
    input  wire        rst_n,
    // ---- 来自 eth_udp_rx_gmii 的 UDP 负载流 ----
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    // ---- 送往 eth_udp_tx_gmii 的一帧 ----
    output reg         tx_start,        // 单拍脉冲：开始发一帧
    output reg  [15:0] tx_len,          // 与 tx_start 同拍有效
    input  wire        eth_req,         // 上游取字节脉冲（payload_req_o）
    input  wire        eth_done,        // 上游发送完成脉冲（tx_done）
    output reg  [7:0]  tx_data,
    // ---- 状态/诊断 ----
    output wire [7:0]  fault_bits,
    output wire [15:0] frames_in,       // 收到的完整协议帧数
    output wire [15:0] frames_out,      // 发出的协议帧数
    output wire        rx_overflow,
    output wire        tx_overflow,
    output wire        link_ok,
    output wire        stop_latched,
    // SET_VEL 解出的目标轮速（mm/s，已按 SET_LIMIT 限幅）——给电机控制用
    output wire signed [15:0] target_v_left,
    output wire signed [15:0] target_v_right,
    // 诊断输出（给调试文本日志用）
    output wire [15:0] hb_age_ms_out,
    output wire [7:0]  cmd_seq_out,
    output wire [15:0] telem_sent_cnt_out
);
    // ------------------------------------------------------------ 接收 FIFO
    localparam integer RF_DEPTH = 1 << RXFIFO_AW;
    reg  [7:0]         rf_mem [0:RF_DEPTH-1];
    reg  [RXFIFO_AW:0] rf_wr, rf_rd;
    wire               rf_empty = (rf_wr == rf_rd);
    wire               rf_full  = ((rf_wr[RXFIFO_AW] != rf_rd[RXFIFO_AW]) &&
                                   (rf_wr[RXFIFO_AW-1:0] == rf_rd[RXFIFO_AW-1:0]));
    reg                rx_of;
    // 注意：写指针必须复位（否则上电为 X，rf_empty 判定失效，RTL 仿真里一个字节都进不来）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rf_wr <= {(RXFIFO_AW+1){1'b0}};
            rx_of <= 1'b0;
        end else if (rx_valid && !rf_full) begin
            rf_mem[rf_wr[RXFIFO_AW-1:0]] <= rx_data;
            rf_wr <= rf_wr + 1'b1;
        end else if (rx_valid && rf_full) begin
            rx_of <= 1'b1;
        end
    end

    // 降速到「每 2 拍 1 字节」，满足 host_frame_rx 的字节间隙要求
    reg        gap;
    reg        rx_byte_v;
    reg  [7:0] rx_byte_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rf_rd     <= {(RXFIFO_AW+1){1'b0}};
            gap       <= 1'b0;
            rx_byte_v <= 1'b0;
            rx_byte_d <= 8'd0;
        end else begin
            rx_byte_v <= 1'b0;
            if (!rf_empty && !gap) begin
                rx_byte_d <= rf_mem[rf_rd[RXFIFO_AW-1:0]];
                rx_byte_v <= 1'b1;
                rf_rd     <= rf_rd + 1'b1;
                gap       <= 1'b1;
            end else begin
                gap <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------ 链路核心
    wire [7:0]  tx_byte;
    wire        tx_valid, tx_ready, tx_done;
    wire        setvel_strobe;
    wire [15:0] good_cnt, crc_cnt, len_cnt, tail_cnt, sync_cnt, timeout_cnt;
    wire [15:0] unsup_cnt, telem_cnt, ack_cnt, ack_drop;
    wire [7:0]  hb_seq, cmd_seq;
    wire [15:0] hb_age;
    wire signed [15:0] tgt_l, tgt_r;

    reg [15:0] good_prev;
    reg [15:0] frames_in_r;
    wire       good_new = (good_cnt != good_prev);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            good_prev   <= 16'd0;
            frames_in_r <= 16'd0;
        end else begin
            good_prev <= good_cnt;
            if (good_new) frames_in_r <= frames_in_r + 16'd1;
        end
    end
    assign frames_in = frames_in_r;

    uart_bridge_core #(
        .CLK_HZ(CLK_HZ),
        .TELEM_PERIOD_MS(TELEM_PERIOD_MS),
        .CMD_TIMEOUT_MS(CMD_TIMEOUT_MS),
        .HB_TIMEOUT_MS(HB_TIMEOUT_MS),
        .BYTE_TIMEOUT_CYCLES(CLK_HZ / 200),        // 5 ms 字节间超时
        .DEFAULT_LIMIT_MM_S(DEFAULT_LIMIT_MM_S),
        .MAX_LEN(MAX_LEN)
    ) u_core (
        .clk(clk), .rst_n(rst_n),
        .rx_byte(rx_byte_d), .rx_valid(rx_byte_v),
        .tx_byte(tx_byte), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .enc_l(32'sd0), .enc_r(32'sd0),
        .yaw_cdeg(16'sd0), .pitch_cdeg(16'sd0), .roll_cdeg(16'sd0),
        .speed_l_mm_s(16'sd0), .speed_r_mm_s(16'sd0),
        .ext_fault(8'h00),
        .target_v_left(tgt_l), .target_v_right(tgt_r),
        .setvel_strobe_out(setvel_strobe),
        .tx_done_out(tx_done),
        .stop_latched_out(stop_latched),
        .link_ok_out(link_ok), .link_lost_out(),
        .fault_out(fault_bits),
        .good_cnt(good_cnt), .crc_err_cnt(crc_cnt), .len_err_cnt(len_cnt),
        .tail_err_cnt(tail_cnt), .sync_err_cnt(sync_cnt), .timeout_cnt(timeout_cnt),
        .unsupported_cnt(unsup_cnt), .telem_sent_cnt(telem_cnt),
        .ack_sent_cnt(ack_cnt), .ack_drop_cnt(ack_drop),
        .hb_seq_out(hb_seq), .cmd_seq_out(cmd_seq), .hb_age_ms_out(hb_age)
    );

    // ---------------- 发送缓冲（TAX：核心写、ETH 侧读）----------------
    reg  [7:0]     tb_mem [0:(1<<TB_AW)-1];
    reg  [TB_AW:0] tb_wr;                   // 写指针（最高位=溢出标志）
    wire [TB_AW:0] tb_wr_next = tb_wr + ((tx_valid && tx_ready) ? 1'b1 : 1'b0);
    reg  [TB_AW:0] tb_rd;
    reg            sending;                 // 一帧已交给 MAC
    reg            assembling;              // 正在收一帧
    reg  [15:0]    tx_out_cnt;
    reg            tx_of;

    // 核心只有在“没有正在发、且缓冲没溢出”时才允许吐字节
    assign tx_ready = !sending && !tb_wr[TB_AW];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tb_wr         <= {(TB_AW+1){1'b0}};
            tb_rd         <= {(TB_AW+1){1'b0}};
            sending       <= 1'b0;
            assembling    <= 1'b0;
            tx_start      <= 1'b0;
            tx_len        <= 16'd0;
            tx_data       <= 8'd0;
            tx_out_cnt    <= 16'd0;
            tx_of         <= 1'b0;
        end else begin
            tx_start <= 1'b0;

            // 1) 核心吐字节：先写缓冲
            if (tx_valid && tx_ready) begin
                if (!tb_wr[TB_AW]) begin
                    tb_mem[tb_wr[TB_AW-1:0]] <= tx_byte;
                    tb_wr <= tb_wr + 1'b1;
                end
                assembling <= 1'b1;
            end

            // 2) 帧结束：交给 MAC
            if (tx_done && !sending) begin
                if (tb_wr_next[TB_AW]) begin
                    tx_of <= 1'b1;                       // 超长帧，丢弃
                end else if (tb_wr_next != {(TB_AW+1){1'b0}}) begin
                    tx_len      <= {8'd0, tb_wr_next[TB_AW-1:0]};
                    tx_start    <= 1'b1;
                    tb_rd       <= {(TB_AW+1){1'b0}};
                    sending     <= 1'b1;
                    tx_out_cnt  <= tx_out_cnt + 16'd1;
                end
                tb_wr      <= {(TB_AW+1){1'b0}};
                assembling <= 1'b0;
            end

            // 3) MAC 取字节
            if (sending && eth_req) begin
                tx_data <= tb_mem[tb_rd[TB_AW-1:0]];
                tb_rd   <= tb_rd + 1'b1;
            end

            // 4) MAC 发完
            if (sending && eth_done) begin
                sending <= 1'b0;
                tb_rd   <= {(TB_AW+1){1'b0}};
            end
        end
    end

    assign hb_age_ms_out     = hb_age;
    assign cmd_seq_out       = cmd_seq;
    assign telem_sent_cnt_out = telem_cnt;
    assign target_v_left  = tgt_l;
    assign target_v_right = tgt_r;
    assign frames_out  = tx_out_cnt;
    assign rx_overflow = rx_of;
    assign tx_overflow = tx_of;
endmodule
