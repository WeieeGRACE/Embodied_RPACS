`timescale 1ns/1ps
// 调试文本输出：周期性输出 "D <16 位十六进制>\r\n"（主工程 event_logger 的 D 行格式）。
// 用途：验证无线串口（WCH-Link，P6-21 = L12）通路；本模块不涉及任何电机输出。
//
// 64 位内容（字段名借用主工程解码器）：
//   [63:56] fault_bits   [55:48] hb_age 低 8 位   [47:40] cmd_seq
//   [39:32] 收帧数低 8 位 [31:24] 发帧数低 8 位    [23:0] 0
module debug_text_logger #(
    parameter integer CLK_HZ    = 125_000_000,
    parameter integer BAUD      = 115200,
    parameter integer PERIOD_MS = 100
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  fault_bits,
    input  wire [7:0]  hb_age_lsb,
    input  wire [7:0]  cmd_seq,
    input  wire [7:0]  frames_in_lsb,
    input  wire [7:0]  frames_out_lsb,
    output reg         uart_tx
);
    localparam integer BIT_CYCLES = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer MS_CYCLES  = CLK_HZ / 1000;
    localparam integer FRAME_LEN  = 18;      // 'D' ' ' 16hex '\r' '\n'

    // ---------------------------------------------------------- 周期触发
    reg [31:0] ms_div;
    reg [15:0] ms_cnt;
    reg        snapshot_req;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_div <= 32'd0; ms_cnt <= 16'd0; snapshot_req <= 1'b0;
        end else begin
            snapshot_req <= 1'b0;
            if (ms_div >= MS_CYCLES - 1) begin
                ms_div <= 32'd0;
                if (ms_cnt >= PERIOD_MS - 1) begin
                    ms_cnt <= 16'd0;
                    snapshot_req <= 1'b1;
                end else begin
                    ms_cnt <= ms_cnt + 16'd1;
                end
            end else begin
                ms_div <= ms_div + 32'd1;
            end
        end
    end

    // ------------------------------------------------------ 快照（打一拍）
    reg [63:0] value_r;
    always @(posedge clk) begin
        if (snapshot_req)
            value_r <= {fault_bits, hb_age_lsb, cmd_seq,
                        frames_in_lsb, frames_out_lsb, 24'd0};
    end

    // ---------------------------------------------------------- 串口发送 8N1
    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;
    reg [9:0]  shreg;
    reg        tx_busy;
    reg        send;
    reg [7:0]  cur_byte;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 16'd0; bit_idx <= 4'd0; shreg <= 10'h3FF;
            tx_busy <= 1'b0; uart_tx <= 1'b1;
        end else if (!tx_busy) begin
            uart_tx <= 1'b1;
            if (send) begin
                shreg    <= {1'b1, cur_byte, 1'b0};
                tx_busy  <= 1'b1;
                baud_cnt <= 16'd0;
                bit_idx  <= 4'd0;
            end
        end else if (baud_cnt >= BIT_CYCLES - 1) begin
            baud_cnt <= 16'd0;
            uart_tx  <= shreg[0];
            shreg    <= {1'b1, shreg[9:1]};
            if (bit_idx == 4'd9) tx_busy <= 1'b0;
            else                 bit_idx <= bit_idx + 4'd1;
        end else begin
            baud_cnt <= baud_cnt + 16'd1;
        end
    end

    // -------------------------------------------------------- 字符序列状态机
    localparam [1:0] S_IDLE = 2'd0, S_PREP = 2'd1, S_SEND = 2'd2, S_WAIT = 2'd3;
    reg [1:0]  state;
    reg [4:0]  ch_idx;
    reg [63:0] value_s;

    function [7:0] hex_digit(input [3:0] n);
        hex_digit = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n});
    endfunction

    wire [3:0] nib = (ch_idx >= 5'd2) ? value_s[4*(5'd17 - ch_idx) +: 4] : 4'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; ch_idx <= 5'd0; value_s <= 64'd0;
            cur_byte <= 8'd0; send <= 1'b0;
        end else begin
            send <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (snapshot_req) begin
                        value_s <= value_r;
                        ch_idx  <= 5'd0;
                        state   <= S_PREP;
                    end
                end
                S_PREP: begin
                    if (ch_idx == 5'd0)       cur_byte <= 8'h44;   // 'D'
                    else if (ch_idx == 5'd1)  cur_byte <= 8'h20;   // ' '
                    else if (ch_idx == 5'd18) cur_byte <= 8'h0D;   // '\r'
                    else if (ch_idx == 5'd19) cur_byte <= 8'h0A;   // '\n'
                    else                      cur_byte <= hex_digit(nib);
                    state <= S_SEND;
                end
                S_SEND: begin
                    if (!tx_busy) begin
                        send  <= 1'b1;
                        state <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (!tx_busy) begin
                        if (ch_idx >= 5'd19) state <= S_IDLE;
                        else begin
                            ch_idx <= ch_idx + 5'd1;
                            state  <= S_PREP;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
