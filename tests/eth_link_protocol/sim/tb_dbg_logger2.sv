`timescale 1ns/1ps
// debug_text_logger 自检（直接观察内部寄存，避免 UART 解码误差）：
//   1) 快照寄存 value_r 的字段排列
//   2) send 脉冲时 cur_byte 的字符序列是否为 "D <16hex>\r\n"
module tb_debug_text_logger;
    localparam integer CLK_HZ    = 125_000_000;
    localparam integer BAUD      = 115200;
    localparam integer PERIOD_MS = 5;

    reg clk = 0;
    always #4 clk = ~clk;              // 125 MHz
    reg rst_n = 0;

    reg [7:0] fault_bits = 8'h22;
    reg [7:0] hb_age_lsb = 8'h10;
    reg [7:0] cmd_seq    = 8'h05;
    reg [7:0] frames_in  = 8'h01;
    reg [7:0] frames_out = 8'h02;
    wire      uart_tx;

    debug_text_logger #(
        .CLK_HZ(CLK_HZ), .BAUD(BAUD), .PERIOD_MS(PERIOD_MS)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .fault_bits(fault_bits), .hb_age_lsb(hb_age_lsb),
        .cmd_seq(cmd_seq), .frames_in_lsb(frames_in),
        .frames_out_lsb(frames_out), .uart_tx(uart_tx)
    );

    reg [7:0] chs [0:63];
    integer   nchs = 0;
    reg [63:0] snap = 0;
    integer   errors = 0;
    integer   i;

    always @(posedge clk) begin
        if (rst_n && dut.send && nchs < 64) begin
            chs[nchs] = dut.cur_byte;
            if (nchs < 6)
                $display("[dbg] send#%0d ch_idx=%0d nib=%h cur=%h val_s=%h",
                         nchs, dut.ch_idx, dut.nib, dut.cur_byte, dut.value_s);
            nchs = nchs + 1;
        end
        if (rst_n && dut.snapshot_req) snap = dut.value_s;
    end

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        #20_000_000;                    // 20 ms（PERIOD_MS=5，需等几行）
        $display("snapshot value = 0x%016X", snap);
        if (snap !== 64'h2210_0501_0200_0000) begin
            $display("FAIL: 快照字段排列 != 期望 0x2210050102000000");
            errors = errors + 1;
        end
        $write("chars(%0d): ", nchs);
        for (i = 0; i < nchs && i < 20; i = i + 1) $write("%02X ", chs[i]);
        $write("\n");
        if (nchs < 18) begin
            $display("FAIL: 字符数不足 (%0d < 18)", nchs);
            errors = errors + 1;
        end
        if (chs[0] !== 8'h44 || chs[1] !== 8'h20) begin
            $display("FAIL: 前缀不是 D+空格");
            errors = errors + 1;
        end
        if (chs[2] !== 8'h32 || chs[3] !== 8'h32) begin
            $display("FAIL: fault_bits 打印不对 (期望 22)");
            errors = errors + 1;
        end
        if (chs[4] !== 8'h31 || chs[5] !== 8'h30) begin
            $display("FAIL: hb_age 打印不对 (期望 10)");
            errors = errors + 1;
        end
        if (chs[6] !== 8'h30 || chs[7] !== 8'h35) begin
            $display("FAIL: cmd_seq 打印不对 (期望 05)");
            errors = errors + 1;
        end
        if (chs[8] !== 8'h30 || chs[9] !== 8'h31) begin
            $display("FAIL: frames_in 打印不对 (期望 01)");
            errors = errors + 1;
        end
        if (chs[10] !== 8'h30 || chs[11] !== 8'h32) begin
            $display("FAIL: frames_out 打印不对 (期望 02)");
            errors = errors + 1;
        end
        if (chs[16] !== 8'h30 || chs[17] !== 8'h30) begin
            $display("FAIL: 末尾补零不对");
            errors = errors + 1;
        end
        if (nchs >= 20 && (chs[18] !== 8'h0D || chs[19] !== 8'h0A)) begin
            $display("FAIL: 行尾不是 CRLF");
            errors = errors + 1;
        end
        if (errors == 0) $display("PASS tb_debug_text_logger");
        else             $display("FAIL tb_debug_text_logger (%0d errors)", errors);
        $finish;
    end
endmodule
