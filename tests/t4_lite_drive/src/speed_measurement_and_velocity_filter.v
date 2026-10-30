`timescale 1ns/1ps
// 有符号编码器增量测速 + 滑动平均滤波（t4_lite_drive 本地副本）。
//
// 来源：主工程 src/motor/speed_measurement_and_velocity_filter.v（50 MHz 版本）。
// 模块名/端口名/参数名/数值行为不变；为 125 MHz 适配，把「滑动平均更新」
// （history_sum 加减 + 除法 + 饱和）从原单拍拆成两级流水：
//   第 1 拍：next_sum = history_sum ± oldest + window_total（进寄存器），
//            同时更新样本窗口；
//   第 2 拍：speed_filtered = sat32(next_sum / FILTER_LENGTH)（除常数在综合时折叠）。
// 因此 speed_filtered/history_sum 比原版晚 1 个 clk 更新（20 ms 窗口，无影响），
// speed_raw/direction/zero_speed/speed_valid 仍在窗口边界同拍更新。
module speed_measurement_and_velocity_filter #(
    parameter integer CLK_HZ = 50000000,
    parameter integer WINDOW_CYCLES = 500000,
    parameter integer FILTER_LENGTH = 4,
    parameter integer ZERO_PAD_WARMUP = 0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire signed [15:0] delta_count,
    output reg  signed [31:0] speed_raw,
    output reg  signed [31:0] speed_filtered,
    output reg  signed [31:0] position,
    output reg  speed_valid,
    output reg  direction,
    output reg  zero_speed
);
    function integer clog2_min1;
        input integer value;
        integer n;
        begin
            n = 0;
            while ((1 << n) < value) n = n + 1;
            if (n < 1) clog2_min1 = 1; else clog2_min1 = n;
        end
    endfunction
    function signed [31:0] sat32;
        input signed [63:0] value;
        begin
            if (value > 64'sh000000007fffffff) sat32 = 32'sh7fffffff;
            else if (value < -64'sh0000000080000000) sat32 = 32'sh80000000;
            else sat32 = value[31:0];
        end
    endfunction

    localparam integer WCW = clog2_min1(WINDOW_CYCLES);
    localparam integer ACW = clog2_min1(FILTER_LENGTH + 1);
    // Enough for FILTER_LENGTH signed samples plus an intermediate replacement.
    localparam integer SUM_WIDTH = 33 + clog2_min1(FILTER_LENGTH);
    // Constant-divisor branches avoid a large general 64-bit divider during
    // filter warm-up. Signed division keeps truncation toward zero.
    function signed [63:0] warmup_average;
        input signed [SUM_WIDTH-1:0] total;
        input [ACW-1:0] count;
        integer divisor;
        begin
            warmup_average = total / FILTER_LENGTH;
            for (divisor = 1; divisor <= FILTER_LENGTH; divisor = divisor + 1)
                if (count == divisor-1) warmup_average = total / divisor;
        end
    endfunction
    reg [WCW-1:0] window_counter;
    reg signed [31:0] window_sum;
    // Capture the completed window first, then run the history/division
    // network on the following clock. This keeps the encoder delta adder out
    // of the long filter path while preserving one result per window.
    reg signed [31:0] window_total_q;
    reg window_total_valid;
    reg signed [SUM_WIDTH-1:0] history_sum;
    reg signed [31:0] samples [0:FILTER_LENGTH-1];
    reg [ACW-1:0] sample_count;
    reg [ACW-1:0] sample_index;
    wire signed [31:0] oldest_sample;
    generate if (WINDOW_CYCLES > 1) begin: prefetch_history
        reg signed [31:0] oldest;
        always @(posedge clk or negedge rst_n)
            if (!rst_n) oldest <= 0;
            else oldest <= samples[sample_index];
        assign oldest_sample = oldest;
    end else begin: immediate_history
        assign oldest_sample = samples[sample_index];
    end endgenerate
    integer i;
    reg signed [SUM_WIDTH-1:0] next_sum;
    reg signed [63:0] avg_calc;

    // ------------------------------------------------------------------
    // 125 MHz 适配新增：滑动平均更新的第二级流水寄存器
    // ------------------------------------------------------------------
    reg signed [SUM_WIDTH-1:0] next_sum_q;
    reg signed [31:0]          total_q;
    reg [ACW-1:0]              count_q;
    reg                        avg_valid;

    // synthesis translate_off
    initial begin
        if (CLK_HZ < 1 || WINDOW_CYCLES < 1 || FILTER_LENGTH < 1) begin
            $display("ERROR: invalid speed filter parameters");
            $fatal(1);
        end
    end
    // synthesis translate_on

    // 第 1 级：窗口有效 -> 更新样本 + 计算 next_sum（纯寄存器输入）
    always @* begin
        next_sum = history_sum;
        if (sample_count == FILTER_LENGTH) begin
            next_sum = next_sum - oldest_sample + window_total_q;
        end else begin
            next_sum = next_sum + window_total_q;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_counter <= 0;
            window_sum <= 0;
            window_total_q <= 0;
            window_total_valid <= 1'b0;
            history_sum <= 0;
            speed_raw <= 0;
            speed_filtered <= 0;
            position <= 0;
            speed_valid <= 1'b0;
            direction <= 1'b0;
            zero_speed <= 1'b1;
            sample_count <= 0;
            sample_index <= 0;
            next_sum_q <= 0;
            total_q <= 0;
            count_q <= 0;
            avg_valid <= 1'b0;
            for (i = 0; i < FILTER_LENGTH; i = i + 1) samples[i] <= 0;
        end else begin
            speed_valid <= 1'b0;
            avg_valid <= 1'b0;
            position <= sat32($signed(position) + $signed(delta_count));
            if (window_total_valid) begin
                // 输出（窗口边界同拍更新，与原版一致）
                speed_raw <= window_total_q;
                direction <= (window_total_q < 0);
                zero_speed <= (window_total_q == 0);
                speed_valid <= 1'b1;
                // 第一级流水：next_sum / 窗口总量 / 样本数 进寄存器
                next_sum_q <= next_sum;
                total_q <= window_total_q;
                count_q <= sample_count;
                avg_valid <= 1'b1;
                // 更新样本历史（与原版同拍）
                samples[sample_index] <= window_total_q;
                if (sample_index == FILTER_LENGTH-1) sample_index <= 0;
                else sample_index <= sample_index + 1'b1;
                if (sample_count < FILTER_LENGTH) sample_count <= sample_count + 1'b1;
                window_total_valid <= 1'b0;
            end else if (avg_valid) begin
                // 第二级流水：滑动平均与饱和（除常数在综合时折叠）
                if (ZERO_PAD_WARMUP) begin
                    avg_calc = next_sum_q / FILTER_LENGTH;
                end else if (count_q < FILTER_LENGTH) begin
                    avg_calc = warmup_average(next_sum_q, count_q);
                end else begin
                    if (next_sum_q < 0) avg_calc = -((-next_sum_q) / FILTER_LENGTH);
                    else avg_calc = next_sum_q / FILTER_LENGTH;
                end
                speed_filtered <= sat32(avg_calc);
                history_sum <= next_sum_q;
            end
            if (window_counter == WINDOW_CYCLES-1) begin
                window_counter <= 0;
                window_total_q <= window_sum + delta_count;
                window_total_valid <= 1'b1;
                window_sum <= 0;
            end else begin
                window_counter <= window_counter + 1'b1;
                window_sum <= window_sum + delta_count;
            end
        end
    end
endmodule
