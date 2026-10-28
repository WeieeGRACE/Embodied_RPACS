`timescale 1ns/1ps
// Board top for the host link (UART bridge) standalone test project.
//
// SAFETY: this bitstream never drives the motors. All motor-driver outputs are tied
// to 0 (STBY = 0 keeps the TB6612 in standby) and the ToF/OLED buses are released.
// Only the UART link, the LEDs and the encoder inputs are active, so the board can be
// re-flashed with the real firmware at any time without any motion risk.
module uart_bridge_top #(
    parameter integer CLK_HZ             = 50_000_000,
    parameter integer BAUD               = 115200,
    parameter integer TELEM_PERIOD_MS    = 20,
    parameter integer CMD_TIMEOUT_MS     = 500,
    parameter integer HB_TIMEOUT_MS      = 300,   // 1.5x the 200 ms heartbeat period
    parameter integer DEFAULT_LIMIT_MM_S = 300,
    // encoder counts per metre (x1 decoding); only used for the telemetry speed field
    parameter integer COUNTS_PER_M       = 1116
)(
    input  wire clk_50m,
    input  wire reset_n,

    // host link (USB serial, CH9102 -> COM14)
    input  wire uart_rx,
    output wire uart_tx,

    // status LEDs
    output wire led_ready,
    output wire led_run,
    output wire led_fault,
    output wire led_heartbeat,

    // wheel encoders (read only)
    input  wire encoder_a_l,
    input  wire encoder_b_l,
    input  wire encoder_a_r,
    input  wire encoder_b_r,

    // motor driver (kept inert: STBY = 0 -> outputs disabled)
    output wire PWMA, AIN1, AIN2,
    output wire PWMB, BIN1, BIN2,
    output wire STBY,

    // IMU (inputs only, unused in this test project)
    input  wire imu_gnd_reserved,
    input  wire imu_spi_sck,
    input  wire imu_spi_mosi,
    input  wire imu_spi_miso,
    input  wire imu_cs_n,
    input  wire imu_int1,
    input  wire imu_int2,

    // ToF buses (released, inputs only)
    input  wire tof_front_gpio1, tof_left_gpio1, tof_right_gpio1,
    inout  wire tof_front_scl, tof_front_sda, tof_front_xshut_n,
    inout  wire tof_left_scl,  tof_left_sda,  tof_left_xshut_n,
    inout  wire tof_right_scl, tof_right_sda, tof_right_xshut_n,

    // ASRPRO voice link (kept idle high so the module sees no traffic)
    input  wire asr_rx,
    output wire asr_tx
);
    // ------------------------------------------------------------ clock/reset
    (* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync;
    reg [15:0] por_cnt;
    reg        por_done;

    always @(posedge clk_50m) begin
        reset_sync <= {reset_sync[1:0], reset_n};
        if (!reset_sync[2]) begin
            por_cnt  <= 16'd0;
            por_done <= 1'b0;
        end else if (por_cnt != 16'hFFFF) begin
            por_cnt  <= por_cnt + 16'd1;
            por_done <= 1'b0;
        end else begin
            por_done <= 1'b1;
        end
    end

    wire rst_n = reset_sync[2] & por_done;

    // ------------------------------------------------------------------ UART
    wire [7:0] rx_byte;
    wire       rx_valid, rx_frame_err;

    uart_rx_8n1 #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart_rx (
        .clk(clk_50m), .rst_n(rst_n), .rx(uart_rx),
        .data(rx_byte), .data_valid(rx_valid), .frame_error(rx_frame_err)
    );

    wire [7:0] tx_byte;
    wire       tx_valid, tx_ready, tx_done;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart_tx (
        .clk(clk_50m), .rst_n(rst_n), .data(tx_byte), .valid(tx_valid),
        .ready(tx_ready), .tx(uart_tx), .done(tx_done)
    );

    // -------------------------------------------------------------- encoders
    (* ASYNC_REG = "TRUE" *) reg [2:0] enc_a_l_s, enc_b_l_s, enc_a_r_s, enc_b_r_s;
    always @(posedge clk_50m) begin
        enc_a_l_s <= {enc_a_l_s[1:0], encoder_a_l};
        enc_b_l_s <= {enc_b_l_s[1:0], encoder_b_l};
        enc_a_r_s <= {enc_a_r_s[1:0], encoder_a_r};
        enc_b_r_s <= {enc_b_r_s[1:0], encoder_b_r};
    end

    reg  signed [31:0] enc_l_cnt, enc_r_cnt;
    reg  [2:0]         enc_a_l_d, enc_a_r_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            enc_l_cnt <= 32'sd0;
            enc_r_cnt <= 32'sd0;
            enc_a_l_d <= 3'b000;
            enc_a_r_d <= 3'b000;
        end else begin
            enc_a_l_d <= enc_a_l_s;
            enc_a_r_d <= enc_a_r_s;
            // x1 decoding: count on the rising edge of channel A, direction from B
            if (enc_a_l_s[2] && !enc_a_l_d[2])
                enc_l_cnt <= enc_b_l_s[2] ? (enc_l_cnt - 32'sd1) : (enc_l_cnt + 32'sd1);
            if (enc_a_r_s[2] && !enc_a_r_d[2])
                enc_r_cnt <= enc_b_r_s[2] ? (enc_r_cnt - 32'sd1) : (enc_r_cnt + 32'sd1);
        end
    end

    // speed estimate over 100 ms (uncalibrated, provisional scale)
    // speed_mm_s = delta_counts_per_100ms * 10 * 1000 / COUNTS_PER_M
    localparam integer SPEED_K = 10000 / COUNTS_PER_M;   // integer constant, no divider

    reg  signed [31:0] enc_l_prev, enc_r_prev;
    reg  signed [15:0] speed_l, speed_r;
    reg  [31:0] speed_div;
    wire        speed_tick = (speed_div >= ((CLK_HZ / 1000) * 100 - 1));

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            speed_div  <= 32'd0;
            enc_l_prev <= 32'sd0;
            enc_r_prev <= 32'sd0;
            speed_l    <= 16'sd0;
            speed_r    <= 16'sd0;
        end else if (speed_tick) begin
            speed_div  <= 32'd0;
            enc_l_prev <= enc_l_cnt;
            enc_r_prev <= enc_r_cnt;
            speed_l <= ((enc_l_cnt - enc_l_prev) * SPEED_K);
            speed_r <= ((enc_r_cnt - enc_r_prev) * SPEED_K);
        end else begin
            speed_div <= speed_div + 32'd1;
        end
    end

    // ------------------------------------------------------------- link core
    wire signed [15:0] target_l, target_r;
    wire        stop_latched, link_ok, link_lost;
    wire [7:0]  fault_bits;
    wire [15:0] good_cnt, crc_err_cnt, len_err_cnt, tail_err_cnt, sync_err_cnt;
    wire [15:0] timeout_cnt, unsupported_cnt, telem_cnt, ack_cnt, ack_drop;
    wire [7:0]  hb_seq, cmd_seq;
    wire [15:0] hb_age;

    uart_bridge_core #(
        .CLK_HZ(CLK_HZ),
        .TELEM_PERIOD_MS(TELEM_PERIOD_MS),
        .CMD_TIMEOUT_MS(CMD_TIMEOUT_MS),
        .HB_TIMEOUT_MS(HB_TIMEOUT_MS),
        .BYTE_TIMEOUT_CYCLES(CLK_HZ / 200),      // 5 ms
        .DEFAULT_LIMIT_MM_S(DEFAULT_LIMIT_MM_S)
    ) u_core (
        .clk(clk_50m), .rst_n(rst_n),
        .rx_byte(rx_byte), .rx_valid(rx_valid),
        .tx_byte(tx_byte), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .enc_l(enc_l_cnt), .enc_r(enc_r_cnt),
        .yaw_cdeg(16'sd0), .pitch_cdeg(16'sd0), .roll_cdeg(16'sd0),
        .speed_l_mm_s(speed_l), .speed_r_mm_s(speed_r),
        .ext_fault(8'h00),
        .target_v_left(target_l), .target_v_right(target_r),
        .stop_latched_out(stop_latched),
        .link_ok_out(link_ok), .link_lost_out(link_lost),
        .fault_out(fault_bits),
        .good_cnt(good_cnt), .crc_err_cnt(crc_err_cnt), .len_err_cnt(len_err_cnt),
        .tail_err_cnt(tail_err_cnt), .sync_err_cnt(sync_err_cnt),
        .timeout_cnt(timeout_cnt), .unsupported_cnt(unsupported_cnt),
        .telem_sent_cnt(telem_cnt), .ack_sent_cnt(ack_cnt),
        .ack_drop_cnt(ack_drop),
        .hb_seq_out(hb_seq), .cmd_seq_out(cmd_seq), .hb_age_ms_out(hb_age)
    );

    // --------------------------------------------------------------- status
    // LED semantics (low-active LEDs on the board: main project drives 0 = on)
    reg       rx_led;
    reg [31:0] rx_led_div;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rx_led <= 1'b1;
            rx_led_div <= 32'd0;
        end else if (rx_valid) begin
            rx_led <= 1'b0;                       // flash on every received byte
            rx_led_div <= 32'd0;
        end else if (rx_led_div >= 32'd2_000_000) begin
            rx_led <= 1'b1;
        end else begin
            rx_led_div <= rx_led_div + 32'd1;
        end
    end

    reg        telem_led;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) telem_led <= 1'b1;
        else if (telem_cnt[0]) telem_led <= ~telem_led;
    end

    assign led_ready     = ~(link_ok & ~stop_latched);    // on when the link is usable
    assign led_fault     = ~(|fault_bits);                // on when any fault bit is set
    assign led_heartbeat = rx_led;                        // flash on RX activity
    assign led_run       = telem_led;                     // 25 Hz-ish blink

    // -------------------------------------------------- inert / released pins
    // SAFETY: motors stay in standby, nothing is driven.
    assign {PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY} = 7'b0;
    assign asr_tx = 1'b1;                                  // idle high
    assign {tof_front_scl, tof_front_sda, tof_front_xshut_n,
            tof_left_scl,  tof_left_sda,  tof_left_xshut_n,
            tof_right_scl, tof_right_sda, tof_right_xshut_n} = 9'bz;

    // NOTE: the remaining diagnostics (good/crc/len/tail/sync/timeout counters,
    // hb_seq/cmd_seq, target speeds) are intentionally left unconnected here.
    // They are what the future motor-side integration will consume; in this test
    // project they stay internal so the bitstream can never drive the wheels.
endmodule
