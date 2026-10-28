`timescale 1ns/1ps
// Board top variant: host link + real IMU attitude in the telemetry.
//
// Same UART link and same safety behaviour as uart_bridge_top, plus:
//   imu660ra_driver  -> raw accel/gyro (SPI, 200 Hz)
//   imu_attitude_fixed -> roll/pitch/yaw in millidegrees
//   telemetry carries yaw/pitch/roll in 0.01 deg (TELEM spec)
//
// SAFETY: identical to uart_bridge_top - every motor driver output is tied to 0
// (STBY = 0), so this bitstream can never move the cart.
module uart_bridge_imu_top #(
    parameter integer CLK_HZ             = 50_000_000,
    parameter integer BAUD               = 115200,
    parameter integer TELEM_PERIOD_MS    = 20,
    parameter integer CMD_TIMEOUT_MS     = 500,
    parameter integer HB_TIMEOUT_MS      = 300,
    parameter integer DEFAULT_LIMIT_MM_S = 300,
    parameter integer COUNTS_PER_M       = 1116,
    parameter CONFIG_HEX_FILE = "../../src/imu/imu660ra_config.hex"
)(
    input  wire clk_50m,
    input  wire reset_n,

    input  wire uart_rx,
    output wire uart_tx,

    output wire led_ready,
    output wire led_run,
    output wire led_fault,
    output wire led_heartbeat,

    input  wire encoder_a_l,
    input  wire encoder_b_l,
    input  wire encoder_a_r,
    input  wire encoder_b_r,

    output wire PWMA, AIN1, AIN2,
    output wire PWMB, BIN1, BIN2,
    output wire STBY,

    // IMU (SPI mode 0)
    input  wire imu_spi_miso,
    output wire imu_spi_sck,
    output wire imu_spi_mosi,
    output wire imu_cs_n,
    input  wire imu_int1,
    input  wire imu_int2,
    input  wire imu_gnd_reserved,

    // ToF buses stay released
    input  wire tof_front_gpio1, tof_left_gpio1, tof_right_gpio1,
    inout  wire tof_front_scl, tof_front_sda, tof_front_xshut_n,
    inout  wire tof_left_scl,  tof_left_sda,  tof_left_xshut_n,
    inout  wire tof_right_scl, tof_right_sda, tof_right_xshut_n,

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
            if (enc_a_l_s[2] && !enc_a_l_d[2])
                enc_l_cnt <= enc_b_l_s[2] ? (enc_l_cnt - 32'sd1) : (enc_l_cnt + 32'sd1);
            if (enc_a_r_s[2] && !enc_a_r_d[2])
                enc_r_cnt <= enc_b_r_s[2] ? (enc_r_cnt - 32'sd1) : (enc_r_cnt + 32'sd1);
        end
    end

    // provisional speed scale (integer constant, no divider)
    localparam integer SPEED_K = 10000 / COUNTS_PER_M;
    reg  signed [31:0] enc_l_prev, enc_r_prev;
    reg  signed [15:0] speed_l, speed_r;
    reg  [31:0]        speed_div;
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

    // ------------------------------------------------------------------- IMU
    wire signed [15:0] acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
    wire               imu_sample_valid;
    wire [31:0]        imu_sample_seq;
    wire [7:0]         imu_chip_id, imu_init_status, imu_error_code;
    wire               imu_ready;
    wire               imu_int1_sync, imu_int2_sync;

    imu660ra_driver #(
        .CLK_HZ(CLK_HZ),
        .CONFIG_HEX_FILE(CONFIG_HEX_FILE)
    ) u_imu (
        .clk(clk_50m), .rst_n(rst_n),
        .spi_miso(imu_spi_miso),
        .imu_int1(imu_int1), .imu_int2(imu_int2),
        .spi_sclk(imu_spi_sck), .spi_mosi(imu_spi_mosi), .spi_cs_n(imu_cs_n),
        .accel_x(acc_x), .accel_y(acc_y), .accel_z(acc_z),
        .gyro_x(gyr_x), .gyro_y(gyr_y), .gyro_z(gyr_z),
        .sample_valid(imu_sample_valid), .sample_sequence(imu_sample_seq),
        .chip_id(imu_chip_id), .init_status(imu_init_status),
        .power_conf_readback(), .init_ctrl_readback(),
        .sensor_error(), .ready(imu_ready), .error_code(imu_error_code),
        .int1_sync(imu_int1_sync), .int2_sync(imu_int2_sync)
    );

    wire               att_update, att_valid, att_calibrating;
    wire signed [31:0] roll_mdeg, pitch_mdeg, yaw_mdeg;
    wire signed [15:0] gyro_bias_x, gyro_bias_y, gyro_bias_z;

    imu_attitude_fixed u_att (
        .clk(clk_50m), .rst_n(rst_n),
        .ready_in(imu_ready), .sample_valid(imu_sample_valid),
        .accel_x(acc_x), .accel_y(acc_y), .accel_z(acc_z),
        .gyro_x(gyr_x), .gyro_y(gyr_y), .gyro_z(gyr_z),
        .calibrating(att_calibrating),
        .update_valid(att_update), .attitude_valid(att_valid),
        .roll_mdeg(roll_mdeg), .pitch_mdeg(pitch_mdeg), .yaw_mdeg(yaw_mdeg),
        .gyro_bias_x(gyro_bias_x), .gyro_bias_y(gyro_bias_y), .gyro_bias_z(gyro_bias_z)
    );

    // millidegrees -> 0.01 deg (multiply by 6554 >> 16 ~= /10, no divider)
    function signed [15:0] mdeg_to_cdeg;
        input signed [31:0] m;
        reg signed [31:0] q;
        begin
            q = (m * 32'sd6554) >>> 16;
            if (q > 32'sd32767)       mdeg_to_cdeg = 16'sd32767;
            else if (q < -32'sd32768) mdeg_to_cdeg = -16'sd32768;
            else                      mdeg_to_cdeg = q[15:0];
        end
    endfunction

    reg signed [15:0] yaw_cdeg, pitch_cdeg, roll_cdeg;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            yaw_cdeg   <= 16'sd0;
            pitch_cdeg <= 16'sd0;
            roll_cdeg  <= 16'sd0;
        end else if (att_update) begin
            yaw_cdeg   <= mdeg_to_cdeg(yaw_mdeg);
            pitch_cdeg <= mdeg_to_cdeg(pitch_mdeg);
            roll_cdeg  <= mdeg_to_cdeg(roll_mdeg);
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
        .BYTE_TIMEOUT_CYCLES(CLK_HZ / 200),
        .DEFAULT_LIMIT_MM_S(DEFAULT_LIMIT_MM_S)
    ) u_core (
        .clk(clk_50m), .rst_n(rst_n),
        .rx_byte(rx_byte), .rx_valid(rx_valid),
        .tx_byte(tx_byte), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .enc_l(enc_l_cnt), .enc_r(enc_r_cnt),
        .yaw_cdeg(yaw_cdeg), .pitch_cdeg(pitch_cdeg), .roll_cdeg(roll_cdeg),
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
    // led_ready: on when the link is usable; led_run blinks with the IMU samples;
    // led_fault on any fault; led_heartbeat flashes on UART RX activity.
    reg       rx_led;
    reg [31:0] rx_led_div;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rx_led <= 1'b1;
            rx_led_div <= 32'd0;
        end else if (rx_valid) begin
            rx_led <= 1'b0;
            rx_led_div <= 32'd0;
        end else if (rx_led_div >= 32'd2_000_000) begin
            rx_led <= 1'b1;
        end else begin
            rx_led_div <= rx_led_div + 32'd1;
        end
    end

    reg run_led;
    always @(posedge clk_50m or negedge rst_n)
        if (!rst_n) run_led <= 1'b1;
        else if (att_update) run_led <= ~run_led;

    assign led_ready     = ~(link_ok & ~stop_latched);
    assign led_fault     = ~(|fault_bits);
    assign led_heartbeat = rx_led;
    assign led_run       = run_led;

    // -------------------------------------------------- inert / released pins
    assign {PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY} = 7'b0;
    assign asr_tx = 1'b1;
    assign {tof_front_scl, tof_front_sda, tof_front_xshut_n,
            tof_left_scl,  tof_left_sda,  tof_left_xshut_n,
            tof_right_scl, tof_right_sda, tof_right_xshut_n} = 9'bz;
endmodule
