`timescale 1ns/1ps
// tests/t4_lite_drive —— 「按编码器里程走一段 + ToF 兜底停车」精简实验工程顶层。
//
// 组成（全部在 125 MHz clk125m 单时钟域内）：
//   * 以太网 UDP 链路 v2：SET_VEL / SET_LIMIT / STOP / CLEAR_FAULT / HEARTBEAT，
//     20 ms 遥测 —— 与 tests/eth_motion_test 完全一致，协议无任何改动；
//   * 双轮 AB 相正交编码器 + asr_distance 里程（65 mm 轮径：UM_PER_REV=204203,
//     COUNTS_PER_REV=1456），遥测 enc_l/enc_r 字段回传「左右轮里程，单位 mm」；
//   * 前向 VL53L0X（软 I2C，约 60 ms 一次）：读数减 60 mm 实测偏置后 <=120 mm
//     即锁存 tof_stop，切断电机（只做停车边界，不参与任何导航/避障）；
//   * TB6612 电机输出（复用主工程 tb6612_driver_interface，含方向死区）。
//
// 安全（全部硬连线，不依赖上位机）：
//   * 心跳 300 ms 超时 / 任一链路故障位 / STOP 锁存 / CMD 500 ms 超时
//                                          -> motors_enabled=0 -> STBY=0
//   * KEY0(key_enable_n) 未使能 / KEY2(key_estop_n) 急停 / tof_stop
//                                          -> motors_enabled=0 + emergency_stop
//   * 复位未释放                          -> inhibit
module t4_lite_top #(
    parameter [47:0] LOCAL_MAC  = 48'h00_0a_35_01_fe_c0,
    parameter [31:0] LOCAL_IP   = 32'hc0_a8_00_02,      // 192.168.0.2
    parameter [15:0] LOCAL_PORT = 16'd5000,
    parameter [31:0] HOST_IP    = 32'hc0_a8_00_01,      // 192.168.0.1
    parameter [15:0] HOST_PORT  = 16'd5001,
    parameter [47:0] HOST_MAC   = 48'hff_ff_ff_ff_ff_ff,
    parameter integer TELEM_PERIOD_MS = 20,
    parameter integer MAX_V_MM_S = 300,
    parameter integer PWM_HZ     = 20000,
    // 65 mm 轮径标定（与 tests/task4_navigation 一致）
    parameter integer UM_PER_REV     = 204203,
    parameter integer COUNTS_PER_REV = 1456,
    parameter integer ENC_FILTER_CYCLES = 125,   // 1 us 去抖 @125 MHz
    parameter integer TOF_I2C_HZ    = 100_000,
    parameter integer TOF_OFFSET_MM = 60,        // 前向实测偏置（读数=真实+60）
    parameter integer TOF_STOP_MM   = 120        // 停车边界
)(
    input  wire        clk_50m,
    input  wire        reset_n,
    input  wire        key_enable_n,     // KEY0，低有效
    input  wire        key_estop_n,      // KEY2，低有效
    input  wire        encoder_a_l,
    input  wire        encoder_b_l,
    input  wire        encoder_a_r,
    input  wire        encoder_b_r,
    // RGMII
    input  wire        rgmii_rx_clk_i,
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rxdv,
    output wire        eth_rst_n,
    output wire        eth_mdc,
    output wire        eth_mdio,
    output wire        rgmii_tx_clk,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_txen,
    // 前向 VL53L0X（软 I2C + XSHUT）
    inout  wire        tof_front_scl,
    inout  wire        tof_front_sda,
    inout  wire        tof_front_xshut_n,
    // 电机（TB6612）
    output wire        PWMA,
    output wire        AIN1,
    output wire        AIN2,
    output wire        PWMB,
    output wire        BIN1,
    output wire        BIN2,
    output wire        STBY,
    // 状态灯
    output wire        led_link,
    output wire        led_fault,
    output wire        led_run,
    output wire        led_ready
);
    wire reset_p = ~reset_n;

    // ---------------------------------------------------------------- PHY
    assign eth_rst_n = 1'b1;
    assign eth_mdc   = 1'b1;
    assign eth_mdio  = 1'b1;

    wire        clk125m, rgmii_rx_clk, gmii_rx_clk, gmii_rxdv, gmii_tx_clk, gmii_txen;
    wire [7:0]  gmii_rxd, gmii_txd;
    wire [47:0] exter_mac;
    wire [31:0] exter_ip;
    wire [15:0] exter_port, rx_data_length;
    wire        rx_payload_valid, rx_pkt_done, rx_pkt_err, data_overflow;
    wire [7:0]  rx_payload_dat;
    wire        tx_payload_req, tx_done, tx_en_pulse;
    wire [7:0]  tx_payload_dat;
    wire [15:0] tx_data_length;

    rgmii_to_gmii u_rgmii_rx (
        .reset(reset_p),
        .rgmii_rx_clk(rgmii_rx_clk), .rgmii_rxd(rgmii_rxd), .rgmii_rxdv(rgmii_rxdv),
        .gmii_rx_clk(gmii_rx_clk), .gmii_rxdv(gmii_rxdv), .gmii_rxd(gmii_rxd),
        .gmii_rxer()
    );

    eth_udp_rx_gmii u_udp_rx (
        .reset_p(reset_p),
        .local_mac(LOCAL_MAC), .local_ip(LOCAL_IP), .local_port(LOCAL_PORT),
        .clk125m_o(clk125m),
        .exter_mac(exter_mac), .exter_ip(exter_ip), .exter_port(exter_port),
        .rx_data_length(rx_data_length),
        .data_overflow_i(data_overflow),
        .payload_valid_o(rx_payload_valid), .payload_dat_o(rx_payload_dat),
        .one_pkt_done(rx_pkt_done), .pkt_error(rx_pkt_err),
        .debug_crc_check(),
        .gmii_rx_clk(gmii_rx_clk), .gmii_rxdv(gmii_rxdv), .gmii_rxd(gmii_rxd)
    );

    Gowin_PLL u_pll (
        .clkout0(rgmii_rx_clk),
        .clkin(rgmii_rx_clk_i)
    );

    // ------------------------------------------------------------ 链路桥接
    wire [7:0]  fault_bits;
    wire [15:0] frames_in, frames_out;
    wire        rx_overflow, tx_overflow, link_ok, stop_latched;
    wire        bridge_tx_start;
    wire [15:0] bridge_tx_len;
    wire [7:0]  bridge_tx_dat;
    wire signed [15:0] target_v_l, target_v_r;
    wire signed [31:0] wheel_mm_l, wheel_mm_r;   // 左右轮里程（mm），遥测用
    reg  signed [15:0] speed_l_mm_s, speed_r_mm_s;   // 100 ms 窗口实测轮速（mm/s）

    eth_link_bridge #(
        .CLK_HZ(125_000_000),
        .TELEM_PERIOD_MS(TELEM_PERIOD_MS),
        .CMD_TIMEOUT_MS(500),
        .HB_TIMEOUT_MS(300),
        .DEFAULT_LIMIT_MM_S(MAX_V_MM_S)
    ) u_bridge (
        .clk(clk125m), .rst_n(reset_n),
        .rx_valid(rx_payload_valid), .rx_data(rx_payload_dat),
        .tx_start(bridge_tx_start), .tx_len(bridge_tx_len),
        .eth_req(tx_payload_req), .eth_done(tx_done), .tx_data(bridge_tx_dat),
        .fault_bits(fault_bits), .frames_in(frames_in), .frames_out(frames_out),
        .rx_overflow(rx_overflow), .tx_overflow(tx_overflow),
        .link_ok(link_ok), .stop_latched(stop_latched),
        // 遥测 enc_l/enc_r <- 左右轮里程（mm），主机据此计算已走距离；
        // speed_l_mm_s/speed_r_mm_s <- 100 ms 窗口实测轮速，主机用它判断是否被切断
        .enc_l(wheel_mm_l), .enc_r(wheel_mm_r),
        .speed_l_mm_s(speed_l_mm_s), .speed_r_mm_s(speed_r_mm_s),
        .target_v_left(target_v_l), .target_v_right(target_v_r)
    );

    // ------------------------------------------------------------ 发送
    reg        tx_en_r;
    reg [15:0] tx_len_r;
    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n) begin
            tx_en_r  <= 1'b0;
            tx_len_r <= 16'd0;
        end else begin
            tx_en_r <= bridge_tx_start;
            if (bridge_tx_start) tx_len_r <= bridge_tx_len;
        end
    end
    assign tx_en_pulse    = tx_en_r;
    assign tx_data_length = tx_len_r;

    eth_udp_tx_gmii u_udp_tx (
        .clk125m(clk125m), .reset_p(reset_p),
        .tx_en_pulse(tx_en_pulse), .tx_done(tx_done),
        .dst_mac(HOST_MAC), .src_mac(LOCAL_MAC),
        .dst_ip(HOST_IP), .src_ip(LOCAL_IP),
        .dst_port(HOST_PORT), .src_port(LOCAL_PORT),
        .data_length(tx_data_length),
        .payload_req_o(tx_payload_req), .payload_dat_i(bridge_tx_dat),
        .gmii_tx_clk(gmii_tx_clk), .gmii_txen(gmii_txen), .gmii_txd(gmii_txd)
    );

    gmii_to_rgmii u_rgmii_tx (
        .reset_n(reset_n),
        .gmii_tx_clk(gmii_tx_clk), .gmii_txd(gmii_txd), .gmii_txen(gmii_txen),
        .gmii_txer(1'b0),
        .rgmii_tx_clk(rgmii_tx_clk), .rgmii_txd(rgmii_txd), .rgmii_txen(rgmii_txen)
    );

    // ------------------------------------------------ 双轮编码器 + 里程（mm）
    wire signed [1:0]  wheel_delta_l, wheel_delta_r;
    wire               enc_fault_l, enc_fault_r;

    dual_ab_quadrature_encoder #(
        .FILTER_CYCLES(ENC_FILTER_CYCLES),
        // 与主工程 / task4 板级工程一致的实车编码器相序：左轮反向、右轮正向，
        // 保证两轮「前进」计数都为正，上位机可直接用 enc_l/enc_r 求行程。
        .INVERT_L(1),
        .INVERT_R(0)
    ) u_encoder (
        .clk(clk125m), .rst_n(reset_n),
        .encoder_a_l(encoder_a_l), .encoder_b_l(encoder_b_l),
        .encoder_a_r(encoder_a_r), .encoder_b_r(encoder_b_r),
        .clear_fault_l(1'b0), .clear_fault_r(1'b0),
        .position_l(), .position_r(),
        .delta_count_l(wheel_delta_l), .delta_count_r(wheel_delta_r),
        .direction_l(), .direction_r(),
        .illegal_transition_l(), .illegal_transition_r(),
        .encoder_fault_l(enc_fault_l), .encoder_fault_r(enc_fault_r)
    );

    wire signed [31:0] wheel_um_l, wheel_um_r;   // 里程（um，Q16 累加）
    asr_distance #(.UM_PER_REV(UM_PER_REV), .COUNTS_PER_REV(COUNTS_PER_REV)) u_distance_l (
        .clk(clk125m), .rst_n(reset_n), .delta(wheel_delta_l), .position_um(wheel_um_l));
    asr_distance #(.UM_PER_REV(UM_PER_REV), .COUNTS_PER_REV(COUNTS_PER_REV)) u_distance_r (
        .clk(clk125m), .rst_n(reset_n), .delta(wheel_delta_r), .position_um(wheel_um_r));

    // 遥测字段单位 = mm：int32 = position_um / 1000（逐位除法，避免 125 MHz 组合除法）
    t4_div1000 u_mm_l (.clk(clk125m), .rst_n(reset_n), .din(wheel_um_l), .dout(wheel_mm_l));
    t4_div1000 u_mm_r (.clk(clk125m), .rst_n(reset_n), .din(wheel_um_r), .dout(wheel_mm_r));

    // ------------------------------------------- 实测轮速（100 ms 窗口，mm/s）
    // 遥测 speed_l_mm_s/speed_r_mm_s：每 100 ms 用「Δmm × 10」得到 mm/s
    // （10 mm/s 量化，只用于主机判断电机是否被安全门切断，不参与控制）。
    localparam integer SPEED_WINDOW_CYCLES = 125_000_000 / 10;   // 100 ms
    reg [23:0] speed_div;                                        // 12_500_000 < 2^24
    reg signed [31:0] last_mm_l, last_mm_r;

    function signed [15:0] sat16_speed(input signed [31:0] v);
        begin
            if (v > 32'sd32767)       sat16_speed = 16'sh7fff;
            else if (v < -32'sd32768) sat16_speed = -16'sd32768;
            else                      sat16_speed = v[15:0];
        end
    endfunction

    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n) begin
            speed_div     <= 24'd0;
            last_mm_l     <= 32'sd0;
            last_mm_r     <= 32'sd0;
            speed_l_mm_s  <= 16'sd0;
            speed_r_mm_s  <= 16'sd0;
        end else if (speed_div >= SPEED_WINDOW_CYCLES - 1) begin
            speed_div    <= 24'd0;
            last_mm_l    <= wheel_mm_l;
            last_mm_r    <= wheel_mm_r;
            speed_l_mm_s <= sat16_speed((wheel_mm_l - last_mm_l) * 32'sd10);
            speed_r_mm_s <= sat16_speed((wheel_mm_r - last_mm_r) * 32'sd10);
        end else begin
            speed_div <= speed_div + 24'd1;
        end
    end

    // ------------------------------------- 前向 VL53L0X（只做停车边界）
    localparam integer TOF_RESET_CYCLES   = 125_000_000 / 1000;   // 1 ms
    localparam integer TOF_BOOT_CYCLES    = 125_000_000 / 50;     // 20 ms
    localparam integer TOF_POLL_CYCLES    = 125_000_000 / 100;    // 10 ms / poll
    localparam integer TOF_TIMEOUT_CYCLES = 125_000_000 / 100;    // 10 ms I2C 卡死超时
    localparam integer TOF_TRIGGER_CYCLES = 125_000_000 / 1000 * 60;  // 60 ms 触发节拍

    reg  [23:0] tof_div;         // 7_500_000 < 2^23
    reg         tof_trigger;
    wire        tof_busy, tof_done, tof_sample_valid;
    wire [7:0]  tof_error;
    wire [15:0] tof_raw_mm;
    wire [15:0] tof_cal_mm;
    reg         tof_stop;

    tof_driver #(
        .CLK_HZ(125_000_000),
        .I2C_HZ(TOF_I2C_HZ),
        .RESET_CYCLES(TOF_RESET_CYCLES),
        .BOOT_CYCLES(TOF_BOOT_CYCLES),
        .POLL_CYCLES(TOF_POLL_CYCLES),
        .MAX_POLLS(100),
        .TIMEOUT_CYCLES(TOF_TIMEOUT_CYCLES)
    ) u_tof (
        .clk(clk125m), .rst_n(reset_n),
        .trigger(tof_trigger), .gpio1(1'b0),
        .scl(tof_front_scl), .sda(tof_front_sda), .xshut_n(tof_front_xshut_n),
        .gpio_sync(),
        .busy(tof_busy), .done(tof_done), .sample_valid(tof_sample_valid),
        .error(tof_error), .distance_mm(tof_raw_mm),
        .ambient(), .signal_count(),
        .model_id(), .revision(), .pre_period(), .final_period(),
        .range_status(), .diagnostic(), .read_mask(), .raw_result()
    );

    // 每 60 ms 给一次触发；忙时脉冲被驱动忽略，下一节拍再发。
    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n) begin
            tof_div     <= 24'd0;
            tof_trigger <= 1'b0;
        end else begin
            tof_trigger <= 1'b0;
            if (tof_div >= TOF_TRIGGER_CYCLES - 1) begin
                tof_div     <= 24'd0;
                tof_trigger <= !tof_busy;
            end else begin
                tof_div <= tof_div + 24'd1;
            end
        end
    end

    // 固定偏置标定：读数 - 60 mm（0/20 无效哨兵原样透传）
    tof_calibrate #(.OFFSET_MM(TOF_OFFSET_MM)) u_tof_cal (
        .din(tof_raw_mm), .dout(tof_cal_mm));

    // 停车边界：任意一次有效采样 <=120 mm 即锁存（复位才解除）
    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n)          tof_stop <= 1'b0;
        else if (tof_sample_valid && (tof_cal_mm <= TOF_STOP_MM)) tof_stop <= 1'b1;
    end

    // ------------------------------------------------- 速度 -> PWM -> H 桥
    localparam integer PWM_TICKS = 125_000_000 / PWM_HZ;   // 20 kHz -> 6250
    wire signed [15:0] pwm_l, pwm_r;
    wire motors_enabled;

    // KEY0 / KEY2 / ToF 停车硬门控（链路安全之外的第二层）
    wire motor_allow = key_estop_n && key_enable_n && !tof_stop;

    eth_motion_bridge #(
        .MAX_V_MM_S(MAX_V_MM_S),
        .PWM_TICKS(PWM_TICKS),
        .SCALE_NUM(PWM_TICKS / MAX_V_MM_S)     // 6250/300=20
    ) u_motion (
        .clk(clk125m), .rst_n(reset_n),
        .target_l(target_v_l), .target_r(target_v_r),
        // tof_stop / key 未使能时也清掉 motors_enabled（与 tb6612 enable 双重门控）
        .fault_bits(fault_bits), .link_ok(link_ok), .stop_latched(stop_latched),
        .key_estop_n(motor_allow),
        .pwm_l(pwm_l), .pwm_r(pwm_r),
        .motors_enabled(motors_enabled),
        .v_limited_l(), .v_limited_r()
    );

    wire motors_enable_gated = motors_enabled && motor_allow;

    // 主工程现成的 TB6612 驱动（含方向死区 + 安全 inhibit），直接复用不改动
    tb6612_driver_interface #(
        .CLK_HZ(125_000_000),
        .PWM_HZ(PWM_HZ),
        .DIR_DEADTIME_CYCLES(2500)
    ) u_driver (
        .clk(clk125m), .rst_n(reset_n),
        .enable(motors_enable_gated),
        .emergency_stop(!key_estop_n || !key_enable_n || tof_stop),
        .fault(fault_bits != 8'h00),
        .watchdog_timeout(!link_ok),
        .pwm_signed_l(pwm_l), .pwm_signed_r(pwm_r),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2),
        .PWMB(PWMB), .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY)
    );

    // ------------------------------------------------------------ 状态灯
    assign led_link  = link_ok;                                       // 链路存活
    assign led_fault = (fault_bits != 8'h00) || tof_stop ||
                       !key_estop_n || !key_enable_n;                 // 故障/停车
    assign led_run   = motors_enable_gated;                           // 电机已使能
    assign led_ready = link_ok && (fault_bits == 8'h00) &&
                       key_enable_n && key_estop_n && !tof_stop;      // 可以接收运动命令
endmodule

// ---------------------------------------------------------------------------
// 32 位有符号 /1000，逐位恢复除法（MSB 先处理）：约 34 拍出一次结果，连续
// 自由运行。结果向 0 截断，与 Verilog 有符号 "/" 语义一致。
// 用途：asr_distance 输出 Q16 µm，遥测 enc 字段要求 mm。
// ---------------------------------------------------------------------------
module t4_div1000 (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] din,
    output reg  signed [31:0] dout
);
    localparam [1:0] IDLE = 2'd0, DIV = 2'd1, FIN = 2'd2;

    reg        sign_r;
    reg [31:0] mag_r;
    reg [10:0] rem_r;      // 余数 < 1000；trial = {rem_r[9:0], bit} < 2000
    reg [31:0] quot_r;
    reg [4:0]  bit_r;
    reg [1:0]  state_r;

    wire [10:0] trial = {rem_r[9:0], mag_r[bit_r]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sign_r <= 1'b0;
            mag_r  <= 32'd0;
            rem_r  <= 11'd0;
            quot_r <= 32'd0;
            bit_r  <= 5'd0;
            state_r <= IDLE;
            dout   <= 32'sd0;
        end else begin
            case (state_r)
                IDLE: begin
                    sign_r  <= din[31];
                    mag_r   <= din[31] ? (32'd0 - din) : din;
                    rem_r   <= 11'd0;
                    quot_r  <= 32'd0;
                    bit_r   <= 5'd31;
                    state_r <= DIV;
                end
                DIV: begin
                    if (trial >= 11'd1000) begin
                        rem_r  <= trial - 11'd1000;
                        quot_r <= {quot_r[30:0], 1'b1};
                    end else begin
                        rem_r  <= trial;
                        quot_r <= {quot_r[30:0], 1'b0};
                    end
                    if (bit_r == 5'd0) state_r <= FIN;
                    else               bit_r   <= bit_r - 5'd1;
                end
                FIN: begin
                    dout    <= sign_r ? (32'd0 - quot_r) : quot_r;
                    state_r <= IDLE;
                end
                default: state_r <= IDLE;
            endcase
        end
    end
endmodule
