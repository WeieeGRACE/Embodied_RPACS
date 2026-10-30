`timescale 1ns/1ps
// 链路 v2（以太网/UDP）+ 电机输出 顶层 —— 明天动车测试用。
//
// 与 tests/eth_link_protocol 的唯一区别：SET_VEL 解出的 target_v_left/right 不再只是
// 遥测里的数字，而是经过 eth_motion_bridge → 主工程 tb6612_driver_interface 真正驱动
// PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY。
//
// 安全设计（全部硬连线，不依赖上位机）：
//   * link_ok 丢失（心跳 300 ms 超时）  -> watchdog_timeout -> STBY=0
//   * 任一故障位（含 CMD_TIMEOUT）      -> fault            -> STBY=0
//   * STOP 锁存                         -> enable=0         -> STBY=0
//   * KEY2（C15，低有效）按下           -> emergency_stop   -> STBY=0
//   * 复位未释放                        -> inhibit
// 另：tb6612_driver_interface 自带方向死区时间，避免 H 桥直通。
module eth_motion_top #(
    parameter [47:0] LOCAL_MAC  = 48'h00_0a_35_01_fe_c0,
    parameter [31:0] LOCAL_IP   = 32'hc0_a8_00_02,      // 192.168.0.2
    parameter [15:0] LOCAL_PORT = 16'd5000,
    parameter [31:0] HOST_IP    = 32'hc0_a8_00_01,      // 192.168.0.1
    parameter [15:0] HOST_PORT  = 16'd5001,
    parameter [47:0] HOST_MAC   = 48'hff_ff_ff_ff_ff_ff,
    parameter integer TELEM_PERIOD_MS = 20,
    parameter integer MAX_V_MM_S = 300,
    parameter integer PWM_HZ     = 20000
)(
    input  wire        clk_50m,
    input  wire        reset_n,
    input  wire        key_estop_n,
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
    output wire        led_run
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

    // ------------------------------------------------- 速度 -> PWM -> H 桥
    localparam integer PWM_TICKS = 125_000_000 / PWM_HZ;   // 20 kHz -> 6250
    wire signed [15:0] pwm_l, pwm_r;
    wire motors_enabled;

    eth_motion_bridge #(
        .MAX_V_MM_S(MAX_V_MM_S),
        .PWM_TICKS(PWM_TICKS),
        .SCALE_NUM(PWM_TICKS / MAX_V_MM_S)     // 6250/300=20
    ) u_motion (
        .clk(clk125m), .rst_n(reset_n),
        .target_l(target_v_l), .target_r(target_v_r),
        .fault_bits(fault_bits), .link_ok(link_ok), .stop_latched(stop_latched),
        .key_estop_n(key_estop_n),
        .pwm_l(pwm_l), .pwm_r(pwm_r),
        .motors_enabled(motors_enabled),
        .v_limited_l(), .v_limited_r()
    );

    // 主工程现成的 TB6612 驱动（含方向死区 + 安全 inhibit），直接复用不改动
    tb6612_driver_interface #(
        .CLK_HZ(125_000_000),
        .PWM_HZ(PWM_HZ),
        .DIR_DEADTIME_CYCLES(2500)
    ) u_driver (
        .clk(clk125m), .rst_n(reset_n),
        .enable(motors_enabled),
        .emergency_stop(!key_estop_n),
        .fault(fault_bits != 8'h00),
        .watchdog_timeout(!link_ok),
        .pwm_signed_l(pwm_l), .pwm_signed_r(pwm_r),
        .PWMA(PWMA), .AIN1(AIN1), .AIN2(AIN2),
        .PWMB(PWMB), .BIN1(BIN1), .BIN2(BIN2), .STBY(STBY)
    );

    // ------------------------------------------------------------ 状态灯
    assign led_link = ~link_ok;
    assign led_fault = ~(fault_bits != 8'h00);
    assign led_run   = ~motors_enabled;
endmodule
