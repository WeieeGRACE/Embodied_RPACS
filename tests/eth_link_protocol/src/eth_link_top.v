`timescale 1ns/1ps
// 链路协议 v2：以太网(UDP) 版本顶层。
//
// 结构与 eth_udp_link 的厂商例程一致（RGMII + UDP 收发 + 125 MHz PLL），只是把
// “收到什么就回什么”的回环换成 `eth_link_bridge`（协议帧编解码 + 安全/遥测）。
//
// 网络参数（写死，便于 Pi 侧固定配置）：
//   FPGA：MAC 00:0a:35:01:fe:c0, IP 192.168.0.2, UDP 5000（接收命令、发送遥测/ACK）
//   主机：IP 192.168.0.1, UDP 5001；目的 MAC 用广播 FF:FF:FF:FF:FF:FF（免 ARP 配置）
module eth_link_top #(
    parameter [47:0] LOCAL_MAC  = 48'h00_0a_35_01_fe_c0,
    parameter [31:0] LOCAL_IP   = 32'hc0_a8_00_02,      // 192.168.0.2
    parameter [15:0] LOCAL_PORT = 16'd5000,
    parameter [31:0] HOST_IP    = 32'hc0_a8_00_01,      // 192.168.0.1
    parameter [15:0] HOST_PORT  = 16'd5001,
    parameter [47:0] HOST_MAC   = 48'hff_ff_ff_ff_ff_ff,
    parameter integer TELEM_PERIOD_MS = 20
)(
    input  wire        clk_50m,          // 板上 50 MHz（本设计由 PHY 的 rx_clk 派生主时钟，此脚保留）
    input  wire        reset_n,
    input  wire        rgmii_rx_clk_i,
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rxdv,
    output wire        eth_rst_n,
    output wire        eth_mdc,
    output wire        eth_mdio,
    output wire        rgmii_tx_clk,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_txen,
    // 状态灯（可选）：link_ok / 有故障 / 收发包活动
    output wire        led_link,
    output wire        led_fault,
    output wire        led_activity
);
    wire         reset_p = ~reset_n;

    // ---------------------------------------------------------------- PHY 侧
    // 例程做法：不复位/不配置 PHY，靠自协商；MDIO 上拉
    assign eth_rst_n = 1'b1;
    assign eth_mdc   = 1'b1;
    assign eth_mdio  = 1'b1;      // 方向按输出处理（板上 PHY 侧有上拉）

    wire        clk125m;
    wire        rgmii_rx_clk;
    wire        gmii_rx_clk, gmii_rxdv, gmii_tx_clk, gmii_txen;
    wire [7:0]  gmii_rxd, gmii_txd;
    wire [47:0] exter_mac;
    wire [31:0] exter_ip;
    wire [15:0] exter_port, rx_data_length;
    wire        rx_payload_valid, rx_pkt_done, rx_pkt_err, data_overflow;
    wire [7:0]  rx_payload_dat;
    wire        tx_payload_req, tx_done;
    wire [7:0]  tx_payload_dat;
    wire        tx_en_pulse;
    wire [15:0] tx_data_length;

    // RGMII <-> GMII
    rgmii_to_gmii u_rgmii_rx (
        .reset(reset_p),
        .rgmii_rx_clk(rgmii_rx_clk), .rgmii_rxd(rgmii_rxd), .rgmii_rxdv(rgmii_rxdv),
        .gmii_rx_clk(gmii_rx_clk), .gmii_rxdv(gmii_rxdv), .gmii_rxd(gmii_rxd),
        .gmii_rxer()
    );

    // UDP 接收（同时产生 clk125m 主时钟）
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

    // 相位调整后的 125 MHz（RX 采样用）
    Gowin_PLL u_pll (
        .clkout0(rgmii_rx_clk),
        .clkin(rgmii_rx_clk_i)
    );

    // ------------------------------------------------------------ 协议桥接层
    wire [7:0]  fault_bits;
    wire [15:0] frames_in, frames_out;
    wire        rx_overflow, tx_overflow, link_ok, stop_latched;
    wire        bridge_tx_start;
    wire [15:0] bridge_tx_len;
    wire [7:0]  bridge_tx_dat;

    // 主时钟：clk125m（UDP RX 模块输出，PHY rxclk 派生）
    eth_link_bridge #(
        .CLK_HZ(125_000_000),
        .TELEM_PERIOD_MS(TELEM_PERIOD_MS),
        .CMD_TIMEOUT_MS(500),
        .HB_TIMEOUT_MS(300),
        .DEFAULT_LIMIT_MM_S(300)
    ) u_bridge (
        .clk(clk125m), .rst_n(reset_n),
        .rx_valid(rx_payload_valid), .rx_data(rx_payload_dat),
        .tx_start(bridge_tx_start), .tx_len(bridge_tx_len),
        .eth_req(tx_payload_req), .eth_done(tx_done), .tx_data(bridge_tx_dat),
        .fault_bits(fault_bits), .frames_in(frames_in), .frames_out(frames_out),
        .rx_overflow(rx_overflow), .tx_overflow(tx_overflow),
        .link_ok(link_ok), .stop_latched(stop_latched)
    );

    // ------------------------------------------- 发送：固定目的主机（广播 MAC）
    // 桥接层给出 tx_start/tx_len 后，启动厂商 TX 引擎；引擎逐字节取数后给 tx_done。
    reg        tx_en_r;
    reg [15:0] tx_len_r;
    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n) begin
            tx_en_r  <= 1'b0;
            tx_len_r <= 16'd0;
        end else begin
            tx_en_r <= bridge_tx_start;          // 单拍延迟，保证 dst 字段稳定
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

    // ---------------------------------------------------------------- 状态灯
    // 板上 LED 低电平点亮；把“链路/故障/活动”三个状态接出来方便肉眼观察
    assign led_link     = ~link_ok;
    assign led_fault    = ~(fault_bits != 8'h00);
    assign led_activity = ~(rx_pkt_done | bridge_tx_start);
endmodule
