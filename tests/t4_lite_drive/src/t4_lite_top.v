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
//   * 速度闭环（t4_wheel_speed_loop）：SET_VEL 的 mm/s 目标经主工程
//     speed_measurement_and_velocity_filter -> dual_speed_pi_pid_controller ->
//     pwm_and_startup_ramp 得到 duty，取代原来 eth_motion_bridge 的开环映射
//     （duty = v*20）。安全门控/motors_enabled/STOP/KEY2/ToF 均原样保留。
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
    // ---- 速度闭环参数（详见 src/t4_wheel_speed_loop.v 文件头注释）----
    // 窗口 20 ms：50 mm/s -> 7 counts/窗口 = 49.1 mm/s（10 ms 窗口会变成 4 counts=56.1）。
    parameter integer SPEED_WINDOW_MS   = 20,
    parameter signed [15:0] SPEED_KP    = 16,     // 与主工程等效的连续域 Kp（=32@10ms 折算）
    parameter signed [15:0] SPEED_KI    = 4,
    parameter signed [15:0] SPEED_KD    = 0,
    parameter integer SPEED_GAIN_SHIFT  = 0,
    parameter integer SPEED_DUTY_DIV    = 2,      // PI/ramp duty 上限 = PWM_TICKS/2
    parameter integer SPEED_RAMP_STEP   = 1,      // 每 PWM 周期最多变 1 tick
    // 65 mm 轮径标定（与 tests/task4_navigation 一致）
    parameter integer UM_PER_REV     = 204203,
    parameter integer COUNTS_PER_REV = 1456,
    parameter integer ENC_FILTER_CYCLES = 125,   // 1 us 去抖 @125 MHz
    parameter integer TOF_I2C_HZ    = 100_000,
    parameter integer TOF_OFFSET_MM = 60,        // 前向实测偏置（读数=真实+60）
    parameter integer TOF_STOP_EN   = 0,        // 1=ToF 兜底停车启用
    parameter integer TOF_STOP_MM   = 120        // 停车边界
)(
    input  wire        clk_50m,
    input  wire        reset_n,
    input  wire        key_enable_n,     // KEY0，低有效
    input  wire        key_estop_n,      // KEY2，低有效
    // IMU660RA SPI（陀螺仪航向修正用）
    input  wire        imu_spi_miso,
    output wire        imu_spi_sck,
    output wire        imu_spi_mosi,
    output wire        imu_cs_n,
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
    // 调试遥测（在 bridge 例化之前声明，避免隐式网）
    wire [15:0] dbg_a;
    wire [15:0] dbg_b;

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
    // 遥测航向（厘度，int16）。IMU 链在 50 MHz 域解算，经 toggle 握手 CDC 后
    // 在本域寄存器里更新，供 eth_link_bridge 每 20 ms 遥测快照取用。
    reg  signed [15:0] yaw_cdeg;

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
        .target_v_left(target_v_l), .target_v_right(target_v_r),
        .dbg_a(dbg_a),
        .dbg_b(dbg_b),
        .yaw_cdeg_in(yaw_cdeg)
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
    // 时序：原实现把「快照 -> 差值 -> ×10 -> 饱和」压在同一个 125 MHz 周期内，
    // 起点是 t4_div1000 的输出寄存器 wheel_mm_l/r，PnR SETUP slack 约 -0.32 ns。
    // 这里改成每 100 ms 只走一次的 3 拍小 FSM，把该组合链拆成 3 段；数值结果与
    // 原来逐位完全相同，仅在本窗口边界后 3 个时钟（24 ns）才更新遥测量。
    localparam integer SPEED_WINDOW_CYCLES = 125_000_000 / 10;   // 100 ms
    reg [23:0] speed_div;                                        // 12_500_000 < 2^24
    reg signed [31:0] last_mm_l, last_mm_r;                      // 上窗口快照
    reg signed [31:0] sample_mm_l, sample_mm_r;                  // 本窗口快照
    reg signed [31:0] delta_mm_l, delta_mm_r;
    reg signed [31:0] prod_mm_l, prod_mm_r;
    reg [1:0]  speed_state;
    localparam [1:0] SP_WAIT = 2'd0, SP_DIFF = 2'd1, SP_MUL = 2'd2, SP_SAT = 2'd3;

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
            sample_mm_l   <= 32'sd0;
            sample_mm_r   <= 32'sd0;
            delta_mm_l    <= 32'sd0;
            delta_mm_r    <= 32'sd0;
            prod_mm_l     <= 32'sd0;
            prod_mm_r     <= 32'sd0;
            speed_state   <= SP_WAIT;
            speed_l_mm_s  <= 16'sd0;
            speed_r_mm_s  <= 16'sd0;
        end else case (speed_state)
            SP_WAIT: begin
                if (speed_div >= SPEED_WINDOW_CYCLES - 1) begin
                    speed_div   <= 24'd0;
                    sample_mm_l <= wheel_mm_l;
                    sample_mm_r <= wheel_mm_r;
                    speed_state <= SP_DIFF;
                end else begin
                    speed_div <= speed_div + 24'd1;
                end
            end
            SP_DIFF: begin
                delta_mm_l  <= sample_mm_l - last_mm_l;
                delta_mm_r  <= sample_mm_r - last_mm_r;
                last_mm_l   <= sample_mm_l;
                last_mm_r   <= sample_mm_r;
                speed_state <= SP_MUL;
            end
            SP_MUL: begin
                prod_mm_l   <= delta_mm_l * 32'sd10;
                prod_mm_r   <= delta_mm_r * 32'sd10;
                speed_state <= SP_SAT;
            end
            SP_SAT: begin
                speed_l_mm_s <= sat16_speed(prod_mm_l);
                speed_r_mm_s <= sat16_speed(prod_mm_r);
                speed_state  <= SP_WAIT;
            end
            default: speed_state <= SP_WAIT;
        endcase
    end

    // ------------------------------- IMU660RA 陀螺仪航向（50 MHz 独立时钟域）
    //
    // 时序问题定位（impl/pnr/t4_lite_top.timing_paths）：原设计把 IMU 驱动和
    // 姿态解算都放在 125 MHz 域，最差路径为
    //   u_attitude/yaw_mdeg_1_s0 -> (组合 /10 饱和网络 n1570..n1585) -> yaw_cdeg_*
    // SETUP slack -10.705 ns（到达 21.098 ns，要求 10.394 ns）；姿态解算内部的
    // 32 位加/除/乘链在 125 MHz 下同样无法收敛。
    //
    // 板上 T9 是独立 50 MHz 晶振（与主工程 src/robot_system_top.cst 的 clk_50m
    // 同一引脚，主工程的 IMU 链就运行在 50 MHz、20 ns 约束下且 SETUP 收敛）。
    // 因此把 imu660ra_driver + imu_attitude_fixed 整体搬到 clk_50m 域：
    //   50 MHz 域：200 Hz 姿态更新 -> 32 拍迭代 /10 + int16 饱和 -> yaw_cdeg_50
    //              寄存器保持，并翻转 yaw_cdeg_50_toggle（数据保持 + toggle 握手）；
    //   125 MHz 域：toggle 经 3 级 FF 同步 + 边沿检测，边沿处一次性锁存 16 位数据，
    //              得到与原来语义相同的 yaw_cdeg（毫度/10，int16 饱和，±180° 回绕）。
    // 两次更新间隔 5 ms，远大于同步器与数据连线延迟，逐位同步该数据总线是安全的。
    // 解卷绕仍由主机侧完成（RTL 不做 asr_yaw_unwrap）。
    localparam integer CLK50_HZ = 50_000_000;
    localparam integer IMU_POWERUP_CYCLES50 = CLK50_HZ / 50;     // 20 ms
    localparam integer IMU_WAIT_CYCLES50    = CLK50_HZ / 50;     // 20 ms
    localparam integer IMU_RETRY_CYCLES50   = CLK50_HZ;          // 1 s
    localparam integer IMU_SAMPLE_CYCLES50  = CLK50_HZ / 200;    // 200 Hz
    localparam integer IMU_SPI_HALF50       = 25;                // ~1 MHz SPI

    // reset_n 在 50 MHz 域内"异步置位、同步释放"，避免复位释放沿相对
    // clk_50m 的亚稳态（125 MHz 域沿用原有 reset_n 用法，不改变既有行为）。
    reg [1:0] rst50_sync;
    always @(posedge clk_50m or negedge reset_n) begin
        if (!reset_n) rst50_sync <= 2'b00;
        else          rst50_sync <= {rst50_sync[0], 1'b1};
    end
    wire rst50_n = rst50_sync[1];

    wire               imu_ready, imu_sample_valid, imu_sensor_error;
    wire signed [15:0] imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz;
    wire [7:0]         imu_error_code;

    imu660ra_driver #(
        .CLK_HZ(CLK50_HZ), .SPI_HALF_PERIOD(IMU_SPI_HALF50),
        .POWERUP_CYCLES(IMU_POWERUP_CYCLES50), .CONFIG_WAIT_CYCLES(IMU_WAIT_CYCLES50),
        .RETRY_CYCLES(IMU_RETRY_CYCLES50), .ID_RETRY_CYCLES(CLK50_HZ / 1000),
        .SAMPLE_CYCLES(IMU_SAMPLE_CYCLES50), .HEALTH_CHECK_CYCLES(CLK50_HZ),
        .CONFIG_HEX_FILE("src/imu660ra_config.hex")
    ) u_imu (
        .clk(clk_50m), .rst_n(rst50_n),
        .spi_miso(imu_spi_miso), .imu_int1(1'b0), .imu_int2(1'b0),
        .spi_sclk(imu_spi_sck), .spi_mosi(imu_spi_mosi), .spi_cs_n(imu_cs_n),
        .accel_x(imu_ax), .accel_y(imu_ay), .accel_z(imu_az),
        .gyro_x(imu_gx), .gyro_y(imu_gy), .gyro_z(imu_gz),
        .sample_valid(imu_sample_valid), .sample_sequence(), .chip_id(), .init_status(),
        .power_conf_readback(), .init_ctrl_readback(), .sensor_error(imu_sensor_error),
        .ready(imu_ready), .error_code(imu_error_code), .int1_sync(), .int2_sync()
    );

    wire               imu_attitude_valid, imu_attitude_update;
    wire signed [31:0] imu_yaw_wrapped;

    imu_attitude_fixed #(.CALIBRATION_SAMPLES(200)) u_attitude (
        .clk(clk_50m), .rst_n(rst50_n), .ready_in(imu_ready), .sample_valid(imu_sample_valid),
        .accel_x(imu_ax), .accel_y(imu_ay), .accel_z(imu_az),
        .gyro_x(imu_gx), .gyro_y(imu_gy), .gyro_z(imu_gz),
        .calibrating(), .attitude_valid(imu_attitude_valid), .update_valid(imu_attitude_update),
        .roll_mdeg(), .pitch_mdeg(), .yaw_mdeg(imu_yaw_wrapped),
        .gyro_bias_x(), .gyro_bias_y(), .gyro_bias_z()
    );

    // update_valid 与 yaw_mdeg 同拍更新；延迟一拍再启动 /10，除数取的才是新航向。
    reg imu_update_d50;
    always @(posedge clk_50m or negedge rst50_n) begin
        if (!rst50_n) imu_update_d50 <= 1'b0;
        else          imu_update_d50 <= imu_attitude_update;
    end

    // 毫度 -> 厘度（32 拍逐位 /10，向 0 截断；单拍只有 5 位比较/减法）。
    // 原组合 /10 + 饱和在 125 MHz 域是 21 ns 的关键路径，这里改成迭代计算，
    // 数值语义保持不变（yaw_mdeg/10 后 int16 饱和）。
    wire               cdeg_div_busy, cdeg_div_done;
    wire signed [15:0] cdeg_div_value;
    reg  signed [31:0] cdeg_div_din;
    reg                cdeg_div_start;

    t4_div10_sat u_yaw_div (
        .clk(clk_50m), .rst_n(rst50_n),
        .start(cdeg_div_start), .din(cdeg_div_din),
        .busy(cdeg_div_busy), .done(cdeg_div_done), .cdeg(cdeg_div_value)
    );

    always @(posedge clk_50m or negedge rst50_n) begin
        if (!rst50_n) begin
            cdeg_div_start <= 1'b0;
            cdeg_div_din   <= 32'sd0;
        end else begin
            cdeg_div_start <= 1'b0;
            if (imu_update_d50 && !cdeg_div_busy) begin
                cdeg_div_start <= 1'b1;
                cdeg_div_din   <= imu_yaw_wrapped;
            end
        end
    end

    // 数据保持 + toggle：结果每 5 ms 才变一次，握手期间保持稳定。
    reg signed [15:0] yaw_cdeg50;
    reg               yaw_cdeg50_toggle;
    always @(posedge clk_50m or negedge rst50_n) begin
        if (!rst50_n) begin
            yaw_cdeg50        <= 16'sd0;
            yaw_cdeg50_toggle <= 1'b0;
        end else if (cdeg_div_done) begin
            yaw_cdeg50        <= cdeg_div_value;
            yaw_cdeg50_toggle <= ~yaw_cdeg50_toggle;
        end
    end

    // 125 MHz 遥测域：toggle 多 FF 同步 + 上升/下降沿检测，边沿处锁存数据总线。
    reg [2:0] yaw_toggle_sync;
    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n) yaw_toggle_sync <= 3'b000;
        else          yaw_toggle_sync <= {yaw_toggle_sync[1:0], yaw_cdeg50_toggle};
    end
    wire yaw_cdeg_update = yaw_toggle_sync[2] ^ yaw_toggle_sync[1];

    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n)          yaw_cdeg <= 16'sd0;
        else if (yaw_cdeg_update) yaw_cdeg <= yaw_cdeg50;
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

    // 停车边界：连续 3 次有效采样（排除 0/20 哨兵，25..2500 量程内）<=120 mm 才锁存。
    // 单次 VL53L0X 跳变/噪声不能直接切断电机。
    reg [2:0] tof_close_cnt;
    wire tof_sample_good = tof_sample_valid && tof_raw_mm >= 16'd25 && tof_raw_mm <= 16'd2500;
    always @(posedge clk125m or negedge reset_n) begin
        if (!reset_n) begin
            tof_stop      <= 1'b0;
            tof_close_cnt <= 3'd0;
        end else if (!tof_stop && TOF_STOP_EN != 0) begin
            if (!tof_sample_good) begin
                tof_close_cnt <= 3'd0;
            end else if (tof_cal_mm <= TOF_STOP_MM) begin
                if (tof_close_cnt >= 2'd4) tof_stop <= 1'b1;
                else                       tof_close_cnt <= tof_close_cnt + 3'd1;
            end else begin
                tof_close_cnt <= 3'd0;
            end
        end
    end

    // ------------------------------------- SET_VEL 限幅 + 安全门 + 双轮速度闭环
    localparam integer PWM_TICKS = 125_000_000 / PWM_HZ;   // 20 kHz -> 6250
    wire signed [15:0] pwm_l, pwm_r;      // 速度环输出：有符号 duty（单位 = clk tick）
    wire motors_enabled;

    // KEY0 / KEY2 / ToF 停车硬门控（链路安全之外的第二层）
    // KEY1 不再参与门控（本工程由主机显式发运动命令）；保留 KEY2 急停 + ToF 兜底停。
    wire motor_allow = key_estop_n && !tof_stop;

    // 旧开环映射（duty = v*20）仍保留在 eth_motion_bridge 内，但顶层不再使用其
    // pwm_l/pwm_r；这里只取它限幅到 +/-MAX_V_MM_S 后的目标速度（SET_VEL 语义不变），
    // 连同它的 motors_enabled 安全门一起原样复用。
    wire signed [15:0] target_limited_l, target_limited_r;

    eth_motion_bridge #(
        .MAX_V_MM_S(MAX_V_MM_S),
        .PWM_TICKS(PWM_TICKS),
        .SCALE_NUM(PWM_TICKS / MAX_V_MM_S)     // 6250/300=20（旧开环映射，本工程不用）
    ) u_motion (
        .clk(clk125m), .rst_n(reset_n),
        .target_l(target_v_l), .target_r(target_v_r),
        // tof_stop / key 未使能时也清掉 motors_enabled（与 tb6612 enable 双重门控）
        .fault_bits(fault_bits), .link_ok(link_ok), .stop_latched(stop_latched),
        .key_estop_n(motor_allow),
        .pwm_l(), .pwm_r(),
        .motors_enabled(motors_enabled),
        .v_limited_l(target_limited_l), .v_limited_r(target_limited_r)
    );

    wire motors_enable_gated = motors_enabled && motor_allow;

    // 主工程速度闭环链：编码器增量 -> 窗口测速 -> PI -> duty 斜坡。
    // 1 ms = 125_000 clk @125 MHz；窗口长度由顶层参数 SPEED_WINDOW_MS 给出。
    t4_wheel_speed_loop #(
        .CLK_HZ(125_000_000),
        .PWM_HZ(PWM_HZ),
        .WINDOW_CYCLES(125_000 * SPEED_WINDOW_MS),
        .FILTER_LENGTH(4),
        .COUNTS_PER_REV(COUNTS_PER_REV),
        .UM_PER_REV(UM_PER_REV),
        .KP(SPEED_KP), .KI(SPEED_KI), .KD(SPEED_KD),
        .GAIN_SHIFT(SPEED_GAIN_SHIFT),
        .DUTY_DIV(SPEED_DUTY_DIV),
        .RAMP_STEP(SPEED_RAMP_STEP)
    ) u_speed_loop (
        .clk(clk125m), .rst_n(reset_n),
        .enable(motors_enable_gated),
        .target_l_mm_s(target_limited_l), .target_r_mm_s(target_limited_r),
        .delta_count_l(wheel_delta_l), .delta_count_r(wheel_delta_r),
        .pwm_l(pwm_l), .pwm_r(pwm_r),
        // 调试观测口（本工程遥测字段不变，不占用 dbg_a/dbg_b）
        .control_l(), .control_r(),
        .speed_count_l(), .speed_count_r(),
        .speed_valid()
    );

    // 主工程现成的 TB6612 驱动（含方向死区 + 安全 inhibit），直接复用不改动
    tb6612_driver_interface #(
        .CLK_HZ(125_000_000),
        .PWM_HZ(PWM_HZ),
        .DIR_DEADTIME_CYCLES(2500)
    ) u_driver (
        .clk(clk125m), .rst_n(reset_n),
        .enable(motors_enable_gated),
        .emergency_stop(!key_estop_n || tof_stop),
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
    // ---- 调试遥测赋值（模块末尾，所有信号都已声明）----
    //   dbg_a = ToF 标定距离(mm)
    //   dbg_b[0]=tof_stop [1]=tof_sample_valid [2]=tof_error!=0 [3]=key_enable_n
    //         [4]=key_estop_n [5]=motor_allow [6]=link_ok [7]=motors_enabled
    assign dbg_a = tof_cal_mm;
    assign dbg_b = {8'd0, motors_enabled, link_ok, motor_allow,
                    key_estop_n, key_enable_n, (tof_error != 8'd0),
                    tof_sample_valid, tof_stop};

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

// ---------------------------------------------------------------------------
// 32 位有符号 /10（向 0 截断）+ int16 饱和，逐位恢复除法（MSB 先处理）：
// 约 34 拍出一次结果，start/done 握手，非连续自由运行。
// 数值语义 = 原顶层组合表达式 sat16(yaw_mdeg / 10)：
//   - 商向 0 截断（与 Verilog 有符号 "/" 一致）；
//   - 结果饱和到 int16（+32767 / -32768）。
// 用途：把 IMU 姿态航向从「毫度」换成遥测「厘度」，同时避免 125 MHz 域里
//      那条约 21 ns 的组合除法关键路径（现在整个 IMU 链在 50 MHz 域，且单拍
//      只有 5 位比较/减法）。
// ---------------------------------------------------------------------------
module t4_div10_sat (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [31:0] din,
    output reg                busy,
    output reg                done,
    output reg  signed [15:0] cdeg
);
    localparam [1:0] IDLE = 2'd0, DIV = 2'd1, SAT = 2'd2;

    reg        sign_r;
    reg [31:0] mag_r;
    reg [4:0]  rem_r;      // 余数 < 10（rem_r[3:0]）；trial = {rem_r[3:0], bit} < 20
    reg [31:0] quot_r;
    reg [5:0]  bit_r;
    reg [1:0]  state_r;

    wire [4:0] trial = {rem_r[3:0], mag_r[bit_r]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sign_r  <= 1'b0;
            mag_r   <= 32'd0;
            rem_r   <= 5'd0;
            quot_r  <= 32'd0;
            bit_r   <= 6'd0;
            state_r <= IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            cdeg    <= 16'sd0;
        end else begin
            done <= 1'b0;
            case (state_r)
                IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;
                        sign_r  <= din[31];
                        mag_r   <= din[31] ? (32'd0 - din) : din;
                        rem_r   <= 5'd0;
                        quot_r  <= 32'd0;
                        bit_r   <= 6'd31;
                        state_r <= DIV;
                    end
                end
                DIV: begin
                    if (trial >= 5'd10) begin
                        rem_r  <= trial - 5'd10;   // trial <= 19 -> 差 <= 9
                        quot_r <= {quot_r[30:0], 1'b1};
                    end else begin
                        rem_r  <= trial;
                        quot_r <= {quot_r[30:0], 1'b0};
                    end
                    if (bit_r == 6'd0) state_r <= SAT;
                    else               bit_r   <= bit_r - 6'd1;
                end
                SAT: begin
                    if (!sign_r) begin
                        cdeg <= (quot_r > 32'd32767) ? 16'sd32767 : quot_r[15:0];
                    end else if (quot_r > 32'd32768) begin
                        cdeg <= -16'sd32768;
                    end else begin
                        cdeg <= 16'sd0 - quot_r[15:0];
                    end
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    state_r <= IDLE;
                end
                default: state_r <= IDLE;
            endcase
        end
    end

endmodule
