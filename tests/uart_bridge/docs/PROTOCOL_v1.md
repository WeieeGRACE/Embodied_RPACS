# 上位机 ↔ FPGA 链路协议 v1（冻结版）

| 项 | 值 |
| --- | --- |
| 文档版本 | v1.0 |
| 冻结时间 | 2026-10-28 |
| 适用范围 | 深度相机/上位机（Orange Pi / PC）→ FPGA 主控 的命令与遥测链路 |
| 状态 | **冻结**：帧格式与消息编码不再改动；新增消息只能追加 TYPE 编号 |
| 关联 | `tests/orangepi_depth_perception/PLAN.md` §9.1（v1 链路）、§9.2（v2 以太网）、§9.4（安全原则） |

---

## 1. 分层模型（为什么这样切）

```text
┌──────────────────────────────────────────────────────────┐
│ 应用层消息   SET_VEL / STOP / HEARTBEAT / SET_LIMIT /     │
│              CLEAR_FAULT / TELEM / ACK / NACK            │
├──────────────────────────────────────────────────────────┤
│ 帧层（与传输无关）                                        │
│   AA 55 | TYPE | LEN | PAYLOAD | CRC16-CCITT | 0D        │
├──────────────────────────────────────────────────────────┤
│ 传输绑定                                                  │
│   A: UART 115200 8N1（当前）                              │
│   B: UDP over 千兆以太网（未来，帧层完全复用）             │
└──────────────────────────────────────────────────────────┘
```

**设计约束**：帧层只依赖"有序字节流"这一假设。因此

- RTL 侧帧解析器只吃 `(byte, byte_valid)`，UART 与以太网只是不同的"字节源适配器"；
- 上位机侧 `protocol.py` 只做编解码，`transport_uart.py` / `transport_udp.py` 负责搬运；
- 换成千兆网时，**帧格式、消息 ID、CRC、时序语义一字不改**，只替换传输绑定。

---

## 2. 帧格式

### 2.1 字节序

```text
偏移  字段        长度   说明
0     SYNC0       1     0xAA
1     SYNC1       1     0x55
2     TYPE        1     消息类型，见 §3
3     LEN         1     PAYLOAD 字节数，0..32
4     PAYLOAD     LEN   UTF-8/二进制负载，多字节整数一律 **小端**
4+LEN CRC16       2     CRC16-CCITT-FALSE，**小端**（低字节在前）
      覆盖范围 = TYPE..PAYLOAD 末尾（不含 AA 55，不含 CRC 自身，不含 0D）
6+LEN TAIL        1     0x0D
```

总长 = `7 + LEN` 字节。

### 2.2 CRC 参数（必须逐位一致，否则两端不通用）

```text
算法      : CRC-16/CCITT-FALSE
多项式    : 0x1021  (x^16 + x^12 + x^5 + 1)
初值      : 0xFFFF
输入反射  : 否
输出反射  : 否
结果异或  : 0x0000
```

**参考实现（C，逐位）**：

```c
uint16_t crc16_ccitt(const uint8_t *p, size_t n, uint16_t crc) {
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
```

### 2.3 CRC 测试向量（两端自检必须全过）

由 `tools/protocol_selftest.py` 生成，RTL 的 `sim/tb_crc16.sv` 使用同一组向量，**双向比对**。

| # | 输入字节（TYPE..PAYLOAD） | 十六进制 | 期望 CRC |
| --- | --- | --- | --- |
| V1 | 空 | `` | `0xFFFF` |
| V2 | ASCII "123456789" | `31 32 33 34 35 36 37 38 39` | **`0x29B1`**（标准校验值，必须过） |
| V3 | 单字节 | `01` | `0xF1D1` |
| V4 | TELEM 全零载荷（TYPE=0x81, LEN=23, 23×`00`） | `81 17` + 23×`00` | `0xD80F` |
| V5 | SET_VEL(200,−150,seq=0x2A) 完整帧 | `AA 55 01 05 C8 00 6A FF 2A 46 E1 0D` | 帧内 CRC=`46 E1`(LE)=`0xE146` |

### 2.4 完整帧示例（SET_VEL：左 200 mm/s、右 −150 mm/s、seq=0x2A）

```text
PAYLOAD = C8 00 6A FF 2A     (int16 LE 200, int16 LE -150, uint8 seq)
CRC 覆盖 01 05 C8 00 6A FF 2A → 0xE146 → 小端 46 E1
整帧 = AA 55 01 05 C8 00 6A FF 2A 46 E1 0D   （12 字节 = 7 + LEN）
```

---

## 3. 消息定义

### 3.1 下行（上位机 → FPGA）

| TYPE | 名称 | LEN | PAYLOAD | 周期 | 语义 |
| --- | --- | --- | --- | --- | --- |
| `0x01` | `SET_VEL` | 5 | `int16 v_left` (mm/s), `int16 v_right` (mm/s), `uint8 seq` | 20 Hz | 设目标速度。**不直接驱动 PWM**，只写"目标速度"寄存器，经安全仲裁后生效。`seq` 由上位机自增，FPGA 原样回显到 `TELEM.cmd_seq` 用于统计丢帧 |
| `0x02` | `STOP` | 0 | — | 事件 | 立即停车，**锁存**；需 `CLEAR_FAULT` 才能解除 |
| `0x03` | `HEARTBEAT` | 1 | `uint8 seq` | 5 Hz | 保活。200 ms 未收到 → `LINK_LOST` |
| `0x04` | `SET_LIMIT` | 2 | `uint16 limit_mm_s` | 事件 | 前进速度上限（默认 300 mm/s） |
| `0x05` | `CLEAR_FAULT` | 0 | — | 事件 | 显式复位锁存故障（`STOP` 锁存 / `LINK_LOST`），**且要求链路已恢复** |

### 3.2 上行（FPGA → 上位机）

| TYPE | 名称 | LEN | PAYLOAD | 周期 |
| --- | --- | --- | --- | --- |
| `0x81` | `TELEM` | 23 | 见下 | 50 Hz |
| `0x82` | `ACK` | 3 | `uint8 acked_type`, `uint8 seq`, `uint8 status` | 事件 |

**TELEM 负载布局（23 B，小端）**

```text
偏移  字段            类型     单位      说明
0     enc_l           int32    计数      左轮编码器累计
4     enc_r           int32    计数      右轮编码器累计
8     yaw             int16    0.01°     IMU 航向（−18000..18000）
10    pitch           int16    0.01°     IMU 俯仰
12    roll            int16    0.01°     IMU 横滚
14    speed_l         int16    mm/s      左轮实测速度
16    speed_r         int16    mm/s      右轮实测速度
18    fault           uint8    —         故障位图，见 §4.3
19    link_seq        uint8    —         最近一次成功接收的 HEARTBEAT 序号
20    cmd_seq         uint8    —         最近一次成功 SET_VEL 的 seq（原样回显，用于丢帧统计）
21    hb_age_ms       uint16   ms        距上次心跳的毫秒数（饱和到 65535）
```

**ACK 状态码**

| status | 含义 |
| --- | --- |
| `0x00` | OK，命令已生效 |
| `0x01` | 被拒绝：链路未恢复（`LINK_LOST` 未清） |
| `0x02` | 被拒绝：`STOP` 锁存中 |
| `0x03` | 被拒绝：超速（超过 `SET_LIMIT`） |
| `0x04` | 不支持的命令 |

---

## 4. 时序、故障与仲裁

### 4.1 时基

| 参数 | 值 | 说明 |
| --- | --- | --- |
| 心跳周期 | **100 ms（10 Hz）**，最小 200 ms（5 Hz） | 上位机发送 |
| 心跳超时 | **300 ms**（可配 100..2000） | FPGA 侧看门狗默认值 |
| 遥测周期 | 20 ms（50 Hz） | FPGA 发送 |
| SET_VEL 周期 | 50 ms（20 Hz） | 上位机发送 |
| 命令超时 | 500 ms | 超过该时间无 `SET_VEL` → 目标速度归零（不锁存故障） |

> **为什么超时不是 200 ms**：v1.0 初稿写的是"心跳 5 Hz + 超时 200 ms"，两者相等，
> PC 侧任何一点调度抖动都会误判失联（实测台架在 5 Hz 下确实反复触发）。
> 现冻结为 **心跳 10 Hz、超时 300 ms**（150% 周期余量）；若上位机只能 5 Hz，
> 仍有 100 ms 余量可用。

### 4.2 失联保护

```text
心跳超时 (300 ms) → link_lost=1, target_speed=0, 锁存
恢复条件：收到合法 HEARTBEAT 且收到 CLEAR_FAULT
任何情况下：停止(STOP) > 失联(LINK_LOST) > 上位机命令 > 语音命令
上电即 link_lost=1：显式使能原则，第一次连接必须发一条 CLEAR_FAULT 才接受速度命令
```

### 4.3 故障位图（TELEM.fault）

| bit | 名称 | 触发 |
| --- | --- | --- |
| 0 | `STOP_LATCHED` | 收到 STOP 或急停 |
| 1 | `LINK_LOST` | 心跳超时 |
| 2 | `ESTOP` | 硬件急停按钮（若接入） |
| 3 | `OVER_LIMIT` | 命令速度超过 SET_LIMIT 被裁剪 |
| 4 | `CRC_ERR` | 累计 CRC 错误 > 阈值（诊断用，自清） |
| 5 | `CMD_TIMEOUT` | 500 ms 无 SET_VEL（诊断用，自清） |
| 6..7 | 保留 | — |

### 4.4 命令仲裁（与 ASRPRO 语音通道共存时）

```text
优先级（高 → 低）：
  1. 硬件急停 / STOP（锁存）
  2. LINK_LOST（锁存）
  3. 上位机 SET_VEL
  4. ASRPRO 语音命令
仲裁结果只写"目标速度/模式"寄存器，不直接写 PWM；
ASRPRO 通道行为与本协议**互不影响**，任一通道停车都必须先于通信恢复生效。
```

---

## 5. 错误处理与统计

| 情况 | 行为 | 计数器 |
| --- | --- | --- |
| 同步字错（非 AA 55） | 丢弃该字节，重新找同步 | `sync_err` |
| LEN > 32 | 整帧丢弃，回到找同步 | `len_err` |
| CRC 错 | 整帧丢弃，不改变任何状态 | `crc_err` |
| 尾字节非 0D | 整帧丢弃 | `tail_err` |
| 字节间超时（> 5 ms） | 丢弃半帧，回到找同步 | `timeout` |
| PAYLOAD 长度与 TYPE 不符 | 丢弃，`status=0x04` | `type_err` |
| 重复 STOP / 重复 HEARTBEAT | 幂等接受 | — |

> **强制规则**：任何解析错误都**只能丢弃**，绝不允许把半帧内容写入任何寄存器。

---

## 6. 传输绑定 A：UART（当前）

| 项 | 值 |
| --- | --- |
| 电平 | 3.3 V TTL，共地 |
| 参数 | 115200 8N1，无流控 |
| FPGA 引脚 | `uart_rx` = U8（输入，内部上拉），`uart_tx` = V8（输出，8 mA） |
| 适配器 | U8/V8 → USB 串口（CH9102，PC 侧 **COM14**） |
| 半帧超时 | 5 ms（≈ 5.7 个字节时间） |
| 吞吐 | 最大约 11.5 kB/s；50 Hz TELEM（30 B/帧）= 1.5 kB/s，余量充足 |

---

## 7. 传输绑定 B：UDP over 千兆以太网（未来，帧格式不变）

| 项 | 值 |
| --- | --- |
| 链路 | RGMII 1000BASE-T，独立测试工程，不进入 v1 里程碑（PLAN §9.2） |
| 传输 | UDP 单播，默认端口 **50000**（命令）/ **50001**（遥测） |
| 载荷 | **一个 UDP 报文恰好承载一个完整帧**（AA 55 … 0D），不复用字节流解析 |
| MTU | 帧长 7+32=39 B ≪ 1500 B，无分片风险 |
| 丢包 | UDP 不重传：靠 20 Hz SET_VEL / 5 Hz HEARTBEAT 的冗余；上位机按 `cmd_seq` 统计丢包率 |
| 乱序 | 单链路单播，实测不乱序；若出现，按 `seq` 丢弃旧帧 |
| RTL 适配 | 新增 `eth_udp_byte_source`，向帧解析器提供 `(byte, valid)`，**`host_frame_rx` 零改动** |

**迁移检查单**：
1. `protocol.py` 的 `encode/decode` 不动；
2. 只新增/替换 transport 实现（`transport_udp.py`）；
3. RTL 只新增字节源适配器，帧解析/命令分发/遥测打包复用；
4. 安全逻辑（心跳、STOP 锁存、仲裁）**不依赖传输层**，原样保留；
5. 保留 UART 作为安全备份通道（PLAN §9.2）。

---

## 8. 测试矩阵

| # | 用例 | 层级 | 期望 |
| --- | --- | --- | --- |
| T01 | CRC 标准向量 V1/V2 | 单元（RTL+Python） | 与标准值一致 |
| T02 | 合法 SET_VEL 帧解析 | 单元 | type/len/payload 正确，good_cnt++ |
| T03 | CRC 错误帧 | 单元 | 丢弃，状态不变，crc_err++ |
| T04 | 残帧（中途停发 5 ms 以上） | 单元 | 丢弃，timeout++ |
| T05 | LEN 超限 / 尾字节错 | 单元 | 丢弃，对应计数 ++ |
| T06 | 连续多帧背靠背 | 单元 | 全部正确解析 |
| T07 | STOP 锁存 + 后续 SET_VEL | 单元 | 目标速度保持 0，ACK 拒绝 |
| T08 | 重复心跳 / 重复 STOP | 单元 | 幂等 |
| T09 | 心跳超时（默认 300 ms） | 单元 | link_lost=1，目标速度归零 |
| T10 | CLEAR_FAULT 恢复时序 | 单元 | 仅在链路恢复后生效 |
| T11 | 遥测 50 Hz 节拍与内容 | 单元 | 周期 20 ms ±1 ms，字段正确 |
| T12 | 全链路回环（RX→解析→遥测→TX） | 集成 | 上位机发命令 → 收到对应遥测 |
| T13 | 上板：心跳 + 遥测 | 板级 | 50 Hz 遥测稳定，无 CRC 错 |
| T14 | 上板：拔线 300 ms 后恢复 | 板级 | 先停车锁存，CLEAR_FAULT 后恢复 |
| T15 | 上板：长时间（10 min）误码率 | 板级 | CRC 错误率 < 1e-6 |

### 8.1 实现与验证状态（2026-10-29）

| # | 用例 | 实现 | 验证手段 | 结果 |
| --- | --- | --- | --- | --- |
| T01 | CRC 向量 | `crc16_ccitt.v` / `protocol.py` | `tb_core.sv` 4 项 + Python 自检 | ✅ V2=0x29B1 等全部一致 |
| T02–T06 | 帧解析与错误谱 | `host_frame_rx.v` | `tb_core.sv` 8 项 | ✅ CRC/尾/LEN/重同步/超时/逐字节 |
| T07–T11 | 命令/安全/遥测 | `host_cmd_decode.v`、`host_link_watchdog.v`、`telem_builder.v` | `tb_core.sv` 20 项 | ✅ 含 ACK 状态码、锁存、恢复 |
| T12 | 全链路回环 | 全套 | `tb_loopback.sv`（真实 115200） | ✅ ACK + TELEM 内容逐字节校验 |
| T13 | 上板心跳+遥测 | `uart_bridge_top.v` | `uart_link.py bench` / `soak_test.py` | ✅ 50.0 Hz，误码 0 |
| T14 | 上板失联恢复 | 同上 | `bench`：停心跳 299 ms 检出，恢复后 CLEAR_FAULT OK | ✅ |
| T15 | 10 min 长稳 | 同上 | `soak_test.py --duration 600` | 见 `records/` |

---

## 9. 变更记录

| 版本 | 日期 | 变更 |
| --- | --- | --- |
| v1.0 | 2026-10-28 | 首版冻结：帧格式、消息、时序、仲裁、UART 绑定、UDP 迁移约定 |
| v1.1 | 2026-10-29 | ① `SET_VEL` 增加 `seq` 字节（LEN 4→5），用于丢帧统计；② 心跳超时 200→**300 ms**（与 5 Hz 心跳等周期会误判，实测证伪）；③ 补 §8.1 实现与验证状态；④ 明确"上电即 link_lost，需 CLEAR_FAULT 显式使能" |
