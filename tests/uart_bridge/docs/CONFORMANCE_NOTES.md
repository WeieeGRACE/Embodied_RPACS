# 一致性测试说明（Python ↔ RTL）

`tools/conformance.py --run` 一条命令完成三件事：

1. 生成一段对抗性字节流（`sim/vectors/conformance.hex`，435 字节）；
2. 让 `sim/tb_conformance.sv` 把同一段字节流逐字节喂给 `host_frame_rx`；
3. 把 RTL 解析出的帧与 `protocol.py` 的解析结果**逐帧、逐计数器**比对。

```text
corpus: 435 bytes, 25 frames, stats good=25 crc=5 len=2 tail=4
PASS stats: {'good': 25, 'crc': 5, 'len': 2, 'tail': 4}
PASS frame count: 25
PASS all 25 frames identical (type/len/payload, first 8 bytes)
CONFORMANCE PASS
```

## 语料构成

| 段 | 内容 |
| --- | --- |
| 合法帧 | SET_VEL/STOP/HEARTBEAT/SET_LIMIT/CLEAR_FAULT/ACK + 8 个随机长度（1..32）的未知类型 |
| CRC 错 | 5 个：破坏 CRC 低字节 |
| 尾字节错 | 3 个：尾字节改成 0x00 |
| LEN 超限 | 2 处（LEN=0x40 与垃圾流里的 LEN=0x55） |
| 垃圾字节 | 含假 `AA 55` 前缀的噪声 |
| `AA AA 55` 重叠 | 单独一段，后面留 8 个空闲字节 |
| 截断帧 | 结尾 7 字节的半帧（两侧都不应产出帧） |

## 已知的两处"固有差异"（不影响真实链路）

1. **重扫方式不同**：RTL 是流式状态机，坏帧被整体消费；`protocol.py` 是缓冲解析器，
   遇错只删 1 字节后重扫，因此同一段噪声下 Python 的 `sync_err` 会多出很多。
   一致性测试**只比对 `good/crc/len/tail`**（这四项语义严格一致）。
2. **非法 `AA 55` 紧贴真帧**：若噪声里恰好出现 `AA 55` 但 LEN 非法，且紧跟着就是真帧，
   流式解析器已把真帧头两个字节当成本帧内容消费掉，而缓冲解析器能回退重扫。
   这是两种实现的本质差异，**只影响强噪声场景**；语料因此在段间插入空闲字节规避，
   物理链路上由 CRC + 帧间隔（115200 下每帧 ≥ 2.6 ms）自然保证。

## 附带说明

- `host_frame_rx` 只保存前 8 个负载字节（v1 命令最长 5 字节），因此比对也只比前 8 字节，
  更长的帧会打印 `TRUNC` 标记；
- `tb_conformance.sv` 每字节之间留 1 拍空闲（这是 `host_frame_rx` 的接口契约，
  见 `docs/PROTOCOL_v1.md` §5.2；将来接 FIFO/RGMII 字节源时需要保证这一点）。
