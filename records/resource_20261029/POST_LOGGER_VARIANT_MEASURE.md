# 验收后“删除 USB 事件日志 + 释放 V8/U8”预研实测（2026-10-29 19:07）

目的：在**不动主工程交付位流**的前提下，用临时变体工程预研“上板验收通过后删除
`system_event_logger`、移除调试 UART 并释放 V8/U8 引脚”这一改动的真实资源收益，
避免按审计估算（0.42–0.45k LUT）做错误预期。

## 方法

- 复制 `src/robot_system_top.v`：删除 `system_event_logger` 实例；顶层去掉 `uart_tx` 输出与
  `uart_rx` 输入（`system_motion` 内部的 ASR `uart_tx` 保留，D18 不受影响）。
- 复制 `src/robot_system_top.cst`：删除 `uart_tx=V8`、`uart_rx=U8` 两条约束。
- 其余 RTL 全部直接引用主工程 `src/`；与主工程 `build.tcl` 相同的 34 个文件、相同选项、`run all`。
- 临时工程位于 `.tmp_nolog/`（git 忽略），不参与交付；主工程源码与交付位流未改动。

## 结果（`build.tcl`，19:07:46 完成，0 error）

| 资源 | 主工程现行 | 预研变体 | 变化 |
|---|---|---|---|
| Logic | 13420/23040 (59%) | **13364/23040 (58%)** | −56 |
| CLS | 8304/11520 (73%) | **8197/11520 (72%)** | −107 |
| Register | 5690 | 5497 | −193 |
| BSRAM | 9/56 | 5/56 | **−4** |
| DSP | 6.5/28 | 6.5/28 | 0 |
| Fmax | 58.287 MHz | **61.113 MHz** | +2.826 MHz |
| Setup/Hold 违例、TNS | 0 / 0 | 0 / 0 | 保持 |
| 受约束 IOB | 104 | 102 | **−2（V8/U8）** |

- 临时变体位流（不交付）：`.tmp_nolog/impl/pnr/tmp_nolog.fs`，6,228,300 字节，
  SHA256 `38A6C449A9DEFF398D8FD33AABEEEB072E37D563F3A0005056BA369691B8FD6F`。
- 报告：`.tmp_nolog/impl/pnr/tmp_nolog.rpt.txt` / `tmp_nolog_tr_content.html`（本地临时目录）。

## 结论

审计估算“−0.42–0.45k LUT”偏高：事件日志的主体存储在 4 个 BSRAM 里，逻辑部分只有几十 LUT。
真实收益是 **4 个 BSRAM、193 个 FF、107 CLS、Fmax 放宽 ~2.8 MHz**，外加 **释放 V8/U8 两个
FPGA 球**（受约束引脚 44→42）。正式执行时机：上板 3–5 次连续避障验收通过后，见
`PROJECT_PLAN.md` §16.4 与 `records/robustness_20261029/FINAL_ACCEPTANCE.md` §4。
