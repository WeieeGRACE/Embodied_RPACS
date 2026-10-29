#!/usr/bin/env python3
"""链路协议 v2（以太网/UDP）验收测试。

对端：`tests/eth_link_protocol` 的位流（GW5A-25A）
    FPGA: 192.168.0.2:5000   收命令、发遥测/ACK（目的 MAC 广播，回包给 192.168.0.1:5001）
    Pi  : 192.168.0.1:5001

复用 `tests/uart_bridge/tools` 的协议库与传输层（同一套帧格式，只换传输）：
    protocol.py / transport.UdpTransport / uart_link.Link

用法::

    python3 eth_link_test.py                       # 默认 3 s 预检 + 命令测试
    python3 eth_link_test.py --duration 10 --json out.json
    python3 eth_link_test.py --only-preflight
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve()


def _find_uart_tools() -> Path:
    """按优先级找协议库：环境变量 > 仓库布局 > 板上 ~/uart_tools。"""
    env = os.environ.get("UART_BRIDGE_TOOLS")
    if env:
        return Path(env)
    candidates = [
        HERE.parents[2] / "uart_bridge" / "tools",   # 仓库内 tests/uart_bridge/tools
        HERE.parent / "uart_tools",                  # 与脚本同目录的 uart_tools
        Path.home() / "uart_tools",                  # 板上部署位置
        Path.cwd() / "uart_tools",
    ]
    for c in candidates:
        try:
            if (c / "protocol.py").is_file():
                return c
        except OSError:
            continue
    return candidates[0]


UART_TOOLS = _find_uart_tools()
if str(UART_TOOLS) not in sys.path:
    sys.path.insert(0, str(UART_TOOLS))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402

ACCEPT = {"telem_hz_min": 40.0, "telem_hz_max": 60.0,
          "max_error_frames": 0, "max_hb_age_ms": 150}


def run(host: str, port: int, local_port: int, duration: float,
        do_commands: bool) -> dict:
    checks: list[dict] = []

    def check(name: str, ok: bool, detail: str = "") -> bool:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})
        print("%s %-34s %s" % ("PASS" if ok else "FAIL", name, detail))
        return ok

    # ---- P1: 建 socket
    tp = T.UdpTransport(remote=(host, port), local=("0.0.0.0", local_port))
    check("P1_udp_socket", True, "local :%d -> %s:%d" % (local_port, host, port))

    link = Link(tp, heartbeat_hz=10.0)
    link.start()
    result: dict = {"target": "%s:%d" % (host, port), "local_port": local_port}

    # 预热 0.5 s（丢掉首包/ARP 等待等启动噪声）
    time.sleep(0.5)
    base = link.parser.stats.as_dict()
    n_base = len(link.telemetry)
    t0 = time.time()
    try:
        # ---- P2/P3: 收到遥测
        while time.time() - t0 < duration:
            time.sleep(0.01)
        frames = list(link.telemetry)[n_base:]
        t_times = [tt for tt in link.telemetry_times][n_base:]
        check("P2_telemetry_received", len(frames) > 0,
              "n=%d / %.1fs" % (len(frames), duration))

        span = (t_times[-1] - t_times[0]) if len(t_times) > 1 else 0.0
        hz = (len(frames) - 1) / span if span > 0 else 0.0
        check("P3_rate_about_50hz",
              ACCEPT["telem_hz_min"] <= hz <= ACCEPT["telem_hz_max"],
              "%.2f Hz" % hz)

        # ---- P4: 解析错误（预热后的增量）
        cur = link.parser.stats.as_dict()
        errs = sum(cur[k] - base[k] for k in ("crc_err", "tail_err", "len_err"))
        syncs = cur["sync_err"] - base["sync_err"]
        check("P4_zero_parse_errors", errs <= ACCEPT["max_error_frames"],
              "crc/tail/len=%d, sync=%d" % (errs, syncs))

        # ---- P5: 心跳年龄
        last = frames[-1] if frames else None
        check("P5_heartbeat_fresh",
              last is not None and last.hb_age_ms <= ACCEPT["max_hb_age_ms"],
              "hb_age=%d ms" % (last.hb_age_ms if last else -1))

        # ---- P6: 故障字节可读
        check("P6_fault_byte_readable", last is not None,
              "fault=0x%02X%s" % (last.fault if last else 0xFF,
                                  ("[" + ",".join(last.fault_names) + "]")
                                  if last and last.fault_names else ""))

        # ---- 命令通路：CLEAR_FAULT -> SET_VEL -> STOP
        if do_commands:
            link.clear_fault()
            ack = link.wait_ack(1.0, P.MSG_CLEAR_FAULT)
            check("C1_clear_fault_ack", ack is not None and ack.status == P.ACK_OK,
                  ack.status_name if ack else "no ACK")

            seq = (int(time.time()) & 0x7F) + 1
            link.set_vel(100, 100, seq)
            ack = link.wait_ack(1.0, P.MSG_SET_VEL)
            check("C2_setvel_ack_ok",
                  ack is not None and ack.status == P.ACK_OK,
                  ack.status_name if ack else "no ACK")
            check("C3_setvel_seq_echo", ack is not None and ack.seq == seq,
                  "seq=%s (期望 %d)" % (ack.seq if ack else "-", seq))
            tele = link.wait_telemetry(1.0, lambda t: t.cmd_seq == seq)
            check("C4_telem_echoes_cmd_seq", tele is not None,
                  "cmd_seq=%s" % (tele.cmd_seq if tele else "-"))

            link.stop_cmd()
            ack = link.wait_ack(1.0, P.MSG_STOP)
            check("C5_stop_ack", ack is not None and ack.status == P.ACK_OK,
                  ack.status_name if ack else "no ACK")
            tele = link.wait_telemetry(1.0,
                                       lambda t: bool(t.fault & P.FAULT_STOP_LATCHED))
            check("C6_stop_fault_bit", tele is not None,
                  "fault=0x%02X" % (tele.fault if tele else 0xFF))
            # 解锁回去，别把板子留在停车状态
            link.clear_fault()
            link.wait_ack(0.8, P.MSG_CLEAR_FAULT)

        result["frames"] = len(frames)
        result["rate_hz"] = round(hz, 2)
        result["parser"] = {k: cur[k] - base[k] for k in cur}
        result["last"] = ({"enc": [last.enc_l, last.enc_r], "yaw_deg": last.yaw_deg,
                           "pitch_deg": last.pitch_deg, "roll_deg": last.roll_deg,
                           "fault": last.fault, "hb_age_ms": last.hb_age_ms}
                          if last else None)
    finally:
        link.stop()

    passed = sum(1 for c in checks if c["ok"])
    result.update({"checks": checks, "passed": passed, "total": len(checks),
                   "ok": passed == len(checks)})
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description="link v2 over UDP acceptance test")
    ap.add_argument("--host", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--local-port", type=int, default=5001)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--only-preflight", action="store_true")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    print("[eth] UDP 链路测试 -> %s:%d（本机 :%d）"
          % (args.host, args.port, args.local_port))
    print("提示：Pi 侧需先配好 eth0=192.168.0.1/24 与静态 ARP（见 README）")
    res = run(args.host, args.port, args.local_port, args.duration,
              not args.only_preflight)
    print("\n%d/%d checks passed" % (res["passed"], res["total"]))
    print("ETH LINK %s" % ("PASS" if res["ok"] else "FAIL"))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
