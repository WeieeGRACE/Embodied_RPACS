#!/usr/bin/env python3
"""板级健壮性测试：往真实链路里灌坏帧/垃圾，确认 FPGA 不会锁死、不会误执行。

序列：
  1. 心跳 + CLEAR_FAULT 建立链路；
  2. 记录基线 `cmd_seq`；
  3. 连发 N 个 **CRC 错** 的 SET_VEL（seq 递增）→ 期望 `cmd_seq` 完全不变；
  4. 灌 2000 个随机字节（含 AA/55 等同步字）→ 期望遥测不中断、`cmd_seq` 不变；
  5. 再发一个**合法** SET_VEL → 期望被接受（证明解析器恢复正常）；
  6. 全程统计遥测帧率与本地解析错误。

用法::

    python board_robustness_test.py --port COM14 --bad 50 --garbage 2000
"""
from __future__ import annotations

import argparse
import json
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402
from uart_link import Link    # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="board link robustness test")
    ap.add_argument("--port", default="COM14")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bad", type=int, default=50, help="corrupted frames")
    ap.add_argument("--garbage", type=int, default=2000, help="random bytes")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

    checks = []

    def check(name: str, ok: bool, detail: str = "") -> bool:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})
        print("%s %-40s %s" % ("PASS" if ok else "FAIL", name, detail))
        return ok

    tp = T.UartTransport(port=args.port, baud=args.baud)
    link = Link(tp, heartbeat_hz=10.0)
    link.start()
    rnd = random.Random(1234)
    result = {"port": args.port, "started": time.strftime("%Y-%m-%dT%H:%M:%S")}
    try:
        time.sleep(0.5)
        link.heartbeat(1)
        time.sleep(0.15)
        link.clear_fault()
        link.wait_ack(0.6, P.MSG_CLEAR_FAULT)
        tele = link.wait_telemetry(1.0)
        check("link_established", tele is not None,
              "fault=0x%02X" % (tele.fault if tele else 0xFF))
        base_seq = link.telemetry[-1].cmd_seq if link.telemetry else 0

        # ---- 1) corrupted SET_VEL frames must be ignored
        for i in range(args.bad):
            f = bytearray(P.msg_set_vel(200, 200, (i + 1) & 0xFF))
            f[-3] ^= 0xFF                     # break CRC
            link.send(bytes(f))
            time.sleep(0.002)
        time.sleep(0.4)
        seq_after_bad = link.telemetry[-1].cmd_seq if link.telemetry else -1
        check("corrupted_setvel_ignored", seq_after_bad == base_seq,
              "cmd_seq %s -> %s" % (base_seq, seq_after_bad))

        # ---- 2) random garbage must not kill the link
        n_before = len(link.telemetry)
        garbage = bytes(rnd.randrange(256) for _ in range(args.garbage))
        # inject in 64-byte chunks with gaps so the UART stays in sync
        for i in range(0, len(garbage), 64):
            link.send(garbage[i:i + 64])
            time.sleep(0.01)
        t0 = time.time()
        time.sleep(1.5)
        gained = len(link.telemetry) - n_before
        rate = gained / (time.time() - t0)
        check("telemetry_survives_garbage", rate > 30.0, "%.1f Hz" % rate)
        seq_after_garbage = link.telemetry[-1].cmd_seq if link.telemetry else -1
        check("garbage_does_not_execute", seq_after_garbage == base_seq,
              "cmd_seq=%s" % seq_after_garbage)

        # ---- 3) a valid frame right after the noise must be accepted
        new_seq = (base_seq + 7) & 0xFF
        link.set_vel(90, 90, new_seq)
        tele = link.wait_telemetry(1.0, lambda t: t.cmd_seq == new_seq)
        check("valid_frame_accepted_after_noise", tele is not None,
              "cmd_seq %s" % (tele.cmd_seq if tele else "-"))

        # ---- 4) link still healthy
        check("link_still_ok", (link.telemetry[-1].fault & P.FAULT_LINK_LOST) == 0,
              "fault=0x%02X" % link.telemetry[-1].fault)

        result["checks"] = checks
        result["host_parser_stats"] = link.parser.stats.as_dict()
        result["frames_seen"] = dict(link.frames_seen)
        result["telem_rate_hz"] = round(link.telem_rate(1.0), 2)
    finally:
        link.stop()

    passed = sum(1 for c in checks if c["ok"])
    result["passed"] = passed
    result["failed"] = len(checks) - passed
    result["ok"] = passed == len(checks)
    result["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    print("\n%d/%d checks passed" % (passed, len(checks)))
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        print("report ->", args.out)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
