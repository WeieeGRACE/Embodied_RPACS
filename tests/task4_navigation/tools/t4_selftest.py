#!/usr/bin/env python3
"""落地测试工具链自检（不需要板子）：用合成/回放日志验证解析 + 判读逻辑。

    python3 tools/t4_selftest.py
    python3 tools/t4_selftest.py --closed-loop records/closed_loop_route6.csv

覆盖场景：
  S1  正常确认 415 mm / WALL=10 / FAULT=0                 -> PASS
  S2  正常但航向偏 -1.2°                                  -> PASS
  S3  航向偏 12°（超容差）                                -> FAIL
  S4  墙段不匹配（WALL=5）                                -> FAIL
  S5  距离过短 300 mm                                     -> FAIL
  S6  未确认墙硬停 519 mm / FAULT=STOP                    -> SAFE_STOP
  S7  超过硬停 600 mm / FAULT=STOP                        -> FAIL
  S8  未知故障码 0x77                                     -> FAIL
  S9  完全没有记录                                        -> FAIL
  S10 分块 + 前后乱码                                     -> PASS（解析健壮）
  S11 两条记录（重复触发）                                -> FAIL
  S12 回放 RTL 闭环仿真 CSV 推导出的记录                  -> PASS（仿真↔工具一致）
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
TOOLS = HERE.parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import t4_log as L           # noqa: E402
import t4_validate as V      # noqa: E402

FAIL = object()


def rec(seg, dist, yaw, wall, fault) -> bytes:
    return L.build_record(seg, dist, yaw, wall, fault)


def run_case(name: str, chunks: list[bytes], expect: str,
             expect_safe_stop: bool = False,
             expect_accepted: bool | None = None) -> tuple[bool, str]:
    p = L.SegmentLogParser()
    recs = []
    for c in chunks:
        recs.extend(p.feed(c))
    p.flush()
    res = V.validate(recs, V.load_spec(None), expect_safe_stop)
    ok = res["verdict"] == expect
    if expect_accepted is not None:
        ok = ok and res["accepted"] == expect_accepted
    detail = "verdict=%s accepted=%s 期望=%s" % (res["verdict"], res["accepted"], expect)
    return ok, detail


def closed_loop_replay(path: Path) -> tuple[bool, str]:
    """从 RTL 闭环仿真 CSV 提取最后状态，合成一条记录再判读。"""
    if not path.exists():
        return False, "找不到 %s" % path
    last = None
    with open(path, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            last = row
    if last is None:
        return False, "CSV 为空"
    seg = 0
    dist = int(round(float(last["segment_mm"])))
    yaw_mdeg = int(round(float(last["true_yaw_deg"]) * 1000))
    wall = int(last["event_id"])
    fault = 0
    data = rec(seg, dist, yaw_mdeg, wall, fault)
    p = L.SegmentLogParser()
    recs = p.feed(data)
    res = V.validate(recs, V.load_spec(None), False)
    ok = res["verdict"] == "PASS"
    detail = ("由 CSV 合成 SEG=%d DIST=%d YAW=%+.3f° WALL=%d FAULT=%d -> verdict=%s"
              % (seg, dist, yaw_mdeg / 1000.0, wall, fault, res["verdict"]))
    return ok, detail


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--closed-loop", default=str(ROOT / "records" / "closed_loop_route6.csv"))
    args = ap.parse_args()

    good = rec(0, 415, 0, 10, 0)
    cases = [
        ("S1_normal_415mm", [good], "PASS", False),
        ("S2_yaw_minus_1.2deg", [rec(0, 412, -1200, 10, 0)], "PASS", False),
        ("S3_yaw_12deg", [rec(0, 412, 12000, 10, 0)], "FAIL", False),
        ("S4_wall_mismatch", [rec(0, 415, 0, 5, 0)], "FAIL", False),
        ("S5_dist_too_short", [rec(0, 300, 0, 10, 0)], "FAIL", False),
        ("S6_hard_stop_519_stop", [rec(0, 519, 0, 0, L.F_STOP)], "SAFE_STOP", False),
        ("S7_overrun_600_stop", [rec(0, 600, 0, 10, L.F_STOP)], "FAIL", False),
        ("S8_unknown_fault", [rec(0, 400, 0, 10, 0x77)], "FAIL", False),
        ("S9_no_record", [b"hello world\r\n"], "FAIL", False),
        ("S10_split_with_garbage",
         [b"\x00\xff junk " + good[:20], good[20:41], good[41:] + b"\r\n\x11"], "PASS", False),
        ("S11_two_records", [good + good], "FAIL", False),
    ]

    print("== 落地工具链自检 ==")
    fails = 0
    for name, chunks, expect, expect_safe in cases:
        ok, detail = run_case(name, chunks, expect, expect_safe)
        print("%s %-28s %s" % ("PASS" if ok else "FAIL", name, detail))
        fails += 0 if ok else 1

    # 安全停车在“允许”模式下的判定
    ok, detail = run_case("S6b_hard_stop_allowed", [rec(0, 519, 0, 0, L.F_STOP)],
                          "SAFE_STOP", expect_safe_stop=True, expect_accepted=True)
    print("%s %-28s %s" % ("PASS" if ok else "FAIL", "S6b_hard_stop_allowed", detail))
    fails += 0 if ok else 1

    # 闭环仿真回放
    ok, detail = closed_loop_replay(Path(args.closed_loop))
    print("%s %-28s %s" % ("PASS" if ok else "FAIL", "S12_closed_loop_replay", detail))
    fails += 0 if ok else 1

    print("")
    if fails == 0:
        print("PASS t4_selftest (13 scenarios)")
        return 0
    print("FAIL t4_selftest (%d scenarios failed)" % fails)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
