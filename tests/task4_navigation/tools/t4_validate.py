#!/usr/bin/env python3
"""任务四 段记录自动判读。

用法::

    python3 t4_validate.py --log floor.log                     # 默认单段探针（route6/seg0）
    python3 t4_validate.py --log floor.log --json out.json
    python3 t4_validate.py --log floor.log --expect-safe-stop   # 允许"安全停车"结果也算通过
    python3 t4_validate.py --log floor.log --spec route6_spec.json

判定级别：
    PASS       段正常确认（FAULT=0、WALL 命中、DIST 在确认区间、YAW 在容差内）
    SAFE_STOP  安全停车（有故障码或墙未命中，但 DIST 未超过硬停里程）——是否算通过由 --expect-safe-stop 决定
    FAIL       无记录/多条记录/距离越界/未知故障码
退出码：0 PASS，2 SAFE_STOP，1 FAIL
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))

import t4_log as L  # noqa: E402


def load_spec(path: str | None) -> dict:
    if path:
        with open(path, "r", encoding="utf-8") as f:
            spec = json.load(f)
        return spec
    # 默认：route6/seg0 单段探针（与 FIELD_TEST_GUIDE.md 一致）
    return {
        "name": "route6_probe",
        "segments": [
            {"segment": 0, "wall": 10, "dist_min": 400, "dist_max": 430,
             "dist_hard_stop_max": 520, "yaw_tol_mdeg": 5000, "final": True},
        ],
    }


def validate(records: list, spec: dict, expect_safe_stop: bool = False) -> dict:
    segs = spec["segments"]
    checks: list[dict] = []

    def add(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    if not records:
        add("record_present", False, "没有解析到任何段记录")
        return {"verdict": "FAIL", "accepted": False, "checks": checks, "records": []}
    add("record_present", True, "%d 条记录" % len(records))

    # 只应有一条（单段）；多条说明重复触发/复位重跑，需要人工确认
    expect_multi = len(segs) > 1
    if not expect_multi:
        add("single_record", len(records) == 1, "记录数=%d" % len(records))
    else:
        add("record_count", len(records) == len(segs),
            "记录数=%d 期望=%d" % (len(records), len(segs)))

    # 逐条与 spec 对齐
    normal_all = True
    for idx, rec in enumerate(records):
        want = segs[idx] if idx < len(segs) else None
        if want is None:
            add("extra_record_%d" % idx, False, str(rec))
            normal_all = False
            continue
        prefix = "seg%d" % idx
        add(prefix + "_segment_id", rec.segment == want["segment"],
            "SEG=%d 期望=%d" % (rec.segment, want["segment"]))
        if rec.fault == L.F_NONE:
            add(prefix + "_fault", True, "FAULT=NONE")
            add(prefix + "_wall", rec.wall == want["wall"],
                "WALL=%s 期望 %s" % (rec.wall_name, L.WALL_NAMES.get(want["wall"], want["wall"])))
            in_range = want["dist_min"] <= rec.distance_mm <= want["dist_max"]
            add(prefix + "_distance_confirmed", in_range,
                "DIST=%dmm 期望 [%d, %d]" % (rec.distance_mm, want["dist_min"], want["dist_max"]))
            add(prefix + "_yaw", abs(rec.yaw_mdeg) <= want.get("yaw_tol_mdeg", 5000),
                "YAW=%.3f° 容差 ±%.1f°"
                % (rec.yaw_deg, want.get("yaw_tol_mdeg", 5000) / 1000.0))
        else:
            # 非正常结束：安全停车（故障码合法、距离未超硬停）
            normal_all = False
            known = rec.fault in L.FAULT_NAMES
            add(prefix + "_fault_known", known,
                "FAULT=%s" % rec.fault_name)
            add(prefix + "_stopped_within_hard_stop",
                rec.distance_mm <= want["dist_hard_stop_max"],
                "DIST=%dmm ≤ 硬停 %dmm" % (rec.distance_mm, want["dist_hard_stop_max"]))

    ok_all = all(c["ok"] for c in checks)
    if not ok_all:
        verdict = "FAIL"
    elif normal_all:
        verdict = "PASS"
    else:
        verdict = "SAFE_STOP"
    accepted = verdict == "PASS" or (verdict == "SAFE_STOP" and expect_safe_stop)
    return {"verdict": verdict, "accepted": accepted, "checks": checks,
            "records": [r.to_dict() for r in records]}


def main() -> int:
    ap = argparse.ArgumentParser(description="validate task4 segment log")
    ap.add_argument("--log", required=True, help="原始串口抓包（二进制/文本均可）")
    ap.add_argument("--spec", default=None, help="期望的段序列 JSON（默认 route6 单段探针）")
    ap.add_argument("--expect-safe-stop", action="store_true",
                    help="允许安全停车（SAFE_STOP）算通过")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    recs, stats = L.records_from_file(args.log)
    spec = load_spec(args.spec)
    res = validate(recs, spec, args.expect_safe_stop)
    res["spec"] = spec.get("name", "custom")
    res["parse"] = stats.as_dict()
    res["log"] = args.log

    print("== 任务四段记录判读：%s ==" % spec.get("name", "custom"))
    print("解析：记录 %d 条，输入 %d 字节，垃圾 %d，残留 %d"
          % (stats.records, stats.bytes_in, stats.garbage_bytes, stats.partial_bytes))
    for r in recs:
        print("  ", r)
    print("")
    for c in res["checks"]:
        print("%s %-34s %s" % ("PASS" if c["ok"] else "FAIL", c["name"], c["detail"]))
    print("\nVERDICT:", res["verdict"],
          "(accepted)" if res["accepted"] else "(not accepted)")
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2, ensure_ascii=False)
        print("-> %s" % args.json)
    if res["accepted"]:
        return 0
    return 2 if res["verdict"] == "SAFE_STOP" else 1


if __name__ == "__main__":
    raise SystemExit(main())
