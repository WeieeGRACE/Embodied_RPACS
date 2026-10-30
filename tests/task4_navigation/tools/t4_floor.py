#!/usr/bin/env python3
"""任务四 落地测试「一条命令」：采集 → 判读 → 出报告。

典型用法（上板时）::

    # 1) 先确认要烧的位流是对的（可选但推荐）
    python3 tools/t4_floor.py --bitstream impl/pnr/task4_navigation_board.fs --check-only

    # 2) 采集 + 判读（60 s 窗口，按提示按启动键）
    python3 tools/t4_floor.py --seconds 60

    # 3) 已经抓到日志，只想重新判读
    python3 tools/t4_floor.py --replay records/floor_20261030_1100.log

产物（默认 records/）：
    floor_<ts>.log / .jsonl / .json / .md
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve()
TOOLS = HERE.parent
ROOT = TOOLS.parent          # tests/task4_navigation
sys.path.insert(0, str(TOOLS))

import t4_log as L           # noqa: E402
import t4_validate as V      # noqa: E402

#: 已验收的探针位流（USB 串口版 / 无线串口版），两个都可接受
EXPECTED_PROBE_SHAS = {
    "F938C41792B9DA32382BA7C9FE321B8BE08DFA45636F1B6E6AC2B1DE70AFC392": "probe-usb(V8)",
    "C2FA191BB19F224EAAE0A525A3FD092CA304BA3E753DC5553D00A0F83DE6D44F": "probe-wireless(L12)",
    "0F1A6B0A1A15AC36A1B0B00B1214337B3476A5CC40D814728D616B1A495BC20C": "probe-wireless-calibrated",
    "CA199023BBEFF841EDF8B19175F7E42F18E289BFB69FAD546BE702267ED01222": "probe-usb-calibrated",
    "BC995764B4C5C72052FF9F0884340612AFBADBF6298FB26F1A0296CAC64A46E3": "distance-mode-wireless",
}


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def write_markdown(path: Path, res: dict, stats: dict, log_path: Path) -> None:
    lines = []
    lines.append("# 任务四地面段测试报告\n")
    lines.append("| 项 | 值 |")
    lines.append("| --- | --- |")
    lines.append("| 时间 | %s |" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("| 记录文件 | `%s` |" % log_path.name)
    lines.append("| 解析 | 记录 %d 条 / 输入 %d 字节 / 垃圾 %d / 残留 %d |"
                 % (stats["records"], stats["bytes_in"],
                    stats["garbage_bytes"], stats["partial_bytes"]))
    lines.append("| 判定 | **%s** |" % res["verdict"])
    lines.append("")
    if res["records"]:
        lines.append("## 段记录\n")
        lines.append("| SEG | DIST(mm) | YAW(°) | WALL | FAULT |")
        lines.append("| --- | --- | --- | --- | --- |")
        for r in res["records"]:
            lines.append("| %d | %d | %.3f | %s | %s |"
                         % (r["segment"], r["distance_mm"], r["yaw_deg"],
                            r["wall_name"], r["fault_name"]))
        lines.append("")
    lines.append("## 逐项检查\n")
    lines.append("| 检查 | 结果 | 详情 |")
    lines.append("| --- | --- | --- |")
    for c in res["checks"]:
        lines.append("| %s | %s | %s |" % (c["name"],
                                           "PASS" if c["ok"] else "FAIL",
                                           c["detail"]))
    lines.append("")
    if res["verdict"] == "PASS":
        lines.append("**结论**：该段正常完成——墙段命中、里程在确认区间、航向在容差内、无故障。")
    elif res["verdict"] == "SAFE_STOP":
        lines.append("**结论**：安全停车（有故障码或未命中墙段），未超过硬停里程；"
                     "请按 FIELD_TEST_GUIDE 排查后重测。")
    else:
        lines.append("**结论**：不通过，详见上表；保留原始日志 `%s`。" % log_path.name)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="task4 floor test: capture + validate + report")
    ap.add_argument("--seconds", type=float, default=60.0, help="采集窗口（秒）")
    ap.add_argument("--port", default=None, help="串口；默认自动探测")
    ap.add_argument("--replay", default=None, help="不采集，直接判读已有日志")
    ap.add_argument("--bitstream", default=None, help="可选：检查要烧的位流 SHA256")
    ap.add_argument("--check-only", action="store_true", help="只查位流，不采集")
    ap.add_argument("--expect-safe-stop", action="store_true", help="安全停车也算通过")
    ap.add_argument("--last-only", action="store_true", help="只判读最后一条记录")
    ap.add_argument("--profile", choices=["wall", "distance"], default="wall",
                    help="wall=要求 ToF 墙事件；distance=里程到达即成功")
    ap.add_argument("--spec", default=None, help="期望段序列 JSON（默认 route6 单段探针）")
    ap.add_argument("--out-dir", default=str(ROOT / "records"))
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---------------- 位流检查
    if args.bitstream:
        bs = Path(args.bitstream)
        if not bs.exists():
            print("位流不存在：%s" % bs, file=sys.stderr)
            return 1
        got = sha256_file(bs)
        name = EXPECTED_PROBE_SHAS.get(got)
        ok = name is not None
        print("[bitstream] %s\n  SHA256=%s\n  -> %s"
              % (bs, got, (name if ok else "MISMATCH（不是已验收的探针位流）")))
        if not ok:
            print("!! 不是已验收的单段探针位流，请勿用于地面测试", file=sys.stderr)
            return 1
    if args.check_only:
        return 0

    # ---------------- 采集
    if args.replay:
        log_path = Path(args.replay)
        if not log_path.exists():
            print("日志不存在：%s" % log_path, file=sys.stderr)
            return 1
    else:
        stamp = time.strftime("%Y%m%d_%H%M")
        log_path = out_dir / ("floor_%s.log" % stamp)
        cmd = [sys.executable, str(TOOLS / "t4_capture.py"),
               "--seconds", str(args.seconds), "--out", str(log_path)]
        if args.port:
            cmd += ["--port", args.port]
        print("[floor] 开始采集 %.0f s：现在上电/复位，等 led_ready 后按启动键（key_enable_n）"
              % args.seconds)
        print("[floor] 按 Ctrl-C 可提前结束")
        try:
            rc = subprocess.call(cmd)
        except KeyboardInterrupt:
            print("\n[floor] 采集被中断，继续用已抓到的数据判读")
            rc = 0
        if rc not in (0, 130):
            print("采集失败 rc=%d" % rc, file=sys.stderr)
            return 1

    # ---------------- 判读 + 报告
    recs, stats = L.records_from_file(str(log_path))
    spec = V.load_spec(args.spec, args.profile)
    res = V.validate(recs, spec, args.expect_safe_stop, args.last_only)
    res["log"] = str(log_path)
    res["parse"] = stats.as_dict()
    res["spec"] = spec.get("name", "custom")

    json_path = log_path.with_suffix(".json")
    md_path = log_path.with_suffix(".md")
    json_path.write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
    write_markdown(md_path, res, stats.as_dict(), log_path)

    print("")
    print("[floor] 判定：%s%s"
          % (res["verdict"], "" if res.get("accepted") else "（未通过）"))
    if res["records"]:
        for r in res["records"]:
            print("         %s" % L.SegmentRecord(**{k: r[k] for k in
                  ("segment", "distance_mm", "yaw_mdeg", "wall", "fault", "ts")}))
    print("[floor] 报告：%s\n[floor] JSON：%s" % (md_path, json_path))
    if res.get("accepted"):
        return 0
    return 2 if res["verdict"] == "SAFE_STOP" else 1


if __name__ == "__main__":
    raise SystemExit(main())
