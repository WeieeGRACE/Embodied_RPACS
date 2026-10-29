#!/usr/bin/env python3
"""把 preflight / soak / link_client 的 JSON 汇总成 Markdown 报告。

用法::

    python3 report.py records/preflight.json records/soak_cam_link.json \
            --out records/report.md --title "深度相机 ↔ FPGA 链路联调"
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def load(paths: list[str]) -> list[tuple[str, dict]]:
    out = []
    for p in paths:
        try:
            with open(p, "r", encoding="utf-8") as f:
                out.append((Path(p).name, json.load(f)))
        except Exception as exc:  # noqa: BLE001
            out.append((Path(p).name, {"_error": str(exc)}))
    return out


def render(items: list[tuple[str, dict]], title: str) -> str:
    lines = ["# %s" % title, "", "- 生成时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"),
             "- 数据来源：%d 个 JSON" % len(items), ""]

    for name, d in items:
        lines.append("## %s" % name)
        lines.append("")
        if "_error" in d:
            lines += ["读取失败：`%s`" % d["_error"], ""]
            continue

        if "checks" in d and isinstance(d["checks"], list):      # preflight 格式
            port = d.get("port", "?")
            lines += ["- 端口：`%s`" % port, "", "| 检查 | 结果 | 详情 |", "| --- | --- | --- |"]
            for c in d["checks"]:
                lines.append("| %s | %s | %s |"
                             % (c["name"], "✅" if c["ok"] else "❌", c.get("detail", "")))
            lines += ["", "**结论：%s**" % ("PASS" if d.get("ok") else "FAIL"), ""]
            if d.get("last_sample"):
                s = d["last_sample"]
                lines += ["末帧快照：enc=%s yaw=%.2f° pitch=%.2f° roll=%.2f° fault=0x%02X%s"
                          % (s.get("enc"), s.get("yaw_deg", 0), s.get("pitch_deg", 0),
                             s.get("roll_deg", 0), s.get("fault", 0),
                             ("[" + ",".join(s.get("fault_names", [])) + "]")
                             if s.get("fault_names") else ""), ""]

        elif "link" in d and "camera" in d:                      # soak 格式
            lk = d.get("link", {})
            t = lk.get("telem", {}) or {}
            cam = d.get("camera", {})
            cpu = d.get("cpu", {})
            parser = lk.get("parser", {})
            lines += ["| 项 | 值 |", "| --- | --- |",
                      "| 链路遥测 | n=%s, 平均 %s Hz, 最低 %s Hz |"
                      % (t.get("n"), t.get("hz_mean"), t.get("hz_min")),
                      "| 相机 | mode=%s, %s 帧, 平均 %s fps |"
                      % (cam.get("mode"), cam.get("frames"), cam.get("fps_mean")),
                      "| CPU | 平均 %s%%, 峰值 %s%% |"
                      % (cpu.get("mean_percent", "-"), cpu.get("max_percent", "-")),
                      "| 解析统计 | %s |" % (parser,),
                      "| 端口 | `%s` @ %s |" % (d.get("port", "?"), d.get("baud", "?")), ""]
            lines += ["| 判定项 | 结果 |", "| --- | --- |"]
            for k, v in d.get("checks", {}).items():
                lines.append("| %s | %s |" % (k, "✅" if v else "❌"))
            lines += ["", "**结论：%s**" % ("PASS" if d.get("ok") else "FAIL"), ""]

        elif "telem" in d:                                        # link_client 格式
            t = d["telem"]
            lines += ["| 项 | 值 |", "| --- | --- |",
                      "| 遥测帧数 | %s |" % t.get("n"),
                      "| 速率 | 平均 %s Hz（%s~%s）|"
                      % (t.get("hz_mean"), t.get("hz_min"), t.get("hz_max")),
                      "| 解析统计 | %s |" % d.get("parser"),
                      "| 故障位分布 | %s |" % d.get("fault_histogram"), ""]
            if d.get("last_sample"):
                s = d["last_sample"]
                lines += ["末帧：enc=%s yaw=%.2f° pitch=%.2f° roll=%.2f° hb_age=%sms"
                          % (s.get("enc"), s.get("yaw_deg", 0), s.get("pitch_deg", 0),
                             s.get("roll_deg", 0), s.get("hb_age_ms")), ""]
        lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="render markdown report from JSON results")
    ap.add_argument("json_files", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="深度相机 ↔ FPGA 链路联调报告")
    args = ap.parse_args()
    md = render(load(args.json_files), args.title)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(md, encoding="utf-8")
    print("-> %s (%d bytes)" % (out, len(md.encode("utf-8"))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
