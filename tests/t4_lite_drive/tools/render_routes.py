# -*- coding: utf-8 -*-
"""Render leg tables for all planned routes into a markdown doc."""
import io
import json
import os

TOOLS = (r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS"
         r"\tests\t4_lite_drive\tools")
DOC = (r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS"
       r"\tests\t4_lite_drive\ROUTES.md")

files = [
    ("route1_A_to_B_plan.json", "A → B"),
    ("route2_A_to_C_plan.json", "A → C"),
    ("route3_A_to_D_plan.json", "A → D"),
    ("route4_A_to_E_plan.json", "A → E"),
    ("route5_ABCDE_plan.json", "A → B → C → D → E（一次演示）"),
]

lines = ["# 任务四路线（按地图墙位规划，可直接执行）", "",
         "> 坐标来自 `task4_navigation/data/map_v1.json`（3×3 m，墙 30 cm，图片估算，可能有偏差）。",
         "> 规划时墙按 150 mm 膨胀，折线用视线平滑，转向按“转完再直行”执行。",
         "> 执行：`python3 route_run.py <文件> --speed 30`；每段开始自动清 STOP 锁存。", ""]

for fname, title in files:
    path = os.path.join(TOOLS, fname)
    data = json.load(io.open(path, encoding="utf-8"))
    legs = data["legs"]
    total = sum(l["mm"] for l in legs if l["type"] == "drive")
    turns = [l["deg"] for l in legs if l["type"] == "turn"]
    lines.append("## %s" % title)
    lines.append("")
    lines.append("文件：`tools/%s`；总里程 **%.0f mm**，转向 **%d** 次。" % (fname, total, len(turns)))
    lines.append("")
    lines.append("| # | 动作 | 参数 |")
    lines.append("| --- | --- | --- |")
    n = 0
    for l in legs:
        n += 1
        if l["type"] == "turn":
            lines.append("| %d | 转 | %+.1f° |" % (n, l["deg"]))
        else:
            lines.append("| %d | 直行 | %d mm |" % (n, l["mm"]))
    lines.append("")

io.open(DOC, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
print("wrote", DOC)
