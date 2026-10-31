# -*- coding: utf-8 -*-
"""Deterministic clearance check for every route file (no noise): min wall distance of the
car body along the planned path, plus the location. Appends a table to route_simulation.md."""
import io
import json
import math
import os
import sys

sys.path.insert(0, r"D:\Gowin\Project\Contest\Embodied_RPACS\work")
import route_sim as S

TOOLS = (r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS"
         r"\tests\t4_lite_drive\tools")
DOC = (r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS"
       r"\tests\t4_lite_drive\records\route_simulation.md")

rows = []
for name in sorted(os.listdir(TOOLS)):
    if not (name.startswith("route") and name.endswith(".json")):
        continue
    legs = json.load(io.open(os.path.join(TOOLS, name), encoding="utf-8"))["legs"]
    x, y, hdg = 2700.0, 300.0, 0.0
    worst = (1e9, None, None)
    for li, leg in enumerate(legs):
        if leg["type"] == "turn":
            for k in range(9):
                h = hdg + leg["deg"] * k / 8.0
                c = S.min_clearance(x, y, h)
                if c < worst[0]:
                    worst = (c, (round(x), round(y), round(h, 1)), ("turn", li, leg["deg"]))
            hdg += leg["deg"]
        elif leg["type"] == "drive":
            dist = float(leg["mm"])
            steps = max(2, int(dist / 25))
            for s in range(steps):
                fx, fy = math.sin(math.radians(hdg)), math.cos(math.radians(hdg))
                x += dist / steps * fx
                y += dist / steps * fy
                c = S.min_clearance(x, y, hdg)
                if c < worst[0]:
                    worst = (c, (round(x), round(y), round(hdg, 1)), ("drive", li, dist))
        else:
            continue
    rows.append((name, len(legs), worst[0], worst[1], worst[2]))
    print("%-30s legs=%2d min_clear=%7.1f mm at %s during %s" %
          (name, len(legs), worst[0], worst[1], worst[2]))

lines = ["", "## 确定性最小墙距（无噪声，按车体 320x240 精确计算）", "",
         "| 路线文件 | 段数 | 最小墙距(mm) | 位置(x,y,hdg) | 所在动作 |",
         "| --- | --- | --- | --- | --- |"]
for (name, n, clr, pose, act) in rows:
    act_s = ("%s %s" % (act[0], act[2])) if act else "—"
    lines.append("| %s | %d | %.1f | %s | %s |" % (name, n, clr, pose, act_s))
lines += ["", "> 注：这是**无噪声**的几何最小距离；考虑航向保持误差(σ0.6°)与里程误差(0.8%)后,",
          "> 蒙特卡洛中最小墙距会降到 0 附近。因此这些路线属于\"贴墙通过\"，只能低速+人工盯着跑。"]
io.open(DOC, "a", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
print("appended to", DOC)
