# -*- coding: utf-8 -*-
"""Generate the RTL/TB wall tables from data/map_v1.json (single source of truth).

Wall id = 1-based index in map_v1.json["walls"]. Patches:
  * tests/task4_navigation/src/nav_map_wall_association.v  (wall_ax/ay/bx/by case bodies)
  * tests/task4_navigation/sim/tb_nav_closed_loop.v        (raycast case(k) wall list)
Usage: python gen_wall_table.py [--check]
  --check : exit 1 if the checked-in RTL/TB tables differ from the map (CI guard).
"""
import io
import json
import re
import sys

ROOT = (r"D:\Gowin\Project\Contest\Embodied_RPACS\Embodied_RPACS\Embodied_RPACS"
        r"\tests\task4_navigation")
MAP = ROOT + r"\data\map_v1.json"
RTL = ROOT + r"\src\nav_map_wall_association.v"
TB = ROOT + r"\sim\tb_nav_closed_loop.v"

m = json.load(io.open(MAP, encoding="utf-8"))
walls = m["walls"]
ids = {}
for i, w in enumerate(walls):
    ids[i + 1] = (int(round(w["a"][0])), int(round(w["a"][1])),
                  int(round(w["b"][0])), int(round(w["b"][1])))

changed = []

# ---------------- RTL functions
t = io.open(RTL, encoding="utf-8").read()
for fn, idx in (("wall_ax", 0), ("wall_ay", 1), ("wall_bx", 2), ("wall_by", 3)):
    lines = []
    row = []
    for wid in sorted(ids):
        row.append("%d:%s=%d;" % (wid, fn, ids[wid][idx]))
        if len(row) == 4:
            lines.append("            " + "".join(row))
            row = []
    if row:
        lines.append("            " + "".join(row))
    lines.append("            default:%s=0;" % fn)
    body = "\n".join(lines) + "\n        "
    pat = re.compile(r"(function automatic integer %s\(input integer id\);\s*begin case\(id\))(.*?)(endcase end)"
                     % fn, re.S)
    mm = pat.search(t)
    assert mm, fn + " block not found"
    new = mm.group(1) + body + mm.group(3)
    if new != mm.group(0):
        t = t[:mm.start()] + new + t[mm.end():]
        changed.append("rtl:" + fn)
io.open(RTL, "w", encoding="utf-8", newline="\n").write(t)

# ---------------- TB raycast case(k)
t = io.open(TB, encoding="utf-8").read()
lines = []
for k in sorted(ids):
    ax, ay, bx, by = ids[k]
    lines.append("                    %d:begin ax=%d;ay=%d;bx=%d;by=%d;end" % (k, ax, ay, bx, by))
lines.append("                    default:begin ax=%d;ay=%d;bx=%d;by=%d;end" %
             (ids[max(ids)][0], ids[max(ids)][1], ids[max(ids)][2], ids[max(ids)][3]))
body = "\n".join(lines) + "\n                "
pat = re.compile(r"(\n                case\(k\))(.*?)(\n                endcase)", re.S)
mm = pat.search(t)
assert mm, "TB case(k) block not found"
new = mm.group(1) + "\n" + body.rstrip() + "\n                endcase"
if new != mm.group(0):
    t = t[:mm.start()] + new + t[mm.end():]
    changed.append("tb:raycast")
io.open(TB, "w", encoding="utf-8", newline="\n").write(t)

# ---------------- summary / check
if "--check" in sys.argv:
    if changed:
        print("OUT OF SYNC:", ", ".join(changed))
        sys.exit(1)
    print("wall tables in sync with", m["map_version"])
else:
    print("map", m["map_version"], "walls", len(ids))
    print("patched:", ", ".join(changed) if changed else "(already in sync)")
