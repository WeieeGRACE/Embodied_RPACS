# -*- coding: utf-8 -*-
"""Decode the P6-21 distance-log diagnostic capture (build_distance.tcl).

Line format is the normal event-logger output; the D payload is repurposed:
  [15:0] front_mm   [31:16] left_mm   [47:32] right_mm
  [50:48] tof_reused (confirmed 20 mm clamp mask)
  [55:51] route_state  [56] fault  [57] auto_run  [58] avoid_mode
  [63:59] zero

Usage:
  python tools/decode_distance_log.py <capture.log> [--changes-only]
The capture may carry a "[HH:MM:SS.mmm] " timestamp prefix from
read_serial_log.ps1 -Timestamped.
"""
import argparse
import io
import re

STATE_NAMES = {0: "IDLE0", 1: "IDLE", 2: "CRUISE", 3: "BRAKE_SELECT", 4: "TURN_OUT",
               5: "WAIT_OUT", 6: "SHIFT", 7: "TURN_PARALLEL", 8: "WAIT_PARALLEL",
               9: "PASS", 10: "BRAKE_EDGE", 11: "EXTRA", 12: "TURN_BACK",
               13: "WAIT_BACK", 14: "RETURN", 15: "TURN_HOME", 16: "WAIT_HOME",
               17: "BLOCKED", 18: "USER_STOP", 19: "FAULT", 20: "POST_HOME",
               21: "BRAKE_SHIFT", 22: "SHIFT_EXTRA", 23: "SAMPLE_SHIFT"}

LINE = re.compile(r"(?:\[(\d\d:\d\d:\d\d\.\d\d\d)\]\s*)?D\s+([0-9A-Fa-f]{16})")


def decode(value):
    return {
        "front": value & 0xFFFF,
        "left": (value >> 16) & 0xFFFF,
        "right": (value >> 32) & 0xFFFF,
        "reused": (value >> 48) & 0x7,
        "route": (value >> 51) & 0x1F,
        "fault": (value >> 56) & 1,
        "auto": (value >> 57) & 1,
        "avoid": (value >> 58) & 1,
    }


def main():
    parser = argparse.ArgumentParser(description="decode the distance-log capture")
    parser.add_argument("log")
    parser.add_argument("--changes-only", action="store_true")
    args = parser.parse_args()

    rows = []
    for line in io.open(args.log, encoding="utf-8", errors="replace").read().splitlines():
        match = LINE.search(line)
        if match:
            rows.append((match.group(1) or "", decode(int(match.group(2), 16))))

    print("file=%s snapshots=%d" % (args.log, len(rows)))
    if not rows:
        return
    print("span %s .. %s" % (rows[0][0], rows[-1][0]))
    states = {}
    for _, row in rows:
        states[row["route"]] = states.get(row["route"], 0) + 1
    print("route histogram:", {STATE_NAMES.get(k, k): v for k, v in sorted(states.items())})
    print("fault frames:", sum(1 for _, r in rows if r["fault"]))
    print("front<=85 frames:", sum(1 for _, r in rows if r["front"] <= 85))
    for key in ("front", "left", "right"):
        values = [r[key] for _, r in rows if 1 <= r[key] <= 8000]
        if values:
            print("%-5s min=%d max=%d" % (key, min(values), max(values)))

    last = None
    print("timeline:")
    for stamp, row in rows:
        sig = (row["route"], row["reused"], row["fault"], row["auto"], row["avoid"],
               None if args.changes_only else (row["front"], row["left"], row["right"]))
        if sig == last:
            continue
        last = sig
        print("  %s %-14s front=%4d left=%4d right=%4d reused=%d%s%s%s"
              % (stamp or "--", STATE_NAMES.get(row["route"], row["route"]),
                 row["front"], row["left"], row["right"], row["reused"],
                 " FAULT" if row["fault"] else "",
                 " AUTO" if row["auto"] else "",
                 " AVOID" if row["avoid"] else ""))


if __name__ == "__main__":
    main()
