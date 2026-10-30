#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t4_logs —— 管理 Pi 上的本地采集会话（start/stop/status/list/purge）。

数据存在 ~/t4_logs/session_*/；联网后用 PC 端 pull_logs.ps1 拉回并清理。
"""
import argparse
import os
import subprocess
import sys
import time

BASE = os.path.expanduser("~/t4_logs")
TOOLS = os.path.dirname(os.path.abspath(__file__))
LOGGER = os.path.join(TOOLS, "t4_pi_logger.py")


def sessions():
    if not os.path.isdir(BASE):
        return []
    out = []
    for d in sorted(os.listdir(BASE)):
        if d.startswith("session_"):
            full = os.path.join(BASE, d)
            size = 0
            files = 0
            for root, _dirs, fs in os.walk(full):
                for f in fs:
                    try:
                        size += os.path.getsize(os.path.join(root, f))
                        files += 1
                    except OSError:
                        pass
            out.append((d, size, files))
    return out


def pid_running():
    try:
        out = subprocess.run(["pgrep", "-f", "t4_pi_logger.py"],
                             capture_output=True, text=True).stdout.strip()
        return out
    except Exception:
        return ""


def cmd_start(args):
    if pid_running():
        print("logger already running:", pid_running())
        return 1
    os.makedirs(BASE, exist_ok=True)
    cmd = [sys.executable, LOGGER, "--port", str(args.port),
           "--max-mb", str(args.max_mb), "--keep", str(args.keep)]
    if args.serial:
        cmd += ["--serial", args.serial]
    if args.camera:
        cmd += ["--camera"]
    if args.duration:
        cmd += ["--duration", str(args.duration)]
    log = open(os.path.join(BASE, "logger.out"), "ab")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True, cwd=TOOLS)
    print("started pid", p.pid)
    time.sleep(0.8)
    print("status:", "running" if pid_running() else "FAILED (check ~/t4_logs/logger.out)")
    print("dir:", BASE)
    return 0


def cmd_stop(_args):
    pid = pid_running()
    if not pid:
        print("not running")
        return 0
    subprocess.run(["pkill", "-f", "t4_pi_logger.py"])
    time.sleep(0.8)
    print("stopped", pid)
    return 0


def cmd_status(_args):
    pid = pid_running()
    print("running:", pid or "no")
    for d, size, files in sessions()[-3:]:
        print("  %-36s %8.1f KB  %d files" % (d, size / 1024.0, files))
        ev = os.path.join(BASE, d, "events.log")
        if os.path.exists(ev):
            with open(ev, encoding="utf-8") as f:
                lines = f.readlines()[-5:]
            for ln in lines:
                print("     ", ln.rstrip())
    return 0


def cmd_size(args):
    d = os.path.join(BASE, args.session)
    files = 0
    total = 0
    for root, _dirs, fs in os.walk(d):
        for f in fs:
            files += 1
            total += os.path.getsize(os.path.join(root, f))
    print('{"files": %d, "bytes": %d}' % (files, total))
    return 0


def cmd_list(_args):
    for d, size, files in sessions():
        print("%-40s %10.2f MB  %3d files" % (d, size / 1048576.0, files))
    if not sessions():
        print("(empty)", BASE)
    return 0


def cmd_purge(args):
    ss = sessions()
    if args.keep >= 0:
        for d, _s, _f in ss[:-args.keep] if len(ss) > args.keep else []:
            full = os.path.join(BASE, d)
            for f in os.listdir(full):
                os.remove(os.path.join(full, f))
            os.rmdir(full)
            print("purged", d)
    if args.all:
        for d, _s, _f in sessions():
            full = os.path.join(BASE, d)
            for f in os.listdir(full):
                os.remove(os.path.join(full, f))
            os.rmdir(full)
            print("purged", d)
    cmd_list(args)
    return 0


def main():
    ap = argparse.ArgumentParser(description="t4 logs manager")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("start")
    p.add_argument("--port", type=int, default=5001)
    p.add_argument("--serial", default=None)
    p.add_argument("--camera", action="store_true")
    p.add_argument("--duration", type=float, default=0.0)
    p.add_argument("--max-mb", type=float, default=200.0)
    p.add_argument("--keep", type=int, default=6)
    sub.add_parser("stop")
    sub.add_parser("status")
    sub.add_parser("list")
    q2 = sub.add_parser("size")
    q2.add_argument("session")
    q = sub.add_parser("purge")
    q.add_argument("--keep", type=int, default=3)
    q.add_argument("--all", action="store_true")
    args = ap.parse_args()
    return {"start": cmd_start, "stop": cmd_stop, "status": cmd_status,
            "list": cmd_list, "size": cmd_size, "purge": cmd_purge}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
