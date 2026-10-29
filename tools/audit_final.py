# -*- coding: utf-8 -*-
"""Final acceptance audit: cross-check the delivered bitstreams, reports and docs.

Run from the repository root:  python tools/audit_final.py
Exit code 0 only when every check passes.
"""
import glob
import hashlib
import io
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_FS = os.path.join(ROOT, "impl/pnr/Embodied_RPACS.fs")
P6_FS = os.path.join(ROOT, "tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.fs")
checks = []


def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))


def sha256(path):
    h = hashlib.sha256()
    with io.open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def read(path):
    """Read UTF-8 or UTF-16 (PowerShell Tee-Object default) log/doc files."""
    with io.open(path, "rb") as f:
        data = f.read()
    if data[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return data.decode("utf-16", errors="replace")
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("utf-8", errors="replace")


MAIN_SHA = "901D7394146E472AF386E5BAB4092C5FD94B62245CF8666C471EDA38F7EAB14B"
P6_SHA = "08DFDED3A43359D8E1237C03751E5D1978A97AFA958ED7C8276291FE706F399E"

# 1. bitstreams
check("main bitstream exists", os.path.isfile(MAIN_FS))
if os.path.isfile(MAIN_FS):
    check("main bitstream size", os.path.getsize(MAIN_FS) == 6228180, str(os.path.getsize(MAIN_FS)))
    got = sha256(MAIN_FS)
    check("main bitstream sha256", got == MAIN_SHA, got)
check("P6-21 bitstream exists", os.path.isfile(P6_FS))
if os.path.isfile(P6_FS):
    got = sha256(P6_FS)
    check("P6-21 bitstream sha256", got == P6_SHA, got)

# 2. post-PnR resource report
rpt = read(os.path.join(ROOT, "impl/pnr/Embodied_RPACS.rpt.txt"))
for label, needle in [
    ("Logic", "15445/23040"),
    ("Register", "6553/23685"),
    ("CLS", "9407/11520"),
    ("BSRAM", "5/56"),
    ("DSP", "8.5/28"),
]:
    check("main report %s" % label, needle in rpt, needle)
tr = read(os.path.join(ROOT, "impl/pnr/Embodied_RPACS_tr_content.html"))
check("main report Fmax 55.050", "55.050(MHz)" in tr)

p6rpt_path = os.path.join(ROOT, "tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.rpt.txt")
if os.path.isfile(p6rpt_path):
    p6rpt = read(p6rpt_path)
    check("P6 report Logic", "15910/23040" in p6rpt)
    check("P6 report CLS", "9612/11520" in p6rpt)
    check("P6 report uart_tx on L12",
          "L12" in read(os.path.join(ROOT, "tests/p6_uart_debug/impl/pnr/Embodied_RPACS_p6uart.pin.html")))

# 3. fault-injection evidence
logs = sorted(glob.glob(os.path.join(ROOT, "records/fault_injection/fixed_final_regression/fixed_regression_*.log")))
check("13 fault-injection logs", len(logs) == 13, str(len(logs)))
for log in logs:
    text = read(log)
    base = os.path.basename(log)
    check("%s FIXED_RUN_OBSERVED" % base, "# FIXED_RUN_OBSERVED scenario=" in text)
    check("%s fixed-mode note" % base, "# NOTE: FIXED_MODE is observation-only" in text)
    check("%s no fault latch" % base,
          "fault_seen=0" in text and "fault_latched=0" in text and "paused=0" in text and "blocked_seen=0" in text)
    check("%s no EVIDENCE FAULT_T0" % base, "EVIDENCE FAULT_T0" not in text)
near = [l for l in logs if "g_near" in l]
if near:
    check("scenario 7 near_shift=1", "near_shift=1" in read(near[0]))
clip = [l for l in logs if "k_clip_fallback" in l]
if clip:
    t = read(clip[0])
    check("scenario 10 fallback completed", "reached_shift=1" in t and "fallback_done=1" in t)
pass_clip = [l for l in logs if "l_pass_clip" in l]
if pass_clip:
    t = read(pass_clip[0])
    check("scenario 11 pass-clip fallback",
          "pass_reached=1" in t and "pass_clip_done=1" in t)

pass_front = [l for l in logs if "m_pass_front_graze" in l]
if pass_front:
    t = read(pass_front[0])
    check("scenario 12 front-graze filter",
          "pass_reached=1" in t and "pass_front_done=1" in t)

summary = read(os.path.join(ROOT, "records/robustness_20261029/fault_injection_final.log"))
check("regression summary 13 scenarios", "PASS fault-injection regression: 13 scenarios" in summary)
seven = read(os.path.join(ROOT, "records/robustness_20261029/seven_suites_final.log"))
check("seven suites PASS", "PASS all seven" in seven)
check("seven suites no errors", "Errors: 0, Warnings: 0" in seven)

# 4. docs consistency
docs = {
    "README.md": read(os.path.join(ROOT, "README.md")),
    "TECHNICAL_REPORT.md": read(os.path.join(ROOT, "TECHNICAL_REPORT.md")),
    "OBSTACLE_BOARD_TEST.md": read(os.path.join(ROOT, "OBSTACLE_BOARD_TEST.md")),
    "FINAL_ACCEPTANCE.md": read(os.path.join(ROOT, "records/robustness_20261029/FINAL_ACCEPTANCE.md")),
}
for name, text in docs.items():
    check("%s has main sha prefix" % name, MAIN_SHA[:8] in text)
check("README full main sha", MAIN_SHA in docs["README.md"])
check("TECH full main sha", MAIN_SHA in docs["TECHNICAL_REPORT.md"])
for name in ("TECHNICAL_REPORT.md", "OBSTACLE_BOARD_TEST.md"):
    check("%s has P6 sha" % name, P6_SHA[:16] in docs[name])
for name, text in docs.items():
    check("%s has Logic 15445" % name, "15445" in text)
for stale in ("13521", "2E4FBB05", "AF7FC649", "59.226", "13518", "8328"):
    bad = [n for n, t in docs.items() if stale in t]
    check("no stale %s in main docs" % stale, not bad, ",".join(bad))

# 5. release copy consistency (optional, ignored by git)
rel = os.path.join(ROOT, "release/Embodied_RPACS_main_20261029_clipfix")
if os.path.isdir(rel):
    sums = read(os.path.join(rel, "SHA256SUMS.txt"))
    check("release sums main sha", MAIN_SHA in sums)
    check("release sums P6 sha", P6_SHA in sums)
    check("release main fs matches", sha256(os.path.join(rel, "impl/pnr/Embodied_RPACS.fs")) == MAIN_SHA)

# 6. git state (tracked files only; the audit log itself is rewritten by every run
# and is committed as evidence, so it must not count as a dirty worktree entry)
status = subprocess.check_output(
    ["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT, text=True)
SELF_LOG = "records/robustness_20261029/final_audit_final.log"
status = "\n".join(line for line in status.splitlines() if SELF_LOG not in line)
check("git worktree clean", status.strip() == "", status.strip()[:200])

# report
failed = [c for c in checks if not c[1]]
for name, ok, detail in checks:
    print("%s %s%s" % ("PASS" if ok else "FAIL", name, " -> " + detail if detail and not ok else ""))
print("-" * 60)
print("SUMMARY %d checks, %d failed" % (len(checks), len(failed)))
sys.exit(1 if failed else 0)
