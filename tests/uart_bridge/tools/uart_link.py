#!/usr/bin/env python3
"""上位机 <-> FPGA 链路工具（命令 / 监视 / 台架验收）。

用法示例::

    python uart_link.py ports
    python uart_link.py selftest
    python uart_link.py monitor --port COM14
    python uart_link.py send --port COM14 setvel --left 120 --right 120
    python uart_link.py bench  --port COM14 --out records/2026-10-29_bench.json

设计要点：
  * 协议编解码全在 protocol.py（与传输无关），本文件只做"会话 + 人机界面"；
  * 心跳在独立线程按 10 Hz 发送，命令行里任何阻塞都不会导致链路误判失联；
  * 所有命令都打印 FPGA 的 ACK 状态，失败原因一目了然；
  * bench 子命令是台架验收（对应 docs/PROTOCOL_v1.md 第 8 节 T13/T14/T15）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P          # noqa: E402
import transport as T         # noqa: E402


# --------------------------------------------------------------------------
class Link:
    """一个链路会话：传输 + 解析 + 心跳线程 + 遥测缓存。"""

    def __init__(self, tp: T.Transport, heartbeat_hz: float = 10.0,
                 verbose: bool = False):
        self.tp = tp
        self.verbose = verbose
        self.parser = P.FrameParser()
        self.heartbeat_period = 1.0 / heartbeat_hz if heartbeat_hz > 0 else 0.0
        self._hb_enabled = threading.Event()
        self._hb_enabled.set()
        self._seq = 0
        self._lock = threading.Lock()
        self._seq_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.telemetry: Deque[P.Telemetry] = deque(maxlen=2000)
        self.telemetry_times: Deque[float] = deque(maxlen=2000)
        self.acks: Deque[P.Ack] = deque(maxlen=200)
        self.frames_seen: Dict[str, int] = {}
        self.on_frame: Optional[Callable[[P.Frame], None]] = None
        self._rx_bytes = 0

    # ---------------------------------------------------------------- 基础
    def start(self) -> None:
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.tp.close()

    def _next_seq(self) -> int:
        """心跳线程与命令行共用同一个序号计数器，必须加锁。"""
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def _rx_loop(self) -> None:
        hb_deadline = time.time()
        while not self._stop.is_set():
            try:
                data = self.tp.read(0.01)
            except Exception:  # noqa: BLE001 - 串口拔插时继续跑
                data = b""
            if data:
                self._rx_bytes += len(data)
                for frame in self.parser.feed(data):
                    self._dispatch(frame)
            if self._hb_enabled.is_set() and self.heartbeat_period > 0:
                now = time.time()
                if now >= hb_deadline:
                    try:
                        self.send_raw(P.msg_heartbeat(self._next_seq()))
                    except Exception:  # noqa: BLE001
                        pass
                    hb_deadline = now + self.heartbeat_period

    def _dispatch(self, frame: P.Frame) -> None:
        self.frames_seen[frame.name] = self.frames_seen.get(frame.name, 0) + 1
        if frame.msg_type == P.MSG_TELEM:
            try:
                self.telemetry.append(P.decode_telem(frame.payload))
                self.telemetry_times.append(time.time())
            except ValueError:
                pass
        elif frame.msg_type == P.MSG_ACK:
            try:
                self.acks.append(P.decode_ack(frame.payload))
            except ValueError:
                pass
        if self.on_frame:
            self.on_frame(frame)

    # ---------------------------------------------------------------- 发送
    def send_raw(self, data: bytes) -> None:
        with self._lock:
            self.tp.send(data)

    send = send_raw

    def heartbeat(self, seq: Optional[int] = None) -> int:
        if seq is None:
            seq = self._next_seq()
        self.send(P.msg_heartbeat(seq))
        return seq

    def set_vel(self, left: int, right: int, seq: Optional[int] = None) -> int:
        if seq is None:
            seq = self._next_seq()
        self.send(P.msg_set_vel(left, right, seq))
        return seq

    def stop_cmd(self) -> None:
        self.send(P.msg_stop())

    def set_limit(self, limit: int) -> None:
        self.send(P.msg_set_limit(limit))

    def clear_fault(self) -> None:
        self.send(P.msg_clear_fault())

    def pause_heartbeat(self) -> None:
        self._hb_enabled.clear()

    def resume_heartbeat(self) -> None:
        self._hb_enabled.set()

    # ---------------------------------------------------------------- 等待
    def wait_telemetry(self, timeout: float = 1.0,
                       predicate: Optional[Callable[[P.Telemetry], bool]] = None
                       ) -> Optional[P.Telemetry]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.telemetry:
                t = self.telemetry[-1]
                if predicate is None or predicate(t):
                    return t
            time.sleep(0.005)
        return None

    def wait_ack(self, timeout: float = 1.0,
                 acked_type: Optional[int] = None) -> Optional[P.Ack]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            for ack in list(self.acks):
                if acked_type is None or ack.acked_type == acked_type:
                    self.acks.remove(ack)
                    return ack
            time.sleep(0.005)
        return None

    def telem_rate(self, window_s: float = 1.0) -> float:
        now = time.time()
        recent = [t for t in self.telemetry_times if now - t <= window_s]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    @property
    def rx_bytes(self) -> int:
        return self._rx_bytes


# --------------------------------------------------------------------------
def _open(args) -> Link:
    if args.transport == "uart":
        tp = T.UartTransport(port=args.port, baud=args.baud)
    elif args.transport == "udp":
        tp = T.UdpTransport(remote=(args.udp_host, args.udp_port),
                            local=("0.0.0.0", args.udp_local_port))
    else:
        tp = T.FileTransport(path=args.file, sink=args.sink)
    return Link(tp, heartbeat_hz=0.0 if args.no_heartbeat else args.heartbeat_hz)


# --------------------------------------------------------------------------
def cmd_ports(args) -> int:
    print("可用串口：")
    for line in T.describe_serial_ports():
        print("  ", line)
    return 0


def cmd_selftest(args) -> int:
    import subprocess
    here = os.path.dirname(os.path.abspath(__file__))
    return subprocess.call([sys.executable, os.path.join(here, "protocol_selftest.py")])


def cmd_monitor(args) -> int:
    link = _open(args)
    log = open(args.log, "w", encoding="utf-8") if args.log else None
    t0 = time.time()
    last_print = 0.0

    def on_frame(frame: P.Frame) -> None:
        if log:
            log.write("%.6f %s\n" % (time.time() - t0, P.describe(frame)))
            log.flush()

    link.on_frame = on_frame
    link.start()
    print("monitor on %s, Ctrl+C 退出" % link.tp.name)
    try:
        while True:
            time.sleep(0.25)
            now = time.time()
            if now - last_print < 1.0:
                continue
            last_print = now
            if link.telemetry:
                t = link.telemetry[-1]
                faults = ("[" + ",".join(t.fault_names) + "]") if t.fault_names else ""
                print("[%6.1fs] enc=(%d,%d) yaw=%+.2f spd=(%d,%d) fault=%02X%s "
                      "link_seq=%d cmd_seq=%d hb=%dms | rate=%.1fHz rx=%dB frames=%s"
                      % (now - t0, t.enc_l, t.enc_r, t.yaw_deg,
                         t.speed_l_mm_s, t.speed_r_mm_s, t.fault, faults,
                         t.link_seq, t.cmd_seq, t.hb_age_ms,
                         link.telem_rate(1.0), link.rx_bytes, link.frames_seen))
            else:
                print("[%6.1fs] no telemetry yet  frames=%s rx=%dB"
                      % (now - t0, link.frames_seen, link.rx_bytes))
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        if log:
            log.close()
    return 0


def cmd_send(args) -> int:
    link = _open(args)
    link.start()
    try:
        time.sleep(0.3)
        if args.command == "setvel":
            seq = link.set_vel(args.left, args.right)
            ack = link.wait_ack(0.6, P.MSG_SET_VEL)
            print("SET_VEL left=%d right=%d seq=%d -> %s"
                  % (args.left, args.right, seq,
                     ("ACK %s" % ack.status_name) if ack else "no ACK"))
        elif args.command == "stop":
            link.stop_cmd()
            ack = link.wait_ack(0.6, P.MSG_STOP)
            print("STOP -> %s" % (("ACK %s" % ack.status_name) if ack else "no ACK"))
        elif args.command == "limit":
            link.set_limit(args.mm_s)
            ack = link.wait_ack(0.6, P.MSG_SET_LIMIT)
            print("SET_LIMIT %d -> %s"
                  % (args.mm_s, ("ACK %s" % ack.status_name) if ack else "no ACK"))
        elif args.command == "clear":
            link.clear_fault()
            ack = link.wait_ack(0.6, P.MSG_CLEAR_FAULT)
            print("CLEAR_FAULT -> %s"
                  % (("ACK %s" % ack.status_name) if ack else "no ACK"))
        elif args.command == "heartbeat":
            for _ in range(args.count):
                link.heartbeat()
                time.sleep(1.0 / max(1.0, args.rate))
            print("sent %d heartbeats" % args.count)
        time.sleep(0.2)
    finally:
        link.stop()
    return 0


def cmd_bench(args) -> int:
    """台架验收：心跳/遥测/命令/ACK/停车/失联/恢复 全链路检查。"""
    result: Dict[str, object] = {"port": args.port, "baud": args.baud,
                                 "started": time.strftime("%Y-%m-%dT%H:%M:%S")}
    checks: List[Dict[str, object]] = []

    def check(name: str, ok: bool, detail: str = "") -> bool:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})
        print("%s %-42s %s" % ("PASS" if ok else "FAIL", name, detail))
        return ok

    link = _open(args)
    t_start = time.time()
    link.start()
    try:
        # 1) 链路与遥测
        tele = link.wait_telemetry(2.0)
        check("telemetry_stream_present", tele is not None,
              "none within 2 s" if tele is None else "fault=0x%02X" % tele.fault)
        time.sleep(1.2)
        rate = link.telem_rate(1.0)
        check("telemetry_rate_50hz", 30.0 <= rate <= 70.0, "%.1f Hz" % rate)

        # 2) 心跳建立链路 + CLEAR_FAULT 解除上电锁存
        link.heartbeat(1)
        time.sleep(0.15)
        link.clear_fault()
        ack = link.wait_ack(0.6, P.MSG_CLEAR_FAULT)
        check("clear_fault_ack_ok", ack is not None and ack.status == P.ACK_OK,
              ack.status_name if ack else "no ACK")
        tele = link.wait_telemetry(0.6,
                                   lambda t: not (t.fault & P.FAULT_LINK_LOST))
        check("link_lost_cleared_by_clear_fault", tele is not None,
              "fault=0x%02X" % (tele.fault if tele else 0xFF))

        # 3) SET_VEL + ACK + 遥测回显 seq
        seq = (int(time.time()) & 0x7F) + 1
        link.set_vel(args.left, args.right, seq)
        ack = link.wait_ack(0.8, P.MSG_SET_VEL)
        check("setvel_ack_ok", ack is not None and ack.status == P.ACK_OK,
              ack.status_name if ack else "no ACK")
        check("setvel_ack_seq_echo", ack is not None and ack.seq == seq,
              "expected %d got %s" % (seq, ack.seq if ack else "-"))
        tele = link.wait_telemetry(0.8, lambda t: t.cmd_seq == seq)
        check("telem_echoes_cmd_seq", tele is not None,
              "cmd_seq=%s" % (tele.cmd_seq if tele else "-"))

        # 4) 限速裁剪
        link.set_limit(50)
        link.wait_ack(0.5, P.MSG_SET_LIMIT)
        seq2 = (seq + 1) & 0xFF
        link.set_vel(300, -300, seq2)
        ack = link.wait_ack(0.8, P.MSG_SET_VEL)
        check("over_limit_rejected_by_ack",
              ack is not None and ack.status == P.ACK_REJECT_LIMIT,
              ack.status_name if ack else "no ACK")
        link.set_limit(args.limit_restore)
        time.sleep(0.05)      # 背靠背命令之间留间隔（ACK 队列深度 1，见协议 §5.1 末说明）

        # 5) STOP 锁存 + 遥测故障位
        link.stop_cmd()
        ack = link.wait_ack(0.8, P.MSG_STOP)
        check("stop_ack_ok", ack is not None and ack.status == P.ACK_OK,
              ack.status_name if ack else "no ACK")
        tele = link.wait_telemetry(0.8,
                                   lambda t: bool(t.fault & P.FAULT_STOP_LATCHED))
        check("stop_fault_bit_in_telemetry", tele is not None,
              "fault=0x%02X" % (tele.fault if tele else 0))
        link.set_vel(100, 100)
        ack = link.wait_ack(0.8, P.MSG_SET_VEL)
        check("setvel_refused_while_stopped",
              ack is not None and ack.status == P.ACK_REJECT_STOP,
              ack.status_name if ack else "no ACK")

        # 6) 解除停车锁存
        link.heartbeat(2)
        time.sleep(0.1)
        link.clear_fault()
        ack = link.wait_ack(0.6, P.MSG_CLEAR_FAULT)
        check("clear_after_stop_ack_ok", ack is not None and ack.status == P.ACK_OK,
              ack.status_name if ack else "no ACK")
        tele = link.wait_telemetry(0.6, lambda t: not (t.fault & P.FAULT_STOP_LATCHED))
        check("stop_bit_cleared", tele is not None,
              "fault=0x%02X" % (tele.fault if tele else 0xFF))

        # 7) 失联：停止心跳 -> LINK_LOST
        link.pause_heartbeat()
        t_lost = time.time()
        tele = link.wait_telemetry(1.5,
                                   lambda t: bool(t.fault & P.FAULT_LINK_LOST))
        dt_ms = (time.time() - t_lost) * 1000.0 if tele else -1.0
        check("link_lost_after_heartbeat_stop", tele is not None,
              ("detected in %.0f ms" % dt_ms) if tele else "not detected within 1.5 s")
        if tele:
            check("link_lost_timing_plausible", 200.0 <= dt_ms <= 1000.0,
                  "%.0f ms (default timeout 300 ms)" % dt_ms)
            check("hb_age_reported", tele.hb_age_ms >= 200,
                  "hb_age=%d ms" % tele.hb_age_ms)

        # 8) 恢复：心跳 + CLEAR_FAULT
        link.resume_heartbeat()
        link.heartbeat(3)
        time.sleep(0.15)
        link.clear_fault()
        ack = link.wait_ack(0.8, P.MSG_CLEAR_FAULT)
        check("recovery_clear_ack_ok", ack is not None and ack.status == P.ACK_OK,
              ack.status_name if ack else "no ACK")
        tele = link.wait_telemetry(0.8,
                                   lambda t: not (t.fault & P.FAULT_LINK_LOST))
        check("link_recovered", tele is not None,
              "fault=0x%02X" % (tele.fault if tele else 0xFF))

        # 9) 误码统计
        stats = link.parser.stats.as_dict()
        result["parser_stats"] = stats
        total_frames = max(1, stats["good"] + stats["crc_err"] + stats["tail_err"]
                           + stats["len_err"])
        err_rate = (stats["crc_err"] + stats["tail_err"] + stats["len_err"]) / total_frames
        check("crc_error_rate_below_1e-3", err_rate <= 1e-3,
              "%.3g (%d good frames)" % (err_rate, stats["good"]))

        result["frames_seen"] = dict(link.frames_seen)
        result["telem_rate_hz"] = round(link.telem_rate(1.0), 2)
        result["rx_bytes"] = link.rx_bytes
        result["duration_s"] = round(time.time() - t_start, 2)
    finally:
        link.stop()

    passed = sum(1 for c in checks if c["ok"])
    result["checks"] = checks
    result["passed"] = passed
    result["failed"] = len(checks) - passed
    result["ok"] = passed == len(checks)
    result["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    print("\n%d/%d checks passed" % (passed, len(checks)))
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        print("report ->", args.out)
    return 0 if result["ok"] else 1


# --------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="host <-> FPGA link tool (protocol v1)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--transport", default="uart", choices=["uart", "udp", "file"])
        p.add_argument("--port", default="COM14")
        p.add_argument("--baud", type=int, default=115200)
        p.add_argument("--udp-host", default="192.168.1.50")
        p.add_argument("--udp-port", type=int, default=50000)
        p.add_argument("--udp-local-port", type=int, default=50001)
        p.add_argument("--file", default="")
        p.add_argument("--sink", default="")
        p.add_argument("--heartbeat-hz", type=float, default=10.0)
        p.add_argument("--no-heartbeat", action="store_true")

    p_ports = sub.add_parser("ports", help="list serial ports")
    p_ports.set_defaults(func=cmd_ports)

    p_self = sub.add_parser("selftest", help="protocol codec self-test")
    p_self.set_defaults(func=cmd_selftest)

    p_mon = sub.add_parser("monitor", help="print telemetry/ACK")
    common(p_mon)
    p_mon.add_argument("--log", default=None)
    p_mon.set_defaults(func=cmd_monitor)

    p_send = sub.add_parser("send", help="send one command")
    common(p_send)
    ssub = p_send.add_subparsers(dest="command", required=True)
    s1 = ssub.add_parser("setvel")
    s1.add_argument("--left", type=int, default=100)
    s1.add_argument("--right", type=int, default=100)
    ssub.add_parser("stop")
    s3 = ssub.add_parser("limit")
    s3.add_argument("mm_s", type=int)
    ssub.add_parser("clear")
    s5 = ssub.add_parser("heartbeat")
    s5.add_argument("--count", type=int, default=5)
    s5.add_argument("--rate", type=float, default=10.0)
    p_send.set_defaults(func=cmd_send)

    p_bench = sub.add_parser("bench", help="board acceptance test")
    common(p_bench)
    p_bench.add_argument("--left", type=int, default=120)
    p_bench.add_argument("--right", type=int, default=120)
    p_bench.add_argument("--limit-restore", type=int, default=300)
    p_bench.add_argument("--out", default=None)
    p_bench.set_defaults(func=cmd_bench)
    return ap


def main(argv: Optional[List[str]] = None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
