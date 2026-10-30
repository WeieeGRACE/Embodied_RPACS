#!/usr/bin/env python3
"""任务四 段记录 UART 解析库。

FPGA 的 `nav_segment_uart_logger` 每段结束/故障时输出一条 54 字节记录（115200 8N1）：

    T4 SEG=<1hex> DIST=<8hex> YAW=<8hex> WALL=<2hex> FAULT=<2hex>\\r\\n

* DIST：无符号 32 位，单位 mm
* YAW ：有符号 32 位补码，单位 0.001°（毫度）
* WALL：墙段 ID（10 = MID_H）
* FAULT：0 正常 / 1 STOP / 2 ESTOP / 3 RANGE / 4 INPUT / 5 GEOMETRY / 6 DONE

解析器容忍任意分块、乱码与丢字节：以 "T4 SEG=" 为同步点重新对齐。
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Iterable, List

# ------------------------------------------------------------------ 常量表
F_NONE, F_STOP, F_ESTOP, F_RANGE, F_INPUT, F_GEOMETRY = 0, 1, 2, 3, 4, 5
F_TIMEOUT, F_EVENT_MISMATCH, F_NO_PROGRESS, F_BLOCKED, F_GOAL = 6, 7, 8, 9, 10
FAULT_NAMES = {
    F_NONE: "NONE",
    F_STOP: "STOP",
    F_ESTOP: "ESTOP",
    F_RANGE: "RANGE",
    F_INPUT: "INPUT",
    F_GEOMETRY: "GEOMETRY",
    F_TIMEOUT: "TIMEOUT",
    F_EVENT_MISMATCH: "EVENT_MISMATCH",
    F_NO_PROGRESS: "NO_PROGRESS",
    F_BLOCKED: "BLOCKED",          # 硬停里程到达但墙事件未确认
    F_GOAL: "GOAL",
}

WALL_NAMES = {
    0: "NONE", 1: "WALL_1", 4: "WALL_4", 5: "WALL_5", 6: "WALL_6",
    7: "WALL_7", 8: "WALL_8", 10: "MID_H",
}

#: 单段探针基线（FIELD_TEST_GUIDE.md）

PROBE_DISTANCE = {
    "name": "route6_distance",
    "segments": [
        {"segment": 0, "wall": None, "require_wall": False,
         "dist_min": 400, "dist_max": 440,
         "dist_hard_stop_max": 520, "yaw_tol_mdeg": 5000, "final": True},
    ],
}

PROBE = {
    "segment": 0,
    "wall": 10,               # MID_H
    "dist_confirmed_min": 400,
    "dist_confirmed_max": 430,
    "dist_hard_stop_max": 520,
    "yaw_tol_mdeg": 5000,     # ±5°
    "forward_stop_mm": 120,
}

#: 记录总长（含 CRLF）
RECORD_LEN = 54

_SEG_RE = re.compile(
    b"T4 SEG=(?P<seg>[0-9A-F]) "
    b"DIST=(?P<dist>[0-9A-F]{8}) "
    b"YAW=(?P<yaw>[0-9A-F]{8}) "
    b"WALL=(?P<wall>[0-9A-F]{2}) "
    b"FAULT=(?P<fault>[0-9A-F]{2})\r\n"
)


@dataclass
class SegmentRecord:
    segment: int
    distance_mm: int
    yaw_mdeg: int
    wall: int
    fault: int
    ts: float | None = None

    @property
    def yaw_deg(self) -> float:
        return self.yaw_mdeg / 1000.0

    @property
    def fault_name(self) -> str:
        return FAULT_NAMES.get(self.fault, "0x%02X" % self.fault)

    @property
    def wall_name(self) -> str:
        return WALL_NAMES.get(self.wall, "#%d" % self.wall)

    def to_dict(self) -> dict:
        return {"segment": self.segment, "distance_mm": self.distance_mm,
                "yaw_mdeg": self.yaw_mdeg, "yaw_deg": round(self.yaw_deg, 3),
                "wall": self.wall, "wall_name": self.wall_name,
                "fault": self.fault, "fault_name": self.fault_name,
                "ts": self.ts}

    def __str__(self) -> str:
        return ("T4 SEG=%d DIST=%dmm YAW=%.3f° WALL=%s FAULT=%s"
                % (self.segment, self.distance_mm, self.yaw_deg,
                   self.wall_name, self.fault_name))


@dataclass
class ParseStats:
    records: int = 0
    bytes_in: int = 0
    garbage_bytes: int = 0
    partial_bytes: int = 0
    resyncs: int = 0

    def as_dict(self) -> dict:
        return {"records": self.records, "bytes_in": self.bytes_in,
                "garbage_bytes": self.garbage_bytes,
                "partial_bytes": self.partial_bytes, "resyncs": self.resyncs}


class SegmentLogParser:
    """增量解析器：feed(bytes) 返回本次新解析出的记录列表。"""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.stats = ParseStats()

    def feed(self, data: bytes, ts: float | None = None) -> List[SegmentRecord]:
        out: List[SegmentRecord] = []
        self.stats.bytes_in += len(data)
        self.buf += data
        while True:
            m = _SEG_RE.search(self.buf)
            if not m:
                # 保留末尾可能不完整的记录，丢弃更早的垃圾
                keep = RECORD_LEN - 1
                if len(self.buf) > keep:
                    head = len(self.buf) - keep
                    # "T4" 起点之前的内容算垃圾
                    idx = self.buf.rfind(b"T4 SEG=")
                    cut = idx if idx >= 0 else head
                    if cut > 0:
                        self.stats.garbage_bytes += cut
                        del self.buf[:cut]
                break
            if m.start() > 0:
                self.stats.garbage_bytes += m.start()
                del self.buf[:m.start()]
                continue
            # 注意：对 bytearray 的 match 在删改 buffer 后会失效，必须先取值
            end = m.end()
            seg = int(m.group("seg"), 16)
            dist = int(m.group("dist"), 16)
            yaw = _to_signed32(int(m.group("yaw"), 16))
            wall = int(m.group("wall"), 16)
            fault = int(m.group("fault"), 16)
            del self.buf[:end]
            rec = SegmentRecord(
                segment=seg,
                distance_mm=dist,
                yaw_mdeg=yaw,
                wall=wall,
                fault=fault,
                ts=ts,
            )
            self.stats.records += 1
            out.append(rec)
        self.stats.partial_bytes = len(self.buf) if self.buf else 0
        return out

    def flush(self) -> List[SegmentRecord]:
        """输入结束：残留的不完整记录计入 partial。"""
        self.stats.partial_bytes = len(self.buf)
        return []


def _to_signed32(value: int) -> int:
    return value - (1 << 32) if value & 0x80000000 else value


# ------------------------------------------------------------------ 工具函数
def build_record(segment: int, distance_mm: int, yaw_mdeg: int,
                 wall: int, fault: int) -> bytes:
    """按 RTL 格式生成一条记录（自检/仿真用）。"""
    yaw_u = yaw_mdeg & 0xFFFFFFFF
    return ("T4 SEG=%X DIST=%08X YAW=%08X WALL=%02X FAULT=%02X\r\n"
            % (segment & 0xF, distance_mm & 0xFFFFFFFF, yaw_u,
               wall & 0xFF, fault & 0xFF)).encode("ascii")


def parse_bytes(data: bytes) -> tuple[List[SegmentRecord], ParseStats]:
    p = SegmentLogParser()
    recs = p.feed(data)
    return recs, p.stats


def records_from_file(path: str) -> tuple[List[SegmentRecord], ParseStats]:
    with open(path, "rb") as f:
        return parse_bytes(f.read())


def iter_records(chunks: Iterable[bytes]) -> List[SegmentRecord]:
    p = SegmentLogParser()
    out: List[SegmentRecord] = []
    for c in chunks:
        out.extend(p.feed(c))
    return out
