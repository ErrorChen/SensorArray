"""Shared parser for the SensorArray compact ASCII runtime protocol."""

from __future__ import annotations

import dataclasses
import math
import time
import zlib
from typing import Callable, Optional


INVALID_CAP_FIXED = -1_000_000


def parse_fields(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for item in line.split(",")[1:]:
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key] = value
    return fields


@dataclasses.dataclass
class CapFrame:
    sequence: int
    timestamp_us: int
    rows: int
    cells: int
    values_fixed: list[int]
    crc_ok: bool

    @property
    def values_pf(self) -> list[Optional[float]]:
        return [None if value == INVALID_CAP_FIXED else value / 1_000_000.0
                for value in self.values_fixed]


@dataclasses.dataclass
class ProtocolCounters:
    lines: int = 0
    cap_frames: int = 0
    crc_errors: int = 0
    malformed: int = 0
    sequence_gaps: int = 0
    missing_frames: int = 0
    non_ascii_chunks: int = 0
    summary_lines: int = 0


class TextProtocolParser:
    """Line parser with C/D/K frame assembly and transport-independent stats."""

    def __init__(self, on_cap_frame: Optional[Callable[[CapFrame], None]] = None) -> None:
        self.counters = ProtocolCounters()
        self.latest_fields: dict[str, dict[str, str]] = {}
        self.latest_battery_mv: Optional[int] = None
        self._on_cap_frame = on_cap_frame
        self._current_sequence: Optional[int] = None
        self._current_timestamp_us = 0
        self._current_rows = 8
        self._current_cells = 64
        self._chunks: dict[int, list[int]] = {}
        self._crc_bytes = bytearray()
        self._last_sequence: Optional[int] = None
        self._first_frame_time: Optional[float] = None
        self._last_frame_time: Optional[float] = None

    def note_non_ascii(self) -> None:
        self.counters.non_ascii_chunks += 1

    def feed_line(self, line: str) -> Optional[CapFrame]:
        line = line.rstrip("\r\n")
        if not line:
            return None
        self.counters.lines += 1
        tag = line.split(",", 1)[0]

        if tag == "C":
            return self._start_cap_frame(line)
        if len(tag) >= 2 and tag[0] == "D" and tag[1:].isdigit():
            self._add_cap_chunk(line, int(tag[1:]))
            return None
        if tag == "K":
            return self._finish_cap_frame(line)
        if tag in {"S50", "F50", "A50", "O50"}:
            fields = parse_fields(line)
            self.latest_fields[tag] = fields
            self.counters.summary_lines += 1
            if tag == "A50" and "bt" in fields:
                try:
                    self.latest_battery_mv = int(fields["bt"])
                except ValueError:
                    self.counters.malformed += 1
            return None
        return None

    def _start_cap_frame(self, line: str) -> None:
        fields = parse_fields(line)
        try:
            sequence = int(fields["seq"], 10)
            timestamp_us = int(fields.get("ts", "0"), 10)
            rows = int(fields.get("rows", "8"), 10)
            cells = int(fields.get("cells", fields.get("n", "64")), 10)
            count = int(fields.get("n", str(cells)), 10)
        except (KeyError, ValueError):
            self.counters.malformed += 1
            self._current_sequence = None
            return None
        if not 1 <= rows <= 8 or cells != rows * 8 or count != cells:
            self.counters.malformed += 1
            self._current_sequence = None
            return None

        self._current_sequence = sequence
        self._current_timestamp_us = timestamp_us
        self._current_rows = rows
        self._current_cells = cells
        self._chunks = {}
        self._crc_bytes = bytearray((line + "\n").encode("ascii"))
        return None

    def _add_cap_chunk(self, line: str, chunk_index: int) -> None:
        if self._current_sequence is None:
            self.counters.malformed += 1
            return
        try:
            values = [int(value, 10) for value in line.split(",")[1:]]
        except ValueError:
            self.counters.malformed += 1
            return
        if not values:
            self.counters.malformed += 1
            return
        self._chunks[chunk_index] = values
        self._crc_bytes.extend((line + "\n").encode("ascii"))

    def _finish_cap_frame(self, line: str) -> Optional[CapFrame]:
        fields = parse_fields(line)
        try:
            sequence = int(fields["seq"], 10)
            expected_crc = int(fields["crc"], 16)
        except (KeyError, ValueError):
            self.counters.malformed += 1
            return None
        if self._current_sequence != sequence:
            self.counters.malformed += 1
            return None

        calculated_crc = zlib.crc32(self._crc_bytes) & 0xFFFFFFFF
        crc_ok = calculated_crc == expected_crc
        if not crc_ok:
            self.counters.crc_errors += 1

        values: list[int] = []
        for chunk_index in sorted(self._chunks):
            values.extend(self._chunks[chunk_index])
        if len(values) != self._current_cells:
            self.counters.malformed += 1
            self._current_sequence = None
            return None

        frame = CapFrame(sequence, self._current_timestamp_us, self._current_rows,
                         self._current_cells, values, crc_ok)
        self.counters.cap_frames += 1
        self._record_sequence(sequence)
        self._current_sequence = None
        if self._on_cap_frame:
            self._on_cap_frame(frame)
        return frame

    def _record_sequence(self, sequence: int) -> None:
        now = time.monotonic()
        if self._first_frame_time is None:
            self._first_frame_time = now
        self._last_frame_time = now
        if self._last_sequence is not None and sequence > self._last_sequence + 1:
            self.counters.sequence_gaps += 1
            self.counters.missing_frames += sequence - self._last_sequence - 1
        elif self._last_sequence is not None and sequence <= self._last_sequence:
            self.counters.sequence_gaps += 1
        self._last_sequence = sequence

    def cap_fps(self) -> float:
        if (self._first_frame_time is None or self._last_frame_time is None or
                self._last_frame_time <= self._first_frame_time):
            return 0.0
        return (self.counters.cap_frames - 1) / (self._last_frame_time - self._first_frame_time)

    def summary(self, prefix: str) -> str:
        battery = "na" if self.latest_battery_mv is None else str(self.latest_battery_mv)
        return (
            f"{prefix},lines={self.counters.lines},cap={self.counters.cap_frames},"
            f"fps={self.cap_fps():.2f},gap={self.counters.sequence_gaps},"
            f"missing={self.counters.missing_frames},crc={self.counters.crc_errors},"
            f"bad={self.counters.malformed},ascii={self.counters.non_ascii_chunks},bt={battery}"
        )


def format_cap_preview(frame: CapFrame) -> str:
    valid = [value for value in frame.values_pf if value is not None and math.isfinite(value)]
    mean = sum(valid) / len(valid) if valid else math.nan
    first = frame.values_pf[0]
    last = frame.values_pf[-1]
    first_text = "invalid" if first is None else f"{first:.6f}"
    last_text = "invalid" if last is None else f"{last:.6f}"
    return (f"CAP,seq={frame.sequence},rows={frame.rows},cells={frame.cells},"
            f"crc={int(frame.crc_ok)},cap0={first_text},"
            f"capLast={last_text},mean={mean:.6f}")
