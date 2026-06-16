"""Shared parser for the SensorArray compact ASCII runtime protocol."""

from __future__ import annotations

import dataclasses
import math
import time
import zlib
from typing import Callable, Optional


INVALID_CAP_FIXED = -1_000_000
SUMMARY_TAGS = {"S50", "F50", "A50", "O50", "SF50", "TR50", "AB50", "OT50", "BL50", "I2C50"}


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


@dataclasses.dataclass
class FragmentStats:
    ok: int = 0
    missing: int = 0
    gap: int = 0
    crc_fail: int = 0
    dropped: int = 0
    tiny: int = 0
    duplicate: int = 0
    out_of_order: int = 0


class FragmentReassembler:
    """Reassembles legacy and current SensorArray G fragments."""

    def __init__(self) -> None:
        self.stats: dict[str, FragmentStats] = {
            "D": FragmentStats(),
            "L": FragmentStats(),
            "C": FragmentStats(),
        }
        self._messages: dict[tuple[str, int], dict[str, object]] = {}
        self._last_mid: dict[str, Optional[int]] = {"D": None, "L": None, "C": None}

    def feed(self, fallback_channel: str, payload: bytes) -> list[tuple[str, bytes]]:
        if not payload.startswith(b"G,"):
            return [(fallback_channel, payload)]
        header, sep, body = payload.partition(b"\n")
        if not sep:
            self._stat(fallback_channel).dropped += 1
            return []
        try:
            fields = header.decode("ascii").split(",")
            ch = fields[1]
            mid = int(fields[2], 10)
            frag_index = int(fields[3], 10)
            frag_count = int(fields[4], 10)
            payload_len = int(fields[5], 10)
            if len(fields) >= 8:
                message_len = int(fields[6], 10)
                expected_crc = int(fields[7], 16)
            else:
                message_len = 0
                expected_crc = int(fields[6], 16)
        except (IndexError, ValueError, UnicodeDecodeError):
            self._stat(fallback_channel).dropped += 1
            return []
        if ch not in self.stats or frag_count <= 0 or frag_index < 0 or frag_index >= frag_count:
            self._stat(fallback_channel).dropped += 1
            return []
        if len(body) != payload_len:
            self.stats[ch].dropped += 1
            return []
        if frag_count > 1 and payload_len <= 1:
            self.stats[ch].tiny += 1
        key = (ch, mid)
        item = self._messages.setdefault(
            key,
            {"n": frag_count, "crc": expected_crc, "mlen": message_len, "parts": {},
             "created": time.monotonic(), "last_index": -1},
        )
        if item["n"] != frag_count or item["crc"] != expected_crc or item["mlen"] != message_len:
            self.stats[ch].dropped += 1
            self._messages.pop(key, None)
            return []
        last_index = int(item.get("last_index", -1))
        if frag_index in item["parts"]:
            self.stats[ch].duplicate += 1
        if frag_index < last_index:
            self.stats[ch].out_of_order += 1
        item["last_index"] = frag_index
        parts = item["parts"]
        assert isinstance(parts, dict)
        parts[frag_index] = bytes(body)
        if len(parts) != frag_count:
            return []
        missing = [index for index in range(frag_count) if index not in parts]
        self._messages.pop(key, None)
        if missing:
            self.stats[ch].missing += len(missing)
            return []
        assembled = b"".join(parts[index] for index in range(frag_count))
        if message_len and len(assembled) != message_len:
            self.stats[ch].dropped += 1
            return []
        if (zlib.crc32(assembled) & 0xFFFFFFFF) != expected_crc:
            self.stats[ch].crc_fail += 1
            return []
        last = self._last_mid[ch]
        if last is not None and mid > last + 1:
            gap = mid - last - 1
            self.stats[ch].missing += gap
            self.stats[ch].gap += gap
        self._last_mid[ch] = mid
        self.stats[ch].ok += 1
        return [(ch, assembled)]

    def _stat(self, fallback_channel: str) -> FragmentStats:
        return self.stats.get(fallback_channel, self.stats["D"])

    def summary(self, ch: str) -> str:
        stats = self.stats[ch]
        return (f"BF,ch={ch},ok={stats.ok},ms={stats.missing},gap={stats.gap},"
                f"cf={stats.crc_fail},tiny={stats.tiny},dup={stats.duplicate},"
                f"ooo={stats.out_of_order},dr={stats.dropped}")


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
        if tag in SUMMARY_TAGS:
            fields = parse_fields(line)
            self.latest_fields[tag] = fields
            self.counters.summary_lines += 1
            if tag in {"A50", "AB50"} and "bt" in fields:
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
            f"bad={self.counters.malformed},ascii={self.counters.non_ascii_chunks},"
            f"sum={self.counters.summary_lines},bt={battery}"
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
