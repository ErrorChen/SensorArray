"""Shared parser for the SensorArray compact ASCII runtime protocol."""

from __future__ import annotations

import dataclasses
import math
import time
import zlib
from typing import Callable, Optional


INVALID_CAP_FIXED = -1_000_000
SUMMARY_TAGS = {
    "S50", "F50", "A50", "O50", "SF50", "TR50", "AB50", "OT50",
    "BL50", "I2C50", "ADS50", "ADST50", "ADSCHK", "ADSCHKSTAT",
    "ABAT", "BATPERIOD", "RESSETTLE",
}


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
    generation: int
    request_id: int
    row_fresh_mask: int
    primary_fresh_mask: int
    secondary_fresh_mask: int

    @property
    def values_pf(self) -> list[Optional[float]]:
        return [None if value == INVALID_CAP_FIXED else value / 1_000_000.0
                for value in self.values_fixed]


@dataclasses.dataclass
class MeasurementFrame:
    sequence: int
    timestamp_us: int
    rows: int
    cells: int
    mode: str
    unit: str
    scale: int
    values_fixed: list[Optional[int]]
    error_reasons: list[int]
    pga_gains: list[int]
    valid_mask: int
    fresh_mask: int
    error_mask: int
    reference: str
    rail_valid: bool
    generation: int
    request_id: int
    rail_age_frames: int
    frame_duration_us: int
    transition_duration_us: int
    gain_change_count: int
    overrange_count: int
    autorange_attempt_count: int
    autorange_fallback_count: int
    io_retry_count: int
    drdy_timeout_count: int
    stale_count: int
    spi_error_count: int
    crc_ok: bool

    @property
    def values_si(self) -> list[Optional[float]]:
        factor = 10.0 ** self.scale
        return [None if value is None else value * factor
                for value in self.values_fixed]


@dataclasses.dataclass
class ProtocolCounters:
    lines: int = 0
    cap_frames: int = 0
    voltage_frames: int = 0
    resistance_frames: int = 0
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
    """Line parser with C/V/R, D/P, K assembly and shared CRC validation."""

    def __init__(
        self,
        on_cap_frame: Optional[Callable[[CapFrame], None]] = None,
        on_measurement_frame: Optional[Callable[[MeasurementFrame], None]] = None,
    ) -> None:
        self.counters = ProtocolCounters()
        self.latest_fields: dict[str, dict[str, str]] = {}
        self.latest_battery_mv: Optional[int] = None
        self._on_cap_frame = on_cap_frame
        self._on_measurement_frame = on_measurement_frame
        self._current_tag: Optional[str] = None
        self._current_fields: dict[str, str] = {}
        self._current_sequence: Optional[int] = None
        self._current_timestamp_us = 0
        self._current_rows = 8
        self._current_cells = 64
        self._chunks: dict[int, list[int]] = {}
        self._measurement_chunks: dict[int, list[Optional[int]]] = {}
        self._error_chunks: dict[int, list[int]] = {}
        self._pga_chunks: dict[int, list[int]] = {}
        self._crc_bytes = bytearray()
        self._last_sequence: Optional[int] = None
        self._first_frame_time: Optional[float] = None
        self._last_frame_time: Optional[float] = None

    def note_non_ascii(self) -> None:
        self.counters.non_ascii_chunks += 1

    def feed_line(self, line: str) -> Optional[CapFrame | MeasurementFrame]:
        line = line.rstrip("\r\n")
        if not line:
            return None
        self.counters.lines += 1
        tag = line.split(",", 1)[0]

        if tag == "C":
            return self._start_cap_frame(line)
        if tag in {"V", "R"}:
            return self._start_measurement_frame(line, tag)
        if len(tag) >= 2 and tag[0] == "D" and tag[1:].isdigit():
            if self._current_tag == "C":
                self._add_cap_chunk(line, int(tag[1:]))
            elif self._current_tag in {"V", "R"}:
                self._add_measurement_chunk(line, int(tag[1:]))
            else:
                self.counters.malformed += 1
            return None
        if len(tag) >= 2 and tag[0] == "P" and tag[1:].isdigit():
            self._add_pga_chunk(line, int(tag[1:]))
            return None
        if tag == "K":
            if self._current_tag == "C":
                return self._finish_cap_frame(line)
            if self._current_tag in {"V", "R"}:
                return self._finish_measurement_frame(line)
            self.counters.malformed += 1
            return None
        if tag in SUMMARY_TAGS:
            fields = parse_fields(line)
            self.latest_fields[tag] = fields
            self.counters.summary_lines += 1
            if tag in {"A50", "AB50", "ABAT"} and "bt" in fields:
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
            self._current_tag = None
            return None
        if not 1 <= rows <= 8 or cells != rows * 8 or count != cells:
            self.counters.malformed += 1
            self._current_sequence = None
            self._current_tag = None
            return None

        self._current_sequence = sequence
        self._current_tag = "C"
        self._current_fields = fields
        self._current_timestamp_us = timestamp_us
        self._current_rows = rows
        self._current_cells = cells
        self._chunks = {}
        self._measurement_chunks = {}
        self._error_chunks = {}
        self._pga_chunks = {}
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

        header = self._current_fields
        try:
            generation = int(header.get("gen", "0"), 10)
            request_id = int(header.get("rid", "0"), 10)
            row_fresh = int(header.get("rf", "0"), 16)
            primary_fresh = int(header.get("pf", "0"), 16)
            secondary_fresh = int(header.get("sf", "0"), 16)
        except ValueError:
            self.counters.malformed += 1
            self._current_sequence = None
            self._current_tag = None
            return None
        frame = CapFrame(sequence, self._current_timestamp_us, self._current_rows,
                         self._current_cells, values, crc_ok, generation,
                         request_id, row_fresh, primary_fresh, secondary_fresh)
        self.counters.cap_frames += 1
        self._record_sequence(sequence)
        self._current_sequence = None
        self._current_tag = None
        if self._on_cap_frame:
            self._on_cap_frame(frame)
        return frame

    def _start_measurement_frame(self, line: str, tag: str) -> None:
        fields = parse_fields(line)
        try:
            sequence = int(fields["seq"], 10)
            timestamp_us = int(fields.get("ts", "0"), 10)
            rows = int(fields["rows"], 10)
            cells = int(fields["cells"], 10)
            count = int(fields.get("n", str(cells)), 10)
            scale = int(fields["scale"], 10)
            int(fields["valid"], 16)
            int(fields["fresh"], 16)
            int(fields["error"], 16)
        except (KeyError, ValueError):
            self.counters.malformed += 1
            self._current_sequence = None
            self._current_tag = None
            return None
        expected_mode = "VOLT" if tag == "V" else "RES"
        expected_unit = "V" if tag == "V" else "ohm"
        expected_scale = -6 if tag == "V" else -3
        if (not 1 <= rows <= 8 or cells != rows * 8 or count != cells or
                fields.get("mode") != expected_mode or
                fields.get("unit") != expected_unit or scale != expected_scale):
            self.counters.malformed += 1
            self._current_sequence = None
            self._current_tag = None
            return None
        self._current_sequence = sequence
        self._current_tag = tag
        self._current_fields = fields
        self._current_timestamp_us = timestamp_us
        self._current_rows = rows
        self._current_cells = cells
        self._chunks = {}
        self._measurement_chunks = {}
        self._error_chunks = {}
        self._pga_chunks = {}
        self._crc_bytes = bytearray((line + "\n").encode("ascii"))
        return None

    def _add_measurement_chunk(self, line: str, chunk_index: int) -> None:
        if self._current_sequence is None or self._current_tag not in {"V", "R"}:
            self.counters.malformed += 1
            return
        values: list[Optional[int]] = []
        reasons: list[int] = []
        try:
            for token in line.split(",")[1:]:
                if token.startswith("X") and len(token) == 3:
                    values.append(None)
                    reasons.append(int(token[1:], 16))
                else:
                    values.append(int(token, 10))
                    reasons.append(0)
        except ValueError:
            self.counters.malformed += 1
            return
        if not values or len(values) > 16:
            self.counters.malformed += 1
            return
        self._measurement_chunks[chunk_index] = values
        self._error_chunks[chunk_index] = reasons
        self._crc_bytes.extend((line + "\n").encode("ascii"))

    def _add_pga_chunk(self, line: str, chunk_index: int) -> None:
        if self._current_sequence is None or self._current_tag not in {"V", "R"}:
            self.counters.malformed += 1
            return
        parts = line.split(",", 1)
        packed = parts[1] if len(parts) == 2 else ""
        if not packed or len(packed) % 2 != 0:
            self.counters.malformed += 1
            return
        try:
            gains = [int(packed[index:index + 2], 16)
                     for index in range(0, len(packed), 2)]
        except ValueError:
            self.counters.malformed += 1
            return
        if len(gains) > 16 or any(gain not in {0, 1, 2, 4, 8, 16, 32}
                                  for gain in gains):
            self.counters.malformed += 1
            return
        self._pga_chunks[chunk_index] = gains
        self._crc_bytes.extend((line + "\n").encode("ascii"))

    def _finish_measurement_frame(self, line: str) -> Optional[MeasurementFrame]:
        fields = parse_fields(line)
        try:
            sequence = int(fields["seq"], 10)
            expected_crc = int(fields["crc"], 16)
        except (KeyError, ValueError):
            self.counters.malformed += 1
            return None
        if self._current_sequence != sequence or self._current_tag not in {"V", "R"}:
            self.counters.malformed += 1
            return None
        calculated_crc = zlib.crc32(self._crc_bytes) & 0xFFFFFFFF
        crc_ok = calculated_crc == expected_crc
        if not crc_ok:
            self.counters.crc_errors += 1

        values: list[Optional[int]] = []
        reasons: list[int] = []
        gains: list[int] = []
        for chunk_index in sorted(self._measurement_chunks):
            values.extend(self._measurement_chunks[chunk_index])
            reasons.extend(self._error_chunks.get(chunk_index, []))
        for chunk_index in sorted(self._pga_chunks):
            gains.extend(self._pga_chunks[chunk_index])
        if (len(values) != self._current_cells or
                len(reasons) != self._current_cells or
                len(gains) != self._current_cells):
            self.counters.malformed += 1
            self._current_sequence = None
            self._current_tag = None
            return None
        header = self._current_fields
        frame = MeasurementFrame(
            sequence=sequence,
            timestamp_us=self._current_timestamp_us,
            rows=self._current_rows,
            cells=self._current_cells,
            mode=header["mode"],
            unit=header["unit"],
            scale=int(header["scale"], 10),
            values_fixed=values,
            error_reasons=reasons,
            pga_gains=gains,
            valid_mask=int(header["valid"], 16),
            fresh_mask=int(header["fresh"], 16),
            error_mask=int(header["error"], 16),
            reference=header.get("ref", "NONE"),
            rail_valid=header.get("rail") == "1",
            generation=int(header.get("gen", "0"), 10),
            request_id=int(header.get("rid", "0"), 10),
            rail_age_frames=int(header.get("age", "0"), 10),
            frame_duration_us=int(header.get("dur", "0"), 10),
            transition_duration_us=int(header.get("tr", "0"), 10),
            gain_change_count=int(header.get("gc", "0"), 10),
            overrange_count=int(header.get("ov", "0"), 10),
            autorange_attempt_count=int(header.get("aa", "0"), 10),
            autorange_fallback_count=int(header.get("fb", "0"), 10),
            io_retry_count=int(header.get("ir", "0"), 10),
            drdy_timeout_count=int(header.get("to", "0"), 10),
            stale_count=int(header.get("st", "0"), 10),
            spi_error_count=int(header.get("spi", "0"), 10),
            crc_ok=crc_ok,
        )
        if self._current_tag == "V":
            self.counters.voltage_frames += 1
        else:
            self.counters.resistance_frames += 1
        self._record_sequence(sequence)
        self._current_sequence = None
        self._current_tag = None
        if self._on_measurement_frame:
            self._on_measurement_frame(frame)
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
            f"volt={self.counters.voltage_frames},res={self.counters.resistance_frames},"
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


def format_measurement_preview(frame: MeasurementFrame) -> str:
    valid = [value for value in frame.values_si if value is not None and math.isfinite(value)]
    mean = sum(valid) / len(valid) if valid else math.nan
    first = frame.values_si[0]
    last = frame.values_si[-1]
    first_text = "invalid" if first is None else f"{first:.9g}"
    last_text = "invalid" if last is None else f"{last:.9g}"
    return (f"{frame.mode},seq={frame.sequence},rows={frame.rows},cells={frame.cells},"
            f"unit={frame.unit},scale={frame.scale},crc={int(frame.crc_ok)},"
            f"v0={first_text},vLast={last_text},mean={mean:.9g}")
