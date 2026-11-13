#!/usr/bin/env python3
"""Stream completed firmware benchmark recordings; never open a device.

The report describes observed transport/instrument behaviour, not calibration
accuracy. Only frames whose header and trailer both arrived inside the explicit
capture window enter configuration metrics. Re-run after another summary has
been finalized; incomplete summaries are listed without reading their raw file.
"""
from __future__ import annotations

import argparse
import base64
from bisect import bisect_right
from collections import Counter, defaultdict
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import sys
import tempfile
import time

from reference_fixture import IDENTITY_FIELDS, _identity, _observations, _trailer_matches
from text_protocol import FragmentReassembler, TextProtocolParser, parse_fields


TOOL = "analyze_firmware_benchmark"
MAX_LINE_BYTES = 131072
MODES = ("CAP", "VOLT", "RES", "MIXED")
ROW_MODE_NAMES = {"C": "CAP", "V": "VOLT", "R": "RES"}
MODE_UNITS = {"CAP": ("pF", -6), "VOLT": ("V", -6), "RES": ("ohm", -3)}
# Exact field order emitted by sensorarrayAsyncLog.c and main.c. Named suffix
# fields prove earlier numeric fields ended before a possible serial truncation.
STK50_FIELDS = ("unit", "seq", "log", "transport", "usb", "bleTx", "bleCtrl", "serialCtrl", "heap",
                "tSlot", "ta", "ts", "tr", "tq", "bSlot", "ba", "bs", "br", "bc", "bf", "bg", "bconf", "trunc")
PERF_FIELDS = ("pub", "fresh", "stale", "mixed", "dropOut", "dropEvent", "startAvgUs", "startN", "usbStage",
               "usbSent", "usbDrop", "usbBlock", "usbBytes", "usbWriteMaxUs", "usbQMax", "usbStackMin",
               "lifePub", "lifeDrop", "queueDrop", "frames", "heap", "heapMin")
USB_COUNTERS = ("sinkQueued", "sinkDropped", "priorityQueued", "driverAcceptedBytes",
                "rxBytes", "partialWrite", "timeout", "driverDrained", "rxOversize",
                "lineDropped", "lineOversize", "discardBytes")
PERF_COUNTERS = ("pub", "fresh", "stale", "mixed", "dropOut", "dropEvent",
                 "emitQueued", "usbSent", "usbDrop", "usbBlock", "usbBytes",
                 "lifePub", "lifeDrop", "queueDrop", "frames")
BLE_COUNTERS = ("mq", "ms", "md", "sentD", "sentL", "dropD", "dropL", "dropC",
                "ctrlRetry", "ctrlExhaust", "fs", "fe", "cg", "stale", "conf", "tiny")
PROVENANCE_TAGS = {"BUILD", "PROTO", "CONFIG", "MODE", "ROWS", "ROWMODES", "CAL",
                   "ARL", "RAIL", "ADS", "SPI", "ICLK", "SPICLK", "ADSCLK", "FRONTENDS",
                   "BAT", "BATPERIOD", "OFFSET", "BASELINE"}
COUNTER_SEMANTICS = {
    "PERF.pub": "Frame publish calls counted before async queue acceptance, including invalid/partial frames and subsequent drops; not ADC conversion count.",
    "PERF.emitQueued": "NOT_REPORTED unless the firmware actually supplies this field; no substitute is inferred.",
    "PERF.fresh": "Firmware freshness classification; partial CRC freshness is not a route-fault verdict.",
    "PERF.usbSent": "When usbStage=queued: packets submitted to the serial sink, not PC receipt or driver completion.",
    "PERF.usbBytes": "Bytes submitted at the USB sink stage identified by usbStage.",
    "PERF.usbDrop": "USB sink packet rejection counter; not necessarily equivalent to scientific frame gaps.",
    "PERF.dropOut": "Async output frame queue drops.",
    "PERF.dropEvent": "Async diagnostic event drops.",
    "PERF.queueDrop": "Transport descriptor queue drops; preserve separately from USB counters.",
    "USBSTREAM.sinkQueued": "Accepted serial sink slots (data, diagnostics and replies), not frames.",
    "USBSTREAM.sinkDropped": "Sink failures, including line-drop events; do not add lineDropped again.",
    "USBSTREAM.driverAcceptedBytes": "Bytes accepted by the ESP-IDF driver, not acknowledgement from the PC.",
    "USBSTREAM.driverDrained": "Observed successful wait_tx_done checks; neither bytes nor frames nor PC receipt.",
    "USBSTREAM.rxBytes": "Firmware command RX bytes; distinct from collector raw received bytes.",
    "USBSTREAM.lineDropped": "Line assembler drops; a scientific frame can have a different loss count.",
    "USBSTREAM.partialWrite": "A serial sink packet deadline expired after some bytes were accepted.",
    "USBSTREAM.timeout": "A serial sink packet was not completely accepted before its deadline.",
    "enumerated": "Preserve unknown; SOF presence does not prove host application receipt.",
    "BL50.mq": "Firmware BLE messages queued; keep separate from messages sent and host receipt.",
    "BL50.sentD": "Firmware data-message send success path after fragment sends; notifications do not acknowledge host application receipt.",
    "BL50.dropD": "Firmware BLE data-message drops, not interchangeable with host frame gaps.",
    "BL50.fs": "Firmware successful fragment-send paths; a different layer from journal notifications and reassembled frames.",
    "BLE.MTU": "Firmware BL50.mtu is preserved; backend_reported MTU is not treated as negotiated verification.",
}


def integer(value):
    try:
        return int(value, 0) if str(value).lower().startswith("0x") else int(value)
    except (TypeError, ValueError):
        return None


def memory_telemetry_record_error(tag, line):
    """Validate structure, never magnitude. Terminal PERF.heapMin stays unknown."""
    expected = STK50_FIELDS if tag == "STK50" else PERF_FIELDS
    tokens = [part.partition("=") for part in line.split(",")[1:]]
    if tuple(key for key, separator, value in tokens) != expected or any(not separator for _, separator, _ in tokens):
        return "INCOMPLETE_OR_UNRECOGNIZED_FIELD_LAYOUT"
    pairs = {"log", "transport", "usb", "bleTx", "bleCtrl", "serialCtrl", "tSlot", "tq", "bSlot"}
    for key, _, value in tokens:
        pattern = "bytes" if tag == "STK50" and key == "unit" else "queued" if tag == "PERF" and key == "usbStage" else r"[0-9]+"
        if tag == "STK50" and key in pairs:
            pattern = r"[0-9]+/[0-9]+"
        elif tag == "STK50" and key == "heap":
            pattern = r"[0-9]+/[0-9]+/[0-9]+"
        if not re.fullmatch(pattern, value):
            return "MALFORMED_FIELD_" + key
    return None


def file_hash(path):
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            size += len(chunk)
    return {"path": str(path), "bytes": size, "sha256": digest.hexdigest()}


def event_lines(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            try:
                yield json.loads(line)
            except (ValueError, TypeError) as exc:
                raise ValueError(f"invalid event JSON at line {number}") from exc


def event_metadata(path):
    starts, ends, count, next_offset = [], [], 0, 0
    last_rx_time = None
    for event in event_lines(path):
        count += 1
        if event.get("type") == "capture_start":
            starts.append(event.get("t"))
        elif event.get("type") == "capture_end":
            ends.append(event.get("t"))
        elif event.get("type") == "rx":
            offset, length = event.get("offset"), event.get("length")
            if not isinstance(offset, int) or not isinstance(length, int) or length <= 0 or offset != next_offset:
                raise ValueError("RX events must describe contiguous non-overlapping raw byte offsets")
            when = event.get("t")
            if not isinstance(when, (int, float)) or not math.isfinite(when) or (last_rx_time is not None and when < last_rx_time):
                raise ValueError("RX event timestamps must be finite and monotonic")
            last_rx_time = when
            next_offset += length
    valid = (len(starts) == len(ends) == 1 and isinstance(starts[0], (float, int)) and
             isinstance(ends[0], (float, int)) and math.isfinite(starts[0]) and math.isfinite(ends[0]) and ends[0] >= starts[0])
    return {"capture_start_seconds": starts[0] if valid else None,
            "capture_end_seconds": ends[0] if valid else None,
            "capture_window_status": "EXPLICIT" if valid else "UNAVAILABLE_OR_AMBIGUOUS",
            "events": count, "rx_bytes": next_offset}


class RxCursor:
    """Forward-only raw-offset to host receipt lookup; no full event array."""
    def __init__(self, path, window):
        self.events = (e for e in event_lines(path) if e.get("type") == "rx")
        self.current = None
        self.window = window
        self.bytes_by_phase = Counter()

    def at(self, offset):
        while self.current is None or offset >= self.current["offset"] + self.current["length"]:
            self.current = next(self.events, None)
            if self.current is None:
                return {"host_seconds": None, "phase": "unknown"}
            self.bytes_by_phase[location({"host_seconds": self.current.get("t")}, self.window)] += self.current["length"]
        if offset < self.current["offset"]:
            raise ValueError("non-monotonic raw offset lookup")
        return {"host_seconds": self.current.get("t"), "phase": self.current.get("phase", "unknown")}


def location(item, window):
    when = item.get("host_seconds")
    start, end = window["capture_start_seconds"], window["capture_end_seconds"]
    if when is None or start is None or end is None:
        return "unknown"
    if when < start:
        return "warmup"
    if when > end:
        return "cleanup"
    # Use time as the authority; event phase is retained for an audit of the
    # writer. The boundary timestamp, not a stale phase label, defines the run.
    return "capture"


class Snapshots:
    def __init__(self):
        self.count = 0
        self.first = self.last = self.before = self.after = None
        self.inside_first = self.inside_last = None

    def add(self, item, window):
        self.count += 1
        if self.first is None:
            self.first = item
        self.last = item
        phase = location(item, window)
        if phase == "warmup":
            self.before = item
        elif phase == "cleanup" and self.after is None:
            self.after = item
        elif phase == "capture":
            self.inside_first = self.inside_first or item
            self.inside_last = item

    def compact(self):
        return {"observed": self.count, "first": self.first, "last": self.last}

    def delta(self, keys, continuity, window_label="query_snapshots_not_exact_capture_boundaries"):
        first = self.before or self.inside_first
        last = self.after or self.inside_last
        result = {"status": "OBSERVED" if first and last and first is not last else "NOT_REPORTED",
                  "first": first, "last": last, "elapsed_seconds": None,
                  "window": window_label, "counters": {}}
        if first and last and first["host_seconds"] is not None and last["host_seconds"] is not None:
            result["elapsed_seconds"] = last["host_seconds"] - first["host_seconds"]
        for key in keys:
            a = integer(first["fields"].get(key)) if first else None
            b = integer(last["fields"].get(key)) if last else None
            reason, delta = None, None
            if a is None or b is None or first is last:
                reason = "NOT_REPORTED"
            elif not continuity:
                reason = "BOOT_CONTINUITY_UNVERIFIED"
            elif b < a:
                # Firmware counter widths vary. Do not turn an unproven reset
                # or wrap into a fabricated positive delta.
                reason = "COUNTER_DECREASE_OR_WRAP_UNRESOLVED"
            else:
                delta = b - a
            result["counters"][key] = {"before": a, "after": b, "delta": delta, "unavailable": reason}
        return result


def expected_wire_contract(declared, config):
    """Full configured profile selects the engine; active rows only mask M's tail."""
    mode, rows = declared.get("mode"), declared.get("rows")
    contract = {"requested_mode": mode, "rows": rows, "wire_tag": {"CAP": "C", "VOLT": "V", "RES": "R"}.get(mode),
                "full_configured_profile": None, "active_wire_profile": None, "configuration_errors": []}
    if mode != "MIXED":
        return contract
    profile = declared.get("profile")
    if not isinstance(profile, str) or not re.fullmatch(r"[CVR]{8}", profile):
        contract["configuration_errors"].append("REQUESTED_FULL_EIGHT_ROW_PROFILE_UNAVAILABLE")
        return contract
    contract["full_configured_profile"] = profile
    if _identity(config)[1] != "consistent_frame_identity":
        contract["configuration_errors"].append("BEFORE_CONFIG_COMPLETE_IDENTITY_UNAVAILABLE")
    homogeneous = len(set(profile)) == 1
    contract["full_profile_homogeneous"] = homogeneous
    contract["wire_tag"] = profile[0] if homogeneous else "M"
    expected_mode = ROW_MODE_NAMES[profile[0]] if homogeneous else "MIXED"
    if isinstance(rows, int) and 1 <= rows <= 8:
        contract["active_wire_profile"] = profile[:rows] + "N" * (8 - rows)
    for key, expected in (("mode", expected_mode), ("rows", str(rows)), ("profile", profile)):
        if config.get(key) != expected:
            contract["configuration_errors"].append("BEFORE_CONFIG_" + key.upper() + "_MISSING_OR_MISMATCH")
    contract["basis"] = "All eight configured modes determine homogeneous versus mixed engine; M profile uses N only after active rows."
    return contract


def frame_wire_contract_errors(frame, header, contract):
    if contract is None:
        return []
    errors = list(contract["configuration_errors"])
    if header["tag"] != contract["wire_tag"]:
        errors.append("WIRE_TAG_MISMATCH")
    if frame.rows != contract["rows"]:
        errors.append("ACTIVE_ROW_COUNT_MISMATCH")
    if header["tag"] == "M":
        if header.get("profile") != contract["active_wire_profile"]:
            errors.append("ACTIVE_WIRE_PROFILE_MISMATCH")
        if header.get("fmt") != "mix1":
            errors.append("MIXED_FORMAT_MISMATCH")
        profile = contract.get("full_configured_profile") or ""
        expected_rows = set(range(1, (contract.get("rows") or 0) + 1))
        if set(frame.row_frames) != expected_rows:
            errors.append("MIXED_ROW_SET_MISMATCH")
        for row, row_frame in frame.row_frames.items():
            expected_mode = ROW_MODE_NAMES.get(profile[row - 1] if row <= len(profile) else "")
            if row_frame.mode != expected_mode:
                errors.append(f"MR{row}_MODE_MISMATCH")
            if (row_frame.unit, row_frame.scale) != MODE_UNITS.get(expected_mode):
                errors.append(f"MR{row}_UNIT_OR_SCALE_MISMATCH")
    elif header["tag"] in {"V", "R"}:
        expected_mode = ROW_MODE_NAMES[header["tag"]]
        if (frame.unit, frame.scale) != MODE_UNITS[expected_mode]:
            errors.append("UNIT_OR_SCALE_MISMATCH")
    return errors


class Group:
    def __init__(self, header, frame, expected_identity=None, expected_contract=None):
        self.identity, self.identity_status = _identity(header)
        self.tag, self.rows = header["tag"], frame.rows
        self.profile = header.get("profile", "")
        self.counts = Counter({"trusted_complete_frames": 0, "integrity_valid_frames": 0,
                               "ordered_integrity_valid_frames": 0, "rejected_untrusted_frames": 0})
        self.errors = Counter()
        self.cell_errors = defaultdict(Counter)
        self.invalid_histogram = Counter()
        self.point_modes = Counter()
        self.first = self.last = None
        self.host_first = self.host_last = None
        self.trusted_host_first = self.trusted_host_last = None
        self.expected_identity = expected_identity
        self.expected_contract = expected_contract
        self.contract_errors = Counter()
        self.rejected_counts = Counter()
        self.rejected_errors = Counter()
        self.rejected_examples = []
        self.missing_trailer_fields = Counter()
        self.invalid_trailer_fields = Counter()
        self.progress = 0
        self.first_sequence = self.last_sequence = None
        self.gap_examples = []
        self.wire_bytes = self.payload_bytes = 0
        self.trusted_wire_bytes = self.trusted_payload_bytes = 0

    def add(self, frame, header, trailer, raw_rows, host, wire_bytes, payload_bytes, raw_trailer=None):
        self.counts["complete_frames"] += 1
        self.wire_bytes += wire_bytes
        self.payload_bytes += payload_bytes
        if self.host_first is None:
            self.host_first = host
        self.host_last = host
        missing_header = [key for key in IDENTITY_FIELDS if key not in header]
        missing_trailer = [key for key in ("session", "bootId", "rev") if key in header and key not in trailer]
        repeated_keys = ("seq", "session", "bootId", "rev", "gen", "rid", "rgen", "rrid", "pgen", "prid")
        invalid_trailer = {}
        for key in repeated_keys:
            if key not in trailer:
                continue
            value = trailer[key]
            if key == "session":
                valid = bool(re.fullmatch(r"[0-9A-Fa-f]{16}", value)) and int(value, 16) != 0
            else:
                number = integer(value)
                valid = bool(re.fullmatch(r"[0-9]+", value)) and number is not None and 0 <= number <= 0xFFFFFFFF
                if key == "rev":
                    valid = valid and number != 0
            if not valid:
                invalid_trailer[key] = value
        differing_trailer = {key: {"header": header[key], "trailer": trailer[key]}
                             for key in repeated_keys
                             if key in header and key in trailer and key not in invalid_trailer and header[key] != trailer[key]}
        differing_config = {key: {"expected_CONFIG": self.expected_identity[key], "header": header.get(key)}
                            for key in IDENTITY_FIELDS if self.expected_identity and header.get(key) != self.expected_identity.get(key)}
        contract_errors = frame_wire_contract_errors(frame, header, self.expected_contract)
        self.counts["wire_contract_errors"] += int(bool(contract_errors))
        self.contract_errors.update(contract_errors)
        identity_ok = (self.identity_status == "consistent_frame_identity" and
                       _trailer_matches(header, trailer) and not invalid_trailer and not differing_config)
        self.counts["ascii_crc_errors"] += int(not frame.crc_ok)
        self.counts["identity_errors"] += int(not identity_ok)
        self.counts["header_identity_missing"] += int(bool(missing_header))
        self.counts["header_identity_invalid"] += int(not missing_header and self.identity_status != "consistent_frame_identity")
        self.counts["trailer_identity_missing"] += int(bool(missing_trailer))
        self.counts["trailer_identity_invalid"] += int(bool(invalid_trailer))
        self.counts["trailer_header_identity_mismatch"] += int(bool(differing_trailer))
        self.counts["header_config_identity_mismatch"] += int(bool(differing_config))
        self.counts["identity_value_mismatch"] += int(bool(differing_trailer or differing_config))
        self.missing_trailer_fields.update(missing_trailer)
        self.invalid_trailer_fields.update(invalid_trailer.keys())
        if not frame.crc_ok or not identity_ok or contract_errors:
            self.counts["rejected_untrusted_frames"] += 1
            for _, _, _, _, _, valid, fresh, acquired, _, reason, _ in _observations(frame, header, raw_rows):
                self.rejected_counts["reported_points"] += 1
                self.rejected_counts["reported_invalid_points"] += int(not valid)
                self.rejected_counts["reported_not_fresh_points"] += int(not fresh)
                self.rejected_counts["reported_unacquired_points"] += int(not acquired)
                if not valid:
                    self.rejected_errors[f"X{reason:02X}" if reason >= 0 else "UNKNOWN_CAP_SENTINEL"] += 1
            if len(self.rejected_examples) < 8:
                self.rejected_examples.append({"sequence": frame.sequence, "host_seconds": host,
                    "ascii_crc_valid": frame.crc_ok, "missing_header_identity": missing_header,
                    "missing_trailer_identity": missing_trailer, "invalid_trailer_identity_fields": invalid_trailer,
                    "differing_trailer_identity": differing_trailer,
                    "differing_CONFIG_identity": differing_config, "wire_contract_errors": contract_errors,
                    "raw_K_ascii_with_newline": raw_trailer})
            return False
        self.counts["integrity_valid_frames"] += 1
        self.counts["trusted_complete_frames"] += 1
        self.trusted_wire_bytes += wire_bytes
        self.trusted_payload_bytes += payload_bytes
        if self.trusted_host_first is None:
            self.trusted_host_first = host
        self.trusted_host_last = host
        if self.last_sequence is not None:
            distance = (frame.sequence - self.last_sequence) & 0xFFFFFFFF
            if distance == 0 or distance >= 0x80000000:
                self.counts["duplicate_or_old_frames"] += 1
                return True
            if frame.timestamp_us < self.last:
                self.counts["timestamp_regressions"] += 1
                return True
            self.progress += distance
            if frame.sequence < self.last_sequence:
                self.counts["sequence_wraps"] += 1
            if distance > 1:
                self.counts["sequence_gaps"] += 1
                self.counts["missing_frames"] += distance - 1
                if len(self.gap_examples) < 16:
                    self.gap_examples.append({"after": self.last_sequence, "before": frame.sequence,
                                              "missing": distance - 1})
        if self.first is None:
            self.first, self.first_sequence = frame.timestamp_us, frame.sequence
        self.last, self.last_sequence = frame.timestamp_us, frame.sequence
        self.counts["ordered_integrity_valid_frames"] += 1
        invalid = stale = unacquired = unknown = 0
        for label, mode, unit, scale, fixed, valid, fresh, acquired, error, reason, known in _observations(frame, header, raw_rows):
            self.point_modes[mode] += 1
            self.counts["observed_points"] += 1
            self.counts["reported_valid_points"] += int(valid)
            invalid += int(not valid)
            stale += int(not fresh)
            unacquired += int(not acquired)
            unknown += int(not known)
            if not valid:
                code = f"X{reason:02X}" if reason >= 0 else "UNKNOWN_CAP_SENTINEL"
                self.errors[code] += 1
                self.cell_errors[label][code] += 1
            numeric = valid and fixed is not None and not error and reason == 0
            units_ok = (unit, scale) == MODE_UNITS.get(mode)
            self.counts["unit_errors"] += int(not units_ok)
            self.counts["usable_numeric_points"] += int(numeric and fresh and acquired and known and units_ok)
            self.counts["open_points"] += int(mode == "RES" and not valid and reason == 13)
            self.counts["inconsistent_points"] += int((valid and not numeric) or (not valid and (fixed is not None or not error)))
        self.counts["invalid_points"] += invalid
        self.counts["not_fresh_points"] += stale
        self.counts["unacquired_points"] += unacquired
        self.counts["unknown_masks_points"] += unknown
        self.counts["frames_with_invalid_points"] += int(invalid != 0)
        self.counts["frames_with_partial_freshness"] += int(stale != 0)
        self.counts["frames_with_unacquired_points"] += int(unacquired != 0)
        self.counts["fully_valid_fresh_acquired_frames"] += int(not (invalid or stale or unacquired or unknown))
        self.invalid_histogram[invalid] += 1
        return True

    def output(self):
        duration = (self.last - self.first) / 1e6 if self.first is not None else None
        parsed_host_duration = self.host_last - self.host_first if self.host_first is not None and self.host_last is not None else None
        host_duration = (self.trusted_host_last - self.trusted_host_first
                         if self.trusted_host_first is not None and self.trusted_host_last is not None else None)
        rates_known = duration and not self.counts["timestamp_regressions"] and not self.counts["duplicate_or_old_frames"]
        frames = self.counts["complete_frames"]
        trusted = self.counts["trusted_complete_frames"]
        return {"identity": self.identity, "identity_status": self.identity_status,
                "wire_tag": self.tag, "rows": self.rows, "profile": self.profile,
                "counts": dict(self.counts), "invalid_reasons": dict(self.errors),
                "wire_contract_error_reasons": dict(self.contract_errors),
                "counts_scope": "complete_frames is parsed framing only; trusted_complete_frames passes CRC, complete identity and the requested wire contract; point quality counts include only ordered trusted frames",
                "rejected_frame_diagnostics": {"reported_quality_counts": dict(self.rejected_counts),
                    "reported_invalid_reasons": dict(self.rejected_errors), "missing_trailer_fields": dict(self.missing_trailer_fields),
                    "invalid_trailer_fields": dict(self.invalid_trailer_fields),
                    "identity_mismatch_semantics": "Value mismatches are observed text disagreements, not proof of a device configuration change; truncated numeric tokens can still be syntactically valid.",
                    "examples": self.rejected_examples, "accepted_scientific_samples": False},
                "point_mode_counts": dict(self.point_modes),
                "invalid_reasons_by_cell": dict(self.cell_errors), "invalid_points_per_frame": dict(self.invalid_histogram),
                "sequence": {"first": self.first_sequence, "last": self.last_sequence,
                             "progress_including_missing": self.progress, "gap_examples": self.gap_examples,
                             "missing_semantics": "missing CRC-and-identity-valid sequence observations; rejected complete frames can contribute, so do not add this count to CRC errors as independent losses",
                             "arithmetic": "u32 modulo; distance 0 or >=2^31 is duplicate/old; never across identity groups"},
                "rates": {"device_span_seconds": duration, "host_receipt_span_seconds": host_duration,
                          "capture_fps": self.progress / duration if rates_known else None,
                          "host_complete_receive_fps": (trusted - 1) / host_duration if host_duration else None,
                          "host_parsed_frame_fps": (frames - 1) / parsed_host_duration if parsed_host_duration else None,
                          "host_parsed_receipt_span_seconds": parsed_host_duration,
                          "capture_basis": "ordered CRC-and-identity-valid frame sequence progression / device timestamp span",
                          "host_basis": "(trusted complete frames - 1) / first-to-last trusted K receipt time; bad CRC, incomplete/mismatched identity and wire-contract failures excluded"},
                "bytes": {"frame_wire_total": self.wire_bytes, "frame_wire_average": self.wire_bytes / frames if frames else None,
                          "trusted_frame_wire_average": self.trusted_wire_bytes / trusted if trusted else None,
                          "trusted_data_payload_ascii_average": self.trusted_payload_bytes / trusted if trusted else None,
                          "data_payload_ascii_total": self.payload_bytes,
                          "data_payload_ascii_average": self.payload_bytes / frames if frames else None,
                          "definition": "wire=header+data/PGA/row lines+K including newlines; payload=data/PGA/row lines; diagnostics excluded"}}


def requested_configuration(summary, label):
    kind = summary.get("benchmark_kind") or ("stability" if label.startswith("stability-") else "configuration")
    mode = summary.get("benchmark_mode") or summary.get("mode_requested")
    rows = integer(summary.get("rows_requested"))
    match = re.search(r"(?:^|-)(cap|volt|res|mixed)([1-8])(?:-|$)", label.lower())
    if match:
        mode = mode or match[1].upper()
        rows = rows or int(match[2])
    if not mode and summary.get("rowmodes_requested"):
        mode = "MIXED"
    return {"kind": kind, "mode": mode, "rows": rows,
            "profile": summary.get("rowmodes_requested") or summary.get("benchmark_profile"),
            "requested_seconds": summary.get("capture_requested_seconds", 600 if kind == "stability" else 120),
            "mixed1_semantics": "one_active_row_full_eight_row_profile_controls_wire_format" if mode == "MIXED" and rows == 1 else None}


def analyse_capture(directory, label, summary, fingerprints=None):
    summary_path = directory / f"{label}.summary.json"
    events_path = directory / f"{label}.events.jsonl"
    if not events_path.exists():
        events_path = directory / f"{label}.events.jsonl.gz"
    raw_path = directory / f"{label}.rx.bin"
    if not raw_path.exists():
        raw_path = directory / f"{label}.rx.bin.gz"
    window = event_metadata(events_path)
    cursor = RxCursor(events_path, window)
    parser = TextProtocolParser()
    expected_config = summary.get("snapshots", {}).get("before", {}).get("CONFIG?", {}).get("fields", {})
    declared = requested_configuration(summary, label)
    contract = expected_wire_contract(declared, expected_config)
    expected_identity = ({key: expected_config[key] for key in IDENTITY_FIELDS}
                         if all(key in expected_config for key in IDENTITY_FIELDS) else None)
    groups = {}
    phases = Counter()
    issues = Counter()
    snapshots = defaultdict(Snapshots)
    diagnostics = defaultdict(Counter)
    examples = defaultdict(list)
    boots = set()
    health_first = health_last = None
    health_entries = {}
    stack_minima = {}
    heap_minima = {}
    memory_accepted = defaultdict(Counter)
    memory_rejected = defaultdict(Counter)
    memory_rejected_examples = []
    digest = hashlib.sha256()
    offset = 0
    pending = None
    discarding_line = False
    opener = gzip.open if raw_path.suffix == ".gz" else open
    with opener(raw_path, "rb") as raw:
        while True:
            wire = raw.readline(MAX_LINE_BYTES + 1)
            if not wire:
                break
            digest.update(wire)
            start_offset, offset = offset, offset + len(wire)
            receipt = cursor.at(offset - 1)
            phase = location(receipt, window)
            if discarding_line or len(wire) > MAX_LINE_BYTES:
                issues["oversize_lines"] += int(not discarding_line)
                discarding_line = not wire.endswith(b"\n")
                continue
            if not wire.endswith(b"\n"):
                issues["unterminated_final_lines"] += 1
                continue
            try:
                line = wire.rstrip(b"\r\n").decode("ascii")
            except UnicodeDecodeError:
                issues["non_ascii_lines"] += 1
                parser.note_non_ascii()
                continue
            tag = line.split(",", 1)[0]
            fields = parse_fields(line)
            item = {**receipt, "byte_offset": start_offset, "fields": fields}
            telemetry_error = memory_telemetry_record_error(tag, line) if tag in {"STK50", "PERF"} else None
            if tag in {"STK50", "PERF"}:
                if telemetry_error:
                    memory_rejected[tag][phase] += 1
                    if len(memory_rejected_examples) < 16:
                        memory_rejected_examples.append({**receipt, "byte_offset": start_offset, "tag": tag,
                            "reason": telemetry_error, "raw_ascii_with_line_ending": wire.decode("ascii")})
                else:
                    memory_accepted[tag][phase] += 1
                    if tag == "PERF":
                        memory_rejected["PERF.heapMin"][phase] += 1
                        if len(memory_rejected_examples) < 16:
                            memory_rejected_examples.append({**receipt, "byte_offset": start_offset, "tag": "PERF.heapMin",
                                "reason": "TERMINAL_NUMERIC_FIELD_HAS_NO_FOLLOWING_MARKER_OR_CRC",
                                "unverified_token": fields["heapMin"]})
            counter_snapshot = ((tag == "PERF" and any(key in fields for key in PERF_COUNTERS)) or
                                (tag == "USBSTREAM" and any(key in fields for key in USB_COUNTERS)))
            if (tag in PROVENANCE_TAGS or counter_snapshot) and not (tag == "PERF" and telemetry_error):
                snapshots[tag].add(item, window)
            if (tag == "CONFIG" and re.fullmatch(r"[0-9A-Fa-f]{16}", fields.get("session", "")) and
                    int(fields["session"], 16) != 0 and integer(fields.get("bootId")) is not None and
                    0 <= integer(fields["bootId"]) <= 0xFFFFFFFF):
                boots.add((fields["session"], fields["bootId"]))
            is_reset = line.startswith(("RST,", "ESP-ROM:")) or "rst:0x" in line.lower() or "guru meditation" in line.lower()
            is_fault = tag in {"MFAULT", "MERR", "RMERR", "APP_FATAL", "FDC_FATAL"} or "watchdog got triggered" in line.lower()
            if is_reset or is_fault:
                kind = "RESET" if is_reset else "FAULT"
                diagnostics[kind][phase] += 1
                if len(examples[kind]) < 16:
                    examples[kind].append({**item, "line": line})
            if phase == "capture":
                if tag in {"ADSHEALTH", "ADSSPI", "BL50"}:
                    snapshots[tag].add(item, window)
                if tag == "ADSDISPOSITION":
                    diagnostics[tag]["records"] += 1
                    for key in ["boundedCrc", "abortError", "firstError", "allCrcRun", "limit", "scope"]:
                        diagnostics[tag][key + "=" + fields.get(key, "unknown")] += 1
                elif tag == "ADSSPIBYTES":
                    diagnostics[tag]["records"] += 1
                    try:
                        xor = int(fields["received"], 16) ^ int(fields["computed"], 16)
                        diagnostics[tag][f"xor={xor:02X}"] += 1
                    except (KeyError, ValueError):
                        diagnostics[tag]["malformed"] += 1
                elif tag == "ADSHEALTH":
                    diagnostics[tag]["records"] += 1
                    diagnostics[tag]["err=" + fields.get("err", "unknown")] += 1
                    health_first = health_first or item
                    health_last = item
                    entry, count = integer(fields.get("modeEntryFrameCount")), integer(fields.get("engineFrameCount"))
                    if entry is not None and count is not None:
                        entry_state = health_entries.setdefault(str(entry), {"first_engine_count": count, "last_engine_count": count,
                            "records": 0, "intervals": Counter(), "absolute_boundaries": {}, "entry_relative_boundaries": {}})
                        entry_state["last_engine_count"] = count
                        entry_state["records"] += 1
                        entry_state["intervals"][fields.get("healthInterval", "unknown")] += 1
                        for target in [128, 256, 384, 512]:
                            if count == target:
                                entry_state["absolute_boundaries"][str(target)] = fields
                            if count - entry == target:
                                entry_state["entry_relative_boundaries"][str(target)] = fields
                elif tag == "STK50" and telemetry_error is None:
                    for key in ["log", "transport", "usb", "bleTx", "bleCtrl", "serialCtrl"]:
                        try:
                            allocated, remaining = map(int, fields[key].split("/"))
                        except (KeyError, ValueError):
                            continue
                        if allocated > 0:
                            stack_minima[key] = min(stack_minima.get(key, remaining), remaining)
                    try:
                        initial, current, minimum = map(int, fields["heap"].split("/"))
                        heap_minima["observed_free_bytes"] = min(heap_minima.get("observed_free_bytes", current), current)
                        heap_minima["firmware_lifetime_minimum_bytes"] = min(heap_minima.get("firmware_lifetime_minimum_bytes", minimum), minimum)
                    except (KeyError, ValueError):
                        pass
            memory = tag == "M" and {"stage", "reason", "ih", "il", "im"} <= fields.keys() and not {"seq", "fmt", "rows", "cells", "profile"} & fields.keys()
            if memory:
                continue
            if tag in {"C", "V", "R", "M"}:
                if pending:
                    issues["truncated_frames_" + pending["phase"]] += 1
                pending = {"header": {"tag": tag, **fields}, "raw_rows": {}, "phase": phase,
                           "wire_bytes": len(wire), "payload_bytes": 0,
                           "digest": hashlib.sha256(wire) if fingerprints is not None else None}
            elif pending and (tag == "MR" or re.fullmatch(r"[DP][0-3]", tag)):
                pending["wire_bytes"] += len(wire)
                pending["payload_bytes"] += len(wire)
                if pending["digest"] is not None:
                    pending["digest"].update(wire)
                if tag == "MR":
                    try:
                        pending["raw_rows"][int(fields["s"])] = {"tokens": line.split(",D=", 1)[1].split(","),
                            "masks_known": "expected" in fields and "acquired" in fields}
                    except (KeyError, ValueError, IndexError):
                        pass
            frame = parser.feed_line(line)
            if tag == "K":
                if frame is not None and pending:
                    frame_phase = phase if phase == pending["phase"] else "boundary_straddling"
                    phases[frame_phase] += 1
                    header = pending["header"]
                    identity, status = _identity(header)
                    if frame.crc_ok and status == "consistent_frame_identity" and _trailer_matches(header, fields):
                        boots.add((identity["session"], identity["bootId"]))
                    if frame_phase == "capture":
                        key = json.dumps([identity, header["tag"], frame.rows, header.get("profile", "")], sort_keys=True)
                        group = groups.setdefault(key, Group(header, frame, expected_identity, contract))
                        trusted = group.add(frame, header, fields, pending["raw_rows"], receipt["host_seconds"],
                                            pending["wire_bytes"] + len(wire), pending["payload_bytes"], wire.decode("ascii"))
                        if fingerprints is not None and trusted:
                            pending["digest"].update(wire)
                            fingerprints.add(label, header, frame, pending["digest"].hexdigest())
                elif pending:
                    issues["malformed_frames_" + pending["phase"]] += 1
                pending = None
    if pending:
        issues["truncated_frames_" + pending["phase"]] += 1
    expected_hash = summary.get("raw_sha256")
    if digest.hexdigest() != expected_hash:
        raise ValueError("raw SHA256 does not match finalized collector summary")
    if offset != window["rx_bytes"]:
        raise ValueError("RX event byte coverage does not match complete raw recording")
    continuity = len(boots) == 1 and not sum(diagnostics["RESET"].values())
    outputs = [g.output() for g in groups.values()]
    counts = Counter()
    errors = Counter()
    for group in outputs:
        counts.update(group["counts"])
        errors.update(group["invalid_reasons"])
    observed_seconds = ((window["capture_end_seconds"] - window["capture_start_seconds"])
                        if window["capture_start_seconds"] is not None else None)
    required_seconds = declared["requested_seconds"]
    coverage = observed_seconds is not None and observed_seconds + 1e-6 >= float(required_seconds)
    matching = (bool(outputs) and not contract["configuration_errors"] and not counts["wire_contract_errors"] and
                all(g["wire_tag"] == contract["wire_tag"] and g["rows"] == declared["rows"] for g in outputs))
    record_status = "INCOMPLETE_OR_MISMATCH"
    if coverage and matching:
        record_status = ("RECORDED" if summary.get("status") in {"CAPTURED", "COMPLETED"} and
                         not counts["rejected_untrusted_frames"] else "RECORDED_WITH_ERRORS")
    after_config = summary.get("snapshots", {}).get("after", {}).get("CONFIG?", {}).get("fields", {})
    before_after_same = bool(expected_identity) and all(
        expected_config.get(key) == after_config.get(key) for key in (*IDENTITY_FIELDS, "mode", "rows", "profile"))
    harness_false_positive = (
        summary.get("status") == "ERROR" and summary.get("error") == "capture_frame_mode_or_units_mismatch" and
        declared["mode"] == "MIXED" and declared["rows"] == 1 and
        contract["wire_tag"] == "M" and contract["active_wire_profile"] == "CNNNNNNN" and
        coverage and matching and before_after_same and not counts["rejected_untrusted_frames"] and
        summary.get("wire_contract_mismatches") == counts["complete_frames"] and
        not sum(diagnostics["FAULT"].values()) and not sum(diagnostics["RESET"].values()))
    contract_review = {"classification": "HARNESS_FALSE_POSITIVE_REVALIDATED" if harness_false_positive else "NOT_RECLASSIFIED",
        "original_collector_error": summary.get("error"), "original_collector_wire_contract_mismatches": summary.get("wire_contract_mismatches"),
        "before_after_CONFIG_identity_mode_rows_profile_unchanged": before_after_same,
        "independent_wire_contract_errors": counts["wire_contract_errors"], "original_summary_modified": False}
    if harness_false_positive:
        contract_review["reason"] = "Collector inferred homogeneous CAP from active row count. Firmware selects the mixed engine from all eight configured modes; each observed M frame and CAP MR row matches the active profile and unchanged full CONFIG."
    return {"label": label, "requested": declared, "status": record_status,
            "collector_status": summary.get("status"), "collector_error": summary.get("error"),
            "expected_config_identity": expected_identity,
            "expected_wire_contract": contract, "collector_contract_revalidation": contract_review,
            "files": {"summary": file_hash(summary_path), "events": file_hash(events_path),
                "raw": {"path": str(raw_path), "uncompressed_bytes": offset, "uncompressed_sha256": digest.hexdigest(),
                        "summary_hash_matches": True, "compression": "gzip" if raw_path.suffix == ".gz" else "none"}},
            "window": {**window, "observed_seconds": observed_seconds, "requested_seconds_met": coverage,
                       "capture_monotonic_start": summary.get("capture_monotonic_start"),
                       "capture_monotonic_end": summary.get("capture_monotonic_end"),
                       "received_raw_bytes_by_phase": dict(cursor.bytes_by_phase),
                       "complete_frame_phases": dict(phases), "rule": "header and K receipts both inside explicit capture window; straddlers excluded"},
            "requested_configuration_observed": matching, "configuration_groups": outputs,
            "capture_counts": dict(counts), "capture_invalid_reasons": dict(errors),
            "quality_count_scope": "Only ordered frames passing CRC, complete identity and the requested wire contract enter point-quality totals; rejected frames remain separate. complete_frames is parsed framing, trusted_complete_frames is the trustworthy subset.",
            "whole_raw_parser_counters": asdict(parser.counters), "whole_raw_issues": dict(issues),
            "sequence_scope_note": "Use group sequence statistics; the legacy parser whole-raw sequence counters are not boot/configuration aware.",
            "diagnostics_capture_receipt_window": {k: dict(v) for k, v in diagnostics.items() if k not in {"FAULT", "RESET"}},
            "faults_and_resets": {k: {"counts_by_receipt_phase": dict(diagnostics[k]), "examples": examples[k]} for k in ["FAULT", "RESET"]},
            "ads_health": {"first": health_first, "last": health_last, "mode_entry_groups": health_entries,
                           "missing_boundary_records": "NOT_ASSUMED_SUCCESSFUL; targets absent from maps are NOT_OBSERVED",
                           "scope": "diagnostic receipt window; queued logs may predate the first captured data identity; original seq/entry retained"},
            "adc_counter_windows": {"health": snapshots["ADSHEALTH"].delta(("crcValidEven", "crcValidOdd", "crcErrors", "crcRetries"), continuity),
                                    "crc_events": snapshots["ADSSPI"].delta(("crcTotal", "retryTotal"), continuity)},
            "counter_boot_continuity_observed": continuity, "observed_boot_sessions": sorted(boots),
            "perf": snapshots["PERF"].delta(PERF_COUNTERS, continuity),
            "usb": snapshots["USBSTREAM"].delta(USB_COUNTERS, continuity),
            "ble_firmware": snapshots["BL50"].delta(BLE_COUNTERS, continuity,
                                                    "first_last_BL50_receipts_not_exact_capture_boundaries"),
            "provenance_queries": {tag: snapshots[tag].compact() for tag in sorted(PROVENANCE_TAGS) if snapshots[tag].count},
            "declared_query_snapshots": summary.get("snapshots", {}),
            "collector_replies": summary.get("replies", {}),
            "collector_configuration_and_restore": {key: summary.get(key) for key in ["mode_requested", "rows_requested", "rowmodes_requested", "usb_restore", "battery_restore", "calibration_written"]},
            "memory": {"stack_minimum_remaining_bytes": stack_minima, "heap": heap_minima,
                       "PERF_query_heap": [{"host_seconds": item["host_seconds"], "heap": item["fields"].get("heap"),
                                            "heapMin": None, "heapMin_status": "UNKNOWN_UNFRAMED_TERMINAL_FIELD",
                                            "usbStackMin": item["fields"].get("usbStackMin")}
                                           for item in (snapshots["PERF"].first, snapshots["PERF"].last) if item],
                       "telemetry_integrity": {
                           "accepted_record_layouts_by_receipt_phase": dict(memory_accepted),
                           "rejected_records_or_fields_by_receipt_phase": dict(memory_rejected),
                           "rejected_examples": memory_rejected_examples,
                           "policy": "STK50 memory requires the full emitted field layout through the named trunc suffix. PERF requires its full emitted layout; earlier heap/usbStackMin fields have following delimiters, but terminal heapMin has no following marker or CRC and remains unknown. No value-magnitude threshold is used; structural validation is not a checksum.",
                           "numeric_minima_source": "Only structurally complete capture-window STK50 records; unverified values remain diagnostic text."},
                       "USB_query_stack": [{"host_seconds": item["host_seconds"], "txStackBytes": item["fields"].get("txStackBytes"),
                                            "txStackMinBytes": item["fields"].get("txStackMinBytes")}
                                           for item in (snapshots["USBSTREAM"].first, snapshots["USBSTREAM"].last) if item],
                       "CPU_utilization": "NOT_RUN", "maximum_contiguous_heap_block": "NOT_RUN",
                       "stack_scope": "observed capture-window STK50 lifetime watermarks; PERF/USB query fields preserved separately"},
            "precision_acceptance": "NOT_EVALUATED", "partial_freshness_is_route_fault": False}


def fingerprint_key(header, frame):
    identity, _ = _identity(header)
    value = [identity, header["tag"], frame.rows, header.get("profile", "")]
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode("ascii")).hexdigest()


class Fingerprints:
    """Disposable disk index, so a long suite does not retain every frame."""
    def __init__(self, path):
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=OFF")
        self.db.execute("PRAGMA synchronous=OFF")
        self.db.execute("PRAGMA cache_size=-8192")
        self.db.execute("CREATE TABLE frames(label TEXT, identity TEXT, seq INTEGER, sha TEXT, seen INTEGER DEFAULT 0, mismatch INTEGER DEFAULT 0, PRIMARY KEY(label,identity,seq))")
        self.observed = defaultdict(Counter)
        self.examples = defaultdict(list)

    def add(self, label, header, frame, digest):
        self.db.execute("INSERT OR IGNORE INTO frames(label,identity,seq,sha) VALUES (?,?,?,?)",
                        (label, fingerprint_key(header, frame), frame.sequence, digest))

    def compare(self, label, header, frame, digest):
        key = (label, fingerprint_key(header, frame), frame.sequence)
        row = self.db.execute("SELECT sha,seen FROM frames WHERE label=? AND identity=? AND seq=?", key).fetchone()
        if row is None:
            self.observed[label]["BLE_frames_without_USB_match"] += 1
            return
        if row[1]:
            self.observed[label]["repeated_BLE_matching_identity_sequence"] += 1
        mismatch = row[0] != digest
        self.db.execute("UPDATE frames SET seen=1,mismatch=MAX(mismatch,?) WHERE label=? AND identity=? AND seq=?",
                        (int(mismatch), *key))
        if mismatch and len(self.examples[label]) < 16:
            self.examples[label].append({"sequence": frame.sequence, "USB_wire_sha256": row[0], "BLE_wire_sha256": digest})

    def result(self, label):
        total, seen, mismatch = self.db.execute("SELECT COUNT(*),COALESCE(SUM(seen),0),COALESCE(SUM(mismatch),0) FROM frames WHERE label=?", (label,)).fetchone()
        return {"USB_unique_integrity_valid_frames": total, "common_identity_sequence_frames": seen,
                "same_wire_bytes_sha256": seen - mismatch, "different_wire_bytes_sha256": mismatch,
                "USB_frames_without_BLE_match": total - seen, **dict(self.observed[label]),
                "mismatch_examples": self.examples[label],
                "unmatched_meaning": "No trusted fingerprint match; a partial or identity-rejected frame may still exist in the other raw recording. This is not a literal byte-absence or independent loss count.",
                "basis": "SHA256 of original scientific header/data/PGA/row/K bytes including line endings, after both CRC and identity checks; no CRC rewriting or sample reconstruction"}

    def close(self):
        self.db.close()


class BleWindow:
    def __init__(self, capture, start, end):
        self.capture, self.label = capture, capture["label"]
        self.start, self.end = start, end
        self.groups = {}
        self.counts = Counter()
        self.fragments = defaultdict(Counter)

    def add_frame(self, frame, pending, trailer, received_ns, fingerprints):
        header = pending["header"]
        identity, status = _identity(header)
        key = json.dumps([identity, header["tag"], frame.rows, header.get("profile", "")], sort_keys=True)
        group = self.groups.setdefault(key, Group(header, frame, self.capture.get("expected_config_identity"),
                                                self.capture.get("expected_wire_contract")))
        trusted = group.add(frame, header, trailer, pending["raw_rows"], received_ns / 1e9,
                            pending["wire_bytes"], pending["payload_bytes"], pending.get("raw_K"))
        if trusted:
            fingerprints.compare(self.label, header, frame, pending["digest"].hexdigest())

    def finish(self, ready_intervals, fingerprints, source_ok):
        overlaps = [(max(a, self.start), min(b, self.end)) for a, b in ready_intervals if a < self.end and b > self.start]
        coverage_ns = sum(b - a for a, b in overlaps)
        continuous = any(a <= self.start and b >= self.end for a, b in ready_intervals)
        reason = None
        if not source_ok:
            status, reason = "NOT_RUN", "JOURNAL_INTEGRITY_OR_SCHEMA_ERROR"
        elif not continuous:
            status = "NOT_RUN"
            reason = "NO_SUBSCRIBED_CONNECTION_DURING_WINDOW" if not coverage_ns else "PARTIAL_OR_DISCONNECTED_WINDOW"
        elif not self.groups:
            status, reason = "NO_COMPLETE_FRAMES", "SUBSCRIBED_WINDOW_WITH_NO_COMPLETE_DATA_FRAME"
        else:
            status = "RECORDED"
        groups = [g.output() for g in self.groups.values()]
        requested = self.capture.get("requested", {})
        contract = self.capture.get("expected_wire_contract") or expected_wire_contract(requested, {})
        requested_matches = (bool(groups) and not contract["configuration_errors"] and
                             all(g["wire_tag"] == contract["wire_tag"] and g["rows"] == requested.get("rows") and
                                 not g["counts"].get("wire_contract_errors") for g in groups))
        usb_identities = {json.dumps([g["identity"], g["wire_tag"], g["rows"], g["profile"]], sort_keys=True)
                          for g in self.capture.get("configuration_groups", [])}
        identity_subset = set(self.groups) <= usb_identities if usb_identities else None
        if status == "RECORDED" and (not requested_matches or identity_subset is False):
            status, reason = "CONFIGURATION_MISMATCH", "BLE_FRAME_CONFIGURATION_NOT_THE_REQUESTED_USB_CONFIGURATION"
        counts, errors = Counter(), Counter()
        for group in groups:
            counts.update(group["counts"])
            errors.update(group["invalid_reasons"])
        if status == "RECORDED" and counts["rejected_untrusted_frames"]:
            status = "RECORDED_WITH_ERRORS"
        comparison = fingerprints.result(self.label) if coverage_ns else {"status": "NOT_RUN", "reason": "NO_COMMON_CONNECTED_OBSERVATION_WINDOW"}
        if coverage_ns:
            comparison["scope"] = "full_capture_window" if continuous else "observed_partial_window_only"
        return {"status": status, "reason": reason,
                "requested_configuration_observed": requested_matches,
                "observed_identity_subset_of_USB": identity_subset,
                "alignment": {"clock": "same_host_time.monotonic_ns", "start_ns": self.start, "end_ns": self.end,
                              "subscribed_coverage_seconds": coverage_ns / 1e9, "continuous_subscriptions": continuous},
                "counts": dict(self.counts), "capture_frame_counts": dict(counts),
                "capture_invalid_reasons": dict(errors), "configuration_groups": groups,
                "fragments_by_channel": dict(self.fragments), "USB_comparison": comparison,
                "comparative_host_fps": (groups[0]["rates"]["host_complete_receive_fps"]
                                         if status in {"RECORDED", "RECORDED_WITH_ERRORS"} and len(groups) == 1 else None),
                "timestamp_semantics": "host notification callback completion, not over-air arrival or device send completion",
                "partial_window_metrics": "Observed partial records are preserved, but comparative FPS is unavailable when the requested connection window is incomplete"}


def ble_prefix_lines(path, evidence):
    """Read a fixed plain-file prefix, or a completed lossless gzip archive."""
    compressed = path.suffix == ".gz"
    stat = path.stat()
    initial_size = stat.st_size
    evidence.update({"snapshot_open_monotonic_ns": time.monotonic_ns(),
                     "snapshot_open_wall_utc": datetime.now(timezone.utc).isoformat(),
                     "storage_mtime_ns_at_open": stat.st_mtime_ns})
    opener = gzip.open if compressed else open
    digest = hashlib.sha256()
    size = 0
    with opener(path, "rb") as stream:
        while compressed or size < initial_size:
            limit = MAX_LINE_BYTES + 1 if compressed else min(MAX_LINE_BYTES + 1, initial_size - size)
            line = stream.readline(limit)
            if not line:
                break
            size += len(line)
            digest.update(line)
            if len(line) > MAX_LINE_BYTES:
                raise ValueError("BLE journal line exceeds bounded analysis size")
            if not line.endswith(b"\n"):
                evidence["incomplete_last_line_ignored"] = True
                break
            yield line
    evidence.update({"path": str(path), "compression": "gzip" if compressed else "none",
                     "storage_bytes_at_open": initial_size, "uncompressed_prefix_bytes": size,
                     "uncompressed_prefix_sha256": digest.hexdigest(),
                     "file_grew_during_analysis": path.stat().st_size != initial_size})


def analyse_ble_journal(path, captures, fingerprints):
    windows = []
    for capture in captures:
        window = capture.get("window", {})
        start, end = window.get("capture_monotonic_start"), window.get("capture_monotonic_end")
        if (isinstance(start, (int, float)) and isinstance(end, (int, float)) and
                math.isfinite(start) and math.isfinite(end) and end >= start):
            windows.append(BleWindow(capture, round(start * 1e9), round(end * 1e9)))
        else:
            capture["ble"] = {"status": "NOT_RUN", "reason": "ABSOLUTE_HOST_MONOTONIC_CAPTURE_WINDOW_UNAVAILABLE"}
    windows.sort(key=lambda item: item.start)
    if any(a.end > b.start for a, b in zip(windows, windows[1:])):
        raise ValueError("overlapping serial capture windows make BLE assignment ambiguous")
    starts = [w.start for w in windows]

    def at(ns):
        index = bisect_right(starts, ns) - 1
        return windows[index] if index >= 0 and ns <= windows[index].end else None

    source, journal_counts, metadata = {}, Counter(), []
    intervals = []
    connected = False
    ready_start = None
    last_ns = None
    reassembler = FragmentReassembler()
    fragment_starts = {}
    parser = TextProtocolParser()
    buffer = b""
    buffer_start = None
    pending = None
    schema_ok = True

    def retire_fragments(reason):
        for key, state in list(reassembler._messages.items()):
            owner = at(fragment_starts.get(key, last_ns))
            if owner:
                owner.fragments[key[0]]["incomplete_messages_" + reason] += 1
                owner.fragments[key[0]]["unobserved_parts_" + reason] += max(0, state["n"] - len(state["parts"]))
        reassembler._messages.clear()
        fragment_starts.clear()

    for raw_line in ble_prefix_lines(path, source):
        try:
            event = json.loads(raw_line)
            ns = event["monotonic_ns"]
            if event.get("schema_version") != 1 or not isinstance(ns, int) or (last_ns is not None and ns < last_ns):
                raise ValueError("BLE schema/timestamp mismatch")
        except (ValueError, KeyError, TypeError):
            schema_ok = False
            journal_counts["invalid_records"] += 1
            continue
        last_ns = ns
        kind = event.get("type")
        journal_counts[str(kind)] += 1
        owner = at(ns)
        if kind == "metadata":
            if event.get("collector") != "capture_benchmark_ble":
                schema_ok = False
            if len(metadata) < 4:
                metadata.append(event)
        elif kind == "connected":
            connected = True
        elif kind == "subscriptions_ready":
            if set(event.get("all_channels_subscribed", [])) != {"C", "D", "L"}:
                schema_ok = False
            if connected and ready_start is None:
                ready_start = ns
            if len(metadata) < 4:
                metadata.append(event)
        elif kind in {"disconnected", "recording_end"}:
            if ready_start is not None:
                intervals.append((ready_start, ns))
                ready_start = None
            connected = False
            if owner:
                owner.counts[kind] += 1
                owner.counts["unfinished_scientific_frames_at_disconnect"] += int(pending is not None)
            retire_fragments(kind)
            parser, buffer, buffer_start, pending = TextProtocolParser(), b"", None, None
        elif kind == "notification":
            channel = event.get("channel")
            try:
                payload = base64.b64decode(event["payload_base64"], validate=True)
                if channel not in {"D", "L", "C"} or len(payload) != event.get("length"):
                    raise ValueError("BLE channel or length mismatch")
            except (KeyError, ValueError, TypeError):
                schema_ok = False
                journal_counts["invalid_notifications"] += 1
                continue
            if owner:
                owner.counts["notifications_" + channel] += 1
                owner.counts["notification_bytes_" + channel] += len(payload)
            key = None
            effective = channel
            if payload.startswith(b"G,"):
                try:
                    parts = payload.split(b"\n", 1)[0].decode("ascii").split(",")
                    if parts[1] in {"D", "L", "C"}:
                        effective = parts[1]
                        key = (effective, int(parts[2]))
                        fragment_starts.setdefault(key, ns)
                except (IndexError, ValueError, UnicodeDecodeError):
                    pass
            before = asdict(reassembler.stats[effective])
            first_fragment_ns = fragment_starts.get(key, ns)
            messages = reassembler.feed(channel, payload)
            after = asdict(reassembler.stats[effective])
            if owner:
                owner.fragments[effective].update({key: after[key] - before[key] for key in before})
            if key not in reassembler._messages:
                fragment_starts.pop(key, None)
            if len(reassembler._messages) > 4096:
                # The shared parser has no expiry API. A documented analysis
                # bound protects long corrupt journals, without calling this
                # a proven radio loss or inventing message-ID gap counts.
                oldest = min(reassembler._messages, key=lambda item: fragment_starts.get(item, ns))
                state = reassembler._messages.pop(oldest)
                window = at(fragment_starts.pop(oldest, ns))
                if window:
                    window.fragments[oldest[0]]["incomplete_evicted_by_analysis_limit"] += 1
                    window.fragments[oldest[0]]["unobserved_parts_at_eviction"] += state["n"] - len(state["parts"])
            for logical_channel, message in messages:
                if owner:
                    owner.counts["logical_messages_" + logical_channel] += 1
                if logical_channel != "D":
                    if owner:
                        for diagnostic in message.splitlines():
                            if diagnostic.startswith((b"MFAULT,", b"MERR,", b"RMERR,", b"APP_FATAL,", b"FDC_FATAL,")):
                                owner.counts["fault_records_" + logical_channel] += 1
                            if diagnostic.startswith((b"RST,", b"ESP-ROM:")) or b"rst:0x" in diagnostic.lower():
                                owner.counts["reset_records_" + logical_channel] += 1
                    continue
                if not buffer:
                    buffer_start = first_fragment_ns
                buffer += message
                if len(buffer) > MAX_LINE_BYTES and b"\n" not in buffer:
                    journal_counts["oversize_data_lines"] += 1
                    buffer, buffer_start, pending = b"", None, None
                    continue
                while b"\n" in buffer:
                    line_bytes, _, buffer = buffer.partition(b"\n")
                    line_first_ns = buffer_start
                    buffer_start = first_fragment_ns if buffer else None
                    wire = line_bytes + b"\n"
                    try:
                        line = line_bytes.rstrip(b"\r").decode("ascii")
                    except UnicodeDecodeError:
                        if owner:
                            owner.counts["non_ascii_data_lines"] += 1
                        continue
                    tag = line.split(",", 1)[0]
                    fields = parse_fields(line)
                    if tag in {"C", "V", "R", "M"}:
                        if pending and owner:
                            owner.counts["truncated_frames"] += 1
                        pending = {"header": {"tag": tag, **fields}, "raw_rows": {}, "start_ns": line_first_ns,
                                   "wire_bytes": 0, "payload_bytes": 0, "digest": hashlib.sha256()}
                    scientific = tag in {"C", "V", "R", "M", "MR", "K"} or re.fullmatch(r"[DP][0-3]", tag)
                    if pending and scientific:
                        pending["wire_bytes"] += len(wire)
                        pending["digest"].update(wire)
                        if tag == "MR" or re.fullmatch(r"[DP][0-3]", tag):
                            pending["payload_bytes"] += len(wire)
                        if tag == "MR":
                            try:
                                pending["raw_rows"][int(fields["s"])] = {"tokens": line.split(",D=", 1)[1].split(","),
                                    "masks_known": "expected" in fields and "acquired" in fields}
                            except (KeyError, ValueError, IndexError):
                                pass
                    malformed_before = parser.counters.malformed
                    frame = parser.feed_line(line)
                    if owner:
                        owner.counts["malformed_lines"] += parser.counters.malformed - malformed_before
                    if tag == "K":
                        if pending:
                            pending["raw_K"] = wire.decode("ascii")
                        start_owner = at(pending["start_ns"]) if pending else None
                        if frame is not None and pending and owner is not None and start_owner is owner:
                            owner.add_frame(frame, pending, fields, ns, fingerprints)
                        elif frame is not None and pending and (owner or start_owner):
                            (owner or start_owner).counts["boundary_straddling_frames_excluded"] += 1
                        elif pending and owner:
                            owner.counts["malformed_frames"] += 1
                        pending = None
    if ready_start is not None and last_ns is not None:
        intervals.append((ready_start, last_ns))
    retire_fragments("journal_prefix_end")
    if pending and last_ns is not None and at(last_ns):
        at(last_ns).counts["unfinished_frames_at_prefix_end"] += 1
    if not journal_counts["metadata"]:
        schema_ok = False
    if source.get("incomplete_last_line_ignored") and (path.suffix == ".gz" or journal_counts["recording_end"]):
        schema_ok = False
    summary_name = path.name[:-3] if path.suffix == ".gz" else path.name
    summary_path = (path.with_name(summary_name.replace(".ble.notifications.jsonl", ".ble.summary.json"))
                    if summary_name.endswith(".ble.notifications.jsonl") else None)
    summary_evidence = {"status": "NOT_FINALIZED_OR_UNAVAILABLE"}
    if summary_path is not None and summary_path.exists():
        try:
            collector_summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (ValueError, OSError):
            collector_summary = None
        if isinstance(collector_summary, dict) and collector_summary.get("journal_sha256"):
            same_size = collector_summary.get("journal_bytes") == source["uncompressed_prefix_bytes"]
            same_hash = collector_summary["journal_sha256"] == source["uncompressed_prefix_sha256"]
            summary_evidence = {"status": "MATCH" if same_size and same_hash else "PREFIX_ONLY" if not journal_counts["recording_end"] else "MISMATCH",
                                "file": file_hash(summary_path), "collector_status": collector_summary.get("status"),
                                "journal_sha256": collector_summary["journal_sha256"], "journal_bytes": collector_summary.get("journal_bytes")}
            if summary_evidence["status"] == "MISMATCH":
                schema_ok = False
    for window in windows:
        window.capture["ble"] = window.finish(intervals, fingerprints, schema_ok)
    return {"source": source, "schema_version": 1, "schema_valid": schema_ok,
            "counts": dict(journal_counts), "metadata": metadata,
            "collector_summary_verification": summary_evidence,
            "recording_end_observed": bool(journal_counts["recording_end"]),
            "ready_intervals_monotonic_ns": intervals,
            "incomplete_fragment_meaning": "unobserved parts at an explicit disconnect/end/prefix boundary; never inferred from message-ID jumps",
            "maximum_inflight_reassembly_messages": 4096}


SELECTION_RULE = ("Keep the initial suite-01 run whenever its requested capture window completed, including runs with errors. "
                  "Only absent/short initial windows use the earliest complete fallback (for example retry-01). "
                  "All attempts remain in the report; FPS is never a selection criterion.")


def choose_capture(captures):
    completed = [capture for capture in captures if capture.get("window", {}).get("requested_seconds_met")]
    initial = [capture for capture in completed if capture["label"].startswith("suite-01-")]
    candidates = initial or completed
    def order(capture):
        when = capture.get("window", {}).get("capture_monotonic_start")
        return (when if isinstance(when, (int, float)) else math.inf, capture["label"])
    return min(candidates, key=order) if candidates else None


def compact_row(mode, rows, captures):
    matching = [c for c in captures if c.get("requested", {}).get("kind") != "stability" and
                c.get("requested", {}).get("mode") == mode and c.get("requested", {}).get("rows") == rows]
    if not matching:
        return {"mode": mode, "rows": rows, "status": "NOT_RUN", "selected_label": None, "captures": []}
    selected = choose_capture(matching)
    return {"mode": mode, "rows": rows, "status": selected["status"] if selected else "INCOMPLETE_OR_ERROR",
            "selected_label": selected["label"] if selected else None,
            "selection_rule": SELECTION_RULE,
            "mixed1_semantics": "one_active_row_full_eight_row_profile_controls_wire_format" if mode == "MIXED" and rows == 1 else None,
            "captures": [{"label": c["label"], "status": c["status"], "collector_status": c.get("collector_status"),
                          "collector_error": c.get("collector_error"), "counts": c.get("capture_counts", {}),
                          "collector_contract_revalidation": c.get("collector_contract_revalidation"),
                          "expected_wire_contract": c.get("expected_wire_contract"),
                          "ble": {key: c.get("ble", {}).get(key) for key in ["status", "reason", "comparative_host_fps", "capture_frame_counts", "USB_comparison"]},
                          "groups": [{"identity": g["identity"], "rates": g["rates"], "bytes": g["bytes"]} for g in c.get("configuration_groups", [])]}
                         for c in matching]}


def _analyse_directory(directory, ble_journal=None, fingerprints=None):
    captures = []
    for path in sorted(directory.glob("*.summary.json")):
        if path.name.endswith(".ble.summary.json"):
            continue
        label = path.name[:-len(".summary.json")]
        try:
            summary = json.loads(path.read_text(encoding="utf-8"))
        except (ValueError, OSError):
            captures.append({"label": label, "status": "NOT_RUN", "reason": "SUMMARY_NOT_FINALIZED"})
            continue
        declared = requested_configuration(summary, label)
        if not summary.get("raw_sha256") or summary.get("status") in {None, "RUNNING", "PREPARING"}:
            captures.append({"label": label, "requested": declared, "status": "NOT_RUN", "reason": "SUMMARY_NOT_FINALIZED"})
            continue
        try:
            captures.append(analyse_capture(directory, label, summary, fingerprints))
        except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
            captures.append({"label": label, "requested": declared, "status": "INPUT_ERROR", "reason": str(exc)})
    ble_result = {"status": "NOT_RUN", "reason": "NO_BLE_JOURNAL"}
    for capture in captures:
        capture["ble"] = {"status": "NOT_RUN", "reason": "NO_BLE_JOURNAL" if ble_journal is None else "SERIAL_CAPTURE_WINDOW_UNAVAILABLE"}
    if ble_journal is not None:
        try:
            ble_result = analyse_ble_journal(ble_journal, captures, fingerprints)
        except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
            ble_result = {"status": "INPUT_ERROR", "reason": str(exc), "path": str(ble_journal)}
            for capture in captures:
                capture["ble"] = {"status": "NOT_RUN", "reason": "BLE_JOURNAL_INPUT_ERROR"}
    stability_captures = [c for c in captures if c.get("requested", {}).get("kind") == "stability"]
    stability = [c["label"] for c in stability_captures]
    table = [compact_row(mode, rows, captures) for mode in MODES for rows in range(1, 9)]
    return {"schema_version": 1, "generator": TOOL, "generated_utc": datetime.now(timezone.utc).isoformat(),
            "tool_source": file_hash(Path(__file__).resolve()), "directory": str(directory),
            "parser_sources": [file_hash(Path(__file__).with_name(name)) for name in ["text_protocol.py", "reference_fixture.py"]],
            "evidence_type": "OFFLINE_ANALYSIS_OF_EXISTING_RECORDINGS", "hardware_actions": False,
            "calibration_or_precision_PASS": False, "counter_semantics": COUNTER_SEMANTICS,
            "ble_journal": ble_result,
            "stability": {"status": ("RECORDED" if any(c["status"] == "RECORDED" for c in stability_captures) else
                                       "INCOMPLETE_OR_ERROR" if stability else "NOT_RUN"),
                          "required_seconds": 600, "captures": stability},
            "configuration_table": table,
            "selection": {"rule": SELECTION_RULE, "selected_labels": {f"{row['mode']}{row['rows']}": row["selected_label"] for row in table}},
            "captures": captures,
            "limitations": ["ASCII CRC success does not establish ADC SPI integrity, freshness, or analog accuracy.",
                "RECORDED means requested time/configuration were observed; inspect invalid samples, faults and drops separately.",
                "Query counter intervals bracket their own timestamps, not exact capture boundaries.",
                "No rates or counter deltas are inferred across boots; group timestamps use microseconds.",
                "Counter decreases are left unresolved; sequence wraps use u32 half-range arithmetic.",
                "Raw ADC samples, CPU utilization, and maximum contiguous heap block cannot be reconstructed from absent fields."]}


def analyse_directory(directory, ble_journal=None):
    if ble_journal is None:
        candidates = sorted(directory.glob("*.ble.notifications.jsonl"))
        candidates += [path for path in sorted(directory.glob("*.ble.notifications.jsonl.gz"))
                       if not path.with_suffix("").exists()]
        if len(candidates) == 1:
            ble_journal = candidates[0]
    if ble_journal is None:
        return _analyse_directory(directory)
    with tempfile.TemporaryDirectory(prefix="sensorarray-benchmark-fingerprints-") as temporary:
        fingerprints = Fingerprints(Path(temporary) / "frames.sqlite")
        try:
            return _analyse_directory(directory, Path(ble_journal), fingerprints)
        finally:
            fingerprints.close()


def write_report(path, result, replace=False):
    if path.exists():
        if not replace:
            raise ValueError("output exists; use a new path or --replace for this tool's own report")
        try:
            previous = json.loads(path.read_text(encoding="utf-8"))
        except (ValueError, OSError) as exc:
            raise ValueError("refusing to replace a non-report file") from exc
        if previous.get("generator") != TOOL:
            raise ValueError("refusing to replace a file not generated by this tool")
    if path.name.endswith((".summary.json", ".events.jsonl", ".rx.bin", ".rx.bin.gz")):
        raise ValueError("output must not use a capture source filename")
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    try:
        if replace:
            temporary.replace(path)
        else:
            # link is an atomic no-clobber install, unlike check + rename.
            os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ble-journal", type=Path, help="notification journal .jsonl or .jsonl.gz; otherwise auto-detect exactly one schema-named journal")
    parser.add_argument("--selection-output", type=Path, help="also save an explicit 32-configuration label mapping for plotting/README")
    parser.add_argument("--replace", action="store_true", help="replace only a prior report generated by this tool")
    args = parser.parse_args(argv)
    try:
        if not args.directory.is_dir():
            raise ValueError("input directory does not exist")
        if args.selection_output and args.selection_output.resolve() == args.output.resolve():
            raise ValueError("selection output must differ from the main report")
        result = analyse_directory(args.directory, args.ble_journal)
        write_report(args.output, result, args.replace)
        if args.selection_output:
            selection = {"schema_version": 1, "generator": TOOL, "artifact_kind": "configuration_selection",
                         "source_report": str(args.output), "source_report_sha256": file_hash(args.output)["sha256"],
                         **result["selection"]}
            write_report(args.selection_output, selection, args.replace)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"output": str(args.output), "captures": len(result["captures"]),
                      "configuration_statuses": dict(Counter(x["status"] for x in result["configuration_table"])),
                      "precision_acceptance": "NOT_EVALUATED"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
