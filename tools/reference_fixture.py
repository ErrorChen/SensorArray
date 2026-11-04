#!/usr/bin/env python3
"""Read-only reference-fixture assessment using the production ASCII parser.

CLI exit 0 means a report was produced without detected failures; NEEDS_LIMITS,
BASELINE_REQUIRED, SCREENING_INCOMPLETE and NOT_RUN are never acceptance PASS.
It sends no device commands and never writes calibration or input recordings.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
from typing import Iterable

from text_protocol import (CapFrame, MeasurementFrame, MixedFrame,
                           TextProtocolParser, parse_fields)

DEFAULT_FIXTURE = Path(__file__).resolve().parents[1] / "tests/fixtures/reference_10k_33pf.json"
COORDINATE = re.compile(r"S([1-8])D([1-8])\Z")
IDENTITY_FIELDS = ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid")
ALL_CELLS = tuple(f"S{row}D{col}" for row in range(1, 9) for col in range(1, 9))


def load_fixture(path: Path) -> dict:
    fixture = json.loads(path.read_text(encoding="utf-8"))
    if fixture.get("schema_version") != 1 or fixture.get("remaining_cells") != "floating":
        raise ValueError("fixture requires schema_version=1 and remaining_cells=floating")
    refs = fixture.get("references", {})
    if not refs:
        raise ValueError("fixture has no reference cells")
    for label, ref in refs.items():
        if not COORDINATE.fullmatch(label):
            raise ValueError(f"invalid coordinate {label!r}")
        expected_unit = {"resistor": "ohm", "capacitor": "pF"}.get(ref.get("kind"))
        if expected_unit is None or ref.get("unit") != expected_unit:
            raise ValueError(f"invalid kind/unit for {label}")
        for key in ("nominal", "measured", "reference_tolerance_abs", "acceptance_abs"):
            value = ref.get(key)
            if value is None and key != "nominal":
                continue
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or value < 0 or
                    (key in {"nominal", "measured"} and value == 0)):
                raise ValueError(f"invalid {label}.{key}")
    minimum = fixture.get("minimum_screening_seconds", 120.0)
    if isinstance(minimum, bool) or not isinstance(minimum, (int, float)) or not math.isfinite(minimum) or minimum <= 0:
        raise ValueError("minimum_screening_seconds must be finite and positive")
    return fixture


def describe_fixture(fixture: dict) -> dict:
    return {
        "name": fixture["name"], "source": fixture.get("source", "unknown"),
        "references": fixture["references"],
        "floating_cells": [label for label in ALL_CELLS if label not in fixture["references"]],
        "floating_count": 64 - len(fixture["references"]),
        "suggested_mixed_profile": "RCCCCCCR",
        "one_row_coverage": [label for label in fixture["references"] if label.startswith("S1")],
        "minimum_screening_seconds": fixture.get("minimum_screening_seconds", 120.0),
        "calibration_writes": False,
    }


def summary(values: list[float]) -> dict:
    if not values:
        return {"samples": 0, "mean": None, "median": None, "stddev": None,
                "min": None, "max": None, "range": None}
    return {"samples": len(values), "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "stddev": statistics.pstdev(values), "min": min(values),
            "max": max(values), "range": max(values) - min(values)}


@dataclass
class CellSamples:
    observed: int = 0
    invalid: int = 0
    stale: int = 0
    unacquired: int = 0
    masks_unknown: int = 0
    rejected_crc: int = 0
    rejected_identity: int = 0
    inconsistent: int = 0
    unit_errors: int = 0
    open_samples: int = 0
    unexpected_res_values: int = 0
    reasons: Counter = field(default_factory=Counter)
    values: list[float] = field(default_factory=list)

    def output(self) -> dict:
        fields = {key: value for key, value in asdict(self).items() if key != "values"}
        fields["reasons"] = dict(self.reasons)
        fields.update(summary(self.values))
        return fields

    def acquisition_errors(self) -> int:
        return (self.stale + self.unacquired + self.masks_unknown + self.rejected_crc +
                self.rejected_identity + self.inconsistent + self.unit_errors)


@dataclass
class CaptureGroup:
    identity: dict
    identity_status: str
    frame_type: str
    rows: int
    profile: str
    frames: int = 0
    duplicate_or_old_frames: int = 0
    sequence_gaps: int = 0
    missing_sequences: int = 0
    timestamp_regressions: int = 0
    first_sequence: int | None = None
    last_sequence: int | None = None
    first_timestamp_us: int | None = None
    last_timestamp_us: int | None = None
    cells: dict[tuple[str, str], CellSamples] = field(default_factory=dict)

    def cell(self, label: str, mode: str) -> CellSamples:
        return self.cells.setdefault((label, mode), CellSamples())

    def add_frame(self, frame) -> bool:
        self.frames += 1
        sequence = frame.sequence
        if self.last_sequence is not None:
            distance = (sequence - self.last_sequence) & 0xFFFFFFFF
            if distance == 0 or distance >= 0x80000000:
                self.duplicate_or_old_frames += 1
                return False
            if distance > 1:
                self.sequence_gaps += 1
                self.missing_sequences += distance - 1
        if self.last_timestamp_us is not None and frame.timestamp_us < self.last_timestamp_us:
            self.timestamp_regressions += 1
            return False
        if self.first_sequence is None:
            self.first_sequence, self.first_timestamp_us = sequence, frame.timestamp_us
        self.last_sequence, self.last_timestamp_us = sequence, frame.timestamp_us
        return True

    def duration(self) -> float:
        if self.first_timestamp_us is None or self.last_timestamp_us is None:
            return 0.0
        return (self.last_timestamp_us - self.first_timestamp_us) / 1_000_000.0


@dataclass
class Capture:
    groups: list[CaptureGroup]
    counters: dict
    truncated_frames: int = 0
    malformed_frames: int = 0
    identity_errors: int = 0
    non_ascii_lines: int = 0
    source: dict = field(default_factory=dict)
    fault_events: list[str] = field(default_factory=list)
    fault_event_count: int = 0
    memory_diagnostic_lines: int = 0


def _identity(header: dict) -> tuple[dict, str]:
    identity = {key: header.get(key) for key in IDENTITY_FIELDS}
    identity.update({"legacy_gen": header.get("gen"), "legacy_rid": header.get("rid")})
    present = all(identity[key] is not None for key in IDENTITY_FIELDS)
    session = identity.get("session") or ""
    try:
        numeric = all(0 <= int(identity[key]) <= 0xFFFFFFFF for key in IDENTITY_FIELDS if key != "session") if present else False
        valid = bool(re.fullmatch(r"[0-9A-Fa-f]{16}", session)) and int(session, 16) != 0 and numeric and int(identity["rev"]) != 0
    except (TypeError, ValueError):
        valid = False
    return identity, "consistent_frame_identity" if valid else "unverified_identity"


def _trailer_matches(header: dict, trailer: dict) -> bool:
    # K is outside measurement CRC: verify all repeated identity fields.
    keys = ("seq", "session", "bootId", "rev", "gen", "rid", "rgen", "rrid", "pgen", "prid")
    required = [key for key in ("session", "bootId", "rev") if key in header]
    if any(key not in trailer for key in required):
        return False
    return all(header[key] == trailer[key] for key in keys if key in header and key in trailer)


def _observations(frame, header: dict, raw_rows: dict):
    if isinstance(frame, MixedFrame):
        for row, row_frame in frame.row_frames.items():
            raw = raw_rows.get(row, {})
            tokens = raw.get("tokens", [])
            for col, fixed in enumerate(row_frame.values_fixed, 1):
                token = tokens[col - 1] if len(tokens) == 8 else ""
                try:
                    reason = int(token[1:], 16) if token.startswith("X") else 0
                except ValueError:
                    reason = -1
                yield (f"S{row}D{col}", row_frame.mode, row_frame.unit, row_frame.scale,
                       fixed, bool(row_frame.valid_mask & (1 << (col - 1))),
                       bool(row_frame.fresh_mask & (1 << (col - 1))),
                       bool(row_frame.acquired_mask & (1 << (col - 1))),
                       bool(row_frame.error_mask & (1 << (col - 1))), reason,
                       "expected" in header and "acquired" in header and raw.get("masks_known", False))
    elif isinstance(frame, CapFrame):
        for index, fixed in enumerate(frame.values_fixed):
            row, col = index // 8 + 1, index % 8 + 1
            bit = 1 << (row - 1)
            bank_fresh = frame.primary_fresh_mask if col <= 4 else frame.secondary_fresh_mask
            yield (f"S{row}D{col}", "CAP", "pF", -6,
                   None if fixed == -1_000_000 else fixed, fixed != -1_000_000,
                   bool(frame.row_fresh_mask & bank_fresh & bit),
                   bool(frame.acquired_mask & (1 << index)), fixed == -1_000_000,
                   0 if fixed != -1_000_000 else -1,
                   "expected" in header and "acquired" in header)
    else:
        for index, fixed in enumerate(frame.values_fixed):
            bit = 1 << index
            yield (f"S{index // 8 + 1}D{index % 8 + 1}", frame.mode, frame.unit, frame.scale,
                   fixed, bool(frame.valid_mask & bit), bool(frame.fresh_mask & bit),
                   bool(frame.acquired_mask & bit), bool(frame.error_mask & bit),
                   frame.error_reasons[index], "expected" in header and "acquired" in header)


def analyse_capture(lines: Iterable[str], source: dict | None = None) -> Capture:
    parser = TextProtocolParser()
    groups: dict[str, CaptureGroup] = {}
    header: dict = {}
    raw_rows: dict = {}
    pending = False
    truncated = malformed_frames = identity_errors = non_ascii = fault_event_count = 0
    fault_events = []
    memory_diagnostic_lines = 0
    for raw_line in lines:
        line = raw_line.rstrip("\r\n")
        try:
            line.encode("ascii")
        except UnicodeEncodeError:
            non_ascii += 1
            continue
        tag = line.split(",", 1)[0]
        if tag == "M":
            memory = parse_fields(line)
            # Legacy firmware used M for heap diagnostics before mixed frames
            # acquired that tag. Recognize only the explicit complete memory
            # record, never a malformed M scientific header with seq/fmt/rows.
            if ({"stage", "reason", "ih", "il", "im"} <= memory.keys() and
                    not {"seq", "fmt", "rows", "cells", "profile", "rgen"} & memory.keys() and
                    memory["stage"] and memory["reason"] and
                    all(memory[key].isdigit() for key in ("ih", "il", "im"))):
                memory_diagnostic_lines += 1
                continue
        if tag in {"MFAULT", "MERR", "RMERR", "APP_FATAL", "FDC_FATAL"} or any(
                marker in line.lower() for marker in ("guru meditation", "task watchdog got triggered", "interrupt wdt timeout")):
            fault_event_count += 1
            if len(fault_events) < 16:
                fault_events.append(line)
        if tag in {"C", "V", "R", "M"}:
            truncated += int(pending)
            pending, header, raw_rows = True, parse_fields(line), {}
            header["tag"] = tag
        elif tag == "MR" and pending:
            try:
                row_fields = parse_fields(line)
                raw_rows[int(row_fields["s"])] = {"tokens": line.split(",D=", 1)[1].split(","),
                                                   "masks_known": "expected" in row_fields and "acquired" in row_fields}
            except (KeyError, ValueError, IndexError):
                pass  # Production parser supplies the malformed counter.
        frame = parser.feed_line(line)
        if tag != "K":
            continue
        had_pending, pending = pending, False
        if frame is None:
            malformed_frames += int(had_pending)
            continue
        identity, identity_status = _identity(header)
        key = json.dumps([identity, header.get("tag"), frame.rows, header.get("profile", "")], sort_keys=True)
        group = groups.setdefault(key, CaptureGroup(identity, identity_status,
                                  header.get("tag", "?"), frame.rows, header.get("profile", "")))
        identity_ok = _trailer_matches(header, parse_fields(line))
        if frame.crc_ok and identity_ok:
            order_ok = group.add_frame(frame)
        else:
            group.frames += 1
            order_ok = False
        identity_errors += int(not identity_ok)
        for label, mode, unit, scale, fixed, valid, fresh, acquired, error, reason, masks_known in _observations(frame, header, raw_rows):
            point = group.cell(label, mode)
            point.observed += 1
            point.invalid += int(not valid)
            point.reasons[f"X{reason:02X}" if reason >= 0 else "unknown"] += int(not valid)
            if not frame.crc_ok:
                point.rejected_crc += 1
                continue
            if not identity_ok or not order_ok:
                point.rejected_identity += 1
                continue
            point.stale += int(not fresh)
            point.unacquired += int(not acquired)
            point.masks_unknown += int(not masks_known)
            if not fresh or not acquired or not masks_known:
                continue
            if (unit, scale) != {"CAP": ("pF", -6), "RES": ("ohm", -3), "VOLT": ("V", -6)}.get(mode):
                point.unit_errors += 1
                continue
            numeric_valid = valid and fixed is not None and not error and reason == 0
            open_valid = mode == "RES" and not valid and fixed is None and error and reason == 13
            if numeric_valid:
                point.values.append(float(fixed) * (10.0 ** scale))
                point.unexpected_res_values += int(mode == "RES")
            elif open_valid:
                point.open_samples += 1
            elif valid or fixed is not None or not error:
                point.inconsistent += 1
    return Capture(list(groups.values()), asdict(parser.counters), truncated + int(pending),
                   malformed_frames, identity_errors, non_ascii, source or {}, fault_events, fault_event_count, memory_diagnostic_lines)


def read_capture(path: Path) -> Capture:
    # Lossless byte hash; non-ASCII bytes stay visible as rejected lines.
    payload = path.read_bytes()
    return analyse_capture(payload.decode("ascii", errors="surrogateescape").splitlines(),
                           {"path": str(path), "sha256": hashlib.sha256(payload).hexdigest(),
                            "bytes": len(payload)})


def _point_status(point: CellSamples | None, identity_status: str, floating=False) -> str:
    if point is None or point.observed == 0:
        return "NOT_OBSERVED"
    if point.acquisition_errors() or (point.invalid and not floating):
        return "DATA_QUALITY_FAILED"
    if floating and (point.open_samples != point.observed or point.values):
        return "FAIL"
    if identity_status != "consistent_frame_identity":
        return "UNVERIFIED_IDENTITY"
    return "PASS" if floating else "READY"


def _reference_result(label: str, ref: dict, group: CaptureGroup, baseline: Capture | None) -> dict:
    mode = "RES" if ref["kind"] == "resistor" else "CAP"
    point = group.cells.get((label, mode))
    result = {"mode": mode, "unit": ref["unit"], "reference": ref,
              "statistics": point.output() if point else CellSamples().output()}
    status = _point_status(point, group.identity_status)
    result["accuracy"] = status
    result["delta_from_nominal"] = result["delta_from_reference"] = result["relative_error_percent"] = None
    if status == "NOT_OBSERVED":
        return result
    result["data_quality"] = status
    values = point.values if point else []
    if not values:
        return result
    evaluated = list(values)
    if mode == "CAP":
        candidates = [(index, candidate, candidate.cells.get((label, "CAP")))
                      for index, candidate in enumerate(baseline.groups if baseline else [])
                      if candidate.rows == group.rows and candidate.cells.get((label, "CAP"))]
        if not candidates:
            result["accuracy"] = "BASELINE_REQUIRED" if status == "READY" else status
            return result
        if len(candidates) != 1:
            result["accuracy"] = "BASELINE_AMBIGUOUS"
            return result
        index, candidate, unloaded = candidates[0]
        result["baseline"] = {"group_index": index, "identity": candidate.identity,
                              "statistics": unloaded.output(),
                              "conditions": "user_declared_same_cell_unloaded; wiring/range/temperature not independently verified"}
        if (baseline.truncated_frames or baseline.malformed_frames or baseline.identity_errors or
                baseline.non_ascii_lines or baseline.fault_event_count or baseline.counters["malformed"] or baseline.counters["crc_errors"] or
                _point_status(unloaded, candidate.identity_status) != "READY" or not unloaded.values):
            result["accuracy"] = "BASELINE_QUALITY_FAILED"
            return result
        # Exact same-cell subtraction only. No other floating channel is used.
        baseline_mean = statistics.fmean(unloaded.values)
        evaluated = [value - baseline_mean for value in values]
        result["baseline_subtracted"] = summary(evaluated)
        result["baseline_uncertainty_not_propagated"] = True
    reference = ref["measured"] if ref.get("measured") is not None else ref["nominal"]
    measured_mean = statistics.fmean(evaluated)
    result["delta_from_nominal"] = measured_mean - ref["nominal"]
    result["delta_from_reference"] = measured_mean - reference
    result["relative_error_percent"] = 100.0 * (measured_mean - reference) / reference
    result["max_abs_error"] = max(abs(value - reference) for value in evaluated)
    limit = ref.get("acceptance_abs")
    if status != "READY":
        result["accuracy"] = status
    elif limit is None:
        result["accuracy"] = "NEEDS_LIMITS"
    else:
        # Every retained sample must be within the explicit limit; spikes are
        # not smoothed away by a mean-only acceptance test.
        result["accuracy"] = "PASS" if result["max_abs_error"] <= limit else "FAIL"
    return result


def evaluate_fixture(fixture: dict, capture: Capture | None = None,
                     baseline: Capture | None = None, provenance: str = "unknown") -> dict:
    report = {"schema_version": 1, "fixture": describe_fixture(fixture),
              "provenance": provenance, "physical_calibration": "NOT_RUN",
              "calibration_coefficients_written": False,
              "status": "NOT_RUN" if capture is None else "REVIEW_REQUIRED",
              "groups": [], "limitations": [
                  "Numeric criteria compare an explicit reference/limit; they do not certify reference uncertainty.",
                  "CAP uses only the same cell's unloaded baseline; input declaration cannot prove matching physical conditions.",
                  "VOLT has no known voltage source in this fixture and is diagnostic only.",
                  "Groups are never averaged across session/revision/mode/rows/profile; legacy identity is unverified.",
                  "No four-point fit or correction is extrapolated to the other 60 cells."]}
    if capture is None:
        report["reason"] = "no_input_capture"
        return report
    report["source"] = capture.source
    report["baseline_source"] = baseline.source if baseline else None
    report["parser_counters"] = capture.counters
    report["memory_diagnostic_lines"] = capture.memory_diagnostic_lines
    report["capture_errors"] = {"truncated_frames": capture.truncated_frames,
                                "malformed_frames": capture.malformed_frames,
                                "identity_errors": capture.identity_errors,
                                "non_ascii_lines": capture.non_ascii_lines,
                                "firmware_fault_events": capture.fault_event_count}
    report["firmware_fault_event_examples"] = capture.fault_events
    failed = any(report["capture_errors"].values()) or capture.counters["crc_errors"] > 0 or capture.counters["malformed"] > 0
    floating = report["fixture"]["floating_cells"]
    for index, group in enumerate(capture.groups):
        references = {label: _reference_result(label, ref, group, baseline)
                      for label, ref in fixture["references"].items()}
        opens = {}
        for label in floating:
            point = group.cells.get((label, "RES"))
            opens[label] = {"status": _point_status(point, group.identity_status, True),
                            "statistics": point.output() if point else CellSamples().output()}
        diagnostics = {f"{label}/{mode}": point.output() for (label, mode), point in group.cells.items()
                       if mode in {"CAP", "VOLT"} and (label not in fixture["references"] or mode == "VOLT" or
                           fixture["references"][label]["kind"] != "capacitor")}
        minimum = fixture.get("minimum_screening_seconds", 120.0)
        screening = "WINDOW_SPAN_MET" if group.duration() >= minimum else "SCREENING_INCOMPLETE"
        if group.sequence_gaps or group.duplicate_or_old_frames or group.timestamp_regressions:
            screening = "SCREENING_INCOMPLETE"
        entry = {"index": index, "identity": group.identity, "identity_status": group.identity_status,
                 "frame_type": group.frame_type, "rows": group.rows, "profile": group.profile,
                 "frames": group.frames, "first_sequence": group.first_sequence, "last_sequence": group.last_sequence,
                 "timestamp_span_seconds": group.duration(), "screening": screening,
                 "screening_is_not_transport_delivery_proof": True,
                 "sequence_gaps": group.sequence_gaps, "missing_sequences": group.missing_sequences,
                 "duplicate_or_old_frames": group.duplicate_or_old_frames,
                 "timestamp_regressions": group.timestamp_regressions,
                 "references": references, "floating_res": opens,
                 "floating_res_observed": sum(v["status"] != "NOT_OBSERVED" for v in opens.values()),
                 "floating_res_pass": sum(v["status"] == "PASS" for v in opens.values()),
                 "diagnostic_only": diagnostics}
        failed |= any(v["accuracy"] in {"FAIL", "DATA_QUALITY_FAILED", "BASELINE_QUALITY_FAILED"} for v in references.values())
        failed |= any(v["status"] in {"FAIL", "DATA_QUALITY_FAILED"} for v in opens.values())
        failed |= bool(group.duplicate_or_old_frames or group.timestamp_regressions)
        report["groups"].append(entry)
    if not capture.groups:
        report["status"] = "NOT_RUN" if not failed else "FAIL"
        report["reason"] = "no_complete_frames"
    elif failed:
        report["status"] = "FAIL"
    elif provenance == "synthetic":
        report["status"] = "SIMULATED_ONLY"
    # REVIEW_REQUIRED intentionally remains even when some point criteria pass:
    # overall physical qualification also needs fixture uncertainty and coverage.
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--input", type=Path, help="Complete raw ASCII capture; never overwritten.")
    parser.add_argument("--baseline-input", type=Path,
                        help="Same-cell unloaded CAP capture; declares matching wiring/range/conditions, not independently verified.")
    parser.add_argument("--provenance", choices=("unknown", "recorded", "synthetic"), default="unknown")
    parser.add_argument("--output", type=Path, help="New JSON file; an existing file is never overwritten.")
    args = parser.parse_args()
    if args.baseline_input and not args.input:
        parser.error("--baseline-input requires --input")
    try:
        fixture = load_fixture(args.fixture)
        capture = read_capture(args.input) if args.input else None
        baseline = read_capture(args.baseline_input) if args.baseline_input else None
        report = evaluate_fixture(fixture, capture, baseline, args.provenance)
        payload = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
        if args.output:
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload)
        else:
            print(payload, end="")
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.exit(2, f"REFERENCE_FIXTURE_ERROR: {error}\n")
    return 1 if report["status"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
