#!/usr/bin/env python3
"""Temporary cross-cell CAP background estimates; no calibration or acceptance.

Each estimate pairs a target and a fixed donor set from the SAME valid frame.
Every donor must be valid, fresh, acquired CAP data. Missing donors never become
zero and never silently change the median's population. Output is exclusive.
CLI exit 0 means analysis was written, not that precision or hardware passed.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass, field
import gzip
import hashlib
import itertools
import json
import math
from pathlib import Path
import statistics
from typing import Iterable

from reference_fixture import (ALL_CELLS, CaptureGroup, _identity, _observations,
                               _trailer_matches, analyse_capture, summary)
from text_protocol import MixedFrame, TextProtocolParser, parse_fields


OCCUPIED = ("S1D1", "S8D8", "S4D4", "S5D5")
TARGETS = ("S4D4", "S5D5")
METHODS = ("same_row_median", "same_column_median", "all_floating_median")
FLOATING = tuple(label for label in ALL_CELLS if label not in OCCUPIED)


def donor_sets(target: str) -> dict[str, tuple[str, ...]]:
    """Predeclare fixed populations: seven row, seven column, sixty global."""
    return {
        "same_row_median": tuple(cell for cell in FLOATING if cell[1] == target[1]),
        "same_column_median": tuple(cell for cell in FLOATING if cell[3] == target[3]),
        "all_floating_median": FLOATING,
    }


def distribution(values: list[float]) -> dict:
    result = summary(values)
    ordered = sorted(values)

    def percentile(fraction):
        if not ordered:
            return None
        position = (len(ordered) - 1) * fraction
        left, right = math.floor(position), math.ceil(position)
        return ordered[left] + (ordered[right] - ordered[left]) * (position - left)

    result.update(p05=percentile(0.05), p95=percentile(0.95))
    return result


def _memory_diagnostic(line: str) -> bool:
    # Preserve the existing wrapper's narrow legacy M/heap discriminator.
    fields = parse_fields(line)
    return (line.startswith("M,") and
            {"stage", "reason", "ih", "il", "im"} <= fields.keys() and
            not {"seq", "fmt", "rows", "cells", "profile", "rgen"} & fields.keys() and
            bool(fields["stage"]) and bool(fields["reason"]) and
            all(fields[key].isdigit() for key in ("ih", "il", "im")))


def _complete_frames(lines: Iterable[str]):
    """Keep original wire text for the shared parser's real CRC calculation."""
    parser = TextProtocolParser()
    header, raw_rows = {}, {}
    for raw in lines:
        line = raw.rstrip("\r\n")
        try:
            line.encode("ascii")
        except UnicodeEncodeError:
            continue
        if _memory_diagnostic(line):
            continue
        tag = line.split(",", 1)[0]
        if tag in {"C", "V", "R", "M"}:
            header, raw_rows = parse_fields(line), {}
            header["tag"] = tag
        elif tag == "MR":
            try:
                fields = parse_fields(line)
                raw_rows[int(fields["s"])] = {
                    "tokens": line.split(",D=", 1)[1].split(","),
                    "masks_known": "expected" in fields and "acquired" in fields,
                }
            except (ValueError, KeyError, IndexError):
                pass  # The shared parser and analyse_capture retain errors.
        frame = parser.feed_line(line)
        if frame is not None:
            yield frame, header, parse_fields(line), raw_rows


def _cap_cells(frame, header, raw_rows) -> dict:
    """Reuse wire masks/units, also require frame AND mixed-row expected bits."""
    cells = {}
    for (label, mode, unit, scale, fixed, valid, fresh, acquired, error,
         reason, masks_known) in _observations(frame, header, raw_rows):
        row, col = int(label[1]), int(label[3])
        bit = 1 << ((row - 1) * 8 + col - 1)
        reasons = []
        if mode != "CAP":
            reasons.append("not_cap_mode")
        if (unit, scale) != ("pF", -6):
            reasons.append("not_cap_units")
        if not masks_known:
            reasons.append("masks_unknown")
        if not frame.expected_mask & bit:
            reasons.append("not_expected")
        if not acquired or not frame.acquired_mask & bit:
            reasons.append("unacquired")
        if isinstance(frame, MixedFrame) and not frame.row_frames[row].expected_mask & (1 << (col - 1)):
            reasons.append("row_not_expected")
        if not fresh:
            reasons.append("stale")
        if not valid or fixed is None or error or reason != 0:
            reasons.append("invalid")
        if fixed is not None and not math.isfinite(float(fixed)):
            reasons.append("nonfinite")
        cells[label] = {"value_pf": float(fixed) / 1_000_000.0 if not reasons else None,
                        "rejections": reasons}
    return cells


@dataclass
class MethodSamples:
    donors: tuple[str, ...]
    background: list[float] = field(default_factory=list)
    target: list[float] = field(default_factory=list)
    difference: list[float] = field(default_factory=list)
    coverage: list[float] = field(default_factory=list)
    rejections: Counter = field(default_factory=Counter)
    missing_donors: dict[str, Counter] = field(default_factory=dict)

    def output(self, frames: int) -> dict:
        return {
            "donor_cells": list(self.donors), "required_donor_count": len(self.donors),
            "complete_frames_considered": frames, "valid_paired_count": len(self.difference),
            "excluded_frame_count": frames - len(self.difference),
            "rejection_counts_nonexclusive": dict(self.rejections),
            "missing_donor_counts_nonexclusive": {
                label: dict(reasons) for label, reasons in self.missing_donors.items()},
            "available_donor_count_in_trusted_frames": distribution(self.coverage),
            "paired_target_total_pf": distribution(self.target),
            "background_pf": distribution(self.background),
            "target_minus_background_pf": distribution(self.difference),
        }


@dataclass
class TargetSamples:
    label: str
    total: list[float] = field(default_factory=list)
    rejections: Counter = field(default_factory=Counter)
    methods: dict[str, MethodSamples] = field(init=False)
    ranges: list[float] = field(default_factory=list)
    pairs: dict[str, list[float]] = field(default_factory=dict)

    def __post_init__(self):
        self.methods = {name: MethodSamples(donors) for name, donors in donor_sets(self.label).items()}
        self.pairs = {f"{left}_minus_{right}": [] for left, right in itertools.combinations(METHODS, 2)}

    def output(self, frames: int) -> dict:
        return {
            "nominal_pf": 33.0, "nominal_source": "user_nominal",
            "precision_acceptance": "NOT_EVALUATED",
            "target_usable_frames": len(self.total),
            "target_total_pf": distribution(self.total),
            "target_rejection_counts_nonexclusive": dict(self.rejections),
            "methods": {name: method.output(frames) for name, method in self.methods.items()},
            "cross_method_sensitivity": {
                "all_methods_paired_count": len(self.ranges),
                "per_frame_estimate_range_pf": distribution(self.ranges),
                "pairwise_estimate_difference_pf": {name: distribution(values) for name, values in self.pairs.items()},
                "interpretation": "Descriptive model sensitivity, not traceable measurement uncertainty.",
            },
        }


def _estimate_target(target: TargetSamples, cells: dict, frame_rejections: list[str]) -> dict:
    point = cells.get(target.label, {"value_pf": None, "rejections": ["not_observed"]})
    reasons = frame_rejections or point["rejections"]
    target.rejections.update(reasons)
    target_value = point["value_pf"] if not reasons else None
    if target_value is not None:
        target.total.append(target_value)
    result = {"target_total_pf": target_value, "target_rejections": reasons, "methods": {}}
    differences = {}
    for name, method in target.methods.items():
        missing, values = {}, []
        if not frame_rejections:
            for donor in method.donors:
                candidate = cells.get(donor, {"value_pf": None, "rejections": ["not_observed"]})
                if candidate["rejections"]:
                    missing[donor] = candidate["rejections"]
                    method.missing_donors.setdefault(donor, Counter()).update(candidate["rejections"])
                else:
                    values.append(candidate["value_pf"])
            method.coverage.append(len(values))
        rejection = list(reasons)
        if missing:
            rejection.append("incomplete_donor_set")
        method.rejections.update(rejection)
        background = statistics.median(values) if not rejection else None
        difference = target_value - background if background is not None else None
        if difference is not None:
            method.target.append(target_value)
            method.background.append(background)
            method.difference.append(difference)
            differences[name] = difference
        result["methods"][name] = {
            "background_pf": background, "target_minus_background_pf": difference,
            "available_donor_count": len(values) if not frame_rejections else None,
            "required_donor_count": len(method.donors), "missing_donors": missing,
            "rejections": rejection,
        }
    sensitivity_range = None
    if len(differences) == len(METHODS):
        sensitivity_range = max(differences.values()) - min(differences.values())
        target.ranges.append(sensitivity_range)
    for left, right in itertools.combinations(METHODS, 2):
        if left in differences and right in differences:
            target.pairs[f"{left}_minus_{right}"].append(differences[left] - differences[right])
    result["cross_method_estimate_range_pf"] = sensitivity_range
    return result


def estimate_capture(lines: Iterable[str], source: dict | None = None) -> dict:
    lines = list(lines)
    # Existing evaluator's quality path is retained alongside paired analysis.
    quality = analyse_capture(lines, source)
    groups = {}
    for frame, header, trailer, raw_rows in _complete_frames(lines):
        identity, identity_status = _identity(header)
        key = json.dumps([identity, header.get("tag"), frame.rows, header.get("profile", "")], sort_keys=True)
        if key not in groups:
            groups[key] = {
                "order": CaptureGroup(identity, identity_status, header.get("tag", "?"), frame.rows, header.get("profile", "")),
                "rejections": Counter(), "targets": {label: TargetSamples(label) for label in TARGETS},
                "per_frame": [],
            }
        group = groups[key]
        order = group["order"]
        rejected = []
        if not frame.crc_ok:
            rejected.append("bad_crc")
        if not _trailer_matches(header, trailer):
            rejected.append("header_trailer_identity_mismatch")
        if identity_status != "consistent_frame_identity":
            rejected.append("unverified_identity")
        if rejected:
            order.frames += 1
        elif not order.add_frame(frame):
            rejected.append("duplicate_old_or_timestamp_regression")
        group["rejections"].update(rejected)
        cells = _cap_cells(frame, header, raw_rows) if not rejected else {}
        group["per_frame"].append({
            "sequence": frame.sequence, "timestamp_us": frame.timestamp_us,
            "frame_rejections": rejected,
            "targets": {label: _estimate_target(target, cells, rejected) for label, target in group["targets"].items()},
        })
    output_groups = []
    for group in groups.values():
        order = group["order"]
        output_groups.append({
            "identity": order.identity, "identity_status": order.identity_status,
            "frame_type": order.frame_type, "rows": order.rows, "profile": order.profile,
            "complete_frames": order.frames,
            "frame_rejection_counts_nonexclusive": dict(group["rejections"]),
            "first_sequence": order.first_sequence, "last_sequence": order.last_sequence,
            "first_timestamp_us": order.first_timestamp_us, "last_timestamp_us": order.last_timestamp_us,
            "trusted_frame_span_seconds": order.duration(),
            "sequence_gaps": order.sequence_gaps, "missing_sequences": order.missing_sequences,
            "duplicate_or_old_frames": order.duplicate_or_old_frames,
            "timestamp_regressions": order.timestamp_regressions,
            "targets": {label: target.output(order.frames) for label, target in group["targets"].items()},
            "per_frame": group["per_frame"],
        })
    pair_count = sum(len(method.difference) for group in groups.values()
                     for target in group["targets"].values() for method in target.methods.values())
    return {
        "schema_version": 1, "analysis_kind": "CROSS_CELL_PROXY",
        "status": "APPROXIMATE_DIAGNOSTIC" if pair_count else "NO_VALID_PAIRED_ESTIMATE",
        "precision_acceptance": "NOT_EVALUATED", "calibration_written": False,
        "source": source or {}, "unit": "pF", "occupied_cells_excluded": list(OCCUPIED),
        "method_policy": {
            "fixed_donor_sets": True, "required_coverage_fraction": 1.0,
            "same_frame_pairing": True, "groups_pooled": False,
            "unverified_identity": "excluded_from_estimates",
            "sequence_order": "uint32 modular within group; zero or >= half-range rejected; never compared across sessions",
            "percentiles": "linear interpolation at (n-1)*fraction; population stddev",
            "counts": "Rejection reason counts may overlap; paired counts and excluded frame counts do not.",
        },
        "limitations": [
            "Other cells are a temporary parasitic-background proxy, not a same-cell unloaded measurement.",
            "Different row/column parasitics, coupling, routing and fixture loading can bias these estimates.",
            "All three methods are reported; none is selected by closeness to the nominal 33 pF.",
            "Negative differences and all eligible outliers are retained without smoothing.",
            "Statistical spread and method sensitivity are descriptive, not traceable uncertainty or an accuracy limit.",
            "Wire session/config identity cannot prove equal topology, temperature or reference traceability.",
            "Mixed non-CAP rows cannot supply CAP donors; incomplete fixed populations produce no estimate.",
            "This report is not a strict reference_fixture baseline input and does not authorize calibration writes.",
        ],
        "capture_quality": {
            "shared_parser_counters": quality.counters,
            "shared_parser_sequence_counters_scope": "global parser only; use per-group sequence counters for identity-aware order",
            "truncated_frames": quality.truncated_frames, "malformed_frames": quality.malformed_frames,
            "identity_errors": quality.identity_errors, "non_ascii_lines": quality.non_ascii_lines,
            "memory_diagnostic_lines": quality.memory_diagnostic_lines,
            "runtime_fault_count": quality.fault_event_count, "runtime_fault_examples": quality.fault_events,
        },
        "groups": output_groups,
    }


def read_input(path: Path) -> tuple[list[str], dict]:
    stored = path.read_bytes()
    compressed = path.suffix == ".gz" or stored[:2] == b"\x1f\x8b"
    payload = gzip.decompress(stored) if compressed else stored
    return payload.decode("ascii", errors="surrogateescape").splitlines(), {
        "path": str(path), "stored_sha256": hashlib.sha256(stored).hexdigest(),
        "stored_bytes": len(stored), "gzip": compressed,
        "raw_sha256": hashlib.sha256(payload).hexdigest(), "raw_bytes": len(payload),
        "provenance": "input recording; synthetic versus physical provenance must be established by its capture record",
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Completed raw ASCII capture, optionally lossless .gz")
    parser.add_argument("--output", type=Path, required=True, help="New JSON report; an existing path is never overwritten")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise FileExistsError(f"output already exists: {args.output}")
        lines, source = read_input(args.input)
        report = estimate_capture(lines, source)
        encoded = json.dumps(report, indent=2, ensure_ascii=True, allow_nan=False) + "\n"
        with args.output.open("x", encoding="utf-8") as output:
            output.write(encoded)
    except (OSError, ValueError, EOFError) as error:
        parser.exit(2, f"error: {error}\n")
    print(json.dumps({"output": str(args.output), "status": report["status"],
                      "groups": len(report["groups"]), "precision_acceptance": "NOT_EVALUATED"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
