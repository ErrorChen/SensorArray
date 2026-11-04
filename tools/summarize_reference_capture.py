#!/usr/bin/env python3
"""Summarize existing reference captures without changing recordings or hardware.

This is an evidence comparison, never a calibration PASS. Statistics stay in
contiguous configuration/rail segments. Use --capture repeatedly for new runs;
without it, select completed repaired-*, crc-fixed-res8-01 and timing-res8-01
summaries. Both raw .rx.bin and lossless .rx.bin.gz archives are supported.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

from reference_fixture import (DEFAULT_FIXTURE, IDENTITY_FIELDS, _identity,
                               _observations, _trailer_matches, analyse_capture,
                               describe_fixture, evaluate_fixture, load_fixture)
from text_protocol import TextProtocolParser, parse_fields


DEFAULT_DIRECTORY = Path(__file__).resolve().parents[1] / "docs/audit/artifacts/reference_fixture"
USB_COUNTERS = ("sinkQueued", "sinkDropped", "priorityQueued", "driverAcceptedBytes",
                "rxBytes", "partialWrite", "timeout", "driverDrained", "rxOversize")
SPI_COUNTERS = ("crcTotal", "retryTotal")
HEALTH_COUNTERS = ("crcValidEven", "crcValidOdd", "crcErrors", "crcRetries")
FRAME_COUNTERS = ("cap_frames", "voltage_frames", "resistance_frames", "mixed_frames")


def integer(value):
    try:
        return int(value, 16 if str(value).lower().startswith("0x") else 10)
    except (ValueError, TypeError):
        return None


def file_evidence(path: Path):
    data = path.read_bytes()
    return {"path": str(path), "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def read_raw(directory: Path, label: str):
    plain = directory / (label + ".rx.bin")
    archived = directory / (label + ".rx.bin.gz")
    path = plain if plain.exists() else archived
    stored = path.read_bytes()
    data = gzip.decompress(stored) if path.suffix == ".gz" else stored
    if plain.exists() and archived.exists():
        if gzip.decompress(archived.read_bytes()) != data:
            raise ValueError(f"raw/archive mismatch for {label}")
    return data, {"path": str(path), "storage_bytes": len(stored),
                  "storage_sha256": hashlib.sha256(stored).hexdigest(),
                  "uncompressed_bytes": len(data),
                  "uncompressed_sha256": hashlib.sha256(data).hexdigest(),
                  "compression": "gzip" if path.suffix == ".gz" else "none"}


class RxTimeline:
    def __init__(self, events):
        self.events = events
        self.rx = sorted((event for event in events if event.get("type") == "rx"),
                         key=lambda event: event["offset"])
        self.offsets = [event["offset"] for event in self.rx]

    def at(self, offset):
        index = bisect_right(self.offsets, offset) - 1
        if index >= 0:
            event = self.rx[index]
            if offset < event["offset"] + event["length"]:
                return {"phase": event.get("phase", "unknown"), "host_rx_seconds": event.get("t")}
        return {"phase": "unknown", "host_rx_seconds": None}


@dataclass
class FrameRecord:
    lines: list[str]
    header: dict
    trailer: dict
    raw_rows: dict
    frame: object
    line_start: int
    line_end: int
    byte_start: int
    byte_end: int


def parse_records(payload: bytes, timeline: RxTimeline):
    parser = TextProtocolParser()
    frames, diagnostics, resets = [], {}, []
    pending, header, raw_rows = [], {}, {}
    offset = first_offset = first_line = 0
    for number, raw in enumerate(payload.splitlines(keepends=True), 1):
        line = raw.rstrip(b"\r\n").decode("ascii", errors="surrogateescape")
        tag = line.split(",", 1)[0]
        fields = parse_fields(line)
        item = {"line_number": number, "byte_offset": offset,
                **timeline.at(offset + len(raw) - 1), "fields": fields}
        if tag in {"USBSTREAM", "ADSHEALTH", "ADSSPI", "ADSSPIBYTES", "ADSFRAME", "CONFIG", "ARL", "CELL"} or tag.startswith("ADSFAULT") or tag in {"MFAULT", "MERR", "RMERR"}:
            diagnostics.setdefault(tag, []).append(item)
        if "RAIL_REFRESH" in line:
            diagnostics.setdefault("RAIL_REFRESH", []).append({**item, "line": line})
        if line.startswith(("RST,", "ESP-ROM:")) or "rst:0x" in line.lower():
            resets.append({**item, "line": line})
        is_memory = (tag == "M" and {"stage", "reason", "ih", "il", "im"} <= fields.keys()
                     and not {"seq", "fmt", "rows", "cells", "profile", "rgen"} & fields.keys())
        if tag in {"C", "V", "R", "M"} and not is_memory:
            pending, header, raw_rows = [], {"tag": tag, **fields}, {}
            first_offset, first_line = offset, number
        if pending or (tag in {"C", "V", "R", "M"} and not is_memory):
            pending.append(line)
        if tag == "MR" and pending:
            try:
                raw_rows[int(fields["s"])] = {"tokens": line.split(",D=", 1)[1].split(","),
                                               "masks_known": "expected" in fields and "acquired" in fields}
            except (KeyError, ValueError, IndexError):
                pass
        frame = None if is_memory else parser.feed_line(line)
        if tag == "K":
            if frame is not None and pending:
                frames.append(FrameRecord(pending, header, fields, raw_rows, frame,
                                          first_line, number, first_offset, offset + len(raw)))
            pending = []
        offset += len(raw)
    return frames, diagnostics, resets


def counter_window(records, keys, same_boot):
    eligible = [record for record in records if any(key in record["fields"] for key in keys)]
    result = {"snapshots": len(eligible), "scope": "first_to_last_observed_snapshot_only",
              "status": "NO_SNAPSHOTS" if not eligible else "ONE_SNAPSHOT" if len(eligible) == 1 else "OBSERVED_WINDOW",
              "same_boot_evidence": same_boot, "counters": {}}
    if not eligible:
        return result
    result["first"] = eligible[0]
    result["last"] = eligible[-1]
    for key in keys:
        start, end = (integer(record["fields"].get(key)) for record in (eligible[0], eligible[-1]))
        reason = None
        if start is None or end is None:
            reason = "field_not_emitted"
        elif len(eligible) < 2:
            reason = "only_one_snapshot"
        elif not same_boot:
            reason = "boot_continuity_unverified"
        elif end < start:
            reason = "counter_decreased_reset_or_wrap_unresolved"
        result["counters"][key] = {"start": start, "end": end,
                                   "delta": None if reason else end - start,
                                   "delta_unavailable_reason": reason}
    return result


def compact_group(group):
    return {key: group[key] for key in ("identity", "identity_status", "frame_type", "rows", "profile",
            "frames", "first_sequence", "last_sequence", "timestamp_span_seconds", "screening",
            "sequence_gaps", "missing_sequences", "duplicate_or_old_frames", "timestamp_regressions")}


def segment_key(record):
    # age is deliberately excluded: every age increment must not create a
    # segment. Returning to an earlier rail value still starts a NEW segment.
    fields = record.header
    return tuple(fields.get(key) for key in ("tag", "rows", "profile", *IDENTITY_FIELDS,
                 "gen", "rid", "rail", "avdd", "avss", "ref", "vexc", "rref"))


def s1d1_observation(record, timeline, order_ok):
    frame, fields = record.frame, record.header
    values = [value for value in _observations(frame, fields, record.raw_rows) if value[0] == "S1D1"]
    identity_ok = _trailer_matches(fields, record.trailer)
    age = integer(fields.get("age"))
    result = {"seq": frame.sequence, "timestamp_us": frame.timestamp_us,
              "rail_age_frames": None if age == 0xFFFFFFFF else age,
              "rail_age_encoding": fields.get("age"), "ascii_crc_ok": frame.crc_ok,
              "trailer_identity_ok": identity_ok, "sequence_and_timestamp_order_ok": order_ok,
              **timeline.at(record.byte_end - 1)}
    if values:
        _, mode, unit, scale, fixed, valid, fresh, acquired, error, reason, known = values[0]
        accepted = frame.crc_ok and identity_ok and order_ok and valid and fresh and acquired and known and not error and reason == 0 and fixed is not None
        result.update({"mode": mode, "unit": unit, "valid": valid, "fresh": fresh,
                       "acquired": acquired, "masks_known": known,
                       "error_reason": f"X{reason:02X}" if reason >= 0 else "unknown",
                       "encoded_value_fixed": fixed,
                       "decoded_value": float(fixed) * (10.0 ** scale) if accepted else None,
                       "decoded_value_eligible": accepted})
    return result


def summarize_segment(records, index, fixture, timeline):
    first, last = records[0], records[-1]
    lines = [line for record in records for line in record.lines]
    analysed = analyse_capture(lines)
    assessed = evaluate_fixture(fixture, analysed, provenance="recorded")
    header = first.header
    avdd, avss = integer(header.get("avdd")), integer(header.get("avss"))
    ages = [integer(record.header.get("age")) for record in records]
    known_ages = [age for age in ages if age is not None and age != 0xFFFFFFFF]
    references = assessed["groups"][0]["references"] if assessed["groups"] else {}
    # Segment equality includes all the group keys; a parser producing two
    # different groups here is a defect in this summary, not permission to mix.
    if len(assessed["groups"]) != 1:
        raise ValueError("segment did not have exactly one immutable identity")
    group = assessed["groups"][0]
    observations = []
    previous = None
    if header["tag"] in {"R", "V", "M"}:
        for record in records:
            frame = record.frame
            order_ok = (previous is None or (0 < (frame.sequence - previous.sequence) & 0xFFFFFFFF < 0x80000000
                                             and frame.timestamp_us >= previous.timestamp_us))
            observations.append(s1d1_observation(record, timeline, order_ok))
            if order_ok and frame.crc_ok and _trailer_matches(record.header, record.trailer):
                previous = frame
    return {"index": index, **compact_group(group),
            "raw_extent": {"first_line": first.line_start, "last_line": last.line_end,
                           "first_byte": first.byte_start, "end_byte_exclusive": last.byte_end},
            "host_header_phase": timeline.at(first.byte_start),
            "host_trailer_phase": timeline.at(last.byte_end - 1),
            "condition": {"header_rail_valid": integer(header.get("rail")),
                          "avdd_uv": avdd, "avss_uv": avss,
                          "span_uv_from_header_split": avdd - avss if avdd is not None and avss is not None else None,
                          "reference": header.get("ref"), "vexc_uv": integer(header.get("vexc")),
                          "rref_ohm": integer(header.get("rref")),
                          "rail_source": "not_encoded_in_measurement_header",
                          "rail_is_validity_flag_not_span": True,
                          "independent_avdd_avss_measurement_proven": False},
            "rail_age_frames": {"first": ages[0] if ages[0] != 0xFFFFFFFF else None,
                                "last": ages[-1] if ages[-1] != 0xFFFFFFFF else None,
                                "min": min(known_ages) if known_ages else None,
                                "max": max(known_ages) if known_ages else None,
                                "unknown_samples": sum(age is None or age == 0xFFFFFFFF for age in ages)},
            "parser_counters": assessed["parser_counters"], "capture_errors": assessed["capture_errors"],
            "expected_reference_statistics": references,
            "floating_res_observed": group["floating_res_observed"],
            "floating_res_open_criteria_met": group["floating_res_pass"],
            "S1D1_timeline": observations,
            "calibration_use": "NOT_APPROVED; no cross-segment mean or coefficient fit",
            "raw_adc": {"status": "NOT_AVAILABLE", "derived_raw_adc_created": False,
                        "note": "Measurement D values are already converted SI values, not raw ADC codes. Bounded ADSSPIBYTES diagnostics cannot replace per-cell valid raw ADC samples."}}


def summarize_capture(directory: Path, label: str, fixture: dict):
    summary_path = directory / (label + ".summary.json")
    summary = json.loads(summary_path.read_text())
    event_path = directory / (label + ".events.jsonl")
    events = [json.loads(line) for line in event_path.read_text().splitlines()] if event_path.exists() else []
    timeline = RxTimeline(events)
    payload, raw_evidence = read_raw(directory, label)
    expected_hash = summary.get("raw_sha256")
    raw_evidence["summary_expected_sha256"] = expected_hash
    raw_evidence["summary_hash_matches"] = expected_hash == raw_evidence["uncompressed_sha256"] if expected_hash else None
    if expected_hash and not raw_evidence["summary_hash_matches"]:
        raise ValueError(f"summary/raw hash mismatch for {label}")
    decoded = payload.decode("ascii", errors="surrogateescape").splitlines()
    assessed = evaluate_fixture(fixture, analyse_capture(decoded), provenance="recorded")
    frames, diagnostics, resets = parse_records(payload, timeline)
    segments = []
    for frame in frames:
        if not segments or segment_key(segments[-1][-1]) != segment_key(frame):
            segments.append([])
        segments[-1].append(frame)
    identities = {(group["identity"].get("session"), group["identity"].get("bootId")) for group in assessed["groups"]}
    identities.update((record["fields"].get("session"), record["fields"].get("bootId")) for record in diagnostics.get("CONFIG", []))
    identities.discard((None, None))
    one_known_boot = len(identities) == 1 and all(pair[0] is not None and pair[1] is not None for pair in identities)
    usb_records = [record for record in diagnostics.get("USBSTREAM", []) if "sinkQueued" in record["fields"]]
    usb_reset_between = bool(usb_records and any(usb_records[0]["byte_offset"] <= reset["byte_offset"] <= usb_records[-1]["byte_offset"] for reset in resets))
    usb = counter_window(usb_records, USB_COUNTERS, one_known_boot and not usb_reset_between)
    usb.update({"window": "initial query before FULL through restored-profile query; includes preparation/cleanup",
                "driverAcceptedBytes_semantics": "bytes accepted by device USB driver, not PC acknowledgement",
                "rxBytes_semantics": "device command RX bytes, not PC measurement RX",
                "counter_cumulative_values_are_not_run_drops": True,
                "enumerated": usb_records[-1]["fields"].get("enumerated", "unknown") if usb_records else "unknown",
                "collector_actual_received_bytes": len(payload)})
    health = diagnostics.get("ADSHEALTH", [])
    spi = diagnostics.get("ADSSPI", [])
    faults = [dict(record, tag=tag) for tag, records in diagnostics.items()
              if tag.startswith("ADSFAULT") or tag in {"MFAULT", "MERR", "RMERR"} for record in records]
    faults.sort(key=lambda record: record["byte_offset"])
    capture_frames = [frame for frame in frames if timeline.at(frame.byte_start)["phase"] == "capture" and timeline.at(frame.byte_end - 1)["phase"] == "capture"]
    capture_trailers = [frame for frame in frames if timeline.at(frame.byte_end - 1)["phase"] == "capture"]
    counters = assessed["parser_counters"]
    analysis_path = directory / (label + ".analysis.json")
    existing_analysis = json.loads(analysis_path.read_text()) if analysis_path.exists() else None
    existing_check = None if existing_analysis is None else {
        "status_matches": existing_analysis.get("status") == assessed["status"],
        "parser_counters_match": existing_analysis.get("parser_counters") == counters,
        "capture_errors_match": existing_analysis.get("capture_errors") == assessed["capture_errors"]}
    return {"label": label, "record_kind": "sampling_capture" if "capture_seconds" in summary else "diagnostic_queries_only",
            "evidence_status": "INTERIM_RECORDED_EVIDENCE_ONLY", "final_acceptance": "NOT_ESTABLISHED",
            "files": {"summary": file_evidence(summary_path), "raw": raw_evidence,
                      "events": file_evidence(event_path) if event_path.exists() else None,
                      "existing_analysis": file_evidence(analysis_path) if analysis_path.exists() else None},
            "existing_analysis_recheck": existing_check,
            "collector": {key: summary.get(key) for key in ("status", "error", "mode_requested", "rowmodes_requested", "rows_requested", "duration_total_budget_seconds", "elapsed_seconds", "prepare_seconds", "capture_seconds", "capture_frame_trailers", "usb_restore")},
            "build_reply": summary.get("replies", {}).get("BUILD?"),
            "proto_reply": summary.get("replies", {}).get("PROTO?"),
            "firmware_commit": "not_inferred_from_current_HEAD; consult matching flash manifest",
            "queries_are_observed_at_their_own_times_not_atomic_combined_state": True,
            "capture_window": {"capture_start_event": next((event for event in events if event.get("type") == "capture_start"), None),
                               "capture_end_event": next((event for event in events if event.get("type") == "capture_end"), None),
                               "all_raw_complete_frames": sum(counters[key] for key in FRAME_COUNTERS),
                               "all_raw_ascii_crc_valid_frames": sum(frame.frame.crc_ok for frame in frames),
                               "capture_phase_complete_frames": len(capture_frames),
                               "capture_phase_trailer_completed_frames": len(capture_trailers),
                               "boundary_straddling_frames": len(capture_trailers) - len(capture_frames),
                               "cleanup_or_prepare_complete_frames": len(frames) - len(capture_trailers),
                               "frames_crc_valid_not_equivalent_to_sensor_sample_validity": True},
            "parser_counters": counters, "capture_errors": assessed["capture_errors"],
            "fixture_analysis_status": assessed["status"],
            "configuration_groups": [compact_group(group) for group in assessed["groups"]],
            "measurement_segments": [summarize_segment(records, index, fixture, timeline) for index, records in enumerate(segments)],
            "usb": usb,
            "ads_health": {"observed_records": len(health), "records": health,
                           "observed_engine_frame_counts": [integer(record["fields"].get("engineFrameCount")) for record in health],
                           "failed_records": [record for record in health if integer(record["fields"].get("err")) not in (None, 0)],
                           "absent_records_are_not_assumed_successful_checks": True},
            "ads_spi": {"observed_records": len(spi), "records": spi,
                        "cumulative_counter_window": counter_window(spi, SPI_COUNTERS, one_known_boot and not resets),
                        "sum_of_observed_event_fields": {key: sum(integer(record["fields"].get(key)) or 0 for record in spi) for key in ("crcFailures", "crcRetries", "crcRecoveredReads")},
                        "event_sums_are_not_complete_capture_totals": True,
                        "crc_valid_parity_and_totals_from_health": counter_window(health, HEALTH_COUNTERS, one_known_boot and not resets),
                        "spi_bytes_diagnostics": diagnostics.get("ADSSPIBYTES", [])},
            "rail_refresh_records": diagnostics.get("RAIL_REFRESH", []),
            "rail_queries": diagnostics.get("ARL", []),
            "cell_queries": diagnostics.get("CELL", []),
            "cell_queries_are_diagnostics_not_valid_samples_without_validity_checks": True,
            "ads_frame_diagnostics": diagnostics.get("ADSFRAME", []),
            "fault_records_before_and_after_cleanup_preserved": faults,
            "reset_observations": resets}


def make_report(directory: Path, labels: list[str], fixture_path: Path):
    fixture = load_fixture(fixture_path)
    return {"schema_version": 1, "status": "INTERIM_EVIDENCE_COMPARISON_NOT_FINAL_PASS",
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "generator": file_evidence(Path(__file__)), "invocation": sys.argv,
            "fixture": describe_fixture(fixture), "fixture_file": file_evidence(fixture_path),
            "limitations": ["Input files are recorded captures; no device is opened and no calibration is written.",
                            "CAP is total LC capacitance; no same-cell unloaded baseline was supplied to this comparison.",
                            "Nominal reference tolerances and device acceptance limits remain unknown.",
                            "No mean crosses a contiguous rail/configuration segment; nothing here is a calibration fit.",
                            "Host RX phase and time describe receipt, not ADC sampling; frame timestamp_us is the firmware timestamp.",
                            "USB and ADS cumulative counter deltas cover only the observed endpoint window; absence of a diagnostic is not zero errors.",
                            "Raw fixture ADC codes and independent physical voltages are unavailable; none are reconstructed from converted values.",
                            "Cross-boot sequence/generation ordering is never used. Unknown counter wrap/reset yields no delta."],
            "captures": [summarize_capture(directory, label, fixture) for label in labels]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY)
    parser.add_argument("--capture", action="append", help="Capture label; repeat for multiple runs.")
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--output", type=Path, required=True, help="New JSON output; refuses overwrite.")
    args = parser.parse_args()
    if args.output.exists():
        parser.exit(2, "SUMMARY_ERROR: output exists; choose a new file\n")
    labels = args.capture or sorted(path.name.removesuffix(".summary.json") for path in args.directory.glob("*.summary.json")
                                   if path.name.startswith(("repaired-", "crc-fixed-res8-01.", "timing-res8-01.")))
    if not labels or any(not re.fullmatch(r"[A-Za-z0-9_-]+", label) for label in labels):
        parser.error("no valid capture labels selected")
    try:
        report = make_report(args.directory, labels, args.fixture)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, ensure_ascii=True, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError, KeyError) as error:
        parser.exit(2, f"SUMMARY_ERROR: {error}\n")
    print(json.dumps({"status": report["status"], "captures": len(report["captures"]), "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
