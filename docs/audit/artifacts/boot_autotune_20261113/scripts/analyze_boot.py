#!/usr/bin/env python3
"""Analyze immutable boot captures; never join truncated or interleaved records.

Exit 0 means the analysis executed, not that firmware/HIL passed. Exit 2 means
an input-integrity or reset-boundary prerequisite was unavailable. Each accepted
record retains its original raw line/byte offsets. Wire timestamps are device
monotonic microseconds; optional receipt timestamps are host capture seconds.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


BASE = Path(__file__).resolve().parents[1]
TAGS = {
    "RST", "SPI_SWEEP", "SPI_SWEEP_TIME", "SPI_SELECTED", "BOOT_SPI_POST",
    "BOOT_BUS_JOIN", "I2C_SWEEP", "I2C_SELECTED", "APP_STACK", "STK50",
    "SPI", "FRONTENDS", "CONFIG", "BOOT", "PERF", "USBSTREAM", "BUILD", "PROTO",
}
REQUIRED = {
    "RST": "reason code boot bootId prevValid prevKind prevStage prevErr guard autoRestarts",
    "SPI_SWEEP": "owner phase profile requestedHz actualHz actualSource required attempted valid even odd crc maxCrc permille status register timeout reason err",
    "SPI_SWEEP_TIME": "owner phase profile requestedHz startUs endUs rawMin rawMax scope",
    "SPI_SELECTED": "owner qualified restored selectedHz actualHz actualSource candidates err cleanup firstCandidateError startUs endUs scope",
    "BOOT_SPI_POST": "budgetMs targetCore priority stackBytes enabled err",
    "BOOT_BUS_JOIN": "spiPhase done timedOut err spiCore i2cCore spiStartUs spiEndUs i2cStartUs i2cEndUs joinUs deadlineUs overlapUs",
    "I2C_SWEEP": "bus testHz ok idOk statusOk orderedOk burstOk nack timeout recover reason map",
    "SPI": "phase done joined timedOut qualified restored hz actualHz err run cleanup core stackBytes stackMinBytes candidates attempted samples even odd crc status reg drdy startUs endUs overlapUs scope",
    "FRONTENDS": "phase known present primary secondary ads pErr sErr aErr pCleanup sCleanup aCleanup aHz aRestoreHz ownerFree quarantine physicalSafe",
    "APP_STACK": "stage freeBytes",
    "STK50": "unit seq log transport",
}
NUMERIC = set("""
code boot bootId prevValid prevErr prevSeq prevHeap autoRestarts qualified restored
requestedHz actualHz required attempted valid even odd crc maxCrc permille status
register timeout err startUs endUs rawMin rawMax selectedHz candidates cleanup
firstCandidateError budgetMs targetCore priority stackBytes enabled done timedOut
spiCore i2cCore spiStartUs spiEndUs i2cStartUs i2cEndUs joinUs deadlineUs overlapUs
bus testHz ok idOk statusOk orderedOk burstOk nack recover hz maxStableHz marginKHz
bus0Hz bus1Hz joined run core stackMinBytes samples reg drdy known present pErr sErr
aErr pCleanup sCleanup aCleanup aHz aRestoreHz ownerFree quarantine hold probes
absentRun presentRun sdInhibited lowSpeed nextUs beginUs freeBytes seq rev boundary
rows mgen mrid rgen rrid pgen prid mtermId merr ptermId perr rtermId rerr
guardAgeS windowAgeS ready txStackBytes txStackMinBytes sinkQueued sinkDropped
driverAcceptedBytes partialWrite lineDropped lineOversize discardBytes
""".split())


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def number(value: str) -> int:
    if not re.fullmatch(r"-?(?:0[xX][0-9a-fA-F]+|[0-9]+)", value):
        raise ValueError("not_an_integer")
    return int(value, 16 if value.lstrip("-").lower().startswith("0x") else 10)


def raw_lines(data: bytes) -> list[dict]:
    result = []
    offset = 0
    chunks = data.split(b"\n")
    for index, chunk in enumerate(chunks):
        terminated = index < len(chunks) - 1
        if not chunk and not terminated:
            break
        raw = chunk[:-1] if chunk.endswith(b"\r") else chunk
        result.append({
            "line": index + 1, "offset": offset,
            "end_offset": offset + len(chunk) + int(terminated),
            "terminated": terminated,
            "raw": raw.decode("utf-8", "backslashreplace"),
        })
        offset += len(chunk) + int(terminated)
    return result


def parse_record(line: dict) -> tuple[dict | None, str | None]:
    text = line["raw"]
    tag = text.split(",", 1)[0]
    if tag not in TAGS:
        return None, None
    if not line["terminated"]:
        return None, "unterminated_line"
    fields = {}
    for token in text.split(",")[1:]:
        if token.count("=") != 1:
            return None, "malformed_key_value"
        key, value = token.split("=", 1)
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9]*", key) or not value or key in fields:
            return None, "empty_invalid_or_duplicate_key"
        fields[key] = value
    missing = set(REQUIRED.get(tag, "").split()) - fields.keys()
    if missing:
        return None, "missing:" + "/".join(sorted(missing))
    if tag == "I2C_SELECTED":
        expected = {"bus", "hz", "err"} if "bus" in fields else {"bus0Hz", "bus1Hz"}
        if not expected <= fields.keys():
            return None, "incomplete_i2c_selection"
    typed = dict(fields)
    for key in NUMERIC & fields.keys():
        try:
            typed[key] = number(fields[key])
        except ValueError:
            return None, "invalid_numeric:" + key
    if tag in {"SPI_SWEEP", "SPI_SWEEP_TIME"}:
        if fields["phase"] not in {"screen", "confirm"} or fields["profile"] not in {"pga1", "bypass"}:
            return None, "invalid_phase_or_profile"
        if not 1000 <= typed["requestedHz"] <= 8000000:
            return None, "invalid_requested_frequency"
    return {**line, "tag": tag, "fields": typed}, None


def compact(record: dict | None) -> dict | None:
    if record is None:
        return None
    return {key: value for key, value in record.items() if key != "terminated"}


def rate_record(record: dict) -> dict:
    fields = record["fields"]
    attempted = fields["attempted"]
    allowed = fields["required"] * fields["permille"] // 1000
    hard_errors = fields["status"] + fields["register"] + fields["timeout"]
    return {
        "record": compact(record),
        "crc_error_rate_percent": 100.0 * fields["crc"] / attempted if attempted else None,
        "configured_max_error_rate_percent": fields["permille"] / 10.0,
        "configured_full_attempt_allowance": allowed,
        "observed_crcs_within_full_attempt_allowance": fields["crc"] <= allowed,
        "attempt_count_complete": attempted == fields["required"],
        "crc_allowance_field_consistent": fields["maxCrc"] == allowed,
        "accepted_parity_sum_consistent": fields["even"] + fields["odd"] == fields["valid"],
        "accepted_plus_crc_equals_attempted": fields["valid"] + fields["crc"] == attempted,
        "no_reported_hard_error_counts": hard_errors == 0,
        "firmware_reported_phase_pass": fields["err"] == 0 and fields["reason"] == "ok",
        "rate_denominator_note": "observed rate uses attempted; early rejection allowance uses required",
    }


def event_receipts(path: Path, total_bytes: int) -> tuple[list[int], list[float], dict]:
    ends, times = [], []
    metadata = {"path": str(path), "exists": path.exists()}
    if not path.exists():
        return ends, times, metadata
    previous_end = 0
    gaps = []
    invalid = []
    commands = []
    for number_, text in enumerate(path.read_text().splitlines(), 1):
        try:
            event = json.loads(text)
        except json.JSONDecodeError:
            invalid.append(number_)
            continue
        if event.get("type") == "rx":
            offset = event.get("offset")
            length = event.get("length")
            if not isinstance(offset, int) or not isinstance(length, int):
                invalid.append(number_)
                continue
            if offset != previous_end:
                gaps.append({"event_line": number_, "expected_offset": previous_end, "offset": offset})
            previous_end = offset + length
            ends.append(previous_end)
            times.append(event["t"])
        if event.get("type") in {"tx_begin", "reset_intent", "serial_gap", "reopened"}:
            commands.append(event)
    metadata.update(sha256=sha(path), received_bytes=previous_end,
                    covers_raw_bytes=previous_end == total_bytes,
                    discontinuities=gaps, invalid_event_lines=invalid, actions=commands)
    # Only use host receipt times if byte alignment is directly established.
    if gaps or previous_end != total_bytes:
        return [], [], metadata
    return ends, times, metadata


def analyze(raw_path: Path, summary_path: Path, events_path: Path,
            build_manifest: Path | None = None) -> tuple[dict, int]:
    data = raw_path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    lines = raw_lines(data)
    ends, times, events = event_receipts(events_path, len(data))
    accepted, rejected = [], []
    for line in lines:
        if ends:
            index = bisect_left(ends, line["end_offset"])
            if index < len(times):
                line["host_receipt_seconds"] = times[index]
        parsed, error = parse_record(line)
        if parsed:
            accepted.append(parsed)
        elif error:
            rejected.append({**line, "reason": error})
    resets = [record for record in accepted if record["tag"] == "RST"]
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    integrity_errors = []
    if summary and summary.get("raw_sha256") != digest:
        integrity_errors.append("summary_raw_hash_mismatch")
    if not resets:
        integrity_errors.append("no_complete_rst_boundary")
    selected_reset = resets[-1] if resets else None
    start = selected_reset["offset"] if selected_reset else len(data)
    later_reset_markers = [r for r in lines if r["offset"] > start and
                           r["raw"].startswith(("ESP-ROM:", "rst:"))]
    stop = later_reset_markers[0]["offset"] if later_reset_markers else len(data)
    segment = [record for record in accepted if start <= record["offset"] < stop]
    by_tag = defaultdict(list)
    for record in segment:
        by_tag[record["tag"]].append(record)
    selected = by_tag["SPI_SELECTED"][-1] if by_tag["SPI_SELECTED"] else None
    spi_query = by_tag["SPI"][-1] if by_tag["SPI"] else None
    frequency = selected["fields"]["selectedHz"] if selected else (
        spi_query["fields"]["hz"] if spi_query else None)
    phases = defaultdict(list)
    phase_times = defaultdict(list)
    for record in by_tag["SPI_SWEEP"]:
        f = record["fields"]
        phases[(f["requestedHz"], f["phase"], f["profile"])].append(rate_record(record))
    for record in by_tag["SPI_SWEEP_TIME"]:
        f = record["fields"]
        phase_times[(f["requestedHz"], f["phase"], f["profile"])].append(compact(record))
    keys = sorted(set(phases) | set(phase_times), key=lambda k: (
        -k[0], 0 if k[1] == "screen" else 1, 0 if k[2] == "pga1" else 1))
    phase_rows = []
    for key in keys:
        records = phases[key]
        phase_rows.append({
            "requested_hz": key[0], "phase": key[1], "profile": key[2],
            "complete_metric_records": records,
            "complete_timing_records": phase_times[key],
            "metric_status": "complete" if len(records) == 1 else
                             "missing_or_corrupted" if not records else "multiple_records_review_required",
        })
    selected_phases = []
    for phase in ("screen", "confirm"):
        for profile in ("pga1", "bypass"):
            key = (frequency, phase, profile)
            rows = phases[key]
            selected_phases.append({
                "requested_hz": frequency, "phase": phase, "profile": profile,
                "status": "complete" if len(rows) == 1 else "unknown",
                "metrics": rows[0] if len(rows) == 1 else None,
                "timing_evidence": phase_times[key],
                "unknown_reason": None if len(rows) == 1 else
                    "no_unique_complete_metric_record; fragments are not concatenated",
            })
    join = by_tag["BOOT_BUS_JOIN"][-1] if by_tag["BOOT_BUS_JOIN"] else None
    overlap = None
    if join:
        f = join["fields"]
        computed = max(0, min(f["spiEndUs"], f["i2cEndUs"]) - max(f["spiStartUs"], f["i2cStartUs"]))
        overlap = {
            "record": compact(join), "unit": "device_monotonic_microseconds_same_boot",
            "spi_duration_us": f["spiEndUs"] - f["spiStartUs"],
            "i2c_duration_us": f["i2cEndUs"] - f["i2cStartUs"],
            "recomputed_overlap_us": computed,
            "matches_reported_overlap": computed == f["overlapUs"],
            "different_reported_cores": f["spiCore"] != f["i2cCore"],
            "positive_overlap": computed > 0,
            "query_overlap_matches": spi_query["fields"]["overlapUs"] == computed if spi_query else None,
        }
    frontend = by_tag["FRONTENDS"][-1] if by_tag["FRONTENDS"] else None
    issues = []
    if frontend:
        f = frontend["fields"]
        if f["primary"] == "unknown" or f["secondary"] == "unknown":
            issues.append("fdc_presence_unknown; this capture is not full frontend acceptance")
        if f["physicalSafe"] == "unknown":
            issues.append("physical analogue safety remains unknown, not verified by GPIO/register presence")
    if later_reset_markers:
        issues.append("later_reset_without_complete_RST_excluded_from_selected_boot")
    if selected_reset:
        for tag in ("BOOT", "CONFIG"):
            if by_tag[tag] and by_tag[tag][-1]["fields"].get("bootId") != selected_reset["fields"]["bootId"]:
                integrity_errors.append(tag.lower() + "_query_boot_id_conflicts_with_rst")
    segment_rejected = [r for r in rejected if start <= r["offset"] < stop]
    metric_rejected = [r for r in segment_rejected if r["raw"].startswith("SPI_SWEEP,")]
    if metric_rejected:
        issues.append("interleaved_or_truncated_spi_metric_lines; no reconstruction performed")
    if any(p["status"] != "complete" for p in selected_phases):
        issues.append("selected_frequency_four_phase_metric_evidence_incomplete")
    orphan_fragments = [compact(r) for r in lines if start <= r["offset"] < stop and
        re.search(r"(?:^|,)valid=[0-9]+.*(?:^|,)crc=[0-9]+", r["raw"]) and
        not r["raw"].startswith("SPI_SWEEP,")]
    embedded = [compact(r) for r in lines if start <= r["offset"] < stop and
        re.search(r".SPI_(?:SWEEP|SELECTED)", r["raw"])]
    spi_stack = None
    if spi_query:
        f = spi_query["fields"]
        spi_stack = {"stack_bytes": f["stackBytes"], "minimum_free_bytes": f["stackMinBytes"],
                     "inferred_peak_used_bytes": f["stackBytes"] - f["stackMinBytes"],
                     "query_line": spi_query["line"], "core": f["core"],
                     "unit": "bytes", "scope": "queried task high-water mark, not every task"}
    roms = [r for r in lines if r["offset"] < start and r["raw"].startswith("ESP-ROM:")]
    prelude_start = roms[-1]["offset"] if roms else start
    boot_identity = [compact(r) for r in lines if prelude_start <= r["offset"] < start and
        re.search(r"app_init: (?:App version:|Project name:|Compile time:|ELF file SHA256:|ESP-IDF:)", r["raw"])]
    fault_lines = [compact(r) for r in lines if start <= r["offset"] < stop and
                   r["raw"].startswith(("MFAULT,", "ADSFAULT,", "APP_FATAL,", "APP_INIT_FATAL,"))]
    all_phase_complete = all(p["status"] == "complete" for p in selected_phases)
    selected_aggregate = None
    if all_phase_complete:
        fields = [p["metrics"]["record"]["fields"] for p in selected_phases]
        attempts = sum(f["attempted"] for f in fields)
        crcs = sum(f["crc"] for f in fields)
        selected_aggregate = {
            "attempted": attempts, "valid": sum(f["valid"] for f in fields),
            "crc": crcs, "crc_error_rate_percent": 100.0 * crcs / attempts if attempts else None,
            "scope": "selected frequency, four complete original metric records only",
            "does_not_replace_each_profile_threshold_check": True,
        }
    output = {
        "schema": "sensorarray_boot_capture_analysis_v1",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "analysis_completed_is_not_firmware_acceptance": True,
        "source": {"raw_path": str(raw_path), "raw_bytes": len(data), "raw_sha256": digest,
                   "summary_path": str(summary_path), "summary_hash_matches":
                       summary.get("raw_sha256") == digest if summary else None,
                   "raw_mutated": False, "truncated_records_reconstructed": False,
                   "terminated_line_count": sum(r["terminated"] for r in lines),
                   "unterminated_tail": compact(lines[-1]) if lines and not lines[-1]["terminated"] else None,
                   "events": events},
        "segmentation": {"rule": "last syntactically complete newline-terminated RST record",
                         "complete_rst_records": [compact(r) for r in resets],
                         "selected_rst": compact(selected_reset), "selected_start_offset": start,
                         "selected_end_offset": stop,
                         "later_unestablished_reset_markers": [compact(r) for r in later_reset_markers],
                         "discarded_pre_rst_line_count": sum(r["offset"] < start for r in lines),
                         "boot_prelude_identity_records": boot_identity,
                         "cross_boot_counter_comparison": False},
        "integrity_errors": integrity_errors,
        "spi": {"selected_record": compact(selected), "query_record": compact(spi_query),
                "phase_records_by_frequency": phase_rows, "selected_four_phases": selected_phases,
                "selected_four_phase_evidence_complete": all_phase_complete,
                "selected_four_phase_aggregate": selected_aggregate,
                "actual_clock_source": "IDF calculated frequency truncated to kHz; not instrument measured",
                "query_counter_scope": spi_query["fields"].get("scope") if spi_query else None,
                "query_crc_rate_percent_all_candidates":
                    100.0 * spi_query["fields"]["crc"] / spi_query["fields"]["attempted"]
                    if spi_query and spi_query["fields"]["attempted"] else None,
                "query_counters_cannot_fill_missing_phase_metrics": True},
        "parallel": overlap,
        "i2c": {"sweep_records": [compact(r) for r in by_tag["I2C_SWEEP"]],
                "selection_records": [compact(r) for r in by_tag["I2C_SELECTED"]]},
        "stack": {"spi_query": spi_stack,
                  "application_stack_records": [compact(r) for r in by_tag["APP_STACK"]],
                  "periodic_task_stack_records": [compact(r) for r in by_tag["STK50"]]},
        "queries": {tag: compact(by_tag[tag][-1]) if by_tag[tag] else None
                    for tag in ("FRONTENDS", "CONFIG", "BOOT", "PERF", "USBSTREAM", "BUILD", "PROTO")},
        "record_integrity": {"rejected_tagged_records": [compact(r) for r in segment_rejected],
                             "orphan_fragments_unassigned": orphan_fragments,
                             "embedded_spi_markers_unassigned": embedded,
                             "complete_spi_metric_count": len(by_tag["SPI_SWEEP"]),
                             "rejected_spi_metric_count": len(metric_rejected)},
        "assessment": {"firmware_reported_spi_qualified":
                           bool(selected and selected["fields"]["qualified"] == 1),
                       "firmware_reported_spi_restored":
                           bool(selected and selected["fields"]["restored"] == 1),
                       "frontend_query_all_three_present": bool(frontend and all(
                           frontend["fields"][key] == "present" for key in ("primary", "secondary", "ads"))),
                       "firmware_fault_lines_observed": fault_lines,
                       "issues_and_limits": issues,
                       "full_firmware_acceptance_claimed": False},
    }
    if build_manifest:
        manifest = json.loads(build_manifest.read_text())
        versions = [re.search(r"App version:\s+(\S+)", r["raw"]).group(1)
                    for r in boot_identity if "App version:" in r["raw"]]
        output["build_context"] = {
            "manifest_path": str(build_manifest), "manifest_sha256": sha(build_manifest),
            "head": manifest.get("head"), "build_version": manifest.get("build_version"),
            "app_sha256": manifest.get("app_sha256"),
            "boot_prelude_app_version": versions[-1] if versions else None,
            "boot_app_version_matches_manifest":
                versions[-1] == manifest.get("build_version") if versions else None,
            "association": "explicit analyst-provided build manifest; boot prelude identity retained separately",
        }
    return output, 2 if integrity_errors else 0


def self_test() -> None:
    good = ("SPI_SWEEP,owner=BOOT,phase=screen,profile=pga1,requestedHz=900000,actualHz=909000,"
            "actualSource=idf_khz,required=200,attempted=200,valid=195,even=95,odd=100,crc=5,"
            "maxCrc=5,permille=25,status=0,register=0,timeout=0,reason=ok,err=0x0\n")
    parsed, err = parse_record(raw_lines(good.encode())[0])
    assert not err and rate_record(parsed)["crc_error_rate_percent"] == 2.5
    assert parse_record(raw_lines(good[:-1].encode())[0])[1] == "unterminated_line"
    assert parse_record(raw_lines(good.replace("attempted=200", "attempted=BOARD_I2C,err=0xFF").encode())[0])[1]
    broken = raw_lines(b"SPI_SWEEP,owner=BOOT,phase=screen,profile=pga1,requestedHz=9\n0,valid=195,crc=5\n")
    assert not parse_record(broken[0])[0] and not parse_record(broken[1])[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", help="capture label in this artifact directory, or a .rx.bin path")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--build-manifest", type=Path)
    args = parser.parse_args()
    self_test()
    raw_path = Path(args.capture) if args.capture.endswith(".rx.bin") else BASE / (args.capture + ".rx.bin")
    prefix = str(raw_path)[:-len(".rx.bin")]
    output_path = args.output or Path(prefix + ".analysis.json")
    result, status = analyze(raw_path, Path(prefix + ".summary.json"),
                             Path(prefix + ".events.jsonl"), args.build_manifest)
    try:
        head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=BASE, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        head = None
    result["execution"] = {"argv": sys.argv, "exit_code": status, "analyzer_head": head,
                           "script_sha256": sha(Path(__file__)), "self_tests": "4 passed"}
    output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output_path), "exit_code": status,
                      "selected_hz": result["spi"]["selected_record"]["fields"]["selectedHz"]
                          if result["spi"]["selected_record"] else None,
                      "selected_four_phase_evidence_complete": result["spi"]["selected_four_phase_evidence_complete"],
                      "issues": result["assessment"]["issues_and_limits"]}, ensure_ascii=False))
    return status


if __name__ == "__main__":
    raise SystemExit(main())
