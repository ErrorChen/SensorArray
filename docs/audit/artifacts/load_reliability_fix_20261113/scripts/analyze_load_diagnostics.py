#!/usr/bin/env python3
"""Analyze SERIAL/SCHED query deltas in explicitly completed load summaries.

No serial, BLE, raw capture, directory scan, or source-summary write is used.
Each sink has its own request/reply windows; these are NOT capture-only deltas.
Exit 0 means analysis completed, never that the firmware passed acceptance.

Example after capture has finished:
  python3 analyze_load_diagnostics.py --summary completed.summary.json --output new-diagnostics.json
Run --self-test for synthetic fixtures only (all fixture files are temporary).
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import tempfile
import unittest


SCHEMA = "sensorarray_load_diagnostics_v1"
VARIANT = "load_reliability_diagnostics_v1"
WORKERS = ("bt", "tr", "log", "ser", "bc", "sc", "wc")
IDENTITY = ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid")
CONFIG_FIELDS = IDENTITY + ("state", "mode", "rows", "profile", "boundary", "err")
SERIAL_COUNTERS = {
    "normalSubmitted": 32, "prioritySubmitted": 32, "normalExpired": 32,
    "priorityExpired": 32, "priorityRejected": 32, "driverShortWrite": 32,
    "driverWriteError": 32, "txYield": 32, "driverAcceptedBytes": 64, "partialWrite": 32,
    "dataSubmitted": 32, "logSubmitted": 32, "dataRejected": 32, "logRejected": 32,
}
NOTES = [
    "SERIAL and SCHED deltas use their separate host monotonic query windows, not the capture interval.",
    "Device sampling time is only bounded by request/reply times; transport latency and read-only retries are retained.",
    "Counter deltas are forward modular differences within one observed boot/session; complete counter cycles are unobservable.",
    "Maximum values and gauges are endpoints only. Maximums are boot-lifetime observations, not maxima measured only in this window.",
    "SCHED fields are independently sampled. Checkpoints/yields and idle progress are not CPU utilization or proof of watchdog safety.",
    "SERIAL submission/driver acceptance counters do not prove PC delivery. This report does not change capture acceptance.",
]


def uint(raw, bits=32, *, sentinel_unknown=False):
    result = {"raw": raw, "value": None, "known": False}
    if raw is None or raw == "unknown":
        result["reason"] = "missing" if raw is None else "unknown"
    elif isinstance(raw, bool) or not re.fullmatch(r"[0-9]+", str(raw)) or int(raw) >= 1 << bits:
        result["reason"] = "invalid_uint" + str(bits)
    elif sentinel_unknown and int(raw) == (1 << bits) - 1:
        result["reason"] = "unknown_sentinel"
    else:
        result.update(value=int(raw), known=True)
    return result


def endpoints(before, after, *, bits=32, sentinel_unknown=False, kind="gauge"):
    return {"kind": kind, "before": uint(before, bits, sentinel_unknown=sentinel_unknown),
            "after": uint(after, bits, sentinel_unknown=sentinel_unknown)}


def counter(before, after, bits, issues, *, skip_zero=False):
    result = endpoints(before, after, bits=bits, kind="counter")
    result.update(bits=bits, modulus=(1 << bits) - int(skip_zero), delta=None,
                  wrap_observed=None, zero_reserved_unknown=skip_zero)
    reasons = list(issues)
    for name in ("before", "after"):
        value = result[name]
        if not value["known"]:
            reasons.append(name + ":" + value["reason"])
        elif skip_zero and value["value"] == 0:
            value.update(known=False, reason="zero_reserved_unknown")
            reasons.append(name + ":zero_reserved_unknown")
    if reasons:
        result["unavailable_reasons"] = reasons
    else:
        a, b = result["before"]["value"], result["after"]["value"]
        result.update(delta=(b - a) % result["modulus"], wrap_observed=b < a)
    return result


def finite_time(raw):
    return isinstance(raw, (int, float)) and not isinstance(raw, bool) and math.isfinite(raw) and raw >= 0


def time_window(record):
    if not isinstance(record, dict):
        return None
    start, end = record.get("started_monotonic"), record.get("finished_monotonic")
    if not finite_time(start) or not finite_time(end) or end < start:
        return None
    return {"started_monotonic": start, "finished_monotonic": end, "duration_seconds": end - start}


def query_window(summary, phase, command):
    record = summary.get("snapshots", {}).get(phase, {}).get(command, {})
    collection = time_window(record)
    result = {"collection_window": collection, "sample_time_bounds": collection,
              "sample_time_bounds_source": "snapshot_query_window_including_retries",
              "raw_fields": record.get("fields", {}), "successful_attempt": None}
    attempts = summary.get("query_attempts", [])
    begin, end = record.get("attempt_begin"), record.get("attempt_end")
    if isinstance(begin, int) and isinstance(end, int) and 0 <= begin < end <= len(attempts):
        candidates = [(i, a) for i, a in enumerate(attempts[begin:end], begin)
                      if a.get("command") == command and a.get("phase") == phase and a.get("outcome") == "complete_reply"]
        if len(candidates) == 1:
            index, attempt = candidates[0]
            actual = time_window(attempt)
            if actual and collection and collection["started_monotonic"] <= actual["started_monotonic"] <= actual["finished_monotonic"] <= collection["finished_monotonic"]:
                result.update(sample_time_bounds=actual, sample_time_bounds_source="successful_readonly_query_attempt",
                              successful_attempt={"index": index, "attempt": attempt.get("attempt")})
    return result


def config_context(summary):
    phases, issues = {}, []
    for name in ("before", "after"):
        snapshots = summary.get("snapshots", {}).get(name, {})
        opening = snapshots.get("CONFIG?", {}).get("fields", {})
        closing = snapshots.get("CONFIG?_closing", {}).get("fields", {})
        coherent = all(opening.get(key) is not None and opening.get(key) == closing.get(key) for key in CONFIG_FIELDS)
        valid_boot = bool(re.fullmatch(r"[0-9A-Fa-f]{16}", str(opening.get("session", "")))) and uint(opening.get("bootId"))["known"]
        phases[name] = {"opening": opening, "closing": closing, "coherent": coherent, "boot_identity_valid": valid_boot}
        if not coherent:
            issues.append(name + ":config_bracket_missing_or_changed")
        if not valid_boot:
            issues.append(name + ":boot_identity_unknown")
    a, b = phases["before"]["opening"], phases["after"]["opening"]
    same_boot = all(phases[name]["boot_identity_valid"] for name in phases) and (
        a["session"].lower() == b["session"].lower() and int(a["bootId"]) == int(b["bootId"]))
    if not same_boot:
        issues.append("boot_or_session_changed_or_unknown")
    return {"phases": phases, "same_boot_session": same_boot,
            "same_configuration": not issues and all(a.get(key) == b.get(key) for key in CONFIG_FIELDS), "issues": issues}


def diagnostic_context(summary, command, config):
    before, after = (query_window(summary, name, command) for name in ("before", "after"))
    a, b = before["sample_time_bounds"], after["sample_time_bounds"]
    issues = list(config["issues"])
    interval = None
    if not a or not b:
        issues.append("query_timing_missing_or_invalid")
    elif b["started_monotonic"] < a["finished_monotonic"]:
        issues.append("query_windows_overlap_or_are_reversed")
    else:
        interval = {"minimum_seconds": b["started_monotonic"] - a["finished_monotonic"],
                    "maximum_seconds": b["finished_monotonic"] - a["started_monotonic"],
                    "reply_end_to_end_seconds": b["finished_monotonic"] - a["finished_monotonic"],
                    "scope": "separate_diagnostic_query_samples_NOT_capture_only"}
    capture_start, capture_end = summary.get("capture_monotonic_start"), summary.get("capture_monotonic_end")
    brackets = None
    if a and b and finite_time(capture_start) and finite_time(capture_end) and capture_start <= capture_end:
        brackets = a["finished_monotonic"] <= capture_start <= capture_end <= b["started_monotonic"]
    return {"before": before, "after": after, "sample_interval_bounds": interval,
            "queries_bracket_capture": brackets, "delta_context_valid": not issues, "issues": issues}


def scheduler_health(fields):
    reasons = []
    if not re.fullmatch(r"0x[0-9A-Fa-f]{1,8}", str(fields.get("initError", ""))):
        reasons.append("init_error_unknown")
    elif int(fields["initError"], 16) != 0:
        reasons.append("hook_init_error")
    if fields.get("registered") != "1":
        reasons.append("hook_unregistered_or_unknown")
    generation = uint(fields.get("idleGen"))
    if fields.get("idleKnown") != "1" or not generation["known"] or generation["value"] == 0:
        reasons.append("idle_progress_unknown")
    for key in ("idleAgeTicks", "maxIdleGapTicks"):
        if not uint(fields.get(key), sentinel_unknown=True)["known"]:
            reasons.append(key + "_unknown")
    for key in ("tickHz", "budgetTicks", "maxUnits"):
        value = uint(fields.get(key))
        if not value["known"] or value["value"] == 0:
            reasons.append(key + "_unavailable")
    return {"usable_diagnostic_state": not reasons, "reasons": reasons,
            "is_firmware_acceptance": False}


def worker_values(raw):
    values = raw.split("/") if isinstance(raw, str) else []
    return values if len(values) == 3 else [None, None, None]


def analyze(summary):
    config = config_context(summary)
    serial = diagnostic_context(summary, "SERIAL?", config)
    a, b = serial["before"]["raw_fields"], serial["after"]["raw_fields"]
    serial["counters"] = {key: counter(a.get(key), b.get(key), bits, serial["issues"]) for key, bits in SERIAL_COUNTERS.items()}
    serial["endpoints"] = {
        "maxQueueAgeUs": endpoints(a.get("maxQueueAgeUs"), b.get("maxQueueAgeUs"), kind="maximum"),
        "driverTxBufferBytes": endpoints(a.get("driverTxBufferBytes"), b.get("driverTxBufferBytes")),
    }
    sched = diagnostic_context(summary, "SCHED?", config)
    a, b = sched["before"]["raw_fields"], sched["after"]["raw_fields"]
    sched["health"] = {"before": scheduler_health(a), "after": scheduler_health(b)}
    sched["state_endpoints"] = {key: {"before": a.get(key), "after": b.get(key)} for key in ("initError", "registered", "idleKnown")}
    sched["endpoints"] = {key: endpoints(a.get(key), b.get(key), sentinel_unknown=key in ("idleAgeTicks", "maxIdleGapTicks"),
                                         kind="maximum" if key == "maxIdleGapTicks" else "gauge")
                           for key in ("tickHz", "budgetTicks", "maxUnits", "graceUnits", "backstopTicks", "quantumTicks", "idleAgeTicks", "maxIdleGapTicks")}
    for phase, fields in (("before", a), ("after", b)):
        if fields.get("idleKnown") != "1":
            for key in ("idleAgeTicks", "maxIdleGapTicks"):
                sched["endpoints"][key][phase].update(known=False, value=None, reason="idle_progress_unknown")
    idle_issues = sched["issues"] + ([] if a.get("idleKnown") == b.get("idleKnown") == "1" else ["idle_progress_unknown"])
    sched["idle_generation"] = counter(a.get("idleGen"), b.get("idleGen"), 32, idle_issues, skip_zero=True)
    sched["workers"] = {}
    for worker in WORKERS:
        av, bv = worker_values(a.get(worker)), worker_values(b.get(worker))
        sched["workers"][worker] = {"before_raw": a.get(worker), "after_raw": b.get(worker),
                                   "checkpoints": counter(av[0], bv[0], 32, sched["issues"]),
                                   "yields": counter(av[1], bv[1], 32, sched["issues"]),
                                   "maxBurstUnits": endpoints(av[2], bv[2], kind="maximum")}
    return {"analysis_status": "ANALYZED_NOT_ACCEPTANCE", "source_status": summary.get("status"), "source_error": summary.get("error"),
            "benchmark_mode": summary.get("benchmark_mode"), "rows_requested": summary.get("rows_requested"),
            "capture": {key: summary.get(key) for key in ("capture_requested_seconds", "capture_seconds", "capture_monotonic_start", "capture_monotonic_end")},
            "source_faults": summary.get("firmware_faults"), "source_reset_observations": summary.get("reset_observations"),
            "source_diagnostic_observation_accepted": summary.get("diagnostic_observation_accepted"),
            "configuration_context": config, "SERIAL": serial, "SCHED": sched}


def read_completed(path):
    # Explicit paths only. A collector creates an empty summary early and writes
    # final JSON once; missing/partial/final-hash-less summaries are rejected.
    before = path.stat()
    data = path.read_bytes()
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("summary_changed_during_read")
    summary = json.loads(data)
    if not isinstance(summary, dict) or summary.get("status") not in ("CAPTURED", "ERROR"):
        raise ValueError("summary_not_final_configuration")
    if summary.get("runner_variant") != VARIANT or not re.fullmatch(r"[0-9a-fA-F]{64}", str(summary.get("raw_sha256", ""))):
        raise ValueError("summary_missing_final_load_variant_or_raw_hash")
    return summary, hashlib.sha256(data).hexdigest(), len(data)


def report(paths):
    entries, rejected = [], 0
    for path in paths:
        try:
            summary, sha, length = read_completed(path)
            entry = analyze(summary)
            entry.update(source_summary=str(path), source_summary_sha256=sha, source_summary_bytes=length,
                         source_raw_sha256=summary["raw_sha256"], raw_file_not_opened=True)
        except (OSError, ValueError, TypeError, AttributeError) as error:
            rejected += 1
            entry = {"analysis_status": "REJECTED_INPUT", "source_summary": str(path), "reason": str(error)}
        entries.append(entry)
    return {"schema": SCHEMA, "notes": NOTES, "inputs_rejected": rejected, "results": entries,
            "analyzer_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}, int(rejected != 0)


def write_report(value, output):
    text = json.dumps(value, indent=2, allow_nan=False) + "\n"
    if output:
        with output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    else:
        sys.stdout.write(text)


def synthetic_summary():
    config = {key: "1" for key in CONFIG_FIELDS}
    config.update(session="0123456789abcdef", mode="VOLT", state="VOLTAGE", rows="1", profile="VVVVVVVV", err="0x0")
    serial = {key: "1" for key in SERIAL_COUNTERS}
    serial.update(maxQueueAgeUs="7", driverTxBufferBytes="2304")
    sched = {"initError": "0x0", "registered": "1", "idleKnown": "1", "tickHz": "100", "budgetTicks": "2",
             "maxUnits": "32", "idleGen": "4294967295", "idleAgeTicks": "0", "maxIdleGapTicks": "1",
             **{worker: "4294967295/2/31" for worker in WORKERS}}
    value = {"runner_variant": VARIANT, "status": "CAPTURED", "error": None, "raw_sha256": "a" * 64,
             "capture_monotonic_start": 12.0, "capture_monotonic_end": 132.0,
             "snapshots": {}, "query_attempts": []}
    for phase, t in (("before", 10.0), ("after", 134.0)):
        value["snapshots"][phase] = {key: {"fields": copy.deepcopy(fields), "started_monotonic": t + offset,
                                          "finished_monotonic": t + offset + 0.1}
                                     for key, fields, offset in (("CONFIG?", config, 0), ("SERIAL?", serial, 0.2),
                                                                 ("SCHED?", sched, 0.4), ("CONFIG?_closing", config, 0.6))}
    value["snapshots"]["after"]["SCHED?"]["fields"].update(idleGen="1", **{w: "1/5/32" for w in WORKERS})
    return value


class SyntheticTests(unittest.TestCase):
    def test_optional_quantum_fields_preserve_old_unknown(self):
        value = synthetic_summary()
        result = analyze(value)["SCHED"]["endpoints"]
        self.assertFalse(result["quantumTicks"]["before"]["known"])
        for phase in ("before", "after"):
            value["snapshots"][phase]["SCHED?"]["fields"].update(
                quantumTicks="2", budgetTicks="4", backstopTicks="4", graceUnits="0")
        result = analyze(value)["SCHED"]["endpoints"]
        self.assertEqual(result["quantumTicks"]["after"]["value"], 2)
        self.assertEqual(result["graceUnits"]["after"]["value"], 0)
        self.assertNotIn("delta", result["quantumTicks"])

    def test_width_wrap_zero_skip_and_seven_workers(self):
        value = synthetic_summary()
        before, after = (value["snapshots"][p]["SERIAL?"]["fields"] for p in ("before", "after"))
        before.update(normalSubmitted=str(2**32 - 1), driverAcceptedBytes=str(2**64 - 1))
        after.update(normalSubmitted="1", driverAcceptedBytes="2")
        result = analyze(value)
        self.assertEqual(result["SERIAL"]["counters"]["normalSubmitted"]["delta"], 2)
        self.assertEqual(result["SERIAL"]["counters"]["driverAcceptedBytes"]["delta"], 3)
        self.assertEqual(result["SCHED"]["idle_generation"]["delta"], 1)
        self.assertEqual(tuple(result["SCHED"]["workers"]), WORKERS)
        for worker in result["SCHED"]["workers"].values():
            self.assertEqual(worker["checkpoints"]["delta"], 2)
            self.assertEqual(worker["yields"]["delta"], 3)
            self.assertNotIn("delta", worker["maxBurstUnits"])

    def test_maxima_and_query_windows_are_not_capture(self):
        result = analyze(synthetic_summary())
        self.assertNotIn("delta", result["SERIAL"]["endpoints"]["maxQueueAgeUs"])
        self.assertNotIn("delta", result["SCHED"]["endpoints"]["maxIdleGapTicks"])
        self.assertTrue(result["SERIAL"]["queries_bracket_capture"])
        self.assertAlmostEqual(result["SERIAL"]["sample_interval_bounds"]["minimum_seconds"], 123.9)
        self.assertNotEqual(result["SERIAL"]["before"]["sample_time_bounds"], result["SCHED"]["before"]["sample_time_bounds"])

    def test_successful_attempt_narrows_retry_window(self):
        value = synthetic_summary()
        record = value["snapshots"]["before"]["SERIAL?"]
        record.update(started_monotonic=8.0, attempt_begin=0, attempt_end=2)
        value["query_attempts"] = [
            {"command": "SERIAL?", "phase": "before", "attempt": 1, "outcome": "failed", "started_monotonic": 8.0, "finished_monotonic": 9.0},
            {"command": "SERIAL?", "phase": "before", "attempt": 2, "outcome": "complete_reply", "started_monotonic": 10.25, "finished_monotonic": 10.29}]
        result = analyze(value)["SERIAL"]["before"]
        self.assertEqual(result["collection_window"]["started_monotonic"], 8.0)
        self.assertEqual(result["sample_time_bounds"]["started_monotonic"], 10.25)
        self.assertEqual(result["successful_attempt"]["attempt"], 2)

    def test_unknown_and_init_error_are_preserved(self):
        value = synthetic_summary()
        value["snapshots"]["before"]["SCHED?"]["fields"].update(initError="0x105", registered="0", idleKnown="0", idleGen="0",
                                                                 idleAgeTicks="unknown", maxIdleGapTicks="4294967295")
        result = analyze(value)["SCHED"]
        self.assertEqual(result["state_endpoints"]["initError"]["before"], "0x105")
        self.assertFalse(result["health"]["before"]["usable_diagnostic_state"])
        self.assertIsNone(result["idle_generation"]["delta"])
        self.assertEqual(result["endpoints"]["idleAgeTicks"]["before"]["raw"], "unknown")
        self.assertFalse(result["endpoints"]["maxIdleGapTicks"]["before"]["known"])

    def test_boot_change_or_incoherent_bracket_blocks_deltas(self):
        for change_boot in (True, False):
            value = synthetic_summary()
            value["snapshots"]["after"]["CONFIG?_closing"]["fields"]["rev"] = "2"
            if change_boot:
                for key in ("CONFIG?", "CONFIG?_closing"):
                    value["snapshots"]["after"][key]["fields"].update(session="fedcba9876543210", bootId="2", rev="2")
            result = analyze(value)
            self.assertIsNone(result["SERIAL"]["counters"]["normalSubmitted"]["delta"])
            self.assertIsNone(result["SCHED"]["workers"]["ser"]["yields"]["delta"])

    def test_missing_unknown_and_bad_timing_never_become_zero(self):
        value = synthetic_summary()
        del value["snapshots"]["after"]["SERIAL?"]["fields"]["normalExpired"]
        value["snapshots"]["after"]["SCHED?"]["fields"]["wc"] = "1/2"
        result = analyze(value)
        self.assertIsNone(result["SERIAL"]["counters"]["normalExpired"]["delta"])
        self.assertIsNone(result["SCHED"]["workers"]["wc"]["yields"]["delta"])
        value["snapshots"]["after"]["SERIAL?"]["started_monotonic"] = 9.0
        self.assertFalse(analyze(value)["SERIAL"]["delta_context_valid"])

    def test_final_summary_only_and_exclusive_output(self):
        with tempfile.TemporaryDirectory(prefix="sa-load-diagnostics-synthetic-") as name:
            directory = Path(name)
            source, output = directory / "synthetic.summary.json", directory / "report.json"
            for text in ("", "{", json.dumps({"status": "RUNNING"}), json.dumps({"status": "CAPTURED"})):
                source.write_text(text)
                value, code = report([source])
                self.assertEqual(code, 1)
                self.assertEqual(value["results"][0]["analysis_status"], "REJECTED_INPUT")
            fixture = synthetic_summary()
            fixture.update(status="ERROR", error="synthetic_watchdog_failure")
            source.write_text(json.dumps(fixture))
            original = source.read_bytes()
            value, code = report([source])
            self.assertEqual(code, 0)  # Analysis completion is not firmware PASS.
            self.assertEqual(value["results"][0]["source_status"], "ERROR")
            write_report(value, output)
            with self.assertRaises(FileExistsError):
                write_report(value, output)
            self.assertEqual(source.read_bytes(), original)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, action="append", default=[], help="Explicit completed load summary; repeat for multiple records.")
    parser.add_argument("--output", type=Path, help="New report path (exclusive creation); omit for stdout.")
    parser.add_argument("--self-test", action="store_true", help="Only temporary synthetic fixtures; no capture files read.")
    args = parser.parse_args(argv)
    if args.self_test:
        if args.summary or args.output:
            parser.error("--self-test cannot be combined with input or output paths")
        result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SyntheticTests))
        return 0 if result.wasSuccessful() else 1
    if not args.summary:
        parser.error("at least one explicit --summary is required")
    value, code = report(args.summary)
    try:
        write_report(value, args.output)
    except OSError as error:
        parser.error(str(error))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
