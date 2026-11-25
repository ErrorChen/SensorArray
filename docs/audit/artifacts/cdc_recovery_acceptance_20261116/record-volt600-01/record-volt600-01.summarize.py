#!/usr/bin/env python3
"""Summarize the closed 600 s recording without altering its gate or source bytes."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[5]
STAGE = ROOT / "docs/audit/artifacts/cdc_recovery_acceptance_20261116"
sys.path.insert(0, str(STAGE / "scripts"))
sys.path.insert(0, str(ROOT / "tools"))
import analyze_cpu_owner
import summarize_diagnostic_screen
import review_closed_voltage_frames as review
import sensorarray_bulk_diagnostics as bulk


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--label", required=True)
    args = parser.parse_args()
    directory = args.directory.resolve()
    output = directory / (args.label + ".metrics.json")
    health_output = directory / (args.label + ".health-coverage.json")
    provenance_output = directory / (args.label + ".metrics.provenance.json")
    if any(p.exists() for p in (output, health_output, provenance_output)):
        raise FileExistsError("refusing to replace recorded metrics")
    inputs = {}

    def read(name, path, expected=None):
        path = Path(path)
        if not path.is_absolute():
            path = ROOT / path
        data, meta = review.read_source(bulk.existing(path))
        if expected is not None and digest(data) != expected:
            raise ValueError(name + "_hash_mismatch")
        inputs[name] = meta
        return data

    result_path = directory / (args.label + ".result.json")
    result = json.loads(read("result", result_path))
    assert result["formal"]["child_termination_known"]
    assert result["formal"]["exit_code"] is not None
    assert {x["kind"] for x in result["observer_closure"]} == {"ble", "debug"}
    assert all(x["termination_known"] and x["exit_code"] is not None for x in result["observer_closure"])
    gate = json.loads(read("gate", result["gate"]["path"], result["gate"]["sha256"]))
    assert gate["status"] == result["gate_status"]
    print(json.dumps({"closed_result_status": result["status"], "original_gate_status": gate["status"]}), flush=True)
    summary = json.loads(read("formal_summary", result["formal_summary"]["file"]["path"],
                              result["formal_summary"]["file"]["sha256"]))
    authority = gate["formal_authority"]
    assert authority["summary"]["sha256"] == inputs["formal_summary"]["raw_sha256"]
    raw = read("formal_raw", summary["raw_file"], summary["raw_sha256"])
    event_data = read("formal_events", summary["events_file"], authority["events"]["sha256"])
    events = [json.loads(line) for line in event_data.splitlines()]
    assert digest(raw) == authority["raw"]["sha256"]
    review.assert_closed(summary, events, raw)
    raw_lines = {line.rstrip(b"\r").decode("ascii") for line in raw.split(b"\n")[:-1]
                 if line.startswith((b"CPUOWNER,", b"BLESVC,", b"CONFIG,"))}
    replies = {(e.get("phase"), e.get("command"), e.get("line")) for e in events if e.get("type") == "reply"}
    cpu = analyze_cpu_owner.analyze_document(summary, raw_lines, replies)
    analysis = gate["analysis"]
    capture, = analysis["captures"]
    assert capture["files"]["summary"]["sha256"] == inputs["formal_summary"]["raw_sha256"]
    metrics = summarize_diagnostic_screen.capture_summary(analysis, summary, cpu)

    ble_ref = result["ble_summary"]["file"]
    ble_summary = json.loads(read("BLE_summary", ble_ref["path"], ble_ref["sha256"]))
    assert gate["BLE_summary"]["sha256"] == inputs["BLE_summary"]["raw_sha256"]
    journal_source = analysis["ble_journal"]["source"]
    journal = read("BLE_journal", journal_source["path"], journal_source["uncompressed_prefix_sha256"])
    assert len(journal) == journal_source["uncompressed_prefix_bytes"]
    collector_check = analysis["ble_journal"]["collector_summary_verification"]
    assert digest(journal) == collector_check["journal_sha256"]
    assert len(journal) == collector_check["journal_bytes"]
    assert journal.endswith(b"\n")
    end_records = [json.loads(line) for line in journal.splitlines() if b'"recording_end"' in line]
    end_records = [record for record in end_records if record.get("type") == "recording_end"]
    assert len(end_records) == analysis["ble_journal"]["counts"]["recording_end"] == 1
    # The actual collector emits recording_end before unsubscribe/disconnect;
    # cleanup notifications and the disconnected record may follow it.
    metrics["BLE"]["collector_status"] = ble_summary["status"]
    metrics["BLE"]["original_closed_journal_hash_verified"] = True
    metrics["BLE"]["recording_end"] = end_records[0]
    metrics["BLE"]["final_journal_record_type"] = json.loads(journal.rsplit(b"\n", 2)[-2])["type"]

    debug_ref = result["debug_summary"]["file"]
    debug = json.loads(read("debug_summary", debug_ref["path"], debug_ref["sha256"]))
    assert gate["debug"]["summary"]["sha256"] == inputs["debug_summary"]["raw_sha256"]
    debug_raw = read("debug_raw", debug["raw_file"], gate["debug"]["raw"]["sha256"])
    debug_event_data = read("debug_events", debug["events_file"], gate["debug"]["events"]["sha256"])
    debug_events = [json.loads(line) for line in debug_event_data.splitlines()]
    review.assert_closed(debug, debug_events, debug_raw, debug=True)
    bounds = (summary["capture_monotonic_start"], summary["capture_monotonic_end"])
    diagnostics, _ = review.diagnostic_lines(debug_raw, debug_events, "CDC", 0.0, bounds)
    health, rejected = [], []
    for record in diagnostics:
        if record["tag"] != "ADSHEALTH":
            continue
        error = bulk.record_error("ADSHEALTH", record["line"], record["fields"], None, "otg_cdc_v1")
        if error:
            rejected.append({**record, "rejection": error})
        else:
            health.append(record)
    inside = [r for r in health if r["inside_formal_host_window"]]
    adc_health = metrics["ADC_CRC"]["health"]
    assert len(health) == adc_health["records_in_stream"]
    if inside:
        assert inside[0]["fields"] == adc_health["first"]["fields"]
        assert inside[-1]["fields"] == adc_health["last"]["fields"]
    intervals = Counter(r["fields"]["healthInterval"] for r in inside)
    engines = [int(r["fields"]["engineFrameCount"]) for r in inside]
    steps = [b - a for a, b in zip(engines, engines[1:])]
    monotonic = all(step > 0 for step in steps)
    interval = int(next(iter(intervals))) if len(intervals) == 1 else None
    periodic = None
    if engines and monotonic and interval and interval > 0:
        expected = set(range(((engines[0] + interval - 1) // interval) * interval, engines[-1] + 1, interval))
        observed = set(engines) & expected
        periodic = {"scope": "regular engine-count multiples within first/last observed health bracket only",
                    "expected_slots": len(expected), "observed_slots": len(observed),
                    "missing_slots": sorted(expected - observed),
                    "coverage_fraction": len(observed) / len(expected) if expected else None,
                    "off_lattice_records": [v for v in engines if v % interval],
                    "outside_observed_bracket": "UNKNOWN; no extrapolation to all 600 seconds"}
    health_summary = {
        "status": "OBSERVED" if inside else "NOT_OBSERVED", "records_in_whole_debug_stream": len(health),
        "records_inside_formal_window": len(inside), "rejected_records": len(rejected),
        "whole_debug_rejected_records": capture["otg_diagnostics"]["CDC"]["rejected_records"],
        "debug_trailing_partial_line_bytes": debug.get("trailing_partial_line_bytes"),
        "debug_observer_window_coverage": metrics["ADC_CRC"]["observer_window_coverage"],
        "first": inside[0] if inside else None, "last": inside[-1] if inside else None,
        "declared_intervals": dict(intervals), "engine_step_counts": dict(Counter(steps)),
        "engine_counter_strictly_increasing": monotonic if engines else None,
        "engine_first": engines[0] if engines else None, "engine_last": engines[-1] if engines else None,
        "engine_span": engines[-1] - engines[0] if engines else None,
        "err_values": dict(Counter(r["fields"]["err"] for r in inside)),
        "healthReadbackDue_values": dict(Counter(r["fields"]["healthReadbackDue"] for r in inside)),
        "precisionFrame_values": dict(Counter(r["fields"]["precisionFrame"] for r in inside)),
        "sequence_minus_engine_count_values": dict(Counter(int(r["fields"]["seq"]) - int(r["fields"]["engineFrameCount"]) for r in inside)),
        "regular_engine_lattice_coverage": periodic,
        "crcRetries_terminal_token": "UNKNOWN_without_following_marker; not inferred from apparent terminal digits",
        "limitations": ["Diagnostic receipt time is not device execution time and debug delivery is best effort.",
                        "Register-health scheduling may also run for an invalid MODE0 cache, away from regular multiples.",
                        "ADC CRC denominator counts checked reads, including retries, within a separate diagnostic bracket; it is not a frame count."]}
    metrics.update(original_result_status=result["status"], original_gate_status=gate["status"],
                   original_gate_science=gate["science"], original_result_errors=result["errors"],
                   health_engine_coverage=health_summary,
                   original_gate_and_recordings_modified=False, hardware_actions=[],
                   CPU_ratio_sum=None)
    provenance = {
        "schema_version": 1, "inputs": inputs,
        "generator": {"path": str(Path(__file__).resolve()), "sha256": digest(Path(__file__).read_bytes())},
        "existing_functions_used": ["summarize_diagnostic_screen.capture_summary", "analyze_cpu_owner.analyze_document",
                                    "review_closed_voltage_frames.assert_closed", "review_closed_voltage_frames.diagnostic_lines",
                                    "sensorarray_bulk_diagnostics.record_error"],
        "tool_sources": {str(Path(m.__file__).resolve()): digest(Path(m.__file__).read_bytes())
                         for m in (summarize_diagnostic_screen, analyze_cpu_owner, review, bulk)},
        "original_result_status": result["status"], "original_gate_status": gate["status"],
        "formal_closure": result["formal"], "observer_closure": result["observer_closure"],
        "hardware_actions": [], "original_recordings_modified": False,
        "method": "Reuse finalized gate analysis after exact summary/raw/events hashes; compute CPU from original formal reply bytes and health cadence from independent debug raw/events."}
    for source in inputs.values():
        stored = Path(source["path"]).read_bytes()
        assert digest(stored) == source["stored_sha256"], "source changed after closure read"
    for path, value in ((output, metrics), (health_output, {"summary": health_summary, "records": inside, "rejected_records": rejected}),
                        (provenance_output, provenance)):
        with path.open("x") as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
    print(json.dumps({"metrics": str(output), "gate_status_unchanged": gate["status"],
                      "USB_missing": metrics["USB"]["missing_observations"],
                      "BLE_missing": metrics["BLE"]["missing_observations"],
                      "health_records": len(inside), "health_periodic_coverage": periodic}), flush=True)


if __name__ == "__main__":
    main()
