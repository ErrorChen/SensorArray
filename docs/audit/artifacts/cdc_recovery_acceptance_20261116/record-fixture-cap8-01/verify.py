#!/usr/bin/env python3
"""Offline verification of an already closed fixture extraction; no device IO."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load(path):
    return json.loads(path.read_text())


def read_original(path):
    if not path.exists():
        path = path.with_name(path.name + ".gz")
    stored = path.read_bytes()
    raw = gzip.decompress(stored) if path.suffix == ".gz" else stored
    return raw, {"path": str(path), "stored_bytes": len(stored),
                 "stored_sha256": digest(stored), "raw_bytes": len(raw),
                 "raw_sha256": digest(raw)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--output-prefix", required=True)
    args = parser.parse_args()
    directory = args.directory.resolve()
    output = args.output_directory.resolve()
    source = lambda suffix: directory / (args.label + "." + suffix)
    derived = lambda suffix: output / (args.output_prefix + "." + suffix)
    command = load(source("command.json"))
    assert command["child_termination_known"] and command["exit_code"] == 0
    assert command["error"] is None
    assert {v["kind"] for v in command["observer_closure"]} == {"ble", "debug"}
    assert all(v["termination_known"] and v["exit_code"] == 0 and
               not v["cleanup_errors"] for v in command["observer_closure"])
    assert digest(source("summary.json").read_bytes()) == command["summary_sha256"]
    summary = load(source("summary.json"))
    gate = load(source("gate.json"))
    selection = load(derived("selection.json"))
    transport = load(derived("transport.json"))
    fixture = load(derived("fixture.json"))
    metrics = load(derived("metrics.json"))
    raw, raw_provenance = read_original(Path(selection["sources"]["raw"]["path"]))
    assert digest(raw) == summary["raw_sha256"] == selection["sources"]["raw"]["raw_sha256"]
    assert len(raw) == summary["rx_bytes"] == selection["sources"]["raw"]["raw_bytes"]
    selected, selected_provenance = read_original(derived("rx.bin"))
    position = 0
    for frame in selection["selected"]:
        original = raw[frame["source_start"]:frame["source_end_exclusive"]]
        assert digest(original) == frame["source_sha256"]
        assert frame["output_start"] == position
        assert selected[frame["output_start"]:frame["output_end_exclusive"]] == original
        position = frame["output_end_exclusive"]
    assert position == len(selected)
    assert digest(selected) == selection["outputs"]["raw"]["sha256"]
    assert selection["counts"]["benchmark_count_matches"]
    original_gate_capture = gate["analysis"]["captures"][0]
    for key in ("capture_counts", "configuration_groups", "faults_and_resets"):
        assert transport[key] == original_gate_capture[key], key
    assert len(fixture["groups"]) == len(metrics["groups"]) == 1
    group = fixture["groups"][0]
    assert group["frames"] == len(selection["selected"])
    assert group["screening"] == "WINDOW_SPAN_MET"
    assert group["timestamp_span_seconds"] >= 120.0
    assert not any(group[key] for key in ("sequence_gaps", "missing_sequences",
                                        "duplicate_or_old_frames", "timestamp_regressions"))
    assert not any(fixture["capture_errors"].values())
    assert metrics["precision_acceptance"] == "NOT_EVALUATED"
    assert metrics["calibration_written"] is False
    assert all(v["verified"] for v in summary["image_authority"]["brackets"])
    assert summary["safe_cap_recovery"] == "CAP8_QUERY_VERIFIED"
    files = {}
    for name, path in [("command", source("command.json")), ("gate", source("gate.json")),
                       ("summary", source("summary.json")), ("selection", derived("selection.json")),
                       ("verifier", Path(__file__).resolve())]:
        data = path.read_bytes()
        files[name] = {"path": str(path), "bytes": len(data), "sha256": digest(data)}
    report = {"schema_version": 1, "status": "VERIFIED_RECORDED_FIXTURE_EXTRACTION",
              "scope": "Closed original USB file extraction and offline gate comparison only",
              "files": files, "source_raw": raw_provenance, "derived_raw": selected_provenance,
              "all_selected_ranges_byte_identical": True,
              "gate_USB_counts_groups_and_faults_exact_match": True,
              "command_exit_code": command["exit_code"], "gate_status": gate["status"],
              "observer_closure": [{k: v[k] for k in ("kind", "termination_known", "exit_code", "cleanup_errors")}
                                   for v in command["observer_closure"]],
              "image_identity": summary["image_authority"]["expected"],
              "image_brackets_verified": True, "safe_cap_recovery": summary["safe_cap_recovery"],
              "group": {k: group[k] for k in ("identity", "frame_type", "rows", "frames", "first_sequence",
                        "last_sequence", "timestamp_span_seconds", "screening", "sequence_gaps",
                        "missing_sequences", "duplicate_or_old_frames", "timestamp_regressions")},
              "USB_counts": transport["capture_counts"],
              "BLE_counts": original_gate_capture["ble"]["capture_frame_counts"],
              "USB_BLE_comparison": original_gate_capture["ble"]["USB_comparison"],
              "references": metrics["groups"][0]["references"],
              "precision_acceptance": "NOT_EVALUATED", "calibration_written": False,
              "limitations": ["CAP cross-cell proxy is not a same-cell unloaded baseline or absolute accuracy.",
                              "RES deviations from the nominal 10000 ohm value are descriptive; reference tolerance and acceptance limits were not supplied.",
                              "Min/max and P05/P95 describe observed samples, not accuracy bounds or confidence intervals."]}
    result_path = derived("validation.json")
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if result_path.exists():
        assert result_path.read_text() == encoded, "refusing to overwrite different validation evidence"
    else:
        result_path.write_text(encoded)
    print(json.dumps({"validation": str(result_path), "sha256": digest(encoded.encode()),
                      "frames": group["frames"], "device_span_seconds": group["timestamp_span_seconds"]}))


if __name__ == "__main__":
    main()
