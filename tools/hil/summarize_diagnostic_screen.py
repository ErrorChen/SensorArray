#!/usr/bin/env python3
"""Summarize closed diagnostic recordings and compare independent run windows.

This is an offline evidence consumer, not an acceptance-threshold calculator.
The reviewer supplies the verdict. Missing observations stay unknown. Outputs
are exclusive-create and no recorded input, firmware or device is modified.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def stability_observation(summary: dict) -> dict:
    """Describe only this recording, never infer acceptance or other runs."""
    if summary.get("benchmark_kind") != "stability":
        return {"status": "NOT_REQUESTED_IN_THIS_CAPTURE", "acceptance": "NOT_EVALUATED"}
    names = ("capture_requested_seconds", "capture_seconds",
             "capture_monotonic_start", "capture_monotonic_end")
    values = [summary.get(name) for name in names]
    numeric = all(type(v) in (int, float) and math.isfinite(v) for v in values)
    complete = False
    if numeric:
        requested, duration, start, end = values
        complete = (requested >= 600 and duration >= requested and end >= start and
                    abs((end-start)-duration) <= 1e-6 and
                    summary.get("capture_duration_satisfied") is True and
                    summary.get("status") == "CAPTURED")
    return {"status": "CAPTURED_600_SECONDS_NOT_ACCEPTED" if complete else "INCOMPLETE_OR_UNKNOWN",
            "acceptance": "NOT_EVALUATED", "scope": "this_single_continuous_recording_only",
            **{name: summary.get(name) for name in names}}


def missing_observations(counts: dict, groups: list) -> dict:
    """Do not interpret an omitted Counter zero as a measured zero."""
    if "missing_frames" in counts:
        value = counts["missing_frames"]
        if type(value) is int and value >= 0:
            return {"status": "KNOWN", "count": value, "basis": "explicit_analyzer_missing_frames"}
        return {"status": "UNKNOWN", "count": None, "basis": "invalid_explicit_count"}
    if len(groups) != 1:
        return {"status": "UNKNOWN", "count": None, "basis": "multiple_or_missing_identity_groups"}
    sequence = groups[0].get("sequence", {})
    n = counts.get("trusted_complete_frames")
    first, last = sequence.get("first"), sequence.get("last")
    if (type(n) is int and n > 0 and
            counts.get("ordered_integrity_valid_frames") == n and
            groups[0].get("counts", {}).get("trusted_complete_frames") == n and
            type(first) is int and type(last) is int and
            0 <= first < 2**32 and 0 <= last < 2**32):
        distance = (last-first) % (2**32)
        if (distance < 2**31 and distance == n-1 and
                sequence.get("progress_including_missing") == distance and
                sequence.get("gap_examples") == []):
            return {"status": "KNOWN", "count": 0,
                    "basis": "one_identity_ordered_count_equals_u32_last_minus_first_plus_one_and_empty_gap_examples",
                    "first": first, "last": last, "ordered_count": n, "progress": distance}
    return {"status": "UNKNOWN", "count": None, "basis": "absence_alone_does_not_prove_zero"}


def layer_window(value: dict) -> dict:
    first, last = value.get("first") or {}, value.get("last") or {}
    return {"status": value.get("status", "UNKNOWN"), "scope": value.get("window", "UNKNOWN"),
            "first_receipt": first.get("monotonic"), "last_receipt": last.get("monotonic"),
            "elapsed_seconds": value.get("elapsed_seconds"),
            "counters": value.get("counters", {}),
            "PC_receipt": "UNKNOWN"}


def capture_summary(analysis: dict, summary: dict, cpu: dict) -> dict:
    captures = analysis["captures"]
    if len(captures) != 1:
        raise ValueError("one_capture_per_input_required")
    c = captures[0]
    if len(c["configuration_groups"]) != 1:
        raise ValueError("one_scientific_identity_group_required")
    group = c["configuration_groups"][0]
    for field in ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid"):
        if group["identity"][field] != cpu["configuration_identity"][field]:
            raise ValueError("CPU_scientific_identity_mismatch:"+field)
    q = c["otg_diagnostics"]["control_queries"]
    layers = {tag: layer_window(q.get(tag, {})) for tag in ("USB", "USBQ", "BLETX")}
    layers["BL50"] = layer_window(c["otg_diagnostics"]["BLE_firmware"])
    layers["BLESVC"] = cpu["BLE_SDK_separate_query_window"]
    return {
        "label": c["label"], "capture_status": c["status"],
        "stability_window": stability_observation(summary),
        "window_seconds": summary["capture_seconds"],
        "requested_seconds_met": c["window"]["requested_seconds_met"],
        "configuration_identity": group["identity"],
        "capture_fps": group["rates"]["capture_fps"],
        "USB": {"FPS": group["rates"]["host_complete_receive_fps"],
                "counts": c["capture_counts"],
                "missing_observations": missing_observations(c["capture_counts"], c["configuration_groups"]),
                "sequence": group["sequence"]},
        "BLE": {"FPS": c["ble"]["comparative_host_fps"], "counts": c["ble"]["capture_frame_counts"],
                "missing_observations": missing_observations(c["ble"]["capture_frame_counts"], c["ble"]["configuration_groups"]),
                "received_fragments": c["ble"]["fragments_by_channel"],
                "common_frames": c["ble"]["USB_comparison"]},
        "ADC_CRC": c["otg_diagnostics"]["adc_crc"],
        "invalid_by_cell": group["invalid_reasons_by_cell"],
        "layer_windows": layers,
        "CPU_owner_roles": cpu["roles"], "CPU_total": "UNKNOWN", "SDK_CPU_time": "UNKNOWN", "blocked_time": "UNKNOWN",
        "memory": c["otg_diagnostics"]["memory"],
        "formal_faults_and_resets": c["faults_and_resets"],
        "CDC_faults": c["otg_diagnostics"]["CDC"]["faults"],
        "CDC_resets": c["otg_diagnostics"]["CDC"]["resets"],
        "WDT_evidence": c["otg_diagnostics"]["WDT_evidence"],
        "precision_acceptance": c["precision_acceptance"],
        "post_capture_recovery": summary.get("safe_cap_recovery", "UNKNOWN"),
    }


def compare_runs(previous: dict, current: dict) -> dict:
    rows = []
    old = {r["role"]: r for r in previous["CPU_owner_roles"]}
    for new in current["CPU_owner_roles"]:
        prior = old.get(new["role"], {})
        def own(r):
            return {key: r.get(key) for key in ("status", "reason", "runtime_delta_us", "observation_span_us",
                                               "runtime_per_own_observation_span_percent")}
        usable = new.get("status") == prior.get("status") == "PAIRED"
        rows.append({"role": new["role"], "name": new["name"], "previous": own(prior), "current": own(new),
                     "ratio_difference_percentage_points": (new["runtime_per_own_observation_span_percent"] -
                         prior["runtime_per_own_observation_span_percent"]) if usable else None})
    return {
        "status": "DESCRIPTIVE_COMPARISON_NOT_CAUSAL_AB",
        "previous_label": previous["label"], "current_label": current["label"],
        "previous_identity": previous["configuration_identity"], "current_identity": current["configuration_identity"],
        "CPU_roles": rows, "CPU_ratio_sum": None, "blocked_time": "UNKNOWN", "SDK_CPU_time": "UNKNOWN",
        "FPS": {name: {"previous": a, "current": b, "difference": b-a} for name, a, b in (
            ("capture", previous["capture_fps"], current["capture_fps"]),
            ("USB", previous["USB"]["FPS"], current["USB"]["FPS"]),
            ("BLE", previous["BLE"]["FPS"], current["BLE"]["FPS"]))},
        "layer_windows": {"previous": previous["layer_windows"], "current": current["layer_windows"]},
        "interpretation": [
            "Each within-run role delta uses its own device sample bracket; ratios are compared descriptively across runs.",
            "No cumulative task/sequence/generation counter is subtracted across boots, even if task numbers match.",
            "Layer counters have independent receipt/query windows; do not sum/subtract them as disjoint losses.",
            "Unavailable/stale CPU remains unknown, not zero. SDK wall minus owner run is not blocked time.",
            "A small ASYNC_LOG ratio change does not establish a repaired BLE bottleneck or the primary cause.",
        ],
    }


def load_run(analysis_path: Path, summary_path: Path, cpu_path: Path) -> tuple[dict, dict]:
    analysis, summary, cpu = read(analysis_path), read(summary_path), read(cpu_path)
    capture = analysis["captures"][0]
    if capture["files"]["summary"]["sha256"] != sha(summary_path) or cpu["inputs"]["summary"]["sha256"] != sha(summary_path):
        raise ValueError("analysis_summary_hash_mismatch")
    raw = Path(summary["raw_file"])
    events = Path(summary["events_file"])
    actual = sha(raw)
    if actual != summary["raw_sha256"] or actual != capture["files"]["raw"]["uncompressed_sha256"]:
        raise ValueError("raw_hash_mismatch")
    if actual != cpu["inputs"]["raw"]["sha256"] or sha(events) != cpu["inputs"]["events"]["sha256"]:
        raise ValueError("CPU_raw_or_event_hash_mismatch")
    return capture_summary(analysis, summary, cpu), {str(p): sha(p) for p in (analysis_path, summary_path, cpu_path)}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("analysis", "summary", "cpu", "identity", "boot", "output", "previous-analysis", "previous-summary", "previous-cpu", "comparison-output"):
        p.add_argument("--"+name, required=True, type=Path)
    p.add_argument("--verdict", required=True, choices=("FAIL_BLE_THROUGHPUT", "INCOMPLETE", "OBSERVED_NOT_ACCEPTED"))
    a = p.parse_args()
    if a.output.exists() or a.comparison_output.exists():
        raise FileExistsError("use new output labels")
    current, inputs = load_run(a.analysis, a.summary, a.cpu)
    previous, previous_inputs = load_run(a.previous_analysis, a.previous_summary, a.previous_cpu)
    identity, boot = read(a.identity), read(a.boot)
    if not re.fullmatch(r"[0-9a-f]{64}", identity["elf_sha256"]):
        raise ValueError("full_ELF_hash_required")
    if (identity["version"] != boot["version"] or identity["elf_sha256"] != boot["elf_sha256"] or
            identity["bin_sha256"] != boot["bin_sha256"]):
        raise ValueError("running_BUILD_identity_mismatch")
    for key in ("session", "bootId"):
        if current["configuration_identity"][key] != boot["config"][key]:
            raise ValueError("capture_boot_identity_mismatch")
    if boot["ready"]["ready"] != "1" or boot["ready"]["err"] != "0x0":
        raise ValueError("boot_not_ready")
    for source in boot["inputs"]:
        if sha(Path(source["path"])) != source["sha256"]:
            raise ValueError("boot_evidence_hash_mismatch")
    inputs.update({str(path): sha(path) for path in (a.identity, a.boot)})
    common = {"generator": {"path": str(Path(__file__)), "sha256": sha(Path(__file__))},
              "hardware_actions": [], "original_recordings_modified": False}
    verdict = {**common, "schema_version": 1, "status": a.verdict, "verdict_source": "explicit_reviewer_argument_not_automatic_threshold",
               "running_image": identity, "inputs": inputs, "screen": current,
               "continuous_600_seconds": current["stability_window"],
               "other_runs_or_32_configuration_acceptance": "NOT_EVALUATED_FROM_THIS_SINGLE_CAPTURE",
               "limitations": ["Full capture duration is not full acceptance or absolute precision validation.",
                               "No observed fault/reset/WDT text is not proof of absence outside available evidence.",
                               "ADC CRC denominator is checked reads within its diagnostic bracket, not scientific frames."]}
    comparison = {**common, "schema_version": 1, "inputs": {**previous_inputs, **inputs}, **compare_runs(previous, current)}
    for path, data in ((a.output, verdict), (a.comparison_output, comparison)):
        with path.open("x") as stream:
            json.dump(data, stream, indent=2); stream.write("\n")
    print(json.dumps({"status": verdict["status"], "label": current["label"],
                      "USB_missing": current["USB"]["missing_observations"],
                      "comparison": comparison["status"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
