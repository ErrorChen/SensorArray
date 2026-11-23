"""Assess closed screen20 numerically without modifying any recorded input."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[5]
base = Path(__file__).resolve().parents[1]
screen = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def ref(path):
    return {"path": str(path.relative_to(root)), "sha256": sha(path)}


criteria_path = base / "acceptance-criteria-20261121-v2.json"
verdict_path = screen / "screen-20-verdict.json"
analysis_path = screen / "screen-20-analysis-01.json"
protection_path = screen / "screen-20-protection-01.json"
post_path = base / "ble-snapshot-postscreen20-01.summary.json"
closure_paths = [base / name for name in (
    "screen-20-volt1-command.json", "screen-20-ble-command.json",
    "screen-20-debug-command.json", "ble-snapshot-postscreen20-01-command.json",
    "screen20-analysis-01-command.json", "screen20-cpu-01-command.json",
    "screen20-protection-01-command.json", "screen-20-verdict-01-command.json")]
inputs = [criteria_path, verdict_path, analysis_path, protection_path, post_path,
          *closure_paths]
before = {str(path): sha(path) for path in inputs}
criteria, verdict, analysis, protection, post = map(load, inputs[:5])
assert sha(criteria_path) == "e0ef35a43a75932b8878aaa5c7cc94a6b562a9e4e1f86e50055ccd164d95ba3c"
assert all(load(path)["exit_code"] == 0 and load(path)["finished_utc"]
           for path in closure_paths)
assert verdict["status"] == "FAIL_BLE_THROUGHPUT"
assert verdict["running_image"]["elf_sha256"] == "7f2d5ad9c73887f70d49c7f8eb45ca889c4c44fcb1fedd293299200d9b39b469"
assert analysis["ble_journal"]["recording_end_observed"] is True
assert analysis["ble_journal"]["source"]["file_grew_during_analysis"] is False
capture = next(c for c in analysis["captures"] if c["label"] == "screen-20-volt1")
assert len(capture["configuration_groups"]) == len(capture["ble"]["configuration_groups"]) == 1
usb = capture["configuration_groups"][0]
ble = capture["ble"]["configuration_groups"][0]
assert ble["identity"] == usb["identity"] == verdict["screen"]["configuration_identity"]
received = ble["counts"]["ordered_integrity_valid_frames"]
missing = ble["counts"]["missing_frames"]
expected = received + missing
assert expected == ((ble["sequence"]["last"] - ble["sequence"]["first"]) & 0xffffffff) + 1
assert (received, missing, expected) == (7461, 202, 7663)
limit = criteria["normal_link"]["nominal_gap_budget_fraction"]
assert limit == 0.025
flow = protection["queries"]["BLEFLOW?"]
net = protection["queries"]["SNAPNET?"]
snap = protection["queries"]["BLESNAP?"]
assert all(q["endpoints_validated"] for q in protection["queries"].values())
assert flow["counter_deltas"]["intentionalSkip"]["delta"] == 0
assert net["counter_deltas"]["policySkip"]["delta"] == 0
assert all(flow[ep]["fields"]["ratePm"] == "1000" and
           flow[ep]["fields"]["stage"] in {"full", "pressure"} for ep in ("before", "after"))
assert all(snap["counter_deltas"][key]["delta"] == 0
           for key in ("queued", "localComplete", "staleNormal", "deadlineMiss"))
assert flow["counter_deltas"]["capacityReject"]["delta"] == 179
post_fields = {a["command"]: a["fields"] for a in post["attempts"] if "fields" in a}
assert post_fields["BUILD?"]["elfSha256"] == verdict["running_image"]["elf_sha256"]
assert post_fields["CONFIG?"]["session"] == ble["identity"]["session"]
assert (post_fields["CONFIG?"]["mode"], post_fields["CONFIG?"]["rows"],
        post_fields["CONFIG?"]["err"]) == ("CAP", "8", "0x0")
record = {
    "schema_version": 1,
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "status": "EXCEEDS_REVISED_NORMAL_LINK_GAP_BUDGET",
    "overall_acceptance": "NOT_ESTABLISHED",
    "assessment_kind": "supplemental_numeric_assessment_of_closed_screen20",
    "criteria_id": criteria["criteria_id"], "criteria": ref(criteria_path),
    "original_verdict": {**ref(verdict_path), "status": verdict["status"], "modified": False},
    "inputs": [ref(path) for path in inputs],
    "closure": {"all_collectors_and_analysis_commands_exit_zero": True,
                "BLE_recording_end_observed": True, "BLE_journal_grew_during_analysis": False,
                "command_records": [ref(path) for path in closure_paths]},
    "evaluated_recording": {
        "image": verdict["running_image"], "configuration_identity": ble["identity"],
        "capture_seconds": verdict["screen"]["window_seconds"],
        "capture_fps": verdict["screen"]["capture_fps"],
        "USB_fps": usb["rates"]["host_complete_receive_fps"],
        "BLE_fps": ble["rates"]["host_complete_receive_fps"],
        "USB_trusted_frames": usb["counts"]["ordered_integrity_valid_frames"],
        "USB_missing_observations": verdict["screen"]["USB"]["missing_observations"],
        "USB_sequence": {key: usb["sequence"][key] for key in ("first", "last")},
        "BLE_sequence": {key: ble["sequence"][key] for key in ("first", "last")},
        "common_trusted_original_byte_matches": capture["ble"]["USB_comparison"]["same_wire_bytes_sha256"]},
    "normal_link_gap_budget_dimension": {
        "received_ordered_trusted_frames": received,
        "missing_trusted_sequence_observations": missing,
        "expected_observations_within_BLE_endpoints": expected,
        "exact_ratio": f"{missing}/{expected}",
        "observed_fraction": missing / expected, "observed_percent": 100 * missing / expected,
        "nominal_limit_fraction": limit, "nominal_limit_percent": 100 * limit,
        "within_nominal_budget": missing / expected <= limit,
        "denominator_scope": "BLE received+missing within one immutable identity and BLE endpoints1108..8770; USB endpoints1110..8773 contain7664frames and are not the BLE denominator.",
        "sampled_stage_before_after": [flow[ep]["fields"]["stage"] for ep in ("before", "after")],
        "sampled_rate_permille": [int(flow[ep]["fields"]["ratePm"]) for ep in ("before", "after")],
        "intentional_output_decimation_subtracted": False,
        "local_rejections_added_to_missing": False,
        "local_stale_or_periodic_counts_subtracted": False,
        "RF_or_RSSI_classification": "NOT_ESTABLISHED_FROM_THIS_GAP_RATIO"},
    "local_protection_windows": {
        "scope": "Each query has its own minimum/maximum host-bracket interval, not exact PC capture boundaries.",
        "queries": {query: {
            "capability": data["capability"], "status": data["status"],
            "host_snapshot_interval_seconds": data["host_snapshot_interval_seconds"],
            "counter_deltas": {key: value["delta"] for key, value in data["counter_deltas"].items()},
            "before_observed_state": data["before"].get("observed_state"),
            "after_observed_state": data["after"].get("observed_state")}
            for query, data in protection["queries"].items()},
        "coalescing_observation": "Source periodic+119 and Transport candidates+128/derived+9 coexist with zero new BLE protected queued/completed/staleNormal/deadlineMiss. Ordinary typed FULL completions satisfy the private full obligation; existing BLESNAP remains protected-only, and its age is not an all-full or PC interval.",
        "remaining_pressure": "CapacityReject+179/attempts+7669 remains observed in its own query bracket. This is local admission pressure, not proven radio loss or exact attribution of all202 PC missing observations; do not subtract counters from202. The added internal2304-byte working copy is a performance hypothesis, and lower rejection counts across different sessions/load do not establish causality or benefit.",
        "PC_snapshot_receipt_from_local_counters": "unknown", "RF_loss": "unknown", "RSSI": "unknown"},
    "PC_adjacent_complete_frame_interval": ble["host_complete_frame_intervals"],
    "science_unchanged": {
        "ADC_CRC": verdict["screen"]["ADC_CRC"],
        "USB_invalid_points": usb["counts"]["invalid_points"],
        "BLE_invalid_points": ble["counts"]["invalid_points"],
        "invalid_by_cell": verdict["screen"]["invalid_by_cell"],
        "meaning": "Complete CRC/identity-valid protocol frames preserve invalid/Xhh points, not all-point validity or absolute precision. ADC CRC checked reads have a separate119.29612240701681-second diagnostic interval."},
    "post_capture_readonly": {
        **ref(post_path), "CONFIG": post_fields["CONFIG?"], "MEM": post_fields["MEM?"],
        "BLEGUARD": post_fields["BLEGUARD?"],
        "scope": "Separate later readonly session; sequential MEM observations do not establish capture-window internal-largest-block minima or exclusive allocation deltas."},
    "other_dimensions": {
        "startup_recovery": "VERIFIED_ON_THIS_IMAGE", "continuous600_seconds": "NOT_RUN_ON_THIS_IMAGE",
        "full32_configuration_suite": "NOT_RUN_ON_THIS_IMAGE", "50_configuration_transitions": "NOT_RUN_ON_THIS_IMAGE",
        "SHORT_protected_snapshot_HIL": "NOT_RUN_ON_THIS_IMAGE", "weak_link_HIL": "NOT_RUN_ON_THIS_IMAGE",
        "absolute_precision": "NOT_EVALUATED", "whole_project_P0_P1_absence": "NOT_ESTABLISHED"},
    "preservation": {"originals_modified": False, "raw_CRC_or_samples_rewritten": False,
                     "raw_recordings_retained": True},
    "generator": {"path": str(Path(__file__).relative_to(root)), "sha256": sha(Path(__file__))},
    "generator_command_record": str((base / "screen-20-criteria-v2-supplement-01-command.json").relative_to(root)),
    "hardware_actions": [], "production_source_modified": False}
assert record["normal_link_gap_budget_dimension"]["within_nominal_budget"] is False
for path in inputs:
    assert sha(path) == before[str(path)], "Input changed during assessment"
output = screen / "screen-20-criteria-v2-supplement.json"
with output.open("x") as stream:
    json.dump(record, stream, ensure_ascii=False, indent=2)
    stream.write("\n")
print(json.dumps({"status": record["status"], "ratio": f"{missing}/{expected}",
                  "percent": record["normal_link_gap_budget_dimension"]["observed_percent"],
                  "overall": record["overall_acceptance"], "sha256": sha(output)}))
