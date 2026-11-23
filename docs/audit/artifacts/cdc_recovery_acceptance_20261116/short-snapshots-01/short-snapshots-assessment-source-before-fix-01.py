"""Summarize the closed real SHORT run without applying the FULL gap budget."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[5]
base = Path(__file__).resolve().parents[1]
here = Path(__file__).resolve().parent


def load(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


paths = [here / name for name in (
    "short-snapshots-analysis-01.json", "short-snapshots-cpu-01.json",
    "short-snapshots-01-protection-03.json", "short-snapshots-01-cap8.summary.json")]
paths += [base / name for name in (
    "ble-snapshot-04-validated-identity.json", "short-snapshots-protection-legacy-closure-02.json",
    "ble-protection-short-closure-host-02.json", "short-snapshots-protection-01-command.json")]
before = {str(path): sha(path) for path in paths}
analysis, cpu, protection, summary, image, closure_run, tests, initial_failure = map(load, paths)
capture = analysis["captures"][0]
usb, ble = capture["configuration_groups"][0], capture["ble"]["configuration_groups"][0]
assert usb["identity"] == ble["identity"] == cpu["configuration_identity"]
assert image["elf_sha256"] == "a6b04a2a68ddcb5c8c39cc28020abbe5db430c1e140cbb3afb4ddb703fdb217a"
assert summary["status"] == "CAPTURED" and summary["capture_seconds"] >= 120
assert summary["safe_cap_recovery"] == "CAP8_QUERY_VERIFIED" and summary["tx_restore"] == "QUERY_VERIFIED"
assert protection["closure"]["kind"] == "legacy_SHORT_restore_tail_verified"
assert protection["closure"]["explicit_final_finished_event"] is False
assert closure_run["original_unchanged"] is True and closure_run["exit_code"] == tests["exit_code"] == 0
assert initial_failure["exit_code"] == 1
assert all(query["status"] == "OBSERVED_LOCAL_COUNTERS" for query in protection["queries"].values())
assert usb["counts"]["ordered_integrity_valid_frames"] == 1416
assert ble["counts"]["ordered_integrity_valid_frames"] == 118
assert capture["ble"]["USB_comparison"]["same_wire_bytes_sha256"] == 118
assert usb["sequence"]["progress_including_missing"] == 1415
for group in (usb, ble):
    assert all(group["counts"][key] == 0 for key in ("wire_contract_errors", "ascii_crc_errors", "identity_errors", "invalid_points", "unacquired_points", "not_fresh_points"))
record = {
    "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
    "status": "PERIODIC_FULL_SNAPSHOTS_OBSERVED_IN_SHORT_MODE",
    "overall_acceptance": "NOT_ESTABLISHED",
    "inputs": [{"path": str(path.relative_to(root)), "sha256": before[str(path)]} for path in paths],
    "running_image": image, "configuration_identity": usb["identity"],
    "capture_seconds": summary["capture_seconds"],
    "USB": {key: usb[key] for key in ("rates", "counts", "sequence", "host_complete_frame_intervals")},
    "BLE_full": {key: ble[key] for key in ("rates", "counts", "sequence", "host_complete_frame_intervals")},
    "BLE_logical_message_counts": capture["ble"]["counts"],
    "original_full_bytes_comparison": capture["ble"]["USB_comparison"],
    "normal_FULL_gap_budget": {"status": "NOT_APPLICABLE_TO_SHORT_PREVIEW_STREAM",
        "reason": "Ordinary CAP data uses SHORT previews while periodic protected data is full. Missing full-frame sequence observations retain their raw count1287 but are not random transport loss against all captured frames.",
        "counter_rewritten_or_subtracted": False},
    "protection_queries": protection["queries"], "closure": protection["closure"],
    "debug": {"faults": capture["otg_diagnostics"]["CDC"]["faults"],
              "resets": capture["otg_diagnostics"]["CDC"]["resets"],
              "memory": capture["otg_diagnostics"]["memory"],
              "ADC_CRC_status": capture["otg_diagnostics"]["adc_crc"]["status"]},
    "CPU": {"roles": cpu["roles"], "total": "UNKNOWN", "SDK_CPU": "UNKNOWN", "blocked": "UNKNOWN"},
    "limitations": [
        "PC maximum observed full-frame gap1.0281269339902792s is not a strict one-second delivery guarantee and excludes capture edges.",
        "Local protected completion/query intervals are separate from PC observation; no per-ticket PC ACK exists.",
        "CAP8 was queried before the historical TX-only restore. No post-restore CONFIG or second finished event is invented.",
        "Every observed point is valid, but absolute capacitance precision, baseline stability and weak-link delivery are not established.",
        "ADC CRC is unobserved during this CAP run, not a measured zero rate.",
        "The original failed analyzer log remains; later compatibility validation does not rewrite source recordings or declare the main FULL throughput screen passed."],
    "generator": {"path": str(Path(__file__).relative_to(root)), "sha256": sha(Path(__file__))},
    "hardware_actions": [], "production_modified": False, "original_recordings_modified": False}
for path in paths:
    assert sha(path) == before[str(path)]
output = here / "short-snapshots-assessment-01.json"
with output.open("x") as stream:
    json.dump(record, stream, ensure_ascii=False, indent=2)
    stream.write("\n")
print(json.dumps({"status": record["status"], "USB_full": 1416, "BLE_full": 118,
                  "max_complete_gap_seconds": ble["host_complete_frame_intervals"]["max_seconds"],
                  "overall": record["overall_acceptance"], "sha256": sha(output)}))
