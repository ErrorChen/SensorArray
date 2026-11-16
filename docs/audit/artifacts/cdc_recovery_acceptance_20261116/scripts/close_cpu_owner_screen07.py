#!/usr/bin/env python3
"""Reconcile completed screen07 evidence offline; never opens a device.

Outputs are exclusive-create. The flash record stays boot-unverified; a new
record closes that historical boundary using later full BUILD/READY evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def parsed_lines(raw: Path) -> list[tuple[str, dict]]:
    result = []
    for line in raw.read_bytes().split(b"\n")[:-1]:
        fields = line.rstrip(b"\r").decode("ascii", errors="replace").split(",")
        values = {}
        for item in fields[1:]:
            key, sep, value = item.partition("=")
            if not sep or not key or key in values:
                break
            values[key] = value
        else:
            result.append((fields[0], values))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=Path, required=True)
    parser.add_argument("--boot-output", type=Path, required=True)
    parser.add_argument("--verdict-output", type=Path, required=True)
    args = parser.parse_args()
    if args.boot_output.exists() or args.verdict_output.exists():
        raise FileExistsError("outputs must use new labels")
    stage = args.stage
    names = {
        "identity": "cpu-owner-01-validated-identity.json",
        "build": "cpu-owner-01-build-manifest.json",
        "tests": "cpu-owner-02-tests-manifest.json",
        "flash": "cpu-owner-01-flash-usj-result.json",
        "boot": "cpu-owner-01-boot.summary.json",
        "state": "cpu-owner-state-01.summary.json",
        "profile": "cpu-owner-resolved-config-01.json",
        "summary": "screen-07/screen-07-volt1.summary.json",
        "analysis": "screen-07/screen-07-analysis-01.json",
        "cpu": "screen-07/screen-07-cpu-owner-deltas-01.json",
    }
    docs = {key: read(stage/name) for key, name in names.items()}
    identity, build, tests, flash, boot, state, profile, summary, analysis, cpu = (
        docs[key] for key in names)
    assert identity["all_exit_zero"] and identity["host_entries"] == 65
    assert all(row["exit_code"] == 0 for row in build["results"])
    assert len(tests["results"]) == 65 and all(row["exit_code"] == 0 for row in tests["results"])
    assert identity["build_test_inputs_identical"] and identity["production_inputs"] == 185
    assert build["inputs_before"] == build["inputs_after"]
    assert tests["inputs_before"] == tests["inputs_after"] == build["inputs_after"]
    assert profile["sdkconfig_sha256"] == build["inputs_before"]["sdkconfig"]
    assert profile["sdkconfig_defaults_sha256"] == build["inputs_before"]["sdkconfig.defaults"]
    assert profile["dual_core"] and not profile["amazon_SMP"] and not profile["defaults_runtime_enabled"]
    assert flash["flash_verified"] and flash["build_manifest_sha256"] == digest(stage/names["build"])
    assert flash["head"] == identity["source_commit"]
    assert flash["app_sha256"] == identity["bin_sha256"] and flash["app_bytes"] == identity["app_bytes"]
    assert re.fullmatch(r"[0-9a-f]{64}", identity["elf_sha256"])
    raw_sources = {}
    for key, raw_name in (("boot", "cpu-owner-01-boot.rx.bin"), ("state", "cpu-owner-state-01.rx.bin")):
        raw_path = stage/raw_name
        assert digest(raw_path) == docs[key]["raw_sha256"]
        raw_sources[key] = {"path": str(raw_path), "sha256": digest(raw_path)}
        lines = parsed_lines(raw_path)
        replies = ([{"command": cmd, "fields": fields} for cmd, fields in boot["replies"].items()]
                   if key == "boot" else state["attempts"])
        for reply in replies:
            if reply["command"] in ("BUILD?", "CONFIG?", "READY?", "BOOT?", "SPISOURCE?"):
                assert (reply["command"][:-1], reply["fields"]) in lines
        observed = next(row["fields"] for row in replies if row["command"] == "BUILD?")
        assert observed["version"] == identity["version"] and observed["elfSha256"] == identity["elf_sha256"]
        assert observed["idf"] == "v5.5.5" and observed["target"] == "esp32s3"
    configs = [row["fields"] for row in state["attempts"] if row["command"] == "CONFIG?"]
    assert len(configs) >= 2 and all(row == configs[0] for row in configs)
    cfg = configs[0]
    assert (cfg["mode"], cfg["rows"], cfg["profile"], cfg["err"]) == ("CAP", "8", "CCCCCCCC", "0x0")
    assert boot["replies"]["CONFIG?"]["session"] == cfg["session"]
    assert boot["replies"]["CONFIG?"]["bootId"] == cfg["bootId"]
    ready = next(row["fields"] for row in state["attempts"] if row["command"] == "READY?")
    assert ready["ready"] == "1" and ready["err"] == "0x0" and ready["bootId"] == cfg["bootId"]
    meta = boot["transport_metadata"]
    assert (meta["vid"], meta["pid"], meta["interface_number"]) == (0x303a, 0x4042, 0)
    assert meta["usb_serial"].upper() == flash["expected_mac"].replace(":", "").upper()
    assert cpu["configuration_identity"]["session"] == cfg["session"]
    assert cpu["configuration_identity"]["bootId"] == cfg["bootId"]
    assert cpu["inputs"]["summary"]["sha256"] == digest(stage/names["summary"])
    inputs = {key: {"path": str(stage/name), "sha256": digest(stage/name)} for key, name in names.items()}
    common = {"generator": {"path": str(Path(__file__)), "sha256": digest(Path(__file__))},
              "hardware_actions": [], "original_recordings_modified": False, "inputs": inputs}
    verification = {
        **common, "schema_version": 1, "status": "RUNNING_BUILD_AND_READY_VERIFIED",
        "source_commit": identity["source_commit"], "version": identity["version"],
        "app_bytes": identity["app_bytes"], "app_bin_sha256": identity["bin_sha256"],
        "running_ELF_sha256": identity["elf_sha256"], "session": cfg["session"], "bootId": cfg["bootId"],
        "configuration": cfg, "ready": ready, "transport_metadata": meta,
        "initial_boot_stage": boot["replies"]["BOOT?"], "raw_sources": raw_sources,
        "profile": profile, "host_test_entries": 65, "target_build_exit": 0,
        "limitations": ["Bin and ELF hashes have distinct meanings; BUILD attests the full ELF descriptor hash.",
                        "The flash record remains historically boot-unverified; these later observations close that boundary.",
                        "Diagnostic profiling is enabled in this image, not in normal defaults.",
                        "No later unflashed workspace fix is covered by this image identity."],
    }
    capture = analysis["captures"][0]
    assert capture["label"] == "screen-07-volt1" and capture["window"]["requested_seconds_met"]
    assert summary["safe_cap_recovery"] == "CAP8_QUERY_VERIFIED"
    group = capture["configuration_groups"][0]
    adc = capture["otg_diagnostics"]["adc_crc"]
    verdict = {
        **common, "schema_version": 1, "status": "FAIL_BLE_THROUGHPUT",
        "meaning": "A complete diagnostic screen, not full performance, precision or reliability acceptance.",
        "running_image": {key: verification[key] for key in ("source_commit", "version", "app_bin_sha256", "running_ELF_sha256", "session", "bootId")},
        "profile_enabled": True, "window_seconds": summary["capture_seconds"],
        "capture_fps": group["rates"]["capture_fps"],
        "USB": {"FPS": group["rates"]["host_complete_receive_fps"], "counts": capture["capture_counts"]},
        "BLE": {"FPS": capture["ble"]["comparative_host_fps"], "counts": capture["ble"]["capture_frame_counts"],
                "received_fragment_validation": capture["ble"]["fragments_by_channel"],
                "common_frame_comparison": capture["ble"]["USB_comparison"]},
        "invalid_reasons_USB": capture["capture_invalid_reasons"],
        "ADC_CRC": adc, "BLE_BL50_independent_window": capture["otg_diagnostics"]["BLE_firmware"],
        "faults_and_resets": capture["faults_and_resets"],
        "CDC_fault_observations": capture["otg_diagnostics"]["CDC"]["faults"],
        "CDC_reset_observations": capture["otg_diagnostics"]["CDC"]["resets"],
        "WDT_evidence": capture["otg_diagnostics"]["WDT_evidence"],
        "memory": capture["otg_diagnostics"]["memory"],
        "CPU_owner": {"path": str(stage/names["cpu"]), "paired_roles": cpu["paired_roles"],
                      "unknown_roles": cpu["unknown_roles"], "aggregate_role_percent": None,
                      "SDK_CPU_time": "UNKNOWN", "blocked_time": "UNKNOWN"},
        "precision_acceptance": capture["precision_acceptance"],
        "current_image_600_seconds": "NOT_RUN", "current_image_32_configurations": "NOT_RUN",
        "post_capture_recovery": summary["safe_cap_recovery"],
        "limitations": ["ADC ratio covers first/last complete ADSHEALTH receipts, not exact scientific-window endpoints.",
                        "Host sequence gaps and BL50 losses have different windows/layers and cannot be summed or subtracted.",
                        "No observed WDT/MFAULT/reset text is not proof that no unrecorded event occurred.",
                        "Each CPU role has its own sample bracket; stale or unavailable is unknown, never zero.",
                        "SDK wall minus owner runtime cannot identify blocked time; SDK and idle tasks are unobserved.",
                        "Profiling changes system work/memory; adjacent firmware screens are not a controlled overhead A/B."],
    }
    for path, value in ((args.boot_output, verification), (args.verdict_output, verdict)):
        with path.open("x") as stream:
            json.dump(value, stream, indent=2); stream.write("\n")
    print(json.dumps({"boot": verification["status"], "screen": verdict["status"], "hardware_actions": []}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
