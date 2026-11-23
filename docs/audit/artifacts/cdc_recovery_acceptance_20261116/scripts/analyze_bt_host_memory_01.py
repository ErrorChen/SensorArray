"""Close the Bluedroid-host PSRAM preference run without inventing a benchmark."""
import datetime as dt
import hashlib
import json
from pathlib import Path

import analyze_wifi_owner_panic_01 as prior
import analyze_wifi_stack_control_01 as stack
import analyze_wifi_memory_control_01 as memory

ROOT, STAGE = prior.ROOT, prior.STAGE
read, record, require = prior.read, prior.record, prior.require


def main():
    before = prior.bind("bt-host-memory-state-01", "bt-host-memory-state-01", 0)
    after = prior.bind("bt-host-memory-post-01", "bt-host-memory-post-01", 0)
    previous = prior.bind("wifi-memory-state-01", "wifi-memory-state-01", 0)
    ap, _, command = stack.closed("wifi-control-05/wifi-control-05", "wifi-control-05", 1,
                                   "BACKEND_UNAVAILABLE_CONTROL_QUERY_OBSERVED")
    image = record(STAGE / "bt-host-memory-01-validated-identity.json")
    config = prior.one(before, "CONFIG?")
    require(config == prior.one(after, "CONFIG?") == ap["authority"]["CONFIG_opening"] ==
            ap["authority"]["CONFIG_closing"], "configuration_changed")
    require(all(prior.one(item, "BUILD?")["elfSha256"] == image["elf_sha256"] and
                prior.one(item, "READY?")["ready"] == "1" and not item["summary"]["faults"] and
                not item["summary"]["resets"] for item in (before, after)), "BUILD_READY_or_observed_fault")
    require(ap["expected"] == {"elfSha256": image["elf_sha256"], "session": config["session"], "bootId": config["bootId"]}, "AP_identity")
    require(before["command"]["finished_monotonic"] < command["started_monotonic"] <
            command["finished_monotonic"] < after["command"]["started_monotonic"], "command_order")
    terminal = ap["first_AP_terminal"]
    require(terminal == ap["latest_AP_terminal"] == ap["latest_AP_after_OFF"] and
            terminal["outcome"] == "quarantined" and int(terminal["err"], 16) == 0x101 and
            int(terminal["cleanupErr"], 16) == 0x103, "AP_failure_or_cache_changed")
    require(ap["off_restore"] == "SAME_BOOT_OUTPUT_GATE_QUERY_VERIFIED", "OFF_gate")
    guard = prior.one(after, "WIFIGUARD?")
    require(guard["phase"] == "driver_init" and guard["completionKnown"] == "1" and
            guard["active"] == "0" and guard["timeouts"] == "0", "SDK_return_scope")

    build = record(STAGE / image["build_manifest"])
    tests = record(STAGE / image["tests_manifest"])
    require(build["inputs_before"] == build["inputs_after"] == tests["inputs_before"] ==
            tests["inputs_after"] and not build["source_changed_during_run"] and
            not tests["source_changed_during_run"], "build_test_source_mismatch")
    require(len(build["inputs_before"]) == image["production_inputs"], "production_input_count")
    change = record(STAGE / "bt-host-psram-config-01/config-change.json")
    validation = record(STAGE / "bt-host-psram-config-01/validation.json")
    target = record(STAGE / "bt-host-psram-independent-target-01/new-target-manifest.json")
    require(target["unchanged"] and target["before"] == target["after"] and
            target["before"]["build/SensorArray.elf"] == image["elf_sha256"], "target_image_binding")
    for path, item in change["changes"].items():
        require(item["afterSha256"] == validation["sourceSha256"][path] ==
                build["inputs_before"][path] == target["before"][path], "config_source_hash_binding")
        require("+CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST=y" in item["diff"], "missing_preference_change")
    for item in validation["results"] + target["results"]:
        require(item["exitCode"] == 0, "source_target_check_failed")
        expected_hash = item.get("logSha256", item.get("sha256"))
        require(hashlib.sha256(read(ROOT / item["log"])).hexdigest() == expected_hash, "source_target_log_hash")
    collector = ROOT / "docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/check_wifi_owner_control.py"
    collector_source = {
        "path": str(collector.relative_to(ROOT)), "recorded_run_sha256": ap["source_sha256"],
        "current_observed_sha256": hashlib.sha256(read(collector)).hexdigest(),
        "archived_run_source_available": False,
        "binding": "RECORDED_SHA_ONLY_ARCHIVE_UNAVAILABLE",
        "limitation": "Root confirms a subsequent optional allocation-trace edit without a separate old-source archive. The current file is not attributed to AP05. Raw/events are independently validated; no reverse-synthesized source is presented as an original.",
        "prior_analysis_failure": "bt-host-memory-control-analysis-01-command.json",
    }
    # Bind the reused offline validation code, without touching any old report.
    for path in (Path(prior.__file__), Path(stack.__file__), Path(memory.__file__)):
        read(path)

    ble = record(STAGE / "bt-host-memory-coexist-01/bt-host-memory-ble-01.ble.summary.json")
    stack.command("bt-host-memory-ble-01", 0)
    receipt = memory.replay_terminals(ble, ap, command)
    before_mem, old_mem = prior.one(before, "MEM?"), prior.one(previous, "MEM?")
    report = {"status": "BT_HOST_PSRAM_PREFERENCE_AP_MEMORY_BLOCKER_REMAINS",
        "utc": dt.datetime.now(dt.timezone.utc).isoformat(), "image": image,
        "CONFIG": config, "READY": prior.one(after, "READY?"),
        "source_binding": {"build_test_production_inputs_equal": True,
            "production_inputs": len(build["inputs_before"]), "config": change,
            "scope": validation["constraints"], "target_ELF_hash_matches": True,
            "AP_collector_source": collector_source},
        "AP": {"capture_exit_code": command["exit_code"], "terminal": terminal,
            "accepted_to_terminal_us": int(terminal["completedUs"]) - int(terminal["acceptedUs"]),
            "phase_start_to_quarantine_us": int(guard["detectedUs"]) - int(guard["startUs"]),
            "guard": guard, "failed_terminal_unchanged_after_OFF": True,
            "OFF_scope": "software output gate only, physical off unknown",
            "observed_faults": ap["faults"], "observed_resets": ap["resets"]},
        "memory": {"unit": "bytes", "readout": "sequential", "before_BLE_AP": before_mem,
            "previous_image_before_BLE_AP": old_mem,
            "observed_cross_image_intFree_difference": int(before_mem["intFree"]) - int(old_mem["intFree"]),
            "AP_with_BLE_before": ap["memory_before"], "AP_with_BLE_after": ap["memory_after"],
            "later_postquery": prior.one(after, "MEM?"),
            "limitation": "Different query instants/images, not exclusive allocation accounting. PSRAM preference retains internal fallback and does not prove sufficient internal capacity for AP."},
        "owner_stack": prior.one(after, "WIFIQ?D"), "BLE_terminal_receipt": receipt,
        "inputs": [{"path": str(path.relative_to(ROOT)), "sha256": digest} for path, digest in sorted(prior.INPUTS.items())],
        "generator": {"path": str(Path(__file__).relative_to(ROOT)), "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        "hardware_actions_by_analysis": [], "original_inputs_modified": False,
        "formal_FPS": "NOT_EVALUATED", "UDP_PC_receipt": "NOT_RUN", "SDK_stall_injection": "NOT_RUN",
        "final600_32_50_SHORT": "NOT_RUN", "overall_acceptance": "BLOCKED_AP_MEMORY_FIX_FIRST"}
    for path, digest in prior.INPUTS.items():
        require(hashlib.sha256(path.read_bytes()).hexdigest() == digest, "input_changed_during_analysis")
    output = STAGE / "bt-host-memory-control-evidence-01.json"
    with output.open("x") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    print(json.dumps({"status": report["status"], "AP_us": report["AP"]["accepted_to_terminal_us"],
        "BLE_terminals": receipt["status"], "cross_image_intFree_difference": report["memory"]["observed_cross_image_intFree_difference"],
        "inputs": len(prior.INPUTS)}))


if __name__ == "__main__":
    main()
