"""Close stack-fix/AP-memory-failure and subsequent manual-restart evidence.

Offline only. Existing failed capture statuses and first query errors remain.
"""
import datetime as dt
import hashlib
import json
from pathlib import Path

import analyze_wifi_owner_panic_01 as prior

ROOT, STAGE = prior.ROOT, prior.STAGE
read, record, require, parse_fields = prior.read, prior.record, prior.require, prior.parse_fields


def command(label, exit_code):
    item = record(STAGE / (label + "-command.json"))
    require(item["exit_code"] == exit_code and "finished_monotonic" in item, "command_not_closed:" + label)
    require(hashlib.sha256(read(ROOT / item["log"])).hexdigest() == item["log_sha256"], "command_log_hash")
    return item


def stream(raw, events):
    end, active, replies = 0, None, []
    for item in events:
        if item["type"] == "rx":
            require(item["offset"] == end and item["length"] > 0, "RX_discontinuity")
            end += item["length"]
            require(end <= len(raw), "RX_overrun")
        elif item["type"] == "tx_begin":
            active = [item["command"], end, False]
        elif item["type"] == "tx_complete":
            require(active and active[0] == item["command"], "TX_without_begin")
            require(item["accepted_bytes"] == len(("\n" + active[0] + "\n").encode()), "TX_incomplete")
            active[2] = True
        elif item["type"] == "reply":
            require(active and active[2] and active[0] == item["command"], "reply_without_TX")
            cursor, hits = active[1], []
            for line in raw[cursor:end].splitlines(keepends=True):
                if line.endswith(b"\n") and line.rstrip(b"\r\n") == item["line"].encode("ascii") and (not cursor or raw[cursor - 1] == 10):
                    hits.append([cursor, cursor + len(line)])
                cursor += len(line)
            require(len(hits) == 1, "reply_not_unique_new_complete_raw")
            replies.append({"command": item["command"], "fields": parse_fields(item["line"]), "raw_range": hits[0]})
            active = None
    require(end == len(raw), "unaccounted_raw")
    return replies


def closed(stem, label, exit_code, status):
    summary = record(STAGE / (stem + ".summary.json"))
    raw = read(STAGE / (stem + ".rx.bin"))
    events = [json.loads(line) for line in read(STAGE / (stem + ".events.jsonl")).splitlines()]
    cmd = command(label, exit_code)
    require(summary["status"] == status and summary["rx_bytes"] == len(raw), "summary_status_or_length")
    if "raw_sha256" in summary:
        require(hashlib.sha256(raw).hexdigest() == summary["raw_sha256"], "raw_hash")
    for name, expected in summary.get("inputs", {}).items():
        require(hashlib.sha256(read(ROOT / name)).hexdigest() == expected, "summary_input_hash")
    replies = stream(raw, events)
    finishes = [item for item in events if item["type"] == "finished"]
    require(len(finishes) == 1 and finishes[0]["status"] == status and finishes[0].get("error") is None, "closed_finished")
    require(not summary["faults"] and not summary.get("resets", summary.get("reset_observations", [])), "observed_fault_or_reset")
    requests = {item["command"] for item in summary.get("requests", [])}
    actual = [(item["command"], item["fields"]) for item in replies if item["command"] not in requests]
    require(actual == [(item["command"], item["fields"]) for item in summary.get("queries", [])], "query_fields_not_raw_bound")
    for item in summary.get("requests", []):
        require([reply["fields"] for reply in replies if reply["command"] == item["command"]] == [item["reply"]], "request_ACK_not_raw_bound")
        require(any(reply["fields"] == item["terminal"] for reply in replies), "terminal_not_raw_bound")
    return summary, replies, cmd


def main():
    before = prior.bind("wifi-stack-state-01", "wifi-stack-state-01", 0)
    after = prior.bind("wifi-stack-post-01", "wifi-stack-post-01", 0)
    ap, ap_replies, ap_command = closed("wifi-control-03/wifi-control-03", "wifi-control-03", 1, "BACKEND_UNAVAILABLE_CONTROL_QUERY_OBSERVED")
    debug, _, debug_command = closed("wifi-stack-debug-01/wifi-stack-debug-01.debug", "wifi-stack-debug-01", 0, "OBSERVED")
    image = record(STAGE / "wifi-stack-01-validated-identity.json")
    origin, config_after = prior.one(before, "CONFIG?"), prior.one(after, "CONFIG?")
    require(origin == config_after == ap["authority"]["CONFIG_opening"] == ap["authority"]["CONFIG_closing"], "configuration_changed")
    require(ap["expected"] == {"elfSha256": image["elf_sha256"], "session": origin["session"], "bootId": origin["bootId"]}, "AP_image_identity")
    require(all(prior.one(capture, "BUILD?")["elfSha256"] == image["elf_sha256"] and prior.one(capture, "READY?")["ready"] == "1" for capture in (before, after)), "BUILD_or_READY")
    terminal = ap["first_AP_terminal"]
    require(terminal == ap["latest_AP_terminal"] == ap["latest_AP_after_OFF"], "failed_terminal_changed_by_OFF")
    require(terminal["outcome"] == "quarantined" and int(terminal["err"], 16) == 0x101 and int(terminal["cleanupErr"], 16) == 0x103, "expected_memory_failure")
    require(ap["requests"][1]["terminal"]["outcome"] == "applied" and ap["off_restore"] == "SAME_BOOT_OUTPUT_GATE_QUERY_VERIFIED", "OFF_gate_not_verified")
    guard = prior.one(after, "WIFIGUARD?")
    require(guard["phase"] == "driver_init" and guard["quarantined"] == "1" and guard["completionKnown"] == "1" and guard["active"] == "0" and guard["timeouts"] == "0", "guard_scope")
    stacks = [prior.one(after, "WIFIQ?" + suffix) for suffix in "DLC"]
    require(all(item["stackKnown"] == "1" and item["stackMinBytes"] == "2884" and item["stackScope"] == "owner_after_job" for item in stacks), "stack_observation")
    require("6144-byte stack" in image["change_scope"], "stack_configuration_not_bound_to_image")

    restart = record(STAGE / "wifi-quarantine-restart-01.summary.json")
    restart_command = command("wifi-quarantine-restart-01", 0)
    restart_events = [json.loads(line) for line in read(ROOT / restart["events_file"]).splitlines()]
    require(restart["status"] == "APPLICATION_RESTART_OBSERVED" and restart["requested_action"] == "RESTART" and restart["action_attempts"] == 1, "restart_not_closed_once")
    require(not any(restart[key] for key in ("automatic_action_retry", "esptool_executed", "flash_written", "system_USB_reset_requested")), "unexpected_recovery_action")
    require(restart_events[-1]["type"] == "finished" and restart_events[-1]["status"] == restart["status"], "restart_final_event")
    epoch_bindings = []
    for epoch in restart["epochs"]:
        raw = read(ROOT / epoch["raw_file"])
        require(len(raw) == epoch["rx_bytes"] and hashlib.sha256(raw).hexdigest() == epoch["raw_sha256"] and epoch["reader_release_known"], "restart_epoch_not_closed")
        replies = stream(raw, [item for item in restart_events if item.get("epoch") == epoch["epoch"]])
        for name, fields in epoch["replies"].items():
            require(any(item["command"] == name and item["fields"] == fields for item in replies), "restart_reply_binding")
        for publication in epoch["publications"]:
            require(any(line.startswith(publication["tag"] + ",") and parse_fields(line) == publication["fields"] for line in raw.decode("ascii").splitlines()), "publication_binding")
        epoch_bindings.append({"epoch": epoch["epoch"], "bytes": len(raw), "raw_sha256": epoch["raw_sha256"], "replies": len(replies)})
    require(restart["before"]["CONFIG?"] == origin and restart["after"]["CONFIG?"]["session"] != origin["session"], "restart_old_new_identity")
    require(restart["before"]["BUILD?"]["elfSha256"] == restart["after"]["BUILD?"]["elfSha256"] == image["elf_sha256"], "restart_ELF")
    require(restart["before"]["USB?"]["backend"] == restart["after"]["USB?"]["backend"] == "otg_cdc_v1", "restart_backend")
    require(ap_command["finished_monotonic"] < after["command"]["started_monotonic"] < after["command"]["finished_monotonic"] < restart_command["started_monotonic"], "evidence_order")

    report = {
        "status": "TESTED_STACK_FAILURE_PATH_RECOVERED_AP_MEMORY_BLOCKER_REMAINS",
        "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "image": {key: image[key] for key in ("elf_sha256", "bin_sha256", "app_bytes", "host_entries", "production_inputs")},
        "session": origin["session"], "boot": origin["bootId"],
        "AP": {"capture_status": ap["status"], "capture_exit_code": ap_command["exit_code"],
            "terminal": terminal, "accepted_to_terminal_us": int(terminal["completedUs"]) - int(terminal["acceptedUs"]),
            "guard_phase_start_to_quarantine_us": int(guard["detectedUs"]) - int(guard["startUs"]),
            "scope": "Known DRIVER_INIT NO_MEM return; cleanupErr103 retains uncertain SDK cleanup, not a timeout/cancel.",
            "OFF_scope": "software output gate only", "failed_AP_terminal_unchanged_after_OFF": True,
            "UDP_PC_receipt": ap["UDP_PC_receipt"], "SDK_stall_injection": ap["SDK_stall_injection"]},
        "stack": {"configured_bytes": 6144, "minimum_remaining_bytes": 2884, "sample_scope": "owner_after_job",
            "conclusion": "Observed tested DRIVER_INIT failure path returns without the earlier stack-overflow reset; successful full AP path not qualified."},
        "memory": {"unit": "bytes", "readout": "sequential", "before_AP": ap["memory_before"],
            "after_AP": ap["memory_after"], "later_postquery": prior.one(after, "MEM?"),
            "limitation": "Different query instants, not exclusive allocation/leak accounting; later intMin8192 is not the earlier8208 sample."},
        "same_boot_post": {"CONFIG": config_after, "BOOT": prior.one(after, "BOOT?"), "READY": prior.one(after, "READY?"), "guard": guard},
        "debug": {"status": debug["status"], "exit_code": debug_command["exit_code"], "raw_bytes": debug["rx_bytes"],
            "observed_faults": debug["faults"], "observed_resets": debug["reset_observations"], "not_a_global_health_guarantee": True},
        "manual_restart": {"status": restart["status"], "command_exit_code": 0,
            "old_CONFIG": restart["before"]["CONFIG?"], "new_CONFIG": restart["after"]["CONFIG?"],
            "action_attempts": 1, "action_to_enumeration_seconds": restart["action_to_enumeration_seconds"],
            "original_post_restart_error_retained": restart["post_restart_error"],
            "READY_query_collected_by_this_run": "READY?" in restart["after"],
            "epochs": epoch_bindings, "causal_scope": restart["causal_scope"],
            "limitation": "Formal OTG RESTART observed from a quarantined, already-returned SDK failure; no claim of canceling an in-flight blocked call or arbitrary recovery."},
        "inputs": [{"path": str(path.relative_to(ROOT)), "sha256": digest} for path, digest in sorted(prior.INPUTS.items())],
        "generator": {"path": str(Path(__file__).relative_to(ROOT)), "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        "hardware_actions_by_analysis": [], "original_inputs_modified": False,
        "final600_32_50_SHORT": "NOT_RUN", "overall_acceptance": "BLOCKED_AP_MEMORY_FIX_FIRST",
    }
    for path, digest in prior.INPUTS.items():
        require(hashlib.sha256(path.read_bytes()).hexdigest() == digest, "input_changed_during_analysis")
    with (STAGE / "wifi-stack-control-evidence-01.json").open("x") as handle:
        json.dump(report, handle, indent=2)
        handle.write("\n")
    print(json.dumps({"status": report["status"], "AP_us": report["AP"]["accepted_to_terminal_us"],
                      "inputs": len(prior.INPUTS), "manual_restart": restart["status"]}))


if __name__ == "__main__":
    main()
