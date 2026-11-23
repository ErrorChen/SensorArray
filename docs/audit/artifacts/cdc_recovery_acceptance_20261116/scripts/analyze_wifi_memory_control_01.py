"""Offline binding of AP04 memory failure and received BLE WTERM records.

No scientific capture window or FPS is synthesized from the receive-only run.
"""
import base64
from collections import Counter
from dataclasses import asdict
import datetime as dt
import hashlib
import json
from pathlib import Path

import analyze_wifi_owner_panic_01 as prior
import analyze_wifi_stack_control_01 as stack
from text_protocol import FragmentReassembler

ROOT, STAGE = prior.ROOT, prior.STAGE
read, record, require = prior.read, prior.record, prior.require


def replay_terminals(summary, ap, command):
    raw = read(ROOT / summary["journal_file"])
    require(len(raw) == summary["journal_bytes"] and
            hashlib.sha256(raw).hexdigest() == summary["journal_sha256"], "journal_hash")
    ready = record(ROOT / summary["ready_file"])
    require(summary["status"] == "RECORDED" and summary["error"] is None and
            not summary["cleanup_errors"] and not summary["unexpected_disconnect"] and
            not summary["reconnect_attempted"], "BLE_not_closed")
    require(summary["application_commands_sent"] == 0 and not summary["serial_opened"] and
            not summary["mode_or_rate_settings_written"], "observer_mutated")
    require(ready["monotonic_ns"] == summary["ready_monotonic_ns"] and
            set(ready["all_channels_subscribed"]) == set(summary["subscribed_channels"]) == set("CDL"), "ready_channels")
    require(summary["ready_monotonic_ns"] < round(command["started_monotonic"] * 1e9) <
            round(command["finished_monotonic"] * 1e9) < summary["recording_end_monotonic_ns"], "AP_outside_subscription")
    parser_path = ROOT / "tools/text_protocol.py"
    parser_sha = hashlib.sha256(read(parser_path)).hexdigest()
    reassembler = FragmentReassembler()
    buffers = {ch: bytearray() for ch in "CDL"}
    fragments = {}
    counts, sizes, kinds = Counter(), Counter(), Counter()
    terminals, events = [], []
    last_ns, offset, maximum = -1, 0, 0
    for number, line in enumerate(raw.splitlines(keepends=True), 1):
        require(line.endswith(b"\n"), "incomplete_journal_line")
        event = json.loads(line)
        ns, kind = event["monotonic_ns"], event["type"]
        require(event["schema_version"] == 1 and isinstance(ns, int) and ns >= last_ns,
                "journal_schema_or_order")
        last_ns = ns
        kinds[kind] += 1
        if kind != "notification":
            events.append(event)
            offset += len(line)
            continue
        channel = event["channel"]
        payload = base64.b64decode(event["payload_base64"], validate=True)
        require(channel in "CDL" and len(payload) == event["length"], "notification_shape")
        counts[channel] += 1
        sizes[channel] += len(payload)
        maximum = max(maximum, len(payload))
        location = {"journal_line": number, "journal_byte_offset": offset,
                    "journal_record_bytes": len(line), "host_monotonic_ns": ns,
                    "ATT_channel": channel, "ATT_bytes": len(payload),
                    "ATT_sha256": hashlib.sha256(payload).hexdigest()}
        key = None
        if payload.startswith(b"G,"):
            parts = payload.split(b"\n", 1)[0].decode("ascii").split(",")
            key = (parts[1], int(parts[2]))
            fragments.setdefault(key, []).append(location)
        for logical, message in reassembler.feed(channel, payload):
            locations = fragments.pop(key, []) if key is not None else [location]
            buffers[logical].extend(message)
            while b"\n" in buffers[logical]:
                value, _, tail = buffers[logical].partition(b"\n")
                buffers[logical] = bytearray(tail)
                if not value.startswith(b"WTERM,"):
                    continue
                text = value.rstrip(b"\r").decode("ascii")
                fields = prior.parse_fields(text)
                # The observed terminals are each one complete raw ATT line.
                # Do not pretend a raw WTERM carries an independent G CRC.
                require(key is None and message == value + b"\n", "unexpected_terminal_envelope")
                expected = [r["terminal"] for r in ap["requests"] if r["terminal"]["id"] == fields.get("id")]
                require(len(expected) == 1 and set(fields) == set(expected[0]), "terminal_shape_or_ID")
                counters = {"terminalAttempts", "terminalQueued", "terminalPending"}
                require({k: v for k, v in fields.items() if k not in counters} ==
                        {k: v for k, v in expected[0].items() if k not in counters}, "terminal_identity_or_result")
                require(round(command["started_monotonic"] * 1e9) <= ns <=
                        round(command["finished_monotonic"] * 1e9), "terminal_outside_AP_command_bracket")
                terminals.append({"logical_channel": logical, "fields": fields, "text": text,
                    "line_bytes_including_LF": len(message), "source_records": locations,
                    "framing": "raw_ATT_complete_ASCII_line_no_independent_G_or_science_CRC",
                    "immutable_fields_match_USB_terminal_query": True,
                    "later_USB_publication_counters": {k: expected[0][k] for k in sorted(counters)}})
        offset += len(line)
    require(offset == len(raw), "journal_tail")
    require(dict(counts) == summary["notification_counts"] and
            dict(sizes) == summary["notification_payload_bytes"] and
            maximum == summary["maximum_observed_notification_bytes"], "journal_notification_summary")
    metadata = [x for x in events if x["type"] == "metadata"]
    ready_events = [x for x in events if x["type"] == "subscriptions_ready"]
    ends = [x for x in events if x["type"] == "recording_end"]
    disconnects = [x for x in events if x["type"] == "disconnected"]
    require(len(metadata) == len(ready_events) == len(ends) == len(disconnects) == 1, "journal_lifecycle_count")
    require(metadata[0]["collector"] == "capture_benchmark_ble" and
            metadata[0]["application_commands_sent"] == 0 and not metadata[0]["serial_opened"], "metadata_observer")
    require(ready_events[0]["monotonic_ns"] == ready["monotonic_ns"] and
            set(ready_events[0]["all_channels_subscribed"]) == set("CDL") and
            ends[0]["monotonic_ns"] == summary["recording_end_monotonic_ns"] and
            ends[0]["error"] is None and ends[0]["stop_reason"] == summary["stop_reason"] and
            disconnects[0]["monotonic_ns"] == summary["disconnected_monotonic_ns"] and
            not disconnects[0]["unexpected"] and not disconnects[0]["reconnect_attempted"], "journal_closure")
    observed = {x["fields"]["id"] for x in terminals}
    return {"status": "BOTH_REQUEST_TERMINALS_OBSERVED" if observed == {"1", "2"} else "TERMINAL_RECEIPT_INCOMPLETE",
        "parser": {"path": str(parser_path.relative_to(ROOT)), "sha256": parser_sha},
        "journal_sha256": summary["journal_sha256"], "notification_counts": dict(counts),
        "records": terminals, "C_notifications": counts["C"],
        "fragment_stats": {ch: asdict(stats) for ch, stats in reassembler.stats.items()},
        "pending_fragment_messages": len(reassembler._messages),
        "trailing_bytes": {ch: len(buf) for ch, buf in buffers.items()},
        "recording_interval_ns": [summary["ready_monotonic_ns"], summary["recording_end_monotonic_ns"]],
        "formal_scientific_capture_window": None, "FPS": "NOT_EVALUATED",
        "scope": "These exact two received terminal lines only; no delivery guarantee or science-frame acceptance. Publication counters are captured before local submission and can differ from later queries."}


def main():
    before = prior.bind("wifi-memory-state-01", "wifi-memory-state-01", 0)
    after = prior.bind("wifi-memory-post-01", "wifi-memory-post-01", 0)
    old = prior.bind("wifi-stack-state-01", "wifi-stack-state-01", 0)
    ap, _, ap_command = stack.closed("wifi-control-04/wifi-control-04", "wifi-control-04", 1,
                                     "BACKEND_UNAVAILABLE_CONTROL_QUERY_OBSERVED")
    image = record(STAGE / "wifi-memory-01-validated-identity.json")
    config = prior.one(before, "CONFIG?")
    require(config == prior.one(after, "CONFIG?") == ap["authority"]["CONFIG_opening"] ==
            ap["authority"]["CONFIG_closing"], "configuration_changed")
    require(all(prior.one(c, "BUILD?")["elfSha256"] == image["elf_sha256"] and
                prior.one(c, "READY?")["ready"] == "1" and not c["summary"]["faults"] and
                not c["summary"]["resets"] for c in (before, after)), "image_or_observed_health")
    require(ap["expected"] == {"elfSha256": image["elf_sha256"], "session": config["session"], "bootId": config["bootId"]}, "AP_identity")
    require(before["command"]["finished_monotonic"] < ap_command["started_monotonic"] <
            ap_command["finished_monotonic"] < after["command"]["started_monotonic"], "capture_order")
    terminal = ap["first_AP_terminal"]
    require(terminal == ap["latest_AP_terminal"] == ap["latest_AP_after_OFF"] and
            terminal["outcome"] == "quarantined" and int(terminal["err"], 16) == 0x101 and
            int(terminal["cleanupErr"], 16) == 0x103, "AP_failure_or_cache_changed")
    require(ap["off_restore"] == "SAME_BOOT_OUTPUT_GATE_QUERY_VERIFIED", "OFF_not_verified")
    ble = record(STAGE / "wifi-memory-coexist-01/wifi-memory-ble-01.ble.summary.json")
    stack.command("wifi-memory-ble-01", 0)
    receipts = replay_terminals(ble, ap, ap_command)
    mem, old_mem = prior.one(before, "MEM?"), prior.one(old, "MEM?")
    guard = prior.one(after, "WIFIGUARD?")
    require(guard["phase"] == "driver_init" and guard["completionKnown"] == "1" and
            guard["active"] == "0" and guard["timeouts"] == "0", "not_returned_init_failure")
    require(all(mem[k] == v for k, v in {"poolReady": "1", "poolErr": "0x0", "poolInt": "1157", "poolExt": "16135"}.items()), "pool_observation")
    result = {"status": "AP_MEMORY_BLOCKER_WITH_OBSERVED_BLE_TERMINALS", "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "image": image, "CONFIG": config, "READY": prior.one(after, "READY?"),
        "AP": {"capture_exit_code": ap_command["exit_code"], "terminal": terminal,
            "accepted_to_terminal_us": int(terminal["completedUs"]) - int(terminal["acceptedUs"]),
            "phase_start_to_quarantine_us": int(guard["detectedUs"]) - int(guard["startUs"]),
            "guard": guard, "failed_terminal_unchanged_after_OFF": True,
            "OFF_scope": "software output gate, physical off unknown",
            "observed_faults": ap["faults"], "observed_resets": ap["resets"]},
        "memory": {"unit": "bytes", "readout": "sequential", "before_BLE_AP": mem,
            "AP_with_BLE_before": ap["memory_before"], "AP_with_BLE_after": ap["memory_after"],
            "postquery": prior.one(after, "MEM?"), "previous_image_before_BLE_AP": old_mem,
            "observed_cross_image_intFree_difference": int(mem["intFree"]) - int(old_mem["intFree"]),
            "limitation": "Different query instants and images; descriptive, not exclusive allocation or leak accounting, and additional free bytes do not establish AP readiness."},
        "owner_stack": prior.one(after, "WIFIQ?D"), "BLE_terminal_receipt": receipts,
        "inputs": [{"path": str(path.relative_to(ROOT)), "sha256": digest} for path, digest in sorted(prior.INPUTS.items())],
        "generator": {"path": str(Path(__file__).relative_to(ROOT)), "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        "hardware_actions_by_analysis": [], "original_inputs_modified": False,
        "UDP_PC_receipt": "NOT_RUN", "SDK_stall_injection": "NOT_RUN",
        "final600_32_50_SHORT": "NOT_RUN", "overall_acceptance": "BLOCKED_AP_MEMORY_FIX_FIRST"}
    for path, digest in prior.INPUTS.items():
        require(hashlib.sha256(path.read_bytes()).hexdigest() == digest, "input_changed")
    output = STAGE / "wifi-memory-control-evidence-01.json"
    with output.open("x") as handle:
        json.dump(result, handle, indent=2)
        handle.write("\n")
    print(json.dumps({"status": result["status"], "AP_us": result["AP"]["accepted_to_terminal_us"],
        "BLE_terminals": receipts["status"], "cross_image_intFree_difference": result["memory"]["observed_cross_image_intFree_difference"],
        "inputs": len(prior.INPUTS)}))


if __name__ == "__main__":
    main()
