"""Bind the closed AP02/PANIC recordings without creating hardware evidence."""
from __future__ import annotations

import copy
import datetime as dt
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[5]
STAGE = ROOT / "docs/audit/artifacts/cdc_recovery_acceptance_20261116"
sys.path.insert(0, str(ROOT / "tools"))
from text_protocol import parse_fields

INPUTS: dict[Path, str] = {}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read(path):
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    require(path not in INPUTS or INPUTS[path] == digest, "input_changed:" + str(path))
    INPUTS[path] = digest
    return data


def record(path):
    return json.loads(read(path))


def bind(stem, command_label, expected_exit):
    summary_path = STAGE / (stem + ".summary.json")
    raw_path, event_path = STAGE / (stem + ".rx.bin"), STAGE / (stem + ".events.jsonl")
    summary, raw = record(summary_path), read(raw_path)
    events = [json.loads(line) for line in read(event_path).splitlines()]
    command = record(STAGE / (command_label + "-command.json"))
    require(command["exit_code"] == expected_exit, "unexpected_capture_exit")
    require(hashlib.sha256(read(ROOT / command["log"])).hexdigest() == command["log_sha256"], "command_log_hash")
    require(summary["rx_bytes"] == len(raw), "raw_length")
    if "raw_sha256" in summary:
        require(summary["raw_sha256"] == hashlib.sha256(raw).hexdigest(), "raw_hash")
    for name, expected in summary.get("inputs", {}).items():
        require(hashlib.sha256(read(ROOT / name)).hexdigest() == expected, "summary_input_hash")
    end, active, replies, finished = 0, None, [], []
    for index, event in enumerate(events):
        kind = event["type"]
        if kind == "rx":
            require(event["offset"] == end and event["length"] > 0, "RX_gap_or_overlap")
            end += event["length"]
            require(end <= len(raw), "RX_beyond_raw")
        elif kind == "tx_begin":
            active = {"command": event["command"], "rx_floor": end, "begin_event": index, "complete": False}
        elif kind == "tx_complete":
            require(active and event["command"] == active["command"], "TX_completion_without_begin")
            require(event["accepted_bytes"] == len(("\n" + event["command"] + "\n").encode()), "TX_incomplete")
            active["complete"] = True
        elif kind == "reply":
            require(active and active["complete"] and event["command"] == active["command"], "reply_without_completed_query")
            needle = event["line"].encode("ascii")
            matches = []
            cursor = active["rx_floor"]
            for line in raw[cursor:end].splitlines(keepends=True):
                stop = cursor + len(line)
                if line.endswith(b"\n") and line.rstrip(b"\r\n") == needle and (cursor == 0 or raw[cursor - 1] == 10):
                    matches.append((cursor, stop))
                cursor = stop
            require(len(matches) == 1, "reply_not_bound_to_unique_new_raw_line:" + event["command"])
            replies.append({"command": event["command"], "fields": parse_fields(event["line"]),
                "raw_start": matches[0][0], "raw_end": matches[0][1],
                "tx_begin_event": active["begin_event"], "reply_event": index})
            active = None
        elif kind == "finished":
            finished.append(event)
    require(end == len(raw), "unaccounted_raw_tail")
    expected_replies = [(a["command"], a["fields"]) for a in summary.get("attempts", summary.get("queries", [])) if "fields" in a]
    actual_replies = [(a["command"], a["fields"]) for a in replies if a["command"] != "WIFI=AP"]
    require(actual_replies == expected_replies, "summary_reply_mismatch")
    for request in summary.get("requests", []):
        matched = [a for a in replies if a["command"] == request["command"]]
        require(len(matched) == 1 and matched[0]["fields"] == request["reply"], "request_reply_mismatch")
    if expected_exit == 1:
        require(summary["status"] == "ERROR" and summary["error"] == "serial_eof_or_disconnected", "unexpected_failure")
        require(len(finished) == 1 and finished[0]["status"] == "ERROR" and
                finished[0]["error"] == summary["error"], "failure_closure")
    else:
        require(summary["status"] == "READONLY_PROBE_RECORDED" and summary["readonly"] is True, "probe_not_closed_readonly")
    return {"summary": summary, "replies": replies, "command": command,
            "binding": {"summary": str(summary_path.relative_to(ROOT)), "raw_bytes": len(raw),
                "raw_sha256": hashlib.sha256(raw).hexdigest(), "rx_events_contiguous": True,
                "reply_count": len(replies), "command_exit_code": expected_exit,
                "finished_event_count": len(finished),
                "closure": "final_summary_hash_and_returned_command; readonly_probe_has_no_finished_event" if not finished else "finished_event_and_returned_command"}}


def one(capture, command):
    found = [item["fields"] for item in capture["replies"] if item["command"] == command]
    require(found and all(item == found[0] for item in found), "missing_or_changing_query:" + command)
    return found[0]


def check_pages(pages, current, origin, elf):
    meta = pages[0]
    for number, page in enumerate(pages):
        require(page["v"] == "1" and page["page"] == str(number) and page["known"] == "1", "PANIC_page_shape")
        require(page["session"].upper() == current["session"].upper() and page["boot"] == current["bootId"], "PANIC_current_identity")
        require(page["record"] == meta["record"] and re.fullmatch(r"[0-9A-F]{8}", page["record"]), "PANIC_record_identity")
    require(meta["originKnown"] == "1" and meta["originSession"].upper() == origin["session"].upper()
            and meta["originBoot"] == origin["bootId"] and meta["elf"].lower() == elf.lower(), "PANIC_origin_identity")
    require(meta["detailKnown"] == "1" and meta["truncated"] == "0", "detail_not_complete")
    total = int(meta["detailLen"])
    require(0 <= total <= 192, "detail_total_bound")
    pieces = []
    for number, page in enumerate(pages[1:], 1):
        offset, length = (number - 1) * 96, min(96, max(0, total - (number - 1) * 96))
        require(page["detailKnown"] == "1" and page["truncated"] == "0" and page["encoding"] == "hex", "detail_page_encoding")
        require(int(page["offset"]) == offset and int(page["length"]) == length and int(page["total"]) == total, "detail_page_length")
        require(re.fullmatch(r"[0-9A-F]*", page["text"]) and len(page["text"]) == 2 * length, "detail_hex_length")
        pieces.append(bytes.fromhex(page["text"]))
    text = b"".join(pieces)
    require(len(text) == total, "detail_join_length")
    return text


def main():
    before = bind("panic-diagnostic-state-01", "panic-diagnostic-state-01", 0)
    ap = bind("wifi-control-02/wifi-control-02", "wifi-control-02", 1)
    after = bind("wifi-panic-state-01", "wifi-panic-state-01", 0)
    debug = bind("wifi-panic-debug-01/wifi-panic-debug-01.debug", "wifi-panic-debug-01", 1)
    old, current = one(before, "CONFIG?"), one(after, "CONFIG?")
    elf = one(before, "BUILD?")["elfSha256"]
    require(one(ap, "CONFIG?") == old and one(ap, "BUILD?")["elfSha256"] == elf and
            one(after, "BUILD?")["elfSha256"] == elf, "preflight_AP_image_or_config_mismatch")
    require(ap["summary"]["expected"] == {"elfSha256": elf, "session": old["session"], "bootId": old["bootId"]}, "AP_expected_identity")
    require(one(before, "READY?")["ready"] == "1" and one(before, "PANIC?")["known"] == "0", "preflight_state")
    require(old["session"] != current["session"] and one(after, "BOOT?")["reset"] == "panic", "new_panic_boot")
    require(before["command"]["finished_monotonic"] < ap["command"]["started_monotonic"] <
            ap["command"]["finished_monotonic"] < after["command"]["started_monotonic"], "command_order")
    pages = [one(after, command) for command in ("PANIC?", "PANIC?1", "PANIC?2")]
    detail = check_pages(pages, current, old, elf)
    require(detail == b"***ERROR*** A stack overflow in task wifiOwner has been detected.", "unexpected_decoded_detail")
    require(pages[0]["abort"] == "1" and pages[0]["infoKnown"] == "1", "abort_evidence")
    require(debug["summary"]["commands_written"] == 0 and debug["summary"]["raw_sha256"] == debug["binding"]["raw_sha256"], "debug_not_passive_or_hash")
    require(ap["summary"]["requests"][0]["terminal"] is None and ap["summary"]["off_restore"] == "FAILED_OR_UNKNOWN", "AP_failure_preservation")

    negative_checks = []
    for name, change in (
        ("page_record_mismatch", lambda p: p[2].update(record="00000000")),
        ("origin_boot_mismatch", lambda p: p[0].update(originBoot="99")),
        ("current_session_mismatch", lambda p: p[1].update(session="0000000000000000")),
        ("detail_length_mismatch", lambda p: p[1].update(length="64")),
        ("nonhex_detail", lambda p: p[1].update(text="ZZ")),
    ):
        mutated = copy.deepcopy(pages)
        change(mutated)
        try:
            check_pages(mutated, current, old, elf)
        except ValueError as error:
            negative_checks.append({"case": name, "synthetic_only": True, "rejected": str(error)})
        else:
            raise AssertionError("synthetic_invalid_page_accepted:" + name)

    rom = record(STAGE / "panic-diagnostic-rom-01.summary.json")
    flash = record(STAGE / "panic-diagnostic-01-flash-usj-result.json")
    image = record(STAGE / "panic-diagnostic-01-validated-identity.json")
    require(rom["action_attempts"] == 1 and rom["ROM_execution"] == "UNVERIFIED" and
            rom["local_publication"]["target"] == "rom_download", "USBBOOT_original_scope")
    require(flash["flash_verified"] and flash["watchdog_call_returned"] and flash["expected_mac"] == "80:b5:4e:cd:d9:ac", "flash_result")
    require(flash["app_sha256"] == image["bin_sha256"] and elf == image["elf_sha256"], "flash_image_identity")
    for command in flash["commands"]:
        log = read(ROOT / command["log"])
        require(command["exit_code"] == 0 and hashlib.sha256(log).hexdigest() == command["log_sha256"], "flash_log_hash")
        if command["stage"] == "identity":
            require(b"MAC: 80:b5:4e:cd:d9:ac" in log and b"Chip is ESP32-S3" in log and
                    b"Uploading stub..." in log and b"Stub running..." in log, "ROM_protocol_identity")
    report = {
        "status": "PANIC_RECORD_CONFIRMED_WIFI_OWNER_STACK_OVERFLOW",
        "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "diagnosis": {"observed_failure": "wifiOwner stack overflow", "decoded_abort_bytes": len(detail),
            "decoded_abort_text": detail.decode("ascii"), "truncated": False,
            "nested_call_trigger": "UNKNOWN_UNDER_INVESTIGATION", "repair_validated": False},
        "origin": {"session": old["session"], "boot": old["bootId"], "elf_sha256": elf},
        "current": {"session": current["session"], "boot": current["bootId"],
            "CONFIG": current, "BOOT": one(after, "BOOT?"), "READY": one(after, "READY?"),
            "readiness_limit": "This probe reports READY0 boot_sweep; CAP8 configuration is not READY1 evidence."},
        "record": {"id": pages[0]["record"], "pages": pages,
            "page_identity_consistent": True, "origin_matches_preflight_and_AP": True,
            "firmware_integrity_reported_known": True, "host_recomputed_RTC_crc32": False,
            "host_binding": "Complete original raw reply bytes + contiguous RX + summary/hash/returned-command evidence",
            "raw_registers_not_backtrace": True, "invented_backtrace": False,
            "metadata_repeat_after_pages": False, "limits": "Three observed pages and both CONFIGs agree; no extra page0 reread is invented."},
        "captures": {"preflight": before["binding"], "AP": ap["binding"], "post_panic": after["binding"], "debug": debug["binding"]},
        "reply_bindings": {"preflight": before["replies"], "AP": ap["replies"], "post_panic": after["replies"]},
        "debug_result": {"status": debug["summary"]["status"], "error": debug["summary"]["error"],
            "raw_bytes": debug["summary"]["rx_bytes"], "interpretation": "Closed recorded EOF failure during the attempt, not missing raw or health PASS."},
        "handoff_chain": {"original_ROM_request_status": rom["status"], "original_ROM_execution": rom["ROM_execution"],
            "unchanged_USJ_epoch_does_not_prove_failure": True,
            "subsequent_ROM_protocol_and_MAC_verified": True, "app_hash_written_verified": True,
            "subsequent_new_app_BUILD_CAP8_READY1_observed": True,
            "tested_scope": "Explicit USBBOOT from USJ recovery then app-only flash/watchdog exit; no physical BOOT operation.",
            "formal_OTG_manual_RESTART": "NOT_RUN"},
        "synthetic_negative_checks": negative_checks,
        "inputs": [{"path": str(path.relative_to(ROOT)), "sha256": digest} for path, digest in sorted(INPUTS.items())],
        "generator": {"path": str(Path(__file__).relative_to(ROOT)), "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        "analysis_hardware_actions": [], "original_recordings_modified": False,
        "overall_acceptance": "BLOCKED_FIX_FIRST", "final600_32_50_SHORT": "NOT_RUN",
    }
    for path, digest in INPUTS.items():
        require(hashlib.sha256(path.read_bytes()).hexdigest() == digest, "input_changed_before_output")
    output = STAGE / "wifi-owner-stack-overflow-evidence-01.json"
    with output.open("x") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({"status": report["status"], "detail_bytes": len(detail), "record": pages[0]["record"],
                      "bound_inputs": len(INPUTS), "negative_checks": len(negative_checks), "output": str(output.relative_to(ROOT))}))


if __name__ == "__main__":
    main()
