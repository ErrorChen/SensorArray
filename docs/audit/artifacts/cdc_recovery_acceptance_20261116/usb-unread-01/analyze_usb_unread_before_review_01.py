#!/usr/bin/env python3
"""Analyze a closed unread-client hold, never inventing a CDC capture summary.

The existing BLE journal/reassembly/scientific parser consumes only an explicit
in-memory window from the recorded hold. Formal USB receipt remains unobserved.
No hardware access, repair of original bytes, or acceptance override occurs.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
import analyze_firmware_benchmark as benchmark
import sensorarray_bulk_diagnostics as diagnostic
from text_protocol import parse_fields

IDENTITY = ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid")
CONFIG = IDENTITY + ("boundary", "state", "mode", "rows", "profile", "err")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reference(path):
    path = Path(path).resolve()
    return {"path": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
            "sha256": sha(path), "stored_bytes": path.stat().st_size}


def load(path):
    return json.loads(Path(path).read_text())


def resolve(path, summary):
    path = Path(path)
    for item in (path, summary.parent / path.name, summary.parent / path):
        if item.is_file():
            return item
        if Path(str(item) + ".gz").is_file():
            return Path(str(item) + ".gz")
    raise ValueError("missing_evidence:" + str(path))


def read_bytes(path):
    path = Path(path)
    return gzip.decompress(path.read_bytes()) if path.suffix == ".gz" else path.read_bytes()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_hold(summary, events, raw):
    """Cross-check actual zero-I/O interval, offsets and immutable authority."""
    hold = summary["hold"]
    start, end = hold["started_monotonic"], hold["finished_monotonic"]
    if (summary["status"] != "UNREAD_HOLD_COMPLETED_RECOVERY_VERIFIED" or
            hold["status"] != "COMPLETE" or hold["FD_kept_open"] is not True or
            not all(finite(v) for v in (start, end, hold["actual_seconds"], hold["requested_seconds"])) or
            end < start or abs(end - start - hold["actual_seconds"]) > 1e-6 or
            hold["actual_seconds"] < hold["requested_seconds"]):
        raise ValueError("hold_incomplete_or_clock_mismatch")
    if any(hold[key] != 0 for key in ("read_calls", "write_calls", "forbidden_read_calls", "forbidden_write_calls")):
        raise ValueError("hold_performed_IO")
    offset = hold["raw_offset_before"]
    if type(offset) is not int or offset != hold["raw_offset_after"] or not 0 <= offset <= len(raw):
        raise ValueError("hold_raw_offset_changed")
    if hashlib.sha256(raw).hexdigest() != summary["raw_sha256"] or len(raw) != summary["rx_bytes"]:
        raise ValueError("formal_raw_hash_or_length_mismatch")
    starts = [i for i, event in enumerate(events) if event.get("type") == "unread_hold_start"]
    ends = [i for i, event in enumerate(events) if event.get("type") == "unread_hold_end"]
    if len(starts) != 1 or len(ends) != 1 or starts[0] >= ends[0]:
        raise ValueError("hold_event_pair_missing_or_repeated")
    begin, finish = starts[0], ends[0]
    for key in ("started_monotonic", "requested_seconds", "deadline_monotonic", "raw_offset_before"):
        if events[begin].get(key) != hold[key]:
            raise ValueError("hold_start_event_disagrees:" + key)
    if any(events[finish].get(key) != value for key, value in hold.items()):
        raise ValueError("hold_end_event_disagrees")
    if any(e.get("type") in {"rx", "tx", "ctrl_rx", "ctrl_tx"} for e in events[begin + 1:finish]):
        raise ValueError("hold_event_IO_observed")
    expected_offset, previous_t = 0, None
    for index, event in enumerate(events):
        when = event.get("t")
        if not finite(when) or previous_t is not None and when < previous_t:
            raise ValueError("formal_event_time_invalid")
        previous_t = when
        if event.get("type") == "rx":
            if event.get("offset") != expected_offset or type(event.get("length")) is not int or event["length"] <= 0:
                raise ValueError("formal_rx_offsets_invalid")
            expected_offset += event["length"]
            if index < begin and expected_offset > offset or index > finish and event["offset"] < offset:
                raise ValueError("formal_rx_crosses_hold_offset")
    if expected_offset != len(raw):
        raise ValueError("formal_raw_receipt_coverage_mismatch")
    snapshots = summary["snapshots"]
    if snapshots["before"]["finished_monotonic"] > start or snapshots["after"]["started_monotonic"] < end:
        raise ValueError("queries_not_outside_hold")
    expected = snapshots["before"]["queries"]["CONFIG?"]["fields"]
    if (expected.get("mode"), expected.get("rows"), expected.get("profile"), expected.get("err")) != (
            "VOLT", "1", "VVVVVVVV", "0x0"):
        raise ValueError("unexpected_hold_configuration")
    build = snapshots["before"]["queries"]["BUILD?"]["fields"]
    for phase in ("before", "after"):
        snap = snapshots[phase]
        if snap["outcome"] != "COHERENT_QUERY_VERIFIED":
            raise ValueError("incoherent_query_bracket")
        phase_events = [e for e in events if e.get("type") == "reply" and e.get("phase") == phase]
        phase_raw = raw[:offset] if phase == "before" else raw[offset:]
        cursor = 0
        for name, query in snap["queries"].items():
            if "fields" not in query:
                if query.get("status") != "CAPABILITY_UNAVAILABLE":
                    raise ValueError("query_fields_unavailable:" + name)
                continue
            command = "CONFIG?" if name == "CONFIG?_closing" else name
            line = query["line"]
            if parse_fields(line) != query["fields"] or line.encode() not in phase_raw.splitlines():
                raise ValueError("query_not_original_complete_raw:" + name)
            matches = [i for i in range(cursor, len(phase_events))
                       if phase_events[i].get("command") == command and phase_events[i].get("line") == line]
            if not matches:
                raise ValueError("query_not_ordered_reply_event:" + name)
            cursor = matches[0] + 1
            times = (snap["started_monotonic"], query["started_monotonic"], query["received_monotonic"],
                     query["finished_monotonic"], snap["finished_monotonic"])
            if not all(finite(t) for t in times) or tuple(sorted(times)) != times:
                raise ValueError("invalid_query_time_bracket:" + name)
        for name in ("CONFIG?", "CONFIG?_closing"):
            if any(snap["queries"][name]["fields"].get(key) != expected.get(key) for key in CONFIG):
                raise ValueError("CONFIG_identity_changed")
        if snap["queries"]["BUILD?"]["fields"] != build:
            raise ValueError("BUILD_changed")
        ready = snap["queries"]["READY?"]["fields"]
        if ready.get("bootId") != expected["bootId"] or ready.get("ready") != "1" or ready.get("err") != "0x0":
            raise ValueError("READY_not_same_boot")
    return expected, build


class NoUsbComparison:
    """Science validation is unchanged; there is deliberately no USB reference."""
    def compare(self, *_args):
        pass

    def result(self, _label):
        return {"status": "NOT_OBSERVED", "reason": "formal_application_performed_no_hold_reads",
                "USB_PC_FPS": None, "post_hold_buffer_drain_used": False}


def analyse_ble_window(journal, start, end, config, label):
    requested = {"mode": config["mode"], "rows": int(config["rows"])}
    window = {"label": label, "requested": requested,
              "expected_config_identity": {key: config[key] for key in IDENTITY},
              "expected_wire_contract": benchmark.expected_wire_contract(requested, config),
              "window": {"capture_monotonic_start": start, "capture_monotonic_end": end}}
    evidence = benchmark.analyse_ble_journal(Path(journal), [window], NoUsbComparison())
    # This is a parser window adapter, never a fabricated capture summary or USB stream.
    return window["ble"], evidence


def counter_delta(before, after):
    if before == "unknown" or after == "unknown":
        return {"before": before, "after": after, "delta": None, "unknown_reason": "explicit_unknown"}
    try:
        a, b = int(before), int(after)
    except (TypeError, ValueError):
        return {"before": before, "after": after, "delta": None, "unknown_reason": "invalid_numeric"}
    if not 0 <= a < 2**64 or not 0 <= b < 2**64 or b < a:
        return {"before": a, "after": b, "delta": None, "unknown_reason": "range_or_decrease_wrap_unresolved"}
    return {"before": a, "after": b, "delta": b-a, "unknown_reason": None}


def query_windows(summary):
    counters = {"USB?": diagnostic.COUNTERS["USB"], "USBQ?": diagnostic.COUNTERS["USBQ"],
                "BLEFLOW?": ("attempts", "capacityReject", "intentionalSkip", "probeSelected"),
                "BLESNAP?": ("queued", "localComplete", "failed", "admitMiss", "deadlineMiss", "superseded", "staleNormal")}
    output = {}
    for command, keys in counters.items():
        before = summary["snapshots"]["before"]["queries"][command]
        after = summary["snapshots"]["after"]["queries"][command]
        if "fields" not in before or "fields" not in after:
            output[command] = {"status": "UNAVAILABLE", "before": before, "after": after}
            continue
        if command.startswith("BLE") and any(before["fields"][k] != after["fields"][k] for k in ("gen", "stream")):
            raise ValueError("BLE_query_epoch_changed")
        output[command] = {
            "status": "VERIFIED_ORIGINAL_QUERY_BRACKET", "before": before, "after": after,
            "interval_seconds": {"minimum": after["started_monotonic"] - before["finished_monotonic"],
                                 "maximum": after["finished_monotonic"] - before["started_monotonic"]},
            "counter_deltas": {key: counter_delta(before["fields"].get(key), after["fields"].get(key)) for key in keys},
            "scope": "independent before/after query brackets, not exact hold endpoints; no PC delivery inferred"}
    return output


def analyze(summary_path, ble_path, debug_path, post_path, identity_path, command_records=()):
    paths = [Path(p) for p in (summary_path, ble_path, debug_path, post_path, identity_path)]
    summary_path, ble_path, debug_path, post_path, identity_path = paths
    summary, debug, post, image = map(load, (summary_path, debug_path, post_path, identity_path))
    formal_raw = resolve(summary["raw_file"], summary_path)
    formal_events = resolve(summary["events_file"], summary_path)
    paths += [formal_raw, formal_events]
    raw = read_bytes(formal_raw)
    events = [json.loads(line) for line in read_bytes(formal_events).splitlines()]
    config, build = validate_hold(summary, events, raw)
    hold = summary["hold"]
    bounds = hold["started_monotonic"], hold["finished_monotonic"]
    if image["elf_sha256"] != build["elfSha256"] or image["version"] != build["version"]:
        raise ValueError("validated_image_mismatch")
    post_fields = {a["command"]: a["fields"] for a in post["attempts"] if "fields" in a}
    if post_fields["BUILD?"] != build or any(post_fields["CONFIG?"][key] != config[key] for key in ("session", "bootId")):
        raise ValueError("postquery_BUILD_or_boot_mismatch")
    if (post_fields["CONFIG?"]["mode"], post_fields["CONFIG?"]["rows"], post_fields["CONFIG?"]["err"]) != ("CAP", "8", "0x0"):
        raise ValueError("postquery_CAP8_not_verified")
    for record in command_records:
        record = Path(record)
        closure = load(record)
        if closure.get("exit_code") != 0 or not closure.get("finished_utc"):
            raise ValueError("collector_not_closed_successfully:" + str(record))
        paths.append(record)
    ble_summary_path = ble_path.with_name(ble_path.name.removesuffix(".gz").replace(".ble.notifications.jsonl", ".ble.summary.json"))
    paths.append(ble_summary_path)
    metadata = debug.get("metadata", {})
    if (debug["status"] not in {"STOPPED", "OBSERVED"} or metadata.get("usb_serial") != summary["transport_metadata"]["usb_serial"] or
            metadata.get("vid") != "303a" or metadata.get("pid") != "4042" or metadata.get("interface_number") != 2):
        raise ValueError("debug_identity_or_closure_mismatch")
    debug_raw = resolve(debug["raw_file"], debug_path)
    debug_events = resolve(debug["events_file"], debug_path)
    paths += [debug_raw, debug_events]
    before_hashes = {str(path): sha(path) for path in paths}
    ble, journal = analyse_ble_window(ble_path, *bounds, config, summary["options"]["label"])
    if (not journal["schema_valid"] or not journal["recording_end_observed"] or
            journal["source"]["file_grew_during_analysis"] or
            journal["collector_summary_verification"]["status"] != "MATCH" or
            not ble["alignment"]["continuous_subscriptions"]):
        raise ValueError("BLE_recording_not_closed_intact_or_full_window")
    stream = diagnostic.read_stream(debug_raw, debug_events, debug["raw_sha256"], event_type="rx", origin=None,
                                    bounds=bounds, source="CDC", memory_validator=benchmark.memory_telemetry_record_error,
                                    expected_backend="otg_cdc_v1")
    if not finite(stream["opened_monotonic"]) or stream["opened_monotonic"] > bounds[0] or debug["ended_monotonic"] < bounds[1]:
        raise ValueError("debug_not_full_observer_window")
    continuity = bool(ble["configuration_groups"] and all(
        all(group["identity"].get(key) == config[key] for key in IDENTITY)
        for group in ble["configuration_groups"]) and not stream["resets"]["counts_by_phase"].get("capture"))
    health = stream["_buckets"]["ADSHEALTH"].report(diagnostic.COUNTERS["ADSHEALTH"], continuity, True)
    values = [health["counters"][key]["delta"] for key in diagnostic.COUNTERS["ADSHEALTH"]]
    denominator = sum(values) if all(value is not None for value in values) else None
    flow = query_windows(summary)
    flow_same = flow["BLEFLOW?"]["status"] == "VERIFIED_ORIGINAL_QUERY_BRACKET"
    counts = ble["capture_frame_counts"]
    missing = counts.get("missing_frames", 0) if counts.get("ordered_integrity_valid_frames", 0) else None
    # An absent Counter key is zero only after an ordered, one-identity sequence proof.
    if "missing_frames" not in counts:
        if len(ble["configuration_groups"]) != 1:
            missing = None
        else:
            group = ble["configuration_groups"][0]
            sequence = group["sequence"]
            if sequence["progress_including_missing"] != counts.get("ordered_integrity_valid_frames", 0) - 1 or sequence["gap_examples"]:
                missing = None
    received = counts.get("ordered_integrity_valid_frames", 0)
    expected = received + missing if missing is not None else None
    sources = [Path(__file__), Path(benchmark.__file__), Path(diagnostic.__file__), ROOT / "tools/text_protocol.py"]
    record = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": "CLOSED_UNREAD_HOLD_ANALYZED", "overall_acceptance": "NOT_ESTABLISHED",
        "evidence_kind": "OFFLINE_ANALYSIS_OF_REAL_CLOSED_UNREAD_HOLD_AND_INDEPENDENT_OBSERVERS",
        "inputs": [reference(path) for path in paths], "analysis_sources": [reference(path) for path in sources],
        "running_image": {"BUILD": build, "validated_identity": reference(identity_path)},
        "configuration": config, "hold": hold, "hold_raw_and_event_consistency_verified": True,
        "parser_adapter": "Existing analyse_ble_journal window interface receives the real hold monotonic bounds and expected CONFIG in memory. No capture summary or USB frame record is fabricated.",
        "USB_PC_receipt": "NOT_OBSERVED", "USB_PC_FPS": None, "post_hold_drain_used_as_hold_receipt": False,
        "ble": ble, "ble_journal": journal,
        "BLE_endpoint_gap": {"received": received, "missing": missing, "expected_within_BLE_endpoints": expected,
                             "fraction": missing / expected if expected else None,
                             "scope": "same-identity trusted complete BLE endpoints only; no USB denominator or RF-loss attribution"},
        "query_windows": flow,
        "independent_debug": {**diagnostic.public_stream(stream), "hold_observer_coverage": "FULL_OBSERVER_WINDOW",
                              "boot_continuity_observed": continuity,
                              "absence_of_fault_text_proves_no_fault": False},
        "ADC_CRC": {"health": health, "valid_even_delta": values[0], "valid_odd_delta": values[1],
                    "rejected_delta": values[2], "checked_attempts": denominator,
                    "rate": values[2] / denominator if denominator else None,
                    "scope": "first/last complete ADSHEALTH inside hold, not exact hold endpoints; checked reads, not scientific frames",
                    "crcRetries": "unknown_unverified_terminal_token",
                    "ADSSPI": stream["_buckets"]["ADSSPI"].report(diagnostic.COUNTERS["ADSSPI"], continuity, True)},
        "BLE_firmware": stream["_buckets"]["BL50"].report(diagnostic.COUNTERS["BL50"], continuity and flow_same, True),
        "post_hold": {"collector_recovery": summary["safe_cap_recovery"], "collector_CONFIG": summary["cleanup_CONFIG"],
                      "postquery": reference(post_path), "CONFIG": post_fields["CONFIG?"],
                      "MEM": post_fields.get("MEM?"), "BLEGUARD": post_fields.get("BLEGUARD?")},
        "limitations": [
            "Application unread hold keeps one formal CDC FD open; Linux may still buffer USB traffic. This is not cable removal, Windows re-enumeration or zero bus traffic.",
            "Formal USB has no application receipt during hold; later buffered drain never becomes hold-time USB FPS.",
            "Capture FPS is inferred from ordered BLE scientific device timestamps/sequences, not directly counted ADC conversions.",
            "Whole protocol integrity preserves invalid/Xhh points; complete does not mean all points valid or precision calibrated.",
            "Local queue expiry/rejection/submission and BLE PC sequence gaps are distinct windows; do not add or subtract them as disjoint losses.",
            "No PC application ACK, cancellation guarantee or absolute one-second snapshot delivery is inferred.",
            "Current-image long-run/full-matrix/weak-link/physical-unplug acceptance is not completed by this single stress observation."],
        "hardware_actions": [], "original_recordings_modified": False, "production_source_modified": False}
    for path in paths:
        if sha(path) != before_hashes[str(path)]:
            raise ValueError("input_changed_during_analysis:" + str(path))
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("summary", "ble-journal", "debug-summary", "post-summary", "identity", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--command-record", action="append", default=[], type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError("use_new_output_label")
    result = analyze(args.summary, args.ble_journal, args.debug_summary, args.post_summary, args.identity, args.command_record)
    with args.output.open("x") as output:
        json.dump(result, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(json.dumps({"status": result["status"], "hold_seconds": result["hold"]["actual_seconds"],
                      "BLE": result["BLE_endpoint_gap"], "USB_PC_FPS": result["USB_PC_FPS"],
                      "ADC_checked_reads": result["ADC_CRC"]["checked_attempts"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
