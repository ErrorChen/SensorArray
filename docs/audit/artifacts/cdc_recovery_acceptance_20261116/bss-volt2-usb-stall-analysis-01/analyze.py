#!/usr/bin/env python3
"""Replay recorded VOLT2 USB/BLE evidence. No device, network or git access.

Run from any directory; JSON is written to stdout. Original recordings are
opened read-only. The caller may redirect stdout to a new evidence file.
"""
from __future__ import annotations

import base64
import bisect
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[5]
STAGE = ROOT / "docs/audit/artifacts/cdc_recovery_acceptance_20261116"
CASE = STAGE / "bss-matrix-02/bss-matrix-02-volt2"
LABEL = CASE.name
SNAPSHOT = Path(__file__).resolve().parent / "source-snapshot"
sys.path.insert(0, str(SNAPSHOT / "tools"))
from text_protocol import FragmentReassembler, TextProtocolParser, parse_fields


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def relative(path):
    return str(Path(path).relative_to(ROOT))


def recorded_bytes(path):
    if path.exists():
        return path.read_bytes()
    return gzip.decompress(Path(str(path) + ".gz").read_bytes())


def read_events(path):
    return [(number, json.loads(line)) for number, line in
            enumerate(recorded_bytes(path).decode().splitlines(), 1) if line.strip()]


def main():
    gate = read_json(CASE / (LABEL + ".gate.json"))
    summary = read_json(CASE / (LABEL + ".summary.json"))
    raw_path = CASE / (LABEL + ".rx.bin")
    raw = recorded_bytes(raw_path)
    events = read_events(CASE / (LABEL + ".events.jsonl"))
    debug_events = read_events(CASE / (LABEL + ".debug.events.jsonl"))
    ble_path = CASE / (LABEL + ".ble.notifications.jsonl")
    ble_events = read_events(ble_path)
    expected = gate["expected"]
    identity = gate["analysis"]["captures"][0]["expected_config_identity"]
    for group in (gate["formal_authority"], gate["debug"]):
        for kind in ("summary", "raw", "events"):
            item = group[kind]
            path = Path(item["path"])
            if not path.is_absolute():
                path = ROOT / path
            assert digest(recorded_bytes(path)) == item["sha256"], (path, "recorded source changed")
    ble_source = gate["analysis"]["ble_journal"]["source"]
    ble_bytes = recorded_bytes(ble_path)
    assert len(ble_bytes) == ble_source["uncompressed_prefix_bytes"]
    assert digest(ble_bytes) == ble_source["uncompressed_prefix_sha256"]
    assert ble_source["file_grew_during_analysis"] is False

    provenance = []
    for item in gate["analysis"]["parser_sources"]:
        original = Path(item["path"])
        path = SNAPSHOT / original.relative_to(ROOT)
        actual = digest(path.read_bytes())
        assert actual == item["sha256"], (path, "parser source changed")
        provenance.append({"path": relative(path), "sha256": actual,
                           "original_path": relative(original),
                           "matches_recorded_parser": True})
    collector_sources = [
        (ROOT / "docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts/capture_cdc_benchmark.py",
         summary["source_sha256"]),
        (ROOT / "tools/run_firmware_benchmark.py", summary["base_runner_source_sha256"]),
    ]
    for original, expected_hash in collector_sources:
        path = STAGE / "historical-runner-sources" / expected_hash / original.name
        actual = digest(path.read_bytes())
        assert actual == expected_hash, (path, "collector source changed")
        provenance.append({"path": relative(path), "sha256": actual,
                           "original_path": relative(original),
                           "matches_recorded_collector": True})
    manifest = read_json(STAGE / "wifi-bss-01-build-manifest.json")
    for name in ("core/transport/sensorarraySerialIo.c",
                 "components/sensorarrayUsbOtg/sensorarrayUsbOtg.c",
                 "components/sensorarrayUsbOtg/include/tusb_config.h",
                 "main/control/sensorarrayUsbControl.c"):
        actual = digest((SNAPSHOT / name).read_bytes())
        assert actual == manifest["inputs_before"][name] == manifest["inputs_after"][name]
        provenance.append({"path": relative(SNAPSHOT / name), "original_path": name,
                           "sha256": actual,
                           "snapshot_source_commit": "f8d8424492c855a402838737ee8718e32f5135d6",
                           "matches_image_build_inputs_before_and_after": True})
    assert digest(raw) == summary["raw_sha256"]
    for name in (LABEL + ".summary.json", LABEL + ".events.jsonl",
                 LABEL + ".rx.bin", LABEL + ".debug.events.jsonl",
                 LABEL + ".debug.rx.bin", LABEL + ".ble.notifications.jsonl",
                 LABEL + ".gate.json"):
        path = CASE / name
        data = recorded_bytes(path)
        provenance.append({"uncompressed_path": relative(path), "bytes": len(data),
                           "uncompressed_sha256": digest(data)})

    rx = [(number, event) for number, event in events if event["type"] == "rx"]
    end = 0
    for _, event in rx:
        assert event["offset"] == end and event["length"] > 0
        end += event["length"]
    assert end == len(raw)
    offsets = [event["offset"] for _, event in rx]

    def locate(offset):
        index = bisect.bisect_right(offsets, offset) - 1
        number, event = rx[index]
        assert event["offset"] <= offset < event["offset"] + event["length"]
        return {"event_line": number, "t": event["t"], "phase": event["phase"],
                "chunk_offset": event["offset"], "chunk_length": event["length"]}

    start_event = next(event for _, event in events if event["type"] == "capture_start")
    end_event = next(event for _, event in events if event["type"] == "capture_end")
    start_t, end_t = start_event["t"], end_event["t"]
    mono_start, mono_end = summary["capture_monotonic_start"], summary["capture_monotonic_end"]

    def identity_ok(header, trailer):
        return (all(header.get(key) == str(value) for key, value in identity.items())
                and all(trailer.get(key) == header.get(key)
                        for key in ("seq", "gen", "rid", "session", "bootId", "rev")))

    parser = TextProtocolParser()
    pending = None
    usb = {}
    truncated = []
    offset = 0
    for wire in raw.splitlines(keepends=True):
        line = wire.decode("ascii").rstrip("\r\n")
        tag = line.split(",", 1)[0]
        if tag in {"C", "V", "R", "M"}:
            if pending:
                truncated.append({"header": pending["header"], "offset": pending["offset"],
                                  "next_header_offset": offset})
            pending = {"header": parse_fields(line), "tag": tag,
                       "offset": offset, "wire": bytearray(wire)}
        elif pending and (tag == "MR" or re.fullmatch(r"[DP][0-3]", tag)):
            pending["wire"].extend(wire)
        frame = parser.feed_line(line)
        if tag == "K":
            if pending and frame is not None:
                header, trailer = pending["header"], parse_fields(line)
                pending["wire"].extend(wire)
                first, last = locate(pending["offset"]), locate(offset + len(wire) - 1)
                if (pending["tag"] == "V" and frame.rows == 2 and frame.crc_ok
                        and identity_ok(header, trailer)
                        and start_t <= first["t"] <= last["t"] <= end_t):
                    seq = int(header["seq"])
                    assert seq not in usb
                    usb[seq] = {"wire": bytes(pending["wire"]), "first": first, "last": last,
                                "offset": pending["offset"], "end_offset": offset + len(wire)}
            pending = None
        offset += len(wire)

    assembler = FragmentReassembler()
    envelope_starts = {}
    ble = {}
    for number, event in ble_events:
        if event["type"] != "notification":
            continue
        payload = base64.b64decode(event["payload_base64"])
        now = event["monotonic_ns"] / 1e9
        key = None
        if payload.startswith(b"G,"):
            fields = payload.split(b"\n", 1)[0].split(b",")
            key = (fields[1].decode("ascii"), int(fields[2]))
            envelope_starts.setdefault(key, {"monotonic": now, "line": number})
        for channel, data in assembler.feed(event["channel"], payload):
            first = envelope_starts.pop(key, {"monotonic": now, "line": number})
            if channel != "D" or not data.startswith(b"V,seq="):
                continue
            lines = data.decode("ascii").splitlines()
            header, trailer = parse_fields(lines[0]), parse_fields(lines[-1])
            one = TextProtocolParser()
            complete = [frame for line in lines if (frame := one.feed_line(line)) is not None]
            if (len(complete) == 1 and complete[0].crc_ok and complete[0].rows == 2
                    and identity_ok(header, trailer)
                    and mono_start <= first["monotonic"] <= now <= mono_end):
                seq = int(header["seq"])
                assert seq not in ble
                ble[seq] = {"wire": data, "header": header, "trailer": trailer,
                            "first_notification_line": first["line"],
                            "complete_notification_line": number,
                            "first_monotonic": first["monotonic"], "complete_monotonic": now}

    usb_seq, ble_seq = sorted(usb), sorted(ble)
    usb_gaps = [{"after": a, "before": b, "missing": list(range(a + 1, b))}
                for a, b in zip(usb_seq, usb_seq[1:]) if b != a + 1]
    assert len(usb) == 5043 and len(ble) == 5050
    assert usb_seq[0] == ble_seq[0] == 76513 and usb_seq[-1] == ble_seq[-1] == 81562
    assert ble_seq == list(range(76513, 81563))
    assert all(usb[seq]["wire"] == ble[seq]["wire"] for seq in usb)
    missing = sorted(set(ble) - set(usb))
    assert missing == [77016, 77021, 77022, 77023, 77024, 77025, 77026]
    details = []
    for seq in missing:
        entry = ble[seq]
        marks = [match.start() for match in re.finditer(rb"seq=" + str(seq).encode() + rb",", raw)]
        info = {"seq": seq, "BLE_bytes": len(entry["wire"]),
                "BLE_crc": entry["trailer"]["crc"], "BLE_crc_valid": True,
                "identity_verified": True, "BLE_sha256": digest(entry["wire"]),
                "BLE_first_notification_line": entry["first_notification_line"],
                "BLE_complete_notification_line": entry["complete_notification_line"],
                "BLE_complete_monotonic": entry["complete_monotonic"],
                "USB_seq_token_offsets_anywhere_in_raw": marks,
                "BLE_entire_wire_in_USB_raw": entry["wire"] in raw}
        if seq == 77016:
            start = raw.index(b"V,seq=77016,")
            stop = raw.index(b"V,seq=77017,")
            partial = raw[start:stop]
            common = 0
            while common < min(len(partial), len(entry["wire"])) and partial[common] == entry["wire"][common]:
                common += 1
            assert len(partial) == 331 and common == 330 and partial[common:] == b"\n"
            info["USB_partial"] = {"offset": start, "next_header_offset": stop,
                "bytes": len(partial), "matching_BLE_prefix_bytes": common,
                "extra_suffix_hex": partial[common:].hex(),
                "missing_original_suffix_bytes": len(entry["wire"]) - common,
                "last_text": partial[-48:].decode(), "first_receipt": locate(start),
                "last_receipt": locate(stop - 1)}
        else:
            assert not marks
        details.append(info)

    usbq = []
    usb_state = []
    for window in summary["diagnostic_windows"]:
        query = window.get("queries", {}).get("USBQ?")
        if query:
            usbq.append({"window": window["name"], "receipt_t": query["t"],
                         "receipt_monotonic": query["monotonic"], "fields": query["fields"]})
        query = window.get("queries", {}).get("USB?")
        if query:
            usb_state.append({"window": window["name"], "receipt_t": query["t"],
                "fields": {key: query["fields"][key] for key in
                           ("epoch", "resets", "mounts", "unmounts", "mounted", "suspended")}})
    changes = []
    for before, after in zip(usbq, usbq[1:]):
        delta = {key: int(after["fields"][key]) - int(before["fields"][key])
                 for key in ("dDrop", "cDrop", "expired", "cExpired", "partial")}
        if any(delta.values()):
            changes.append({"before": before, "after": after, "delta": delta})
    assert len(changes) == 1
    assert changes[0]["delta"] == {"dDrop": 6, "cDrop": 0, "expired": 1, "cExpired": 0, "partial": 1}
    assert all(row["fields"] == usb_state[0]["fields"] for row in usb_state)

    formal_pauses = []
    for (n1, a), (n2, b) in zip(rx, rx[1:]):
        if 16.0 < b["t"] < 18.0 and b["t"] - a["t"] > 0.05:
            formal_pauses.append({"before_event_line": n1, "after_event_line": n2,
                "before_t": a["t"], "after_t": b["t"], "seconds": b["t"] - a["t"],
                "before_raw_end": a["offset"] + a["length"], "after_raw_offset": b["offset"]})
    observer_pauses = {}
    for name, source, event_type in (("debug", debug_events, "rx"), ("BLE", ble_events, "notification")):
        selected = [(number, event, event["monotonic"] if name == "debug" else event["monotonic_ns"] / 1e9)
                    for number, event in source if event["type"] == event_type]
        observer_pauses[name] = [{"before_event_line": n1, "after_event_line": n2,
                                "before_monotonic": t1, "after_monotonic": t2, "seconds": t2 - t1}
                               for (n1, _, t1), (n2, _, t2) in zip(selected, selected[1:])
                               if 176501 < t2 < 176502.8 and t2 - t1 > 0.05]

    debug_raw = recorded_bytes(CASE / (LABEL + ".debug.rx.bin"))
    debug_evidence = []
    for match in re.finditer(rb"[^\n]*(?:SF50,seq=77001-77050|ADSHEALTH,seq=77024)[^\n]*", debug_raw):
        debug_evidence.append({"offset": match.start(), "line": match.group().decode()})
    result = {
        "schema_version": 1,
        "status": "CONFIRMED_USB_INCOMPLETE_FRAMES_WITH_BLE_MATCH_AND_LOCAL_BACKPRESSURE_SIGNATURE",
        "hardware_actions": [], "git_actions": [], "original_recordings_modified": False,
        "expected_image": expected, "configuration_identity": identity,
        "recorded_gate": {"status": gate["status"], "failures": gate["science"]["failures"]},
        "provenance": provenance,
        "raw_accounting": {"bytes": len(raw), "rx_events": len(rx),
                           "contiguous_nonoverlapping_offsets": True, "summary_sha_matches": True},
        "window": {"start_event_t": start_t, "end_event_t": end_t,
                   "capture_start_monotonic": mono_start, "capture_end_monotonic": mono_end},
        "USB": {"trusted_complete_frames": len(usb), "first": usb_seq[0], "last": usb_seq[-1],
                "gaps": usb_gaps, "whole_raw_parser_counters": asdict(parser.counters),
                "truncated_headers": truncated},
        "BLE": {"trusted_complete_frames": len(ble), "first": ble_seq[0], "last": ble_seq[-1],
                "missing_within_endpoints": 0, "common_wire_identical": len(usb),
                "common_wire_differences": 0, "only_BLE_sequences": missing},
        "missing_frame_evidence": details, "USBQ_changes": changes, "USB_state_samples": usb_state,
        "formal_RX_pauses_near_failure": formal_pauses,
        "other_observer_pauses_near_failure": observer_pauses,
        "debug_corrobation": debug_evidence,
        "causal_limits": [
            "Missing complete USB frames are real in the recorded byte stream, not parser-only or capture-edge gaps.",
            "77016 has the exact first 330 BLE wire bytes plus LF; the remaining original 383 bytes are absent.",
            "The 6 DATA rejections and one normal expiry/partial strongly match the six absent frames and one partial frame, but counters carry no per-sequence trace.",
            "normalExpired and partialWrite aggregate DATA/LOG lanes. Their delta alone is not a direct attribution to sequence 77016.",
            "The matching-image OTG code expires started normal records at their original 250000-us enqueue deadline and inserts LF after a partial expiry.",
            "No priority bytes interleave between the affected raw headers; priority rejection/expiry counters remain unchanged. This does not prove every internal scheduling decision.",
            "Formal USB, debug USB and BLE host receipts pause and resume together nearby. Callback timing cannot distinguish host scheduling, host USB/controller backlog or device service delay.",
            "No inference of radio loss, capture-rate reduction, SDK panic, USB reset or invalid-point reconstruction is made.",
        ],
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
