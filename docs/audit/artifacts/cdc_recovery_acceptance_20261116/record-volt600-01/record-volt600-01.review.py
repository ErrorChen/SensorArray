#!/usr/bin/env python3
"""Offline scope check for a closed 600 s result; does not change its verdict."""
from bisect import bisect_right
import gzip
import hashlib
import json
from pathlib import Path
import re

BASE = Path(__file__).resolve().parent
LABEL = BASE.name
load = lambda suffix: json.loads((BASE / (LABEL + suffix)).read_text())
sha = lambda data: hashlib.sha256(data).hexdigest()


def source(meta):
    path = Path(meta["path"])
    if not path.exists():
        path = Path(str(path) + ".gz")
    data = path.read_bytes()
    raw = gzip.decompress(data) if path.suffix == ".gz" else data
    assert sha(raw) == meta["raw_sha256"]
    return raw


def main():
    metrics = load(".metrics.json")
    provenance = load(".metrics.provenance.json")
    gate = load(".gate.json")
    raw = source(provenance["inputs"]["formal_raw"])
    wire_events = source(provenance["inputs"]["formal_events"])
    offsets, receipts, cursor, bounds = [], [], 0, {}
    for line in wire_events.splitlines():
        event = json.loads(line)
        if event.get("type") in ("capture_start", "capture_end"):
            bounds[event["type"]] = event["t"]
        if event.get("type") == "rx":
            assert event["offset"] == cursor
            offsets.append(event["offset"])
            receipts.append(event["t"])
            cursor += event["length"]
    assert cursor == len(raw)
    def receipt(offset):
        return receipts[bisect_right(offsets, offset) - 1]
    def fields(wire):
        return {k.decode(): v.decode() for part in wire.rstrip(b"\r\n").split(b",")[1:]
                for k, separator, v in [part.partition(b"=")] if separator}
    headers = list(re.finditer(rb"^[CVRM],seq=(\d+),[^\n]*\n", raw, re.MULTILINE))
    gaps = []
    for before, after in zip(headers, headers[1:]):
        a, b = int(before.group(1)), int(after.group(1))
        distance = (b - a) & 0xffffffff
        if not 1 < distance < 0x80000000:
            continue
        trailer_start = raw.rfind(b"\nK,seq=", before.end(), after.start()) + 1
        assert trailer_start > before.end()
        trailer_end = raw.index(b"\n", trailer_start) + 1
        trailer = fields(raw[trailer_start:trailer_end])
        assert int(trailer["seq"]) == a
        gaps.append({"missing_first": (a + 1) & 0xffffffff, "missing_last": (b - 1) & 0xffffffff,
                     "missing_count": distance - 1,
                     "before_header": fields(before.group()), "after_header": fields(after.group()),
                     "before_header_offset": before.start(), "before_K_offset": trailer_start,
                     "before_K_end_exclusive": trailer_end, "after_header_offset": after.start(),
                     "before_K_receipt_seconds": receipt(trailer_end - 1),
                     "after_header_last_byte_receipt_seconds": receipt(after.end() - 1),
                     "both_before_formal_capture_start": receipt(trailer_end - 1) < bounds["capture_start"] and receipt(after.end() - 1) < bounds["capture_start"],
                     "cause": "NOT_ESTABLISHED"})
    counters = gate["analysis"]["captures"][0]["whole_raw_parser_counters"]
    assert len(gaps) == counters["sequence_gaps"]
    assert sum(g["missing_count"] for g in gaps) == counters["missing_frames"]
    assert all(g["both_before_formal_capture_start"] for g in gaps)
    health = load(".health-coverage.json")
    health_gaps = [{"before": a, "after": b,
                    "engine_step": int(b["fields"]["engineFrameCount"]) - int(a["fields"]["engineFrameCount"])}
                   for a, b in zip(health["records"], health["records"][1:])
                   if int(b["fields"]["engineFrameCount"]) - int(a["fields"]["engineFrameCount"]) != 64]
    evidence = {"schema_version": 1, "original_gate_status": gate["status"],
                "formal_window": bounds, "whole_raw_sequence_gaps": gaps,
                "health_non64_engine_steps": health_gaps,
                "formal_USB_missing_observations": metrics["USB"]["missing_observations"],
                "BLE_missing_observations": metrics["BLE"]["missing_observations"],
                "raw_SHA256": sha(raw),
                "inputs": {str(BASE / (LABEL + suffix)): sha((BASE / (LABEL + suffix)).read_bytes())
                           for suffix in (".metrics.json", ".metrics.provenance.json", ".gate.json", ".health-coverage.json")},
                "generator": {"path": str(Path(__file__)), "sha256": sha(Path(__file__).read_bytes())},
                "limits": ["Formal-window receipt does not imply a gap-free full raw file.",
                           "The missing intermediate health log is an observation gap; its cause and health observations outside the first/last bracket remain unknown.",
                           "CPU owner percentages use separate sample brackets; total CPU, SDK CPU and blocked time remain unknown.",
                           "Heap observations include PSRAM; internal free and largest contiguous block are not established by these totals."],
                "hardware_actions": [], "original_recordings_modified": False}
    target = BASE / (LABEL + ".review.json")
    with target.open("x") as stream:
        json.dump(evidence, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(json.dumps({"review": str(target), "whole_raw_gaps": gaps,
                      "health_non64_steps": [{"before_engine": g["before"]["fields"]["engineFrameCount"],
                                             "after_engine": g["after"]["fields"]["engineFrameCount"],
                                             "engine_step": g["engine_step"]} for g in health_gaps]}))


if __name__ == "__main__":
    main()
