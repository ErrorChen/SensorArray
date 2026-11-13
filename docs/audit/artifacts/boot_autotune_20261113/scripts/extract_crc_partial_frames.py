#!/usr/bin/env python3
"""Extract an unmodified recorded CRC-partial frame and its immediate successor.

Only explicitly completed captures may be supplied. No input is discovered or
read automatically. The firmware's shared TextProtocolParser verifies the
captured K CRC; this tool never generates/replaces a wire CRC or repairs lines.
Exit 0: selected and written; 2: evidence insufficient; 3: input/output error.
--self-test uses existing recorded/synthetic fixture bytes, not active captures.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
from reference_fixture import IDENTITY_FIELDS, _identity, _trailer_matches
from text_protocol import TextProtocolParser, MeasurementFrame, MixedFrame


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def fields(text: str) -> dict[str, str]:
    result = {}
    for token in text.split(",")[1:]:
        if token.count("=") != 1:
            raise ValueError("malformed key/value record")
        key, value = token.split("=", 1)
        if key in result or not re.fullmatch(r"[A-Za-z][A-Za-z0-9]*", key) or not value:
            raise ValueError("duplicate/empty/invalid field")
        result[key] = value
    return result


def integer(value: str, *, hexadecimal: bool = False) -> int:
    pattern = r"(?:0[xX])?[0-9a-fA-F]+" if hexadecimal else r"[0-9]+"
    if not re.fullmatch(pattern, value):
        raise ValueError("not an unsigned integer")
    return int(value, 16 if hexadecimal else 10)


def lines(data: bytes):
    offset = 0
    for number, raw in enumerate(data.splitlines(keepends=True), 1):
        complete = raw.endswith(b"\n")
        try:
            text = raw.removesuffix(b"\n").removesuffix(b"\r").decode("ascii")
        except UnicodeDecodeError:
            text = None
        yield {"line": number, "offset": offset, "end_offset": offset + len(raw),
               "complete": complete, "text": text}
        offset += len(raw)


def parse_frame(data: bytes, first: dict, last: dict, epoch: int) -> dict:
    result = {"offset": first["offset"], "end_offset": last["end_offset"],
              "line": first["line"], "end_line": last["line"], "epoch": epoch,
              "error": None, "header": {}, "trailer": {}, "frame": None}
    payload = data[result["offset"]:result["end_offset"]]
    try:
        records = list(lines(payload))
        if not records or any(not r["complete"] or r["text"] is None for r in records):
            raise ValueError("unterminated or non-ASCII frame record")
        tag = records[0]["text"].split(",", 1)[0]
        if tag not in {"C", "V", "R", "M"}:
            raise ValueError("not a frame header")
        result["tag"] = tag
        header, trailer = fields(records[0]["text"]), fields(records[-1]["text"])
        result["header"], result["trailer"] = header, trailer
        identity, identity_state = _identity(header)
        if identity_state != "consistent_frame_identity":
            raise ValueError("missing or invalid immutable identity")
        result["identity"] = {k: identity[k] for k in IDENTITY_FIELDS}
        required_trailer = {"seq", "crc", "session", "bootId", "rev"}
        required_trailer |= {"rgen", "rrid", "pgen", "prid"} if tag == "M" else {"gen", "rid"}
        if not required_trailer <= trailer.keys() or not _trailer_matches(header, trailer):
            raise ValueError("trailer identity absent or disagrees with header")
        if not records[-1]["text"].startswith("K,") or not re.fullmatch(r"[0-9A-Fa-f]{8}", trailer["crc"]):
            raise ValueError("missing complete wire K CRC")
        if not {"expected", "acquired"} <= header.keys():
            raise ValueError("explicit acquisition masks absent")
        parser = TextProtocolParser()
        parsed = None
        for index, record in enumerate(records):
            line = record["text"]
            record_tag = line.split(",", 1)[0]
            if 0 < index < len(records) - 1:
                allowed = record_tag == "MR" if tag == "M" else bool(re.fullmatch(r"[DP][0-3]", record_tag))
                if not allowed:
                    raise ValueError("interleaved/unknown record in contiguous encoder slice")
            emitted = parser.feed_line(line)
            if emitted is not None:
                if parsed is not None or index != len(records) - 1:
                    raise ValueError("multiple/early parsed frames")
                parsed = emitted
        if parsed is None or not parsed.crc_ok or parser.counters.crc_errors or parser.counters.malformed:
            raise ValueError("shared parser rejected frame or captured CRC")
        sequence = integer(header["seq"])
        if sequence > 0xFFFFFFFF or parsed.sequence != sequence:
            raise ValueError("invalid sequence")
        result.update(frame=parsed, sequence=sequence,
                      expected=integer(header["expected"], hexadecimal=True),
                      acquired=integer(header["acquired"], hexadecimal=True),
                      length=len(payload), sha256=digest(payload))
        # Preserve token evidence as well as parser masks. In MIXED, parser
        # exposes None but not the Xhh reason, so the exact MR token is needed.
        tokens = []
        if tag in {"V", "R"}:
            for record in records:
                if re.match(r"D[0-3],", record["text"]):
                    tokens.extend(record["text"].split(",")[1:])
        elif tag == "M":
            by_row = {}
            for record in records:
                if record["text"].startswith("MR,"):
                    prefix, values = record["text"].split(",D=", 1)
                    by_row[integer(fields(prefix)["s"])] = values.split(",")
            for row in range(1, parsed.rows + 1):
                tokens.extend(by_row[row])
        result["tokens"] = tokens
    except (KeyError, ValueError, IndexError, UnicodeError) as error:
        result["error"] = str(error)
    return result


def scan(data: bytes) -> tuple[list[dict], list[dict], dict]:
    frames, diagnostics, counts = [], [], Counter()
    pending = None
    epoch = 0
    for record in lines(data):
        text = record["text"]
        if text is None:
            counts["non_ascii_records"] += 1
            continue
        tag = text.split(",", 1)[0]
        reset = text.startswith("ESP-ROM:") or (tag == "RST" and record["complete"])
        if reset:
            epoch += 1
            pending = None
        if tag == "ADSDISPOSITION":
            diagnostic = {**record, "epoch": epoch, "error": None}
            try:
                if not record["complete"]:
                    raise ValueError("unterminated diagnostic")
                f = fields(text)
                required = {"seq", "boundedCrc", "crcRejected", "acquired", "expected",
                            "allCrcRun", "limit", "firstError", "abortError", "scope"}
                if not required <= f.keys() or f["scope"] != "ads_sweep":
                    raise ValueError("incomplete/unrecognized diagnostic schema")
                diagnostic["fields"] = f
                for key in ("seq", "boundedCrc", "allCrcRun", "limit"):
                    diagnostic[key] = integer(f[key])
                for key in ("crcRejected", "acquired", "expected", "firstError", "abortError"):
                    diagnostic[key] = integer(f[key], hexadecimal=True)
                if diagnostic["seq"] > 0xFFFFFFFF:
                    raise ValueError("diagnostic sequence out of range")
            except ValueError as error:
                diagnostic["error"] = str(error)
            diagnostics.append(diagnostic)
        is_header = tag in {"C", "V", "R", "M"} and ",seq=" in text and ",rows=" in text
        if is_header:
            if pending:
                frames.append({"error": "new header before previous K", "offset": pending["offset"],
                               "end_offset": record["offset"], "epoch": epoch})
            pending = record
        elif tag == "K" and pending:
            frames.append(parse_frame(data, pending, record, epoch))
            pending = None
    if pending:
        frames.append({"error": "unterminated final frame", "offset": pending["offset"],
                       "end_offset": len(data), "epoch": epoch})
    counts["frame_attempts"] = len(frames)
    counts["complete_verified_frames"] = sum(not f["error"] for f in frames)
    counts["diagnostic_records"] = len(diagnostics)
    counts["rejected_diagnostic_records"] = sum(d["error"] is not None for d in diagnostics)
    return frames, diagnostics, dict(counts)


def masks(frame: dict) -> tuple[int, int]:
    parsed = frame["frame"]
    if isinstance(parsed, MeasurementFrame):
        return parsed.valid_mask, parsed.fresh_mask
    if isinstance(parsed, MixedFrame):
        valid = fresh = 0
        for row, data in parsed.row_frames.items():
            valid |= data.valid_mask << ((row - 1) * 8)
            fresh |= data.fresh_mask << ((row - 1) * 8)
        return valid, fresh
    raise ValueError("candidate is not V/R/M")


def match_partial(frame: dict, diagnostics: list[dict]) -> list[dict] | None:
    if not diagnostics or any(d["error"] for d in diagnostics):
        return None
    missing = frame["expected"] & ~frame["acquired"]
    if not missing or frame["acquired"] & ~frame["expected"]:
        return None
    valid, fresh = masks(frame)
    if missing & (valid | fresh):
        return None
    explained = 0
    for d in diagnostics:
        if d["boundedCrc"] != 1 or d["abortError"] != 0 or d["crcRejected"] == 0:
            return None
        expected = d["expected"]
        if not expected or expected & ~frame["expected"] or d["acquired"] & ~expected:
            return None
        if frame["acquired"] & expected != d["acquired"] or d["crcRejected"] & ~expected:
            return None
        absent = expected & ~d["acquired"]
        if absent & ~d["crcRejected"]:
            return None
        for cell, token in enumerate(frame["tokens"]):
            if d["crcRejected"] & (1 << cell) and token != "X02":
                return None
        explained |= absent
    return diagnostics if explained == missing else None


def select(frames: list[dict], diagnostics: list[dict], tag: str | None = None):
    by_seq, boot_identities = defaultdict(list), defaultdict(set)
    for d in diagnostics:
        if d["error"] is None:
            by_seq[(d["epoch"], d["seq"])].append(d)
    for frame in frames:
        if not frame["error"]:
            boot_identities[frame["epoch"]].add((frame["identity"]["session"], frame["identity"]["bootId"]))
    rejects = Counter()
    for index, frame in enumerate(frames[:-1]):
        if frame["error"] or frame.get("tag") not in ({tag} if tag else {"V", "R", "M"}):
            continue
        if frame["acquired"] == frame["expected"]:
            continue
        if len(boot_identities[frame["epoch"]]) != 1:
            rejects["ambiguous_boot_epoch"] += 1
            continue
        evidence = match_partial(frame, by_seq[(frame["epoch"], frame["sequence"])])
        if evidence is None:
            rejects["missing_or_inconsistent_disposition"] += 1
            continue
        successor = frames[index + 1]  # Never skip a broken/lost intervening frame.
        if successor["error"] or successor["epoch"] != frame["epoch"] or \
                successor.get("sequence") != ((frame["sequence"] + 1) & 0xFFFFFFFF):
            rejects["immediate_successor_missing_or_invalid"] += 1
            continue
        keys = (*IDENTITY_FIELDS, "gen", "rid", "rows", "cells", "mode", "profile", "unit", "scale")
        if successor["tag"] != frame["tag"] or any(frame["header"].get(k) != successor["header"].get(k) for k in keys):
            rejects["successor_configuration_changed"] += 1
            continue
        return (frame, successor, evidence), dict(rejects)
    return None, dict(rejects)


def frame_evidence(frame: dict, path: Path, source: dict) -> dict:
    valid, fresh = masks(frame)
    return {"file": relative(path), "provenance": "recorded_unmodified_device_encoder_output",
            "source": source, "offset": frame["offset"], "end_offset": frame["end_offset"],
            "length": frame["length"], "line": frame["line"], "end_line": frame["end_line"],
            "sha256": frame["sha256"], "tag": frame["tag"], "sequence": frame["sequence"],
            "identity": frame["identity"], "header": frame["header"], "trailer": frame["trailer"],
            "valid": f"{valid:016X}", "fresh": f"{fresh:016X}",
            "expected": f"{frame['expected']:016X}", "acquired": f"{frame['acquired']:016X}",
            "wire_crc": frame["trailer"]["crc"], "actual_parser_crc_ok": True,
            "raw_bytes_preserved": True}


def self_test() -> int:
    # Read only committed fixture files; never glob/open ongoing captures.
    fixture = ROOT / "tests/fixtures/reference_capture/res8_crc_rejected.recorded.txt"
    data = fixture.read_bytes()
    frames, diagnostics, _ = scan(data)
    assert len(frames) == 1 and not frames[0]["error"] and frames[0]["frame"].crc_ok
    assert data[frames[0]["offset"]:frames[0]["end_offset"]] == data
    assert select(frames, diagnostics)[0] is None  # No claimed partial evidence.
    broken = data.replace(b"crc=A91D891A", b"crc=00000000")
    assert scan(broken)[0][0]["error"] is not None
    assert scan(data[:-1])[0][0]["error"] is not None
    assert scan(data.replace(b"D0,", b"DISRUPTION,x=1\nD0,", 1))[0][0]["error"] is not None
    assert scan(b"ADSDISPOSITION,seq=1,boundedCrc=1")[1][0]["error"] is not None
    try:
        fields("TEST,a=1,a=2")
        raise AssertionError("duplicate field accepted")
    except ValueError:
        pass
    print("CRC_PARTIAL_EXTRACTOR_SELF_TEST,passed=6,scope=fixture_bytes_and_rejection_paths,no_capture_read=1")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, action="append", help="completed immutable .rx.bin; repeat to search in supplied order")
    parser.add_argument("--capture-complete", action="store_true", help="explicitly attest input captures are finished")
    parser.add_argument("--tag", choices=("V", "R", "M"))
    parser.add_argument("--manifest", type=Path, default=ROOT / "tests/fixtures/reference_capture/crc_partial_20261113.manifest.json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if not args.raw or not args.capture_complete:
        parser.error("--raw and --capture-complete required; active captures must not be read")
    manifest_path = args.manifest.resolve()
    if manifest_path.exists():
        raise ValueError("manifest already exists; choose a new name, never overwrite evidence")
    report = {"schema": "sensorarray_recorded_crc_partial_pair_v1",
              "generated_utc": datetime.now(timezone.utc).isoformat(), "status": "NOT_SELECTED",
              "analysis_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "parser": {"module": "tools/text_protocol.py", "class": "TextProtocolParser",
                         "sha256": digest((ROOT / "tools/text_protocol.py").read_bytes())},
              "inputs": [], "frames": [], "selection_rules": {
                  "complete_ascii_newline_records": True, "contiguous_encoder_bytes": True,
                  "same_boot_and_configuration": True, "successor_sequence": "plus_one_modulo_2^32",
                  "diagnostic_scope": "same_reset_epoch_and_sequence",
                  "no_crc_regeneration_or_fragment_repair": True}}
    chosen = None
    for raw_path in args.raw:
        before = raw_path.stat()
        data = raw_path.read_bytes()
        after = raw_path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError("source changed while reading; capture not immutable")
        source = {"path": relative(raw_path), "bytes": len(data), "sha256": digest(data)}
        frames, diagnostics, counts = scan(data)
        pair, rejected = select(frames, diagnostics, args.tag)
        report["inputs"].append({**source, "counts": counts, "rejected_candidates": rejected})
        if pair:
            chosen = (data, source, pair)
            break
    if chosen:
        data, source, (partial, successor, evidence) = chosen
        stem = manifest_path.name.removesuffix(".manifest.json").removesuffix(".json")
        paths = [manifest_path.parent / (stem + suffix) for suffix in (".partial.recorded.txt", ".next.recorded.txt")]
        if any(path.exists() for path in paths):
            raise ValueError("fixture name already exists; choose a new manifest prefix")
        report.update(status="SELECTED", frames=[frame_evidence(f, p, source) for f, p in zip((partial, successor), paths)],
                      disposition_records=[{k: v for k, v in d.items() if k not in {"error", "complete"}} for d in evidence])
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        for frame, path in zip((partial, successor), paths):
            payload = data[frame["offset"]:frame["end_offset"]]
            with path.open("xb") as handle:
                handle.write(payload)
            if path.read_bytes() != payload:
                raise ValueError("written fixture did not preserve source bytes")
    else:
        report["reason"] = "No complete CRC-verified partial frame, matching disposition, and immediate same-configuration successor were available; no bytes were reconstructed."
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with manifest_path.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    print(json.dumps({"status": report["status"], "manifest": relative(manifest_path), "frames": len(report["frames"])}))
    return 0 if chosen else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(json.dumps({"status": "ERROR", "reason": str(error)}), file=sys.stderr)
        raise SystemExit(3)
