#!/usr/bin/env python3
"""Independent byte/token/CRC/arithmetic check of a closed voltage review."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys
import zlib

sys.dont_write_bytecode = True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def fields(line):
    return dict(token.split("=", 1) for token in line.split(",")[1:] if "=" in token)


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory", type=Path)
    ap.add_argument("--write-result", action="store_true")
    args = ap.parse_args()
    root = args.directory.resolve()
    manifest_bytes = (root / "manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    source = {}
    for key, meta in manifest["sources"].items():
        stored = Path(meta["path"]).read_bytes()
        assert sha(stored) == meta["stored_sha256"], key
        data = gzip.decompress(stored) if meta["gzip"] else stored
        assert sha(data) == meta["raw_sha256"] and len(data) == meta["raw_bytes"], key
        source[key] = data
    for name, meta in manifest["outputs"].items():
        data = (root / name).read_bytes()
        assert sha(data) == meta["sha256"] and len(data) == meta["bytes"], name
    selected = (root / "selected.rx.bin").read_bytes()
    rebuilt = bytearray()
    for record in manifest["selected"]:
        piece = source["formal_raw"][record["source_start"]:record["source_end_exclusive"]]
        assert sha(piece) == record["source_sha256"]
        assert piece == selected[record["output_start"]:record["output_end_exclusive"]]
        rebuilt.extend(piece)
    assert bytes(rebuilt) == selected
    frames = read_jsonl(root / "frames.jsonl")
    point_values, by_source = 0, {}
    for f in frames:
        piece = source["formal_raw"][f["source_start"]:f["source_end_exclusive"]]
        assert sha(piece) == f["source_sha256"]
        numeric, reasons, gains, body = [], [], [], bytearray()
        for wire in piece.splitlines(keepends=True):
            line = wire.rstrip(b"\r\n").decode("ascii")
            tag = line.split(",", 1)[0]
            if tag == "V":
                header = fields(line)
            elif tag == "K":
                trailer = fields(line)
            if tag == "V" or re.fullmatch(r"[DP][0-3]", tag):
                body.extend((line + "\n").encode("ascii"))
            if re.fullmatch(r"D[0-3]", tag):
                for token in line.split(",")[1:]:
                    numeric.append(None if token.startswith("X") else int(token))
                    reasons.append(int(token[1:], 16) if token.startswith("X") else 0)
            if re.fullmatch(r"P[0-3]", tag):
                packed = line.split(",", 1)[1]
                gains.extend(int(packed[i:i + 2], 16) for i in range(0, len(packed), 2))
        if "parsed" not in f:
            continue
        assert numeric == f["values_uv"] and numeric == f["parsed"]["values_fixed"]
        assert reasons == f["parsed"]["error_reasons"] and gains == f["parsed"]["pga_gains"]
        assert (zlib.crc32(body) == int(trailer["crc"], 16)) == f["parsed"]["crc_ok"]
        assert f["header"] == {**header, "tag": "V"} and f["trailer"] == trailer
        assert len(numeric) == int(header["cells"])
        for i, value in enumerate(numeric):
            bit = 1 << i
            usable = value is not None and reasons[i] == 0 and all(int(header[k], 16) & bit for k in ["valid", "fresh", "expected", "acquired"]) and not int(header["error"], 16) & bit
            assert bool(usable) == f["usable_points"][i]
        point_values += len(numeric)
        by_source[f["source_start"]] = f
    pairs = read_jsonl(root / "adjacent-pairs.jsonl")
    eligible, candidates, all64 = 0, 0, 0
    for p in pairs:
        if not p["eligible"]:
            continue
        a, b = by_source[p["from_source_start"]], by_source[p["to_source_start"]]
        expected = [v - u if a["usable_points"][i] and b["usable_points"][i] else None for i, (u, v) in enumerate(zip(a["values_uv"], b["values_uv"]))]
        assert expected == p["signed_delta_uv"]
        values = [v for v in expected if v is not None]
        ups, downs = sum(v >= 18000 for v in values), sum(v <= -18000 for v in values)
        candidate = max(ups, downs) >= 2 and max(ups, downs) * 2 >= len(values)
        board = len(values) == 64 and max(ups, downs) == 64
        assert (ups, downs, candidate, board) == (p["up_ge_18mV"], p["down_le_minus18mV"], p["same_direction_ge18mV_candidate"], p["all64_points_same_direction_ge18mV"])
        assert p["device_delta_us"] == b["device_timestamp_us"] - a["device_timestamp_us"]
        eligible += 1
        candidates += int(candidate)
        all64 += int(board)
    diagnostics = read_jsonl(root / "diagnostics.jsonl")
    for d in diagnostics:
        wire = source[d["source"]][d["source_start"]:d["source_end_exclusive"]]
        assert sha(wire) == d["sha256"]
        assert wire.rstrip(b"\r\n").decode("ascii", "surrogateescape") == d["line"]
    summary = json.loads((root / "summary.json").read_text())
    assert summary["pair_screen"]["eligible_pairs"] == eligible
    assert summary["pair_screen"]["candidate_pairs"] == candidates
    assert summary["pair_screen"]["all64_pairs"] == all64
    result = {"status": "PASS", "evidence_kind": "independent_recalculation_of_recorded_bytes_no_synthetic_raw",
              "created_utc": datetime.now(timezone.utc).isoformat(), "invocation": [sys.executable, *sys.argv],
              "validator_sha256": sha(Path(__file__).read_bytes()), "review_manifest_sha256": sha(manifest_bytes),
              "source_hashes": "all_match", "output_hashes": "all_match", "selected_ranges_round_trip": True,
              "frame_count": len(frames), "point_values_and_Xhh_verified": point_values, "CRC_independently_recomputed": True,
              "adjacent_pairs_independently_recomputed": eligible, "candidate_count": candidates, "all64_count": all64,
              "diagnostic_lines_byte_verified": len(diagnostics), "hardware_actions": False}
    if args.write_result:
        with (root / "validation.json").open("x") as stream:
            json.dump(result, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
