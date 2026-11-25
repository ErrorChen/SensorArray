#!/usr/bin/env python3
"""Independent original-token/CRC and Decimal normalization verification."""
from datetime import datetime, timezone
from decimal import Decimal, getcontext
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import sys
import zlib

sys.dont_write_bytecode = True
getcontext().prec = 60
OUT = Path(__file__).resolve().parent


def sha(data):
    return hashlib.sha256(data).hexdigest()


def fields(line):
    return dict(token.split("=", 1) for token in line.split(",")[1:] if "=" in token)


def check_close(actual, expected, tolerance=Decimal("1e-8")):
    assert abs(Decimal(str(actual)) - expected) <= tolerance, (actual, str(expected))


def stats(values):
    ordered = sorted(values)
    def q(n, d):
        p = Decimal(len(ordered) - 1) * n / d
        i = int(p)
        return ordered[i] + (ordered[min(i + 1, len(ordered) - 1)] - ordered[i]) * (p - i)
    return {"n": len(values), "mean": sum(values) / len(values), "min": ordered[0], "max": ordered[-1],
            "p05": q(5, 100), "p95": q(95, 100), "median": q(1, 2)}


def main():
    target = OUT / "validation.json"
    if target.exists():
        raise FileExistsError(f"refusing overwrite: {target}")
    manifest_bytes = (OUT / "manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    for path, meta in manifest["inputs"].items():
        assert sha(Path(path).read_bytes()) == meta["stored_sha256"], path
    for name, meta in manifest["outputs"].items():
        data = (OUT / name).read_bytes()
        assert sha(data) == meta["sha256"] and len(data) == meta["bytes"], name
    result = json.loads((OUT / "results.json").read_text())
    plan = json.loads((OUT / "analysis-plan.json").read_text())
    selection = json.loads(Path(manifest["source_selection_range_manifest"]).read_text())
    raw = Path(selection["sources"]["raw"]["path"]).read_bytes()
    frames = [json.loads(l) for l in (OUT / "frames.jsonl").read_text().splitlines()]
    paired = [json.loads(l) for l in (OUT / "paired-values.jsonl").read_text().splitlines()]
    observed = {"S1D1": [], "S8D8": []}
    all_point_tokens = invalid_tokens = 0
    for f in frames:
        wire = raw[f["source_start"]:f["source_end_exclusive"]]
        assert sha(wire) == f["source_sha256"]
        body, values, reasons = bytearray(), [], []
        for raw_line in wire.splitlines(keepends=True):
            line = raw_line.rstrip(b"\r\n").decode("ascii")
            tag = line.split(",", 1)[0]
            if tag in {"R", "D0", "D1", "D2", "D3", "P0", "P1", "P2", "P3"}:
                body.extend((line + "\n").encode())
            if tag in {"D0", "D1", "D2", "D3"}:
                for token in line.split(",")[1:]:
                    values.append(None if token.startswith("X") else int(token))
                    reasons.append(int(token[1:], 16) if token.startswith("X") else 0)
            if tag == "R":
                header = fields(line)
            if tag == "K":
                trailer = fields(line)
        assert len(values) == 64 and zlib.crc32(body) == int(trailer["crc"], 16)
        assert {**header, "tag": "R"} == f["header"] and trailer == f["trailer"]
        all_point_tokens += len(values)
        invalid_tokens += sum(v is None for v in values)
        for point, index in [("S1D1", 0), ("S8D8", 63)]:
            bit = 1 << index
            assert values[index] == f["points"][point]["raw_milliohm"] and reasons[index] == f["points"][point]["error_reason"]
            eligible = f["trusted"] and values[index] is not None and values[index] > 0 and reasons[index] == 0 and all(int(header[k], 16) & bit for k in ["valid", "fresh", "expected", "acquired"]) and not int(header["error"], 16) & bit
            assert bool(eligible) == f["points"][point]["eligible"]
            if eligible:
                observed[point].append(Decimal(values[index]) / 1000)
    # Separate implementation: Decimal, original S1D1 integer sum, no S8D8 input.
    frozen = json.loads((OUT / "coefficient.json").read_text())
    original_sum = sum(f["points"]["S1D1"]["raw_milliohm"] for f in frames if f["points"]["S1D1"]["eligible"])
    exact_k = Fraction(plan["training_assumed_reference_ohm"] * 1000 * len(observed["S1D1"]), original_sum)
    assert original_sum == frozen["sum_original_milliohm"]
    assert (exact_k.numerator, exact_k.denominator) == (frozen["factor"]["numerator"], frozen["factor"]["denominator"])
    coefficient = Decimal(10000) / (sum(observed["S1D1"]) / len(observed["S1D1"]))
    check_close(frozen["factor"]["decimal"], coefficient, Decimal("1e-15"))
    for point in ["S1D1", "S8D8"]:
        for key, values in [("raw_ohm", observed[point]), ("normalized_ohm", [v * coefficient for v in observed[point]])]:
            expected = stats(values)
            reported = result["points"][point][key]
            assert reported["n"] == expected["n"]
            for field in ["mean", "min", "max", "p05", "p95", "median"]:
                check_close(reported[field], expected[field])
    ratios, differences = [], []
    by_seq = {f["sequence"]: f for f in frames}
    for p in paired:
        f = by_seq[p["sequence"]]
        a, b = f["points"]["S1D1"]["raw_milliohm"], f["points"]["S8D8"]["raw_milliohm"]
        assert (a, b) == (p["S1D1_raw_milliohm"], p["S8D8_raw_milliohm"])
        r = p["S8D8_over_S1D1"]
        assert Fraction(b, a) == Fraction(r["numerator"], r["denominator"])
        ratios.append(Decimal(b) / a)
        differences.append(Decimal(b - a) / 1000)
        for point, value in [("S1D1", a), ("S8D8", b)]:
            normalized = p[point + "_normalized_ohm"]
            assert Fraction(value, 1000) * exact_k == Fraction(normalized["numerator"], normalized["denominator"])
    for key, data, tolerance in [("raw_ratio_S8_over_S1", ratios, Decimal("1e-14")), ("raw_difference_ohm", differences, Decimal("1e-10"))]:
        for field, expected in stats(data).items():
            check_close(result["paired"][key][field], Decimal(expected), tolerance)
    validation = {"status": "PASS_REPLAY_AND_ARITHMETIC_ONLY", "absolute_accuracy": "NOT_ESTABLISHED",
                  "created_utc": datetime.now(timezone.utc).isoformat(), "invocation": [sys.executable, *sys.argv],
                  "validator_sha256": sha(Path(__file__).read_bytes()), "analysis_manifest_sha256": sha(manifest_bytes),
                  "plan_sha256": sha((OUT / "analysis-plan.json").read_bytes()),
                  "physical_model_report_sha256": sha((OUT / "physical-model.md").read_bytes()),
                  "all_input_and_output_hashes_match": True, "frame_count": len(frames), "independent_CRC_count": len(frames),
                  "original_point_tokens": all_point_tokens, "original_invalid_tokens_retained": invalid_tokens,
                  "reference_valid_n": {p: len(v) for p, v in observed.items()}, "paired_n": len(paired),
                  "S1_only_coefficient_independently_recomputed": True, "S8_held_out_mean_range_quantiles_recomputed_Decimal60": True,
                  "paired_ratios_and_normalized_values_exact_fraction_verified": True,
                  "firmware_or_NVS_write": False, "raw_modified": False, "hardware_actions": False}
    with target.open("x") as stream:
        json.dump(validation, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(validation, ensure_ascii=False))


if __name__ == "__main__":
    main()
