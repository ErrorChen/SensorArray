#!/usr/bin/env python3
"""Replay closed RES8 bytes; fit S1D1 only, freeze k, then evaluate S8D8.

No device APIs, SDK mutation, NVS access, invented raw data, or global gain.
"""
from __future__ import annotations
from collections import Counter
from datetime import datetime, timezone
from fractions import Fraction
import gzip
import hashlib
import io
import json
from pathlib import Path
import statistics
import sys
import zlib

sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[4]
sys.path.insert(0, str(ROOT / "tools"))
import analyze_firmware_benchmark as benchmark
from reference_fixture import IDENTITY_FIELDS, _identity
from text_protocol import MeasurementFrame, TextProtocolParser, parse_fields


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()


def fraction_record(value):
    return {"numerator": value.numerator, "denominator": value.denominator, "decimal": float(value)}


def distribution(values):
    values = sorted(values)
    if not values:
        return {"n": 0, "mean": None, "min": None, "max": None, "p05": None, "p95": None, "median": None, "population_stddev": None}
    def q(numerator, denominator):
        position = Fraction((len(values) - 1) * numerator, denominator)
        lo = position.numerator // position.denominator
        remainder = position - lo
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * remainder
    mean = sum(values, Fraction()) / len(values)
    return {"n": len(values), "mean": float(mean), "min": float(values[0]), "max": float(values[-1]),
            "p05": float(q(5, 100)), "p95": float(q(95, 100)), "median": float(q(1, 2)),
            "population_stddev": statistics.pstdev([float(v) for v in values]),
            "mean_exact": fraction_record(mean)}


def observed_error(values, nominal):
    errors = [x - nominal for x in values]
    return {"reference": "nominal_assumption_only", "nominal_ohm": nominal,
            "signed_error_ohm": distribution(errors),
            "signed_error_percent": distribution([x / nominal * 100 for x in errors]),
            "max_absolute_error_ohm": float(max(map(abs, errors))) if errors else None,
            "absolute_accuracy": "NOT_ESTABLISHED", "acceptance_limit": None}


def main():
    output_names = ["coefficient.json", "frames.jsonl", "paired-values.jsonl", "results.json", "manifest.json", "report.md"]
    for name in output_names:
        if (OUT / name).exists():
            raise FileExistsError(f"refusing overwrite: {OUT / name}")
    inputs = {}
    def read(path):
        path = path.resolve()
        stored = path.read_bytes()
        raw = gzip.decompress(stored) if path.suffix == ".gz" else stored
        inputs[str(path)] = {"stored_sha256": sha(stored), "stored_bytes": len(stored),
                             "raw_sha256": sha(raw), "raw_bytes": len(raw), "gzip": path.suffix == ".gz"}
        return raw
    plan = json.loads(read(OUT / "analysis-plan.json"))
    selection_path = (OUT / plan["source_selection"]).resolve()
    selection = json.loads(read(selection_path))
    source_raw = read(Path(selection["sources"]["raw"]["path"]))
    source_summary_bytes = read(Path(selection["sources"]["summary"]["path"]))
    source_summary = json.loads(source_summary_bytes)
    events_bytes = read(Path(selection["sources"]["events"]["path"]))
    events = [json.loads(x) for x in events_bytes.splitlines()]
    for key, raw in [("raw", source_raw), ("summary", source_summary_bytes), ("events", events_bytes)]:
        assert sha(raw) == selection["sources"][key]["raw_sha256"], key
    assert sha(source_raw) == source_summary["raw_sha256"] and len(source_raw) == source_summary["rx_bytes"]
    finished = [e for e in events if e.get("type") == "finished"]
    assert len(finished) == 1 and events[-1] == finished[0] and finished[0]["status"] == source_summary["status"]
    source_directory = Path(selection["sources"]["summary"]["path"]).parent
    label = source_directory.name
    command = json.loads(read(source_directory / (label + ".command.json")))
    gate = json.loads(read(source_directory / (label + ".gate.json")))
    assert command["child_termination_known"] and command["summary_sha256"] == sha(source_summary_bytes)
    assert command["gate_status"] == gate["status"]
    assert gate["formal_authority"]["summary"]["sha256"] == sha(source_summary_bytes)
    assert gate["formal_authority"]["raw"]["sha256"] == sha(source_raw)
    assert gate["formal_authority"]["events"]["sha256"] == sha(events_bytes)
    derived = read(Path(selection["outputs"]["raw"]["path"]))
    assert sha(derived) == selection["outputs"]["raw"]["sha256"]
    fixture = json.loads(read(ROOT / "tests/fixtures/reference_10k_33pf.json"))
    config = source_summary["snapshots"]["before"]["CONFIG?"]["fields"]
    assert _identity(config)[1] == "consistent_frame_identity"
    assert config["rows"] == "8" and config["mode"] == "RES"
    contract = benchmark.expected_wire_contract({"mode": "RES", "rows": 8}, config)
    frames, group, reconstruction = [], None, bytearray()
    for item in selection["selected"]:
        piece = source_raw[item["source_start"]:item["source_end_exclusive"]]
        assert sha(piece) == item["source_sha256"]
        assert piece == derived[item["output_start"]:item["output_end_exclusive"]]
        reconstruction.extend(piece)
        parser, parsed, chunk_counts = TextProtocolParser(), [], Counter()
        header, trailer, trailer_wire = {}, {}, None
        independent_values, independent_errors, independent_gains, crc_body = [], [], [], bytearray()
        for wire in io.BytesIO(piece):
            line = wire.rstrip(b"\r\n").decode("ascii", "strict")
            tag = line.split(",", 1)[0]
            if tag == "R":
                header = {**parse_fields(line), "tag": tag}
            if tag == "K":
                trailer, trailer_wire = parse_fields(line), wire.decode("ascii")
            if tag in {"R", "D0", "D1", "D2", "D3", "P0", "P1", "P2", "P3"}:
                crc_body.extend((line + "\n").encode("ascii"))
            if tag in {"D0", "D1", "D2", "D3"}:
                chunk_counts[tag] += 1
                for token in line.split(",")[1:]:
                    independent_values.append(None if token.startswith("X") else int(token))
                    independent_errors.append(int(token[1:], 16) if token.startswith("X") else 0)
            if tag in {"P0", "P1", "P2", "P3"}:
                chunk_counts[tag] += 1
                packed = line.split(",", 1)[1]
                independent_gains.extend(int(packed[i:i + 2], 16) for i in range(0, len(packed), 2))
            value = parser.feed_line(line)
            if value is not None:
                parsed.append(value)
        assert len(parsed) == 1 and isinstance(parsed[0], MeasurementFrame) and parsed[0].mode == "RES"
        f = parsed[0]
        assert f.values_fixed == independent_values and f.error_reasons == independent_errors and f.pga_gains == independent_gains
        assert (zlib.crc32(crc_body) == int(trailer["crc"], 16)) == f.crc_ok
        assert dict(chunk_counts) == {f"{tag}{i}": 1 for tag in "DP" for i in range(4)}
        group = group or benchmark.Group(header, f, _identity(config)[0], contract)
        trusted = group.add(f, header, trailer, {}, item["trailer_last_byte_receipt"]["t"], len(piece), len(piece), trailer_wire)
        trusted = bool(trusted and not parser.counters.malformed and header["gen"] == header["mgen"] and header["rid"] == header["mrid"])
        points = {}
        for point, index in [("S1D1", 0), ("S8D8", 63)]:
            bit, value = 1 << index, f.values_fixed[index]
            eligible = bool(trusted and value is not None and value > 0 and f.error_reasons[index] == 0 and
                            all(mask & bit for mask in [f.valid_mask, f.fresh_mask, f.expected_mask, f.acquired_mask]) and not f.error_mask & bit)
            points[point] = {"raw_milliohm": value, "error_reason": f.error_reasons[index], "pga_field": f.pga_gains[index],
                             "eligible": eligible, "valid": bool(f.valid_mask & bit), "fresh": bool(f.fresh_mask & bit),
                             "acquired": bool(f.acquired_mask & bit), "expected": bool(f.expected_mask & bit), "error_bit": bool(f.error_mask & bit)}
        frames.append({"sequence": f.sequence, "device_timestamp_us": f.timestamp_us, "header": header, "trailer": trailer,
                       "source_start": item["source_start"], "source_end_exclusive": item["source_end_exclusive"], "source_sha256": item["source_sha256"],
                       "host_complete_seconds": item["trailer_last_byte_receipt"]["t"], "trusted": trusted, "crc_ok": f.crc_ok,
                       "points": points, "all_cell_invalid_reasons": dict(Counter(f"X{x:02X}" for value, x in zip(f.values_fixed, f.error_reasons) if value is None))})
    assert bytes(reconstruction) == derived
    assert len(frames) == selection["counts"]["selected_parsed_frames"]
    # Fit solely from the predeclared S1D1 values. S8D8 never enters this sum.
    train = [(f["sequence"], f["points"]["S1D1"]["raw_milliohm"]) for f in frames if f["points"]["S1D1"]["eligible"]]
    nominal = int(plan["training_assumed_reference_ohm"])
    k = Fraction(nominal * 1000 * len(train), sum(value for _, value in train))
    coefficient = {"method": plan["coefficient_method"], "training_point": "S1D1", "validation_point_used_in_fit": False,
                   "eligible_n": len(train), "sum_original_milliohm": sum(value for _, value in train),
                   "nominal_reference_ohm": nominal, "reference_traceable_value": None,
                   "factor": fraction_record(k), "training_sequence_value_sha256": sha(encoded(train)),
                   "normalization_only_not_physical_component_estimate": True}
    coefficient_bytes = encoded(coefficient)
    with (OUT / "coefficient.json").open("xb") as output:
        output.write(coefficient_bytes)
    # Re-read the frozen coefficient before evaluating S8D8; no refit or blend.
    frozen = json.loads((OUT / "coefficient.json").read_text())
    frozen_k = Fraction(frozen["factor"]["numerator"], frozen["factor"]["denominator"])
    assert frozen_k == k
    points = {}
    for point in ["S1D1", "S8D8"]:
        eligible = [f for f in frames if f["points"][point]["eligible"]]
        values = [Fraction(f["points"][point]["raw_milliohm"], 1000) for f in eligible]
        normalized = [value * frozen_k for value in values]
        points[point] = {"observed_n": len(frames), "eligible_n": len(values), "excluded_n": len(frames) - len(values),
                         "raw_ohm": distribution(values), "raw_error_vs_nominal": observed_error(values, 10000),
                         "normalized_ohm": distribution(normalized), "normalized_error_vs_nominal": observed_error(normalized, 10000),
                         "role": "training_fit_mean_is_10000_by_construction" if point == "S1D1" else "held_out_channel_at_same_nominal_value",
                         "pga_fields": dict(Counter(f["points"][point]["pga_field"] for f in eligible)),
                         "absolute_accuracy_acceptance": "NEEDS_TRACEABLE_REFERENCE_VALUES_AND_ERROR_LIMITS"}
    paired = []
    for f in frames:
        a, b = f["points"]["S1D1"], f["points"]["S8D8"]
        if not (a["eligible"] and b["eligible"]):
            continue
        ratio = Fraction(b["raw_milliohm"], a["raw_milliohm"])
        na, nb = Fraction(a["raw_milliohm"], 1000) * frozen_k, Fraction(b["raw_milliohm"], 1000) * frozen_k
        paired.append({"sequence": f["sequence"], "source_sha256": f["source_sha256"],
                       "S1D1_raw_milliohm": a["raw_milliohm"], "S8D8_raw_milliohm": b["raw_milliohm"],
                       "S1D1_normalized_ohm": fraction_record(na), "S8D8_normalized_ohm": fraction_record(nb),
                       "S8D8_minus_S1D1_raw_ohm": fraction_record(Fraction(b["raw_milliohm"] - a["raw_milliohm"], 1000)),
                       "S8D8_over_S1D1": fraction_record(ratio)})
    ratios = [Fraction(p["S8D8_over_S1D1"]["numerator"], p["S8D8_over_S1D1"]["denominator"]) for p in paired]
    diffs = [Fraction(p["S8D8_minus_S1D1_raw_ohm"]["numerator"], p["S8D8_minus_S1D1_raw_ohm"]["denominator"]) for p in paired]
    means_ratio = Fraction(points["S8D8"]["raw_ohm"]["mean_exact"]["numerator"], points["S8D8"]["raw_ohm"]["mean_exact"]["denominator"]) / Fraction(points["S1D1"]["raw_ohm"]["mean_exact"]["numerator"], points["S1D1"]["raw_ohm"]["mean_exact"]["denominator"])
    recorded_queries = [line.decode("ascii") for line in source_raw.splitlines() if line.startswith((b"CAL,", b"ARL,"))]
    results = {"schema_version": 1, "status": "FIXTURE_SINGLE_VALUE_NORMALIZATION_CROSS_CHECK_ONLY",
               "identity": gate["expected"], "configuration": config, "frames": len(frames),
               "first_sequence": frames[0]["sequence"], "last_sequence": frames[-1]["sequence"],
               "all_frames_trusted": all(f["trusted"] for f in frames), "source_gate_status": gate["status"],
               "source_raw_sha256": sha(source_raw), "selected_raw_sha256": sha(derived),
               "coefficient": coefficient, "coefficient_sha256": sha(coefficient_bytes), "points": points,
               "paired": {"n": len(paired), "raw_ratio_S8_over_S1": distribution(ratios),
                          "ratio_deviation_ppm": distribution([(r - 1) * 1000000 for r in ratios]),
                          "raw_difference_ohm": distribution(diffs), "ratio_of_raw_means": fraction_record(means_ratio),
                          "semantics": "same-frame pairing, not simultaneous analogue sampling; mean of ratios differs from ratio of means; paired ratios never modify frozen k"},
               "wire_profile": [dict(zip(["ref", "rail", "avdd", "avss", "vexc", "rref"], key), frames=count) for key, count in Counter(tuple(f["header"].get(k) for k in ["ref", "rail", "avdd", "avss", "vexc", "rref"]) for f in frames).items()],
               "recorded_CAL_and_ARL": recorded_queries, "reported_rail_source_note": source_summary.get("rail_source_note"),
               "reference_metadata": fixture["references"], "global_calibration_supported_by_this_evidence": False,
               "uncertainty": {"reference_resistors_actual_values": None, "reference_tolerance": None, "board_R38_R45_actual_values": None,
                              "switch_contact_path_offsets": None, "rail_traceable_uncertainty": None,
                              "absolute_acceptance_limit_ohm": None, "sampling_distribution_is_accuracy_bound": False},
               "validation": {"exact_source_range_roundtrip": True, "real_parser_and_independent_tokens_match": True,
                              "independent_zlib_CRC_matches_parser": True, "factor_uses_only_S1D1": True,
                              "factor_frozen_before_S8D8_statistics": True, "no_outlier_removal_or_invalid_fill": True},
               "hardware_actions": False, "firmware_or_NVS_written": False, "unmeasured_component_values_inferred": False}
    dependencies = [Path(__file__), OUT / "analysis-plan.json", ROOT / "tools/text_protocol.py", ROOT / "tools/reference_fixture.py", ROOT / "tools/analyze_firmware_benchmark.py",
                    ROOT / "core/measure/ads/sensorarrayAdsMath.c", ROOT / "core/measure/ads/sensorarrayAdsMatrix.c", ROOT / "core/measure/ads/sensorarrayAdsMatrix.h",
                    ROOT / "core/measure/ads/sensorarrayAdsGap.c", ROOT / "core/config/sensorarrayCalibration.c", ROOT / "core/config/sensorarrayCalibration.h",
                    ROOT / "main/main.c", ROOT / "main/kconfigs/Kconfig.runtime", ROOT / "sdkconfig", ROOT / "datasheets/circuit_page3-3.png", ROOT / "datasheets/circuit_page4-4.png"]
    dep_meta = [{"path": str(p), "bytes": p.stat().st_size, "sha256": sha(p.read_bytes())} for p in dependencies]
    lines = ["# 10 kΩ 单值归一化交叉检查 / Single-value fixture normalization", "",
             "结论：仅对当前夹具做单值归一化交叉检查，绝对精度及全局增益校准均未建立。 / This is a one-value fixture cross-check; absolute accuracy and a global gain calibration are not established.", "",
             f"{len(frames)} 个原始 RES8 帧，seq {frames[0]['sequence']}–{frames[-1]['sequence']}，两点各 {len(train)} 个有效值。原始 raw SHA256 `{sha(source_raw)}`；ELF `{gate['expected']['elfSha256']}`。",
             f"预定 S1D1 单独拟合：k = 10000 / mean(S1D1) = **{float(k):.12f}**，精确分数 `{k.numerator}/{k.denominator}`。S8D8未进入拟合；之后只乘此固定k。S1D1归一化均值10000是构造结果，不是验证通过。", "",
             "| 测点 / Point | 角色 / Role | 有效 n | 均值 Ω / Mean | min–max Ω | P05–P95 Ω | 均值相对标称误差 / Mean error |",
             "|---|---|---:|---:|---|---|---|" ]
    for point in ["S1D1", "S8D8"]:
        for label, key in [("原始 / original", "raw_ohm"), ("归一化 / normalized", "normalized_ohm")]:
            d = points[point][key]
            error = d["mean"] - 10000
            lines.append(f"| {point} | {label} | {d['n']} | {d['mean']:.6f} | {d['min']:.6f}–{d['max']:.6f} | {d['p05']:.6f}–{d['p95']:.6f} | {error:+.6f} Ω / {error/100:.6f}% |")
    rat = results["paired"]["raw_ratio_S8_over_S1"]
    dif = results["paired"]["raw_difference_ohm"]
    lines += ["", f"同帧配对比值 S8D8/S1D1：n={rat['n']}，均值 {rat['mean']:.12f}，范围 {rat['min']:.12f}–{rat['max']:.12f}，P05–P95 {rat['p05']:.12f}–{rat['p95']:.12f}。原始配对差均值 {dif['mean']:+.6f} Ω。比值只是诊断，未用于逐帧重标定。", "",
              "原值为整数mΩ；归一化值是单列派生量，原包/CRC/invalid未改。min/max与P05/P95仅描述观察分布，不是参考不确定度、精度界或置信区间。两个标称10000Ω不等于已知实际值；参考实测值、容差、温度与允许误差限仍未知，故不标绝对精度PASS。", "",
              "RES物理公式及接口见 physical-model.md。电阻、开关/接触、ADC/参考/rail误差不能由单个标称值分开识别。不同列具有独立板上参考电阻；本系数不应用到其它阻值、其它点或VOLT，也不推算任何未量器件阻值。", "",
              f"The coefficient is fitted only to S1D1's arithmetic mean, then frozen before evaluating S8D8. At the assumed 10000 Ω nominal, S8D8's normalized mean is {points['S8D8']['normalized_ohm']['mean']:.6f} Ω, a conditional deviation of {points['S8D8']['normalized_ohm']['mean']-10000:+.6f} Ω. This tests consistency between two paths at one nominal value; it does not identify a transferable gain, offset, actual reference component value, or voltage correction. No firmware, NVS, calibration command or raw scientific data was changed.", "",
              "复算 / Reproduction: execute `python3 analyse.py` in a fresh copy of this directory containing only the script, analysis-plan.json and physical-model.md, or inspect the recorded invocation. Existing outputs are never overwritten. The manifest binds source selection, original byte ranges, parser, production-source and schematic hashes.", ""]
    outputs = {"frames.jsonl": b"".join((json.dumps(f, ensure_ascii=False, separators=(",", ":")) + "\n").encode() for f in frames),
               "paired-values.jsonl": b"".join((json.dumps(p, separators=(",", ":")) + "\n").encode() for p in paired),
               "results.json": encoded(results), "report.md": ("\n".join(lines) + "\n").encode()}
    for path, metadata in inputs.items():
        assert sha(Path(path).read_bytes()) == metadata["stored_sha256"], "source changed during replay"
    out_manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "invocation": [sys.executable, *sys.argv],
                    "inputs": inputs, "tools_and_production_sources": dep_meta,
                    "coefficient_written_before_held_out_statistics": True,
                    "outputs": {name: {"bytes": len(data), "sha256": sha(data)} for name, data in {"coefficient.json": coefficient_bytes, **outputs}.items()},
                    "source_selection_range_manifest": str(selection_path), "original_raw_modified": False, "hardware_actions": False}
    for name, data in {**outputs, "manifest.json": encoded(out_manifest)}.items():
        with (OUT / name).open("xb") as output:
            output.write(data)
    print(json.dumps({"status": results["status"], "frames": len(frames), "k": float(k),
                      "S8D8_normalized_mean_ohm": points["S8D8"]["normalized_ohm"]["mean"], "absolute_accuracy": "NOT_ESTABLISHED"}))


if __name__ == "__main__":
    main()
