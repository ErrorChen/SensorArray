#!/usr/bin/env python3
"""Summarize real voltage-review findings and the largest point excursion.

Reads previously hash-verified derived evidence; no devices or new raw data.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True


def sha(data):
    return hashlib.sha256(data).hexdigest()


def enc(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory", type=Path)
    args = ap.parse_args()
    root = args.directory.resolve()
    sources = {}
    def read(name, lines=False):
        data = (root / name).read_bytes()
        sources[name] = {"bytes": len(data), "sha256": sha(data)}
        return [json.loads(l) for l in data.splitlines()] if lines else json.loads(data)
    manifest = read("manifest.json")
    summary = read("summary.json")
    frames = read("frames.jsonl", True)
    pairs = [p for p in read("adjacent-pairs.jsonl", True) if p["eligible"]]
    diagnostics = read("diagnostics.jsonl", True)
    lags = read("lag101.jsonl", True)
    validation = read("validation.json")
    assert validation["status"] == "PASS" and validation["review_manifest_sha256"] == sources["manifest.json"]["sha256"]
    for name, meta in sources.items():
        if name in manifest["outputs"]:
            assert meta["sha256"] == manifest["outputs"][name]["sha256"]
    biggest = max(((abs(v), i, p) for p in pairs for i, v in enumerate(p["signed_delta_uv"]) if v is not None), key=lambda x: x[0])
    index = biggest[1]
    point = f"S{index // 8 + 1}D{index % 8 + 1}"
    increasing = max((p for p in pairs if p["signed_delta_uv"][index] is not None), key=lambda p: p["signed_delta_uv"][index])
    decreasing = min((p for p in pairs if p["signed_delta_uv"][index] is not None), key=lambda p: p["signed_delta_uv"][index])
    chosen = sorted({p["to_sequence"]: p for p in [increasing, decreasing]}.values(), key=lambda p: p["to_sequence"])
    first = min(p["from_sequence"] for p in chosen) - 2
    last = max(p["to_sequence"] for p in chosen) + 2
    by_seq = {f["sequence"]: f for f in frames if f["trusted"]}
    context = {"selection": "point with largest absolute comparable consecutive delta; that point's largest positive and negative deltas; show two extra sequences at each edge, no interpolation",
               "point": point, "window": {"first_sequence": first, "last_sequence": last},
               "examples": [], "original_point_series": [], "explicit_sequence_diagnostics": [],
               "total_comparable_point_pairs": sum(p["valid_comparable_points"] for p in pairs),
               "comparable_point_count_per_frame_pair": dict(Counter(p["valid_comparable_points"] for p in pairs)),
               "all_lag101_intervals_contiguous": all(p["all_101_steps_observed_consecutively"] for p in lags)}
    for p in chosen:
        a, b = by_seq[p["from_sequence"]], by_seq[p["to_sequence"]]
        context["examples"].append({"from_sequence": p["from_sequence"], "to_sequence": p["to_sequence"],
            "point": point, "before_uv": a["values_uv"][index], "after_uv": b["values_uv"][index],
            "signed_delta_uv": p["signed_delta_uv"][index], "before_gain": a["parsed"]["pga_gains"][index], "after_gain": b["parsed"]["pga_gains"][index],
            "other_points_maximum_abs_delta_uv": max((abs(v) for i, v in enumerate(p["signed_delta_uv"]) if i != index and v is not None), default=None),
            "all_comparable_points_median_delta_uv": p["delta_distribution_uv"]["median"],
            "from_same_sequence_diagnostics": a["same_sequence_diagnostics"], "to_same_sequence_diagnostics": b["same_sequence_diagnostics"],
            "before_source_sha256": a["source_sha256"], "after_source_sha256": b["source_sha256"]})
    for seq in range(first, last + 1):
        f = by_seq.get(seq)
        if f:
            context["original_point_series"].append({"sequence": seq, "value_uv": f["values_uv"][index], "error_reason": f["parsed"]["error_reasons"][index],
                "usable": f["usable_points"][index], "gain": f["parsed"]["pga_gains"][index], "device_timestamp_us": f["device_timestamp_us"],
                "header": f["header"], "source_start": f["source_start"], "source_end_exclusive": f["source_end_exclusive"], "source_sha256": f["source_sha256"]})
    for d in diagnostics:
        token = d["fields"].get("seq", "")
        if token.isdigit() and first <= int(token) <= last:
            context["explicit_sequence_diagnostics"].append(d)
    h = summary["diagnostics"]["details"]["health"]
    delta = h["counter_deltas"]
    checked = delta["crcValidEven"] + delta["crcValidOdd"] + delta["crcErrors"]
    dt, lag = summary["timing"]["adjacent_device_us"], summary["timing"]["lag101_device_us"]
    identity = summary["expected_identity"]
    rows_count = int(summary["configuration"]["rows"])
    candidate_count = summary["pair_screen"]["candidate_pairs"]
    all64_count = summary["pair_screen"]["all64_pairs"]
    any_large = summary["pair_screen"]["any_large_delta_pairs"]
    invalid_cells = {name: value["invalid_reasons"] for name, value in summary["per_point"].items() if value["invalid_reasons"]}
    battery = summary["diagnostics"]["details"]["battery"]
    precision = summary["diagnostics"]["details"]["precision_aggregate"]
    local_diagnostic_summary = [{"seq": d["fields"].get("seq"), "tag": d["tag"], "txn": d["fields"].get("txn"),
                                 "err": d["fields"].get("err", d["fields"].get("originalError")),
                                 "cleanupError": d["fields"].get("cleanupError"), "precisionFrame": d["fields"].get("precisionFrame")}
                                for d in context["explicit_sequence_diagnostics"] if d["tag"] in {"ADSHEALTH", "BATRAW"}]
    lines = [f"# VOLT{rows_count} / E1 有限窗口复核 — Limited recorded-window review", "",
             f"ELF `{identity['elfSha256']}`，session `{identity['session']}` / boot {identity['bootId']}；原始 USB SHA256 `{summary['raw_sha256']}`。", "",
             f"本窗 {summary['integrity']['trusted_frames']} 个可信完整 V 帧，seq {summary['integrity']['first_sequence']}–{summary['integrity']['last_sequence']}；CRC、配置身份及原始 token 独立复算通过。固定规则是同配置相邻序号、两端 valid/fresh/acquired/expected 且无 error，同向变化至少18mV的点达到可比点一半（至少2点）。{context['total_comparable_point_pairs']:,}个可比点对中，有任意单点变化达到18mV的相邻帧对为{any_large}；多点候选{candidate_count}，全64点候选{all64_count}。保留 {summary['integrity']['invalid_token_points']} 个 invalid token：`{invalid_cells}`。不能跨 invalid 插值或宣称整板所有时刻稳定。", "",
             f"每帧 rail/电源值分布：`{summary['rails']['header_profiles']}`；与 ARL external 值匹配 {summary['rails']['trusted_headers_matching_external_values']}帧。相邻 age 差分布 `{summary['rails']['age_deltas']}`，rail值变化{summary['rails']['changed_pairs']}对，年龄回退{summary['rails']['age_decrease_pairs']}对。帧未单独编码 external 来源；ARL 的 raw=0/mon=0 不代表测得零，也不等于连续物理电源监测。", "",
             f"设备相邻帧间隔 {dt['min']/1000:.3f}–{dt['max']/1000:.3f}ms，中位 {dt['median']/1000:.3f}ms；每101序号的设备时间为 {lag['min']/1e6:.6f}–{lag['max']/1e6:.6f}s，中位 {lag['median']/1e6:.6f}s（{lag['n']}个重叠完整区间）。不能称固定5秒周期或逐点采样间隔。", "",
             f"可见 {h['observed']} 条 ADSHEALTH，声明interval `{h['declared_intervals']}`、err `{h['error_values']}`，独立健康窗 CRC 拒绝 {delta['crcErrors']}/{checked:,}；{battery['raw_triple_records']}笔 BATRAW，真实三样本完整性{battery['all_have_three_codes']}，原始/cleanup错误 `{battery['original_errors']}` / `{battery['cleanup_errors']}`，INTREF `{battery['register_INTREF']}`。ADS50为{precision['ADS50_records']}条聚合（n分布`{precision['n_values']}`），不能恢复完整逐帧precision状态；只有对应ADSHEALTH明确给出precisionFrame。没有矩阵各点逐次raw ADC或同步模拟波形。", "",
             "局部样例 / Local examples:", ""]
    for e in context["examples"]:
        lines.append(f"- {point} seq {e['from_sequence']}→{e['to_sequence']}: {e['before_uv']}→{e['after_uv']}µV，Δ{e['signed_delta_uv']:+d}µV；PGA字段{e['before_gain']}→{e['after_gain']}；其余点最大|Δ| {e['other_points_maximum_abs_delta_uv']}µV，全点中位Δ {e['all_comparable_points_median_delta_uv']}µV。")
    lines += ["", f"局部区间的可见序号诊断：`{local_diagnostic_summary}`。这是序号邻近关联，不能证明因果；被动10k/33pF夹具其余60点未占用。不能宣布E1唯一根因、绝对精度或所有工况已解决。", "",
              f"The closed window contains {summary['integrity']['trusted_frames']} CRC/identity-valid V frames. Independent checks preserve every signed integer and Xhh token. Across {context['total_comparable_point_pairs']:,} comparable consecutive point pairs, {any_large} frame pairs contain a change reaching18mV; the half-of-available-points screen flags{candidate_count}, and the separate all64 screen flags{all64_count}. The {summary['integrity']['invalid_token_points']} invalid tokens remain unchanged. Comparable-point counts per adjacent pair are `{context['comparable_point_count_per_frame_pair']}`; missing/invalid intervals are never interpolated.", "",
              f"Header rail-value changes: {summary['rails']['changed_pairs']}; age decreases: {summary['rails']['age_decrease_pairs']}; external-query value matches: {summary['rails']['trusted_headers_matching_external_values']}. This is calibration provenance, not continuous measured supply verification. Frame-to-frame device timing is{dt['min']/1000:.3f}–{dt['max']/1000:.3f}ms;101 sequence increments span{lag['min']/1e6:.6f}–{lag['max']/1e6:.6f}s, not a fixed five-second period. Per-cell sample timing is unavailable.", "",
              f"The signed local examples above preserve exact before/after values and the maximum change among all other points. The nearby explicit-sequence diagnostics are retained in local-context.json. These are temporal associations, not causal proof. The{h['observed']} health snapshots show{delta['crcErrors']}/{checked:,} checked-response CRC rejections over their own bracket. Battery raw triples and aggregate precision logs exist; matrix-cell raw ADC codes and simultaneous analogue waveforms do not. The passive fixture has no known stable voltage stimulus, and the original E1 complete raw recording is absent. This window does not establish absolute precision, a unique physical cause, or universal repair.", "",
              "复算 / Reproduce: run the command in `manifest.json`, then `validate_voltage_frame_review.py <output> --write-result` and `summarize_voltage_review_context.py <output>`. Each refuses overwriting its result. `context-manifest.json` binds this supplement to the original review and validation hashes.", ""]
    outputs = {"local-context.json": enc(context), "findings.md": ("\n".join(lines)).encode()}
    result_manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "invocation": [sys.executable, *sys.argv],
                       "generator_sha256": sha(Path(__file__).read_bytes()), "inputs": sources,
                       "outputs": {name: {"bytes": len(data), "sha256": sha(data)} for name, data in outputs.items()}, "hardware_actions": False}
    for name, content in {**outputs, "context-manifest.json": enc(result_manifest)}.items():
        with (root / name).open("xb") as output:
            output.write(content)
    print(json.dumps({"status": "COMPLETE", "point": point, "comparisons": context["total_comparable_point_pairs"], "all101_contiguous": context["all_lag101_intervals_contiguous"]}))


if __name__ == "__main__":
    main()
