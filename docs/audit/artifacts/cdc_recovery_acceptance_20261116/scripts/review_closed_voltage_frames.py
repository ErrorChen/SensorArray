#!/usr/bin/env python3
"""Audit an already closed formal-USB VOLT recording; no device access.

Preserves original signed microvolts and Xhh invalids. The fixed 18 mV
candidate screen is descriptive, not a new firmware or acceptance threshold.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import statistics
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
import analyze_firmware_benchmark as benchmark
from reference_fixture import IDENTITY_FIELDS, _identity
from text_protocol import MeasurementFrame, TextProtocolParser, parse_fields

HELPER = Path(__file__).with_name("analyse_closed_fixture_window.py")
spec = importlib.util.spec_from_file_location("closed_window", HELPER)
closed = importlib.util.module_from_spec(spec)
spec.loader.exec_module(closed)
THRESHOLD_UV = 18_000


def sha(value):
    return hashlib.sha256(value).hexdigest()


def encode(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()


def jsonl(values):
    return b"".join((json.dumps(v, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n").encode() for v in values)


def read_source(path):
    return closed.read_source(path)


def find(directory, label, suffix):
    return closed.existing(directory, label, suffix)


def distribution(values):
    values = sorted(values)
    if not values:
        return {"n": 0, "min": None, "max": None, "mean": None, "median": None,
                "p05": None, "p95": None, "population_stddev": None}
    def quantile(q):
        x = (len(values) - 1) * q
        lo = int(x)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (x - lo)
    return {"n": len(values), "min": values[0], "max": values[-1],
            "mean": statistics.fmean(values), "median": statistics.median(values),
            "p05": quantile(.05), "p95": quantile(.95),
            "population_stddev": statistics.pstdev(values)}


def point_name(index):
    return f"S{index // 8 + 1}D{index % 8 + 1}"


def assert_closed(summary, events, raw, debug=False):
    if not summary.get("raw_sha256") or summary["raw_sha256"] != sha(raw) or summary.get("rx_bytes") != len(raw):
        raise ValueError("finalized source raw SHA/byte mismatch")
    ends = [e for e in events if e.get("type") == "finished"]
    if len(ends) != 1 or events[-1] != ends[0] or ends[0].get("status") != summary.get("status"):
        raise ValueError("source lacks a unique matching final finished event")
    cursor, last_time = 0, None
    for e in events:
        if e.get("type") != "rx":
            continue
        t = e.get("monotonic") if debug else e.get("t")
        if e.get("offset") != cursor or not isinstance(e.get("length"), int) or e["length"] <= 0:
            raise ValueError("source RX offsets are not contiguous")
        if not isinstance(t, (int, float)) or not math.isfinite(t) or (last_time is not None and t < last_time):
            raise ValueError("source RX times invalid or regressed")
        last_time, cursor = t, cursor + e["length"]
    if cursor != len(raw):
        raise ValueError("RX events do not cover complete raw")


def diagnostic_lines(raw, events, source, absolute_origin, absolute_window):
    rx = [e for e in events if e.get("type") == "rx"]
    ends = [e["offset"] + e["length"] for e in rx]
    result, offset, tag_counts = [], 0, Counter()
    for wire in io.BytesIO(raw):
        start, offset = offset, offset + len(wire)
        if not wire.endswith(b"\n"):
            continue
        line = wire.rstrip(b"\r\n").decode("ascii", "surrogateescape")
        tag = line.split(",", 1)[0]
        tag_counts[tag] += 1
        if not (tag.startswith(("ADS", "BAT", "RAIL", "PRECISION", "ABAT", "AB50", "ARL")) or
                tag in {"MFAULT", "ADSFAULTCTX", "RST", "RESET"} or
                any(token in line for token in ["RAIL_REFRESH", "RAILCRC", "ADSCRC", "ADSFAULT", "MATRIX_CRC"])):
            continue
        e = rx[bisect_right(ends, offset - 1)]
        when = e.get("monotonic", absolute_origin + e["t"])
        result.append({"source": source, "source_start": start, "source_end_exclusive": offset,
                       "sha256": sha(wire), "line": line, "tag": tag, "fields": parse_fields(line),
                       "host_receipt_monotonic": when,
                       "inside_formal_host_window": absolute_window[0] <= when <= absolute_window[1],
                       "time_semantics": "host receipt of final line byte, not a physical edge"})
    return result, dict(tag_counts)


def analyze_frame(piece, selected, config, contract, groups):
    parser = TextProtocolParser()
    returned, chunks = [], Counter()
    header = {**selected["header"], "tag": selected["wire_tag"]}
    trailer, trailer_wire = {}, None
    for wire in io.BytesIO(piece):
        line = wire.rstrip(b"\r\n").decode("ascii", "strict")
        tag = line.split(",", 1)[0]
        if (tag.startswith(("D", "P")) and tag[1:].isdigit() and int(tag[1:]) <= 3):
            chunks[tag] += 1
        if tag == "K":
            trailer, trailer_wire = parse_fields(line), wire.decode("ascii")
        frame = parser.feed_line(line)
        if frame is not None:
            returned.append(frame)
    base = {"source_start": selected["source_start"], "source_end_exclusive": selected["source_end_exclusive"],
            "source_sha256": selected["source_sha256"], "header": header, "trailer": trailer,
            "header_receipt_seconds": selected["header_first_byte_receipt"]["t"],
            "complete_receipt_seconds": selected["trailer_last_byte_receipt"]["t"],
            "parser_counters": asdict(parser.counters)}
    if len(returned) != 1 or not isinstance(returned[0], MeasurementFrame) or returned[0].mode != "VOLT":
        return {**base, "trusted": False, "rejection": "not_one_complete_VOLT_frame"}
    frame = returned[0]
    identity, identity_status = _identity(header)
    key = tuple(identity.get(k) for k in IDENTITY_FIELDS)
    group = groups.setdefault(key, benchmark.Group(header, frame, _identity(config)[0], contract))
    trusted = group.add(frame, header, trailer, {}, base["complete_receipt_seconds"], len(piece), len(piece), trailer_wire)
    expected_chunks = {f"{tag}{i}": 1 for tag in "DP" for i in range((frame.cells + 15) // 16)}
    chunk_structure_ok = dict(chunks) == expected_chunks
    legacy_mode_domain_ok = header.get("gen") == header.get("mgen") and header.get("rid") == header.get("mrid")
    trusted = bool(trusted and chunk_structure_ok and legacy_mode_domain_ok and not parser.counters.malformed)
    usable = [bool(frame.values_fixed[i] is not None and frame.error_reasons[i] == 0 and
                   all(mask & (1 << i) for mask in [frame.valid_mask, frame.fresh_mask, frame.expected_mask, frame.acquired_mask]) and
                   not frame.error_mask & (1 << i)) for i in range(frame.cells)]
    return {**base, "trusted": trusted, "identity": identity, "identity_status": identity_status,
            "chunk_structure_ok": chunk_structure_ok, "legacy_MODE_domain_ok": legacy_mode_domain_ok,
            "sequence": frame.sequence, "device_timestamp_us": frame.timestamp_us,
            "parsed": asdict(frame), "values_uv": frame.values_fixed,
            "usable_points": usable, "usable_count": sum(usable),
            "value_semantics": "original signed D-token integers in microvolts (scale=-6); null is an original Xhh, never zero"}


def analyze_pairs(frames):
    pairs, lags = [], []
    for a, b in zip(frames, frames[1:]):
        same = all(a.get("identity", {}).get(k) == b.get("identity", {}).get(k) for k in IDENTITY_FIELDS)
        consecutive = "sequence" in a and "sequence" in b and (b["sequence"] - a["sequence"]) & 0xFFFFFFFF == 1
        if not (a["trusted"] and b["trusted"] and same and consecutive):
            pairs.append({"from_sequence": a.get("sequence"), "to_sequence": b.get("sequence"), "eligible": False,
                          "same_configuration": same, "consecutive_sequence": consecutive,
                          "both_trusted": a["trusted"] and b["trusted"]})
            continue
        deltas = [y - x if a["usable_points"][i] and b["usable_points"][i] else None
                  for i, (x, y) in enumerate(zip(a["values_uv"], b["values_uv"]))]
        available = [v for v in deltas if v is not None]
        up = [i for i, v in enumerate(deltas) if v is not None and v >= THRESHOLD_UV]
        down = [i for i, v in enumerate(deltas) if v is not None and v <= -THRESHOLD_UV]
        dominant = up if len(up) >= len(down) else down
        direction = "up" if len(up) >= len(down) else "down"
        candidate = len(dominant) >= 2 and 2 * len(dominant) >= len(available)
        gains_changed = [i for i, (x, y) in enumerate(zip(a["parsed"]["pga_gains"], b["parsed"]["pga_gains"])) if x != y]
        pairs.append({"from_sequence": a["sequence"], "to_sequence": b["sequence"], "eligible": True,
                      "from_source_start": a["source_start"], "to_source_start": b["source_start"],
                      "device_delta_us": b["device_timestamp_us"] - a["device_timestamp_us"],
                      "host_complete_delta_seconds": b["complete_receipt_seconds"] - a["complete_receipt_seconds"],
                      "valid_comparable_points": len(available), "signed_delta_uv": deltas,
                      "delta_distribution_uv": distribution(available), "up_ge_18mV": len(up), "down_le_minus18mV": len(down),
                      "same_direction_ge18mV_candidate": candidate, "candidate_direction": direction if candidate else None,
                      "dominant_points": [point_name(i) for i in dominant],
                      "dominant_rows": sorted({i // 8 + 1 for i in dominant}),
                      "dominant_columns": sorted({i % 8 + 1 for i in dominant}),
                      "all_active_points_same_direction_ge18mV": len(available) == len(deltas) and len(dominant) == len(deltas),
                      "all64_points_same_direction_ge18mV": len(available) == 64 and len(dominant) == 64,
                      "dominant_18_to_19mV_points": [point_name(i) for i in dominant if abs(deltas[i]) <= 19000],
                      "pga_changed_points": [point_name(i) for i in gains_changed],
                      "large_delta_with_pga_change": [point_name(i) for i in dominant if i in gains_changed],
                      "rail_values_changed": any(a["header"].get(k) != b["header"].get(k) for k in ["ref", "rail", "avdd", "avss"]),
                      "rail_age_delta_frames": int(b["header"]["age"]) - int(a["header"]["age"])})
    bad_prefix = [0]
    for p in pairs:
        bad_prefix.append(bad_prefix[-1] + int(not p["eligible"]))
    by_key = {}
    for index, f in enumerate(frames):
        if not f["trusted"]:
            continue
        key = tuple(f["identity"].get(k) for k in IDENTITY_FIELDS)
        match = by_key.get((key, (f["sequence"] - 101) & 0xFFFFFFFF))
        if match:
            previous, previous_index = match
            lags.append({"from_sequence": previous["sequence"], "to_sequence": f["sequence"], "sequence_distance": 101,
                         "device_delta_us": f["device_timestamp_us"] - previous["device_timestamp_us"],
                         "host_complete_delta_seconds": f["complete_receipt_seconds"] - previous["complete_receipt_seconds"],
                         "observed_sequence_steps": index - previous_index,
                         "all_101_steps_observed_consecutively": index - previous_index == 101 and bad_prefix[index] == bad_prefix[previous_index],
                         "meaning": "observed elapsed time across 101 sequence increments; not a demonstrated event period"})
        by_key[(key, f["sequence"])] = (f, index)
    return pairs, lags


def diagnostic_statistics(diagnostics, pairs):
    inside = [d for d in diagnostics if d["inside_formal_host_window"]]
    by_tag = defaultdict(list)
    for d in inside:
        by_tag[d["tag"]].append(d)
    health, battery = by_tag["ADSHEALTH"], by_tag["BATRAW"]
    pair_by_sequence = {p["to_sequence"]: p for p in pairs if p["eligible"]}
    health_stats = {"observed": len(health), "counter_deltas": {}, "sequence_associated_pairs": []}
    if health:
        health_stats.update(first_diagnostic_id=health[0]["diagnostic_id"], last_diagnostic_id=health[-1]["diagnostic_id"],
                            first_fields=health[0]["fields"], last_fields=health[-1]["fields"],
                            host_observation_span_seconds=health[-1]["host_receipt_monotonic"] - health[0]["host_receipt_monotonic"],
                            error_values=dict(Counter(d["fields"].get("err", "unknown") for d in health)),
                            declared_intervals=dict(Counter(d["fields"].get("healthInterval", "unknown") for d in health)),
                            precisionFrame_values=dict(Counter(d["fields"].get("precisionFrame", "unknown") for d in health)))
        for key in ["crcValidEven", "crcValidOdd", "crcErrors", "crcRetries", "engineFrameCount"]:
            first, last = health[0]["fields"].get(key), health[-1]["fields"].get(key)
            if first is not None and last is not None and int(last) >= int(first):
                health_stats["counter_deltas"][key] = int(last) - int(first)
            else:
                health_stats["counter_deltas"][key] = None
        health_stats["engine_count_intervals"] = distribution([int(b["fields"]["engineFrameCount"]) - int(a["fields"]["engineFrameCount"]) for a, b in zip(health, health[1:])])
        for d in health:
            seq = int(d["fields"]["seq"])
            p = pair_by_sequence.get(seq)
            if p:
                health_stats["sequence_associated_pairs"].append({"diagnostic_id": d["diagnostic_id"], "sequence": seq,
                    "max_abs_delta_uv": max((abs(v) for v in p["signed_delta_uv"] if v is not None), default=None),
                    "multi_point_candidate": p["same_direction_ge18mV_candidate"]})
    raw_triples, sample_intervals, txn_durations = [], [], []
    for d in battery:
        f = d["fields"]
        values = [int(x) for x in f.get("raw", "").split("/") if x]
        times = [int(x) for x in f.get("timeUs", "").split("/") if x]
        raw_triples.append({"diagnostic_id": d["diagnostic_id"], "sequence": f.get("seq"), "transaction": f.get("txn"),
                            "raw_codes": values, "device_sample_times_us": times, "status_tokens": f.get("status"),
                            "originalError": f.get("originalError"), "cleanupError": f.get("cleanupError")})
        sample_intervals.extend(y - x for x, y in zip(times, times[1:]))
        if "transactionStartUs" in f and "transactionEndUs" in f:
            txn_durations.append(int(f["transactionEndUs"]) - int(f["transactionStartUs"]))
    ads50, registers, ab50 = by_tag["ADS50"], by_tag["BATREG"], by_tag["AB50"]
    return {"health": health_stats,
            "battery": {"raw_triple_records": len(battery), "all_have_three_codes": all(len(x["raw_codes"]) == 3 for x in raw_triples) if raw_triples else None,
                        "first": raw_triples[0] if raw_triples else None, "last": raw_triples[-1] if raw_triples else None,
                        "original_errors": dict(Counter(x["originalError"] for x in raw_triples)), "cleanup_errors": dict(Counter(x["cleanupError"] for x in raw_triples)),
                        "sample_interval_us": distribution(sample_intervals), "transaction_duration_us": distribution(txn_durations),
                        "register_phases": dict(Counter(d["fields"].get("phase", "unknown") for d in registers)),
                        "register_known": dict(Counter(d["fields"].get("known", "unknown") for d in registers)),
                        "register_INTREF": dict(Counter(d["fields"].get("INTREF", "unknown") for d in registers)),
                        "register_INTERFACE": dict(Counter(d["fields"].get("INTERFACE", "unknown") for d in registers)),
                        "AB50_valid": dict(Counter(d["fields"].get("valid", "unknown") for d in ab50)),
                        "AB50_rail_values": dict(Counter(d["fields"].get("rail", "unknown") for d in ab50)),
                        "AB50_lastGoodFrame_first_last": [ab50[0]["fields"].get("lastGoodFrame"), ab50[-1]["fields"].get("lastGoodFrame")] if ab50 else None},
            "precision_aggregate": {"ADS50_records": len(ads50), "n_values": dict(Counter(d["fields"].get("n", "unknown") for d in ads50)),
                                    "precisionFrames_values": dict(Counter(d["fields"].get("precisionFrames", "unknown") for d in ads50)),
                                    "records_with_sequence": sum("seq" in d["fields"] for d in ads50),
                                    "per_frame_precision_state": "only explicit ADSHEALTH precisionFrame for its logged seq; ADS50 is aggregate, not a complete per-frame map"},
            "rail_refresh_record_count": sum("RAIL_REFRESH" in d["line"] for d in inside),
            "raw_matrix_ADC_codes": "NOT_REPORTED_AS_PER_CELL_CODES_BY_FORMAL_VOLT_PROTOCOL; BATRAW is battery only"}


def report_text(stats):
    integrity, pair = stats["integrity"], stats["pair_screen"]
    dt, lag = stats["timing"]["adjacent_device_us"], stats["timing"]["lag101_device_us"]
    rows = ["# VOLT8 原始帧复核 / Recorded VOLT frame review", "",
            f"状态 / Status: **{stats['status']}**。仅为本次被动夹具的有限观察；绝对电压精度、原 E1 因果和全场景修复均未验收。 / This finite passive-fixture observation does not establish absolute voltage accuracy, the original E1 cause, or all-condition repair.", "",
            f"正式 USB 原始 SHA-256 / Formal raw SHA-256: `{stats['raw_sha256']}`。",
            f"{integrity['trusted_frames']} trusted / {integrity['selected_candidates']} selected complete candidates; seq {integrity['first_sequence']}–{integrity['last_sequence']}; identity `{stats['expected_identity']}`.",
            f"invalid 点 / points: {integrity['invalid_token_points']}; 非连续或不可信相邻对 / rejected adjacent pairs: {pair['ineligible_pairs']}.", "",
            "固定观察规则 / Fixed descriptive rule: 同配置、seq+1、两帧 CRC/身份/结构可信；仅比较两端 valid/fresh/expected/acquired 且无 error 的点。至少两个点同向变化 ≥18,000 µV，且占可比点至少一半，记为候选。另列完整64点同向变化，不能把半数或少数点称为全板跃升。 / Require consecutive trustworthy frames and usable points at both endpoints; at least two same-direction changes ≥18 mV covering at least half the comparable points flag a candidate. An all-64-point condition is reported separately.", "",
            f"可比相邻对 / Eligible adjacent pairs: {pair['eligible_pairs']}; 多点候选 / candidates: **{pair['candidate_pairs']}**; 全64点同向 ≥18 mV / all-64 candidates: **{pair['all64_pairs']}**; 至少一离群变化 ≥18 mV / any-point pairs: {pair['any_large_delta_pairs']}.",
            f"相邻设备帧间隔 min/median/max / adjacent device interval (µs): {dt['min']} / {dt['median']} / {dt['max']}。",
            f"101 序号间隔 min/median/max / 101-sequence device interval (µs): {lag['min']} / {lag['median']} / {lag['max']}，n={lag['n']}；主机收件时间另存，不称固定5秒周期。 / Host receipt intervals are separately retained; no fixed five-second period is inferred.", "",
            "外部 rail / External rail: 每帧保留 ref/rail/avdd/avss/age；帧本身不含 src=external，来源只能与真实 ARL 查询及其原始字节关联。 / Every header retains its actual rail fields. The frame has no external-source field; external provenance comes from recorded ARL evidence, not invented per-frame metadata.",
            f"ARL 外部快照数 / external ARL records: {stats['rails']['external_ARL_records']}; 可信帧值匹配 / header-value matches: {stats['rails']['trusted_headers_matching_external_values']}/{integrity['trusted_frames']}; rail 值变化对 / changed pairs: {stats['rails']['changed_pairs']}; age 回退对 / age decreases: {stats['rails']['age_decrease_pairs']}.", "",
            "可关联诊断 / Observable diagnostics: 仅把真实 seq 相同的 BATRAW/BATREG/ADSHEALTH 记录列为序号关联。BATRAW 是电池原始 ADC 三样本，不是64个矩阵点的 raw ADC。ADS50 是50帧聚合，缺逐点/逐帧标识时不能反推 precision 或 raw。诊断 CDC 是 best effort，缺记录表示未观察到。 / Exact-sequence associations do not prove causality. Battery raw triples are not matrix-cell raw ADC codes; aggregate ADS50 cannot identify every frame's precision state. Missing best-effort debug records remain unobserved.",
            f"本窗诊断标签 / in-window diagnostic tags: `{stats['diagnostics']['inside_counts']}`。",
            f"ADSHEALTH 计数差 / counter deltas: `{stats['diagnostics']['details']['health']['counter_deltas']}`；其分母为独立健康观察窗内已校验 ADC 响应，不能当帧数。 / These counters cover checked ADC responses in a separate health bracket, not the frame denominator.",
            f"BATRAW 记录 / records: {stats['diagnostics']['details']['battery']['raw_triple_records']}; 原始/收尾错误 / original/cleanup errors: `{stats['diagnostics']['details']['battery']['original_errors']}` / `{stats['diagnostics']['details']['battery']['cleanup_errors']}`。",
            f"可见 RAIL_REFRESH / observed records: {stats['diagnostics']['details']['rail_refresh_record_count']}；未观察到不等于证明从未发生。 / Absence in best-effort logs is not proof that an event never occurred.", "",
            "夹具 / Fixture: S1D1/S8D8 为10k电阻，S4D4/S5D5为33pF电容，其余60点按现有夹具定义悬空；本窗没有外部已知稳定电压源或同时模拟波形，不能将悬空点当0V标准或将共同变化唯一归因供电。 / The fixture has two nominal resistors, two capacitors and 60 unpopulated cells; no controlled voltage source or simultaneous analogue waveform establishes voltage accuracy or unique cause.", "",
            "候选及最大变化样例 / Candidate and largest-change examples:", ""]
    for example in stats["pair_screen"]["candidate_examples"]:
        rows.append(f"- seq {example['from_sequence']}→{example['to_sequence']}: {example['candidate_direction']}; {len(example['dominant_points'])}/{example['valid_comparable_points']} points; rows {example['dominant_rows']}; signed median {example['delta_distribution_uv']['median']} µV.")
    if not stats["pair_screen"]["candidate_examples"]:
        rows.append("- 无满足上述多点规则的候选。 / No multi-point candidate meets the stated rule.")
    for example in stats["pair_screen"]["largest_point_examples"]:
        rows.append(f"- {example['point']}, seq {example['from_sequence']}→{example['to_sequence']}: {example['before_uv']}→{example['after_uv']} µV; Δ={example['delta_uv']} µV; PGA {example['before_gain']}→{example['after_gain']}.")
    rows.extend(["", "复算 / Reproduction: `manifest.json` 保存源文件/工具 hash、闭合校验、每帧原始字节范围和命令；`frames.jsonl` 保存所有候选原值及 invalid，`adjacent-pairs.jsonl` 保存所有连续对的有符号差，`lag101.jsonl` 保存所有101序号间隔，`diagnostics.jsonl` 保存真实诊断原文/偏移。`selected.rx.bin` 仅拼接原始完整帧范围，未重编码或修补。 / Source and tool hashes, closure checks and exact byte ranges are retained. All raw numeric values and invalids remain available; selected bytes are unchanged.", ""])
    return ("\n".join(rows) + "\n").encode()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--directory", type=Path, required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--output-directory", type=Path, required=True)
    args = ap.parse_args()
    directory, destination = args.directory.resolve(), args.output_directory.resolve()
    if Path(args.label).name != args.label:
        raise ValueError("label must be a basename")
    if destination.exists() and any(destination.iterdir()):
        raise ValueError("refusing to overwrite a nonempty evidence directory")
    sources = {}
    def load(suffix, key):
        raw, meta = read_source(find(directory, args.label, suffix))
        sources[key] = meta
        return raw
    command = json.loads(load(".command.json", "command"))
    gate = json.loads(load(".gate.json", "gate"))
    summary_bytes = load(".summary.json", "summary")
    summary = json.loads(summary_bytes)
    if not command.get("child_termination_known") or command.get("exit_code") is None or command.get("gate_status") != gate.get("status"):
        raise ValueError("command and gate are not finally closed")
    if command.get("summary_sha256") != sha(summary_bytes):
        raise ValueError("command-finalized summary SHA mismatch")
    if gate.get("formal_authority", {}).get("summary", {}).get("sha256") != sha(summary_bytes):
        raise ValueError("gate-finalized formal summary SHA mismatch")
    if summary.get("transport") != "usb_cdc" or summary.get("benchmark_mode") != "VOLT":
        raise ValueError("requires a formal USB CDC VOLT capture")
    raw = load(".rx.bin", "formal_raw")
    event_bytes = load(".events.jsonl", "formal_events")
    events = [json.loads(x) for x in event_bytes.splitlines()]
    if gate["formal_authority"]["raw"]["sha256"] != sha(raw) or gate["formal_authority"]["events"]["sha256"] != sha(event_bytes):
        raise ValueError("gate-finalized raw/events SHA mismatch")
    assert_closed(summary, events, raw)
    window = benchmark.event_metadata(Path(sources["formal_events"]["path"]))
    if window["capture_window_status"] != "EXPLICIT":
        raise ValueError("explicit formal capture window unavailable")
    origin = summary["capture_monotonic_start"] - window["capture_start_seconds"]
    absolute_window = (summary["capture_monotonic_start"], summary["capture_monotonic_end"])
    payload, selected, excluded, truncated = closed.select_frames(raw, events, window)
    config = summary["snapshots"]["before"]["CONFIG?"]["fields"]
    if _identity(config)[1] != "consistent_frame_identity":
        raise ValueError("complete authoritative before-CONFIG identity missing")
    if any(config.get(k) != str(gate["expected"].get(k)) for k in ["session", "bootId"]):
        raise ValueError("before-CONFIG differs from gate identity")
    contract = benchmark.expected_wire_contract({"mode": "VOLT", "rows": int(summary["rows_requested"])}, config)
    groups = {}
    frames = [analyze_frame(raw[s["source_start"]:s["source_end_exclusive"]], s, config, contract, groups) for s in selected]
    if not frames:
        raise ValueError("no complete candidate in finalized capture")
    diagnostics, formal_tags = diagnostic_lines(raw, events, "formal_raw", origin, absolute_window)
    debug_summary = json.loads(load(".debug.summary.json", "debug_summary"))
    debug_raw = load(".debug.rx.bin", "debug_raw")
    debug_events = [json.loads(x) for x in load(".debug.events.jsonl", "debug_events").splitlines()]
    assert_closed(debug_summary, debug_events, debug_raw, debug=True)
    if gate.get("debug", {}).get("summary", {}).get("sha256") != sha(load(".debug.summary.json", "debug_summary")):
        raise ValueError("gate debug summary hash mismatch")
    debug_lines, debug_tags = diagnostic_lines(debug_raw, debug_events, "debug_raw", debug_summary["started_monotonic"], absolute_window)
    diagnostics.extend(debug_lines)
    seq_events = defaultdict(list)
    for i, d in enumerate(diagnostics):
        d["diagnostic_id"] = i
        if d["fields"].get("seq", "").isdigit():
            seq_events[int(d["fields"]["seq"])].append(i)
    for frame in frames:
        frame["same_sequence_diagnostics"] = seq_events.get(frame.get("sequence"), [])
    pairs, lags = analyze_pairs(frames)
    trusted = [f for f in frames if f["trusted"]]
    eligible = [p for p in pairs if p["eligible"]]
    candidates = [p for p in eligible if p["same_direction_ge18mV_candidate"]]
    external = [d for d in diagnostics if d["tag"] == "ARL" and d["fields"].get("src") == "external"]
    ext_values = {(d["fields"].get("avdd"), d["fields"].get("avss")) for d in external}
    for f in frames:
        matching = [d for d in external if
                    d["host_receipt_monotonic"] <= origin + f["complete_receipt_seconds"] and
                    all(f["header"].get(k) == d["fields"].get(k) for k in ["avdd", "avss"])]
        f["rail_provenance"] = {"source_field_present_in_frame": "src" in f["header"],
                                "matching_preceding_external_ARL_ids": [d["diagnostic_id"] for d in matching],
                                "interpretation": "header rail values consistent with preceding recorded external calibration; not an independent physical reading or per-frame source assertion"}
    per_point = {}
    for i in range(int(summary["rows_requested"]) * 8):
        per_point[point_name(i)] = {"signed_voltage_uv": distribution([f["values_uv"][i] for f in trusted if f["usable_points"][i]]),
                                   "invalid_reasons": dict(Counter(f"X{f['parsed']['error_reasons'][i]:02X}" for f in trusted if f["values_uv"][i] is None)),
                                   "unusable_count": sum(not f["usable_points"][i] for f in trusted),
                                   "signed_adjacent_delta_uv": distribution([p["signed_delta_uv"][i] for p in eligible if p["signed_delta_uv"][i] is not None])}
    by_seq = {f["sequence"]: f for f in trusted}
    extreme = sorted(((abs(v), p, i, v) for p in eligible for i, v in enumerate(p["signed_delta_uv"]) if v is not None), key=lambda x: x[0], reverse=True)[:8]
    examples = [{"point": point_name(i), "from_sequence": p["from_sequence"], "to_sequence": p["to_sequence"],
                 "before_uv": by_seq[p["from_sequence"]]["values_uv"][i], "after_uv": by_seq[p["to_sequence"]]["values_uv"][i], "delta_uv": value,
                 "before_gain": by_seq[p["from_sequence"]]["parsed"]["pga_gains"][i], "after_gain": by_seq[p["to_sequence"]]["parsed"]["pga_gains"][i],
                 "candidate": p["same_direction_ge18mV_candidate"],
                 "same_sequence_diagnostics": seq_events.get(p["to_sequence"], [])} for _, p, i, value in extreme]
    transport = benchmark.analyse_capture(directory, args.label, summary, debug_summaries=())
    stats = {"schema_version": 1, "status": "OBSERVED_NOT_PRECISION_OR_CAUSAL_ACCEPTANCE",
             "source_gate_status": gate["status"], "expected_identity": gate["expected"], "configuration": config,
             "raw_sha256": sha(raw), "window": window,
             "method": {"threshold_uv": THRESHOLD_UV, "same_direction_fraction_minimum": "1/2", "minimum_same_direction_points": 2,
                        "pair_eligibility": "same complete CONFIG identity, sequence+1, CRC/identity/wire valid; both points valid/fresh/expected/acquired and error-free",
                        "all_board_screen": "all64 comparable points must each change in the same direction by at least18mV",
                        "rounding": "none before differencing; original integer microvolts", "distribution": "observed min/max, linearly interpolated P05/P95, population stddev; not accuracy/CI",
                        "missing_or_invalid_substitution": "none", "causal_or_precision_PASS": False},
             "integrity": {"selected_candidates": len(frames), "trusted_frames": len(trusted),
                           "first_sequence": trusted[0]["sequence"] if trusted else None, "last_sequence": trusted[-1]["sequence"] if trusted else None,
                           "invalid_token_points": sum(v is None for f in trusted for v in f["values_uv"]),
                           "invalid_mask_points": sum(f["parsed"]["cells"] - f["parsed"]["valid_mask"].bit_count() for f in trusted),
                           "unacquired_points": sum((f["parsed"]["expected_mask"] & ~f["parsed"]["acquired_mask"]).bit_count() for f in trusted),
                           "not_fresh_points": sum(f["parsed"]["cells"] - f["parsed"]["fresh_mask"].bit_count() for f in trusted),
                           "unusable_points": sum(not value for f in trusted for value in f["usable_points"]),
                           "invalid_reasons": dict(Counter(f"X{reason:02X}" for f in trusted for value, reason in zip(f["values_uv"], f["parsed"]["error_reasons"]) if value is None)),
                           "benchmark_capture_complete_frames": transport["capture_counts"].get("complete_frames"),
                           "source_frame_count_matches_benchmark": len(trusted) == transport["capture_counts"].get("trusted_complete_frames"),
                           "parser_counters_selected_sum": dict(sum((Counter(f["parser_counters"]) for f in frames), Counter()))},
             "timing": {"adjacent_device_us": distribution([p["device_delta_us"] for p in eligible]),
                        "adjacent_host_complete_seconds": distribution([p["host_complete_delta_seconds"] for p in eligible]),
                        "frame_duration_us": distribution([f["parsed"]["frame_duration_us"] for f in trusted]),
                        "lag101_device_us": distribution([p["device_delta_us"] for p in lags]),
                        "lag101_host_seconds": distribution([p["host_complete_delta_seconds"] for p in lags]),
                        "cell_sample_interval": "NOT_REPORTED; frame timestamps/durations do not establish each cell's sampling time"},
             "rails": {"header_profiles": [dict(zip(["ref", "rail", "avdd", "avss"], key), frames=count) for key, count in Counter(tuple(f["header"].get(k) for k in ["ref", "rail", "avdd", "avss"]) for f in trusted).items()],
                       "external_ARL_records": len(external), "external_ARL_diagnostic_ids": [d["diagnostic_id"] for d in external],
                       "trusted_headers_matching_external_values": sum((f["header"].get("avdd"), f["header"].get("avss")) in ext_values for f in trusted),
                       "changed_pairs": sum(p["rail_values_changed"] for p in eligible),
                       "age_decrease_pairs": sum(p["rail_age_delta_frames"] < 0 for p in eligible),
                       "age_deltas": dict(Counter(p["rail_age_delta_frames"] for p in eligible)),
                       "source_limit": "frame ref=AVDD_AVSS is the ADC reference, not an external-calibration-source field; fixed external provenance is query evidence"},
             "pair_screen": {"eligible_pairs": len(eligible), "ineligible_pairs": len(pairs) - len(eligible),
                             "candidate_pairs": len(candidates), "all64_pairs": sum(p["all64_points_same_direction_ge18mV"] for p in eligible),
                             "any_large_delta_pairs": sum(p["up_ge_18mV"] + p["down_le_minus18mV"] > 0 for p in eligible),
                             "maximum_same_direction_points": max((len(p["dominant_points"]) for p in eligible), default=None),
                             "candidate_examples": candidates[:5], "largest_point_examples": examples},
             "diagnostics": {"formal_all_tags": formal_tags, "debug_all_tags": debug_tags,
                             "inside_counts": dict(Counter(d["tag"] for d in diagnostics if d["inside_formal_host_window"])),
                             "all_counts": dict(Counter(d["tag"] for d in diagnostics)),
                             "details": diagnostic_statistics(diagnostics, pairs),
                             "same_sequence_links_are_causal_proof": False,
                             "debug_source_is_formal_science": False, "missing_best_effort_logs": "NOT_OBSERVED"},
             "per_point": per_point,
             "limits": ["Passive10k/33pF plus60 floating cells, no controlled stable voltage source.", "Original E1 complete raw files absent; this is a later finite-condition observation, not an exact A/B recreation.",
                        "No simultaneous oscilloscope/analogue-state verification; numeric common changes cannot uniquely identify a physical cause.", "A frame timestamp is not a per-cell sample timestamp; host receipt is not over-air or USB wire time."]}
    outputs = {"selected.rx.bin": payload, "frames.jsonl": jsonl(frames), "adjacent-pairs.jsonl": jsonl(pairs),
               "lag101.jsonl": jsonl(lags), "diagnostics.jsonl": jsonl(diagnostics), "summary.json": encode(stats),
               "whole-recording-transport.json": encode(transport), "report.md": report_text(stats)}
    dependencies = [Path(__file__).resolve(), HELPER, ROOT / "tests/fixtures/reference_10k_33pf.json"] + [ROOT / "tools" / name for name in ["text_protocol.py", "analyze_firmware_benchmark.py", "reference_fixture.py", "estimate_floating_cap_background.py", "sensorarray_bulk_diagnostics.py"]]
    manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(), "invocation": [sys.executable, *sys.argv],
                "sources": sources, "tools": [benchmark.file_hash(p) for p in dependencies],
                "closure": {"command_final": True, "command_summary_hash_match": True, "formal_finished_event": events[-1], "debug_finished_event": debug_events[-1],
                            "raw_hash_and_byte_counts_match": True, "RX_ranges_contiguous_complete": True},
                "selection_method": "original header first byte and final K last byte received inside explicit formal host capture window; no quality filtering at byte selection",
                "selected": selected, "excluded_complete": excluded, "truncated_candidates": truncated,
                "outputs": {k: {"bytes": len(v), "sha256": sha(v)} for k, v in outputs.items()},
                "hardware_actions": False, "calibration_written": False, "original_bytes_modified": False}
    for src in sources.values():
        if sha(Path(src["path"]).read_bytes()) != src["stored_sha256"]:
            raise ValueError("source changed during replay")
    destination.mkdir(parents=True, exist_ok=True)
    for filename, value in {**outputs, "manifest.json": encode(manifest)}.items():
        with (destination / filename).open("xb") as stream:
            stream.write(value)
    print(json.dumps({"output": str(destination), "frames": len(trusted), "candidates": len(candidates), "all64": stats["pair_screen"]["all64_pairs"], "source_gate": gate["status"]}))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
        raise SystemExit(f"CLOSED_VOLTAGE_REVIEW_ERROR: {exc}")
