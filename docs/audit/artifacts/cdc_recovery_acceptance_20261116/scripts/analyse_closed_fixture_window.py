#!/usr/bin/env python3
"""Replay one finalized formal-USB recording; never open a device or overwrite.

Example (also works per finalized RES/VOLT/MIXED matrix label):
  python3 <this-script> --directory <capture-directory> --label <capture-label>

The derived raw contains unchanged source byte ranges, including bad frames.
All reports are diagnostic; no unloaded baseline or precision PASS is inferred.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from datetime import datetime, timezone
import gzip
import hashlib
import io
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "tools"))
import analyze_firmware_benchmark as benchmark
from estimate_floating_cap_background import (
    FLOATING, _memory_diagnostic, distribution, estimate_capture,
)
from reference_fixture import (
    DEFAULT_FIXTURE, IDENTITY_FIELDS, analyse_capture, evaluate_fixture, load_fixture,
)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def existing(directory, label, suffix):
    path = directory / (label + suffix)
    if not path.exists():
        path = Path(str(path) + ".gz")
    if not path.is_file():
        raise ValueError(f"missing finalized source: {path}")
    return path


def read_source(path):
    stored = path.read_bytes()
    compressed = path.suffix == ".gz"
    raw = gzip.decompress(stored) if compressed else stored
    return raw, {"path": str(path), "stored_bytes": len(stored),
                 "stored_sha256": sha(stored), "gzip": compressed,
                 "raw_bytes": len(raw), "raw_sha256": sha(raw)}


def select_frames(raw, events, window):
    rx = [e for e in events if e.get("type") == "rx"]
    ends = [e["offset"] + e["length"] for e in rx]
    start, end = window["capture_start_seconds"], window["capture_end_seconds"]

    def receipt(offset):
        index = bisect_right(ends, offset)
        if index >= len(rx) or offset < rx[index]["offset"]:
            raise ValueError(f"source byte has no RX receipt: {offset}")
        return {"event_index": index, "t": rx[index]["t"],
                "rx_offset": rx[index]["offset"], "rx_length": rx[index]["length"],
                "recorded_phase": rx[index].get("phase")}

    selected, excluded, truncated = [], [], []
    output = bytearray()
    pending = None
    offset = 0
    for wire in io.BytesIO(raw):
        first, offset = offset, offset + len(wire)
        line = wire.rstrip(b"\r\n").decode("ascii", errors="surrogateescape")
        tag = line.split(",", 1)[0]
        if not wire.endswith(b"\n"):
            continue
        is_header = tag in {"C", "R", "V", "M"} and not _memory_diagnostic(line)
        if is_header:
            if pending:
                truncated.append({**pending, "source_end_exclusive": first,
                                  "reason": "next_header_before_K"})
            pending = {"source_start": first, "header_end_exclusive": offset,
                       "wire_tag": tag, "header": benchmark.parse_fields(line)}
        elif tag == "K" and pending:
            info = {**pending, "source_end_exclusive": offset,
                    "header_first_byte_receipt": receipt(pending["source_start"]),
                    "header_last_byte_receipt": receipt(pending["header_end_exclusive"] - 1),
                    "trailer_last_byte_receipt": receipt(offset - 1)}
            first_t = info["header_first_byte_receipt"]["t"]
            last_t = info["trailer_last_byte_receipt"]["t"]
            piece = raw[pending["source_start"]:offset]
            info.update(source_bytes=len(piece), source_sha256=sha(piece))
            if start <= first_t <= last_t <= end:
                info.update(output_start=len(output), output_end_exclusive=len(output) + len(piece))
                selected.append(info)
                output.extend(piece)
            else:
                info["reason"] = "not_all_frame_bytes_received_inside_capture_window"
                excluded.append(info)
            pending = None
    if pending:
        truncated.append({**pending, "source_end_exclusive": len(raw), "reason": "EOF_before_K"})
    payload = bytes(output)
    # Independently reconstruct every output range from source and its digest.
    for item in selected:
        source = raw[item["source_start"]:item["source_end_exclusive"]]
        derived = payload[item["output_start"]:item["output_end_exclusive"]]
        if source != derived or sha(source) != item["source_sha256"]:
            raise ValueError("source/output range reconstruction failed")
    return payload, selected, excluded, truncated


def metrics_report(capture, fixture_report, proxy, expected_config):
    groups = []
    for group, assessment in zip(capture.groups, fixture_report["groups"]):
        references = {}
        for label, result in assessment["references"].items():
            point = group.cells.get((label, result["mode"]))
            references[label] = {**result, "empirical_distribution": distribution(point.values if point else [])}
        floating_cap = {label: distribution(group.cells[(label, "CAP")].values)
                        for label in FLOATING if (label, "CAP") in group.cells}
        medians = [v["median"] for v in floating_cap.values() if v["median"] is not None]
        groups.append({"identity": group.identity, "frame_type": group.frame_type,
                       "rows": group.rows, "profile": group.profile, "frames": group.frames,
                       "first_sequence": group.first_sequence, "last_sequence": group.last_sequence,
                       "device_timestamp_span_seconds": group.duration(),
                       "fixture_screening": assessment["screening"],
                       "matches_before_CONFIG": all(group.identity.get(k) == expected_config.get(k) for k in IDENTITY_FIELDS),
                       "references": references, "floating_cap_per_cell": floating_cap,
                       "floating_cap_time_medians_across_cells": distribution(medians),
                       "floating_res_pass": assessment["floating_res_pass"],
                       "floating_res_observed": assessment["floating_res_observed"]})
    return {"schema_version": 1, "precision_acceptance": "NOT_EVALUATED",
            "calibration_written": False, "fixture_report_status": fixture_report["status"],
            "interval_semantics": "min/max and P05/P95 are observed sample spread, not accuracy bounds or confidence intervals; population stddev",
            "floating_summary_semantics": "Distribution across per-cell time medians; never pool different cells into one target time series.",
            "groups": groups,
            "cap_proxy_summary": [{k: v for k, v in group.items() if k != "per_frame"} for group in proxy["groups"]]}


def make_markdown(label, source, window, transport, metrics, paths):
    lines = [f"# Recorded fixture window: {label}", "",
             "Derived only from finalized formal USB raw bytes. No device commands, baseline substitution, calibration writes, or precision PASS.", "",
             f"Source raw SHA256: `{source['raw_sha256']}`.",
             f"Host capture window: {window['capture_start_seconds']:.9f}–{window['capture_end_seconds']:.9f} s (relative receipt clock).",
             f"Transport replay status: `{transport['status']}`; fixture report: `{metrics['fixture_report_status']}`.", "",
             "All intervals below describe observed samples. P05–P95 is not a confidence interval. CAP values are LC total capacitance; a same-cell unloaded baseline is absent.", ""]

    def num(value):
        return "unavailable" if value is None else f"{value:.6f}"

    for group in metrics["groups"]:
        lines.extend([f"## {group['frame_type']} / {group['rows']} rows / rev {group['identity'].get('rev')}", "",
                      f"Session `{group['identity'].get('session')}`, boot `{group['identity'].get('bootId')}`; sequences {group['first_sequence']}–{group['last_sequence']}; {group['frames']} frames.",
                      f"Device first-to-last timestamp span: {group['device_timestamp_span_seconds']:.6f} s; strict fixture screening: `{group['fixture_screening']}`.", "",
                      "| Point | Unit | Valid n | Invalid n | Mean | Min–max | P05–P95 | Accuracy status |",
                      "|---|---|---:|---:|---:|---|---|---|"])
        for point, result in group["references"].items():
            stats = result["empirical_distribution"]
            if not result["statistics"]["observed"]:
                continue
            lines.append(f"| {point} | {result['unit']} | {stats['samples']} | {result['statistics']['invalid']} | {num(stats['mean'])} | {num(stats['min'])}–{num(stats['max'])} | {num(stats['p05'])}–{num(stats['p95'])} | {result['accuracy']} |")
        lines.extend(["", f"Floating RES cells meeting OPEN checks: {group['floating_res_pass']}/{group['floating_res_observed']} observed."])
        floating = group["floating_cap_time_medians_across_cells"]
        if floating["samples"]:
            lines.append(f"Floating CAP per-cell time medians ({floating['samples']} cells): {num(floating['min'])}–{num(floating['max'])} pF. Per-cell distributions remain in metrics JSON.")
        lines.append("")
    for group in metrics["cap_proxy_summary"]:
        if not any(t["target_usable_frames"] for t in group["targets"].values()):
            continue
        lines.extend(["## Cross-cell CAP background proxy", "",
                      "All three fixed donor populations are reported. Missing/invalid/stale/unacquired donors exclude that frame/method; no shrinking, zero fill, smoothing, or selection by closeness to 33 pF.", "",
                      "| Point | Method | Paired n | Background mean (pF) | Difference mean (pF) | Difference min–max (pF) | Difference P05–P95 (pF) |",
                      "|---|---|---:|---:|---:|---|---|"])
        for point, target in group["targets"].items():
            for name, method in target["methods"].items():
                d = method["target_minus_background_pf"]
                lines.append(f"| {point} | {name} | {method['valid_paired_count']} | {num(method['background_pf']['mean'])} | {num(d['mean'])} | {num(d['min'])}–{num(d['max'])} | {num(d['p05'])}–{num(d['p95'])} |")
        lines.append("")
    lines.extend(["## Reproduction and provenance", "",
                  "The selection manifest records source file hashes, tool hashes, invocation, every retained source/output byte range and its SHA256, excluded frames and truncated candidates. Only whole original ranges are concatenated; wire text and CRC are never rebuilt.", "",
                  "A frame's first header byte and final K byte must both arrive inside the explicit capture_start/capture_end window. This is at least as strict as the benchmark parser's complete-header/final-K rule; their counts are compared in the manifest.", "",
                  "The transport JSON audits diagnostics and malformed/incomplete material from the full original raw stream, so science-only slicing cannot erase a recorded fault. CAP proxy output is an approximate diagnostic, not a strict same-cell baseline.", ""])
    lines.extend(f"- [{name}]({path})" for name, path in paths.items())
    return ("\n".join(lines) + "\n").encode("utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-directory", type=Path, help="Existing directory; default is the source directory")
    parser.add_argument("--output-prefix", help="Default: <label>.fixture-window; outputs must not exist")
    args = parser.parse_args()
    directory = args.directory.resolve()
    destination = (args.output_directory or directory).resolve()
    if not directory.is_dir() or not destination.is_dir() or Path(args.label).name != args.label:
        parser.error("source/output directories must exist and label must be a basename")
    prefix = args.output_prefix or args.label + ".fixture-window"
    if Path(prefix).name != prefix:
        parser.error("output-prefix must be a basename")
    paths = {name: destination / (prefix + suffix) for name, suffix in {
        "raw": ".rx.bin", "selection": ".selection.json", "transport": ".transport.json",
        "fixture": ".fixture.json", "proxy": ".cap-proxy.json", "metrics": ".metrics.json", "report": ".report.md"}.items()}
    for path in paths.values():
        if path.exists():
            raise FileExistsError(f"refusing overwrite: {path}")
    summary_path = directory / (args.label + ".summary.json")
    summary_bytes, summary_source = read_source(summary_path)
    summary = json.loads(summary_bytes)
    if summary.get("status") not in {"CAPTURED", "COMPLETED", "ERROR", "INTERRUPTED"} or not summary.get("raw_sha256"):
        raise ValueError("summary is not finalized")
    if summary.get("transport") not in {"usb_cdc", "usb_bulk"}:
        raise ValueError("only finalized formal USB recordings are supported")
    raw_path = existing(directory, args.label, ".rx.bin")
    event_path = existing(directory, args.label, ".events.jsonl")
    raw, raw_source = read_source(raw_path)
    event_bytes, event_source = read_source(event_path)
    events = [json.loads(line) for line in event_bytes.decode("utf-8").splitlines()]
    finished = [e for e in events if e.get("type") == "finished"]
    if len(finished) != 1 or events[-1] != finished[0] or finished[0].get("status") != summary["status"]:
        raise ValueError("recording has no unique matching terminal finished event")
    if raw_source["raw_sha256"] != summary["raw_sha256"] or len(raw) != summary.get("rx_bytes"):
        raise ValueError("finalized summary raw hash/byte count mismatch")
    window = benchmark.event_metadata(event_path)
    if window["capture_window_status"] != "EXPLICIT" or window["rx_bytes"] != len(raw):
        raise ValueError("explicit capture window or contiguous complete RX coverage unavailable")
    payload, selected, excluded, truncated = select_frames(raw, events, window)
    if not selected:
        raise ValueError("no whole original frames within explicit capture window")
    transport = benchmark.analyse_capture(directory, args.label, summary, debug_summaries=())
    source = {"path": str(paths["raw"]), "sha256": sha(payload), "bytes": len(payload),
              "selection_manifest": str(paths["selection"]), "original": raw_source,
              "provenance": "recorded; byte-exact concatenation of finalized formal USB frame ranges"}
    lines = payload.decode("ascii", errors="surrogateescape").splitlines()
    capture = analyse_capture(lines, source)
    fixture = evaluate_fixture(load_fixture(DEFAULT_FIXTURE), capture, provenance="recorded")
    proxy = estimate_capture(lines, source)
    expected_config = summary.get("snapshots", {}).get("before", {}).get("CONFIG?", {}).get("fields", {})
    metrics = metrics_report(capture, fixture, proxy, expected_config)
    outputs = {"raw": payload, "transport": encoded(transport), "fixture": encoded(fixture),
               "proxy": encoded(proxy), "metrics": encoded(metrics)}
    outputs["report"] = make_markdown(args.label, raw_source, window, transport, metrics, paths)
    dependencies = [Path(__file__).resolve(), DEFAULT_FIXTURE] + [ROOT / "tools" / name for name in
        ["analyze_firmware_benchmark.py", "reference_fixture.py", "estimate_floating_cap_background.py", "text_protocol.py", "sensorarray_bulk_diagnostics.py"]]
    manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                "invocation": [sys.executable, *sys.argv], "cwd": str(Path.cwd()),
                "sources": {"summary": summary_source, "events": event_source, "raw": raw_source},
                "tools": [benchmark.file_hash(p) for p in dependencies],
                "closure": {"finished_event": finished[0], "collector_status": summary["status"],
                            "raw_hash_matches_summary": True, "complete_contiguous_rx_coverage": True,
                            "sources_rehashed_unchanged_before_output": True},
                "window": window,
                "method": {"time_authority": "capture_start/end event t, not event phase labels",
                           "bounds": "inclusive receipt time; source/output byte ranges use exclusive end",
                           "whole_frame": "first header byte and final K byte must both be received within window",
                           "quality_filter_at_selection": "none; malformed/CRC-invalid/identity-invalid complete candidates retained",
                           "source_bytes_reencoded": False, "boundary_frames_repaired": False,
                           "whole_recording_quality_report": str(paths["transport"]),
                           "scope": "formal USB science only; no BLE or debug source merged"},
                "counts": {"selected_frame_candidates": len(selected), "excluded_complete_candidates": len(excluded),
                           "truncated_candidates": len(truncated), "selected_parsed_frames": sum(g.frames for g in capture.groups),
                           "benchmark_capture_parsed_frames": transport["capture_counts"].get("complete_frames", 0),
                           "collector_capture_parsed_frames": summary.get("capture_parsed_frames"),
                           "benchmark_count_matches": sum(g.frames for g in capture.groups) == transport["capture_counts"].get("complete_frames", 0)},
                "selected": selected, "excluded": excluded, "truncated": truncated,
                "outputs": {key: {"path": str(paths[key]), "bytes": len(value), "sha256": sha(value)} for key, value in outputs.items()},
                "precision_acceptance": "NOT_EVALUATED", "calibration_written": False}
    for item in [summary_source, event_source, raw_source]:
        if sha(Path(item["path"]).read_bytes()) != item["stored_sha256"]:
            raise ValueError("source changed during replay; refusing derived outputs")
    for name, content in outputs.items():
        with paths[name].open("xb") as output:
            output.write(content)
    with paths["selection"].open("xb") as output:
        output.write(encoded(manifest))
    print(json.dumps({"report": str(paths["report"]), "selection": str(paths["selection"]),
                      "frames": manifest["counts"], "fixture_status": fixture["status"],
                      "proxy_status": proxy["status"], "precision_acceptance": "NOT_EVALUATED"}))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, EOFError) as error:
        raise SystemExit(f"CLOSED_FIXTURE_REPLAY_ERROR: {error}")
