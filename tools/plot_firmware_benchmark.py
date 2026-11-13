#!/usr/bin/env python3
"""Plot four scientific FPS panels from analyze_firmware_benchmark JSON.

No captures or devices are opened. Missing, incomplete, or ambiguous rate
measurements remain gaps, never zero-valued samples. Existing figures are not
overwritten. Use --synthetic for conspicuously labelled rendering tests only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys


MODES = ("CAP", "VOLT", "RES", "MIXED")
SERIES = (
    ("capture", "Device capture", "#0072B2", "o", "-"),
    ("usb", "USB received", "#D55E00", "s", "--"),
    ("ble", "BLE received", "#009E73", "^", "-."),
)


def finite_rate(value):
    """A genuine measured zero is allowed; null/missing is always NaN."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return math.nan
    return float(value) if math.isfinite(value) and value >= 0 else math.nan


def extract_rates(report, label_prefix=None, selection=None):
    data = {mode: {key: [math.nan] * 8 for key, *_ in SERIES} for mode in MODES}
    selected = {}
    skipped = []
    seen = set()
    table = report.get("configuration_table")
    if not isinstance(table, list):
        raise ValueError("input is missing configuration_table from the benchmark analyzer")
    if selection is not None:
        if label_prefix is not None:
            raise ValueError("use either --selection or --label-prefix")
        expected = {f"{mode}{rows}" for mode in MODES for rows in range(1, 9)}
        if not isinstance(selection, dict) or set(selection) != expected:
            raise ValueError("selection must explicitly map all 32 configurations to a label or null")
    for row in table:
        mode, rows = row.get("mode"), row.get("rows")
        if mode not in MODES or not isinstance(rows, int) or not 1 <= rows <= 8:
            raise ValueError("configuration_table contains an unsupported mode or row count")
        if (mode, rows) in seen:
            raise ValueError(f"duplicate table entry for {mode} {rows}")
        seen.add((mode, rows))
        candidates = [item for item in row.get("captures", [])
                      if label_prefix is None or item.get("label", "").startswith(label_prefix)]
        if selection is not None:
            requested = selection[f"{mode}{rows}"]
            candidates = [item for item in candidates if requested is not None and item.get("label") == requested]
            if requested is not None and not candidates:
                raise ValueError(f"selected label {requested!r} is absent for {mode} {rows}")
        if len(candidates) > 1:
            raise ValueError(f"multiple runs for {mode} {rows}; choose an unambiguous --label-prefix")
        if not candidates:
            skipped.append({"mode": mode, "rows": rows, "reason": "NOT_RUN"})
            continue
        capture = candidates[0]
        selected[f"{mode}{rows}"] = capture.get("label")
        groups = capture.get("groups", [])
        if capture.get("status") not in {"RECORDED", "RECORDED_WITH_ERRORS"} or len(groups) != 1:
            skipped.append({"mode": mode, "rows": rows, "reason": "INCOMPLETE_OR_MULTIPLE_CONFIG_IDENTITIES"})
            continue
        rates = groups[0].get("rates", {})
        data[mode]["capture"][rows - 1] = finite_rate(rates.get("capture_fps"))
        data[mode]["usb"][rows - 1] = finite_rate(rates.get("host_complete_receive_fps"))
        ble = capture.get("ble") or {}
        if ble.get("status") in {"RECORDED", "RECORDED_WITH_ERRORS"}:
            data[mode]["ble"][rows - 1] = finite_rate(ble.get("comparative_host_fps"))
    return data, selected, skipped


def render(report_path, output_directory=None, stem="firmware-benchmark-fps",
           label_prefix=None, synthetic=False, selection_path=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", stem):
        raise ValueError("stem must be a simple filename without path separators")
    raw = report_path.read_bytes()
    report = json.loads(raw)
    source_sha = hashlib.sha256(raw).hexdigest()
    selection = None
    if selection_path is not None:
        choice = json.loads(selection_path.read_text(encoding="utf-8"))
        if choice.get("source_report_sha256") not in {None, source_sha}:
            raise ValueError("selection references a different source report SHA256")
        selection = choice.get("selected_labels")
        if selection is None:
            raise ValueError("selection JSON must contain selected_labels")
    data, selected, skipped = extract_rates(report, label_prefix, selection)
    synthetic = synthetic or str(report.get("evidence_type", "")).startswith("SYNTHETIC")
    directory = output_directory or report_path.parent
    if not directory.is_dir():
        raise ValueError("output directory must already exist")
    if synthetic and not stem.startswith("synthetic-"):
        stem = "synthetic-" + stem
    outputs = {extension: directory / f"{stem}.{extension}" for extension in ("svg", "png")}
    if any(path.exists() or path.with_name(path.name + ".tmp").exists() for path in outputs.values()):
        raise ValueError("figure output exists; choose a new stem (existing figures are never overwritten)")
    matplotlib.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                                "svg.fonttype": "none", "svg.hashsalt": source_sha[:32],
                                "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    title = "SYNTHETIC RENDER TEST - NOT MEASURED" if synthetic else "SensorArray firmware throughput"
    figure.suptitle(title, fontsize=17, fontweight="bold", y=0.98,
                   color="#A32626" if synthetic else "#18212B")
    figure.text(0.5, 0.935, "Simultaneous USB + BLE | Window-average frame rates",
                ha="center", fontsize=11, color="#4D5965")
    handles = []
    for axis, mode in zip(axes.flat, MODES):
        finite_values = []
        for key, label, color, marker, style in SERIES:
            values = data[mode][key]
            handle, = axis.plot(range(1, 9), values, label=label, color=color,
                                linestyle=style, linewidth=1.65, marker=marker,
                                markersize=5, markeredgewidth=0.8,
                                markerfacecolor="white" if key == "usb" else color)
            if mode == MODES[0]:
                handles.append(handle)
            finite_values += [value for value in values if math.isfinite(value)]
        axis.set_title(mode, loc="left", fontsize=12, fontweight="bold", pad=8)
        axis.set_xlim(0.75, 8.25)
        axis.set_xticks(range(1, 9))
        axis.tick_params(labelbottom=True)
        axis.set_xlabel("Active rows")
        axis.set_ylabel("Frames per second")
        axis.grid(axis="y", color="#D9DFE5", linewidth=0.7, alpha=0.8)
        axis.set_axisbelow(True)
        if finite_values:
            axis.set_ylim(0, max(finite_values) * 1.12 if max(finite_values) else 1)
            axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        else:
            axis.set_yticks([])
            axis.text(0.5, 0.5, "Not measured", transform=axis.transAxes,
                      ha="center", va="center", color="#6B7280")
    figure.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.9),
                  ncol=3, frameon=False, handlelength=3, columnspacing=2.2)
    figure.text(0.08, 0.067, "Missing or incomplete measurements are left blank; lines do not bridge missing row counts.",
                fontsize=9, color="#4D5965")
    figure.text(0.08, 0.042, "MIXED 1: one active CAP row; full CVRCVRCV profile uses M/MR frames. Averages are not accuracy acceptance.",
                fontsize=9, color="#4D5965")
    figure.text(0.08, 0.017, "RECORDED_WITH_ERRORS runs remain plotted; rejected frames are excluded from receive FPS.",
                fontsize=9, color="#4D5965")
    figure.subplots_adjust(left=0.08, right=0.98, bottom=0.15, top=0.815,
                           hspace=0.42, wspace=0.23)
    description = (f"Source: {report_path.name}; SHA256: {source_sha}; "
                   f"labels: {json.dumps(selected, sort_keys=True)}; "
                   f"synthetic={synthetic}; window averages are not accuracy acceptance.")
    try:
        for extension, path in outputs.items():
            temporary = path.with_name(path.name + ".tmp")
            metadata = ({"Title": title, "Creator": "plot_firmware_benchmark.py", "Description": description, "Date": None}
                        if extension == "svg" else {"Title": title, "Software": "plot_firmware_benchmark.py", "Description": description})
            try:
                with temporary.open("xb") as stream:
                    figure.savefig(stream, format=extension, dpi=200, facecolor="white", metadata=metadata)
                os.link(temporary, path)  # Atomic no-clobber output installation.
            finally:
                temporary.unlink(missing_ok=True)
    finally:
        plt.close(figure)
    return {"source": str(report_path), "source_sha256": source_sha, "synthetic": synthetic,
            "selection_file": str(selection_path) if selection_path else None,
            "outputs": {key: str(path) for key, path in outputs.items()}, "selected_runs": selected,
            "missing_values": {mode: {key: sum(not math.isfinite(x) for x in values) for key, values in series.items()}
                               for mode, series in data.items()}, "skipped_configurations": skipped}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--stem", default="firmware-benchmark-fps")
    parser.add_argument("--label-prefix", help="select a run family explicitly when a configuration has multiple runs")
    parser.add_argument("--selection", type=Path, help="explicit selected_labels mapping emitted by the benchmark analyzer")
    parser.add_argument("--synthetic", action="store_true", help="prominent synthetic-test labels; never use to describe real measurements")
    args = parser.parse_args(argv)
    try:
        result = render(args.input, args.output_directory, args.stem, args.label_prefix, args.synthetic, args.selection)
    except (OSError, ValueError, ImportError) as exc:
        parser.exit(2, f"PLOT_ERROR: {exc}\n")
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
