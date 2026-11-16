"""Describe closed host callback spacing; this is not over-air event timing."""
import argparse
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics


def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    return {str(p): values[round((len(values) - 1) * p / 100)]
            for p in (0, 10, 25, 50, 75, 90, 95, 99, 100)}


def analyze(journal, analysis):
    source = json.loads(analysis.read_text())["captures"][0]
    window = source["window"]
    start = window["capture_monotonic_start"]
    end = window["capture_monotonic_end"]
    assert end > start
    rows = []
    for line in journal.read_text().splitlines():
        row = json.loads(line)
        timestamp = row.get("monotonic_ns", 0) / 1e9
        if row.get("type") != "notification" or not start <= timestamp <= end:
            continue
        payload = base64.b64decode(row["payload_base64"], validate=True)
        assert len(payload) == row["length"]
        rows.append((timestamp, row["channel"], payload))
    assert rows and all(a[0] <= b[0] for a, b in zip(rows, rows[1:]))
    result = {}
    for channel in ("all", "D", "L", "C"):
        selected = [r for r in rows if channel == "all" or r[1] == channel]
        gaps = [(b[0] - a[0]) * 1000 for a, b in zip(selected, selected[1:])]
        sizes = [len(r[2]) for r in selected]
        # G header is already physically observed ASCII, not a reconstruction.
        fragments = Counter()
        for _, _, payload in selected:
            if payload.startswith(b"G,"):
                fields = payload.split(b"\n", 1)[0].split(b",")
                fragments[fields[4].decode("ascii")] += 1
        result[channel] = {
            "notifications": len(selected), "bytes": sum(sizes),
            "notifications_per_capture_second": len(selected) / (end - start),
            "payload_mean_bytes": statistics.mean(sizes) if sizes else None,
            "callback_gap_ms_percentiles": quantiles(gaps),
            "gap_count": len(gaps),
            "gap_le_1ms": sum(g <= 1 for g in gaps),
            "gap_ge_10ms": sum(g >= 10 for g in gaps),
            "gap_ge_20ms": sum(g >= 20 for g in gaps),
            "gap_ge_40ms": sum(g >= 40 for g in gaps),
            "observed_G_fragment_count_histogram": dict(fragments),
        }
    return {
        "scope": "Host callback receipts inside the explicit USB benchmark window; same host monotonic clock. Includes all subscribed channels, not radio capture or controller connection-event timing.",
        "limitations": "Callback batching and host scheduling affect spacing. These observations cannot distinguish peer/controller scheduling from firmware readiness, or establish over-air packet loss.",
        "inputs": [{"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                   for p in (journal, analysis)],
        "window_monotonic": [start, end], "channels": result,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--journal", type=Path, required=True)
    parser.add_argument("--analysis", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.journal, args.analysis)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result["channels"], indent=2))
