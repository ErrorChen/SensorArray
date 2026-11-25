#!/usr/bin/env python3
"""Preserve whole-recording sequence gaps outside a closed fixture window."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_bytes(path):
    if not path.exists():
        path = path.with_name(path.name + ".gz")
    data = path.read_bytes()
    return gzip.decompress(data) if path.suffix == ".gz" else data


def write_once(path, text):
    if path.exists():
        assert path.read_text() == text, "refusing to replace different evidence"
    else:
        path.write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--output-prefix", required=True)
    args = parser.parse_args()
    directory = args.output_directory.resolve()
    path = lambda suffix: directory / (args.output_prefix + "." + suffix)
    load = lambda suffix: json.loads(path(suffix).read_text())
    selection, transport, fixture, metrics = (load(suffix) for suffix in
                                             ("selection.json", "transport.json", "fixture.json", "metrics.json"))
    original = read_bytes(Path(selection["sources"]["raw"]["path"]))
    assert sha(original) == selection["sources"]["raw"]["raw_sha256"]
    for item in selection["outputs"].values():
        data = read_bytes(Path(item["path"]))
        assert len(data) == item["bytes"] and sha(data) == item["sha256"]
    candidates = sorted((dict(frame, selection=kind) for kind in ("selected", "excluded")
                         for frame in selection[kind]), key=lambda frame: frame["source_start"])
    start = selection["window"]["capture_start_seconds"]
    end = selection["window"]["capture_end_seconds"]
    gaps = []
    for before, after in zip(candidates, candidates[1:]):
        distance = (int(after["header"]["seq"]) - int(before["header"]["seq"])) & 0xffffffff
        if not 1 < distance < 0x80000000:
            continue
        keys = ("selection", "source_start", "source_end_exclusive", "header",
                "header_first_byte_receipt", "trailer_last_byte_receipt")
        before_window = before["trailer_last_byte_receipt"]["t"] < start and after["trailer_last_byte_receipt"]["t"] < start
        gaps.append({"missing_first": (int(before["header"]["seq"]) + 1) & 0xffffffff,
                     "missing_last": (int(after["header"]["seq"]) - 1) & 0xffffffff,
                     "missing_count": distance - 1, "both_complete_frames_before_capture_start": before_window,
                     "adjacent_original_byte_ranges": before["source_end_exclusive"] == after["source_start"],
                     "before": {key: before[key] for key in keys}, "after": {key: after[key] for key in keys}})
    counters = transport["whole_raw_parser_counters"]
    assert len(gaps) == counters["sequence_gaps"]
    assert sum(gap["missing_count"] for gap in gaps) == counters["missing_frames"]
    assert all(gap["both_complete_frames_before_capture_start"] for gap in gaps)
    assert all(gap[side]["header_first_byte_receipt"]["recorded_phase"] == "prepare"
               and gap[side]["trailer_last_byte_receipt"]["recorded_phase"] == "prepare"
               for gap in gaps for side in ("before", "after"))
    group = fixture["groups"][0]
    assert group["missing_sequences"] == group["sequence_gaps"] == 0
    assert counters["crc_errors"] == counters["malformed"] == 0
    assert transport["capture_counts"]["identity_errors"] == 0
    validation = load("validation.json")
    comparison = validation["USB_BLE_comparison"]
    assert comparison["same_wire_bytes_sha256"] == group["frames"]
    assert comparison["different_wire_bytes_sha256"] == comparison["USB_frames_without_BLE_match"] == 0
    assert validation["BLE_counts"]["complete_frames"] == group["frames"]
    check = {"schema_version": 1, "scope": "Formal measurement window distinguished from full raw recording",
             "source_raw": selection["sources"]["raw"],
             "selection_sha256": sha(path("selection.json").read_bytes()),
             "generator": {"path": str(Path(__file__).resolve()), "sha256": sha(Path(__file__).read_bytes())},
             "all_extractor_output_hashes_verified": True,
             "host_window_seconds": [start, end], "formal_identity": group["identity"],
             "formal_frames": group["frames"], "formal_first_sequence": group["first_sequence"],
             "formal_last_sequence": group["last_sequence"],
             "strict_device_span_seconds": group["timestamp_span_seconds"], "minimum_seconds": 120.0,
             "formal_sequence_gaps": 0, "formal_missing_sequences": 0,
             "whole_raw_parser_counters": counters, "outside_window_gaps": gaps,
             "outside_gap_cause": "NOT_ESTABLISHED; location does not identify host, USB, queue or other cause",
             "precision_acceptance": "NOT_EVALUATED", "calibration_written": False}
    write_once(path("boundary-check.json"), json.dumps(check, indent=2, sort_keys=True) + "\n")
    lines = [f"# Fixture review: {args.output_prefix.upper()}", "",
             f"Formal USB window: **{group['frames']} frames**, sequences **{group['first_sequence']}–{group['last_sequence']}**, "
             f"device span **{group['timestamp_span_seconds']:.6f} s**, strict requirement **>=120 s met**. "
             "Formal sequence gaps, CRC errors and identity errors are zero. The recorded gate reports an identical complete USB/BLE frame set; invalid point semantics are preserved.", "",
             "The complete raw file is broader than this measurement window. Its parser reports:", ""]
    for gap in gaps:
        lines.append(f"- Missing complete-frame sequence observations **{gap['missing_first']}–{gap['missing_last']}** "
                     f"({gap['missing_count']}), between original byte ranges ending and starting at "
                     f"**{gap['before']['source_end_exclusive']} / {gap['after']['source_start']}**. "
                     f"Both neighboring complete frames were received during `prepare`, before the formal start at **{start:.9f} s**. "
                     "They are retained in the selection manifest as excluded boundary material. This proves their location, not their cause.")
    lines += ["", "Do not describe the whole file as gap-free. See the full byte offsets, headers and receipt events in "
              f"[boundary check]({path('boundary-check.json')}).", ""]
    if group["frame_type"] == "R":
        lines += ["| Point | Valid / invalid n | Mean (ohm) | Observed min–max (ohm) | Mean minus nominal 10000 ohm | Nominal deviation |", "|---|---:|---:|---|---:|---:|"]
        for label in ("S1D1", "S8D8"):
            ref = metrics["groups"][0]["references"][label]
            stats = ref["statistics"]
            lines.append(f"| {label} | {stats['samples']} / {stats['invalid']} | {stats['mean']:.6f} | "
                         f"{stats['min']:.6f}–{stats['max']:.6f} | {ref['delta_from_nominal']:+.6f} ohm | {ref['relative_error_percent']:+.6f}% |")
        counts = transport["capture_counts"]
        assert counts["invalid_points"] == counts["open_points"] == 62 * group["frames"]
        assert transport["capture_invalid_reasons"] == {"X0D": counts["open_points"]}
        lines += ["", f"All **{group['floating_res_pass']}/{group['floating_res_observed']}** designated floating cells meet fresh acquired OPEN checks. "
                  f"The full 64-point stream retains **{counts['invalid_points']}** invalid-numeric **X0D OPEN** observations "
                  f"({counts['open_points']} OPEN; 62 per frame), including the two capacitor positions. "
                  "The two resistor positions have no invalid samples. Complete receipt does not mean every point has a numeric value.", "",
                  "These are observed deviations from a nominal value, not errors against a traceable measured resistor standard. "
                  "Measured reference values, resistor/reference tolerances and acceptance limits were not supplied; both results remain `NEEDS_LIMITS`. "
                  "Min/max and P05–P95 describe sample spread, not confidence intervals or accuracy bounds."]
    else:
        lines += ["CAP values are total LC capacitance. Both capacitor references remain `BASELINE_REQUIRED`: "
                  "the three fixed cross-cell background proxies are descriptive and do not replace the same cell's unloaded baseline. "
                  "The report preserves all row, column and all-floating proxy results; no method is selected because it is closer to 33 pF."]
    lines += ["", f"[Measurement report]({path('report.md')}) · [Metrics]({path('metrics.json')}) · "
              f"[Closure and byte verification]({path('validation.json')})", "",
              "No hardware actions, calibration writes or absolute precision acceptance were performed.", ""]
    write_once(path("review.md"), "\n".join(lines))
    print(json.dumps({"review": str(path("review.md")), "boundary_check": str(path("boundary-check.json")),
                      "formal_missing": 0, "whole_raw_missing": counters["missing_frames"]}))


if __name__ == "__main__":
    main()
