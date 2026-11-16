#!/usr/bin/env python3
"""Run real metadata extraction, pinned-old/new differential, and wire regressions."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import zlib

from text_protocol import TextProtocolParser

ROOT = Path(__file__).resolve().parents[1]
BASELINE_SHA256 = "3b6d1df8ceeeee99d312d543d5e6b0b7b70f054a4e033aefa0bf188479ea6fcf"


def verify_wire(path: Path) -> None:
    data = path.read_bytes()
    body, trailer = data.rsplit(b"K,", 1)
    fields = dict(field.split(b"=", 1) for field in trailer.strip().split(b",") if b"=" in field)
    assert int(fields[b"crc"], 16) == zlib.crc32(body), path
    protocol = TextProtocolParser()
    frames = []
    for line in data.decode("ascii").splitlines():
        frame = protocol.feed_line(line)
        if frame is not None:
            frames.append(frame)
    assert len(frames) == 1 and frames[0].crc_ok, path
    assert protocol.counters.malformed == 0, path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", action="store_true",
                        help="Also report old/new host elapsed time; not ESP32 CPU or BLE performance")
    parser.add_argument("--sanitize", action="store_true",
                        help="Execute the boundary corpus with host address/undefined-behavior sanitizers")
    args = parser.parse_args()
    baseline_dir = ROOT / "tests/fixtures/transport_frame_meta"
    baseline = baseline_dir / "baseline.c.txt"
    fixture_info = json.loads((baseline_dir / "baseline.json").read_text())
    assert fixture_info["sha256"] == BASELINE_SHA256
    assert hashlib.sha256(baseline.read_bytes()).hexdigest() == BASELINE_SHA256
    production = ROOT / "core/transport/sensorarrayTransportFrameMeta.c"
    old_text, new_text = baseline.read_text(), production.read_text()
    # This optimization owns only metadata extraction. Keep the old helper
    # and separate preview code byte-for-byte, not merely semantically close.
    assert old_text.split("static uint32_t u32(", 1)[0] == new_text.split("enum {", 1)[0]
    marker = "bool sensorarrayTransportBuildPreview("
    assert old_text[old_text.index(marker):] == new_text[new_text.index(marker):]
    fixtures = sorted((ROOT / "tests/fixtures/config_identity").glob("*.frame"))
    fixtures += sorted((ROOT / "tests/fixtures/reference_capture").glob("*.recorded.txt"))
    assert fixtures
    wire_hashes = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in fixtures}
    for path in fixtures:
        verify_wire(path)
    compiler = os.environ.get("CC", "cc")
    flags = ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror"]
    if args.sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    includes = ["-I", str(ROOT / "core/transport")]
    with tempfile.TemporaryDirectory(prefix="sensorarray-frame-meta-") as directory:
        temp = Path(directory)
        legacy = temp / "legacy_regression"
        subprocess.run([compiler, *flags, *includes,
                        str(ROOT / "tests/host/test_transport_frame_metadata.c"),
                        str(production), "-o", str(legacy)], check=True)
        subprocess.run([str(legacy)], check=True)
        old_object = temp / "baseline.o"
        subprocess.run([compiler, *flags, *includes,
                        "-DsensorarrayTransportParseFrameMeta=sensorarrayTransportBaselineParseFrameMeta",
                        "-DsensorarrayTransportBuildPreview=sensorarrayTransportBaselineBuildPreview",
                        "-x", "c", "-c", str(baseline), "-o", str(old_object)], check=True)
        executable = temp / "differential"
        subprocess.run([compiler, *flags, *includes,
                        str(ROOT / "tests/host/test_transport_frame_metadata_differential.c"),
                        str(production), str(old_object), "-o", str(executable)], check=True)
        command = [str(executable)]
        if args.benchmark:
            command.append("--benchmark")
        subprocess.run(command + [str(path) for path in fixtures], check=True)
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == digest
               for path, digest in wire_hashes.items()), "wire fixture changed during regression"
    print(f"TRANSPORT_METADATA_BASELINE_SHA,sha256={BASELINE_SHA256},git_history_required=0")
    print(f"TRANSPORT_METADATA_WIRE_PASS,frames={len(fixtures)},crc=zlib_and_real_parser,bytes=unchanged")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
