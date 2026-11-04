#!/usr/bin/env python3
from pathlib import Path
import os
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="sensorarray-frame-meta-") as temp:
    exe = Path(temp) / "test"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "core/transport"), str(root / "tests/host/test_transport_frame_metadata.c"),
        str(root / "core/transport/sensorarrayTransportFrameMeta.c"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
