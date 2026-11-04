#!/usr/bin/env python3
"""Execute the firmware's persistent FDC worker ownership state machine."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="sensorarray-worker-") as temp:
    exe = Path(temp) / "worker"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "core/measure/fdc"),
                    str(root / "tests/host/test_fdc_worker_lifecycle.c"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
