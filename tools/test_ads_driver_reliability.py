#!/usr/bin/env python3
"""Run synthetic time/SPI/DRDY failures against the real ADS driver source."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise RuntimeError("host C compiler required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-ads-driver-") as temp:
        binary = Path(temp) / "test_ads_driver"
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-I", str(root / "tests/host/ads_stubs"),
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "components/ads126xAdc/include"),
            str(root / "tests/host/test_ads_driver_reliability.c"),
            "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
