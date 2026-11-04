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
        for s3 in (0, 1):
            binary = Path(temp) / f"test_ads_driver_s3_{s3}"
            subprocess.run([
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                f"-DCONFIG_IDF_TARGET_ESP32S3={s3}",
                "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
                "-I", str(root / "tests/host/ads_stubs"),
                "-I", str(root / "tests/host/include"),
                "-I", str(root / "components/ads126xAdc/include"),
                str(root / "tests/host/test_ads_driver_reliability.c"),
                "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)
            print(f"ADS_SPI_TIMING_SELECTION,passed=1,esp32s3={s3}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
