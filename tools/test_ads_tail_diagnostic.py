#!/usr/bin/env python3
"""Execute production55+GPIO1 driver and exact board ownership functions.

Synthetic GPIO/SPI only: does not claim pad timing, no glitches, or HIL.
"""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import tempfile
from test_main_config_commit import function


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", help="compile that commit's actual driver as a negative control")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        raise RuntimeError("native C compiler required")
    source = (root / "core/board/sensorarrayBringup.c").read_text()
    bodies = "\n".join(function(source, name) for name in (
        "static bool sensorarrayAdsDedicatedSpiOwner(",
        "static esp_err_t sensorarrayInitSpi(",
    ))
    harness = (root / "tests/host/test_ads_tail_diagnostic.c").read_text()
    harness = harness.replace('"test_ads_driver_reliability.c"', '"driver_bus.h"')
    harness = harness.replace("/* PRODUCTION_BOARD_OWNERSHIP */", bodies)
    with tempfile.TemporaryDirectory(prefix="sensorarray-tail-") as directory:
        p = Path(directory)
        driver = subprocess.check_output(["git", "show", args.baseline +
            ":components/ads126xAdc/ads126xAdc.c"], cwd=root, text=True) if args.baseline else \
            (root / "components/ads126xAdc/ads126xAdc.c").read_text()
        (p / "production_ads.c").write_text(driver)
        bus = (root / "tests/host/test_ads_driver_reliability.c").read_text().replace(
            '"../../components/ads126xAdc/ads126xAdc.c"', '"production_ads.c"')
        (p / "driver_bus.h").write_text(bus)
        (p / "test.c").write_text(harness)
        cmd = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-DCONFIG_IDF_TARGET_ESP32S3=1", "-DCONFIG_BOARD_SPI_HOST=2",
            "-DCONFIG_BOARD_SPI_SCLK_GPIO=47", "-DCONFIG_BOARD_SPI_MOSI_GPIO=21",
            "-DCONFIG_BOARD_SPI_MISO_GPIO=14", "-DCONFIG_BOARD_ADS126X_CS_GPIO=-1",
            "-DCONFIG_SENSORARRAY_SPI_USE_DMA=1", "-DCONFIG_ADS126X_SPI_CLOCK_HZ=2000000",
            "-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE=1",
            "-DCONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES=64"]
        for path in (p, root / "tests/host/ads_stubs", root / "tests/host/include",
                     root / "components/ads126xAdc/include"):
            cmd += ["-I", str(path)]
        subprocess.run(cmd + [str(p / "test.c"), "-o", str(p / "test")], check=True)
        subprocess.run([str(p / "test")], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
