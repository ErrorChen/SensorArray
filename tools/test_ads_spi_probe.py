#!/usr/bin/env python3
"""Execute the actual boot SPI probe and driver against a synthetic DMA/DRDY bus."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        raise RuntimeError("host C compiler required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-spi-probe-") as temp:
        directory = Path(temp)
        (directory / "sensorarrayTypes.h").write_text(
            '#pragma once\n#include "ads126xAdc.h"\n'
            'typedef struct { ads126xAdcHandle_t ads; bool adsReady; } sensorarrayState_t;\n'
        )
        binary = directory / "test_ads_spi_probe"
        command = [
            cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-DCONFIG_ADS126X_SPI_CLOCK_HZ=2000000",
            "-DCONFIG_SENSORARRAY_ADS_START_GPIO=48",
            "-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES=400",
            "-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES=1600",
            "-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES=32",
            "-I", temp, "-I", str(root / "tests/host/ads_stubs"),
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "components/ads126xAdc/include"),
            str(root / "tests/host/test_ads_spi_probe.c"), "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
        command.remove("-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES=400")
        command += ["-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES=128",
                    "-DPROBE_TEST_CONFIG_INSUFFICIENT=1"]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
