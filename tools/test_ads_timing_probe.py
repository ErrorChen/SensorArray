#!/usr/bin/env python3
"""Execute production timing probe, Gap wrapper and ADS driver on a synthetic bus."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        raise RuntimeError("native C compiler required")
    source = (root / "core/measure/ads/sensorarrayAdsGap.c").read_text()
    functions = [function(source, name) for name in (
        "static void sensorarrayAdsTransactionBegin(",
        "static void sensorarrayAdsTransactionEnd(",
        "static bool sensorarrayAdsSnapshotFromRegisterCache(",
        "static uint8_t sensorarrayAdsBatteryRegisterMask(",
        "static bool sensorarrayAdsBatterySnapshotsMatch(",
        "static void sensorarrayAdsPopulateRestoredRegisterCache(",
        "esp_err_t sensorarrayAdsGapRunTimingProbeKind(",
        "esp_err_t sensorarrayAdsGapRunTimingProbe(",
    )]
    # The populate helper has a forward declaration; extract its actual body.
    marker = "static void sensorarrayAdsPopulateRestoredRegisterCache("
    functions[-3] = function(source[source.rindex(marker):], marker)
    end = source.index("} sensorarrayAdsBatteryCompare_t;") + len("} sensorarrayAdsBatteryCompare_t;")
    comparison = source[source.rindex("typedef struct {", 0, end):end]
    driver_bus = (root / "tests/host/test_ads_driver_reliability.c").read_text()
    driver_bus = driver_bus.replace('"../../components/ads126xAdc/ads126xAdc.c"',
        '"' + str(root / "components/ads126xAdc/ads126xAdc.c") + '"')
    driver_bus = driver_bus.replace("uint32_t ulTaskNotifyTake(", "uint32_t driverDefaultNotify(")
    driver_bus = driver_bus.replace("int gpio_get_level(", "int driverDefaultGpioLevel(")
    driver_bus = driver_bus.replace("int64_t esp_timer_get_time(", "int64_t driverDefaultTimer(")
    # Model this target's actual100Hz tick in the reused synthetic RTOS bus.
    driver_bus = driver_bus.replace("ticks * 1000u", "ticks * (1000000u / configTICK_RATE_HZ)")
    driver_bus = driver_bus.replace("ticks * 1000", "ticks * (1000000u / configTICK_RATE_HZ)")
    driver_bus = driver_bus.replace("if (desired_hz == 1900000)",
        "if (desired_hz == 550000) { return 551724; }\n    if (desired_hz == 1900000)")
    harness = (root / "tests/host/test_ads_timing_probe.c").read_text()
    harness = harness.replace("/* PRODUCTION_GAP_TYPES */", comparison)
    harness = harness.replace("/* PRODUCTION_GAP_FUNCTIONS */", "\n".join(functions))
    with tempfile.TemporaryDirectory(prefix="sensorarray-timing-probe-") as directory:
        temp = Path(directory)
        (temp / "driver_bus.h").write_text(driver_bus)
        shutil.copytree(root / "tests/host/ads_stubs/freertos", temp / "freertos")
        (temp / "freertos/FreeRTOS.h").write_text(
            (root / "tests/host/ads_stubs/freertos/FreeRTOS.h").read_text()
            .replace("configTICK_RATE_HZ 1000u", "configTICK_RATE_HZ 100u"))
        (temp / "test.c").write_text(harness)
        command = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-DCONFIG_IDF_TARGET_ESP32S3=1", "-DCONFIG_SENSORARRAY_ADS_START_GPIO=48", "-DCONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US=600"]
        for path in (temp, root / "tests/host/ads_stubs", root / "tests/host/include",
                     root / "components/ads126xAdc/include", root / "core/measure/ads",
                     root / "core/measure/include", root / "core/config", root):
            command += ["-I", str(path)]
        binary = temp / "test"
        subprocess.run(command + [str(temp / "test.c"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
