#!/usr/bin/env python3
"""Execute production rail profile/save/restore against a synthetic register bank."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "core/measure/ads/sensorarrayAdsGap.c").read_text()
    harness = (root / "tests/host/test_ads_rail_profile.c").read_text()
    definitions = []
    for name in ("sensorarrayAdsSavedState_t", "sensorarrayAdsRailDiag_t",
                 "sensorarrayAdsBatteryCompare_t", "sensorarrayAdsFreshSample_t"):
        end = source.index("} " + name + ";") + len("} " + name + ";")
        definitions.append(source[source.rindex("typedef struct {", 0, end):end])
    harness = harness.replace("/* INSERT_PRODUCTION_TYPES */", "\n".join(definitions))
    names = (
        "static uint8_t sensorarrayAdsBatteryRegisterMask(",
        "static bool sensorarrayAdsBatterySnapshotsMatch(",
        "static esp_err_t sensorarrayAdsSaveState(",
        "static esp_err_t sensorarrayAdsWriteCalibration(",
        "static esp_err_t sensorarrayAdsRestoreState(",
        "static esp_err_t sensorarrayAdsVerifyRailMonitorProfile(",
        "static esp_err_t sensorarrayAdsReadRailFreshAdc1(",
        "static esp_err_t sensorarrayAdsReadRailMonitorUv(",
    )
    driver = (root / "components/ads126xAdc/ads126xAdc.c").read_text()
    driver_function = function(driver, "esp_err_t ads126xAdcReadAdc1FreshChecked(")
    harness = harness.replace("/* INSERT_PRODUCTION_FUNCTIONS */", driver_function + "\n" + "\n".join(function(source, name) for name in names))
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise RuntimeError("native C compiler required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-rail-profile-") as directory:
        temp = Path(directory)
        generated = temp / "test.c"
        generated.write_text(harness)
        executable = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
        for path in (root / "tests/host/ads_stubs", root / "tests/host/include",
                     root / "components/ads126xAdc/include", root / "core/measure/ads",
                     root / "core/measure/include", root / "core/config"):
            command += ["-I", str(path)]
        subprocess.run(command + [str(generated), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
