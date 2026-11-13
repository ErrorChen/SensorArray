#!/usr/bin/env python3
"""Compile the actual boot SPI owner/join with pthread and monotonic-clock shims.

Synthetic timing/failure injection only; this does not claim HIL overlap or SPI
electrical qualification. Production task storage and publication are exercised.
"""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
main_source = (root / "main/main.c").read_text()
source = (root / "core/board/sensorarrayFrontendStartup.c").read_text()
begin = source.index("/* 本轮新增: one boot-only ADS owner.")
end = source.index("static bool sensorarrayAppFdcIdMatches(", begin)
production = source[begin:end]
production += source[source.index("bool sensorarrayFrontendStartupAdsOwnerQuiescent("):]
header = (root / "core/board/sensorarrayAdsSpiProbe.h").read_text()
types = header[header.index("typedef struct {"):header.index("/* 本轮新增: exclusive")]

# The actual caller must not initialize a matrix/cache/route/gap while an
# outstanding boot owner can still be using ADS. A failed join returns first.
frontends = main_source[main_source.index("static esp_err_t sensorarrayInitFrontends("):]
frontends = frontends[:frontends.index("static void sensorarrayInitAsyncLogging(")]
ordered = ["sensorarrayFrontendStartupRun(",
           "sensorarrayFdcMatrixEngineInit(", "sensorarrayAdsMatrixEngineInit(",
           "sensorarrayRouteControllerInit(", "sensorarrayAdsGapInit(",
           "sensorarrayApplyMeasurementMode(",
           "sensorarrayTransportSetRuntimeQueryCallback("]
positions = [frontends.index(token) for token in ordered]
assert positions == sorted(positions), "boot ownership/join caller ordering changed"
assert "return err;" in frontends[positions[0]:positions[1]]
startup = source[source.index("esp_err_t sensorarrayFrontendStartupRun("):]
ordered = ["sensorarrayBringupInitAds(", "sensorarrayBoardEnsureGpioIsrService()",
           "sensorarrayBootSpiStart(",
           "sensorarrayBootSpiNoteI2c(true)", "sensorarrayInitFdcDevice(",
           "sensorarrayBootSpiNoteI2c(false)", "sensorarrayBootSpiJoin()"]
positions = [startup.index(token) for token in ordered]
assert positions == sorted(positions), "parallel startup ordering changed"
assert "return err;" in startup[positions[-1]:]
assert main_source.count('if (strcmp(command, "SPI?") == 0)') == 2, "early and normal SPI? required"
assert "vTaskDelete" not in production, "boot DRDY target task cannot be deleted"
assert "vTaskSuspend(NULL)" in production, "one-shot boot worker must retain its task handle"

test = (root / "tests/host/test_boot_spi_parallel.c").read_text()
test = test.replace("/* PRODUCTION_RESULT_TYPES */", types)
test = test.replace("/* PRODUCTION_BOOT_JOB */", production)
with tempfile.TemporaryDirectory(prefix="sensorarray-boot-spi-") as temp:
    c_file = Path(temp) / "test_boot_spi_parallel.c"
    c_file.write_text(test)
    for enabled in (1, 0):
        executable = Path(temp) / f"boot_spi_{enabled}"
        command = [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                   "-Werror", "-pthread", "-O1", "-g",
                   f"-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE={enabled}",
                   str(c_file), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True, timeout=20)
print("boot SPI production job/join and caller ordering: PASS (enabled + disabled)")
