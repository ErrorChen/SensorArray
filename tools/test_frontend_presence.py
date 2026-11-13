#!/usr/bin/env python3
"""Run the production frontend-presence policy/I/O adapter on a synthetic bus."""
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
    with tempfile.TemporaryDirectory(prefix="sensorarray-presence-") as temp:
        directory = Path(temp)
        (directory / "freertos").mkdir()
        (directory / "freertos/FreeRTOS.h").write_text(
            "#pragma once\ntypedef int portMUX_TYPE;\n#define portMUX_INITIALIZER_UNLOCKED 0\n"
            "void presenceTestEnter(void); void presenceTestExit(void);\n"
            "#define portENTER_CRITICAL(x) ((void)(x), presenceTestEnter())\n"
            "#define portEXIT_CRITICAL(x) ((void)(x), presenceTestExit())\n"
        )
        (directory / "sensorarrayTypes.h").write_text("""#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#define ADS126X_POWER_RESET 0x10u
typedef struct { bool quarantined; } ads126xAdcHandle_t;
bool ads126xAdcIsQuarantined(const ads126xAdcHandle_t *);
typedef struct { bool Enabled, Installed, Offline, Recovering;
    uint32_t FrequencyHz, TransactionCount; } BoardSupportI2cCtx_t;
typedef struct Fdc2214CapDevice Fdc2214CapDevice_t;
typedef int Fdc2214CapChannel_t;
typedef struct { int unused; } sensorarrayFdcInitDiag_t;
typedef struct { const BoardSupportI2cCtx_t *i2cCtx; uint8_t i2cAddr; bool ready; } sensorarrayFdcDeviceState_t;
typedef struct { ads126xAdcHandle_t ads; bool adsReady;
    sensorarrayFdcDeviceState_t fdcPrimary, fdcSecondary; } sensorarrayState_t;
esp_err_t boardSupportSetI2cFrequency(const BoardSupportI2cCtx_t *, uint32_t);
esp_err_t boardSupportI2cReadReg16Sequence(void *, uint8_t, const uint8_t *, size_t, uint16_t *, uint32_t);
""")
        binary = directory / "presence"
        subprocess.run([
            cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", temp,
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "tests/host/ads_stubs"),
            "-I", str(root / "core/board"),
            "-I", str(root / "main"),
            str(root / "core/board/sensorarrayFrontendPresence.c"),
            str(root / "tests/host/test_frontend_presence.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
