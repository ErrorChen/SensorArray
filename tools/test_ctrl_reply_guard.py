#!/usr/bin/env python3
"""Compile and execute the firmware's control-reply truncation guard test."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


def findCompiler() -> tuple[str, bool]:
    clionGcc = sorted(
        Path(r"C:\Program Files\JetBrains").glob(r"CLion *\bin\mingw\bin\gcc.exe"),
        reverse=True,
    )
    candidates = [
        os.environ.get("CC", ""),
        shutil.which("clang") or "",
        shutil.which("clang-18") or "",
        shutil.which("gcc") or "",
        str(clionGcc[0]) if clionGcc else "",
        r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe",
    ]
    for candidate in candidates:
        if not candidate or not Path(candidate).is_file():
            continue
        version = subprocess.run(
            [candidate, "--version"], check=True, text=True, capture_output=True
        ).stdout
        native = "Target: riscv32-esp" not in version and "Target: xtensa" not in version
        return candidate, native
    raise RuntimeError("no host C compiler found (set CC or install clang)")


def writeFakeHeaders(fakeDir: Path) -> None:
    (fakeDir / "esp_err.h").write_text(
        "#pragma once\n"
        "typedef int esp_err_t;\n"
        "#define ESP_OK 0\n"
        "#define ESP_FAIL -1\n"
        "#define ESP_ERR_INVALID_ARG 0x102\n"
        "#define ESP_ERR_INVALID_SIZE 0x103\n"
        "#define ESP_ERR_INVALID_STATE 0x104\n"
        "#define ESP_ERR_NOT_FOUND 0x105\n"
        "#define ESP_ERR_NOT_SUPPORTED 0x106\n"
        "#define ESP_ERR_TIMEOUT 0x107\n"
        "#define ESP_ERR_INVALID_RESPONSE 0x10C\n",
        encoding="ascii",
    )
    (fakeDir / "freertos").mkdir()
    (fakeDir / "freertos" / "FreeRTOS.h").write_text(
        "#pragma once\n"
        "typedef int portMUX_TYPE;\n",
        encoding="ascii",
    )
    (fakeDir / "sensorarrayWifi.h").write_text(
        "#pragma once\n"
        "#include <stdbool.h>\n"
        "#include <stddef.h>\n"
        "#include <stdint.h>\n"
        "#include \"esp_err.h\"\n"
        "typedef enum {\n"
        "    SENSORARRAY_WIFI_CH_DATA = 0,\n"
        "    SENSORARRAY_WIFI_CH_LOG = 1,\n"
        "    SENSORARRAY_WIFI_CH_CTRL = 2,\n"
        "} sensorarrayWifiChannel_t;\n"
        "typedef struct { uint32_t ipv4; uint16_t port; } sensorarrayWifiPeer_t;\n"
        "bool sensorarrayWifiIsReady(void);\n",
        encoding="ascii",
    )
    (fakeDir / "sensorarrayBle.h").write_text(
        "#pragma once\n"
        "#include <stdbool.h>\n"
        "#include <stddef.h>\n"
        "#include <stdint.h>\n"
        "#include \"esp_err.h\"\n"
        "typedef enum {\n"
        "    SENSORARRAY_BLE_TX_FAST = 0,\n"
        "    SENSORARRAY_BLE_TX_SAFE,\n"
        "} sensorarrayBleTxMode_t;\n"
        "const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode);\n"
        "sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void);\n"
        "void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode);\n",
        encoding="ascii",
    )
    (fakeDir / "sensorarrayScanConfig.h").write_text(
        "#pragma once\n"
        "#include <stddef.h>\n"
        "#include \"esp_err.h\"\n"
        "esp_err_t sensorarrayScanConfigHandleCommand(const char *command,\n"
        "                                             size_t length,\n"
        "                                             char *response,\n"
        "                                             size_t responseSize);\n",
        encoding="ascii",
    )


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    compiler, native = findCompiler()
    compilerEnvironment = os.environ.copy()
    compilerEnvironment["PATH"] = (
        str(Path(compiler).parent)
        + os.pathsep
        + compilerEnvironment.get("PATH", "")
    )
    sources = [
        root / "tests" / "host" / "test_ctrl_reply_guard.c",
    ]
    with tempfile.TemporaryDirectory(prefix="sensorarray-ctrl-reply-test-") as tempDir:
        temp = Path(tempDir)
        fakeDir = temp / "fakes"
        fakeDir.mkdir()
        writeFakeHeaders(fakeDir)
        common = [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-I",
            str(fakeDir),
            "-I",
            str(root / "core" / "transport"),
        ]
        if not native:
            subprocess.run(
                common
                + ["-c", str(sources[0]), "-o", str(temp / "ctrl-reply.o")],
                check=True,
                env=compilerEnvironment,
            )
            print("CTRL_REPLY_GUARD_COMPILE,passed=1,target=embedded,execution=skipped")
            return 0
        executable = temp / "test_ctrl_reply_guard.exe"
        subprocess.run(
            common + [str(sources[0]), "-o", str(executable)],
            check=True,
            env=compilerEnvironment,
        )
        completed = subprocess.run(
            [str(executable)],
            check=True,
            text=True,
            capture_output=True,
            env=compilerEnvironment,
        )
        print(completed.stdout, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
