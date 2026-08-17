#!/usr/bin/env python3
"""Compile and execute the firmware's FDCISO mailbox test."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


def findCompiler() -> tuple[str, bool]:
    candidates = [
        os.environ.get("CC", ""),
        shutil.which("clang") or "",
        shutil.which("clang-18") or "",
        shutil.which("gcc") or "",
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
    (fakeDir / "freertos").mkdir()
    (fakeDir / "freertos" / "FreeRTOS.h").write_text(
        "#pragma once\n"
        "#include <stdint.h>\n"
        "typedef int32_t BaseType_t;\n"
        "typedef uint32_t UBaseType_t;\n"
        "typedef struct QueueDefinition *QueueHandle_t;\n"
        "typedef struct StaticQueueDefinition { unsigned char dummy; } StaticQueue_t;\n"
        "#define pdTRUE 1\n"
        "#define pdFALSE 0\n"
        "#define pdPASS 1\n"
        "#define pdFAIL 0\n"
        "typedef struct { int locked; } portMUX_TYPE;\n"
        "#define portMUX_INITIALIZER_UNLOCKED {0}\n"
        "static inline void portENTER_CRITICAL(portMUX_TYPE *mux) { (void)mux; }\n"
        "static inline void portEXIT_CRITICAL(portMUX_TYPE *mux) { (void)mux; }\n",
        encoding="ascii",
    )
    (fakeDir / "freertos" / "queue.h").write_text(
        "#pragma once\n"
        '#include "FreeRTOS.h"\n'
        "QueueHandle_t xQueueCreateStatic(UBaseType_t queueLength,\n"
        "                                 UBaseType_t itemSize,\n"
        "                                 void *storage,\n"
        "                                 StaticQueue_t *queueStruct);\n"
        "BaseType_t xQueueSend(QueueHandle_t queue,\n"
        "                      const void *item,\n"
        "                      BaseType_t ticksToWait);\n"
        "BaseType_t xQueueReceive(QueueHandle_t queue,\n"
        "                         void *item,\n"
        "                         BaseType_t ticksToWait);\n"
        "UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t queue);\n",
        encoding="ascii",
    )
    (fakeDir / "sensorarrayConfig.h").write_text(
        "#pragma once\n"
        "#define CONFIG_SENSORARRAY_BLE_CAP_TEXT_EVERY_N_FRAMES 1u\n"
        "#define CONFIG_SENSORARRAY_CAPTURE_FPS_LIMIT_ENABLE 0\n"
        "#define CONFIG_SENSORARRAY_CAPTURE_FPS_LIMIT 0u\n"
        "#define CONFIG_SENSORARRAY_OUTPUT_RATE_LIMIT_ENABLE 0\n"
        "#define CONFIG_SENSORARRAY_OUTPUT_RATE_LIMIT 0u\n",
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
    source = root / "tests" / "host" / "test_fdc_iso_mailbox.c"
    with tempfile.TemporaryDirectory(prefix="sensorarray-fdc-iso-mailbox-test-") as tempDir:
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
            "-I",
            str(root / "core" / "measure" / "include"),
            "-I",
            str(root / "main" / "control"),
            "-I",
            str(root / "tests" / "host" / "include"),
        ]
        if not native:
            subprocess.run(
                common + ["-c", str(source), "-o", str(temp / "fdc-iso.o")],
                check=True,
                env=compilerEnvironment,
            )
            print("FDC_ISO_MAILBOX_COMPILE,passed=1,target=embedded,execution=skipped")
            return 0
        executable = temp / "test_fdc_iso_mailbox.exe"
        subprocess.run(
            common + [str(source), "-o", str(executable)],
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
