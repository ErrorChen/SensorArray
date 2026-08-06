#!/usr/bin/env python3
"""Compile and run hardware-independent mode, math, and PGA tests."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


def find_compiler() -> tuple[str, bool]:
    clion_gcc = sorted(
        Path(r"C:\Program Files\JetBrains").glob(
            r"CLion *\bin\mingw\bin\gcc.exe"
        ),
        reverse=True,
    )
    candidates = [
        os.environ.get("CC", ""),
        shutil.which("clang") or "",
        str(clion_gcc[0]) if clion_gcc else "",
        r"C:\Espressif\tools\esp-clang\esp-19.1.2_20250312\esp-clang\bin\clang.exe",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            version = subprocess.run([candidate, "--version"], check=True,
                                     text=True, capture_output=True).stdout
            native = "Target: riscv32-esp" not in version and "Target: xtensa" not in version
            return candidate, native
    raise RuntimeError("no host C compiler found (set CC or install clang)")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    compiler, native = find_compiler()
    compiler_environment = os.environ.copy()
    compiler_environment["PATH"] = (
        str(Path(compiler).parent) + os.pathsep +
        compiler_environment.get("PATH", "")
    )
    sources = [
        root / "tests" / "host" / "test_measurement_logic.c",
        root / "core" / "measure" / "sensorarrayMeasurementMode.c",
        root / "core" / "measure" / "sensorarrayMeasurementSelfTest.c",
        root / "core" / "measure" / "ads" / "sensorarrayAdsMath.c",
        root / "core" / "measure" / "ads" / "sensorarrayAdsAutoRange.c",
        root / "core" / "measure" / "ads" / "sensorarrayAdsCache.c",
        root / "core" / "measure" / "ads" / "sensorarrayBatteryScheduler.c",
    ]
    with tempfile.TemporaryDirectory(prefix="sensorarray-measure-test-") as temp_dir:
        common = [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-I",
            str(root / "core" / "measure" / "include"),
            "-I",
            str(root / "core" / "measure" / "ads"),
            "-I",
            str(root / "main" / "output"),
        ]
        if not native:
            for index, source in enumerate(sources):
                subprocess.run(common + ["-c", str(source), "-o",
                                          str(Path(temp_dir) / f"logic-{index}.o")],
                               check=True, env=compiler_environment)
            print("MEASUREMENT_LOGIC_COMPILE,passed=1,target=embedded,execution=skipped")
            return 0
        executable = Path(temp_dir) / "test_measurement_logic.exe"
        subprocess.run(common + [*(str(source) for source in sources),
                                 "-o", str(executable)], check=True,
                       env=compiler_environment)
        completed = subprocess.run([str(executable)], check=True, text=True,
                                   capture_output=True, env=compiler_environment)
        print(completed.stdout, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
