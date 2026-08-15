#!/usr/bin/env python3
"""Compile and run the homogeneous ROWMODES first-frame metadata regression."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path


def findCompiler() -> tuple[str, bool]:
    candidates = [
        os.environ.get("CC", ""),
        shutil.which("gcc") or "",
        shutil.which("clang") or "",
    ]
    for candidate in candidates:
        if not candidate or not Path(candidate).is_file():
            continue
        version = subprocess.run(
            [candidate, "--version"], check=True, text=True, capture_output=True
        ).stdout
        native = "Target: riscv32-esp" not in version and "Target: xtensa" not in version
        return candidate, native
    raise RuntimeError("no host C compiler found (set CC or install gcc/clang)")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    compiler, native = findCompiler()
    environment = os.environ.copy()
    environment["PATH"] = (
        str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
    )
    sources = [
        root / "tests" / "host" / "test_rowmodes_frame_metadata.c",
        root / "core" / "measure" / "sensorarrayMeasurementMode.c",
    ]
    with tempfile.TemporaryDirectory(prefix="sensorarray-rowmodes-metadata-") as tempDir:
        temp = Path(tempDir)
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
            str(root / "tests" / "host" / "include"),
        ]
        if not native:
            for index, source in enumerate(sources):
                subprocess.run(
                    common + ["-c", str(source), "-o", str(temp / f"metadata-{index}.o")],
                    check=True,
                    env=environment,
                )
            print("ROWMODES_FRAME_METADATA_COMPILE,passed=1,target=embedded,execution=skipped")
            return 0
        executable = temp / "test_rowmodes_frame_metadata.exe"
        subprocess.run(
            common + [str(source) for source in sources] + ["-o", str(executable)],
            check=True,
            env=environment,
        )
        completed = subprocess.run(
            [str(executable)], check=True, text=True, capture_output=True, env=environment
        )
        print(completed.stdout, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
