#!/usr/bin/env python3
"""Compile and run frame acquisition/timing semantics C tests on the host."""

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
        shutil.which("gcc") or shutil.which("cc") or "",
        str(clion_gcc[0]) if clion_gcc else "",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            version = subprocess.run(
                [candidate, "--version"], check=True, text=True,
                capture_output=True,
            ).stdout
            native = "Target: riscv32-esp" not in version and \
                "Target: xtensa" not in version
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
        root / "tests" / "host" / "test_frame_semantics.c",
        root / "core" / "measure" / "sensorarrayFrameBuilder.c",
    ]
    common = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(root / "tests" / "host" / "frame_stubs"),
        "-I",
        str(root / "tests" / "host" / "include"),
        "-I",
        str(root / "main"),
        "-I",
        str(root / "core" / "measure" / "include"),
        "-I",
        str(root / "core" / "config"),
        "-I",
        str(root / "core" / "boardSupport" / "include"),
        "-I",
        str(root / "components" / "ads126xAdc" / "include"),
        "-I",
        str(root / "components" / "fdc2214Cap" / "include"),
        "-I",
        str(root / "components" / "tmuxSwitch" / "include"),
    ]
    with tempfile.TemporaryDirectory(
        prefix="sensorarray-frame-semantics-test-"
    ) as temp_dir:
        if not native:
            for index, source in enumerate(sources):
                subprocess.run(
                    common + ["-c", str(source), "-o",
                              str(Path(temp_dir) / f"frame-{index}.o")],
                    check=True,
                    env=compiler_environment,
                )
            print("FRAME_SEMANTICS_COMPILE,passed=1,target=embedded,execution=skipped")
            return 0
        executable = Path(temp_dir) / "test_frame_semantics.exe"
        subprocess.run(
            common + [*(str(source) for source in sources),
                      "-o", str(executable)],
            check=True,
            env=compiler_environment,
        )
        completed = subprocess.run(
            [str(executable)], check=True, text=True, capture_output=True,
            env=compiler_environment,
        )
        print(completed.stdout, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
