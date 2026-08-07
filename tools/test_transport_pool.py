#!/usr/bin/env python3
"""Compile and execute the firmware's portable transport pool/policy tests."""

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
        root / "tests" / "host" / "test_transport_pool.c",
        root / "core" / "transport" / "sensorarrayTransportPool.c",
        root / "core" / "transport" / "sensorarrayTransportPolicy.c",
    ]
    with tempfile.TemporaryDirectory(prefix="sensorarray-transport-test-") as tempDir:
        common = [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-I",
            str(root / "core" / "transport"),
        ]
        if not native:
            for index, source in enumerate(sources):
                subprocess.run(
                    common
                    + ["-c", str(source), "-o", str(Path(tempDir) / f"pool-{index}.o")],
                    check=True,
                    env=compilerEnvironment,
                )
            print("TRANSPORT_POOL_COMPILE,passed=1,target=embedded,execution=skipped")
            return 0
        executable = Path(tempDir) / "test_transport_pool.exe"
        subprocess.run(
            common + [*(str(source) for source in sources), "-o", str(executable)],
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
