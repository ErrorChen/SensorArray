#!/usr/bin/env python3
"""Execute production async-log storage allocation policy under a fake heap."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise RuntimeError("host C compiler required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-log-storage-") as tmp:
        binary = Path(tmp) / "test_async_log_storage"
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(root / "tests/host/workspace_stubs"),
            "-I", str(root / "main/output"),
            str(root / "tests/host/test_async_log_storage.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
