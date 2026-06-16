#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

LEGACY = {
    "CONFIG_SENSORARRAY_FDC_CAP_PRINT_DECIMALS",
    "CONFIG_SENSORARRAY_FDC_CAP_DECIMALS",
    "CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS",
    "CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS",
    "CONFIG_SENSORARRAY_FRAME_PERIOD_MS",
    "CONFIG_SENSORARRAY_ENABLE_BLE",
    "CONFIG_SENSORARRAY_BLE_ENABLE",
    "CONFIG_SENSORARRAY_ENABLE_WIRED",
    "CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_ENABLE",
    "CONFIG_SENSORARRAY_FDC_DISABLE_READY_STATUS_POLL",
    "CONFIG_SENSORARRAY_FDC_SUPPRESS_STATUS_READ_BEFORE_INTB",
}

CODE_EXTS = {".c", ".h", ".inc", ".cmake", ".txt"}
SKIP_DIRS = {"build", ".git", ".venv", "docs/archive"}
SKIP_FILES = {
    Path("main/sensorarrayConfig.h"),
    Path("main/sensorarrayConfigDerived.h"),
}


def iter_files() -> list[Path]:
    files: list[Path] = []
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.suffix not in CODE_EXTS:
            continue
        rel = path.relative_to(ROOT)
        if rel in SKIP_FILES:
            continue
        rel_posix = rel.as_posix()
        if any(rel_posix == d or rel_posix.startswith(d + "/") for d in SKIP_DIRS):
            continue
        if "/kconfigs/" in rel_posix or rel_posix.endswith("Kconfig.projbuild"):
            continue
        files.append(path)
    return files


def main() -> int:
    pattern = re.compile(r"\b(" + "|".join(re.escape(name) for name in sorted(LEGACY)) + r")\b")
    hits: list[tuple[Path, int, str]] = []
    for path in iter_files():
        rel = path.relative_to(ROOT)
        for lineno, line in enumerate(path.read_text(errors="ignore").splitlines(), 1):
            if pattern.search(line):
                hits.append((rel, lineno, line.strip()))

    if hits:
        print("legacyConfigStillUsedInCode:")
        for rel, lineno, line in hits:
            print(f"  {rel}:{lineno}: {line}")
        return 1

    print("legacyConfigStillUsedInCode: none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
