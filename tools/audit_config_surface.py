#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CONFIG_RE = re.compile(r"\bCONFIG_([A-Z0-9_]+)\b")
KCONFIG_DEF_RE = re.compile(r"^\s*config\s+([A-Z0-9_]+)\b")
FALLBACK_RE = re.compile(r"^\s*#\s*ifndef\s+CONFIG_([A-Z0-9_]+)")

LEGACY = {
    "SENSORARRAY_FDC_CAP_PRINT_DECIMALS",
    "SENSORARRAY_FDC_CAP_DECIMALS",
    "SENSORARRAY_FDC_MATRIX_PERIOD_MS",
    "SENSORARRAY_FDC_MATRIX_TARGET_FPS",
    "SENSORARRAY_FRAME_PERIOD_MS",
    "SENSORARRAY_ENABLE_BLE",
    "SENSORARRAY_BLE_ENABLE",
    "SENSORARRAY_ENABLE_WIRED",
    "SENSORARRAY_FDC_AFTER_INTB_RECHECK_ENABLE",
    "SENSORARRAY_FDC_DISABLE_READY_STATUS_POLL",
    "SENSORARRAY_FDC_SUPPRESS_STATUS_READ_BEFORE_INTB",
}

GROUPS = {
    "cap decimals": ("CAP_DECIMAL", "CAP_PRINT_DECIMAL"),
    "target fps / frame period": ("TARGET_FPS", "PERIOD_MS", "FRAME_PERIOD"),
    "output enable": ("OUTPUT_", "ENABLE_WIRED", "ENABLE_BLE"),
    "log periods": ("LOG_PERIOD", "EVERY_N_FRAMES", "SUMMARY_PERIOD"),
    "ready policy": ("READY_POLICY", "READY_MODE", "DISABLE_READY_STATUS_POLL"),
    "stale unread": ("STALE_UNREAD", "UNREAD_NO_DRDY"),
    "rescue cooldown": ("RESCUE", "COOLDOWN"),
    "core affinity": ("TASK_CORE", "_CORE"),
    "transport enable": ("TRANSPORT", "OUTPUT_WIFI", "OUTPUT_USB", "OUTPUT_BLE"),
    "BLE enable/status/cap": ("BLE",),
    "Wi-Fi enable/mode": ("WIFI", "WI_FI"),
}

SOURCE_EXTS = {".c", ".h", ".inc", ".cmake", ".txt"}
SCAN_FILES = [
    path for path in sorted(ROOT.rglob("Kconfig*"))
    if path.is_file() and "build" not in path.parts and ".git" not in path.parts
]
SCAN_FILES.extend([
    ROOT / "sdkconfig.defaults",
    ROOT / "sdkconfig.defaults.esp32s3",
    ROOT / "sdkconfig.rename",
    ROOT / "main" / "sensorarrayConfig.h",
    ROOT / "main" / "sensorarrayConfigDerived.h",
])


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def iter_source_files() -> list[Path]:
    files: list[Path] = []
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.suffix not in SOURCE_EXTS:
            continue
        rp = rel(path)
        if rp.startswith(("build/", ".git/", ".venv/", "docs/archive/", "example/")):
            continue
        files.append(path)
    return files


def load_text(path: Path) -> str:
    return path.read_text(errors="ignore") if path.exists() else ""


def collect_defined() -> dict[str, list[str]]:
    defined: dict[str, list[str]] = defaultdict(list)
    for path in SCAN_FILES:
        if not path.exists():
            continue
        if path.name == "sdkconfig.rename":
            for line in load_text(path).splitlines():
                for name in CONFIG_RE.findall(line):
                    defined[name].append(rel(path))
            continue
        for lineno, line in enumerate(load_text(path).splitlines(), 1):
            match = KCONFIG_DEF_RE.match(line)
            if match:
                defined[match.group(1)].append(f"{rel(path)}:{lineno}")
    return defined


def collect_defaults() -> set[str]:
    defaults: set[str] = set()
    for path in [ROOT / "sdkconfig.defaults", ROOT / "sdkconfig.defaults.esp32s3"]:
        for name in CONFIG_RE.findall(load_text(path)):
            defaults.add(name)
    return defaults


def collect_used() -> dict[str, list[str]]:
    used: dict[str, list[str]] = defaultdict(list)
    for path in iter_source_files():
        rp = rel(path)
        if rp.startswith("main/kconfig") or rp.startswith("main/kconfigs"):
            continue
        for lineno, line in enumerate(load_text(path).splitlines(), 1):
            for name in CONFIG_RE.findall(line):
                used[name].append(f"{rp}:{lineno}")
    return used


def collect_fallbacks() -> dict[str, list[str]]:
    fallbacks: dict[str, list[str]] = defaultdict(list)
    for path in iter_source_files():
        for lineno, line in enumerate(load_text(path).splitlines(), 1):
            match = FALLBACK_RE.match(line)
            if match:
                fallbacks[match.group(1)].append(f"{rel(path)}:{lineno}")
    return fallbacks


def print_list(title: str, items: list[str], limit: int = 80) -> None:
    print(f"{title}: {len(items)}")
    for item in items[:limit]:
        print(f"  {item}")
    if len(items) > limit:
        print(f"  ... +{len(items) - limit}")


def main() -> int:
    defined = collect_defined()
    defaults = collect_defaults()
    used = collect_used()
    fallbacks = collect_fallbacks()

    defined_names = set(defined)
    used_names = set(used)
    fallback_names = set(fallbacks)

    defined_but_not_used = sorted(defined_names - used_names - defaults)
    used_but_not_defined = sorted(
        name for name in used_names - defined_names
        if not name.startswith(("IDF_", "ESP_", "FREERTOS_", "SOC_", "LWIP_", "BT_", "NVS_", "PARTITION_", "LOG_", "COMPILER_"))
    )
    fallback_but_not_defined = sorted(fallback_names - defined_names)
    defaults_only = sorted(defaults - defined_names)

    legacy_hits = []
    for name in sorted(LEGACY):
        if name in used:
            for where in used[name]:
                if where.startswith("main/sensorarrayConfig.h") or where.startswith("main/sensorarrayConfigDerived.h"):
                    continue
                legacy_hits.append(f"CONFIG_{name} {where}")

    print("configFilesSummary:")
    for path in SCAN_FILES:
        if path.exists():
            print(f"  {rel(path)}")
    print_list("definedButNotUsed", defined_but_not_used)
    print_list("usedButNotDefined", used_but_not_defined)
    print_list("fallbackButNotDefined", fallback_but_not_defined)
    print_list("sdkconfigDefaultsOnly", defaults_only)
    print_list("legacyConfigStillUsedInCode", legacy_hits)

    print("duplicateSemanticGroups:")
    for group, tokens in GROUPS.items():
        names = sorted(
            name for name in defined_names | defaults | fallback_names
            if any(token in name for token in tokens)
        )
        print(f"  {group}: {', '.join(names) if names else '-'}")

    severe = bool(legacy_hits or used_but_not_defined)
    return 1 if severe else 0


if __name__ == "__main__":
    sys.exit(main())
