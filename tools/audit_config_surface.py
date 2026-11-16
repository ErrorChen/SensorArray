#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CONFIG_RE = re.compile(r"\bCONFIG_([A-Z0-9_]+)\b")
KCONFIG_DEF_RE = re.compile(r"^\s*(?:menuconfig|config)\s+([A-Z0-9_]+)\b")
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

SOURCE_EXTS = {".c", ".h", ".inc", ".cmake"}
C_SOURCE_EXTS = {".c", ".h", ".inc"}
# Match whole C literals/comments before looking for identifiers. Replacing
# with spaces preserves diagnostic line numbers; continued // comments remain
# comments after the C translation phase that splices backslash-newline.
C_NON_CODE_RE = re.compile(
    r'//(?:\\\r?\n|[^\n])*|/\*[\s\S]*?\*/'
    r'|"(?:\\[\s\S]|[^"\\])*"'
    r"|'(?:\\[\s\S]|[^'\\])*'"
)
SDK_COMPAT_PREFIXES = (
    "IDF_", "ESP_", "FREERTOS_", "SOC_", "LWIP_", "BT_", "NVS_",
    "PARTITION_", "LOG_", "COMPILER_",
)
# Exact SDK symbol, not a SPIRAM_* or project-prefix exemption. Used only
# without an installed SDK. Verified against ESP-IDF v5.5.5:
# components/esp_psram/esp32s3/Kconfig.spiram:1 (config SPIRAM).
SDK_COMPAT_NAMES = {"SPIRAM"}


def is_generated_build_path(path: Path) -> bool:
    """Exclude ESP-IDF output without hiding any source-tree configuration."""
    relative_parts = path.relative_to(ROOT).parts
    if not relative_parts:
        return False
    top_level = relative_parts[0]
    return top_level == "build" or top_level.startswith(("build_", "build-"))


def iter_kconfig_files() -> list[Path]:
    """Keep managed dependency declarations available to project references."""
    return [
        path for path in sorted(ROOT.rglob("Kconfig*"))
        if path.is_file() and not is_generated_build_path(path) and ".git" not in path.parts
    ]


SCAN_FILES = iter_kconfig_files()
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
        if not path.is_file() or (path.suffix not in SOURCE_EXTS and path.name != "CMakeLists.txt"):
            continue
        if is_generated_build_path(path):
            continue
        rp = rel(path)
        # IDF-managed downloads contain unbuilt examples and ordinary C
        # identifiers such as CONFIG_TOTAL_LEN / CONFIG_COMPLETE. Their code
        # is outside this project's usage audit, just like IDF source itself.
        # Exclude only this root directory, not owned components or symbol
        # prefixes; iter_kconfig_files still supplies dependency declarations.
        if rp.startswith(("build/", ".git/", ".venv/", "docs/archive/", "example/",
                          "managed_components/")):
            continue
        files.append(path)
    return files


def load_text(path: Path) -> str:
    return path.read_text(errors="ignore") if path.exists() else ""


def source_code(path: Path) -> str:
    text = load_text(path)
    if path.suffix in C_SOURCE_EXTS:
        return C_NON_CODE_RE.sub(
            lambda match: "".join("\n" if char == "\n" else " " for char in match.group()),
            text,
        )
    return text


def find_sdk_root() -> Path | None:
    candidates: list[Path] = []
    if os.environ.get("IDF_PATH"):
        candidates.append(Path(os.environ["IDF_PATH"]))
    try:
        description = json.loads(load_text(ROOT / "build" / "project_description.json"))
        if isinstance(description, dict) and description.get("idf_path"):
            candidates.append(Path(description["idf_path"]))
    except (ValueError, TypeError):
        pass
    for candidate in candidates:
        if (candidate / "components").is_dir() and (candidate / "Kconfig").is_file():
            return candidate
    return None


def collect_sdk_defined(sdk_root: Path | None) -> set[str]:
    """Recognise declared SDK symbols; enabled values are not declarations."""
    defined: set[str] = set()
    if sdk_root is not None:
        paths = [sdk_root / "Kconfig"]
        paths.extend((sdk_root / "components").rglob("Kconfig*"))
        for path in paths:
            if not path.is_file():
                continue
            for line in load_text(path).splitlines():
                match = KCONFIG_DEF_RE.match(line)
                if match:
                    defined.add(match.group(1))
    return defined


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
        for lineno, line in enumerate(source_code(path).splitlines(), 1):
            for name in CONFIG_RE.findall(line):
                used[name].append(f"{rp}:{lineno}")
    return used


def collect_fallbacks() -> dict[str, list[str]]:
    fallbacks: dict[str, list[str]] = defaultdict(list)
    for path in iter_source_files():
        for lineno, line in enumerate(source_code(path).splitlines(), 1):
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

    sdk_root = find_sdk_root()
    sdk_defined = collect_sdk_defined(sdk_root)
    defined_names = set(defined)
    used_names = set(used)
    fallback_names = set(fallbacks)

    defined_but_not_used = sorted(defined_names - used_names - defaults)
    used_but_not_defined = sorted(
        name for name in used_names - defined_names - sdk_defined
        if sdk_root is not None or
        (name not in SDK_COMPAT_NAMES and not name.startswith(SDK_COMPAT_PREFIXES))
    )
    fallback_but_not_defined = sorted(fallback_names - defined_names - sdk_defined)
    defaults_only = sorted(defaults - defined_names)

    legacy_hits = []
    for name in sorted(LEGACY):
        if name in used:
            for where in used[name]:
                if where.startswith("main/sensorarrayConfig.h") or where.startswith("main/sensorarrayConfigDerived.h"):
                    continue
                legacy_hits.append(f"CONFIG_{name} {where}")

    print(f"sdkConfigSource: {sdk_root if sdk_root else 'unavailable (SDK compatibility exceptions)'}")
    print("sourceUsageScope: project sources; managed_components source uses excluded, Kconfig declarations retained")
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
