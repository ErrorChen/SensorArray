#!/usr/bin/env python3
"""Fail when user-facing command literals drift out of the command reference.

The firmware deliberately has several parser layers.  This checker extracts
the string literals compared by those parsers instead of maintaining a second
hand-written command registry in Python.  Prefix literals such as ``MODE=``
are valid entries: the documentation must contain the prefix and its accepted
value aliases.
"""

from __future__ import print_function

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Set


ROOT = Path(__file__).resolve().parents[1]
COMMAND_DOC = ROOT / "docs" / "command-reference.md"
SOURCE_FILES = (
    ROOT / "core" / "config" / "sensorarrayScanConfig.c",
    ROOT / "core" / "transport" / "sensorarrayTransportCommand.c",
    ROOT / "main" / "main.c",
    ROOT / "main" / "control" / "sensorarrayCommandMailbox.c",
)

# Command variables used by the four parser layers.  Restricting the first
# argument avoids extracting response/log strings from unrelated strcmp calls.
COMMAND_COMPARE_RE = re.compile(
    r"\b(?:strcmp|strncmp)\s*\(\s*(?:text|command|commandText)\s*,\s*"
    r'"([^"\\]*(?:\\.[^"\\]*)*)"',
    re.MULTILINE,
)

# MODE and BATPERIOD aliases are compared after a command prefix is removed.
VALUE_COMPARE_RE = re.compile(
    r"\b(?:strcmp|strncmp)\s*\(\s*value\s*,\s*"
    r'"([^"\\]*(?:\\.[^"\\]*)*)"',
    re.MULTILINE,
)

# FPSCAP/OUTCAP prefixes are parameters to a shared helper rather than direct
# strcmp/strncmp operands at their call sites.
FPS_PREFIX_RE = re.compile(
    r"sensorarrayCommandParseFpsCap\s*\(\s*commandText\s*,\s*"
    r'"([^"\\]*(?:\\.[^"\\]*)*)"',
    re.MULTILINE | re.DOTALL,
)

NYI_LITERALS = {
    "WIFI=STA",
    "WIFI=APSTA",
    "WIFI_SCAN",
    "WIFI_STA_SSID=",
    "WIFI_STA_PASS=",
    "WIFI_SAVE=1",
    "WIFI_CONNECT=1",
    "WIFI_FORGET=1",
}

REQUIRED_COLUMNS = (
    "Command",
    "Type",
    "Serial",
    "BLE FF10",
    "Wi-Fi CTRL",
    "Arguments",
    "Response",
    "Apply timing",
    "Persistent",
    "Status",
    "Notes",
)

# A sudden large drop means a parser was moved/renamed and the extractor must
# be updated; silently passing with zero commands would defeat the guard.
MINIMUM_EXPECTED_LITERALS = 50


def decode_c_literal(value: str) -> str:
    """Decode the small ASCII escape subset used by command literals."""
    return bytes(value, "utf-8").decode("unicode_escape")


def add_matches(found: Dict[str, List[str]], source: Path,
                patterns: Iterable[re.Pattern]) -> None:
    text = source.read_text(encoding="utf-8", errors="strict")
    relative = source.relative_to(ROOT).as_posix()
    for pattern in patterns:
        for match in pattern.finditer(text):
            literal = decode_c_literal(match.group(1))
            found.setdefault(literal, []).append(relative)


def extract_literals() -> Dict[str, List[str]]:
    found: Dict[str, List[str]] = {}
    for source in SOURCE_FILES:
        if not source.is_file():
            raise FileNotFoundError("command source missing: {}".format(source))
        patterns = [COMMAND_COMPARE_RE]
        if source.name in ("main.c", "sensorarrayCommandMailbox.c"):
            patterns.append(VALUE_COMPARE_RE)
        if source.name == "sensorarrayCommandMailbox.c":
            patterns.append(FPS_PREFIX_RE)
        add_matches(found, source, patterns)
    return found


def missing_columns(document: str) -> List[str]:
    header_lines = [line for line in document.splitlines()
                    if line.startswith("| Command |")]
    combined = "\n".join(header_lines)
    return [column for column in REQUIRED_COLUMNS if column not in combined]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print extracted literals and their parser source files.",
    )
    args = parser.parse_args()

    try:
        literals = extract_literals()
    except (OSError, UnicodeError) as error:
        print("COMMAND_DOC_CHECK,passed=0,reason=source_read,detail={}".format(error))
        return 1

    if args.list:
        for literal in sorted(literals):
            print("{}\t{}".format(literal, ",".join(sorted(set(literals[literal])))))

    if len(literals) < MINIMUM_EXPECTED_LITERALS:
        print(
            "COMMAND_DOC_CHECK,passed=0,reason=extractor_drift,commands={},minimum={}".format(
                len(literals), MINIMUM_EXPECTED_LITERALS
            )
        )
        return 1

    try:
        document = COMMAND_DOC.read_text(encoding="utf-8", errors="strict")
    except (OSError, UnicodeError) as error:
        print("COMMAND_DOC_CHECK,passed=0,reason=doc_read,detail={}".format(error))
        return 1

    missing: Set[str] = {literal for literal in literals if literal not in document}
    columns = missing_columns(document)
    nyi_missing = sorted(literal for literal in NYI_LITERALS
                         if literal not in literals or literal not in document)
    status_missing = "sta_nyi" not in document or "NYI" not in document

    if missing or columns or nyi_missing or status_missing:
        print(
            "COMMAND_DOC_CHECK,passed=0,commands={},missing={},columns={},nyi={},status={}".format(
                len(literals), len(missing), len(columns), len(nyi_missing),
                int(status_missing)
            )
        )
        for literal in sorted(missing):
            print("COMMAND_DOC_MISSING,literal={!r},source={}".format(
                literal, ",".join(sorted(set(literals[literal])))
            ))
        for column in columns:
            print("COMMAND_DOC_COLUMN_MISSING,name={}".format(column))
        for literal in nyi_missing:
            print("COMMAND_DOC_NYI_MISSING,literal={!r}".format(literal))
        if status_missing:
            print("COMMAND_DOC_STATUS_MISSING,required=NYI+sta_nyi")
        return 1

    print("COMMAND_DOC_CHECK,passed=1,commands={},nyi={}".format(
        len(literals), len(NYI_LITERALS)
    ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
