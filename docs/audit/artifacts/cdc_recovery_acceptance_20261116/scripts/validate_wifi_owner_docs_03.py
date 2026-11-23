#!/usr/bin/env python3
"""Read-only checks for the five documented Wi-Fi/manual-restart additions.

Writes a new evidence record exclusively; does not modify source or old evidence.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[5]
STAGE = ROOT / "docs/audit/artifacts/cdc_recovery_acceptance_20261116"
DOCUMENTS = [
    "README.md",
    "docs/integration/firmware_host_contract.md",
    "docs/audit/firmware_reliability_fix_report.md",
    "docs/architecture.md",
    "docs/ble-protocol.md",
]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    errors: list[str] = []
    documents = []
    for name in DOCUMENTS:
        path = ROOT / name
        content = path.read_text()
        checked = 0
        for target in re.findall(r"\[[^\]\n]*\]\(([^)\n]+)\)", content):
            parsed = urlsplit(target.strip("<>"))
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            linked = path.parent / unquote(parsed.path)
            checked += 1
            if not linked.exists():
                errors.append(f"{name}: missing local target {target}")
        if sum(line.startswith("```") for line in content.splitlines()) % 2:
            errors.append(f"{name}: unbalanced triple-backtick fences")
        documents.append({"path": name, "sha256": sha(path), "local_links_checked": checked})

    readme = (ROOT / "README.md").read_text()
    contract = (ROOT / "docs/integration/firmware_host_contract.md").read_text()
    for anchor in ("wifi-owner-v1", "manual-restart-noos"):
        if f'<a id="{anchor}"></a>' not in readme:
            errors.append(f"README missing explicit anchor {anchor}")
    for anchor in ("wifi-owner-contract-v1", "manual-restart-contract"):
        if f'<a id="{anchor}"></a>' not in contract:
            errors.append(f"contract missing explicit anchor {anchor}")
    if "\nStill in test\n" not in readme:
        errors.append("user's Still in test marker missing")
    contract_block = contract.split('<a id="wifi-owner-contract-v1"></a>', 1)[1].split(
        "## build06 typed metadata", 1
    )[0]
    handoff_path = STAGE / "wifi-owner-contract-handoff-02.json"
    handoff = json.loads(handoff_path.read_text())
    documented_fields = 0
    for command, details in handoff["queries"].items():
        for field in details["fields"]:
            documented_fields += 1
            if field not in contract_block:
                errors.append(f"missing handoff field {command}: {field}")

    freeze_path = STAGE / "final-acceptance-runner-freeze-01.json"
    freeze = json.loads(freeze_path.read_text())
    for item in freeze["files"]:
        if sha(ROOT / item["path"]) != item["sha256"]:
            errors.append(f"frozen runner source changed: {item['path']}")

    checked_sources = []
    for record, key in (
        ("wifi-owner-host-02/manifest.json", "production_sha256"),
        ("manual-restart-independent-review-01.json", "sources"),
    ):
        evidence = json.loads((STAGE / record).read_text())
        for name, expected in evidence[key].items():
            expected = expected.get("sha256") if isinstance(expected, dict) else expected
            source = ROOT / name
            if not expected or not source.is_file():
                errors.append(f"unverifiable evidence source: {record}: {name}")
                continue
            if sha(source) != expected:
                errors.append(f"source differs from documented evidence: {name}")
            checked_sources.append({"path": name, "sha256": expected})

    diff = subprocess.run(
        ["git", "diff", "--check", "--", *DOCUMENTS],
        cwd=ROOT, capture_output=True, text=True, check=False,
    )
    if diff.returncode:
        errors.append("git diff --check failed")
    report = {
        "status": "PASS" if not errors else "FAIL",
        "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": "Five documentation files only; no hardware, build or new host execution",
        "documents": documents,
        "handoff": {"path": str(handoff_path.relative_to(ROOT)), "sha256": sha(handoff_path)},
        "documented_query_fields_checked": documented_fields,
        "runner_freeze": {"path": str(freeze_path.relative_to(ROOT)), "sha256": sha(freeze_path)},
        "frozen_runner_sources_checked": len(freeze["files"]),
        "documented_production_sources_checked": checked_sources,
        "git_diff_check": {"exit_code": diff.returncode, "stdout": diff.stdout, "stderr": diff.stderr},
        "errors": errors,
        "hardware_actions": [],
        "overall_acceptance": "NOT_ESTABLISHED",
    }
    destination = STAGE / "wifi-owner-doc-validation-03.json"
    with destination.open("x") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    print(json.dumps({"status": report["status"], "documents": len(documents),
                      "local_links_checked": sum(x["local_links_checked"] for x in documents),
                      "errors": errors, "report": str(destination.relative_to(ROOT))}, ensure_ascii=False))
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
