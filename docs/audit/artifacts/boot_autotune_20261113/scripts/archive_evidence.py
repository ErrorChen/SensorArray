#!/usr/bin/env python3
"""Losslessly archive completed capture bytes; never remove or rewrite originals.

Run from any directory. Empty/incomplete summaries are skipped. A valid final
raw_sha256 is required, including captures whose status is ERROR. Exit 0 means
archival integrity checks passed, NOT firmware/measurement acceptance.

The input scope is this artifact directory (non-recursive) plus the one named
reference preflight. Private device-backup directories are never traversed.
"""
from __future__ import annotations

import argparse
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[5]
ARTIFACTS = Path(__file__).resolve().parents[1]
PREFLIGHT = ROOT / "docs/audit/artifacts/reference_fixture/20261113-spi-boot-preflight-01.summary.json"
CHUNK = 1024 * 1024


class IncompleteSummary(Exception):
    pass


def signature(path: Path) -> tuple[int, ...]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"not a regular non-symlink file: {path}")
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def digest_stream(stream) -> tuple[str, int]:
    digest, count = hashlib.sha256(), 0
    while chunk := stream.read(CHUNK):
        digest.update(chunk)
        count += len(chunk)
    return digest.hexdigest(), count


def digest_file(path: Path) -> tuple[str, int]:
    with path.open("rb") as stream:
        return digest_stream(stream)


def verify_gzip(path: Path, raw_hash: str, raw_bytes: int) -> dict:
    before = signature(path)
    with path.open("rb") as stream:
        header = stream.read(10)
    if len(header) != 10 or header[:3] != b"\x1f\x8b\x08" or header[4:8] != b"\0" * 4:
        raise ValueError(f"gzip header is not mtime=0: {path}")
    with gzip.open(path, "rb") as stream:
        restored_hash, restored_bytes = digest_stream(stream)
    if (restored_hash, restored_bytes) != (raw_hash, raw_bytes):
        raise ValueError(f"existing/staged gzip does not restore the original bytes: {path}")
    compressed_hash, compressed_bytes = digest_file(path)
    if signature(path) != before:
        raise ValueError(f"gzip changed during verification: {path}")
    return {"sha256": compressed_hash, "bytes": compressed_bytes,
            "mtime": 0, "uncompressed_sha256": restored_hash,
            "uncompressed_bytes": restored_bytes, "round_trip_verified": True}


def archive_pair(summary: Path, repo: Path) -> dict:
    """Archive one final summary/raw pair; raises without overwriting bad gzip."""
    summary_before = signature(summary)
    summary_bytes = summary.read_bytes()
    try:
        report = json.loads(summary_bytes)
    except (ValueError, UnicodeError) as error:
        raise IncompleteSummary("empty_or_invalid_json") from error
    if not isinstance(report, dict):
        raise IncompleteSummary("summary_not_object")
    expected_hash = report.get("raw_sha256")
    if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_hash):
        raise IncompleteSummary("missing_final_raw_sha256")
    if str(report.get("status", "")).upper() in {"RUNNING", "PENDING", "STARTED", "IN_PROGRESS"}:
        raise IncompleteSummary("status_not_terminal")
    raw = summary.with_name(summary.name.removesuffix(".summary.json") + ".rx.bin")
    archived = raw.with_name(raw.name + ".gz")
    raw_before = signature(raw)
    if report.get("raw_file"):
        declared = Path(report["raw_file"])
        if not declared.is_absolute():
            declared = repo / declared
        if declared.resolve() != raw.resolve():
            raise ValueError(f"summary raw_file does not match its sibling: {summary}")
    raw_hash, raw_bytes = digest_file(raw)
    if raw_hash != expected_hash.lower():
        raise ValueError(f"raw SHA256 does not match completed summary: {raw}")
    if "rx_bytes" in report and report["rx_bytes"] != raw_bytes:
        raise ValueError(f"raw length does not match summary rx_bytes: {raw}")
    temporary = None
    try:
        if not archived.exists():
            with tempfile.NamedTemporaryFile(mode="w+b", dir=raw.parent,
                    prefix=".archive-", suffix=".tmp", delete=False) as output:
                temporary = Path(output.name)
                with raw.open("rb") as source, gzip.GzipFile(
                        filename="", mode="wb", fileobj=output,
                        compresslevel=9, mtime=0) as compressed:
                    while chunk := source.read(CHUNK):
                        compressed.write(chunk)
                output.flush()
                os.fsync(output.fileno())
            verify_gzip(temporary, raw_hash, raw_bytes)
            if signature(raw) != raw_before or signature(summary) != summary_before:
                raise ValueError(f"capture changed during compression: {raw}")
            try:
                os.link(temporary, archived)  # Atomic publish; never replaces an existing gzip.
            except FileExistsError:
                pass  # A concurrent writer is accepted only after the same verification below.
        compressed_info = verify_gzip(archived, raw_hash, raw_bytes)
        if signature(raw) != raw_before or signature(summary) != summary_before:
            raise ValueError(f"capture changed during verification: {raw}")
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    faults = report.get("firmware_faults")
    return {
        "raw": {"path": raw.relative_to(repo).as_posix(),
                "sha256": raw_hash, "bytes": raw_bytes, "preserved": True},
        "gzip": {"path": archived.relative_to(repo).as_posix(), **compressed_info},
        "summary": {"path": summary.relative_to(repo).as_posix(),
                    "sha256": hashlib.sha256(summary_bytes).hexdigest(),
                    "bytes": len(summary_bytes),
                    "status_present": "status" in report, "status": report.get("status"),
                    "error_present": "error" in report, "error": report.get("error"),
                    "measurement_acceptance": report.get("measurement_acceptance"),
                    "firmware_fault_count": len(faults) if isinstance(faults, list) else None},
        "acceptance_inferred_from_archive": False,
    }


def run(excluded: set[str]) -> int:
    index_path = ARTIFACTS / "evidence-index.json"
    records = {}
    if index_path.exists():
        signature(index_path)
        previous = json.loads(index_path.read_bytes())
        if previous.get("schema_version") != 1:
            raise ValueError("refusing to replace an unrecognized evidence index")
        records = {entry["raw"]["path"]: entry for entry in previous["records"]}
    skipped, errors, verified = [], [], 0
    candidates = sorted(ARTIFACTS.glob("*.summary.json")) + [PREFLIGHT]
    for summary in candidates:
        label = summary.name.removesuffix(".summary.json")
        relative = summary.relative_to(ROOT).as_posix()
        # Exclusion occurs before opening/stat-ing any active capture path.
        if label in excluded:
            skipped.append({"summary": relative, "reason": "explicitly_excluded"})
            continue
        if "device-backup" in label.lower():
            skipped.append({"summary": relative, "reason": "private_backup_excluded"})
            continue
        if not summary.exists():
            skipped.append({"summary": relative, "reason": "summary_not_created"})
            continue
        try:
            record = archive_pair(summary, ROOT)
            records[record["raw"]["path"]] = record
            verified += 1
        except IncompleteSummary as error:
            skipped.append({"summary": relative, "reason": str(error)})
        except (OSError, ValueError, EOFError, TypeError) as error:
            errors.append({"summary": relative, "error": str(error)})
    index = {
        "schema_version": 1,
        "path_base": "repository_root",
        "scope": "nonrecursive boot_autotune_20261113 captures plus named reference preflight only",
        "compression": {"format": "gzip", "level": 9, "mtime": 0, "filename_header": ""},
        "originals_deleted_or_overwritten": False,
        "exit_0_meaning": "Archival integrity checks passed; NOT firmware, transport, or scientific measurement acceptance. Completed ERROR captures are preserved. Missing summary status/error remain unknown, not PASS.",
        "archive_script_sha256": digest_file(Path(__file__).resolve())[0],
        "records": [records[key] for key in sorted(records)],
        "skipped": skipped,
        "archive_errors": errors,
    }
    encoded = (json.dumps(index, indent=2, ensure_ascii=False) + "\n").encode()
    if not index_path.exists() or index_path.read_bytes() != encoded:
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="wb", dir=ARTIFACTS,
                    prefix=".evidence-index-", suffix=".tmp", delete=False) as output:
                temporary = Path(output.name)
                output.write(encoded)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, index_path)  # Only the owned generated index is replaced.
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    print(json.dumps({"verified_this_run": verified, "indexed": len(records),
                      "skipped": skipped, "archive_errors": errors,
                      "index": index_path.relative_to(ROOT).as_posix(),
                      "measurement_acceptance": "NOT_INFERRED"}, ensure_ascii=False))
    return 1 if errors else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exclude", action="append", default=[], metavar="LABEL",
                        help="Skip this capture label without reading it; repeat as needed.")
    args = parser.parse_args()
    # No persistent lock file and no waiting behind another archive process.
    with Path(__file__).open("rb") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Another evidence archive run is active; no files changed.", file=sys.stderr)
            return 2
        return run(set(args.exclude))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"Archive failed: {error}", file=sys.stderr)
        raise SystemExit(1)
