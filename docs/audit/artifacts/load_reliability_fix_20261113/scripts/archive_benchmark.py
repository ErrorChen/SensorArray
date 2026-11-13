#!/usr/bin/env python3
"""Losslessly archive finalized benchmark evidence, preserving every original.

Only this artifact directory is scanned, non-recursively. A final summary hash
is required before its serial raw/events or BLE notification journal is read.
Empty, invalid and in-progress summaries are skipped. Completed ERROR records
are retained. Exit 0 means archive integrity, never measurement acceptance.

Run --self-test for temporary synthetic fixtures only. Run normal archival
after measurement has stopped; gzip CPU/I/O can affect an active benchmark.
--only/--exclude filter labels before opening any matching evidence file.
No private backup directory is traversed; existing mismatching gzip is refused.
"""
from __future__ import annotations

import argparse
import contextlib
import fcntl
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[5]
ARTIFACTS = Path(__file__).resolve().parents[1]
CHUNK = 1024 * 1024
SCHEMA = "firmware_benchmark_evidence_v1"
# No capture byte-count exceptions are authorized for this repair run.
KNOWN_POST_RESTORE_SCOPES = {}


class IncompleteSummary(Exception):
    pass


def signature(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"not a regular non-symlink file: {path}")
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def digest_stream(stream):
    digest, count = hashlib.sha256(), 0
    while chunk := stream.read(CHUNK):
        digest.update(chunk)
        count += len(chunk)
    return digest.hexdigest(), count


def digest_file(path):
    with path.open("rb") as stream:
        return digest_stream(stream)


def verify_gzip(path, original_hash, original_bytes):
    before = signature(path)
    with path.open("rb") as stream:
        header = stream.read(10)
    if len(header) != 10 or header[:3] != b"\x1f\x8b\x08" or header[4:8] != b"\0" * 4:
        raise ValueError(f"gzip header is not mtime=0: {path}")
    with gzip.open(path, "rb") as stream:
        restored_hash, restored_bytes = digest_stream(stream)
    if (restored_hash, restored_bytes) != (original_hash, original_bytes):
        raise ValueError(f"gzip does not restore exact original bytes: {path}")
    compressed_hash, compressed_bytes = digest_file(path)
    if signature(path) != before:
        raise ValueError(f"gzip changed during verification: {path}")
    return {"sha256": compressed_hash, "bytes": compressed_bytes, "mtime": 0,
            "uncompressed_sha256": restored_hash, "uncompressed_bytes": restored_bytes,
            "round_trip_verified": True}


def check_unchanged(guards):
    for path, before in guards.items():
        if signature(path) != before:
            raise ValueError(f"evidence changed during archival: {path}")


def archive_file(original, guards, repo, expected_hash=None, expected_bytes=None):
    before = signature(original)
    guards = {**guards, original: before}
    original_hash, original_bytes = digest_file(original)
    if expected_hash is not None and original_hash != expected_hash.lower():
        raise ValueError(f"original SHA256 differs from finalized summary: {original}")
    if expected_bytes is not None and original_bytes != expected_bytes:
        raise ValueError(f"original byte count differs from finalized summary: {original}")
    check_unchanged(guards)
    archived = original.with_name(original.name + ".gz")
    temporary = None
    try:
        if not archived.exists():
            with tempfile.NamedTemporaryFile(mode="w+b", dir=original.parent,
                    prefix=".benchmark-archive-", suffix=".tmp", delete=False) as output:
                temporary = Path(output.name)
                with original.open("rb") as source, gzip.GzipFile(filename="", mode="wb",
                        fileobj=output, compresslevel=9, mtime=0) as compressed:
                    while chunk := source.read(CHUNK):
                        compressed.write(chunk)
                output.flush()
                os.fsync(output.fileno())
            verify_gzip(temporary, original_hash, original_bytes)
            check_unchanged(guards)
            try:
                os.link(temporary, archived)  # Atomic publication; cannot replace any existing file.
            except FileExistsError:
                pass  # A racing archive is accepted only after full verification below.
        compressed_info = verify_gzip(archived, original_hash, original_bytes)
        check_unchanged(guards)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {"original": {"path": original.relative_to(repo).as_posix(),
                          "sha256": original_hash, "bytes": original_bytes, "preserved": True},
            "gzip": {"path": archived.relative_to(repo).as_posix(), **compressed_info}}


def strict_json(payload):
    def invalid_constant(value):
        raise ValueError("non-JSON numeric constant: " + value)
    return json.loads(payload, parse_constant=invalid_constant)


def sibling_reference(report, key, expected, repo):
    declared = report.get(key)
    if declared is None:
        return
    if not isinstance(declared, str):
        raise ValueError("summary file reference is not a string: " + key)
    path = Path(declared)
    if not path.is_absolute():
        path = repo / path
    if path.resolve() != expected.resolve():
        raise ValueError(f"summary {key} does not name its in-scope sibling: {expected}")


def command_evidence(events):
    commands = []
    with events.open("rb") as stream:
        for number, payload in enumerate(stream, 1):
            try:
                event = strict_json(payload)
            except (ValueError, UnicodeError) as error:
                raise ValueError(f"completed events JSONL invalid at line {number}: {events}") from error
            if not isinstance(event, dict):
                raise ValueError(f"event is not an object at line {number}: {events}")
            if event.get("type") == "tx_begin":
                command = event.get("command")
                if not isinstance(command, str):
                    raise ValueError(f"tx_begin lacks a command at line {number}: {events}")
                commands.append({"line": number, "t": event.get("t"),
                                 "phase": event.get("phase"), "command": command})
    return {"source": "unchanged_events_jsonl_tx_begin_records",
            "count": len(commands), "commands": commands,
            "interpretation": "Host command intent; not proof of device application or delivery."}


def reviewed_byte_scope(label, report, summary_payload, events, actual_bytes):
    """Accept only the hash-pinned legacy record with a proved restore boundary."""
    known = KNOWN_POST_RESTORE_SCOPES.get(label)
    if (known is None or report.get("benchmark_kind") != "stability" or
            report.get("rx_bytes") != known["declared_bytes"] or
            actual_bytes != known["full_bytes"] or
            report.get("raw_sha256") != known["raw_sha256"] or
            report.get("script_sha256") != known["script_sha256"] or
            hashlib.sha256(summary_payload).hexdigest() != known["summary_sha256"]):
        raise ValueError("original byte count differs from finalized summary: unreviewed scope")
    if digest_file(events)[0] != known["events_sha256"]:
        raise ValueError("reviewed byte scope events SHA256 mismatch")
    total, rx_count, restore_rx_count = 0, 0, 0
    first_restore, previous = None, None
    with events.open("rb") as stream:
        for number, payload in enumerate(stream, 1):
            event = strict_json(payload)
            if not isinstance(event, dict):
                raise ValueError("reviewed byte scope event is not an object")
            if event.get("phase") == "final_restore" and first_restore is None:
                if (event.get("type") != "tx_begin" or event.get("command") != "MODE=CAP" or
                        event.get("offset") != known["declared_bytes"] or
                        total != known["declared_bytes"] or previous is None or
                        previous.get("type") != "finished" or previous.get("phase") != "cleanup" or
                        previous.get("offset") != known["declared_bytes"]):
                    raise ValueError("reviewed byte scope final_restore boundary mismatch")
                first_restore = {"line": number, "offset": total, "t": event.get("t"),
                                 "monotonic": event.get("monotonic"), "command": "MODE=CAP"}
            if first_restore is not None and event.get("phase") != "final_restore":
                raise ValueError("reviewed byte scope leaves final_restore phase")
            if event.get("type") == "rx":
                offset, length = event.get("offset"), event.get("length")
                if (type(offset) is not int or type(length) is not int or
                        offset != total or length <= 0):
                    raise ValueError("reviewed byte scope RX offsets are not contiguous")
                total += length
                rx_count += 1
                restore_rx_count += first_restore is not None
            previous = event
    if first_restore is None or restore_rx_count == 0 or total != actual_bytes:
        raise ValueError("reviewed byte scope RX event total differs from full raw file")
    return {"policy": "exact_legacy_stability_final_restore_record_v1",
            "declared_rx_bytes_preserved": known["declared_bytes"],
            "declared_rx_bytes_scope": "Collector.run return, through cleanup finished event",
            "archived_full_raw_bytes": actual_bytes,
            "additional_bytes": actual_bytes - known["declared_bytes"],
            "additional_bytes_phase": "final_restore",
            "raw_sha256_scope": "full original including final_restore; still strictly verified",
            "rx_event_offsets_contiguous": True, "rx_event_bytes": total,
            "rx_events": rx_count, "final_restore_rx_events": restore_rx_count,
            "first_final_restore": first_restore,
            "summary_unchanged": True,
            "review": "stability-byte-count-scope-review.json"}


def archive_summary(summary, repo):
    summary_before = signature(summary)
    summary_payload = summary.read_bytes()
    try:
        report = strict_json(summary_payload)
    except (ValueError, UnicodeError) as error:
        raise IncompleteSummary("empty_or_invalid_json") from error
    if not isinstance(report, dict):
        raise IncompleteSummary("summary_not_object")
    if str(report.get("status", "")).upper() in {"RUNNING", "PENDING", "STARTED", "IN_PROGRESS"}:
        raise IncompleteSummary("status_not_terminal")
    ble = summary.name.endswith(".ble.summary.json")
    label = summary.name.removesuffix(".ble.summary.json" if ble else ".summary.json")
    hash_key, bytes_key = ("journal_sha256", "journal_bytes") if ble else ("raw_sha256", "rx_bytes")
    expected_hash = report.get(hash_key)
    if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_hash):
        raise IncompleteSummary("missing_final_" + hash_key)
    expected_bytes = report.get(bytes_key)
    if expected_bytes is not None and (not isinstance(expected_bytes, int) or isinstance(expected_bytes, bool) or expected_bytes < 0):
        raise ValueError("invalid finalized byte count: " + str(summary))
    guards = {summary: summary_before}
    byte_scope = None
    if ble:
        journal = summary.with_name(label + ".ble.notifications.jsonl")
        sibling_reference(report, "journal_file", journal, repo)
        guards[journal] = signature(journal)
        sources = [("ble_notifications", journal, expected_hash, expected_bytes)]
        commands = {"source": "summary_application_command_declaration",
                    "declared_application_commands_sent": report.get("application_commands_sent"),
                    "subscribed_channels": report.get("subscribed_channels"),
                    "note": "CCCD subscriptions are separate from application commands; declaration is not inferred from exit status."}
    else:
        raw = summary.with_name(label + ".rx.bin")
        events = summary.with_name(label + ".events.jsonl")
        sibling_reference(report, "raw_file", raw, repo)
        sibling_reference(report, "events_file", events, repo)
        guards[raw], guards[events] = signature(raw), signature(events)
        if expected_bytes is not None and raw.stat().st_size != expected_bytes:
            byte_scope = reviewed_byte_scope(label, report, summary_payload, events, raw.stat().st_size)
            expected_bytes = byte_scope["archived_full_raw_bytes"]
        sources = [("serial_rx", raw, expected_hash, expected_bytes), ("serial_events", events, None, None)]
        commands = command_evidence(events)
    check_unchanged(guards)
    files = []
    for role, path, expected, size in sources:
        files.append({"role": role, **archive_file(path, guards, repo, expected, size)})
    check_unchanged(guards)
    return {"label": label, "kind": "ble_journal" if ble else "serial_capture",
            "summary": {"path": summary.relative_to(repo).as_posix(),
                        "sha256": hashlib.sha256(summary_payload).hexdigest(), "bytes": len(summary_payload),
                        "status_present": "status" in report, "status": report.get("status"),
                        "error_present": "error" in report, "error": report.get("error"),
                        "measurement_acceptance": report.get("measurement_acceptance")},
            "capture": {key: report.get(key) for key in (
                "benchmark_kind", "benchmark_mode", "mode_requested", "rows_requested", "rowmodes_requested",
                "capture_requested_seconds", "capture_seconds", "capture_duration_satisfied",
                "capture_monotonic_start", "capture_monotonic_end", "safe_cap_recovery", "usb_restore",
                "connected_monotonic_ns", "ready_monotonic_ns", "recording_end_monotonic_ns",
                "unexpected_disconnect", "stop_reason")},
            "commands": commands, "files": files, "byte_count_scope_review": byte_scope,
            "acceptance_inferred_from_archive": False}


def write_index(path, encoded):
    if path.exists() and path.read_bytes() == encoded:
        return
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=path.parent,
                prefix=".evidence-index-", suffix=".tmp", delete=False) as output:
            temporary = Path(output.name)
            output.write(encoded)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)  # Only the owned generated index is replaceable.
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def run(artifacts=ARTIFACTS, repo=ROOT, *, only=None, excluded=None, index_path=None):
    only, excluded = set(only or ()), set(excluded or ())
    index_path = (Path(index_path) if index_path is not None else artifacts / "evidence-index.json").resolve()
    if index_path.parent.resolve() != artifacts.resolve() or index_path.suffix != ".json":
        raise ValueError("evidence index must be a JSON file directly in this artifact directory")
    records = {}
    if index_path.exists():
        signature(index_path)
        previous = strict_json(index_path.read_bytes())
        if previous.get("schema") != SCHEMA:
            raise ValueError("refusing to replace an unrecognized evidence index")
        records = {record["summary"]["path"]: record for record in previous["records"]}
    skipped, errors, verified = [], [], []
    for summary in sorted(artifacts.glob("*.summary.json")):
        relative = summary.relative_to(repo).as_posix()
        label = summary.name.removesuffix(".ble.summary.json" if summary.name.endswith(".ble.summary.json") else ".summary.json")
        # Filter by filename before stat/open. Inactive directories and backups
        # are never traversed, and explicitly excluded active files are unread.
        if label in excluded or (only and label not in only) or "device-backup" in label.lower():
            skipped.append({"summary": relative, "reason": "excluded_before_read"})
            continue
        try:
            records[relative] = archive_summary(summary, repo)
            verified.append(relative)
        except IncompleteSummary as error:
            records.pop(relative, None)
            skipped.append({"summary": relative, "reason": str(error)})
        except (OSError, ValueError, EOFError, TypeError) as error:
            records.pop(relative, None)  # Do not present an older checksum as currently verified.
            errors.append({"summary": relative, "error": str(error)})
    index = {"schema": SCHEMA, "path_base": "repository_root",
             "scope": "nonrecursive load_reliability_fix_20261113 finalized serial captures and BLE journals only",
             "compression": {"format": "gzip", "level": 9, "mtime": 0, "filename_header": ""},
             "originals_deleted_or_overwritten": False,
             "archive_script": {"path": "docs/audit/artifacts/load_reliability_fix_20261113/scripts/archive_benchmark.py",
                                "sha256": digest_file(Path(__file__).resolve())[0]},
             "archive_command": ["python3", "docs/audit/artifacts/load_reliability_fix_20261113/scripts/archive_benchmark.py"] +
                 [part for label in sorted(only) for part in ("--only", label)] +
                 [part for label in sorted(excluded) for part in ("--exclude", label)] +
                 (["--index", index_path.relative_to(repo).as_posix()]
                  if index_path != artifacts / "evidence-index.json" else []),
             "exit_0_meaning": "Archive integrity only, NOT firmware, transport or scientific measurement acceptance. ERROR captures remain ERROR; absent status remains unknown.",
             "records": [records[key] for key in sorted(records)], "verified_this_run": verified,
             "skipped": skipped, "archive_errors": errors}
    encoded = (json.dumps(index, indent=2, ensure_ascii=False) + "\n").encode("utf-8")
    write_index(index_path, encoded)
    print(json.dumps({"verified": len(verified), "indexed": len(records), "skipped": skipped,
                      "archive_errors": errors, "index": index_path.relative_to(repo).as_posix(),
                      "measurement_acceptance": "NOT_INFERRED"}, ensure_ascii=False))
    return 1 if errors else 0


def self_test():
    """Temporary synthetic evidence only; never lists/reads active artifacts."""
    with tempfile.TemporaryDirectory(prefix="benchmark-archive-test-") as directory:
        repo = Path(directory)
        artifacts = repo / "artifacts"
        artifacts.mkdir()
        raw, events = artifacts / "serial.rx.bin", artifacts / "serial.events.jsonl"
        raw.write_bytes(b"C,synthetic=1\n\x00\xff" * 100)
        events.write_bytes(b'{"type":"tx_begin","phase":"prepare","t":0.1,"command":"MODE=CAP"}\n'
                           b'{"type":"rx","offset":0,"length":1600}\n')
        summary = artifacts / "serial.summary.json"
        summary.write_text(json.dumps({"status": "CAPTURED", "error": None,
            "raw_sha256": digest_file(raw)[0], "rx_bytes": raw.stat().st_size,
            "raw_file": str(raw), "events_file": str(events)}))
        journal = artifacts / "radio.ble.notifications.jsonl"
        journal.write_bytes(b'{"schema_version":1,"type":"notification","payload_base64":"AP8="}\n' * 100)
        ble_summary = artifacts / "radio.ble.summary.json"
        ble_summary.write_text(json.dumps({"status": "ERROR", "error": "synthetic_disconnect",
            "journal_file": str(journal), "journal_sha256": digest_file(journal)[0],
            "journal_bytes": journal.stat().st_size, "application_commands_sent": 0}))
        (artifacts / "active.summary.json").write_bytes(b"")
        (artifacts / "invalid.summary.json").write_bytes(b"{")
        (artifacts / "pending.summary.json").write_text(json.dumps({"status": "RUNNING", "raw_sha256": "0" * 64}))
        (artifacts / "excluded.summary.json").symlink_to(repo / "do-not-open")
        backup = artifacts / "device-backup"
        backup.mkdir()
        (backup / "private.summary.json").write_text("must never be read")
        originals = {path: path.read_bytes() for path in (raw, events, summary, journal, ble_summary)}
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, excluded={"excluded"}) == 0
        index = strict_json((artifacts / "evidence-index.json").read_bytes())
        assert len(index["records"]) == 2 and len(index["skipped"]) == 4
        assert sum(len(r["files"]) for r in index["records"]) == 3
        assert next(r for r in index["records"] if r["kind"] == "ble_journal")["summary"]["status"] == "ERROR"
        assert next(r for r in index["records"] if r["kind"] == "serial_capture")["commands"]["commands"][0]["command"] == "MODE=CAP"
        archives = [Path(str(path) + ".gz") for path in (raw, events, journal)]
        for path in archives:
            assert gzip.decompress(path.read_bytes()) == originals[Path(str(path)[:-3])]
            assert path.read_bytes()[4:8] == b"\0" * 4
        snapshots = {path: (path.read_bytes(), signature(path)) for path in archives + [artifacts / "evidence-index.json"]}
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, excluded={"excluded"}) == 0
        assert all((path.read_bytes(), signature(path)) == before for path, before in snapshots.items())
        assert all(path.read_bytes() == content for path, content in originals.items())
        # A separately named index preserves the earlier command/index result.
        old_index = (artifacts / "evidence-index.json").read_bytes()
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, only={"radio"}, index_path=artifacts / "evidence-index-02.json") == 0
        assert (artifacts / "evidence-index.json").read_bytes() == old_index
        assert (artifacts / "evidence-index-02.json").exists()
        # A pre-existing corrupt archive is never replaced or silently trusted.
        bad = archives[0]
        bad.write_bytes(b"synthetic corruption")
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, only={"serial"}) == 1
        assert bad.read_bytes() == b"synthetic corruption"
        assert raw.read_bytes() == originals[raw]
        # Completed-summary SHA mismatch cannot publish new archives.
        wrong = artifacts / "wrong.rx.bin"
        wrong.write_bytes(b"wrong synthetic payload")
        (artifacts / "wrong.events.jsonl").write_bytes(b"")
        (artifacts / "wrong.summary.json").write_text(json.dumps({"status": "ERROR", "raw_sha256": "0" * 64}))
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, only={"wrong"}) == 1
        assert not Path(str(wrong) + ".gz").exists()
        # A valid full-file hash alone cannot excuse arbitrary count mismatch.
        (artifacts / "wrong.summary.json").write_text(json.dumps({"status": "ERROR",
            "raw_sha256": digest_file(wrong)[0], "rx_bytes": wrong.stat().st_size - 1}))
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, only={"wrong"}) == 1
        assert not Path(str(wrong) + ".gz").exists()
        # Even a valid hash cannot authorize an out-of-directory file reference.
        (artifacts / "wrong.summary.json").write_text(json.dumps({"status": "ERROR", "raw_sha256": digest_file(wrong)[0], "raw_file": str(repo / "outside.bin")}))
        with contextlib.redirect_stdout(io.StringIO()):
            assert run(artifacts, repo, only={"wrong"}) == 1
        assert not Path(str(wrong) + ".gz").exists()
        # A writer racing compression cannot publish an apparently final gzip.
        changing = artifacts / "changing.rx.bin"
        changing.write_bytes(b"synthetic source before concurrent write")
        previous_verify = verify_gzip
        def change_after_staged_verification(path, digest, size):
            result = previous_verify(path, digest, size)
            if path.name.startswith(".benchmark-archive-"):
                with changing.open("ab") as stream:
                    stream.write(b"more bytes from a simulated active writer")
            return result
        globals()["verify_gzip"] = change_after_staged_verification
        try:
            try:
                archive_file(changing, {summary: signature(summary)}, repo)
            except ValueError as error:
                assert "changed during archival" in str(error)
            else:
                raise AssertionError("concurrently changed source was accepted")
        finally:
            globals()["verify_gzip"] = previous_verify
        assert not Path(str(changing) + ".gz").exists()
        assert not any(backup.glob("*.gz"))

        # Exercise the exact reviewed-scope branch with a separately pinned
        # synthetic record, never by accepting an unknown real-world producer.
        label = "synthetic-stability"
        stable_raw = artifacts / (label + ".rx.bin")
        stable_events = artifacts / (label + ".events.jsonl")
        stable_summary = artifacts / (label + ".summary.json")
        stable_raw.write_bytes(b"beforeafter")
        event_objects = [
            {"type": "rx", "phase": "cleanup", "offset": 0, "length": 6},
            {"type": "finished", "phase": "cleanup", "offset": 6},
            {"type": "tx_begin", "phase": "final_restore", "offset": 6, "command": "MODE=CAP"},
            {"type": "rx", "phase": "final_restore", "offset": 6, "length": 5},
            {"type": "reply", "phase": "final_restore", "offset": 11, "command": "CONFIG?"},
        ]
        def save_events(objects):
            stable_events.write_text("".join(json.dumps(event) + "\n" for event in objects))
        save_events(event_objects)
        stable_report = {"status": "CAPTURED", "benchmark_kind": "stability", "rx_bytes": 6,
                         "raw_sha256": digest_file(stable_raw)[0], "script_sha256": "1" * 64}
        stable_summary.write_text(json.dumps(stable_report))
        scope = {"summary_sha256": digest_file(stable_summary)[0],
                 "events_sha256": digest_file(stable_events)[0],
                 "raw_sha256": stable_report["raw_sha256"], "script_sha256": "1" * 64,
                 "declared_bytes": 6, "full_bytes": 11}
        KNOWN_POST_RESTORE_SCOPES[label] = scope
        try:
            record = archive_summary(stable_summary, repo)
            review = record["byte_count_scope_review"]
            assert review["additional_bytes"] == 5 and review["first_final_restore"]["offset"] == 6
            assert review["rx_event_bytes"] == 11 and review["declared_rx_bytes_preserved"] == 6
            assert gzip.decompress(Path(str(stable_raw) + ".gz").read_bytes()) == b"beforeafter"
            assert strict_json(stable_summary.read_bytes())["rx_bytes"] == 6
            cases = [
                (0, {"offset": 1}, "not contiguous"),
                (1, {"type": "other"}, "boundary mismatch"),
                (2, {"offset": 7}, "boundary mismatch"),
                (3, {"phase": "capture"}, "leaves final_restore"),
                (3, {"length": 4}, "total differs"),
            ]
            for index, change, error_text in cases:
                objects = [dict(event) for event in event_objects]
                objects[index].update(change)
                save_events(objects)
                scope["events_sha256"] = digest_file(stable_events)[0]
                try:
                    reviewed_byte_scope(label, stable_report, stable_summary.read_bytes(), stable_events, 11)
                except ValueError as error:
                    assert error_text in str(error), str(error)
                else:
                    raise AssertionError("invalid restore boundary or RX stream was accepted")
            save_events(event_objects)
            # An unexpected event mutation is rejected before phase inspection.
            try:
                reviewed_byte_scope(label, stable_report, stable_summary.read_bytes(), stable_events, 11)
            except ValueError as error:
                assert "events SHA256 mismatch" in str(error)
            else:
                raise AssertionError("unreviewed event hash was accepted")
        finally:
            del KNOWN_POST_RESTORE_SCOPES[label]
    print("ARCHIVE_BENCHMARK_SELF_TEST,PASS,synthetic_only=1,lossless=1,idempotent=1,no_overwrite=1,active_skipped=1,concurrent_change_rejected=1,scope_checked=1,ERROR_preserved=1,legacy_restore_scope_verified=1,arbitrary_count_mismatch_rejected=1,separate_index_preserved=1")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", action="append", default=[], metavar="LABEL",
                        help="Read only these labels; all others excluded before stat/open.")
    parser.add_argument("--exclude", action="append", default=[], metavar="LABEL",
                        help="Skip a label before stat/open, including an active BLE journal.")
    parser.add_argument("--self-test", action="store_true", help="Use only temporary synthetic data; do not read artifact captures.")
    parser.add_argument("--index", type=Path, help="Use a separate JSON index in this artifact directory, preserving earlier archive results.")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    with Path(__file__).open("rb") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Another archive process is active; no evidence changed.", file=sys.stderr)
            return 2
        index_path = args.index
        if index_path is not None and index_path.parent == Path("."):
            index_path = ARTIFACTS / index_path
        return run(only=args.only, excluded=args.exclude, index_path=index_path)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"Archive failed: {error}", file=sys.stderr)
        raise SystemExit(1)
