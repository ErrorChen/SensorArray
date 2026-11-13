#!/usr/bin/env python3
"""Fifty bounded MODE/ROWS/ROWMODES requests with atomic authority and frames.

Reuses the existing one-reader POSIX Collector, retry logic and wire validator.
Only read-only queries retry; setters are sent once per explicit request.
Existing external rail calibration is required and never changed. RECOVERY?
and RESTART? are observations, never recovery/restart requests. Final state is
query-verified CAP8 plus the original USBSTREAM profile. No NVS, CAL, FPS,
flash, reset, modem-bit ioctl or input flush is requested.

Run only after the operator authorizes the physical port and fixture:
  python3 switch_and_query.py --port /dev/ttyACM0 --output-directory OUT --label switch50-01
Exit 0 establishes this control/identity test, not analog measurement accuracy.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
from capture_reference_fixture import CaptureError, PosixPort
import run_firmware_benchmark as base
import run_firmware_benchmark_retry as retry
from text_protocol import TextProtocolParser, parse_fields

EXTRA_REQUIRED = {
    "RECOVERY?": "session bootId busy phase pendingId pendingLevel pendingKnown pendingRev lastId lastLevel last reason acceptedKnown acceptedRev resultKnown rev boundary originalError cleanupError routeApplied routeKnown routeVerified terminalQueued physicalSafe",
    "RESTART?": "session bootId busy phase pendingId pendingKnown pendingRev lastId last reason acceptedKnown acceptedRev resultKnown rev boundary originalError publishError terminalQueued serialQueued serialError netQueued netError",
}
CAPABILITIES = {"config_identity_v1", "config_query_v1", "recovery_query_v1", "restart_query_v1"}
CONFIG_KEYS = base.IDENTITY + ("mode", "rows", "profile", "boundary", "state", "err")


@dataclass(frozen=True)
class Options:
    port: str
    output_directory: Path
    label: str
    duration: float = 300.0
    prepare_seconds: float = 30.0
    cleanup_seconds: float = 20.0
    step_seconds: float = 5.0
    command_timeout: float = 1.0
    progress_seconds: float = 20.0
    frames_per_step: int = 3


def request_plan():
    """Exactly 50 accepted requests; setup/cleanup are separately recorded."""
    result = []
    for block in range(5):
        result.extend((("MODE", "CAP"), ("MODE", "CAP"),
                       ("ROWS", str((2 * block) % 8 + 1)), ("MODE", "VOLT"),
                       ("ROWS", str((2 * block + 1) % 8 + 1)), ("MODE", "RES"),
                       ("ROWMODES", "CVRCVRCV"), ("ROWS", str((2 * block + 2) % 8 + 1)),
                       ("ROWMODES", "VRCCVRCC"), ("MODE", "CAP")))
    return result


def identity(fields):
    return tuple(fields.get(key) for key in base.IDENTITY)


def same_config(a, b):
    return all(a.get(key) is not None and a.get(key) == b.get(key) for key in CONFIG_KEYS)


def at_or_after32(sequence, boundary):
    return ((sequence - boundary) & 0xFFFFFFFF) < 0x80000000


@contextmanager
def query_schemas():
    # Process-local extension; neither the frozen retry source nor other
    # running capture processes are changed.
    previous = retry.REQUIRED
    retry.REQUIRED = {**previous, **EXTRA_REQUIRED}
    try:
        yield
    finally:
        retry.REQUIRED = previous


class SwitchCollector(retry.RetryBenchmarkCollector):
    def __init__(self, options, port, raw, events):
        self.switch_options = options
        self.known_configurations = set()
        self.frame_expected = False
        self.step_frames = []
        self.stale_frames = 0
        self.query_windows = []
        self.state_bundles = []
        settings = base.BenchmarkOptions(options.port, options.output_directory, options.label,
                                        prepare_seconds=options.duration - options.cleanup_seconds,
                                        warmup_seconds=0, cleanup_seconds=options.cleanup_seconds,
                                        command_timeout=options.command_timeout, progress_seconds=options.progress_seconds)
        config = base.Configuration(options.label, "CAP", 8, "CCCCCCCC", 0, "switch_identity")
        super().__init__(settings, config, port, raw, events)
        self.work_deadline = self.total_deadline - options.cleanup_seconds
        self.prepare_deadline = min(self.started + options.prepare_seconds, self.work_deadline)

    def query(self, command, tag, deadline):
        record = {"command": command, "phase": self.phase, "started_monotonic": time.monotonic()}
        self.query_windows.append(record)
        try:
            fields = super().query(command, tag, deadline)
            if command in EXTRA_REQUIRED:
                if not re.fullmatch(r"[0-9A-Fa-f]{16}", fields["session"]) or fields["busy"] not in ("0", "1"):
                    raise CaptureError("invalid_control_query_identity:" + command)
            record["fields"] = dict(fields)
            return fields
        except CaptureError as error:
            record["error"] = str(error)
            raise
        finally:
            record["finished_monotonic"] = time.monotonic()

    def transition(self, kind, value, phase_deadline=None):
        if self.expected_identity:
            self.known_configurations.add(identity(self.expected_identity))
        result = super().transition(kind, value, phase_deadline)
        if result.get("err") != "0x0":
            raise CaptureError("applied_configuration_error:" + str(result))
        self.known_configurations.add(identity(result))
        return result

    def state_bundle(self, name, deadline):
        self.phase = name
        for attempt in range(2):
            records = {}
            evidence = {"name": name, "attempt": attempt + 1, "queries": records}
            self.state_bundles.append(evidence)
            for command, tag in (("CONFIG?", "CONFIG"), ("RECOVERY?", "RECOVERY"),
                                 ("RESTART?", "RESTART"), ("CONFIG?", "CONFIG")):
                fields = self.query(command, tag, deadline)
                records["CONFIG?_closing" if command == "CONFIG?" and command in records else command] = dict(self.query_windows[-1])
            opening, closing = (records[key]["fields"] for key in ("CONFIG?", "CONFIG?_closing"))
            if not same_config(opening, closing):
                evidence["outcome"] = "configuration_changed_retry"
                continue
            if self.expected_identity and not same_config(closing, self.expected_identity):
                raise CaptureError("configuration_changed_during_control_queries")
            for command in EXTRA_REQUIRED:
                state = records[command]["fields"]
                if any(state.get(key) != closing.get(key) for key in ("session", "bootId")):
                    raise CaptureError("control_query_boot_mismatch:" + command)
                if state["busy"] != "0":
                    raise CaptureError("external_control_pending:" + command)
                # rev in these replies belongs to their last result, and may
                # legitimately differ from CONFIG's current revision.
            evidence["outcome"] = "coherent_configuration_same_boot"
            return records
        raise CaptureError("control_query_bundle_configuration_changed")

    def observe(self, line):
        if self.phase != "capture" or self.parser is None:
            return
        tag, fields = line.split(",", 1)[0], parse_fields(line)
        if tag in ("C", "V", "R", "M") and "seq" in fields:
            if any(fields.get(key) != self.expected_identity.get(key) for key in ("session", "bootId")):
                raise CaptureError("frame_boot_changed")
            current = identity(fields)
            if current != identity(self.expected_identity):
                if current not in self.known_configurations:
                    raise CaptureError("frame_unknown_configuration")
                self.frame_expected = False
                self.event("queued_previous_configuration", sequence=fields.get("seq"), revision=fields.get("rev"))
                return
            if not self.frame_expected:
                self.parser = TextProtocolParser()
            self.frame_expected = True
        if not self.frame_expected:
            self.stale_frames += int(tag == "K")
            return
        previous_count = self.parsed_frames
        super().observe(line)  # actual shared CRC/units/MR/full-eight-profile validator
        if self.parser.counters.crc_errors or self.parser.counters.malformed:
            raise CaptureError("current_configuration_frame_crc_or_format")
        if self.parsed_frames == previous_count:
            return
        if self.identity_mismatches or self.wire_contract_mismatches:
            raise CaptureError("current_configuration_frame_identity_or_units")
        header = self.frame_header
        sequence, boundary = int(header["seq"]), int(self.expected_identity["boundary"])
        if not at_or_after32(sequence, boundary):
            raise CaptureError("frame_before_applied_boundary")
        if self.step_frames:
            delta = (sequence - self.step_frames[-1]["sequence"]) & 0xFFFFFFFF
            if not 0 < delta < 0x80000000:
                raise CaptureError("frame_sequence_not_forward_within_boot")
        domains = {"C": {"gen": "rgen", "rid": "rrid"},
                   "V": {"gen": "mgen", "rid": "mrid"}, "R": {"gen": "mgen", "rid": "mrid"},
                   "M": {key: key for key in ("rgen", "rrid", "pgen", "prid")}}[self.frame_tag]
        if any(header.get(wire) != self.expected_identity.get(domain) or fields.get(wire) != header.get(wire)
               for wire, domain in domains.items()):
            raise CaptureError("legacy_frame_identity_domain_mismatch")
        self.step_frames.append({"sequence": sequence, "tag": self.frame_tag,
                                 "identity": {key: header.get(key) for key in base.IDENTITY},
                                 "legacy": {key: header.get(key) for key in domains},
                                 "trailer": {key: fields.get(key) for key in ("seq", "crc", "session", "bootId", "rev")},
                                 "applied_boundary": boundary})

    def verify_frames(self, authority, deadline):
        self.configuration = base.Configuration(self.configuration.label, authority["mode"], int(authority["rows"]),
                                                authority["profile"], 0, "switch_identity")
        self.parser = TextProtocolParser()
        self.frame_expected = False
        self.step_frames = []
        self.phase = "capture"
        self.event("identity_window_start", authority=authority)
        while len(self.step_frames) < self.switch_options.frames_per_step:
            if time.monotonic() >= deadline:
                raise CaptureError("identity_frame_deadline_expired")
            self.pump(deadline)
            self.lines.clear()
            self.check_fault()
        frames = list(self.step_frames)
        self.phase = "switch"
        closing = self.query("CONFIG?", "CONFIG", deadline)
        if not same_config(authority, closing):
            raise CaptureError("configuration_changed_around_verified_frames")
        self.event("identity_window_end", sequences=[f["sequence"] for f in frames], config_closing=closing)
        return frames

    def execute(self):
        results, failure, cleanup_errors = [], None, []
        final_cap, usb_restore = "NOT_ATTEMPTED", "NOT_ATTEMPTED"
        try:
            self.original_usb = self.query("USBSTREAM?", "USBSTREAM", self.prepare_deadline)
            if self.original_usb.get("v") not in ("FULL", "DEBUG"):
                raise CaptureError("original_USB_profile_unknown")
            self.query("BUILD?", "BUILD", self.prepare_deadline)
            proto = self.query("PROTO?", "PROTO", self.prepare_deadline)
            if proto.get("version") != "2" or CAPABILITIES - set(proto.get("caps", "").split("/")):
                raise CaptureError("required_control_query_capability_missing")
            self.expected_identity = self.query("CONFIG?", "CONFIG", self.prepare_deadline)
            self.transition("MODE", "CAP", self.prepare_deadline)
            self.transition("ROWS", 8, self.prepare_deadline)
            self.verify_cap(self.prepare_deadline)
            rail = self.query("RAIL?", "ARL", self.prepare_deadline)
            if not self.rail_valid(rail):
                raise CaptureError("existing_external_rail_required_no_calibration_written")
            self.query("FPS?", "FPS", self.prepare_deadline)  # record only; no FPS setter
            self.state_bundle("before", self.prepare_deadline)
            self.check_fault()
            self.set_usb("FULL", self.prepare_deadline)
            for number, (kind, value) in enumerate(request_plan(), 1):
                self.phase = "switch"
                deadline = min(self.work_deadline, time.monotonic() + self.switch_options.step_seconds)
                started = time.monotonic()
                authority = self.transition(kind, value, deadline)
                frames = self.verify_frames(authority, deadline)
                record = {"request_number": number, "command": kind + "=" + value,
                          "started_monotonic": started, "finished_monotonic": time.monotonic(),
                          "transition": self.transitions[-1], "frames": frames}
                results.append(record)
                self.event("switch_verified", **record)
                print(json.dumps({"type": "switch_progress", "verified": number, "total": 50,
                                  "command": record["command"], "sequences": [f["sequence"] for f in frames]}), flush=True)
            self.state_bundle("after", self.work_deadline)
            self.check_fault()
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            failure = str(error) or "interrupted"
            self.event("test_error", error=failure)
        finally:
            self.phase = "cleanup"
            cleanup_deadline = min(self.total_deadline, time.monotonic() + self.switch_options.cleanup_seconds)
            try:
                self.transition("MODE", "CAP", cleanup_deadline)
                self.transition("ROWS", 8, cleanup_deadline)
                self.verify_cap(cleanup_deadline)
                config = self.query("CONFIG?", "CONFIG", cleanup_deadline)
                if (config.get("mode"), config.get("rows"), config.get("profile"), config.get("err")) != ("CAP", "8", "CCCCCCCC", "0x0"):
                    raise CaptureError("final_CAP8_unverified")
                final_cap = "CAP8_QUERY_VERIFIED"
            except (OSError, CaptureError, KeyboardInterrupt) as error:
                final_cap = "ERROR"
                cleanup_errors.append("CAP8:" + (str(error) or "interrupted"))
            try:
                if not self.original_usb:
                    raise CaptureError("original_USB_profile_unknown")
                self.set_usb(self.original_usb["v"], cleanup_deadline)
                restored = self.query("USBSTREAM?", "USBSTREAM", cleanup_deadline)
                if any(restored.get(k) != self.original_usb.get(k) for k in ("v", "dataEvery", "diagEvery")):
                    raise CaptureError("original_USB_profile_unverified")
                usb_restore = "VERIFIED_QUERY"
            except (OSError, CaptureError, KeyboardInterrupt) as error:
                usb_restore = "ERROR"
                cleanup_errors.append("USBSTREAM:" + (str(error) or "interrupted"))
        if self.fault_events or self.reset_observations:
            failure = failure or "firmware_fault_or_reset_observed"
        failure = ";".join(([failure] if failure else []) + cleanup_errors) or None
        passed = failure is None and len(results) == 50
        return {"schema": "switch_and_query_v1", "status": "CONTROL_IDENTITY_PASS" if passed else "ERROR", "error": failure,
                "completed_requests": len(results), "planned_requests": 50, "results": results,
                "transitions_including_setup_cleanup": self.transitions, "state_bundles": self.state_bundles,
                "query_windows": self.query_windows, "query_attempts": self.query_attempts, "replies": self.replies,
                "safe_cap_recovery": final_cap, "usb_restore": usb_restore, "original_usb": self.original_usb,
                "firmware_faults": list(self.faults), "reset_observations": self.reset_observations,
                "queued_previous_configuration_frames": self.stale_frames, "elapsed_seconds": time.monotonic() - self.started,
                "total_deadline_seconds": self.switch_options.duration, "rx_bytes": self.raw_bytes,
                "setters_retried": False, "calibration_written": False, "nonvolatile_written": False,
                "recovery_requested": False, "restart_requested": False, "fps_written": False,
                "measurement_acceptance": "NOT_EVALUATED", "physical_safety": "NOT_VERIFIED",
                "modem_bit_ioctl": False, "input_flushed": False}


def provenance(command):
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    status = subprocess.check_output(["git", "status", "--porcelain=v1", "-z"], cwd=ROOT)
    diff = subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=ROOT)
    sources = (Path(__file__), Path(base.__file__), Path(retry.__file__), ROOT / "tools/capture_reference_fixture.py", ROOT / "tools/text_protocol.py")
    return {"command": command, "workspace_head": head, "workspace_dirty": bool(status),
            "dirty_hash": hashlib.sha256(status + b"\0" + diff).hexdigest(),
            "dirty_hash_scope": "git status porcelain-z + NUL + tracked git diff --binary HEAD; untracked file contents excluded",
            "workspace_identity_is_not_flashed_firmware_proof": True,
            "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}


def run(options, *, port_factory=PosixPort, command=None):
    if not re.fullmatch(r"[A-Za-z0-9_-]+", options.label) or not 1 <= options.frames_per_step <= 16:
        raise ValueError("invalid_label_or_frames_per_step")
    if any(not math.isfinite(value) or value <= 0 for value in (options.duration, options.prepare_seconds,
           options.cleanup_seconds, options.step_seconds, options.command_timeout, options.progress_seconds)):
        raise ValueError("timing_budgets_must_be_finite_positive")
    if options.duration <= options.prepare_seconds + options.cleanup_seconds:
        raise ValueError("total_budget_has_no_switch_interval")
    paths = base.record_paths(options.output_directory, options.label)
    if any(path.exists() for path in paths.values()):
        raise FileExistsError("choose_a_new_label_existing_evidence_is_never_overwritten")
    metadata = provenance(command or sys.argv)
    metadata["started_utc"] = datetime.now(timezone.utc).isoformat()
    metadata["started_monotonic"] = time.monotonic()
    options.output_directory.mkdir(parents=True, exist_ok=True)
    port = collector = None
    with paths["raw"].open("xb") as raw_stream, paths["events"].open("x") as events, paths["summary"].open("x") as summary:
        raw = base.HashingRaw(raw_stream)
        try:
            port = port_factory(options.port)
            with query_schemas():
                collector = SwitchCollector(options, port, raw, events)
                result = collector.execute()
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            result = {"schema": "switch_and_query_v1", "status": "ERROR", "error": str(error) or "interrupted", "completed_requests": 0}
        finally:
            if port:
                port.close()
        raw.flush()
        result.update(provenance=metadata, exit_code=0 if result["status"] == "CONTROL_IDENTITY_PASS" else 1,
                      raw_file=str(paths["raw"]), events_file=str(paths["events"]), raw_sha256=raw.sha.hexdigest(),
                      trailing_partial_line_bytes=len(collector.line_buffer) if collector else 0,
                      finished_utc=datetime.now(timezone.utc).isoformat(), finished_monotonic=time.monotonic())
        summary.write(json.dumps(result, indent=2) + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--duration", type=float, default=300.0, help="Total bound including setup, every query and cleanup.")
    args = parser.parse_args(argv)
    try:
        result = run(Options(**vars(args)))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({key: result.get(key) for key in ("status", "error", "completed_requests", "safe_cap_recovery", "usb_restore", "exit_code")}), flush=True)
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
