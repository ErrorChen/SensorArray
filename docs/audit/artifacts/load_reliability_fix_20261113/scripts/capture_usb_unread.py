#!/usr/bin/env python3
"""Bounded application-unread USB test, with one continuously open serial fd.

Defaults: VOLT1/FPS uncapped, warmup 5 s, read 20 s, do not read 5 s,
read 20 s, query recovery evidence, then CAP8 and original USBSTREAM profile.
This does NOT simulate an absent USB host or physical unplug. The OS/USB
driver may continue accepting bytes while the application is not reading.
No rail calibration, NVS, FPS, recovery, restart, flash or BLE command is sent.
Only read-only queries retry; setters are never replayed on missing ACK.

Exit 0 means this control/data-resumption check passed. Sequence gaps and
rejected partial/CRC records remain explicit; it does not mean lossless USB,
continuous analog validity, physical isolation or simultaneous BLE success.
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
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
from capture_reference_fixture import CaptureError, PosixPort
from text_protocol import TextProtocolParser, parse_fields
import run_firmware_benchmark_retry as retry
import capture_load_benchmark as load
import switch_and_query as control

base = retry.original
CAPABILITIES = load.REQUIRED_CAPABILITIES | {"config_identity_v1", "recovery_query_v1"}
READ_PHASES = {"warmup", "receive_before", "receive_after"}
EXTRA_REQUIRED = {"RECOVERY?": control.EXTRA_REQUIRED["RECOVERY?"]}


@dataclass(frozen=True)
class Options:
    port: str
    output_directory: Path
    label: str
    duration: float = 120.0
    prepare_seconds: float = 30.0
    warmup_seconds: float = 5.0
    before_seconds: float = 20.0
    unread_seconds: float = 5.0
    after_seconds: float = 20.0
    query_seconds: float = 20.0
    cleanup_seconds: float = 10.0
    command_timeout: float = 2.0
    progress_seconds: float = 20.0


@contextmanager
def query_schemas():
    previous = retry.REQUIRED, retry.validate_reply
    retry.REQUIRED = {**previous[0], **load.EXTRA_REQUIRED, **EXTRA_REQUIRED}
    retry.validate_reply = load.validate_reply
    try:
        yield
    finally:
        retry.REQUIRED, retry.validate_reply = previous


class UnreadCollector(load.LoadBenchmarkCollector):
    def __init__(self, options, port, raw, events):
        self.unread_options = options
        self.read_suspended = False
        self.suspended_fd = None
        self.known_identities = set()
        self.frame_expected = False
        self.receive_windows = {}
        self.unread_window = None
        self.control_snapshots = {}
        self.control_snapshot_attempts = []
        self.last_valid_sequence = None
        self.resume_control_verified = False
        settings = base.BenchmarkOptions(options.port, options.output_directory, options.label,
                                        prepare_seconds=options.duration - options.cleanup_seconds,
                                        warmup_seconds=0, cleanup_seconds=options.cleanup_seconds,
                                        command_timeout=options.command_timeout, progress_seconds=options.progress_seconds)
        config = base.Configuration(options.label, "VOLT", 1, "VVVVVVVV", 0, "usb_application_unread")
        super().__init__(settings, config, port, raw, events)
        self.prepare_deadline = min(self.started + options.prepare_seconds, self.total_deadline - options.cleanup_seconds)
        self.work_deadline = self.total_deadline - options.cleanup_seconds

    def progress(self):
        now = time.monotonic()
        if now < self.next_progress:
            return
        window = self.unread_window if self.phase == "application_unread" else self.receive_windows.get(self.phase)
        print(json.dumps({"type": "usb_unread_progress", "phase": self.phase,
                          "elapsed_seconds": round(now - self.started, 3), "rx_bytes": self.raw_bytes,
                          "application_read_suspended": self.read_suspended,
                          "window_requested_seconds": window.get("requested_seconds") if window else None,
                          "window_elapsed_seconds": round(now - window["started_monotonic"], 3) if window else None}), flush=True)
        self.next_progress = now + self.unread_options.progress_seconds

    def pump(self, deadline):
        if self.read_suspended:
            raise CaptureError("runner_attempted_read_during_unread_window")
        return super().pump(deadline)

    def send(self, command, deadline):
        if self.read_suspended:
            raise CaptureError("runner_attempted_command_during_unread_window")
        return super().send(command, deadline)

    def query(self, command, tag, deadline):
        fields = super().query(command, tag, deadline)
        if command == "PROTO?" and CAPABILITIES - set(fields.get("caps", "").split("/")):
            raise CaptureError("unread_test_capability_missing")
        return fields

    def transition(self, kind, value, phase_deadline=None):
        if self.expected_identity:
            self.known_identities.add(control.identity(self.expected_identity))
        result = super().transition(kind, value, phase_deadline)
        if result.get("err") != "0x0":
            raise CaptureError("applied_configuration_error")
        self.known_identities.add(control.identity(result))
        return result

    def check_running_faults(self):
        self.check_fault()
        if self.reset_observations:
            raise CaptureError("reset_observed_during_unread_test")

    def observe(self, line):
        if self.phase not in READ_PHASES or self.parser is None:
            return
        window = self.receive_windows[self.phase]
        tag, fields = line.split(",", 1)[0], parse_fields(line)
        if tag in {"C", "V", "R", "M"} and "seq" in fields:
            if any(fields.get(key) != self.expected_identity.get(key) for key in ("session", "bootId")):
                raise CaptureError("frame_boot_changed")
            current = control.identity(fields)
            if current != control.identity(self.expected_identity):
                if current not in self.known_identities:
                    raise CaptureError("frame_unknown_configuration")
                self.frame_expected = False
                window["previous_configuration_headers"] += 1
                return
            if not self.frame_expected:
                self.parser = TextProtocolParser()
            self.frame_expected = True
        if not self.frame_expected:
            window["prefix_or_previous_configuration_trailers"] += int(tag == "K")
            return
        # Shared parser/validator is deliberately reused. Here phases describe
        # receive windows, not device sampling time or a new firmware epoch.
        before = (self.parsed_frames, self.crc_valid_frames, self.identity_mismatches,
                  self.wire_contract_mismatches, self.parser.counters.malformed, self.parser.counters.crc_errors)
        phase = self.phase
        self.phase = "capture"
        try:
            super().observe(line)
        finally:
            self.phase = phase
        window["malformed_records"] += self.parser.counters.malformed - before[4]
        window["crc_rejected_records"] += self.parser.counters.crc_errors - before[5]
        if self.parsed_frames == before[0]:
            return
        if self.identity_mismatches != before[2] or self.wire_contract_mismatches != before[3]:
            raise CaptureError("received_frame_identity_or_units_mismatch")
        header = self.frame_header
        if any(header.get(k) != self.expected_identity.get(domain) or fields.get(k) != header.get(k)
               for k, domain in (("gen", "mgen"), ("rid", "mrid"))):
            raise CaptureError("VOLT_legacy_mode_identity_mismatch")
        sequence = int(header["seq"])
        if not control.at_or_after32(sequence, int(self.expected_identity["boundary"])):
            raise CaptureError("frame_before_applied_boundary")
        window["complete_frames"] += 1
        if self.crc_valid_frames == before[1]:
            return
        window["crc_valid_identity_frames"] += 1
        if self.last_valid_sequence is not None:
            delta = (sequence - self.last_valid_sequence) & 0xFFFFFFFF
            if delta == 0:
                window["duplicate_sequences"] += 1
            elif delta < 0x80000000:
                window["sequence_gaps"] += int(delta > 1)
                window["missing_sequences"] += delta - 1
            else:
                window["out_of_order_sequences"] += 1
        self.last_valid_sequence = sequence
        sample = {"sequence": sequence, "device_timestamp_us": header.get("ts"),
                  "receive_monotonic": time.monotonic(), "raw_bytes_received": self.raw_bytes,
                  "identity": {k: header.get(k) for k in base.IDENTITY},
                  "legacy": {k: header.get(k) for k in ("gen", "rid")},
                  "trailer": {k: fields.get(k) for k in ("seq", "crc", "session", "bootId", "rev")}}
        if window["first_frame"] is None:
            window["first_frame"] = sample
        window["last_frame"] = sample
        if len(window["first_frame_examples"]) < 8:
            window["first_frame_examples"].append(sample)

    def receive(self, name, seconds):
        started = time.monotonic()
        until = started + seconds
        if until > self.work_deadline:
            raise CaptureError("insufficient_receive_budget:" + name)
        self.phase = name
        self.parser = TextProtocolParser()
        self.frame_expected = False
        window = {"requested_seconds": seconds, "started_monotonic": started,
                  "finished_monotonic": None, "raw_offset_start": self.raw_bytes,
                  "complete_frames": 0, "crc_valid_identity_frames": 0, "crc_rejected_records": 0,
                  "malformed_records": 0, "previous_configuration_headers": 0,
                  "prefix_or_previous_configuration_trailers": 0, "sequence_gaps": 0,
                  "missing_sequences": 0, "duplicate_sequences": 0, "out_of_order_sequences": 0,
                  "first_frame": None, "last_frame": None, "first_frame_examples": [],
                  "received_data_may_have_been_queued_before_window": True,
                  "frame_generation_wall_clock": "UNKNOWN"}
        self.receive_windows[name] = window
        self.event("receive_window_start", name=name, monotonic=started, raw_offset=self.raw_bytes)
        try:
            while time.monotonic() < until:
                self.pump(until)
                self.lines.clear()
                self.check_running_faults()
        finally:
            finished = time.monotonic()
            window.update(finished_monotonic=finished, actual_seconds=finished - started,
                          raw_offset_end=self.raw_bytes)
            self.event("receive_window_end", name=name, monotonic=finished, raw_offset=self.raw_bytes,
                       actual_seconds=finished - started)
        if window["crc_valid_identity_frames"] == 0:
            raise CaptureError("no_complete_crc_identity_frame:" + name)

    def pause_reading(self):
        self.check_running_faults()
        started = time.monotonic()
        until = started + self.unread_options.unread_seconds
        if until > self.work_deadline:
            raise CaptureError("insufficient_unread_budget")
        self.phase = "application_unread"
        self.suspended_fd = self.port.fd
        self.read_suspended = True
        window = {"requested_seconds": self.unread_options.unread_seconds, "started_monotonic": started,
                  "raw_offset_start": self.raw_bytes, "fd_open": True, "fd": self.suspended_fd,
                  "host_present": "NOT_TESTED", "physical_unplug": False,
                  "scope": "This process makes no read or command call; OS/USB buffering may continue.",
                  "firmware_fault_detection_deferred_until_read_resumes": True}
        self.unread_window = window
        self.event("application_unread_start", **window)
        try:
            while time.monotonic() < until:
                self.progress()  # stdout/file progress only; no serial I/O
                time.sleep(min(0.05, max(0, until - time.monotonic())))
        finally:
            finished = time.monotonic()
            self.read_suspended = False
            window.update(finished_monotonic=finished, actual_seconds=finished - started,
                          raw_offset_end=self.raw_bytes, fd_unchanged=self.port.fd == self.suspended_fd)
            self.event("application_unread_end", **window)
        if self.raw_bytes != window["raw_offset_start"] or not window["fd_unchanged"]:
            raise CaptureError("unread_window_IO_ownership_violation")

    def snapshot(self, name, deadline):
        self.phase = name
        for attempt in range(2):
            window = {}
            evidence = {"name": name, "attempt": attempt + 1, "queries": window}
            self.control_snapshot_attempts.append(evidence)
            for command, tag in (("CONFIG?", "CONFIG"), ("SERIAL?", "SERIAL"), ("SCHED?", "SCHED"),
                                 ("FRONTENDS?", "FRONTENDS"), ("RECOVERY?", "RECOVERY"),
                                 ("PERF?", "PERF"), ("FPS?", "FPS"), ("CONFIG?", "CONFIG")):
                fields = self.query(command, tag, deadline)
                key = "CONFIG?_closing" if command == "CONFIG?" and command in window else command
                window[key] = dict(self.load_query_windows[-1])
                self.check_running_faults()
            opening, closing = window["CONFIG?"]["fields"], window["CONFIG?_closing"]["fields"]
            if not control.same_config(opening, closing):
                evidence["outcome"] = "configuration_changed_retry"
                continue
            if not control.same_config(closing, self.expected_identity):
                raise CaptureError("configuration_changed_around_unread_window")
            recovery = window["RECOVERY?"]["fields"]
            if any(recovery.get(k) != closing.get(k) for k in ("session", "bootId")) or recovery["busy"] != "0":
                raise CaptureError("recovery_query_boot_mismatch_or_busy")
            health = load.scheduler_health(window["SCHED?"]["fields"])
            evidence["scheduler_health"] = health
            if not health["usable"]:
                raise CaptureError("scheduler_evidence_unavailable:" + ",".join(health["reasons"]))
            frontend = window["FRONTENDS?"]["fields"]
            if any(frontend.get(k) != value for k, value in (("hold", "0"), ("known", "7"), ("present", "7"), ("quarantine", "0"))):
                raise CaptureError("frontends_evidence_unavailable_or_held")
            fps = window["FPS?"]["fields"]
            if (fps.get("cfcap"), fps.get("ofcap")) != ("0", "0"):
                raise CaptureError("uncapped_FPS_required_no_FPS_written")
            evidence["outcome"] = "coherent_configuration_control_queries_complete"
            self.control_snapshots[name] = window
            return window
        raise CaptureError("unread_snapshot_configuration_changed")

    def execute(self):
        failure, cleanup_errors = None, []
        cap_restore = usb_restore = "NOT_ATTEMPTED"
        try:
            self.original_usb = self.query("USBSTREAM?", "USBSTREAM", self.prepare_deadline)
            if self.original_usb.get("v") not in {"DEBUG", "FULL"}:
                raise CaptureError("unknown_original_USB_profile")
            self.query("BUILD?", "BUILD", self.prepare_deadline)
            self.query("PROTO?", "PROTO", self.prepare_deadline)
            self.expected_identity = self.query("CONFIG?", "CONFIG", self.prepare_deadline)
            self.transition("MODE", "CAP", self.prepare_deadline)
            self.verify_cap(self.prepare_deadline)
            rail = self.query("RAIL?", "ARL", self.prepare_deadline)
            if not self.rail_valid(rail):
                raise CaptureError("existing_external_rail_required_no_calibration_written")
            self.transition("ROWS", 1, self.prepare_deadline)
            self.transition("MODE", "VOLT", self.prepare_deadline)
            self.snapshot("before", self.prepare_deadline)
            self.set_usb("FULL", self.prepare_deadline)
            self.check_running_faults()
            self.receive("warmup", self.unread_options.warmup_seconds)
            self.receive("receive_before", self.unread_options.before_seconds)
            self.pause_reading()
            self.receive("receive_after", self.unread_options.after_seconds)
            self.snapshot("after", min(self.work_deadline, time.monotonic() + self.unread_options.query_seconds))
            self.resume_control_verified = True
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            failure = str(error) or "interrupted"
            self.event("test_error", error=failure)
        finally:
            self.phase = "cleanup"
            deadline = min(self.total_deadline, time.monotonic() + self.unread_options.cleanup_seconds)
            try:
                self.transition("MODE", "CAP", deadline)
                self.transition("ROWS", 8, deadline)
                self.verify_cap(deadline)
                config = self.query("CONFIG?", "CONFIG", deadline)
                if (config.get("mode"), config.get("rows"), config.get("profile"), config.get("err")) != ("CAP", "8", "CCCCCCCC", "0x0"):
                    raise CaptureError("final_CAP8_unverified")
                cap_restore = "CAP8_QUERY_VERIFIED"
            except (OSError, CaptureError, KeyboardInterrupt) as error:
                cap_restore = "ERROR"
                cleanup_errors.append("CAP8:" + (str(error) or "interrupted"))
            try:
                if not self.original_usb:
                    raise CaptureError("unknown_original_USB_profile")
                self.set_usb(self.original_usb["v"], deadline)
                final_usb = self.query("USBSTREAM?", "USBSTREAM", deadline)
                if any(final_usb.get(k) != self.original_usb.get(k) for k in ("v", "dataEvery", "diagEvery")):
                    raise CaptureError("original_USB_profile_unverified")
                usb_restore = "VERIFIED_QUERY"
            except (OSError, CaptureError, KeyboardInterrupt) as error:
                usb_restore = "ERROR"
                cleanup_errors.append("USBSTREAM:" + (str(error) or "interrupted"))
        if self.fault_events or self.reset_observations:
            failure = failure or "firmware_fault_or_reset_observed"
        failure = ";".join(([failure] if failure else []) + cleanup_errors) or None
        success = failure is None and self.resume_control_verified
        return {"schema": "usb_application_unread_v1", "status": "CONTROL_RECOVERY_PASS" if success else "ERROR", "error": failure,
                "receive_windows": self.receive_windows, "unread_window": self.unread_window,
                "snapshots": self.control_snapshots, "snapshot_attempts": self.control_snapshot_attempts,
                "diagnostic_query_windows": self.load_query_windows, "query_attempts": self.query_attempts,
                "replies": self.replies, "transitions": self.transitions, "original_usb": self.original_usb,
                "resume_control_verified": self.resume_control_verified, "safe_cap_recovery": cap_restore,
                "usb_restore": usb_restore, "firmware_faults": list(self.faults), "reset_observations": self.reset_observations,
                "elapsed_seconds": time.monotonic() - self.started, "total_deadline_seconds": self.unread_options.duration,
                "rx_bytes": self.raw_bytes, "required_capabilities": sorted(CAPABILITIES), "scheduler_workers": list(load.WORKERS),
                "setters_retried": False, "fps_written": False, "calibration_written": False, "nonvolatile_written": False,
                "recovery_requested": False, "restart_requested": False, "physical_unplug": False,
                "absent_USB_host_tested": False, "BLE_tested_by_this_process": False, "input_flushed": False,
                "measurement_acceptance": "NOT_EVALUATED", "physical_safety": "NOT_VERIFIED",
                "sequence_gap_policy": "Recorded, not required to be zero; incomplete/CRC-bad records never count as valid frames.",
                "post_unread_frame_age": "UNKNOWN: receive times are not sampling times; host/driver queued bytes remain possible.",
                "scheduler_semantics": "Yields are not CPU utilization or proof of watchdog safety."}


def run(options, *, port_factory=PosixPort, command=None):
    timings = (options.duration, options.prepare_seconds, options.warmup_seconds, options.before_seconds,
               options.unread_seconds, options.after_seconds, options.query_seconds, options.cleanup_seconds,
               options.command_timeout, options.progress_seconds)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", options.label) or any(not math.isfinite(v) or v <= 0 for v in timings):
        raise ValueError("label_and_timing_budgets_invalid")
    if options.duration > 120 or options.duration <= sum((options.prepare_seconds, options.warmup_seconds,
            options.before_seconds, options.unread_seconds, options.after_seconds, options.cleanup_seconds)):
        raise ValueError("total_deadline_must_fit_windows_plus_queries_and_be_at_most_120s")
    paths = base.record_paths(options.output_directory, options.label)
    if any(p.exists() for p in paths.values()):
        raise FileExistsError("choose_new_label_existing_evidence_is_never_overwritten")
    metadata = control.provenance(command or sys.argv)
    for path in (Path(__file__), Path(load.__file__), Path(control.__file__)):
        metadata["source_sha256"][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    metadata.update(started_utc=datetime.now(timezone.utc).isoformat(), started_monotonic=time.monotonic())
    options.output_directory.mkdir(parents=True, exist_ok=True)
    port = collector = None
    with paths["raw"].open("xb") as raw_stream, paths["events"].open("x") as events, paths["summary"].open("x") as summary:
        raw = base.HashingRaw(raw_stream)
        try:
            port = port_factory(options.port)
            with query_schemas():
                collector = UnreadCollector(options, port, raw, events)
                result = collector.execute()
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            result = {"schema": "usb_application_unread_v1", "status": "ERROR", "error": str(error) or "interrupted"}
        finally:
            if port:
                port.close()
        raw.flush()
        result.update(provenance=metadata, exit_code=0 if result["status"] == "CONTROL_RECOVERY_PASS" else 1,
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
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--warmup-seconds", type=float, default=5)
    parser.add_argument("--before-seconds", type=float, default=20)
    parser.add_argument("--unread-seconds", type=float, default=5)
    parser.add_argument("--after-seconds", type=float, default=20)
    args = parser.parse_args(argv)
    try:
        result = run(Options(**vars(args)))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({k: result.get(k) for k in ("status", "error", "resume_control_verified", "safe_cap_recovery", "usb_restore", "exit_code")}), flush=True)
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
