#!/usr/bin/env python3
"""One-reader firmware benchmark: exact capture windows, immutable raw evidence.

Examples (only run after the fixture and physical port are authorized):
  python3 tools/run_firmware_benchmark.py --port /dev/ttyACM0 --output-directory OUT --label run01 --suite
  python3 tools/run_firmware_benchmark.py --port /dev/ttyACM0 --output-directory OUT --label run02 --stability --mode VOLT --rows 1

The normal suite is 32 configurations, each with 5 s warmup and 120 s capture.
--stability adds a separate 600 s capture before the optional suite. Preparation
and query/cleanup time do not count as capture. Existing external rail evidence
is required. Only explicit --rail-avdd-uv/--rail-avss-uv may restore missing RAM
calibration, in query-verified CAP with INTREF off. No CAL SAVE, flash, reset,
threshold change, input flush or modem-bit ioctl is requested.

CAPTURED means the requested window was recorded, not physical accuracy PASS.
Raw bytes are never overwritten; events record chunks, not duplicated frames.
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import ExitStack
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time

from capture_reference_fixture import (CaptureError, CaptureOptions, Collector, PosixPort,
                                       add_transport_arguments, validate_transport_options, open_capture_port)
from text_protocol import TextProtocolParser, parse_fields


MODES = ("CAP", "VOLT", "RES", "MIXED")
IDENTITY = ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid")
QUERIES = (("CONFIG?", "CONFIG"), ("PERF?", "PERF"), ("USBSTREAM?", "USBSTREAM"),
           ("FPS?", "FPS"), ("FRONTENDS?", "FRONTENDS"), ("SPI?", "SPI"),
           ("MODE?", "MODE"), ("BAT?", "ABAT"))


@dataclass(frozen=True)
class BenchmarkOptions:
    port: str
    output_directory: Path
    label: str
    mode: str = "VOLT"
    rows: int = 1
    profile: str = "CVRCVRCV"
    suite: bool = False
    stability: bool = False
    capture_seconds: float = 120.0
    stability_seconds: float = 600.0
    warmup_seconds: float = 5.0
    prepare_seconds: float = 45.0
    cleanup_seconds: float = 10.0
    command_timeout: float = 4.0
    progress_seconds: float = 20.0
    rail_avdd_uv: int | None = None
    rail_avss_uv: int | None = None
    rail_source_note: str | None = None
    transport: str = "posix"
    usb_vid: int = 0x303A
    usb_pid: int = 0x4041
    usb_serial: str | None = None


@dataclass(frozen=True)
class Configuration:
    label: str
    mode: str
    rows: int
    profile: str
    seconds: float
    kind: str = "configuration"


def configurations(options):
    result = []
    if options.stability:
        result.append(Configuration(f"{options.label}-stability-{options.mode.lower()}{options.rows}",
                                    options.mode, options.rows, options.profile,
                                    options.stability_seconds, "stability"))
    if options.suite:
        result.extend(Configuration(f"{options.label}-{mode.lower()}{rows}", mode, rows,
                                    options.profile, options.capture_seconds)
                      for mode in MODES for rows in range(1, 9))
    elif not options.stability:
        result.append(Configuration(options.label, options.mode, options.rows,
                                    options.profile, options.capture_seconds))
    return result


def expected_wire_mode(configuration):
    # Production IsHomogeneous examines all eight configured rows, including
    # rows outside this frame's active coverage. CVRCVRCV + ROWS=1 therefore
    # still emits M/MR/K; only its active MR row is CAP.
    if configuration.mode == "MIXED" and len(set(configuration.profile)) == 1:
        return {"C": "CAP", "V": "VOLT", "R": "RES"}[configuration.profile[0]]
    return configuration.mode


def record_paths(directory, label, *, bulk=False):
    paths = {"raw": directory / (label + ".rx.bin"),
            "events": directory / (label + ".events.jsonl"),
            "summary": directory / (label + ".summary.json")}
    if bulk:
        paths["ctrl_raw"] = directory / (label + ".ctrl.rx.bin")
    return paths


class HashingRaw:
    def __init__(self, stream):
        self.stream, self.sha = stream, hashlib.sha256()

    def write(self, data):
        count = self.stream.write(data)
        if count != len(data):
            raise OSError("raw_record_partial_write")
        self.sha.update(data)
        return count

    def flush(self):
        self.stream.flush()


class ObservedLines(deque):
    def __init__(self, observe):
        super().__init__()
        self.observe = observe

    def append(self, line):
        self.observe(line)
        super().append(line)


class BenchmarkCollector(Collector):
    """One owner per endpoint; no flush or cross-endpoint partial-line joins."""
    def __init__(self, settings, configuration, port, raw, events, *, carry=b"", original_usb=None,
                 ctrl_raw=None, ctrl_carry=b""):
        self.settings, self.configuration = settings, configuration
        self.fault_events = 0
        self.expected_identity = None
        self.parser = None
        self.identity_mismatches = 0
        self.identity_examples = []
        self.wire_contract_mismatches = 0
        self.wire_contract_examples = []
        self.frame_header = None
        self.frame_tag = None
        self.parsed_frames = self.crc_valid_frames = 0
        self.warmup_started = self.warmup_finished = None
        self.snapshots = {}
        self.transitions = []
        self.volatile_rail_written = False
        self.safe_cap_recovery = "NOT_ATTEMPTED"
        self.interrupted = False
        total = settings.prepare_seconds + settings.warmup_seconds + configuration.seconds + settings.cleanup_seconds
        capture_options = CaptureOptions(settings.port, total, settings.output_directory,
                                         configuration.label, mode=configuration.mode,
                                         rows=configuration.rows, command_timeout=settings.command_timeout,
                                         progress_seconds=settings.progress_seconds,
                                         transport=settings.transport, usb_vid=settings.usb_vid,
                                         usb_pid=settings.usb_pid, usb_serial=settings.usb_serial)
        super().__init__(capture_options, port, raw, events, ctrl_raw=ctrl_raw)
        self.prepare_deadline = self.started + settings.prepare_seconds
        self.cleanup_budget = settings.cleanup_seconds
        self.lines = ObservedLines(self.observe)
        self.line_buffer = bytearray(carry)
        self.ctrl_line_buffer = bytearray(ctrl_carry)
        self.original_usb = original_usb
        self.faults = deque(maxlen=32)
        self.event("segment", label=configuration.label, benchmark_kind=configuration.kind,
                   benchmark_mode=configuration.mode, rows=configuration.rows,
                   port_session="continuous", carry_prefix_bytes=len(carry),
                   ctrl_carry_prefix_bytes=len(ctrl_carry))

    def event(self, kind, **fields):
        if kind == "firmware_fault":
            self.fault_events += 1
        super().event(kind, **fields)

    def progress(self):
        now = time.monotonic()
        if now >= self.next_progress:
            print(json.dumps({"type": "benchmark_progress", "label": self.configuration.label,
                              "phase": self.phase, "elapsed_seconds": round(now - self.started, 3),
                              "capture_seconds": round(now - self.capture_started, 3)
                              if self.capture_started and not self.capture_finished else
                              round((self.capture_finished or 0) - (self.capture_started or 0), 3),
                              "requested_capture_seconds": self.configuration.seconds,
                              "rx_bytes": self.raw_bytes, "capture_trailers": self.capture_trailers}), flush=True)
            self.next_progress = now + self.options.progress_seconds

    def observe(self, line):
        if self.phase != "capture" or self.parser is None:
            return
        tag = line.split(",", 1)[0]
        fields = parse_fields(line)
        if tag == "M" and "seq" not in fields:  # Memory diagnostics are not mixed data.
            return
        if self.is_bulk and self.observation_channel == "ctrl" and (
                tag in {"C", "V", "R", "M", "MR", "K"} or re.fullmatch(r"[DP][0-9]+", tag)):
            self.wire_contract_mismatches += 1
            if len(self.wire_contract_examples) < 8:
                self.wire_contract_examples.append({"reason": "measurement_on_CTRL_endpoint", "line": line})
            return
        if tag in {"C", "V", "R", "M"} and "seq" in fields:
            self.frame_header = fields
            self.frame_tag = tag
        frame = self.parser.feed_line(line)
        if frame is not None:
            self.parsed_frames += 1
            self.crc_valid_frames += int(frame.crc_ok)
            header = self.frame_header or {}
            identity_ok = all(header.get(k) == self.expected_identity.get(k) for k in IDENTITY)
            identity_ok = identity_ok and all(fields.get(k) == header.get(k) for k in ("session", "bootId", "rev"))
            identity_ok = identity_ok and str(frame.rows) == str(self.configuration.rows)
            if not identity_ok:
                self.identity_mismatches += 1
                if len(self.identity_examples) < 8:
                    self.identity_examples.append({"seq": frame.sequence, "header": header, "trailer": fields})
            active_profile = self.configuration.profile[:self.configuration.rows]
            expected_mode = expected_wire_mode(self.configuration)
            expected_tag = {"CAP": "C", "VOLT": "V", "RES": "R", "MIXED": "M"}[expected_mode]
            wire_ok = self.frame_tag == expected_tag
            units = {"CAP": ("pF", -6), "VOLT": ("V", -6), "RES": ("ohm", -3)}
            if expected_mode == "MIXED":
                wire_ok = wire_ok and getattr(frame, "profile", None) == active_profile + "N" * (8 - len(active_profile))
                row_frames = getattr(frame, "row_frames", {})
                wire_ok = wire_ok and set(row_frames) == set(range(1, self.configuration.rows + 1))
                for row, char in enumerate(active_profile, 1):
                    actual = row_frames.get(row)
                    mode = {"C": "CAP", "V": "VOLT", "R": "RES"}[char]
                    wire_ok = wire_ok and actual is not None and actual.mode == mode and (actual.unit, actual.scale) == units[mode]
            elif expected_mode != "CAP":
                wire_ok = wire_ok and getattr(frame, "mode", None) == expected_mode and (getattr(frame, "unit", None), getattr(frame, "scale", None)) == units[expected_mode]
            if not wire_ok:
                self.wire_contract_mismatches += 1
                if len(self.wire_contract_examples) < 8:
                    self.wire_contract_examples.append({"seq": frame.sequence, "expected_mode": expected_mode,
                                                        "wire_tag": self.frame_tag, "header": header})

    def check_fault(self, baseline=0):
        if self.fault_events > baseline or (self.phase == "capture" and any(
                item["phase"] == "capture" for item in self.reset_observations)):
            raise CaptureError("firmware_fault_or_reset_during_" + self.phase)

    def query(self, command, tag, phase_deadline):
        return super().query(command, tag, phase_deadline)

    def transition(self, kind, value, phase_deadline=None):
        phase_deadline = phase_deadline or self.prepare_deadline
        deadline = self.command_deadline(phase_deadline)
        tags = {"MODE": ("MACK", "MAPP", "MERR", "new", "new", "m", "mode", "mrid"),
                "ROWS": ("RCMD", "RAPP", "RERR", "req", "new", "r", "rows", "rrid"),
                "ROWMODES": ("RMACK", "RMAPP", "RMERR", "new", "profile", "p", "profile", "prid")}
        ack_tag, app_tag, err_tag, ack_key, app_key, domain, config_key, id_key = tags[kind]
        baseline, accepted, terminal = self.fault_events, None, None
        early = {}
        command = f"{kind}={value}"
        self.lines.clear()
        self.send(command, deadline)
        while time.monotonic() < deadline:
            try:
                line = self.next_line(deadline)
            except CaptureError as error:
                if "deadline" not in str(error):
                    raise
                break
            self.check_fault(baseline)
            tag, fields = line.split(",", 1)[0], parse_fields(line)
            if tag == "ERR":
                raise CaptureError(f"device_error:{command}:{line}")
            if tag == ack_tag and fields.get(ack_key) == str(value):
                # A delayed ACK for the already-applied request cannot stand
                # in for this newly issued command. IDs are domain-owned and
                # compared only within the same observed boot/session.
                if self.expected_identity and fields.get("id") in {
                        self.expected_identity.get(id_key), self.expected_identity.get(domain + "termId")}:
                    self.event("stale_ack_ignored", command=command, line=line)
                    continue
                accepted = fields.get("id")
                if not accepted or accepted == "0":
                    raise CaptureError(f"missing_request_id:{line}")
                self.event("accepted", command=command, line=line)
            elif tag in {app_tag, err_tag} and fields.get("id"):
                if len(early) < 16:
                    early[fields["id"]] = (tag, fields, line)
            if accepted in early:
                tag, fields, line = early[accepted]
                if tag == err_tag:
                    raise CaptureError(f"transition_rejected:{line}")
                if fields.get(app_key) == str(value):
                    terminal = fields
                    self.event("applied", command=command, line=line)
                    break
        if accepted is None:
            raise CaptureError("transition_ack_unconfirmed:" + command)
        # Even an observed terminal must agree with atomic CONFIG. A missing
        # terminal can be recovered only for the accepted request in its domain.
        config = self.query("CONFIG?", "CONFIG", phase_deadline)
        if (config.get(config_key) != str(value) or config.get(id_key) != accepted or
                config.get(domain + "termId") != accepted or config.get(domain + "term") != "applied"):
            raise CaptureError(f"transition_config_unconfirmed:{command}:{config}")
        if self.expected_identity and any(config.get(k) != self.expected_identity.get(k) for k in ("session", "bootId")):
            raise CaptureError("boot_changed_during_transition")
        self.expected_identity = config
        method = "ACK_TERMINAL_CONFIG" if terminal else "ACK_CONFIG_RECOVERED_TERMINAL"
        self.transitions.append({"command": command, "request_id": accepted, "verification": method,
                                 "config": config})
        self.event("transition_verified", command=command, request_id=accepted, verification=method)
        return config

    def verify_cap(self, deadline):
        mode = self.query("MODE?", "MODE", deadline)
        if (mode.get("state") != "CAPACITANCE" or mode.get("active") != "CAP" or
                mode.get("route") != "CAP" or mode.get("pending", "NONE") != "NONE" or
                mode.get("intref") != "0" or mode.get("refmux") != "24"):
            raise CaptureError("CAP_reference_state_unverified:" + str(mode))
        self.event("cap_reference_query_verified", evidence="MODE_cached_route_readback", analog_isolation="not_claimed")

    @staticmethod
    def rail_valid(rail):
        try:
            avdd, avss = int(rail["avdd"]), int(rail["avss"])
            return (rail.get("rv") == "1" and rail.get("rs") == "ok" and rail.get("src") == "external" and
                    rail.get("split") == "external_measured" and avdd > 0 > avss and
                    int(rail["rail"]) == avdd - avss)
        except (KeyError, ValueError):
            return False

    def ensure_rail(self):
        rail = self.query("RAIL?", "ARL", self.prepare_deadline)
        avdd, avss = self.settings.rail_avdd_uv, self.settings.rail_avss_uv
        if not self.rail_valid(rail):
            if avdd is None or avss is None:
                raise CaptureError("external_rail_calibration_missing_explicit_values_required")
            self.verify_cap(self.prepare_deadline)
            command = f"RAILCFG={avdd},{avss}"
            deadline = self.command_deadline(self.prepare_deadline)
            accepted, terminals = None, {}
            self.volatile_rail_written = True  # Armed before write: lost ACK does not mean no mutation.
            self.lines.clear()
            self.send(command, deadline)
            while time.monotonic() < deadline:
                line = self.next_line(deadline)
                self.check_fault()
                tag, fields = line.split(",", 1)[0], parse_fields(line)
                if tag in {"ERR", "RERR"}:
                    raise CaptureError("rail_rejected:" + line)
                if fields.get("avdd") != str(avdd) or fields.get("avss") != str(avss):
                    continue
                if tag == "RACK":
                    accepted = fields.get("id")
                if tag == "RAPP":
                    terminals[fields.get("id")] = fields
                if accepted and accepted in terminals:
                    if terminals[accepted].get("state") != "applied":
                        raise CaptureError("rail_not_applied")
                    self.event("volatile_rail_applied", request_id=accepted, avdd_uv=avdd, avss_uv=avss,
                               source_note=self.settings.rail_source_note, nonvolatile_written=False)
                    break
            else:
                raise CaptureError("rail_terminal_timeout")
            rail = self.query("RAIL?", "ARL", self.prepare_deadline)
        if not self.rail_valid(rail):
            raise CaptureError("external_rail_query_unverified")
        if avdd is not None and (rail.get("avdd") != str(avdd) or rail.get("avss") != str(avss)):
            raise CaptureError("existing_external_rail_differs_from_explicit_values")
        self.event("rail_evidence", fields=rail, absolute_voltage_acceptance="NOT_ESTABLISHED")

    def snapshot_window(self, name, deadline):
        self.phase = name
        for attempt in range(2):
            window = {}
            queries = tuple(("USB?", "USB") if self.is_bulk and command == "USBSTREAM?" else (command, tag)
                            for command, tag in QUERIES)
            for command, tag in queries:
                fields = self.query(command, tag, deadline)
                now = time.monotonic()
                window[command] = {"fields": fields, "monotonic": now, "t": now - self.started}
            closing = self.query("CONFIG?", "CONFIG", deadline)
            if all(closing.get(k) == window["CONFIG?"]["fields"].get(k) for k in IDENTITY):
                self.snapshots[name] = window
                self.event("snapshot_window", name=name, revision=closing.get("rev"), attempt=attempt + 1)
                return window
            self.event("snapshot_retry", name=name, reason="configuration_revision_changed")
        raise CaptureError("snapshot_window_configuration_changed:" + name)

    def interval(self, seconds, phase, deadline, *, fault_check=True):
        self.phase = phase
        until = time.monotonic() + seconds
        if until > deadline:
            raise CaptureError("insufficient_phase_budget:" + phase)
        while time.monotonic() < until:
            self.pump(until)
            self.lines.clear()
            if fault_check:
                self.check_fault()

    def run(self):
        failure = None
        try:
            if self.is_bulk:
                self.query("USB?", "USB", self.prepare_deadline)
                self.restore_status = "NOT_APPLICABLE_BULK_ALWAYS_FULL"
            else:
                usb = self.query("USBSTREAM?", "USBSTREAM", self.prepare_deadline)
                if usb.get("v") not in {"FULL", "DEBUG"}:
                    raise CaptureError("unknown_original_usb_profile")
                if self.original_usb is None:
                    self.original_usb = usb
            for command, tag in (("BUILD?", "BUILD"), ("PROTO?", "PROTO"), ("CAL?", "CAL")):
                fields = self.query(command, tag, self.prepare_deadline)
                if command == "PROTO?":
                    self.validate_protocol(fields)
                if command == "PROTO?" and (fields.get("version") != "2" or "config_query_v1" not in fields.get("caps", "")):
                    raise CaptureError("atomic_config_query_capability_required")
            self.expected_identity = self.query("CONFIG?", "CONFIG", self.prepare_deadline)
            self.transition("MODE", "CAP")
            self.verify_cap(self.prepare_deadline)
            self.ensure_rail()
            self.transition("ROWS", self.configuration.rows)
            if self.configuration.mode == "MIXED":
                self.transition("ROWMODES", self.configuration.profile)
            elif self.configuration.mode != "CAP":
                self.transition("MODE", self.configuration.mode)
            self.check_fault()
            if not self.is_bulk:
                self.restore_required = True
                self.set_usb("FULL", self.prepare_deadline)
            self.warmup_started = time.monotonic()
            # Warmup has its own budget; preparation/query time is excluded.
            self.interval(self.settings.warmup_seconds, "warmup",
                          self.prepare_deadline + self.settings.warmup_seconds)
            self.warmup_finished = time.monotonic()
            before = self.snapshot_window("before", self.prepare_deadline + self.settings.warmup_seconds)
            self.check_fault()
            config = before["CONFIG?"]["fields"]
            if any(config.get(k) != self.expected_identity.get(k) for k in IDENTITY):
                raise CaptureError("configuration_changed_before_capture")
            if not self.is_bulk and before["USBSTREAM?"]["fields"].get("v") != "FULL":
                raise CaptureError("FULL_output_not_applied")
            fps = before["FPS?"]["fields"]
            if fps.get("cfcap") != "0" or fps.get("ofcap") != "0":
                raise CaptureError("benchmark_requires_existing_unlimited_FPS_caps_no_automatic_change")
            if self.configuration.mode == "VOLT":
                mode = before["MODE?"]["fields"]
                if mode.get("intref") != "0" or mode.get("refmux") != "24":
                    raise CaptureError("VOLT_reference_state_unverified")
            self.parser = TextProtocolParser()
            self.phase = "capture"
            self.capture_started = time.monotonic()
            self.capture_deadline = self.capture_started + self.configuration.seconds
            if self.capture_deadline > self.total_deadline - self.settings.cleanup_seconds:
                raise CaptureError("insufficient_complete_capture_budget")
            self.event("capture_start", monotonic=self.capture_started, offset=self.raw_bytes,
                       requested_seconds=self.configuration.seconds)
            self.lines.clear()
            while time.monotonic() < self.capture_deadline:
                self.pump(self.capture_deadline)
                self.lines.clear()
                self.check_fault()
            self.capture_finished = time.monotonic()
            if self.parsed_frames == 0:
                raise CaptureError("no_complete_frames_during_capture")
            if self.identity_mismatches:
                raise CaptureError("capture_frame_identity_mismatch")
            if self.wire_contract_mismatches:
                raise CaptureError("capture_frame_mode_or_units_mismatch")
            if self.parser.counters.crc_errors:
                raise CaptureError("capture_ascii_crc_failed")
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            self.interrupted = isinstance(error, KeyboardInterrupt)
            failure = str(error) or "interrupted"
            self.event("error", reason=failure)
        finally:
            if self.capture_started is not None:
                self.capture_finished = self.capture_finished or time.monotonic()
                self.event("capture_end", monotonic=self.capture_finished, offset=self.raw_bytes,
                           frame_trailers=self.capture_trailers,
                           actual_seconds=self.capture_finished - self.capture_started)
            self.phase = "cleanup"
            cleanup_deadline = min(self.total_deadline, time.monotonic() + self.settings.cleanup_seconds)
            cleanup_errors = []
            # Keep the actual diagnostic tail after priority MFAULT. The capture
            # timer has already stopped. This tail is never counted as capture.
            if self.faults:
                try:
                    self.interval(min(2.0, self.settings.cleanup_seconds / 5), "fault_tail", cleanup_deadline,
                                  fault_check=False)
                except (OSError, CaptureError) as error:
                    cleanup_errors.append("fault_tail:" + str(error))
            try:
                after = self.snapshot_window("after", cleanup_deadline - self.settings.cleanup_seconds / 2)
                if self.capture_started and any(after["CONFIG?"]["fields"].get(k) != self.expected_identity.get(k) for k in IDENTITY):
                    failure = failure or "configuration_changed_after_capture"
            except (OSError, CaptureError, KeyboardInterrupt) as error:
                cleanup_errors.append("end_queries:" + (str(error) or "interrupted"))
            self.phase = "cleanup"
            try:
                self.transition("MODE", "CAP", cleanup_deadline)
                self.transition("ROWS", 8, cleanup_deadline)
                self.verify_cap(cleanup_deadline)
                config = self.query("CONFIG?", "CONFIG", cleanup_deadline)
                if (config.get("mode") != "CAP" or config.get("rows") != "8" or config.get("profile") != "CCCCCCCC" or
                        config.get("state") != "CAPACITANCE" or config.get("err") != "0x0"):
                    raise CaptureError("final_CAP8_config_unverified")
                self.safe_cap_recovery = "CAP8_QUERY_VERIFIED"
            except (OSError, CaptureError, KeyboardInterrupt) as error:
                self.safe_cap_recovery = "ERROR"
                cleanup_errors.append("CAP8_recovery:" + (str(error) or "interrupted"))
            if not self.is_bulk:
                try:
                    if self.original_usb is None:
                        raise CaptureError("original_USB_profile_unknown")
                    self.set_usb(self.original_usb["v"], cleanup_deadline)
                    verified = self.query("USBSTREAM?", "USBSTREAM", cleanup_deadline)
                    if any(verified.get(k) != self.original_usb.get(k) for k in ("v", "dataEvery", "diagEvery") if k in self.original_usb):
                        raise CaptureError("original_USB_cadence_unverified")
                    self.restore_status = "VERIFIED_QUERY"
                except (OSError, CaptureError, KeyboardInterrupt) as error:
                    self.restore_status = "ERROR"
                    cleanup_errors.append("usb_restore:" + (str(error) or "interrupted"))
            if cleanup_errors:
                failure = ";".join(([failure] if failure else []) + cleanup_errors)
            self.event("finished", status="ERROR" if failure else "CAPTURED", error=failure)
        duration = (self.capture_finished - self.capture_started) if self.capture_started else 0.0
        return {**self.transport_result(), "status": "ERROR" if failure else "CAPTURED", "error": failure,
                "benchmark_kind": self.configuration.kind, "benchmark_mode": self.configuration.mode,
                "mode_requested": self.configuration.mode, "rows_requested": self.configuration.rows,
                "rowmodes_requested": self.configuration.profile if self.configuration.mode == "MIXED" else None,
                "mixed_active_profile": self.configuration.profile[:self.configuration.rows] if self.configuration.mode == "MIXED" else None,
                "mixed_active_rows_homogeneous": self.configuration.mode == "MIXED" and
                    len(set(self.configuration.profile[:self.configuration.rows])) == 1,
                "mixed_full_profile_homogeneous": self.configuration.mode == "MIXED" and
                    len(set(self.configuration.profile)) == 1,
                "expected_wire_mode": expected_wire_mode(self.configuration),
                "wire_contract_version": "row_profile_full8_v2",
                "capture_requested_seconds": self.configuration.seconds, "capture_seconds": duration,
                "capture_duration_satisfied": duration >= self.configuration.seconds,
                "capture_monotonic_start": self.capture_started, "capture_monotonic_end": self.capture_finished,
                "warmup_requested_seconds": self.settings.warmup_seconds,
                "warmup_seconds": (self.warmup_finished - self.warmup_started) if self.warmup_finished else 0,
                "prepare_budget_seconds": self.settings.prepare_seconds, "cleanup_budget_seconds": self.settings.cleanup_seconds,
                "duration_total_budget_seconds": self.options.duration, "elapsed_seconds": time.monotonic() - self.started,
                "capture_frame_trailers": self.capture_trailers, "capture_parsed_frames": self.parsed_frames,
                "capture_crc_valid_frames": self.crc_valid_frames,
                "host_complete_frame_fps": self.parsed_frames / duration if duration else None,
                "host_fps_is_not_firmware_capture_or_bus_completion": True,
                "parser_counters": asdict(self.parser.counters) if self.parser else {},
                "identity_mismatches": self.identity_mismatches, "identity_examples": self.identity_examples,
                "wire_contract_mismatches": self.wire_contract_mismatches,
                "wire_contract_examples": self.wire_contract_examples,
                "rx_bytes": self.raw_bytes, "tx_os_accepted_bytes": self.tx_bytes, "partial_writes": self.partial_writes,
                "initial_usb_profile": self.original_usb, "usb_restore": self.restore_status,
                "safe_cap_recovery": self.safe_cap_recovery, "transitions": self.transitions,
                "snapshots": self.snapshots, "replies": self.replies,
                "firmware_faults": list(self.faults), "firmware_fault_events": self.fault_events,
                "reset_observations": self.reset_observations, "interrupted": self.interrupted,
                "calibration_written": False, "volatile_rail_calibration_attempted": self.volatile_rail_written,
                "rail_source_note": self.settings.rail_source_note, "measurement_acceptance": "NOT_EVALUATED",
                "rail_policy": "fixed_external_reference_required_for_all_suite_modes_including_CAP",
                "fps_policy": "existing_cfcap_and_ofcap_must_be_zero_no_setter_issued",
                "absolute_voltage_acceptance": "NOT_ESTABLISHED", "analog_isolation_claimed": False,
                "port": self.settings.port, "modem_bit_ioctl": False, "input_flushed": False,
                "reset_requested": False, "flashed": False}


def run_benchmark(options, *, port_factory=None):
    validate_transport_options(options)
    if any(not math.isfinite(value) or value <= 0 for value in (
            options.capture_seconds, options.stability_seconds, options.warmup_seconds,
            options.prepare_seconds, options.cleanup_seconds, options.command_timeout, options.progress_seconds)):
        raise ValueError("all timing budgets must be finite and positive")
    if options.mode not in MODES or options.rows not in range(1, 9) or not re.fullmatch(r"[CVR]{8}", options.profile):
        raise ValueError("mode/rows/profile invalid")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", options.label):
        raise ValueError("label requires A-Z, a-z, digits, underscore or hyphen")
    if (options.rail_avdd_uv is None) != (options.rail_avss_uv is None):
        raise ValueError("both explicit rail values are required")
    if options.rail_avdd_uv is not None and not (options.rail_avdd_uv > 0 > options.rail_avss_uv):
        raise ValueError("rail values require positive AVDD and negative AVSS")
    options.output_directory.mkdir(parents=True, exist_ok=True)
    plan = configurations(options)
    index_path = options.output_directory / (options.label + ".benchmark.json")
    all_paths = [index_path]
    for config in plan:
        paths = record_paths(options.output_directory, config.label, bulk=options.transport == "usb_bulk")
        all_paths.extend(paths.values())
        all_paths.append(Path(str(paths["raw"]) + ".gz"))
        if "ctrl_raw" in paths:
            all_paths.append(Path(str(paths["ctrl_raw"]) + ".gz"))
    if any(path.exists() for path in all_paths):
        raise FileExistsError("benchmark output exists; choose a new --label (no rerun or overwrite)")
    index = {"schema": "firmware_benchmark_v1", "status": "RUNNING", "port": options.port,
             "transport": options.transport, "usb_serial": options.usb_serial,
             "one_port_session": True, "plan": [asdict(c) for c in plan], "results": [],
             "measurement_acceptance": "NOT_EVALUATED", "automatic_retry": False}
    port, carry, ctrl_carry, original_usb = None, b"", b"", None
    with index_path.open("x", encoding="utf-8") as index_file:
        try:
            port = port_factory(options.port) if port_factory is not None else open_capture_port(options)
            index["transport_metadata"] = port.metadata()
            index["session_monotonic_start"] = time.monotonic()
            for config in plan:
                paths = record_paths(options.output_directory, config.label, bulk=options.transport == "usb_bulk")
                with ExitStack() as stack:
                    raw_file = stack.enter_context(paths["raw"].open("xb"))
                    ctrl_file = stack.enter_context(paths["ctrl_raw"].open("xb")) if "ctrl_raw" in paths else None
                    events = stack.enter_context(paths["events"].open("x", encoding="utf-8"))
                    report = stack.enter_context(paths["summary"].open("x", encoding="utf-8"))
                    raw = HashingRaw(raw_file)
                    collector = BenchmarkCollector(options, config, port, raw, events,
                                                   carry=carry, original_usb=original_usb,
                                                   ctrl_raw=ctrl_file, ctrl_carry=ctrl_carry)
                    result = collector.run()
                    carry = bytes(collector.line_buffer)
                    ctrl_carry = bytes(collector.ctrl_line_buffer)
                    original_usb = collector.original_usb
                    raw.flush()
                    result.update(raw_file=str(paths["raw"]), events_file=str(paths["events"]),
                                  raw_sha256=raw.sha.hexdigest(), trailing_partial_line_bytes=len(carry))
                    if "ctrl_raw" in paths:
                        result["ctrl_raw_file"] = str(paths["ctrl_raw"])
                    report.write(json.dumps(result, indent=2) + "\n")
                index["results"].append({"label": config.label, "summary_file": str(paths["summary"]),
                                         "status": result["status"], "error": result["error"],
                                         "capture_seconds": result["capture_seconds"]})
                print(json.dumps({"type": "benchmark_configuration_finished", **index["results"][-1]}), flush=True)
                # A failed configuration is never rerun. Continue only after
                # verified CAP8 + original USB restoration; unknown hardware
                # state cannot be carried into the next configuration.
                if result["interrupted"] or result["safe_cap_recovery"] != "CAP8_QUERY_VERIFIED" or result["usb_restore"] not in {"VERIFIED_QUERY", "NOT_APPLICABLE_BULK_ALWAYS_FULL"}:
                    break
            index["status"] = "COMPLETE" if len(index["results"]) == len(plan) and all(r["status"] == "CAPTURED" for r in index["results"]) else "ERROR"
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            index["status"], index["error"] = "ERROR", str(error) or "interrupted"
        finally:
            if port:
                try:
                    port.close()
                except OSError as error:
                    index["status"] = "ERROR"
                    index["close_error"] = str(error)
            index["session_monotonic_end"] = time.monotonic()
            completed = {r["label"] for r in index["results"]}
            index["not_run"] = [c.label for c in plan if c.label not in completed]
            index_file.write(json.dumps(index, indent=2) + "\n")
    return index


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_transport_arguments(parser)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--mode", choices=MODES, default="VOLT")
    parser.add_argument("--rows", type=int, choices=range(1, 9), default=1)
    parser.add_argument("--profile", default="CVRCVRCV", help="Eight configured row modes; a nonuniform full profile keeps M/MR/K even when only its CAP row is active.")
    parser.add_argument("--suite", action="store_true", help="All CAP/VOLT/RES/MIXED × 1–8 rows in one port session.")
    parser.add_argument("--stability", action="store_true", help="Run an independent full 600-second window before the optional suite.")
    parser.add_argument("--capture-seconds", type=float, default=120.0)
    parser.add_argument("--rail-avdd-uv", type=int)
    parser.add_argument("--rail-avss-uv", type=int)
    parser.add_argument("--rail-source-note", help="Provenance of explicitly supplied RAM calibration; never an accuracy claim.")
    args = parser.parse_args(argv)
    if args.capture_seconds < 120:
        parser.error("--capture-seconds must be at least 120; prepare/warmup/cleanup are additional")
    try:
        result = run_benchmark(BenchmarkOptions(**vars(args)))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({"status": result["status"], "completed": len(result["results"]),
                      "not_run": result["not_run"]}), flush=True)
    return 0 if result["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
