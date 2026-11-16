#!/usr/bin/env python3
"""Bounded POSIX raw capture for the explicitly connected reference fixture.

No flash, reset, modem-bit ioctl, input flush, or calibration write is performed.
--duration is the TOTAL deadline, including preparation and USBSTREAM restore;
actual sampling seconds are reported separately. OS/driver open behaviour is
observed, not a guarantee that a particular USB implementation never resets.
"""
from __future__ import annotations

import argparse
from collections import deque
from contextlib import ExitStack
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import select
import sys
import time

from text_protocol import parse_fields

try:
    import fcntl
    import termios
except ImportError:  # Windows formal bulk does not need POSIX terminal modules.
    fcntl = termios = None


class CaptureError(RuntimeError):
    pass


# Only these documented, side-effect-free queries may wait past a response
# without a command identity. Setters and transition waits remain strict.
READONLY_QUERY_COMMANDS = frozenset({
    "BUILD?", "PROTO?", "CONFIG?", "BOOT?", "STATE?", "MODE?", "ROWS?",
    "ROWMODES?", "CAL?", "RAIL?", "USB?", "USBQ?", "USBSTREAM?", "SERIAL?",
    "SCHED?", "SPI?", "FRONTENDS?", "PERF?", "FPS?", "BAT?", "BLELINK?",
    "BLETX?", "BLESVC?", "SCHEDWHY?", "TX?", "ST?", "BTX?", "WIFI?", "RECOVERY?", "RESTART?",
})


class ReceivedLine(str):
    """This round: retain endpoint provenance while preserving parser API."""
    def __new__(cls, text, channel):
        value = super().__new__(cls, text)
        value.channel = channel
        return value


class PosixPort:
    """One reader with cooperative flock and kernel exclusion of new opens.

    Kernel TIOCEXCL cannot prove that an older uncooperative reader is absent.
    No TIOCMGET/TIOCMSET/TIOCMBIS/TIOCMBIC or reset API is used.
    """
    def __init__(self, path: str):
        if fcntl is None or termios is None:
            raise CaptureError("POSIX_terminal_backend_unavailable")
        self.fd = -1
        self.exclusive = False
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.fd = fd
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            self.exclusive = True
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL |
                          getattr(termios, "CRTSCTS", 0) | getattr(termios, "PARODD", 0))
            attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[3] = 0
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except BaseException:
            self.close()
            raise

    transport = "posix"

    def metadata(self):
        return {"transport": self.transport, "ownership": "flock+TIOCEXCL",
                "existing_readers": "unverified", "baud_setting": 115200,
                "tx_acceptance": "OS_write_only_not_device_or_PC_ack"}

    def read_some(self, max_bytes, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        readable, _, _ = select.select([self.fd], [], [], min(0.5, remaining))
        if not readable:
            return None
        try:
            chunk = os.read(self.fd, max_bytes)
        except BlockingIOError:
            return None
        if not chunk:
            raise CaptureError("serial_eof_or_disconnected")
        return "data", chunk

    def write_some(self, data, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return 0
        _, writable, _ = select.select([], [self.fd], [], min(0.5, remaining))
        if not writable:
            return 0
        try:
            return os.write(self.fd, data)
        except BlockingIOError:
            return 0

    def close(self):
        if self.fd >= 0:
            if self.exclusive:
                try:
                    fcntl.ioctl(self.fd, termios.TIOCNXCL)
                except OSError:
                    pass
            # Leave HUPCL cleared. Restoring an old HUPCL/B0 setting on close
            # could request a hangup; capture never requests that operation.
            os.close(self.fd)
            self.fd = -1


@dataclass
class CaptureOptions:
    port: str
    duration: float
    output_directory: Path
    label: str
    mode: str | None = None
    rows: int | None = None
    command_timeout: float = 10.0
    progress_seconds: float = 20.0
    rowmodes: str | None = None
    transport: str = "posix"
    usb_vid: int = 0x303A
    usb_pid: int | None = None
    usb_serial: str | None = None


def add_transport_arguments(parser):
    """This round: shared CLI selection; never silently fall back to CDC."""
    parser.add_argument("--port", help="Legacy POSIX path; formal USB roles are selected by stable serial.")
    parser.add_argument("--transport", choices=("posix", "usb_bulk", "usb_cdc"), default="posix")
    parser.add_argument("--usb-vid", type=lambda value: int(value, 0), default=0x303A)
    parser.add_argument("--usb-pid", type=lambda value: int(value, 0), help="Default 4041 for vendor bulk, 4042 for formal CDC.")
    parser.add_argument("--usb-serial", help="Stable 12-hex MAC without colons; required for formal USB.")


def transport_usb_pid(options):
    return options.usb_pid if options.usb_pid is not None else (0x4042 if options.transport == 'usb_cdc' else 0x4041)


def validate_transport_options(options):
    if options.transport == "posix":
        if not options.port:
            raise ValueError("--port is required for POSIX capture")
    elif options.transport in {"usb_bulk", "usb_cdc"}:
        if not options.usb_serial or not re.fullmatch(r"[0-9A-Fa-f]{12}", options.usb_serial):
            raise ValueError("--usb-serial requires 12 hex MAC digits without colons")
        if options.port:
            raise ValueError("--port cannot select or override a formal USB role")
        pid = transport_usb_pid(options)
        if not (0 < options.usb_vid <= 65535 and 0 < pid <= 65535):
            raise ValueError("USB VID/PID out of range")
        if options.usb_vid == 0x303A and pid in {0x1001, 0x4001}:
            raise ValueError("legacy JTAG/CDC-only PID cannot select formal bulk")
        if options.transport == 'usb_cdc' and (options.usb_vid, pid) != (0x303A, 0x4042):
            raise ValueError('formal_CDC_requires_303a4042_interface0')
    else:
        raise ValueError("unknown capture transport")


def open_capture_port(options):
    if options.transport == "usb_bulk":
        from sensorarray_usb_bulk import UsbBulkPort
        return UsbBulkPort(options.usb_serial, vid=options.usb_vid, pid=transport_usb_pid(options))
    if options.transport == 'usb_cdc':
        from sensorarray_usb_cdc import UsbCdcPort
        return UsbCdcPort(options.usb_serial, vid=options.usb_vid, pid=transport_usb_pid(options))
    return PosixPort(options.port)


class Collector:
    def __init__(self, options: CaptureOptions, port, raw, events, *, ctrl_raw=None):
        self.options, self.port, self.raw, self.events = options, port, raw, events
        self.transport_metadata = port.metadata()
        self.is_bulk = self.transport_metadata.get("transport") == "usb_bulk"
        self.is_cdc = self.transport_metadata.get("transport") == "usb_cdc"
        self.is_otg = self.is_bulk or self.is_cdc
        if self.is_bulk and ctrl_raw is None:
            raise ValueError("bulk_requires_separate_CTRL_raw_recording")
        self.ctrl_raw = ctrl_raw
        self.ctrl_raw_bytes = 0
        self.ctrl_raw_sha = hashlib.sha256()
        self.ctrl_line_buffer = bytearray()
        self.observation_channel = "data"
        self.started = time.monotonic()
        self.total_deadline = self.started + options.duration
        self.cleanup_budget = min(10.0, options.duration / 4)
        self.capture_deadline = self.total_deadline - self.cleanup_budget
        self.prepare_deadline = min(self.started + 45.0, self.capture_deadline)
        self.phase = "prepare"
        self.raw_bytes = 0
        self.tx_bytes = 0
        self.partial_writes = 0
        self.uncorrelated_query_errors = 0
        self.line_buffer = bytearray()
        self.lines = deque()
        self.original_usb = None
        self.restore_required = False
        self.restore_status = "NOT_NEEDED"
        self.capture_started = None
        self.capture_finished = None
        self.capture_trailers = 0
        self.faults = []
        self.reset_observations = []
        self.replies = {}
        self.next_progress = self.started + options.progress_seconds
        self.event("opened", port=options.port, transport=self.transport_metadata,
                   modem_bit_ioctl=False, reset_requested=False, input_flushed=False,
                   hupcl=False if not self.is_bulk else None,
                   duration_total_seconds=options.duration,
                   raw_layout="DATA_rx_and_CTRL_ctrl_rx_separate" if self.is_bulk else
                   "formal_CDC_DATA_CTRL_shared_raw" if self.is_cdc else "single_POSIX_rx")

    def event(self, kind, **fields):
        self.events.write(json.dumps({"t": time.monotonic() - self.started,
                                      "type": kind, "phase": self.phase, **fields}) + "\n")
        self.events.flush()

    def progress(self):
        now = time.monotonic()
        if now >= self.next_progress:
            print(json.dumps({"type": "capture_progress", "phase": self.phase,
                              "elapsed_seconds": round(now - self.started, 3),
                              "rx_bytes": self.raw_bytes, "capture_trailers": self.capture_trailers}), flush=True)
            self.next_progress = now + self.options.progress_seconds

    def pump(self, deadline):
        self.progress()
        limit = min(deadline, self.total_deadline)
        if limit <= time.monotonic():
            return
        received = self.port.read_some(65536, limit)
        if received is None:
            return
        channel, chunk = received
        if channel not in {"data", "ctrl"} or not chunk:
            raise CaptureError("invalid_transport_read_result")
        if channel == "ctrl":
            if not self.is_bulk or self.ctrl_raw is None:
                raise CaptureError("unexpected_CTRL_endpoint")
            offset = self.ctrl_raw_bytes
            if self.ctrl_raw.write(chunk) != len(chunk):
                raise CaptureError("CTRL_raw_record_partial_write")
            self.ctrl_raw.flush()
            self.ctrl_raw_sha.update(chunk)
            self.ctrl_raw_bytes += len(chunk)
            self.event("ctrl_rx", offset=offset, length=len(chunk), channel=channel)
            pending = self.ctrl_line_buffer
        else:
            offset = self.raw_bytes
            if self.raw.write(chunk) != len(chunk):
                raise CaptureError("DATA_raw_record_partial_write")
            self.raw.flush()
            self.raw_bytes += len(chunk)
            self.event("rx", offset=offset, length=len(chunk), channel=channel)
            pending = self.line_buffer
        pending.extend(chunk)
        while b"\n" in pending:
            line_bytes, _, remainder = pending.partition(b"\n")
            pending[:] = remainder
            line = line_bytes.rstrip(b"\r").decode("ascii", errors="backslashreplace")
            self.observation_channel = channel
            self.lines.append(ReceivedLine(line, channel))
            if line.startswith(("RST,", "ESP-ROM:")) or "rst:0x" in line.lower():
                if len(self.reset_observations) < 16:
                    self.reset_observations.append({"phase": self.phase, "line": line, "channel": channel})
                self.event("reset_observed", line=line, channel=channel, causation="unknown")
                if self.phase == "capture":
                    self.faults.append("reset_observed_during_capture:" + line)
            if self.phase == "capture" and channel == "data" and line.startswith("K,"):
                self.capture_trailers += 1
            if line.startswith(("MFAULT,", "APP_FATAL", "FDC_FATAL")) or any(
                    marker in line.lower() for marker in ("guru meditation", "task watchdog got triggered", "interrupt wdt timeout")):
                self.faults.append(line)
                self.event("firmware_fault", line=line, channel=channel)
        if len(pending) > 65536:
            self.event("line_overflow", channel=channel, buffered_bytes=len(pending))
            raise CaptureError("transport_line_exceeded_65536_bytes:" + channel)

    def send(self, command: str, deadline):
        # Terminate a previously interrupted command without flushing RX.
        # Bulk writes route exclusively to CTRL OUT in UsbBulkPort.
        payload = ("\n" + command + "\n").encode("ascii")
        offset = 0
        self.event("tx_begin", command=command, wire_ascii=payload.decode("ascii"),
                   channel="ctrl" if self.is_bulk else "data")
        while offset < len(payload):
            limit = min(deadline, self.total_deadline)
            if limit <= time.monotonic():
                self.event("tx_incomplete", command=command, accepted_bytes=offset, total_bytes=len(payload))
                raise CaptureError(f"write_timeout:{command}")
            written = self.port.write_some(payload[offset:], limit)
            if not isinstance(written, int) or not 0 <= written <= len(payload) - offset:
                raise CaptureError("transport_invalid_write_count")
            if written > 0:
                self.partial_writes += int(written < len(payload) - offset)
                offset += written
                self.tx_bytes += written
            self.progress()
        self.event("tx_complete", command=command, accepted_bytes=offset,
                   acceptance=self.transport_metadata["tx_acceptance"])

    def validate_protocol(self, fields):
        capabilities = set(re.split(r"[/;,|\s]+", fields.get("caps", "")))
        required = 'usb_cdc_data_v1' if self.is_cdc else 'usb_vendor_bulk_v1'
        if self.is_otg and (fields.get("version") != "2" or not {required, "config_query_v1"} <= capabilities):
            raise CaptureError("formal_USB_capability_required:" + required)

    def transport_result(self):
        return {"transport": self.transport_metadata["transport"],
                "transport_metadata": self.port.metadata(),
                "raw_scope": "DATA_endpoint_only" if self.is_bulk else "formal_CDC_DATA_CTRL_shared_stream" if self.is_cdc else "POSIX_serial_stream",
                "ctrl_rx_bytes": None if self.is_cdc else self.ctrl_raw_bytes,
                "ctrl_raw_sha256": self.ctrl_raw_sha.hexdigest() if self.is_bulk else None,
                "trailing_partial_CTRL_line_bytes": None if self.is_cdc else len(self.ctrl_line_buffer),
                "CTRL_byte_accounting": "included_in_shared_raw_not_a_separate_endpoint_counter" if self.is_cdc else "separate_endpoint_when_bulk",
                "tx_host_accepted_bytes": self.tx_bytes,
                "uncorrelated_query_errors": self.uncorrelated_query_errors,
                "CTRL_and_DATA_order": "independent_endpoints_complete_lines_only" if self.is_bulk else "one_stream"}

    def command_deadline(self, phase_deadline):
        deadline = min(phase_deadline, self.total_deadline,
                       time.monotonic() + self.options.command_timeout)
        if deadline <= time.monotonic():
            raise CaptureError("phase_deadline_expired")
        return deadline

    def next_line(self, deadline):
        while time.monotonic() < deadline:
            if self.lines:
                line = self.lines.popleft()
                if self.is_bulk and getattr(line, "channel", None) != "ctrl":
                    continue  # DATA text cannot satisfy a control reply/terminal.
                return line
            self.pump(deadline)
        raise CaptureError("reply_deadline_expired")

    def wait(self, predicate, deadline, command):
        while time.monotonic() < deadline:
            line = self.next_line(deadline)
            if line.startswith("ERR,"):
                self.event("command_error", command=command, line=line)
                raise CaptureError(f"device_error:{command}:{line}")
            if predicate(line):
                self.event("reply", command=command, line=line)
                return line
        raise CaptureError(f"reply_timeout:{command}")

    def query(self, command, tag, phase_deadline):
        deadline = self.command_deadline(phase_deadline)
        # Already-read complete lines remain in the raw record but cannot
        # satisfy a query issued later. This is not a device input flush.
        self.lines.clear()
        self.send(command, deadline)
        while time.monotonic() < deadline:
            line = self.next_line(deadline)
            if line.startswith("ERR,"):
                # One possible source is a tty echoing a device record prefix
                # before raw mode is applied; send()'s leading LF would then
                # complete that old input. The actual source remains unknown.
                # Keep it as evidence; it neither rejects this query nor
                # proves success. Never extend the deadline or replay a setter.
                if (command in READONLY_QUERY_COMMANDS and
                        line == "ERR,cmd=UNKNOWN,reason=unsupported"):
                    self.uncorrelated_query_errors += 1
                    self.event("uncorrelated_error", command=command, line=line,
                               correlation="unknown_command_identity",
                               action="wait_readonly_reply_with_original_deadline")
                    continue
                self.event("command_error", command=command, line=line)
                raise CaptureError(f"device_error:{command}:{line}")
            if line.startswith(tag + ","):
                self.event("reply", command=command, line=line)
                fields = parse_fields(line)
                self.replies[command] = fields
                return fields
        raise CaptureError("reply_deadline_expired")

    def set_usb(self, mode, phase_deadline):
        if self.is_otg:
            raise CaptureError("USBSTREAM_setter_is_legacy_USJ_only")
        deadline = self.command_deadline(phase_deadline)
        self.lines.clear()
        self.send(f"USBSTREAM={mode}", deadline)
        line = self.wait(lambda value: value.startswith("USBSTREAM,") and
                         parse_fields(value).get("v") == mode and
                         parse_fields(value).get("state") == "applied",
                         deadline, f"USBSTREAM={mode}")
        return parse_fields(line)

    def transition(self, kind, value):
        deadline = self.command_deadline(self.prepare_deadline)
        self.lines.clear()
        self.send(f"{kind}={value}", deadline)
        # ROWMODES uses new=<profile> in RMACK, but profile=<profile> in
        # RMAPP/RMERR. These are distinct from the homogeneous MODE domain.
        ack_tag, applied_tag, error_tag, ack_value, applied_value = {
            "MODE": ("MACK", "MAPP", "MERR", "new", "new"),
            "ROWS": ("RCMD", "RAPP", "RERR", "req", "new"),
            "ROWMODES": ("RMACK", "RMAPP", "RMERR", "new", "profile"),
        }[kind]
        request_id = None
        terminals = {}
        while time.monotonic() < deadline:
            line = self.next_line(deadline)
            tag = line.split(",", 1)[0]
            fields = parse_fields(line)
            if tag == "ERR":
                raise CaptureError(f"device_error:{kind}:{line}")
            if tag == ack_tag and fields.get(ack_value) == str(value):
                request_id = fields.get("id")
                if not request_id:
                    raise CaptureError(f"missing_request_id:{line}")
                self.event("accepted", command=f"{kind}={value}", line=line)
            elif tag == applied_tag and fields.get(applied_value) == str(value):
                terminals[fields.get("id")] = line
            elif tag == error_tag:
                if request_id is None or fields.get("id") == request_id:
                    raise CaptureError(f"transition_rejected:{line}")
            if request_id and request_id in terminals:
                self.event("applied", command=f"{kind}={value}", line=terminals[request_id])
                break
        else:
            raise CaptureError(f"transition_terminal_timeout:{kind}={value}")
        if kind == "MODE":
            snapshot = self.query("MODE?", "MODE", self.prepare_deadline)
            if snapshot.get("active") != str(value):
                raise CaptureError(f"mode_verification_failed:{snapshot}")
        elif kind == "ROWS":
            snapshot = self.query("ROWS?", "ROWS", self.prepare_deadline)
            if (snapshot.get("active") != str(value) or
                    snapshot.get("pending", str(value)) != str(value) or
                    snapshot.get("requestId", "0") != "0"):
                raise CaptureError(f"rows_verification_failed:{snapshot}")
        else:
            snapshot = self.query("ROWMODES?", "ROWMODES", self.prepare_deadline)
            if (snapshot.get("active") != str(value) or
                    snapshot.get("pending") != "none" or
                    snapshot.get("state") != "applied" or
                    snapshot.get("rid") != request_id):
                raise CaptureError(f"rowmodes_verification_failed:{snapshot}")

    def run(self):
        failure = None
        try:
            for command, tag in (("BUILD?", "BUILD"), ("PROTO?", "PROTO"), ("MODE?", "MODE"),
                                 ("ROWS?", "ROWS"), ("ROWMODES?", "ROWMODES"), ("CAL?", "CAL"),
                                 ("RAIL?", "ARL"), ("USB?", "USB") if self.is_otg else ("USBSTREAM?", "USBSTREAM")):
                fields = self.query(command, tag, self.prepare_deadline)
                if command == "PROTO?":
                    self.validate_protocol(fields)
                if command == "PROTO?" and fields.get("version", "1").isdigit() and int(fields.get("version", "1")) >= 2:
                    self.query("CONFIG?", "CONFIG", self.prepare_deadline)
                if command == "USBSTREAM?":
                    if fields.get("v") not in {"DEBUG", "FULL"}:
                        raise CaptureError(f"unknown_usb_profile:{fields}")
                    self.original_usb = fields
            if self.options.mode:
                self.transition("MODE", self.options.mode)
            if self.options.rows:
                self.transition("ROWS", self.options.rows)
            if self.options.rowmodes:
                # Establish the requested row coverage before applying its
                # profile. Each command is terminal+query verified in order.
                self.transition("ROWMODES", self.options.rowmodes)
            # The write may apply even if its reply times out; cleanup must
            # therefore be armed before attempting this reversible mutation.
            if self.is_otg:
                self.restore_status = "NOT_APPLICABLE_CDC_ALWAYS_FULL" if self.is_cdc else "NOT_APPLICABLE_BULK_ALWAYS_FULL"
            else:
                self.restore_required = True
                self.set_usb("FULL", self.prepare_deadline)
            self.phase = "capture"
            self.capture_started = time.monotonic()
            self.event("capture_start")
            self.lines.clear()
            while time.monotonic() < self.capture_deadline:
                self.pump(self.capture_deadline)
                self.lines.clear()
                if self.faults:
                    raise CaptureError("firmware_fault_during_capture")
            self.capture_finished = time.monotonic()
            self.event("capture_end", frame_trailers=self.capture_trailers)
            if not self.capture_trailers:
                raise CaptureError("no_measurement_frame_trailers_during_capture")
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            failure = str(error) or "interrupted"
            self.event("error", reason=failure)
        finally:
            if self.capture_started is not None and self.capture_finished is None:
                self.capture_finished = time.monotonic()
            self.phase = "cleanup"
            if self.restore_required and self.original_usb:
                try:
                    mode = self.original_usb["v"]
                    restored = self.set_usb(mode, self.total_deadline)
                    # Setter reports configured cadence; compare old values
                    # rather than silently assuming the label restores them.
                    for key in ("dataEvery", "diagEvery"):
                        if key in self.original_usb and restored.get(key) != self.original_usb[key]:
                            raise CaptureError(f"restore_cadence_mismatch:{key}")
                    verified = self.query("USBSTREAM?", "USBSTREAM", self.total_deadline)
                    if any(verified.get(key) != self.original_usb.get(key) for key in ("v", "dataEvery", "diagEvery") if key in self.original_usb):
                        raise CaptureError("restore_query_mismatch")
                    self.restore_status = "VERIFIED_QUERY"
                except (OSError, CaptureError, KeyboardInterrupt) as error:
                    self.restore_status = "ERROR"
                    failure = (failure + ";" if failure else "") + "usb_restore_failed:" + str(error)
                    self.event("restore_error", reason=str(error))
            self.event("finished", status="ERROR" if failure else "CAPTURED", error=failure,
                       usb_restore=self.restore_status)
        return {**self.transport_result(), "status": "ERROR" if failure else "CAPTURED", "error": failure,
                "measurement_acceptance": "NOT_EVALUATED", "calibration_written": False,
                "port": self.options.port, "mode_requested": self.options.mode,
                "rowmodes_requested": self.options.rowmodes,
                "rows_requested": self.options.rows, "duration_total_budget_seconds": self.options.duration,
                "elapsed_seconds": time.monotonic() - self.started,
                "prepare_seconds": (self.capture_started - self.started) if self.capture_started else None,
                "capture_seconds": (self.capture_finished - self.capture_started) if self.capture_started else 0.0,
                "capture_frame_trailers": self.capture_trailers,
                "frame_trailers_are_not_crc_or_value_validation": True,
                "rx_bytes": self.raw_bytes, "tx_os_accepted_bytes": self.tx_bytes,
                "partial_writes": self.partial_writes, "usb_restore": self.restore_status,
                "initial_usb_profile": self.original_usb, "replies": self.replies,
                "firmware_faults": self.faults, "reset_observations": self.reset_observations,
                "modem_bit_ioctl": False, "reset_requested": False, "flashed": False,
                "open_reset_cannot_be_excluded_by_software_intent": True}


def capture(options: CaptureOptions) -> dict:
    validate_transport_options(options)
    if options.mode and options.rowmodes:
        raise ValueError("--mode and --rowmodes are mutually exclusive")
    if options.rowmodes is not None and not re.fullmatch(r"[CR]{8}", options.rowmodes):
        raise ValueError("--rowmodes requires eight C/R characters for this passive reference fixture")
    options.output_directory.mkdir(parents=True, exist_ok=True)
    paths = {"raw": options.output_directory / f"{options.label}.rx.bin",
             "events": options.output_directory / f"{options.label}.events.jsonl",
             "summary": options.output_directory / f"{options.label}.summary.json"}
    if options.transport == "usb_bulk":
        paths["ctrl_raw"] = options.output_directory / f"{options.label}.ctrl.rx.bin"
    if any(path.exists() for path in paths.values()):
        raise FileExistsError("capture output exists; choose a new --label")
    # Every output uses exclusive creation. No old scientific data is replaced.
    with ExitStack() as stack:
        raw = stack.enter_context(paths["raw"].open("xb"))
        ctrl_raw = stack.enter_context(paths["ctrl_raw"].open("xb")) if "ctrl_raw" in paths else None
        events = stack.enter_context(paths["events"].open("x", encoding="utf-8"))
        summary_file = stack.enter_context(paths["summary"].open("x", encoding="utf-8"))
        port = None
        try:
            port = open_capture_port(options)
            result = Collector(options, port, raw, events, ctrl_raw=ctrl_raw).run()
        except (OSError, CaptureError) as error:
            result = {"status": "ERROR", "error": str(error), "measurement_acceptance": "NOT_EVALUATED",
                      "modem_bit_ioctl": False, "reset_requested": False, "flashed": False}
        finally:
            if port:
                try:
                    port.close()
                except OSError as error:
                    result["close_error"] = str(error)
                    result["status"] = "ERROR"
        raw.flush()
        result["raw_file"] = str(paths["raw"])
        result["events_file"] = str(paths["events"])
        if "ctrl_raw" in paths:
            result["ctrl_raw_file"] = str(paths["ctrl_raw"])
        result["raw_sha256"] = hashlib.sha256(paths["raw"].read_bytes()).hexdigest()
        summary_file.write(json.dumps(result, indent=2) + "\n")
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_transport_arguments(parser)
    parser.add_argument("--duration", type=float, default=150.0, help="Total wall-clock budget, including preparation and restore.")
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--label", required=True)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--mode", choices=("CAP", "RES"))
    selection.add_argument("--rowmodes", help="Eight C/R characters, e.g. RCCCCCCR; applied after --rows and verified by RMAPP plus ROWMODES?.")
    parser.add_argument("--rows", type=int, choices=range(1, 9))
    parser.add_argument("--command-timeout", type=float, default=10.0, help="Per-command budget, at most 10 s.")
    parser.add_argument("--progress-seconds", type=float, default=20.0, help="Progress interval, at most 20 s.")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.label):
        parser.error("--label accepts only letters, digits, underscore and hyphen")
    if not 0 < args.duration <= 3600 or not 0 < args.command_timeout <= 10 or not 0 < args.progress_seconds <= 20:
        parser.error("duration must be in (0,3600], command-timeout (0,10], progress-seconds (0,20]")
    try:
        result = capture(CaptureOptions(**vars(args)))
    except (OSError, ValueError) as error:
        parser.exit(2, f"CAPTURE_REFERENCE_ERROR: {error}\n")
    print(json.dumps({"status": result["status"], "error": result.get("error"),
                      "rx_bytes": result.get("rx_bytes", 0), "capture_seconds": result.get("capture_seconds", 0),
                      "usb_restore": result.get("usb_restore", "NOT_NEEDED"), "raw_file": result["raw_file"]}), flush=True)
    return 0 if result["status"] == "CAPTURED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
