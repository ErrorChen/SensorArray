#!/usr/bin/env python3
"""Structured CAP/VOLT/RES serial validation for SensorArray hardware.

Known components are opt-in acceptance hints. They are never assumed by the
firmware or by this tool unless explicitly supplied on the command line.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional, Union

from text_protocol import CapFrame, MeasurementFrame, TextProtocolParser


Frame = Union[CapFrame, MeasurementFrame]
COORDINATE = re.compile(r"^S([1-8])D([1-8])$")


def parse_coordinate(value: str) -> tuple[int, int]:
    match = COORDINATE.match(value.upper())
    if not match:
        raise argparse.ArgumentTypeError("coordinate must look like S1D1")
    return int(match.group(1)), int(match.group(2))


def parse_known_resistor(value: str) -> tuple[tuple[int, int], float]:
    try:
        coordinate, expected = value.split("=", 1)
        resistance = float(expected)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "known resistor must look like S1D1=10002.5"
        ) from error
    if resistance <= 0:
        raise argparse.ArgumentTypeError("known resistance must be positive")
    return parse_coordinate(coordinate), resistance


def parse_fields(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in line.split(",")[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            result[key] = value
    return result


@dataclass
class StageResult:
    mode: str
    rows: int
    frames: int = 0
    observed_frames: int = 0
    crc_errors: int = 0
    sequence_first: int = 0
    sequence_last: int = 0
    elapsed_seconds: float = 0.0
    fps: float = 0.0
    acquisition_fps: float = 0.0
    valid_cells_min: int = 64
    error_cells_max: int = 0
    freshness_failures: int = 0
    frame_duration_us_max: int = 0
    gain_changes_max: int = 0
    autorange_attempts_max: int = 0
    autorange_fallbacks_max: int = 0
    io_retries_total: int = 0
    overrange_total: int = 0
    drdy_timeouts_total: int = 0
    stale_total: int = 0
    spi_errors_total: int = 0


@dataclass
class ValidationResult:
    port: str
    baud: int
    started_at: str
    firmware_ads_boot: str = ""
    self_test: str = ""
    protocol_self_test: str = ""
    fdc_boot: str = ""
    rail_status: dict[str, str] = field(default_factory=dict)
    mode_status: dict[str, dict[str, str]] = field(default_factory=dict)
    stages: list[StageResult] = field(default_factory=list)
    known_resistors: dict[str, dict[str, object]] = field(default_factory=dict)
    known_capacitors: dict[str, dict[str, object]] = field(default_factory=dict)
    probed_cells: dict[str, dict[str, str]] = field(default_factory=dict)
    cycles_completed: int = 0
    ads_frame_events: list[dict[str, str]] = field(default_factory=list)
    ads_error_summary: dict[str, int] = field(default_factory=dict)
    resets: int = 0
    watchdogs: int = 0
    fatal_lines: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)


def summarise_ads_frame_events(events: list[dict[str, str]]) -> dict[str, int]:
    totals = {
        "events": len(events),
        "timeouts": 0,
        "stale": 0,
        "spi": 0,
        "status": 0,
        "overrange": 0,
        "max_consecutive_io_frames": 0,
    }
    last_sequence: Optional[int] = None
    consecutive = 0
    for fields in events:
        def number(name: str) -> int:
            try:
                return int(fields.get(name, "0"), 0)
            except ValueError:
                return 0

        totals["timeouts"] += number("timeout")
        totals["stale"] += number("stale")
        totals["spi"] += number("spi")
        totals["status"] += number("status")
        totals["overrange"] += number("overrange")
        sequence = number("seq")
        has_io_error = any(number(name) != 0 for name in
                           ("timeout", "stale", "spi", "status"))
        if has_io_error:
            consecutive = consecutive + 1 if (
                last_sequence is not None and sequence == last_sequence + 1
            ) else 1
            last_sequence = sequence
            totals["max_consecutive_io_frames"] = max(
                totals["max_consecutive_io_frames"], consecutive
            )
        else:
            consecutive = 0
            last_sequence = None
    return totals


class SerialValidator:
    def __init__(self, connection: object, raw_log: object,
                 result: ValidationResult) -> None:
        self.connection = connection
        self.raw_log = raw_log
        self.result = result
        self.protocol = TextProtocolParser()
        self.frames: list[tuple[float, Frame]] = []

    def write_command(self, command: str) -> None:
        wire = command.rstrip("\r\n") + "\n"
        self.raw_log.write(f"HOST>{wire}")
        self.raw_log.flush()
        self.connection.write(wire.encode("ascii"))

    def read_once(self) -> tuple[str, Optional[Frame]]:
        raw = self.connection.readline()
        if not raw:
            return "", None
        self.raw_log.write(raw.decode("ascii", errors="backslashreplace"))
        self.raw_log.flush()
        try:
            line = raw.rstrip(b"\r\n").decode("ascii")
        except UnicodeDecodeError:
            self.protocol.note_non_ascii()
            return "", None
        frame = self.protocol.feed_line(line)
        if frame is not None:
            self.frames.append((time.monotonic(), frame))
        self._observe_line(line)
        return line, frame

    def _observe_line(self, line: str) -> None:
        if line.startswith("ADSBOOT,"):
            self.result.firmware_ads_boot = line
        elif line.startswith("MSELF,"):
            self.result.self_test = line
        elif line.startswith("PSELF,"):
            self.result.protocol_self_test = line
        elif line.startswith(("APP_FDC,stage=boot_sweep_return",
                              "APP_FDC,stage=boot_sweep_skip")):
            self.result.fdc_boot = line
        elif line.startswith("ADSFRAME,"):
            self.result.ads_frame_events.append(parse_fields(line))
        lowered = line.lower()
        if line.startswith("RST,") or "rst:" in lowered:
            self.result.resets += 1
        if any(marker in lowered for marker in
               ("task watchdog", "interrupt wdt", "rst=task_wdt",
                "wdt timeout", "wdt_reset")):
            self.result.watchdogs += 1
        if line.startswith(("APP_FATAL", "MFAULT", "Guru Meditation")):
            self.result.fatal_lines.append(line)

    def wait_line(self, predicate: Callable[[str], bool], timeout: float,
                  description: str) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line, _ = self.read_once()
            if line and predicate(line):
                return line
        raise RuntimeError(f"timeout waiting for {description}")

    def wait_frame(self, predicate: Callable[[Frame], bool], timeout: float,
                   description: str) -> Frame:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, frame = self.read_once()
            if frame is not None and predicate(frame):
                return frame
        raise RuntimeError(f"timeout waiting for {description}")

    def switch_mode(self, mode: str, timeout: float) -> tuple[int, int]:
        frame_start = len(self.frames)
        self.write_command(f"MODE={mode}")
        ack_id: Optional[int] = None
        applied_sequence: Optional[int] = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and (ack_id is None or applied_sequence is None):
            line, _ = self.read_once()
            if line.startswith("MACK,"):
                fields = parse_fields(line)
                if fields.get("new") == mode:
                    ack_id = int(fields["id"], 10)
            elif line.startswith("MAPP,"):
                fields = parse_fields(line)
                if fields.get("new") == mode:
                    applied_id = int(fields["id"], 10)
                    if ack_id is not None and applied_id != ack_id:
                        raise RuntimeError(
                            f"mode request id mismatch ack={ack_id} applied={applied_id}"
                        )
                    ack_id = applied_id if ack_id is None else ack_id
                    applied_sequence = int(fields["seq"], 10)
            elif line.startswith("MERR,"):
                raise RuntimeError(f"mode transition failed: {line}")
        if ack_id is None or applied_sequence is None:
            raise RuntimeError(f"missing accepted/applied event for MODE={mode}")
        expected_mode = {"CAP": None, "VOLT": "VOLT", "RES": "RES"}[mode]
        def matches(item: Frame) -> bool:
            return ((mode == "CAP" and isinstance(item, CapFrame)) or
                    (isinstance(item, MeasurementFrame) and
                     item.mode == expected_mode))

        post_request_frames = [item for _, item in self.frames[frame_start:]
                               if item.sequence >= applied_sequence]
        wrong_mode = [item for item in post_request_frames if not matches(item)]
        if wrong_mode:
            first = wrong_mode[0]
            raise RuntimeError(
                f"MODE={mode} mixed frame seq={first.sequence} at/after "
                f"applied seq {applied_sequence}"
            )
        candidates = [item for item in post_request_frames if matches(item)]
        frame = candidates[0] if candidates else self.wait_frame(
            lambda item: matches(item) and item.sequence >= applied_sequence,
            timeout, f"first {mode} frame")
        # Output decimation is independent of acquisition: the applied frame
        # itself may not be published. It is sufficient that the first
        # published frame is at/after the boundary and has only the new mode.
        return ack_id, applied_sequence

    def query_mode(self, mode: str, timeout: float) -> dict[str, str]:
        self.write_command("MODE?")
        line = self.wait_line(lambda item: item.startswith("MODE,"), timeout,
                              "MODE? snapshot")
        fields = parse_fields(line)
        expected = {
            "CAP": {"active": "CAP", "sw": "HIGH", "source": "GND",
                    "matrixRef": "GND", "intref": "0", "vbias": "1",
                    "refmux": "24"},
            "VOLT": {"active": "VOLT", "sw": "HIGH", "source": "GND",
                     "matrixRef": "GND", "intref": "0", "vbias": "1",
                     "refmux": "24"},
            "RES": {"active": "RES", "sw": "LOW", "source": "REF",
                    "matrixRef": "REFOUT", "intref": "1", "vbias": "1",
                    "refmux": "00"},
        }[mode]
        for key, value in expected.items():
            if fields.get(key) != value:
                raise RuntimeError(
                    f"MODE? {mode} expected {key}={value}, got {fields.get(key)}"
                )
        self.result.mode_status[mode] = fields
        return fields

    def set_rail_calibration(self, avdd_uv: int, avss_uv: int,
                             timeout: float) -> dict[str, str]:
        self.write_command(f"RAILCFG={avdd_uv},{avss_uv}")
        accepted = self.wait_line(
            lambda line: line.startswith("RACK,"), timeout,
            "RAILCFG accepted",
        )
        request_id = parse_fields(accepted).get("id")
        if request_id is None:
            raise RuntimeError(f"RAILCFG accepted without id: {accepted}")
        self.wait_line(
            lambda line: line.startswith("RAPP,") and
                         parse_fields(line).get("id") == request_id,
            timeout,
            "RAILCFG applied",
        )
        self.write_command("RAIL?")
        line = self.wait_line(
            lambda item: item.startswith("ARL,"), timeout,
            "RAILCFG snapshot",
        )
        fields = parse_fields(line)
        expected = {
            "src": "external",
            "avdd": str(avdd_uv),
            "avss": str(avss_uv),
            "rv": "1",
        }
        for key, value in expected.items():
            if fields.get(key) != value:
                raise RuntimeError(
                    f"RAILCFG expected {key}={value}, got {fields.get(key)}"
                )
        self.result.rail_status = fields
        return fields

    def set_rows(self, rows: int, timeout: float) -> None:
        self.write_command(f"ROWS={rows}")
        accepted = self.wait_line(lambda line: line.startswith("RCMD,"), timeout,
                                  f"ROWS={rows} accepted")
        request_id = int(parse_fields(accepted)["id"], 10)
        applied = self.wait_line(
            lambda line: line.startswith("RAPP,") and
                         parse_fields(line).get("id") == str(request_id),
            timeout,
            f"ROWS={rows} applied",
        )
        if int(parse_fields(applied)["new"], 10) != rows:
            raise RuntimeError(f"wrong applied row count: {applied}")

    def sample_stage(self, mode: str, rows: int, timeout: float,
                     frame_count: int = 2) -> StageResult:
        expected = "RES" if mode == "RES" else mode
        samples: list[tuple[float, Frame]] = []
        stage = StageResult(mode=mode, rows=rows)
        expected_fresh = (1 << (rows * 8)) - 1
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and len(samples) < frame_count:
            _, frame = self.read_once()
            if frame is None or frame.rows != rows:
                continue
            if mode == "CAP" and not isinstance(frame, CapFrame):
                continue
            if mode != "CAP" and (not isinstance(frame, MeasurementFrame) or
                                  frame.mode != expected):
                continue
            stage.observed_frames += 1
            freshness_ok = True
            if isinstance(frame, CapFrame):
                expected_rows = (1 << rows) - 1
                freshness_ok = (
                    frame.row_fresh_mask == expected_rows and
                    frame.primary_fresh_mask == expected_rows and
                    frame.secondary_fresh_mask == expected_rows
                )
            else:
                freshness_ok = frame.fresh_mask == expected_fresh
            if not frame.crc_ok:
                stage.crc_errors += 1
                continue
            if not freshness_ok:
                stage.freshness_failures += 1
                continue
            samples.append((time.monotonic(), frame))
        if len(samples) < frame_count:
            raise RuntimeError(
                f"only {len(samples)} clean {mode} rows={rows} frames "
                f"from {stage.observed_frames} observed"
            )
        stage.frames = len(samples)
        stage.sequence_first = samples[0][1].sequence
        stage.sequence_last = samples[-1][1].sequence
        stage.elapsed_seconds = samples[-1][0] - samples[0][0]
        if stage.elapsed_seconds > 0 and len(samples) > 1:
            stage.fps = (len(samples) - 1) / stage.elapsed_seconds
            stage.acquisition_fps = (
                samples[-1][1].sequence - samples[0][1].sequence
            ) / stage.elapsed_seconds
        for _, frame in samples:
            if isinstance(frame, CapFrame):
                valid = sum(value != -1_000_000 for value in frame.values_fixed)
                errors = frame.cells - valid
            else:
                valid = bin(frame.valid_mask).count("1")
                errors = bin(frame.error_mask).count("1")
                stage.frame_duration_us_max = max(
                    stage.frame_duration_us_max, frame.frame_duration_us
                )
                stage.gain_changes_max = max(
                    stage.gain_changes_max, frame.gain_change_count
                )
                stage.autorange_attempts_max = max(
                    stage.autorange_attempts_max,
                    frame.autorange_attempt_count,
                )
                stage.autorange_fallbacks_max = max(
                    stage.autorange_fallbacks_max,
                    frame.autorange_fallback_count,
                )
                stage.io_retries_total += frame.io_retry_count
                stage.overrange_total += frame.overrange_count
                stage.drdy_timeouts_total += frame.drdy_timeout_count
                stage.stale_total += frame.stale_count
                stage.spi_errors_total += frame.spi_error_count
            stage.valid_cells_min = min(stage.valid_cells_min, valid)
            stage.error_cells_max = max(stage.error_cells_max, errors)
        self.result.stages.append(stage)
        return stage

    def query_cell(self, coordinate: tuple[int, int], timeout: float) -> dict[str, str]:
        row, d_line = coordinate
        self.write_command(f"CELL?=S{row}D{d_line}")
        line = self.wait_line(
            lambda item: item.startswith("CELL,") and
                         parse_fields(item).get("cell") == f"S{row}D{d_line}",
            timeout,
            f"CELL?=S{row}D{d_line}",
        )
        return parse_fields(line)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=120.0,
                        help="Total validation time budget in seconds.")
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=60.0,
        help=("Bounded wait for self-tests, ADS identity, FDC boot calibration, "
              "and the first fresh dual-FDC CAP frame."),
    )
    parser.add_argument("--rows", default="1,2,4,8")
    parser.add_argument("--modes", default="CAP,VOLT,RES")
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--known-resistor", action="append", default=[],
                        type=parse_known_resistor)
    parser.add_argument("--known-capacitor", action="append", default=[],
                        type=parse_coordinate)
    parser.add_argument("--probe-cell", action="append", default=[],
                        type=parse_coordinate,
                        help="Record CELL? telemetry without imposing an expected value.")
    parser.add_argument("--rail-avdd-uv", type=int,
                        help="Externally measured AVDD above GND in microvolts.")
    parser.add_argument("--rail-avss-uv", type=int,
                        help="Externally measured signed AVSS below GND in microvolts.")
    parser.add_argument("--resistor-tolerance-percent", type=float, default=25.0)
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help="Do not hard-reset the target after opening the serial port.",
    )
    parser.add_argument("--output-directory", type=Path)
    args = parser.parse_args()
    rows = [int(value) for value in args.rows.split(",") if value]
    modes = [value.strip().upper() for value in args.modes.split(",") if value]
    if any(value not in {1, 2, 4, 8} for value in rows):
        parser.error("--rows accepts a comma-separated subset of 1,2,4,8")
    if any(value not in {"CAP", "VOLT", "RES"} for value in modes):
        parser.error("--modes accepts CAP,VOLT,RES")
    if (args.rail_avdd_uv is None) != (args.rail_avss_uv is None):
        parser.error("--rail-avdd-uv and --rail-avss-uv must be supplied together")
    if "VOLT" in modes and args.rail_avdd_uv is None:
        parser.error("VOLT validation requires explicit --rail-avdd-uv and --rail-avss-uv")
    if args.rail_avdd_uv is not None and (
            args.rail_avdd_uv <= 0 or args.rail_avss_uv is None or
            args.rail_avss_uv >= 0):
        parser.error("rail calibration requires AVDD > 0 and signed AVSS < 0")
    if args.startup_timeout <= 0:
        parser.error("--startup-timeout must be positive")
    stamp = time.strftime("%Y%m%d-%H%M%S")
    output_dir = args.output_directory or Path("validation_artifacts") / stamp
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "serial.log"
    json_path = output_dir / "summary.json"
    result = ValidationResult(port=args.port, baud=args.baud,
                              started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"))
    try:
        import serial
    except ImportError:
        print("VALIDATION_FAILED,reason=missing_pyserial")
        return 2

    connection = serial.Serial()
    connection.port = args.port
    connection.baudrate = args.baud
    connection.timeout = 0.25
    connection.write_timeout = 1.0
    connection.dtr = False
    connection.rts = False
    try:
        connection.open()
    except Exception as error:
        print(f"VALIDATION_FAILED,reason=serial_open,detail={error}")
        return 2

    validator: Optional[SerialValidator] = None
    try:
        with raw_path.open("w", encoding="utf-8", newline="") as raw_log:
            validator = SerialValidator(connection, raw_log, result)
            if not args.no_reset:
                # Match esptool's HardReset behaviour, including its Windows
                # usbser.sys workaround: repeat the current DTR state after
                # each RTS change so the combined control-line state is sent.
                # DTR remains false, therefore IO0 stays high and the target
                # boots the application rather than the ROM downloader.
                connection.reset_input_buffer()
                connection.setDTR(False)
                connection.setRTS(True)
                connection.setDTR(False)
                time.sleep(0.2)
                connection.reset_input_buffer()
                connection.setRTS(False)
                connection.setDTR(False)
                time.sleep(0.5)
            startup_deadline = time.monotonic() + args.startup_timeout
            while time.monotonic() < startup_deadline:
                validator.read_once()
                if (result.firmware_ads_boot and result.self_test and
                        result.protocol_self_test):
                    break
            if "passed=1" not in result.self_test:
                raise RuntimeError("measurement self-test did not pass")
            if "passed=1" not in result.protocol_self_test:
                raise RuntimeError("protocol self-test did not pass")
            # ADSBOOT is emitted before the required FDC boot sweep. Sending
            # MODE at that point can be accepted by Core 0 but cannot be
            # applied by Core 1 until calibration ends. A fresh dual-FDC CAP
            # frame is the actual runtime-ready boundary and avoids treating a
            # correctly deferred request as a mode failure.
            startup_remaining = startup_deadline - time.monotonic()
            if startup_remaining <= 0:
                raise RuntimeError("startup deadline expired before CAP readiness")
            validator.wait_frame(
                lambda frame: isinstance(frame, CapFrame) and frame.crc_ok and
                    frame.row_fresh_mask == (1 << frame.rows) - 1 and
                    frame.primary_fresh_mask == (1 << frame.rows) - 1 and
                    frame.secondary_fresh_mask == (1 << frame.rows) - 1,
                startup_remaining,
                "first fresh dual-FDC CAP frame after boot calibration",
            )
            # A single boot breadcrumb/reset line is expected when the serial
            # port opens. Fault counters below cover only the validation run.
            result.resets = 0
            result.watchdogs = 0
            result.fatal_lines.clear()
            validator.write_command("ST=SER")
            validator.wait_line(lambda line: line.startswith("ACK,cmd=ST"), 5.0,
                                "ST=SER acknowledgement")
            validator.write_command("TX=FULL")
            validator.wait_line(lambda line: line.startswith("ACK,cmd=TX"), 5.0,
                                "TX=FULL acknowledgement")
            validator.write_command("FPSCAP=OFF")
            validator.write_command("OUTCAP=OFF")
            validator.write_command("RAIL?")
            rail_line = validator.wait_line(
                lambda line: line.startswith("ARL,"), 10.0, "RAIL? snapshot"
            )
            result.rail_status = parse_fields(rail_line)

            stage_count = max(1, len(modes) * len(rows) + args.cycles * 4)
            stage_timeout = max(5.0, args.duration / stage_count * 3.0)
            latest_frames: dict[str, Frame] = {}
            for mode in modes:
                if mode == "VOLT" and args.rail_avdd_uv is not None:
                    validator.set_rail_calibration(
                        args.rail_avdd_uv, args.rail_avss_uv, stage_timeout
                    )
                validator.switch_mode(mode, stage_timeout)
                validator.query_mode(mode, stage_timeout)
                for row_count in rows:
                    validator.set_rows(row_count, stage_timeout)
                    validator.sample_stage(mode, row_count, stage_timeout)
                validator.set_rows(8, stage_timeout)
                latest_frames[mode] = validator.wait_frame(
                    lambda frame: frame.rows == 8 and
                        ((mode == "CAP" and isinstance(frame, CapFrame)) or
                         (isinstance(frame, MeasurementFrame) and
                          frame.mode == ("RES" if mode == "RES" else mode))),
                    stage_timeout,
                    f"{mode} rows=8 acceptance frame",
                )

            cap_frame = latest_frames.get("CAP")
            if isinstance(cap_frame, CapFrame):
                for row, d_line in args.known_capacitor:
                    index = (row - 1) * 8 + d_line - 1
                    valid = cap_frame.values_fixed[index] != -1_000_000
                    key = f"S{row}D{d_line}"
                    result.known_capacitors[key] = {
                        "valid": valid,
                        "fixed_pf6": cap_frame.values_fixed[index],
                    }
                    if not valid:
                        result.failures.append(f"known capacitor {key} invalid in CAP")

            if "RES" in modes:
                validator.switch_mode("RES", stage_timeout)
                validator.set_rows(8, stage_timeout)
                validator.wait_frame(
                    lambda frame: isinstance(frame, MeasurementFrame) and
                                  frame.mode == "RES" and frame.rows == 8,
                    stage_timeout,
                    "RES rows=8 before cell queries",
                )
                known_by_coordinate = dict(args.known_resistor)
                probe_coordinates = list(dict.fromkeys(
                    list(args.probe_cell) + list(known_by_coordinate)
                ))
                for coordinate in probe_coordinates:
                    row, d_line = coordinate
                    key = f"S{row}D{d_line}"
                    fields = validator.query_cell(coordinate, stage_timeout)
                    result.probed_cells[key] = fields
                    if coordinate not in known_by_coordinate:
                        continue
                    expected_ohms = known_by_coordinate[coordinate]
                    measured_ohms: Optional[float] = None
                    if fields.get("valid") == "1" and fields.get("scale") == "-3":
                        measured_ohms = int(fields["value"], 10) / 1000.0
                    error_percent = None if measured_ohms is None else abs(
                        measured_ohms - expected_ohms) / expected_ohms * 100.0
                    accepted = (error_percent is not None and
                                error_percent <= args.resistor_tolerance_percent)
                    result.known_resistors[key] = {
                        "expected_ohms": expected_ohms,
                        "measured_ohms": measured_ohms,
                        "error_percent": error_percent,
                        "accepted": accepted,
                        "telemetry": fields,
                    }
                    if not accepted:
                        result.failures.append(
                            f"known resistor {key} outside tolerance or invalid"
                        )

            for _ in range(args.cycles):
                for mode in ("CAP", "VOLT", "RES", "CAP"):
                    if mode == "VOLT" and args.rail_avdd_uv is not None:
                        validator.set_rail_calibration(
                            args.rail_avdd_uv, args.rail_avss_uv, stage_timeout
                        )
                    validator.switch_mode(mode, stage_timeout)
                    validator.query_mode(mode, stage_timeout)
                result.cycles_completed += 1
    except Exception as error:
        result.failures.append(str(error))
    finally:
        connection.close()

    result.ads_error_summary = summarise_ads_frame_events(result.ads_frame_events)
    if result.ads_error_summary["max_consecutive_io_frames"] >= 3:
        result.failures.append("sustained ADS I/O/status errors for 3 or more frames")

    if validator is not None:
        if validator.protocol.counters.crc_errors:
            result.failures.append(
                f"total CRC errors={validator.protocol.counters.crc_errors}"
            )
        if validator.protocol.counters.non_ascii_chunks:
            result.failures.append(
                f"non-ASCII chunks={validator.protocol.counters.non_ascii_chunks}"
            )
    if result.resets or result.watchdogs or result.fatal_lines:
        result.failures.append(
            f"runtime faults resets={result.resets} watchdogs={result.watchdogs} "
            f"fatal={len(result.fatal_lines)}"
        )
    json_path.write_text(json.dumps(asdict(result), indent=2, ensure_ascii=False),
                         encoding="utf-8")
    passed = not result.failures
    print(f"MEASUREMENT_VALIDATION,passed={int(passed)},artifact={output_dir},"
          f"cycles={result.cycles_completed},failures={len(result.failures)}")
    for failure in result.failures:
        print(f"VALIDATION_FAILURE,{failure}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
