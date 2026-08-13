#!/usr/bin/env python3
"""Repeatable serial and BLE HIL validation for SensorArray firmware.

The tool deliberately keeps all firmware protocol parsing in ``text_protocol``.
This module owns orchestration only: port/device selection, command/reply
correlation, mode-boundary checks, BLE subscription matrices, stress phases,
and reset/panic observation.

Exit status:
    0 - requested HIL phases passed
    1 - hardware was reached but an acceptance check failed
    2 - prerequisite unavailable or target could not be selected safely
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import json
import os
import re
import statistics
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Set, TextIO, Tuple

from text_protocol import (CapFrame, FragmentReassembler, MeasurementFrame,
                           TextProtocolParser, parse_fields)


EXIT_PASS = 0
EXIT_FAIL = 1
EXIT_SKIPPED = 2

BLE_BASE_SUFFIX = "-0000-1000-8000-00805f9b34fb"


def normalizeUuid(value: str) -> str:
    """Return a lower-case Bluetooth UUID, accepting 16-bit shorthand."""

    text = value.strip().lower().strip("{}")
    if text.startswith("0x"):
        text = text[2:]
    if re.fullmatch(r"[0-9a-f]{1,4}", text):
        return "0000%s%s" % (text.zfill(4), BLE_BASE_SUFFIX)
    if re.fullmatch(r"[0-9a-f]{8}", text):
        return "%s%s" % (text, BLE_BASE_SUFFIX)
    try:
        return str(uuid.UUID(text)).lower()
    except (ValueError, AttributeError) as error:
        raise ValueError("invalid BLE UUID: %s" % value) from error


SERVICE_UUID = normalizeUuid("00FF")
CTRL_RX_UUID = normalizeUuid("FF10")
CTRL_TX_UUID = normalizeUuid("FF11")
DATA_TX_UUID = normalizeUuid("FF20")
LOG_TX_UUID = normalizeUuid("FF30")

CHANNEL_UUIDS = {
    "C": CTRL_TX_UUID,
    "D": DATA_TX_UUID,
    "L": LOG_TX_UUID,
}

CHANNEL_ALIASES = {
    "11": "C", "FF11": "C", "C": "C", "CTRL": "C", "CONTROL": "C",
    "20": "D", "FF20": "D", "D": "D", "DATA": "D",
    "30": "L", "FF30": "L", "L": "L", "LOG": "L",
}

MODE_NAMES = {"CAP", "RES", "VOLT"}
DEFAULT_QUERY_COMMANDS = ("TX?", "ST?", "BTX?", "WIFI?", "MODE?", "STATE?")
FULL_LOG_CASE_REQUIRED_TAGS = frozenset(
    ("SF50", "TR50", "ADS50", "ADST50", "AB50"))
BLE_UNSUBSCRIBE_SETTLE_SECONDS = 0.15
BLE_UNSUBSCRIBE_QUIET_SECONDS = 0.20


class HilFailure(RuntimeError):
    """The target was reached, but an acceptance condition failed."""


class HilSkipped(RuntimeError):
    """A host prerequisite or unambiguous target selection was unavailable."""


def serialMissingEmissions(previousSequence: int, currentSequence: int) -> int:
    """Return omitted USB publications while rejecting stale sequencing."""

    sequenceDelta = currentSequence - previousSequence
    if sequenceDelta <= 0:
        raise HilFailure(
            "invalid sequence %d -> %d; sequence must advance" %
            (previousSequence, currentSequence))
    return sequenceDelta - 1


@dataclasses.dataclass(frozen=True)
class SerialCandidate:
    device: str
    description: str
    hardwareId: str


@dataclasses.dataclass(frozen=True)
class FaultEvent:
    kind: str
    source: str
    line: str
    observedAt: float


@dataclasses.dataclass(frozen=True)
class LineRecord:
    observedAt: float
    channel: str
    line: str


@dataclasses.dataclass(frozen=True)
class FrameRecord:
    observedAt: float
    channel: str
    frame: Any


@dataclasses.dataclass(frozen=True)
class KnownResistance:
    row: int
    drive: int
    minimumOhms: float
    maximumOhms: float

    @property
    def label(self) -> str:
        return "S%dD%d" % (self.row, self.drive)

    @property
    def index(self) -> int:
        return (self.row - 1) * 8 + self.drive - 1


class FaultDetector:
    """Classify reset and panic evidence only after the test is armed.

    Serial ports can expose an already-running boot banner when first opened.
    Callers therefore establish a CRC-valid frame first and then arm the
    detector.  Every matching line after that boundary is unexpected.
    """

    _panicMarkers = (
        "guru meditation", "stack canary", "panic'ed", "panic handler",
        "loadprohibited", "storeprohibited", "cache_resume_icache",
        "abort() was called",
    )
    _watchdogMarkers = (
        "task watchdog", "interrupt wdt", "watchdog timeout", "wdt timeout",
        "wdt_reset", "rst=task_wdt",
    )

    def __init__(self) -> None:
        self._armed = False
        self._events: List[FaultEvent] = []
        self._ignored: List[FaultEvent] = []
        self._lock = threading.Lock()

    @staticmethod
    def classify(line: str) -> Optional[str]:
        lowered = line.lower()
        if any(marker in lowered for marker in FaultDetector._panicMarkers):
            return "panic"
        if any(marker in lowered for marker in FaultDetector._watchdogMarkers):
            return "watchdog"
        if lowered.startswith("watchdog"):
            return "watchdog"
        if line.startswith(("APP_FATAL", "MFAULT", "BLECORRUPT", "LOGTRUNC",
                            "TXDROP", "CMDERR")):
            return "firmware_fault"
        if ("esp-rom:" in lowered or lowered.startswith("esp-rom") or
                re.search(r"(^|[,\s])rst:", lowered) or
                lowered.startswith("rst,reason=")):
            return "reset"
        return None

    def arm(self, clearExisting: bool = True) -> None:
        with self._lock:
            if clearExisting:
                self._events.clear()
            self._armed = True

    def observe(self, line: str, source: str) -> Optional[FaultEvent]:
        kind = self.classify(line)
        if kind is None:
            return None
        event = FaultEvent(kind, source, line, time.monotonic())
        with self._lock:
            # A boot/reset banner seen before the first ready frame is an
            # expected startup boundary.  A panic, watchdog or firmware fault
            # is never benign merely because the device later rebooted and
            # produced a valid frame.
            if self._armed or kind != "reset":
                self._events.append(event)
            else:
                self._ignored.append(event)
        return event

    def events(self) -> List[FaultEvent]:
        with self._lock:
            return list(self._events)

    def ignoredEvents(self) -> List[FaultEvent]:
        with self._lock:
            return list(self._ignored)

    def assertHealthy(self) -> None:
        events = self.events()
        if events:
            first = events[0]
            raise HilFailure("unexpected %s on %s: %s" %
                             (first.kind, first.source, first.line))


class HilOutput:
    """Thread-safe console/status writer with an optional full wire log."""

    # Notification callbacks must return promptly on Windows.  Flushing every
    # wire line can synchronously block the WinRT callback hundreds of times a
    # second during an 8x8 RES stream, which in turn can make the host terminate
    # an otherwise healthy BLE link.  Status lines remain immediately durable;
    # raw wire data is flushed at this bounded interval and again on close.
    WIRE_FLUSH_INTERVAL_SECONDS = 1.0

    def __init__(self, logPath: Optional[Path]) -> None:
        self._lock = threading.Lock()
        self._logFile: Optional[TextIO] = None
        self._lastWireFlushAt = time.monotonic()
        if logPath is not None:
            logPath.parent.mkdir(parents=True, exist_ok=True)
            self._logFile = logPath.open("w", encoding="utf-8", newline="\n")

    def emit(self, line: str) -> None:
        with self._lock:
            print(line, flush=True)
            if self._logFile:
                self._logFile.write(line + "\n")
                self._logFile.flush()
                self._lastWireFlushAt = time.monotonic()

    def wire(self, source: str, line: str) -> None:
        with self._lock:
            if self._logFile:
                self._logFile.write("%s>%s\n" % (source, line))
                now = time.monotonic()
                if now - self._lastWireFlushAt >= self.WIRE_FLUSH_INTERVAL_SECONDS:
                    self._logFile.flush()
                    self._lastWireFlushAt = now

    def close(self) -> None:
        with self._lock:
            if self._logFile:
                self._logFile.flush()
                self._logFile.close()
                self._logFile = None


def enumerateSerialPorts() -> List[SerialCandidate]:
    try:
        from serial.tools import list_ports
    except ImportError as error:
        raise HilSkipped("missing pyserial; install tools/requirements-hil.txt") from error
    candidates = [
        SerialCandidate(item.device, item.description or "", item.hwid or "")
        for item in list_ports.comports()
    ]
    return sorted(candidates, key=lambda item: item.device.upper())


def selectSerialPort(explicitPort: Optional[str], environmentPort: Optional[str],
                     candidates: Sequence[SerialCandidate]) -> str:
    """Select a port without guessing when more than one candidate exists."""

    if explicitPort:
        return explicitPort
    if environmentPort:
        return environmentPort
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        raise HilSkipped("no serial ports found")
    raise HilSkipped("multiple serial ports found; use --port or SENSORARRAY_SERIAL_PORT")


def printSerialCandidates(output: HilOutput,
                          candidates: Sequence[SerialCandidate]) -> None:
    if not candidates:
        output.emit("SERIAL_CANDIDATE,count=0")
        return
    for item in candidates:
        output.emit("SERIAL_CANDIDATE,port=%s,description=%s,hwid=%s" %
                    (item.device, _safeField(item.description),
                     _safeField(item.hardwareId)))


def _safeField(value: str) -> str:
    return value.replace("\r", " ").replace("\n", " ").replace(",", ";")


def parseKnownResistance(value: str) -> KnownResistance:
    match = re.fullmatch(
        r"S([1-8])D([1-8]):([0-9]+(?:\.[0-9]+)?):([0-9]+(?:\.[0-9]+)?)",
        value.strip().upper(),
    )
    if not match:
        raise argparse.ArgumentTypeError(
            "expected S<1-8>D<1-8>:<minimum-ohms>:<maximum-ohms>")
    minimum = float(match.group(3))
    maximum = float(match.group(4))
    if minimum < 0.0 or maximum <= minimum:
        raise argparse.ArgumentTypeError("resistance range must satisfy 0 <= min < max")
    return KnownResistance(int(match.group(1)), int(match.group(2)), minimum, maximum)


def defaultKnownResistances() -> List[KnownResistance]:
    """Return a fresh default list so argparse callers never share mutations."""

    return [parseKnownResistance("S1D1:5000:20000"),
            parseKnownResistance("S8D8:5000:20000")]


def parseModes(value: str) -> List[str]:
    modes = [item.strip().upper() for item in value.split(",") if item.strip()]
    if not modes or any(item not in MODE_NAMES for item in modes):
        raise argparse.ArgumentTypeError("modes must be a comma-separated CAP/RES/VOLT list")
    return modes


def parseSubscriptionCase(value: str) -> Set[str]:
    text = value.strip().upper()
    if text in {"", "NONE", "0", "-"}:
        return set()
    channels: Set[str] = set()
    for token in re.split(r"[+/]", text):
        canonical = CHANNEL_ALIASES.get(token.strip())
        if canonical is None:
            raise argparse.ArgumentTypeError("unknown BLE subscription channel: %s" % token)
        channels.add(canonical)
    return channels


def parseSubscriptionCases(value: str) -> List[Set[str]]:
    cases = [parseSubscriptionCase(item) for item in value.split(",")]
    if not cases:
        raise argparse.ArgumentTypeError("at least one subscription case is required")
    return cases


def formatSubscriptionCase(channels: Iterable[str]) -> str:
    ordered = [item for item in ("C", "D", "L") if item in set(channels)]
    return "+".join(ordered) if ordered else "NONE"


def frameMode(frame: Any) -> str:
    if isinstance(frame, CapFrame):
        return "CAP"
    if isinstance(frame, MeasurementFrame):
        return frame.mode
    return "UNKNOWN"


def isRuntimeReadyFrame(frame: Any) -> bool:
    if not bool(getattr(frame, "crc_ok", False)):
        return False
    if isinstance(frame, CapFrame):
        expectedMask = (1 << frame.rows) - 1
        return (frame.row_fresh_mask == expectedMask and
                frame.primary_fresh_mask == expectedMask and
                frame.secondary_fresh_mask == expectedMask)
    return isinstance(frame, MeasurementFrame)


def isPreApplyFrame(frame: Any, appliedSequence: Optional[int]) -> bool:
    """Identify an in-flight frame acquired before an apply boundary.

    BLE delivery can lag acquisition.  Consequently, a frame acquired just
    before MAPP may arrive after the host has observed MAPP.  The sequence in
    MAPP is the authoritative boundary: pre-boundary frames are drained,
    whereas any wrong-mode frame at or after the boundary remains a hard
    stale-frame failure.
    """

    return (appliedSequence is not None and
            getattr(frame, "sequence", appliedSequence) < appliedSequence)


def expectedCommandPrefix(command: str) -> str:
    text = command.strip().upper()
    if text in {"MODE?", "STATE?"}:
        return "MODE,"
    stem = re.split(r"[=?]", text, maxsplit=1)[0]
    return "ACK,cmd=%s" % stem


def isTransientSnapshotBusy(command: str, line: str) -> bool:
    """Identify the bounded seqlock collision used by MODE?/STATE?."""

    if command.strip().upper() not in ("MODE?", "STATE?") or not line.startswith("ERR,"):
        return False
    fields = parse_fields(line)
    return (fields.get("cmd", "").upper() == "MODE" and
            fields.get("reason", "").lower() == "snapshot_busy")


def validateModeFrame(frame: Any, mode: str, requestId: int,
                      generation: Optional[int], appliedSequence: Optional[int]) -> None:
    if frameMode(frame) != mode:
        raise HilFailure("stale/mixed frame after MODE=%s: got %s seq=%s" %
                         (mode, frameMode(frame), getattr(frame, "sequence", "na")))
    if not frame.crc_ok:
        raise HilFailure("internal frame CRC failed for %s seq=%d" %
                         (mode, frame.sequence))
    if appliedSequence is not None and frame.sequence < appliedSequence:
        raise HilFailure("frame seq=%d precedes applied seq=%d" %
                         (frame.sequence, appliedSequence))
    # R/V frame gen/rid fields belong to measurement-mode state.  The
    # compatibility C/K frame contract predates mode switching and keeps those
    # fields assigned to scan-config (ROWS) generation/request id.  CAP mode
    # correlation therefore comes from MAPP plus sequence/type, not by
    # reinterpreting the established C/K fields.
    if isinstance(frame, MeasurementFrame):
        if frame.request_id != requestId:
            raise HilFailure("frame request id=%d, expected %d" %
                             (frame.request_id, requestId))
        if generation is not None and frame.generation != generation:
            raise HilFailure("frame generation=%d, expected %d" %
                             (frame.generation, generation))
    if frame.cells != frame.rows * 8:
        raise HilFailure("incomplete frame rows=%d cells=%d" %
                         (frame.rows, frame.cells))


def checkKnownResistances(frames: Sequence[Any],
                          ranges: Sequence[KnownResistance]) -> Dict[str, float]:
    evidence: Dict[str, float] = {}
    resistanceFrames = [item for item in frames
                        if isinstance(item, MeasurementFrame) and item.mode == "RES"]
    for expected in ranges:
        values = []
        for frame in resistanceFrames:
            if expected.index >= len(frame.values_si):
                continue
            value = frame.values_si[expected.index]
            if value is not None:
                values.append(float(value))
        if not values:
            raise HilFailure("%s had no valid RES samples" % expected.label)
        minimumValid = max(1, (len(resistanceFrames) + 1) // 2)
        if len(values) < minimumValid:
            raise HilFailure("%s valid RES samples %d/%d below 50%%" %
                             (expected.label, len(values), len(resistanceFrames)))
        median = float(statistics.median(values))
        if not expected.minimumOhms < median < expected.maximumOhms:
            raise HilFailure("%s median %.3f ohm outside %.3f..%.3f" %
                             (expected.label, median, expected.minimumOhms,
                              expected.maximumOhms))
        evidence[expected.label] = median
    return evidence


def _slashIntegers(value: str, expectedCount: int) -> List[int]:
    parts = value.split("/")
    if len(parts) != expectedCount:
        raise HilFailure("expected %d slash-separated integers, got %s" %
                         (expectedCount, value))
    try:
        return [int(item, 10) for item in parts]
    except ValueError as error:
        raise HilFailure("invalid integer telemetry: %s" % value) from error


def validateStackTelemetry(records: Sequence[LineRecord], minimumLogBytes: int,
                           minimumOtherBytes: int, heapLeakToleranceBytes: int,
                           required: bool) -> Dict[str, Any]:
    """Validate cumulative task/pool telemetry emitted by STK50.

    Counter checks use deltas from the first post-warmup sample so pre-existing
    diagnostic history does not create a false failure.  Stack high-water marks
    are lifetime minima and are therefore checked directly on the last sample.
    """

    samples = [parse_fields(item.line) for item in records
               if item.line.startswith("STK50,")]
    if not samples:
        if required:
            raise HilFailure("no STK50 telemetry received during full long-run")
        return {"samples": 0, "required": False}
    first = samples[0]
    last = samples[-1]
    taskFields = ("log", "transport", "usb", "bleTx", "bleCtrl", "serialCtrl")
    tasks: Dict[str, Dict[str, int]] = {}
    for taskName in taskFields:
        configured, minimum = _slashIntegers(last[taskName], 2)
        tasks[taskName] = {"configuredBytes": configured,
                           "minimumRemainingBytes": minimum}
        threshold = minimumLogBytes if taskName == "log" else minimumOtherBytes
        if minimum < threshold:
            raise HilFailure("%s stack minimum %d bytes below %d" %
                             (taskName, minimum, threshold))

    scalarCounters = ("ta", "ts", "tr", "ba", "bs", "br", "bc", "bf", "trunc")
    counterDelta: Dict[str, Any] = {}
    for fieldName in scalarCounters:
        delta = int(last[fieldName], 10) - int(first[fieldName], 10)
        counterDelta[fieldName] = delta
        if delta:
            raise HilFailure("STK50 counter %s increased by %d" %
                             (fieldName, delta))
    firstQueue = _slashIntegers(first["tq"], 2)
    lastQueue = _slashIntegers(last["tq"], 2)
    queueDelta = [lastQueue[index] - firstQueue[index] for index in range(2)]
    counterDelta["queueDropData"] = queueDelta[0]
    counterDelta["queueDropLog"] = queueDelta[1]
    if any(queueDelta):
        raise HilFailure("transport queue drops increased DATA=%d LOG=%d" %
                         (queueDelta[0], queueDelta[1]))

    heapSamples = [_slashIntegers(item["heap"], 3) for item in samples]
    currentHeap = [item[1] for item in heapSamples]
    heapDelta = currentHeap[-1] - currentHeap[0]
    monotonicallyFalling = (len(currentHeap) >= 3 and
                            all(currentHeap[index] < currentHeap[index - 1]
                                for index in range(1, len(currentHeap))))
    if monotonicallyFalling and -heapDelta > heapLeakToleranceBytes:
        raise HilFailure("free heap fell monotonically by %d bytes" % -heapDelta)
    transportSlots = _slashIntegers(last["tSlot"], 2)
    bleSlots = _slashIntegers(last["bSlot"], 2)
    return {
        "samples": len(samples),
        "tasks": tasks,
        "heap": {
            "initialFreeBytes": heapSamples[-1][0],
            "warmupFreeBytes": currentHeap[0],
            "finalFreeBytes": currentHeap[-1],
            "minimumFreeBytes": heapSamples[-1][2],
            "warmupToFinalDeltaBytes": heapDelta,
            "monotonicallyFalling": monotonicallyFalling,
        },
        "transportSlots": {"used": transportSlots[0], "highWater": transportSlots[1]},
        "bleSlots": {"used": bleSlots[0], "highWater": bleSlots[1]},
        "counterDelta": counterDelta,
        "firstSequence": int(first.get("seq", "0"), 10),
        "lastSequence": int(last.get("seq", "0"), 10),
    }


def validateBleFirmwareCounters(records: Sequence[LineRecord],
                                required: bool,
                                allowedMessageDrops: int = 0) -> Dict[str, Any]:
    samples = [parse_fields(item.line) for item in records
               if item.line.startswith("BL50,")]
    if not samples:
        if required:
            raise HilFailure("no BL50 counter telemetry received")
        return {"samples": 0, "required": False}
    first = samples[0]
    last = samples[-1]
    deltas = {}
    for fieldName in ("md", "fe", "tiny", "cg", "dropD", "dropL", "dropC",
                      "ctrlExhaust"):
        if fieldName not in first or fieldName not in last:
            continue
        delta = int(last[fieldName], 10) - int(first[fieldName], 10)
        deltas[fieldName] = delta
        if fieldName in ("cg", "dropD", "dropL"):
            continue
        allowed = allowedMessageDrops if fieldName == "md" else 0
        if delta > allowed:
            raise HilFailure("BL50 counter %s increased by %d" %
                             (fieldName, delta))
    return {
        "samples": len(samples),
        "counterDelta": deltas,
        "channelDropDelta": {
            "DATA": deltas.get("dropD"),
            "LOG": deltas.get("dropL"),
            "CTRL": deltas.get("dropC"),
        },
        "last": last,
    }


def telemetryWindowRecords(monitor: "SerialMonitor", startIndex: int) -> List[LineRecord]:
    before = monitor.lines[:startIndex]
    baseline: List[LineRecord] = []
    for prefix in ("BL50,", "STK50,"):
        match = next((item for item in reversed(before)
                      if item.line.startswith(prefix)), None)
        if match is not None:
            baseline.append(match)
    return baseline + monitor.lines[startIndex:]


class SerialMonitor:
    """Single-reader serial monitor shared by serial HIL and BLE sidecar."""

    def __init__(self, connection: Any, port: str, output: HilOutput,
                 detector: FaultDetector) -> None:
        self.connection = connection
        self.port = port
        self.output = output
        self.detector = detector
        self.protocol = TextProtocolParser()
        self.lines: List[LineRecord] = []
        self.frames: List[FrameRecord] = []
        self.nonAsciiLines = 0
        self.readerError = ""
        self._condition = threading.Condition()
        self._writeLock = threading.Lock()
        self._stopEvent = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @classmethod
    def open(cls, port: str, baud: int, output: HilOutput,
             detector: FaultDetector) -> "SerialMonitor":
        try:
            import serial
        except ImportError as error:
            raise HilSkipped("missing pyserial; install tools/requirements-hil.txt") from error
        connection = serial.Serial()
        connection.port = port
        connection.baudrate = baud
        connection.timeout = 0.2
        connection.write_timeout = 1.0
        # Keep both control lines inactive before opening.  Unlike a flash or
        # boot validator, HIL must not intentionally reset the target.
        connection.dtr = False
        connection.rts = False
        try:
            connection.open()
        except Exception as error:
            raise HilSkipped("cannot open serial port %s: %s" % (port, error)) from error
        return cls(connection, port, output, detector)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._readerLoop,
                                        name="sensorarray-serial-monitor",
                                        daemon=True)
        self._thread.start()
        self.output.emit("SERIAL_OPEN,port=%s,baud=%s" %
                         (self.port, self.connection.baudrate))

    def close(self) -> None:
        self._stopEvent.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self.connection.close()
        finally:
            self.output.emit("SERIAL_CLOSE,port=%s" % self.port)

    def _readerLoop(self) -> None:
        try:
            while not self._stopEvent.is_set():
                raw = self.connection.readline()
                if not raw:
                    continue
                try:
                    line = raw.rstrip(b"\r\n").decode("ascii")
                except UnicodeDecodeError:
                    self.protocol.note_non_ascii()
                    self.nonAsciiLines += 1
                    self.output.wire("SER_ASCII_ERROR", repr(bytes(raw)))
                    continue
                observedAt = time.monotonic()
                self.output.wire("SER", line)
                event = self.detector.observe(line, "serial")
                if event and self.detector.events():
                    self.output.emit("HIL_FAULT,source=serial,kind=%s,line=%s" %
                                     (event.kind, _safeField(event.line)))
                frame = self.protocol.feed_line(line)
                with self._condition:
                    self.lines.append(LineRecord(observedAt, "S", line))
                    if frame is not None:
                        self.frames.append(FrameRecord(observedAt, "S", frame))
                    self._condition.notify_all()
        except Exception as error:  # pragma: no cover - host driver dependent
            self.readerError = repr(error)
            with self._condition:
                self._condition.notify_all()

    def send(self, command: str) -> None:
        wire = command.rstrip("\r\n") + "\n"
        self.output.wire("HOST_SERIAL", wire.rstrip("\n"))
        with self._writeLock:
            self.connection.write(wire.encode("ascii"))
            self.connection.flush()

    def lineCount(self) -> int:
        with self._condition:
            return len(self.lines)

    def frameCount(self) -> int:
        with self._condition:
            return len(self.frames)

    def waitLine(self, predicate: Callable[[str], bool], startIndex: int,
                 timeout: float, description: str) -> Tuple[LineRecord, int]:
        deadline = time.monotonic() + timeout
        cursor = startIndex
        with self._condition:
            while True:
                while cursor < len(self.lines):
                    record = self.lines[cursor]
                    cursor += 1
                    if predicate(record.line):
                        return record, cursor
                self.detector.assertHealthy()
                if self.readerError:
                    raise HilFailure("serial reader failed: %s" % self.readerError)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise HilFailure("timeout waiting for %s" % description)
                self._condition.wait(min(remaining, 0.25))

    def waitFrame(self, predicate: Callable[[Any], bool], startIndex: int,
                  timeout: float, description: str) -> Tuple[FrameRecord, int]:
        deadline = time.monotonic() + timeout
        cursor = startIndex
        with self._condition:
            while True:
                while cursor < len(self.frames):
                    record = self.frames[cursor]
                    cursor += 1
                    if predicate(record.frame):
                        return record, cursor
                self.detector.assertHealthy()
                if self.readerError:
                    raise HilFailure("serial reader failed: %s" % self.readerError)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise HilFailure("timeout waiting for %s" % description)
                self._condition.wait(min(remaining, 0.25))


def waitSerialCommand(monitor: SerialMonitor, command: str,
                      timeout: float) -> str:
    deadline = time.monotonic() + timeout
    expectedPrefix = expectedCommandPrefix(command)
    commandStem = re.split(r"[=?]", command.strip().upper(), maxsplit=1)[0]
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise HilFailure("timeout waiting for %s response after snapshot retries" %
                             command)
        startIndex = monitor.lineCount()
        monitor.send(command)

        def matches(line: str) -> bool:
            if line.startswith(expectedPrefix) or isTransientSnapshotBusy(command, line):
                return True
            if line.startswith("ERR,"):
                errorCommand = parse_fields(line).get("cmd", "").upper()
                if errorCommand == commandStem or (
                        commandStem == "STATE" and errorCommand == "MODE"):
                    raise HilFailure("command rejected: %s" % line)
            return False

        record, _ = monitor.waitLine(matches, startIndex, remaining,
                                     "%s response" % command)
        if isTransientSnapshotBusy(command, record.line):
            monitor.detector.assertHealthy()
            time.sleep(0.01)
            continue
        return record.line


def waitSerialCommandReady(monitor: SerialMonitor, command: str,
                           timeout: float) -> str:
    """Retry the first control query while lengthy boot calibration finishes."""
    deadline = time.monotonic() + timeout
    lastError = ""
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            return waitSerialCommand(monitor, command, min(3.0, remaining))
        except HilFailure as error:
            monitor.detector.assertHealthy()
            if monitor.readerError:
                raise
            lastError = str(error)
    raise HilFailure("control task not ready for %s within %.1fs: %s" %
                     (command, timeout, lastError))


def setSerialRailCalibration(monitor: SerialMonitor, avddUv: int,
                             avssUv: int, timeout: float) -> Dict[str, int]:
    """Apply one explicit board rail split and verify its frame-boundary commit."""
    lineStart = monitor.lineCount()
    command = "RAILCFG=%d,%d" % (avddUv, avssUv)
    monitor.send(command)
    ackRecord, _ = monitor.waitLine(
        lambda line: line.startswith("RACK,"), lineStart, timeout,
        "RAILCFG RACK")
    ackFields = parse_fields(ackRecord.line)
    requestId = int(ackFields["id"], 10)
    if (int(ackFields.get("avdd", "0"), 10) != avddUv or
            int(ackFields.get("avss", "0"), 10) != avssUv):
        raise HilFailure("RAILCFG acknowledgement mismatch: %s" % ackRecord.line)
    appliedRecord, _ = monitor.waitLine(
        lambda line: (line.startswith("RAPP,") and
                      parse_fields(line).get("id") == str(requestId) and
                      parse_fields(line).get("source") == "external" and
                      parse_fields(line).get("state") == "applied"),
        lineStart, timeout, "RAILCFG RAPP")
    appliedFields = parse_fields(appliedRecord.line)
    if (int(appliedFields.get("avdd", "0"), 10) != avddUv or
            int(appliedFields.get("avss", "0"), 10) != avssUv):
        raise HilFailure("RAILCFG application mismatch: %s" % appliedRecord.line)
    return {
        "requestId": requestId,
        "generation": int(appliedFields["gen"], 10),
        "appliedSequence": int(appliedFields["seq"], 10),
        "avddUv": avddUv,
        "avssUv": avssUv,
    }


def switchSerialMode(monitor: SerialMonitor, mode: str, frameTarget: int,
                     timeout: float) -> Tuple[List[Any], Dict[str, int]]:
    lineStart = monitor.lineCount()
    frameStart = monitor.frameCount()
    monitor.send("MODE=%s" % mode)
    ackRecord, _ = monitor.waitLine(
        lambda line: line.startswith("MACK,") and parse_fields(line).get("new") == mode,
        lineStart, timeout, "MODE=%s MACK" % mode)
    ackFields = parse_fields(ackRecord.line)
    requestId = int(ackFields["id"], 10)
    appliedRecord, _ = monitor.waitLine(
        lambda line: (line.startswith("MAPP,") and
                      parse_fields(line).get("new") == mode and
                      parse_fields(line).get("id") == str(requestId)),
        lineStart, timeout, "MODE=%s MAPP" % mode)
    appliedFields = parse_fields(appliedRecord.line)
    generation = int(appliedFields["gen"], 10)
    appliedSequence = int(appliedFields["seq"], 10)

    acceptedFrames: List[Any] = []
    cursor = frameStart
    deadline = time.monotonic() + timeout
    while len(acceptedFrames) < frameTarget:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise HilFailure("MODE=%s produced %d/%d frames" %
                             (mode, len(acceptedFrames), frameTarget))
        record, cursor = monitor.waitFrame(lambda _frame: True, cursor, remaining,
                                           "MODE=%s frame" % mode)
        if (record.observedAt < appliedRecord.observedAt or
                isPreApplyFrame(record.frame, appliedSequence)):
            continue
        validateModeFrame(record.frame, mode, requestId, generation, appliedSequence)
        acceptedFrames.append(record.frame)
    evidence = {"requestId": requestId, "generation": generation,
                "appliedSequence": appliedSequence}
    return acceptedFrames, evidence


def collectSerialLongRun(monitor: SerialMonitor, mode: str,
                         initialEvidence: Dict[str, int], frameTarget: int,
                         secondsTarget: float, maximumSeconds: float,
                         queryInterval: float, frameTimeout: float,
                         knownRanges: Sequence[KnownResistance]) -> Dict[str, Any]:
    startedAt = time.monotonic()
    deadline = startedAt + maximumSeconds
    nextQueryAt = startedAt + queryInterval if queryInterval > 0.0 else float("inf")
    cursor = monitor.frameCount()
    frames: List[Any] = []
    stateReplies: List[Dict[str, str]] = []
    lastSequence: Optional[int] = None
    sequenceGaps = 0
    missingEmissions = 0
    while len(frames) < frameTarget or time.monotonic() - startedAt < secondsTarget:
        now = time.monotonic()
        if now >= deadline:
            raise HilFailure("%s long-run deadline reached at frames=%d seconds=%.1f" %
                             (mode, len(frames), now - startedAt))
        if now >= nextQueryAt:
            stateReplies.append(parse_fields(waitSerialCommand(
                monitor, "STATE?", min(5.0, max(1.0, deadline - now)))))
            nextQueryAt = time.monotonic() + queryInterval
        waitBudget = min(frameTimeout, max(0.1, deadline - time.monotonic()))
        record, cursor = monitor.waitFrame(lambda _frame: True, cursor, waitBudget,
                                           "%s long-run frame" % mode)
        validateModeFrame(record.frame, mode, initialEvidence["requestId"],
                          initialEvidence["generation"],
                          initialEvidence["appliedSequence"])
        if lastSequence is not None:
            # The USB text sink may publish every acquisition frame or omit
            # complete publications under backpressure. A positive sequence
            # delta is therefore valid; duplicate/reversed sequencing remains
            # a hard failure and skipped sequence numbers are counted.
            try:
                missing = serialMissingEmissions(lastSequence,
                                                  record.frame.sequence)
            except HilFailure as error:
                raise HilFailure("serial %s %s" % (mode, error)) from error
            if missing:
                sequenceGaps += 1
                missingEmissions += missing
        lastSequence = record.frame.sequence
        frames.append(record.frame)
        monitor.detector.assertHealthy()
    elapsed = time.monotonic() - startedAt
    sanity = checkKnownResistances(frames, knownRanges) if mode == "RES" else {}
    return {
        "mode": mode,
        "frames": len(frames),
        "seconds": elapsed,
        "fps": len(frames) / max(elapsed, 0.001),
        "knownResistanceOhms": sanity,
        "stateQueries": stateReplies,
        "sequenceGaps": sequenceGaps,
        "missingEmissions": missingEmissions,
    }


def firstNonAscii(payload: bytes) -> Optional[Tuple[int, int]]:
    for offset, value in enumerate(payload):
        if value > 0x7F:
            return offset, value
    return None


class BleObserver:
    """BLE callback sink with strict envelope and inner-frame validation."""

    def __init__(self, output: HilOutput, detector: FaultDetector) -> None:
        self.output = output
        self.detector = detector
        self.resetWindow()

    def resetWindow(self) -> None:
        self.reassembler = FragmentReassembler()
        self.protocol = TextProtocolParser()
        self.buffers: Dict[str, bytearray] = {
            "C": bytearray(), "D": bytearray(), "L": bytearray(),
        }
        self.lines: Dict[str, List[LineRecord]] = {"C": [], "D": [], "L": []}
        self.frames: List[FrameRecord] = []
        self.rawNotifications: Dict[str, int] = {"C": 0, "D": 0, "L": 0}
        self.completeMessages: Dict[str, int] = {"C": 0, "D": 0, "L": 0}
        self.bytesReceived: Dict[str, int] = {"C": 0, "D": 0, "L": 0}
        self.callbackErrors: List[str] = []
        self.crossChannelErrors: List[str] = []
        self.staleFragments = 0
        self.startedAt = time.monotonic()

    def callback(self, fallbackChannel: str) -> Callable[[Any, bytearray], None]:
        def receive(_sender: Any, payload: bytearray) -> None:
            try:
                self.feedNotification(fallbackChannel, bytes(payload))
            except Exception as error:  # callback exceptions must reach the test loop
                detail = "%s: %s" % (type(error).__name__, error)
                self.callbackErrors.append(detail)
                self.output.emit("BLE_CALLBACK_ERROR,ch=%s,detail=%s" %
                                 (fallbackChannel, _safeField(detail)))
        return receive

    def feedNotification(self, fallbackChannel: str, payload: bytes) -> None:
        self.rawNotifications[fallbackChannel] += 1
        self.bytesReceived[fallbackChannel] += len(payload)
        nonAscii = firstNonAscii(payload)
        # Fragment bodies are ASCII protocol bytes too.  Record the precise
        # position before allowing the shared reassembler to reject the item.
        if nonAscii is not None:
            self.protocol.note_non_ascii()
            offset, value = nonAscii
            self.callbackErrors.append("non-ASCII byte %02X at %d on %s" %
                                       (value, offset, fallbackChannel))
        for actualChannel, message in self.reassembler.feed(fallbackChannel, payload):
            if actualChannel != fallbackChannel:
                self.crossChannelErrors.append("%s characteristic carried %s envelope" %
                                               (fallbackChannel, actualChannel))
            self._processMessage(actualChannel, message)

    def _processMessage(self, channel: str, message: bytes) -> None:
        if channel not in self.buffers:
            self.callbackErrors.append("unknown logical channel %s" % channel)
            return
        self.completeMessages[channel] += 1
        self.buffers[channel].extend(message)
        while b"\n" in self.buffers[channel]:
            rawLine, _, remainder = self.buffers[channel].partition(b"\n")
            self.buffers[channel][:] = remainder
            rawLine = rawLine.rstrip(b"\r")
            try:
                line = rawLine.decode("ascii")
            except UnicodeDecodeError as error:
                self.protocol.note_non_ascii()
                self.callbackErrors.append("non-ASCII line on %s at %d" %
                                           (channel, error.start))
                continue
            observedAt = time.monotonic()
            self.output.wire("BLE_%s" % channel, line)
            event = self.detector.observe(line, "ble-%s" % channel)
            if event and self.detector.events():
                self.output.emit("HIL_FAULT,source=ble-%s,kind=%s,line=%s" %
                                 (channel, event.kind, _safeField(event.line)))
            self.lines[channel].append(LineRecord(observedAt, channel, line))
            if channel == "D":
                frame = self.protocol.feed_line(line)
                if frame is not None:
                    self.frames.append(FrameRecord(observedAt, channel, frame))

    def lineCount(self, channel: str) -> int:
        return len(self.lines[channel])

    def frameCount(self) -> int:
        return len(self.frames)

    async def waitLine(self, channel: str, predicate: Callable[[str], bool],
                       startIndex: int, timeout: float,
                       description: str) -> Tuple[LineRecord, int]:
        deadline = time.monotonic() + timeout
        cursor = startIndex
        while time.monotonic() < deadline:
            while cursor < len(self.lines[channel]):
                record = self.lines[channel][cursor]
                cursor += 1
                if predicate(record.line):
                    return record, cursor
            self.detector.assertHealthy()
            self.raiseCallbackError()
            await asyncio.sleep(0.02)
        raise HilFailure("timeout waiting for %s" % description)

    async def waitFrame(self, predicate: Callable[[Any], bool], startIndex: int,
                        timeout: float, description: str) -> Tuple[FrameRecord, int]:
        deadline = time.monotonic() + timeout
        cursor = startIndex
        while time.monotonic() < deadline:
            while cursor < len(self.frames):
                record = self.frames[cursor]
                cursor += 1
                if predicate(record.frame):
                    return record, cursor
            self.detector.assertHealthy()
            self.raiseCallbackError()
            await asyncio.sleep(0.02)
        raise HilFailure("timeout waiting for %s" % description)

    def expireStaleFragments(self, maximumAge: float) -> None:
        messages = getattr(self.reassembler, "_messages", {})
        now = time.monotonic()
        staleKeys = []
        for key, item in list(messages.items()):
            created = float(item.get("created", now))
            if now - created > maximumAge:
                staleKeys.append(key)
        for key in staleKeys:
            messages.pop(key, None)
            self.staleFragments += 1

    def pendingFragments(self) -> int:
        return len(getattr(self.reassembler, "_messages", {}))

    def raiseCallbackError(self) -> None:
        if self.callbackErrors:
            raise HilFailure("BLE callback error: %s" % self.callbackErrors[0])
        if self.crossChannelErrors:
            raise HilFailure(self.crossChannelErrors[0])

    def wireStats(self) -> Dict[str, Any]:
        fragmentStats = {}
        for channel, stats in self.reassembler.stats.items():
            fragmentStats[channel] = dataclasses.asdict(stats)
        counters = dataclasses.asdict(self.protocol.counters)
        return {
            "notifications": dict(self.rawNotifications),
            "messages": dict(self.completeMessages),
            "bytes": dict(self.bytesReceived),
            "frames": len(self.frames),
            "fragment": fragmentStats,
            "protocol": counters,
            "pendingBoundaryFragments": self.pendingFragments(),
            "staleFragments": self.staleFragments,
            "callbackErrors": list(self.callbackErrors),
            "crossChannelErrors": list(self.crossChannelErrors),
        }


def validateBleWindow(observer: BleObserver, strictSequence: bool = True,
                      maximumStaleAssemblies: int = 0) -> Dict[str, Any]:
    if maximumStaleAssemblies < 0:
        raise HilFailure("maximum stale assemblies must be non-negative")
    observer.raiseCallbackError()
    observer.expireStaleFragments(5.0)
    failures: List[str] = []
    for channel, stats in observer.reassembler.stats.items():
        for fieldName in ("missing", "gap", "crc_fail", "dropped", "tiny",
                          "duplicate", "out_of_order"):
            value = int(getattr(stats, fieldName))
            if value:
                failures.append("fragment %s.%s=%d" % (channel, fieldName, value))
    counters = observer.protocol.counters
    if counters.crc_errors:
        failures.append("internal frame crc=%d" % counters.crc_errors)
    if counters.malformed:
        failures.append("malformed frame lines=%d" % counters.malformed)
    if counters.non_ascii_chunks:
        failures.append("non-ASCII chunks=%d" % counters.non_ascii_chunks)
    if counters.sequence_regressions:
        failures.append("DATA sequence regressions=%d" %
                        counters.sequence_regressions)
    if strictSequence and counters.sequence_gaps:
        failures.append("DATA sequence gaps=%d missing=%d" %
                        (counters.sequence_gaps, counters.missing_frames))
    if observer.staleFragments > maximumStaleAssemblies:
        failures.append("stale fragment assemblies=%d limit=%d" %
                        (observer.staleFragments, maximumStaleAssemblies))
    if failures:
        raise HilFailure("; ".join(failures))
    return observer.wireStats()


def fastHighLoadIncompleteAssemblyBudget(completedFrames: int) -> int:
    """Return a strict 0.2% budget, rounded down to whole assemblies.

    Rounding down guarantees the accepted count never exceeds 0.2% of the
    completed DATA-frame population.  Keeping this policy outside
    ``validateBleWindow`` also prevents the relaxation from leaking into
    subscription-matrix, reconnect, or SAFE-mode validation windows.  It is
    used only by the FAST full-load mode-stress and RES long-run phases, where
    DATA/LOG delivery is intentionally non-blocking.
    """
    if completedFrames < 0:
        raise HilFailure("completed frame count must be non-negative")
    return completedFrames // 500


def requiredGattProperties() -> Dict[str, Set[str]]:
    return {
        CTRL_RX_UUID: {"write", "write_without_response"},
        CTRL_TX_UUID: {"read", "notify", "indicate"},
        DATA_TX_UUID: {"read", "notify", "indicate"},
        LOG_TX_UUID: {"read", "notify", "indicate"},
    }


def validateGatt(client: Any) -> Dict[str, Any]:
    serviceUuids = {normalizeUuid(service.uuid) for service in client.services}
    if SERVICE_UUID not in serviceUuids:
        raise HilFailure("SensorArray service %s missing" % SERVICE_UUID)
    found: Dict[str, Set[str]] = {}
    for service in client.services:
        for characteristic in service.characteristics:
            characteristicUuid = normalizeUuid(characteristic.uuid)
            found[characteristicUuid] = {
                str(item).lower().replace("write-without-response", "write_without_response")
                for item in characteristic.properties
            }
    for characteristicUuid, required in requiredGattProperties().items():
        if characteristicUuid not in found:
            raise HilFailure("required characteristic %s missing" % characteristicUuid)
        properties = found[characteristicUuid]
        missing = required - properties
        if missing:
            raise HilFailure("%s missing properties %s" %
                             (characteristicUuid, sorted(missing)))
    return {key: sorted(value) for key, value in found.items()
            if key in requiredGattProperties()}


async def startSubscriptions(client: Any, observer: BleObserver,
                             channels: Set[str],
                             forceIndicate: bool = False) -> None:
    started: Set[str] = set()
    try:
        for channel in ("C", "D", "L"):
            if channel in channels:
                options = {"force_indicate": True} if forceIndicate else {}
                await client.start_notify(CHANNEL_UUIDS[channel],
                                          observer.callback(channel), **options)
                started.add(channel)
                # Bleak/BlueZ exposes StartNotify but has no API to choose
                # Indicate when a characteristic supports both Notify and
                # Indicate.  The firmware therefore prefers Indicate and
                # uses a bounded Notify fallback for SAFE on Linux.
    except Exception:
        await stopSubscriptions(client, started)
        raise


async def stopSubscriptions(client: Any, channels: Set[str]) -> None:
    for channel in ("L", "D", "C"):
        if channel in channels:
            try:
                await client.stop_notify(CHANNEL_UUIDS[channel])
            except Exception:
                # A normal disconnect removes CCCDs before host cleanup.
                if getattr(client, "is_connected", False):
                    raise


async def resetBleWindowAfterUnsubscribe(observer: BleObserver) -> None:
    """Start a new strict window after queued host callbacks have drained.

    On Windows, ``stop_notify`` completes after the CCCD write, but callbacks
    already queued on the asyncio loop may still arrive briefly.  Resetting the
    reassembler immediately would turn the tail of the old message into a false
    incomplete assembly in the next phase.  Call this helper only after every
    characteristic active in the prior phase has been unsubscribed.
    """

    await asyncio.sleep(BLE_UNSUBSCRIBE_SETTLE_SECONDS)
    observer.resetWindow()


async def assertBleChannelQuietAfterUnsubscribe(
        client: Any, observer: BleObserver, channel: str,
        detector: FaultDetector,
        settleSeconds: float = BLE_UNSUBSCRIBE_SETTLE_SECONDS,
        quietSeconds: float = BLE_UNSUBSCRIBE_QUIET_SECONDS) -> Dict[str, int]:
    """Allow already-dispatched callbacks to drain, then require channel silence.

    Windows BLE callbacks can already be queued when ``stop_notify`` returns.
    Notifications arriving during the short settling interval are recorded but
    tolerated. Any notification after that boundary indicates a CCCD/lifecycle
    problem rather than a harmless in-flight callback.
    """

    if channel not in observer.rawNotifications:
        raise HilFailure("unknown BLE channel for quiet check: %s" % channel)
    if settleSeconds < 0.0 or quietSeconds < 0.0:
        raise HilFailure("BLE quiet-check durations must be non-negative")

    countAtStop = observer.rawNotifications[channel]
    settleDeadline = time.monotonic() + settleSeconds
    while time.monotonic() < settleDeadline:
        if not getattr(client, "is_connected", False):
            raise HilFailure("BLE disconnected during unsubscribe settle")
        detector.assertHealthy()
        observer.raiseCallbackError()
        await asyncio.sleep(min(0.02, max(0.0, settleDeadline - time.monotonic())))

    countAfterSettle = observer.rawNotifications[channel]
    quietDeadline = time.monotonic() + quietSeconds
    while time.monotonic() < quietDeadline:
        if not getattr(client, "is_connected", False):
            raise HilFailure("BLE disconnected during unsubscribe quiet check")
        detector.assertHealthy()
        observer.raiseCallbackError()
        await asyncio.sleep(min(0.02, max(0.0, quietDeadline - time.monotonic())))

    countAfterQuiet = observer.rawNotifications[channel]
    trailingNotifications = countAfterQuiet - countAfterSettle
    if trailingNotifications:
        raise HilFailure(
            "received %d %s notifications after unsubscribe settle" %
            (trailingNotifications, channel))
    return {
        "graceNotifications": countAfterSettle - countAtStop,
        "trailingNotifications": trailingNotifications,
    }


async def waitBleCommand(client: Any, observer: BleObserver, command: str,
                         timeout: float,
                         writeWithResponse: bool = False) -> str:
    deadline = time.monotonic() + timeout
    expectedPrefix = expectedCommandPrefix(command)
    commandStem = re.split(r"[=?]", command.strip().upper(), maxsplit=1)[0]
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise HilFailure(
                "timeout waiting for %s BLE response after snapshot retries" % command)
        startIndex = observer.lineCount("C")
        wire = command.rstrip("\r\n") + "\n"
        observer.output.wire("HOST_BLE", wire.rstrip("\n"))
        # FF10 supports both Write and Write Without Response.  FF11 is the
        # authoritative application-level acknowledgement, so high-load HIL
        # uses WWR and correlates that reply.  On Windows, a Write-With-Response
        # future can report GattCommunicationStatus.UNREACHABLE after the
        # command was already executed and its FF11 reply was delivered.
        await client.write_gatt_char(
            CTRL_RX_UUID, wire.encode("ascii"), response=writeWithResponse)

        def matches(line: str) -> bool:
            if line.startswith(expectedPrefix) or isTransientSnapshotBusy(command, line):
                return True
            if line.startswith("ERR,"):
                errorCommand = parse_fields(line).get("cmd", "").upper()
                if errorCommand == commandStem or (
                        commandStem == "STATE" and errorCommand == "MODE"):
                    raise HilFailure("BLE command rejected: %s" % line)
            return False

        record, _ = await observer.waitLine("C", matches, startIndex, remaining,
                                            "%s BLE response" % command)
        if isTransientSnapshotBusy(command, record.line):
            observer.detector.assertHealthy()
            await asyncio.sleep(0.01)
            continue
        return record.line


async def setBleRailCalibration(client: Any, observer: BleObserver,
                                avddUv: int, avssUv: int,
                                timeout: float,
                                sidecar: Optional[SerialMonitor] = None) -> Dict[str, int]:
    """Apply RAILCFG over FF10 and correlate its frame-boundary serial event."""
    controlStart = observer.lineCount("C")
    logStart = observer.lineCount("L")
    serialStart = sidecar.lineCount() if sidecar else 0
    wire = "RAILCFG=%d,%d\n" % (avddUv, avssUv)
    observer.output.wire("HOST_BLE", wire.rstrip("\n"))
    await client.write_gatt_char(CTRL_RX_UUID, wire.encode("ascii"), response=False)
    ackRecord, _ = await observer.waitLine(
        "C", lambda line: line.startswith("RACK,"), controlStart, timeout,
        "RAILCFG BLE RACK")
    ackFields = parse_fields(ackRecord.line)
    requestId = int(ackFields["id"], 10)
    if (int(ackFields.get("avdd", "0"), 10) != avddUv or
            int(ackFields.get("avss", "0"), 10) != avssUv):
        raise HilFailure("BLE RAILCFG acknowledgement mismatch: %s" % ackRecord.line)
    appliedPredicate = lambda line: (
        line.startswith("RAPP,") and
        parse_fields(line).get("id") == str(requestId) and
        parse_fields(line).get("source") == "external" and
        parse_fields(line).get("state") == "applied")
    if sidecar:
        appliedRecord, _ = await asyncio.get_running_loop().run_in_executor(
            None,
            lambda: sidecar.waitLine(appliedPredicate, serialStart, timeout,
                                     "RAILCFG BLE serial RAPP"))
    else:
        appliedRecord, _ = await observer.waitLine(
            "L", appliedPredicate, logStart, timeout, "RAILCFG BLE RAPP")
    appliedFields = parse_fields(appliedRecord.line)
    if (int(appliedFields.get("avdd", "0"), 10) != avddUv or
            int(appliedFields.get("avss", "0"), 10) != avssUv):
        raise HilFailure("BLE RAILCFG application mismatch: %s" % appliedRecord.line)
    return {
        "requestId": requestId,
        "generation": int(appliedFields["gen"], 10),
        "appliedSequence": int(appliedFields["seq"], 10),
        "avddUv": avddUv,
        "avssUv": avssUv,
    }


async def switchBleMode(client: Any, observer: BleObserver, mode: str,
                        frameTarget: int, timeout: float,
                        logSubscribed: bool,
                        sidecar: Optional[SerialMonitor] = None
                        ) -> Tuple[List[Any], Dict[str, int]]:
    controlStart = observer.lineCount("C")
    logStart = observer.lineCount("L")
    serialStart = sidecar.lineCount() if sidecar else 0
    frameStart = observer.frameCount()
    wire = "MODE=%s\n" % mode
    observer.output.wire("HOST_BLE", wire.rstrip("\n"))
    await client.write_gatt_char(CTRL_RX_UUID, wire.encode("ascii"), response=False)
    ackRecord, _ = await observer.waitLine(
        "C",
        lambda line: line.startswith("MACK,") and parse_fields(line).get("new") == mode,
        controlStart, timeout, "MODE=%s BLE MACK" % mode)
    requestId = int(parse_fields(ackRecord.line)["id"], 10)
    generation: Optional[int] = None
    appliedSequence: Optional[int] = None
    appliedAt = ackRecord.observedAt
    if logSubscribed or sidecar:
        appliedPredicate = lambda line: (
            line.startswith("MAPP,") and
            parse_fields(line).get("new") == mode and
            parse_fields(line).get("id") == str(requestId))
        if sidecar:
            appliedRecord, _ = await asyncio.get_running_loop().run_in_executor(
                None,
                lambda: sidecar.waitLine(appliedPredicate, serialStart, timeout,
                                         "MODE=%s BLE serial MAPP" % mode))
        else:
            appliedRecord, _ = await observer.waitLine(
                "L", appliedPredicate, logStart, timeout,
                "MODE=%s BLE MAPP" % mode)
        appliedFields = parse_fields(appliedRecord.line)
        generation = int(appliedFields["gen"], 10)
        appliedSequence = int(appliedFields["seq"], 10)
        appliedAt = appliedRecord.observedAt

    acceptedFrames: List[Any] = []
    cursor = frameStart
    deadline = time.monotonic() + timeout
    while len(acceptedFrames) < frameTarget:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise HilFailure("MODE=%s BLE produced %d/%d frames" %
                             (mode, len(acceptedFrames), frameTarget))
        record, cursor = await observer.waitFrame(lambda _frame: True, cursor,
                                                  remaining,
                                                  "MODE=%s BLE frame" % mode)
        if record.observedAt < appliedAt:
            continue
        if generation is None:
            generation = int(record.frame.generation)
        if appliedSequence is None:
            appliedSequence = int(record.frame.sequence)
        if isPreApplyFrame(record.frame, appliedSequence):
            continue
        validateModeFrame(record.frame, mode, requestId, generation, appliedSequence)
        acceptedFrames.append(record.frame)
    evidence = {
        "requestId": requestId,
        "generation": int(generation),
        "appliedSequence": int(appliedSequence),
    }
    return acceptedFrames, evidence


async def waitForBleTraffic(client: Any, observer: BleObserver,
                            dataFrames: int, logMessages: int,
                            minimumSeconds: float, timeout: float,
                            detector: FaultDetector,
                            requiredLogTags: Optional[Set[str]] = None) -> None:
    startedAt = time.monotonic()
    deadline = startedAt + timeout
    requiredTags = set(requiredLogTags or ())
    while True:
        elapsed = time.monotonic() - startedAt
        observedTags = {
            record.line.split(",", 1)[0]
            for record in observer.lines["L"]
        }
        missingTags = requiredTags - observedTags
        if (len(observer.frames) >= dataFrames and
                observer.completeMessages["L"] >= logMessages and
                elapsed >= minimumSeconds and not missingTags):
            return
        if not getattr(client, "is_connected", False):
            raise HilFailure("BLE disconnected while waiting for traffic")
        detector.assertHealthy()
        observer.raiseCallbackError()
        observer.expireStaleFragments(5.0)
        if time.monotonic() >= deadline:
            raise HilFailure(
                "BLE traffic timeout frames=%d/%d logs=%d/%d seconds=%.1f/%.1f missingTags=%s" %
                (len(observer.frames), dataFrames, observer.completeMessages["L"],
                 logMessages, elapsed, minimumSeconds,
                 "|".join(sorted(missingTags)) or "none"))
        await asyncio.sleep(0.05)


def requiredLogTagsForSubscription(profileName: str,
                                   channels: Set[str]) -> Set[str]:
    """Require the documented periodic diagnostics in the full FF30-only case."""

    if profileName == "full" and channels == {"L"}:
        return set(FULL_LOG_CASE_REQUIRED_TAGS)
    return set()


async def runSubscriptionCase(client: Any, observer: BleObserver,
                              sidecar: SerialMonitor, channels: Set[str],
                              queryCommands: Sequence[str], dataFrames: int,
                              logMessages: int, minimumSeconds: float,
                              timeout: float, detector: FaultDetector,
                              minimumLogStackBytes: int,
                              minimumOtherStackBytes: int,
                              heapLeakToleranceBytes: int,
                              requireFirmwareTelemetry: bool,
                              allowedMessageDrops: int = 3,
                              requiredLogTags: Optional[Set[str]] = None) -> Dict[str, Any]:
    await resetBleWindowAfterUnsubscribe(observer)
    sidecarFrameStart = sidecar.frameCount()
    sidecarLineStart = sidecar.lineCount()
    await startSubscriptions(client, observer, channels)
    startedAt = time.monotonic()
    try:
        replies = []
        if "C" in channels:
            for command in queryCommands:
                replies.append(await waitBleCommand(client, observer, command,
                                                    min(10.0, timeout)))
        requiredData = dataFrames if "D" in channels else 0
        requiredLogs = logMessages if "L" in channels else 0
        await waitForBleTraffic(client, observer, requiredData, requiredLogs,
                                minimumSeconds, timeout, detector,
                                requiredLogTags=requiredLogTags)
        if not channels and sidecar.frameCount() <= sidecarFrameStart:
            raise HilFailure("serial measurement stopped while BLE connected unsubscribed")
        for channel in {"C", "D", "L"} - channels:
            if observer.rawNotifications[channel]:
                raise HilFailure("received %s notifications while unsubscribed" % channel)
        stats = validateBleWindow(observer)
        telemetryRecords = telemetryWindowRecords(sidecar, sidecarLineStart)
        stackTelemetry = validateStackTelemetry(
            telemetryRecords, minimumLogStackBytes, minimumOtherStackBytes,
            heapLeakToleranceBytes, required=requireFirmwareTelemetry)
        bleCounters = validateBleFirmwareCounters(
            telemetryRecords, required=requireFirmwareTelemetry,
            allowedMessageDrops=allowedMessageDrops)
        detector.assertHealthy()
        return {
            "subscriptions": formatSubscriptionCase(channels),
            "seconds": time.monotonic() - startedAt,
            "controlReplies": replies,
            "observedLogTags": sorted({
                record.line.split(",", 1)[0]
                for record in observer.lines["L"]
            }),
            "wire": stats,
            "firmwareStack": stackTelemetry,
            "firmwareBleCounters": bleCounters,
        }
    finally:
        await stopSubscriptions(client, channels)


def setSerialRows(monitor: SerialMonitor, rows: int, timeout: float) -> Dict[str, str]:
    startIndex = monitor.lineCount()
    monitor.send("ROWS=%d" % rows)
    accepted, _ = monitor.waitLine(
        lambda line: (line.startswith("RCMD,") and
                      parse_fields(line).get("req") == str(rows)),
        startIndex, timeout, "ROWS=%d accepted" % rows)
    requestId = parse_fields(accepted.line).get("id")
    applied, _ = monitor.waitLine(
        lambda line: (line.startswith("RAPP,") and
                      parse_fields(line).get("id") == requestId and
                      parse_fields(line).get("new") == str(rows)),
        startIndex, timeout, "ROWS=%d applied" % rows)
    return parse_fields(applied.line)


async def setBleRows(client: Any, observer: BleObserver, rows: int,
                     timeout: float,
                     sidecar: Optional[SerialMonitor] = None) -> Dict[str, str]:
    controlStart = observer.lineCount("C")
    logStart = observer.lineCount("L")
    serialStart = sidecar.lineCount() if sidecar else 0
    wire = "ROWS=%d\n" % rows
    observer.output.wire("HOST_BLE", wire.rstrip("\n"))
    await client.write_gatt_char(CTRL_RX_UUID, wire.encode("ascii"), response=False)
    accepted, _ = await observer.waitLine(
        "C",
        lambda line: (line.startswith("RCMD,") and
                      parse_fields(line).get("req") == str(rows)),
        controlStart, timeout, "ROWS=%d BLE accepted" % rows)
    requestId = parse_fields(accepted.line).get("id")
    appliedPredicate = lambda line: (
        line.startswith("RAPP,") and
        parse_fields(line).get("id") == requestId and
        parse_fields(line).get("new") == str(rows))
    if sidecar:
        applied, _ = await asyncio.get_running_loop().run_in_executor(
            None,
            lambda: sidecar.waitLine(appliedPredicate, serialStart, timeout,
                                     "ROWS=%d BLE serial applied" % rows))
    else:
        applied, _ = await observer.waitLine(
            "L", appliedPredicate, logStart, timeout,
            "ROWS=%d BLE applied" % rows)
    return parse_fields(applied.line)


def writeSummary(outputDirectory: Optional[Path], name: str,
                 summary: Dict[str, Any]) -> None:
    if outputDirectory is None:
        return
    outputDirectory.mkdir(parents=True, exist_ok=True)
    path = outputDirectory / (name + "_summary.json")
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def resolveOutput(args: argparse.Namespace, name: str) -> Tuple[HilOutput, Optional[Path]]:
    outputDirectory = Path(args.outputDirectory) if args.outputDirectory else None
    logPath = outputDirectory / (name + ".log") if outputDirectory else None
    return HilOutput(logPath), outputDirectory


def applySerialProfile(args: argparse.Namespace) -> None:
    full = args.profile == "full"
    if args.longRunFrames is None:
        args.longRunFrames = 2000 if full else 20
    if args.longRunSeconds is None:
        args.longRunSeconds = 120.0 if full else 0.0
    if args.longRunMaximumSeconds is None:
        args.longRunMaximumSeconds = 2400.0 if full else 180.0


def runSerialHil(args: argparse.Namespace) -> int:
    applySerialProfile(args)
    output, outputDirectory = resolveOutput(args, "serial_hil")
    summary: Dict[str, Any] = {
        "kind": "serial",
        "profile": args.profile,
        "passed": False,
        "startedAt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    monitor: Optional[SerialMonitor] = None
    try:
        candidates = enumerateSerialPorts()
        printSerialCandidates(output, candidates)
        port = selectSerialPort(args.port, os.environ.get("SENSORARRAY_SERIAL_PORT"),
                                candidates)
        detector = FaultDetector()
        monitor = SerialMonitor.open(port, args.baud, output, detector)
        monitor.start()
        # Recover USB observability before waiting for a frame.  A previous
        # interrupted test may have left ST=BLE/WIFI even though the serial
        # control task itself is still available.
        initialStream = parse_fields(waitSerialCommandReady(
            monitor, "ST?", args.startupTimeout)).get("v", "auto")
        initialTx = parse_fields(waitSerialCommand(
            monitor, "TX?", args.commandTimeout)).get("v", "rel")
        waitSerialCommand(monitor, "ST=SER", args.commandTimeout)
        waitSerialCommand(monitor, "TX=FULL", args.commandTimeout)
        initialRecord, _ = monitor.waitFrame(
            isRuntimeReadyFrame, 0,
            args.startupTimeout, "initial CRC-valid frame")
        output.emit("SERIAL_READY,mode=%s,seq=%d,rows=%d,cells=%d" %
                    (frameMode(initialRecord.frame), initialRecord.frame.sequence,
                     initialRecord.frame.rows, initialRecord.frame.cells))
        ignoredFaults = [dataclasses.asdict(item) for item in detector.ignoredEvents()]
        detector.arm(clearExisting=True)
        baselineCrcErrors = monitor.protocol.counters.crc_errors
        baselineMalformed = monitor.protocol.counters.malformed
        baselineNonAsciiLines = monitor.nonAsciiLines
        setSerialRows(monitor, 8, args.modeTimeout)

        railCalibration = None
        if args.railAvddUv is not None:
            railCalibration = setSerialRailCalibration(
                monitor, args.railAvddUv, args.railAvssUv, args.modeTimeout)

        commandReplies = []
        for command in args.query:
            commandReplies.append(waitSerialCommand(monitor, command,
                                                    args.commandTimeout))

        modeResults = []
        allResistanceFrames: List[Any] = []
        for _cycle in range(args.modeCycles):
            for mode in args.modes:
                stageStartedAt = time.monotonic()
                frames, evidence = switchSerialMode(
                    monitor, mode, args.framesPerMode, args.modeTimeout)
                stageSeconds = time.monotonic() - stageStartedAt
                stateFields = parse_fields(waitSerialCommand(
                    monitor, "STATE?", args.commandTimeout))
                if stateFields.get("active") != mode:
                    raise HilFailure("STATE? active=%s after MODE=%s" %
                                     (stateFields.get("active"), mode))
                if (int(stateFields.get("gen", "-1"), 10) != evidence["generation"] or
                        int(stateFields.get("rid", "-1"), 10) != evidence["requestId"]):
                    raise HilFailure(
                        "STATE? mode correlation gen/rid=%s/%s expected=%d/%d" %
                        (stateFields.get("gen"), stateFields.get("rid"),
                         evidence["generation"], evidence["requestId"]))
                if mode == "RES":
                    allResistanceFrames.extend(frames)
                modeResults.append({
                    "mode": mode,
                    "frames": len(frames),
                    "firstSequence": frames[0].sequence,
                    "lastSequence": frames[-1].sequence,
                    "seconds": stageSeconds,
                    "fps": len(frames) / max(stageSeconds, 0.001),
                    "state": stateFields,
                    **evidence,
                })
                detector.assertHealthy()
        resistanceEvidence = checkKnownResistances(
            allResistanceFrames, args.knownResistor) if allResistanceFrames else {}

        longRun = None
        if args.longRunFrames > 0 or args.longRunSeconds > 0.0:
            _, evidence = switchSerialMode(monitor, args.longRunMode, 1,
                                           args.modeTimeout)
            longRunLineStart = monitor.lineCount()
            longRun = collectSerialLongRun(
                monitor, args.longRunMode, evidence, args.longRunFrames,
                args.longRunSeconds, args.longRunMaximumSeconds,
                args.queryInterval, args.frameTimeout,
                args.knownResistor if args.longRunMode == "RES" else [])
            longRun["telemetry"] = validateStackTelemetry(
                monitor.lines[longRunLineStart:], args.minimumLogStackBytes,
                args.minimumOtherStackBytes, args.heapLeakToleranceBytes,
                required=args.profile == "full")
            output.emit("SERIAL_LONG_RUN,mode=%s,frames=%d,seconds=%.3f,fps=%.3f" %
                        (args.longRunMode, longRun["frames"], longRun["seconds"],
                         longRun["fps"]))

        # Leave the board in the documented default measurement mode.  Runtime
        # stream/TX settings are restored when their query values map to a
        # public command.
        if not modeResults or modeResults[-1]["mode"] != "CAP" or longRun:
            switchSerialMode(monitor, "CAP", 1, args.modeTimeout)
        txRestore = {"rel": "REL", "short": "SHORT", "full": "FULL"}.get(
            initialTx.lower())
        if txRestore:
            waitSerialCommand(monitor, "TX=%s" % txRestore, args.commandTimeout)
        streamRestore = {
            "auto": "AUTO", "ser": "SER", "ble": "BLE",
            "wifi": "WIFI", "all": "ALL",
        }.get(initialStream.lower())
        if streamRestore:
            waitSerialCommand(monitor, "ST=%s" % streamRestore, args.commandTimeout)

        detector.assertHealthy()
        nonAsciiDelta = monitor.nonAsciiLines - baselineNonAsciiLines
        crcDelta = monitor.protocol.counters.crc_errors - baselineCrcErrors
        malformedDelta = monitor.protocol.counters.malformed - baselineMalformed
        if nonAsciiDelta or crcDelta or malformedDelta:
            raise HilFailure(
                "serial wire errors after ready nonAscii=%d crc=%d malformed=%d" %
                (nonAsciiDelta, crcDelta, malformedDelta))
        summary.update({
            "passed": True,
            "port": port,
            "baud": args.baud,
            "ignoredStartupFaults": ignoredFaults,
            "commands": commandReplies,
            "railCalibration": railCalibration,
            "modes": modeResults,
            "knownResistanceOhms": resistanceEvidence,
            "longRun": longRun,
            "protocol": dataclasses.asdict(monitor.protocol.counters),
            "postReadyWireErrors": {
                "nonAscii": nonAsciiDelta,
                "crc": crcDelta,
                "malformed": malformedDelta,
            },
            "unexpectedFaults": [],
        })
        output.emit("SERIAL_HIL_PASS,modes=%d,longFrames=%d,crcErrors=0,resets=0" %
                    (len(modeResults), longRun["frames"] if longRun else 0))
        writeSummary(outputDirectory, "serial_hil", summary)
        return EXIT_PASS
    except HilSkipped as error:
        summary["skipReason"] = str(error)
        output.emit("SERIAL_HIL_NOT_RUN,reason=%s" % _safeField(str(error)))
        writeSummary(outputDirectory, "serial_hil", summary)
        return EXIT_SKIPPED
    except Exception as error:
        summary["failure"] = "%s: %s" % (type(error).__name__, error)
        if monitor:
            summary["unexpectedFaults"] = [dataclasses.asdict(item)
                                             for item in monitor.detector.events()]
        output.emit("SERIAL_HIL_FAIL,reason=%s,detail=%s" %
                    (type(error).__name__, _safeField(str(error))))
        writeSummary(outputDirectory, "serial_hil", summary)
        return EXIT_FAIL
    finally:
        if monitor:
            monitor.close()
        output.close()


@dataclasses.dataclass(frozen=True)
class BleProfile:
    connectedIdleSeconds: float
    caseDataFrames: int
    ff20OnlyFrames: int
    ff20OnlySeconds: float
    caseLogMessages: int
    caseTimeout: float
    txModeFrames: int
    modeCycles: int
    framesPerMode: int
    subscribeCycles: int
    reconnectCycles: int
    longRunFrames: int
    longRunSeconds: float
    longRunMaximumSeconds: float


def buildBleProfile(args: argparse.Namespace) -> BleProfile:
    full = args.profile == "full"
    defaults = {
        "connectedIdleSeconds": 60.0 if full else 3.0,
        "caseDataFrames": 10 if full else 2,
        "ff20OnlyFrames": 1000 if full else 5,
        "ff20OnlySeconds": 0.0,
        "caseLogMessages": 3 if full else 1,
        "caseTimeout": 900.0 if full else 90.0,
        "txModeFrames": 10 if full else 2,
        "modeCycles": 20 if full else 1,
        "framesPerMode": 5 if full else 2,
        "subscribeCycles": 100 if full else 2,
        "reconnectCycles": 30 if full else 2,
        "longRunFrames": 2000 if full else 20,
        "longRunSeconds": 120.0 if full else 0.0,
        "longRunMaximumSeconds": 2400.0 if full else 180.0,
    }
    values = {}
    for fieldName, default in defaults.items():
        requested = getattr(args, fieldName)
        values[fieldName] = default if requested is None else requested
    return BleProfile(**values)


async def discoverBleDevice(args: argparse.Namespace, output: HilOutput,
                            BleakScanner: Any) -> Any:
    try:
        devices = await BleakScanner.discover(timeout=args.scanSeconds)
    except Exception as error:
        raise HilSkipped("BLE scan failed: %s" % error) from error
    requestedAddress = args.address or os.environ.get("SENSORARRAY_BLE_ADDRESS")
    candidates = []
    for device in devices:
        name = getattr(device, "name", None) or ""
        address = getattr(device, "address", "")
        if requestedAddress:
            if address.casefold() == requestedAddress.casefold():
                candidates.append(device)
        elif name.startswith(args.namePrefix):
            candidates.append(device)
    for device in candidates:
        output.emit("BLE_CANDIDATE,name=%s,address=%s" %
                    (_safeField(getattr(device, "name", "") or ""),
                     _safeField(getattr(device, "address", "") or "")))
    if not candidates:
        selector = requestedAddress or ("name-prefix=" + args.namePrefix)
        raise HilSkipped("SensorArray BLE device not found (%s)" % selector)
    if len(candidates) > 1 and not requestedAddress:
        raise HilSkipped("multiple SensorArray BLE devices found; use --address or SENSORARRAY_BLE_ADDRESS")
    return candidates[0]


def createBleClient(BleakClient: Any, device: Any,
                    disconnectedCallback: Optional[Callable[[Any], None]] = None) -> Any:
    options: Dict[str, Any] = {"timeout": 20.0}
    if disconnectedCallback is not None:
        options["disconnected_callback"] = disconnectedCallback
    if sys.platform == "win32":
        options["winrt"] = {"use_cached_services": False}
    return BleakClient(device, **options)


async def runTxModePhase(client: Any, observer: BleObserver,
                         profile: BleProfile, detector: FaultDetector,
                         timeout: float) -> Dict[str, Any]:
    modes = []
    for mode in ("FAST", "SAFE", "FAST"):
        activeChannels: Set[str] = set()
        forceIndicate = mode == "SAFE"
        await resetBleWindowAfterUnsubscribe(observer)
        # Configure FF11's CCCD for the mode being requested before writing
        # BTX=.  The acknowledgement itself uses the new mode, so this avoids
        # relying on a Notify CCCD to carry an Indicate (or vice versa).
        await startSubscriptions(client, observer, {"C"}, forceIndicate)
        activeChannels.add("C")
        try:
            reply = await waitBleCommand(client, observer, "BTX=%s" % mode, timeout)
            query = await waitBleCommand(client, observer, "BTX?", timeout)
            if parse_fields(reply).get("v", "").upper() != mode:
                raise HilFailure("BTX=%s reply mismatch: %s" % (mode, reply))
            if parse_fields(query).get("v", "").upper() != mode:
                raise HilFailure("BTX? expected %s: %s" % (mode, query))
            await startSubscriptions(client, observer, {"D", "L"}, forceIndicate)
            activeChannels.update({"D", "L"})
            await waitForBleTraffic(client, observer, profile.txModeFrames, 1,
                                    0.0, profile.caseTimeout, detector)
            # SAFE indications intentionally preserve the non-blocking
            # acquisition contract.  When producer throughput exceeds the
            # confirmed ATT link, positive sequence gaps are expected and are
            # reported in wire statistics.  Regressions, malformed frames and
            # all inner/outer CRC failures still fail validateBleWindow().
            stats = validateBleWindow(observer, strictSequence=(mode == "FAST"))
            modes.append({
                "mode": mode,
                "cccd": "INDICATE" if forceIndicate else "NOTIFY",
                "reply": reply,
                "query": query,
                "frames": profile.txModeFrames,
                "wire": stats,
            })
        finally:
            await stopSubscriptions(client, activeChannels)
    return {"modes": modes}


async def runModeStressPhase(client: Any, observer: BleObserver,
                             profile: BleProfile, detector: FaultDetector,
                             timeout: float,
                             knownRanges: Sequence[KnownResistance],
                             sidecar: SerialMonitor) -> Dict[str, Any]:
    channels = {"C", "D", "L"}
    await resetBleWindowAfterUnsubscribe(observer)
    await startSubscriptions(client, observer, channels)
    transitions = []
    resistanceFrames: List[Any] = []
    try:
        for cycle in range(profile.modeCycles):
            for mode in ("CAP", "RES", "VOLT", "CAP"):
                stageStartedAt = time.monotonic()
                frames, evidence = await switchBleMode(
                    client, observer, mode, profile.framesPerMode,
                    timeout, logSubscribed=True, sidecar=sidecar)
                stageSeconds = time.monotonic() - stageStartedAt
                if mode == "RES":
                    resistanceFrames.extend(frames)
                transitions.append({"cycle": cycle + 1, "mode": mode,
                                    "frames": len(frames),
                                    "seconds": stageSeconds,
                                    "fps": len(frames) / max(stageSeconds, 0.001),
                                    **evidence})
                detector.assertHealthy()
            await waitBleCommand(client, observer, "STATE?", timeout)
        maximumIncompleteMessages = fastHighLoadIncompleteAssemblyBudget(
            len(observer.frames))
        stats = validateBleWindow(
            observer, strictSequence=False,
            maximumStaleAssemblies=maximumIncompleteMessages)
        sanity = checkKnownResistances(resistanceFrames, knownRanges)
        return {"cycles": profile.modeCycles, "transitions": transitions,
                "knownResistanceOhms": sanity,
                "incompleteAssemblyBudget": maximumIncompleteMessages,
                "wire": stats}
    finally:
        await stopSubscriptions(client, channels)


async def runSubscribeStressPhase(client: Any, observer: BleObserver,
                                  profile: BleProfile,
                                  detector: FaultDetector,
                                  sidecar: SerialMonitor,
                                  minimumLogStackBytes: int,
                                  minimumOtherStackBytes: int,
                                  heapLeakToleranceBytes: int,
                                  requireTelemetry: bool) -> Dict[str, Any]:
    sidecarLineStart = sidecar.lineCount()
    completed = 0
    quietChecks = 0
    graceNotifications = 0
    for cycle in range(profile.subscribeCycles):
        for channel in ("D", "L"):
            channels = {channel}
            await resetBleWindowAfterUnsubscribe(observer)
            await startSubscriptions(client, observer, channels)
            try:
                await waitForBleTraffic(
                    client, observer,
                    dataFrames=1 if channel == "D" else 0,
                    logMessages=1 if channel == "L" else 0,
                    minimumSeconds=0.0,
                    timeout=profile.caseTimeout,
                    detector=detector)
                validateBleWindow(observer)
                detector.assertHealthy()
            finally:
                await stopSubscriptions(client, channels)
            quiet = await assertBleChannelQuietAfterUnsubscribe(
                client, observer, channel, detector)
            quietChecks += 1
            graceNotifications += quiet["graceNotifications"]
            completed += 1
    telemetryRecords = telemetryWindowRecords(sidecar, sidecarLineStart)
    return {
        "cycles": profile.subscribeCycles,
        "subscriptionWindows": completed,
        "unsubscribeQuietChecks": quietChecks,
        "unsubscribeGraceNotifications": graceNotifications,
        "firmwareStack": validateStackTelemetry(
            telemetryRecords, minimumLogStackBytes, minimumOtherStackBytes,
            heapLeakToleranceBytes, required=requireTelemetry),
        "firmwareBleCounters": validateBleFirmwareCounters(
            telemetryRecords, required=requireTelemetry,
            allowedMessageDrops=(profile.subscribeCycles * 2) + 2),
    }


async def runCommandParity(client: Any, observer: BleObserver,
                           sidecar: SerialMonitor, timeout: float) -> Dict[str, str]:
    serialStart = sidecar.lineCount()
    sidecar.send("STATE?")
    serialFuture = asyncio.get_running_loop().run_in_executor(
        None,
        lambda: sidecar.waitLine(lambda line: line.startswith("MODE,"),
                                 serialStart, timeout, "serial STATE? parity"))
    bleLine = await waitBleCommand(client, observer, "STATE?", timeout)
    serialRecord, _ = await serialFuture
    serialFields = parse_fields(serialRecord.line)
    bleFields = parse_fields(bleLine)
    stableKeys = (
        "state", "active", "pending", "pid", "gen", "rid", "route",
        "sw", "source", "matrixRef", "intref", "vbias", "refmux", "pga",
        "fdcPrimary", "fdcPrimaryVerified", "fdcSecondary",
        "fdcSecondaryVerified",
    )
    mismatches = [key for key in stableKeys
                  if serialFields.get(key) != bleFields.get(key)]
    if mismatches:
        raise HilFailure("serial/BLE STATE? mismatch keys=%s" % ",".join(mismatches))
    return {key: serialFields.get(key, "") for key in stableKeys}


async def runBleLongRunPhase(client: Any, observer: BleObserver,
                             sidecar: SerialMonitor, profile: BleProfile,
                             detector: FaultDetector, commandTimeout: float,
                             frameTimeout: float, queryInterval: float,
                             knownRanges: Sequence[KnownResistance],
                             minimumLogStackBytes: int,
                             minimumOtherStackBytes: int,
                             heapLeakToleranceBytes: int,
                             requireTelemetry: bool) -> Dict[str, Any]:
    channels = {"C", "D", "L"}
    await resetBleWindowAfterUnsubscribe(observer)
    await startSubscriptions(client, observer, channels)
    try:
        _, evidence = await switchBleMode(client, observer, "RES", 1,
                                          commandTimeout, logSubscribed=True,
                                          sidecar=sidecar)
        # Build a clean reassembly boundary before starting acceptance counts.
        # Resetting a parser while notifications are still active can retain
        # the tail of a message whose first fragment belonged to the previous
        # window; that is a harness-created orphan, not a firmware fragment
        # failure.  Turning all CCCDs off, allowing queued WinRT callbacks to
        # drain, then subscribing again makes every fragment error in the new
        # window attributable to the tested link.
        await stopSubscriptions(client, channels)
        await asyncio.sleep(BLE_UNSUBSCRIBE_SETTLE_SECONDS)
        observer.resetWindow()
        await startSubscriptions(client, observer, channels)
        startedAt = time.monotonic()
        deadline = startedAt + profile.longRunMaximumSeconds
        nextQueryAt = startedAt
        cursor = 0
        frames: List[Any] = []
        stateReplies: List[Dict[str, str]] = []
        lastSequence: Optional[int] = None
        sequenceGaps = 0
        missingFrames = 0
        while (len(frames) < profile.longRunFrames or
               time.monotonic() - startedAt < profile.longRunSeconds):
            now = time.monotonic()
            if now >= deadline:
                raise HilFailure("BLE RES long-run deadline frames=%d seconds=%.1f" %
                                 (len(frames), now - startedAt))
            if queryInterval > 0.0 and now >= nextQueryAt:
                stateReplies.append(parse_fields(await waitBleCommand(
                    client, observer, "STATE?", commandTimeout)))
                nextQueryAt = time.monotonic() + queryInterval
            record, cursor = await observer.waitFrame(
                lambda _frame: True, cursor,
                min(frameTimeout, max(0.1, deadline - time.monotonic())),
                "BLE RES long-run frame")
            validateModeFrame(record.frame, "RES", evidence["requestId"],
                              evidence["generation"], evidence["appliedSequence"])
            if lastSequence is not None:
                if record.frame.sequence <= lastSequence:
                    raise HilFailure("BLE RES sequence regression %d -> %d" %
                                     (lastSequence, record.frame.sequence))
                if record.frame.sequence > lastSequence + 1:
                    sequenceGaps += 1
                    missingFrames += record.frame.sequence - lastSequence - 1
            lastSequence = record.frame.sequence
            frames.append(record.frame)
            detector.assertHealthy()
            observer.expireStaleFragments(5.0)
        elapsed = time.monotonic() - startedAt
        parity = await runCommandParity(client, observer, sidecar, commandTimeout)
        # FAST uses unconfirmed notifications and DATA/LOG are deliberately
        # non-blocking.  A central may therefore miss an entire fragmented
        # message even when every GATT submission succeeded.  Completed
        # envelopes remain fully strict (index/count/length/outer CRC and DATA
        # inner CRC); only a small, explicit 0.2% incomplete-message budget is
        # accepted and reported.  Firmware-side fragment errors (BL50 fe) must
        # still remain zero.
        maximumIncompleteMessages = fastHighLoadIncompleteAssemblyBudget(len(frames))
        stats = validateBleWindow(
            observer, strictSequence=False,
            maximumStaleAssemblies=maximumIncompleteMessages)
        sanity = checkKnownResistances(frames, knownRanges)
        telemetry = validateStackTelemetry(
            observer.lines["L"], minimumLogStackBytes, minimumOtherStackBytes,
            heapLeakToleranceBytes, required=requireTelemetry)
        firmwareCounters = validateBleFirmwareCounters(
            observer.lines["L"], required=requireTelemetry,
            # DATA/LOG are deliberately non-blocking. Their per-channel
            # deltas are reported, while dropC/ctrlExhaust and fragment
            # failures remain unconditional failures.
            allowedMessageDrops=0x7FFFFFFF)
        return {
            "frames": len(frames),
            "seconds": elapsed,
            "fps": len(frames) / max(elapsed, 0.001),
            "sequenceGaps": sequenceGaps,
            "missingFrames": missingFrames,
            "knownResistanceOhms": sanity,
            "stateQueries": stateReplies,
            "commandParity": parity,
            "incompleteAssemblyBudget": maximumIncompleteMessages,
            "wire": stats,
            "telemetry": telemetry,
            "firmwareBleCounters": firmwareCounters,
        }
    finally:
        await stopSubscriptions(client, channels)


async def runReconnectStress(device: Any, BleakClient: Any, output: HilOutput,
                             detector: FaultDetector, profile: BleProfile,
                             commandTimeout: float, restoreStream: str,
                             restoreTx: str, restoreBtx: str,
                             sidecar: SerialMonitor,
                             minimumLogStackBytes: int,
                             minimumOtherStackBytes: int,
                             heapLeakToleranceBytes: int,
                             requireTelemetry: bool) -> Dict[str, Any]:
    cycles = []
    sidecarLineStart = sidecar.lineCount()
    for cycle in range(profile.reconnectCycles):
        observer = BleObserver(output, detector)
        client = createBleClient(BleakClient, device)
        activeChannels: Set[str] = set()
        try:
            await client.connect()
            gatt = validateGatt(client)
            await startSubscriptions(client, observer, {"C"})
            activeChannels.add("C")
            await waitBleCommand(client, observer, "BTX=FAST", commandTimeout)
            await startSubscriptions(client, observer, {"D", "L"})
            activeChannels.update({"D", "L"})
            await waitBleCommand(client, observer, "ST=AUTO", commandTimeout)
            await waitBleCommand(client, observer, "TX=FULL", commandTimeout)
            reply = await waitBleCommand(client, observer, "STATE?", commandTimeout)
            await waitForBleTraffic(client, observer, 1, 1, 0.0,
                                    profile.caseTimeout, detector)
            stats = validateBleWindow(observer)
            restoreStats = None
            detector.assertHealthy()
            if cycle + 1 == profile.reconnectCycles:
                txValue = {"rel": "REL", "short": "SHORT", "full": "FULL"}.get(
                    restoreTx.lower())
                if txValue:
                    await waitBleCommand(client, observer, "TX=%s" % txValue,
                                         commandTimeout)
                streamValue = {
                    "auto": "AUTO", "ser": "SER", "ble": "BLE",
                    "wifi": "WIFI", "all": "ALL",
                }.get(restoreStream.lower())
                if streamValue:
                    await waitBleCommand(client, observer, "ST=%s" % streamValue,
                                         commandTimeout)
                if restoreBtx.upper() == "SAFE":
                    await stopSubscriptions(client, activeChannels)
                    activeChannels.clear()
                    await resetBleWindowAfterUnsubscribe(observer)
                    await startSubscriptions(client, observer, {"C"},
                                             forceIndicate=True)
                    activeChannels.add("C")
                    await waitBleCommand(client, observer, "BTX=SAFE",
                                         commandTimeout)
                    await waitBleCommand(client, observer, "BTX?", commandTimeout)
                    restoreStats = validateBleWindow(observer)
            cycles.append({"cycle": cycle + 1, "state": reply,
                           "gatt": gatt, "wire": stats,
                           "restoreWire": restoreStats})
        except Exception as error:
            raise HilFailure("reconnect cycle %d/%d failed: %s" %
                             (cycle + 1, profile.reconnectCycles, error)) from error
        finally:
            if getattr(client, "is_connected", False):
                await stopSubscriptions(client, activeChannels)
                await client.disconnect()
        # Give the ESP32 advertising task a bounded opportunity to restart.
        if cycle + 1 < profile.reconnectCycles:
            await asyncio.sleep(0.5)
    telemetryRecords = telemetryWindowRecords(sidecar, sidecarLineStart)
    return {
        "cycles": len(cycles),
        "details": cycles,
        "firmwareStack": validateStackTelemetry(
            telemetryRecords, minimumLogStackBytes, minimumOtherStackBytes,
            heapLeakToleranceBytes, required=requireTelemetry),
        "firmwareBleCounters": validateBleFirmwareCounters(
            telemetryRecords, required=requireTelemetry,
            allowedMessageDrops=(profile.reconnectCycles * 3) + 2),
    }


def parsePhases(value: str) -> Set[str]:
    allowed = {
        "matrix", "tx-modes", "mode-stress", "subscribe-stress",
        "reconnect", "long-run",
    }
    phases = {item.strip().lower() for item in value.split(",") if item.strip()}
    if not phases:
        raise argparse.ArgumentTypeError("at least one BLE phase is required")
    unknown = phases - allowed
    if unknown:
        raise argparse.ArgumentTypeError("unknown BLE phase(s): %s" %
                                         ",".join(sorted(unknown)))
    return phases


async def runBleHil(args: argparse.Namespace) -> int:
    profile = buildBleProfile(args)
    output, outputDirectory = resolveOutput(args, "ble_hil")
    summary: Dict[str, Any] = {
        "kind": "ble",
        "profile": args.profile,
        "passed": False,
        "startedAt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "phases": {},
    }
    sidecar: Optional[SerialMonitor] = None
    disconnectEvents: List[float] = []
    try:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as error:
            raise HilSkipped("missing bleak; install tools/requirements-hil.txt") from error

        candidates = enumerateSerialPorts()
        printSerialCandidates(output, candidates)
        serialPort = selectSerialPort(
            args.serialPort,
            os.environ.get("SENSORARRAY_SERIAL_PORT"),
            candidates,
        )
        detector = FaultDetector()
        sidecar = SerialMonitor.open(serialPort, args.serialBaud, output, detector)
        sidecar.start()
        # Serial control remains available regardless of stream selection.
        # Save user-visible settings, then establish a recoverable baseline
        # before waiting for USB frames or opening Notify CCCDs.
        initialStream = parse_fields(waitSerialCommandReady(
            sidecar, "ST?", args.startupTimeout)).get("v", "auto")
        initialTx = parse_fields(waitSerialCommand(
            sidecar, "TX?", args.commandTimeout)).get("v", "rel")
        initialBtx = parse_fields(waitSerialCommand(
            sidecar, "BTX?", args.commandTimeout)).get("v", "FAST")
        waitSerialCommand(sidecar, "ST=SER", args.commandTimeout)
        waitSerialCommand(sidecar, "TX=FULL", args.commandTimeout)
        waitSerialCommand(sidecar, "BTX=FAST", args.commandTimeout)
        readyRecord, _ = sidecar.waitFrame(
            isRuntimeReadyFrame, 0,
            args.startupTimeout, "serial sidecar CRC-valid frame")
        output.emit("SERIAL_SIDECAR_READY,port=%s,mode=%s,seq=%d" %
                    (serialPort, frameMode(readyRecord.frame),
                     readyRecord.frame.sequence))
        ignoredFaults = [dataclasses.asdict(item) for item in detector.ignoredEvents()]
        detector.arm(clearExisting=True)
        baselineCrcErrors = sidecar.protocol.counters.crc_errors
        baselineMalformed = sidecar.protocol.counters.malformed
        baselineNonAsciiLines = sidecar.nonAsciiLines

        device = await discoverBleDevice(args, output, BleakScanner)
        summary["device"] = {
            "name": getattr(device, "name", None),
            "address": getattr(device, "address", None),
        }
        summary["serialSidecar"] = serialPort
        summary["ignoredStartupFaults"] = ignoredFaults

        observer = BleObserver(output, detector)
        client = createBleClient(
            BleakClient, device,
            disconnectedCallback=lambda _client: disconnectEvents.append(time.monotonic()))
        await client.connect()
        try:
            summary["gatt"] = validateGatt(client)
            output.emit("BLE_CONNECTED,name=%s,address=%s,mtu=%s" %
                        (_safeField(getattr(device, "name", "") or ""),
                         _safeField(getattr(device, "address", "") or ""),
                         getattr(client, "mtu_size", "unknown")))

            # Establish deterministic 8x8/FULL/AUTO conditions through the
            # same FF10 handler used by production clients.  Subscribe all
            # three TX characteristics temporarily so MAPP/RAPP and frames are
            # verifiable before the isolated subscription matrix begins.
            await resetBleWindowAfterUnsubscribe(observer)
            await startSubscriptions(client, observer, {"C", "D", "L"})
            normalisedStream = parse_fields(await waitBleCommand(
                client, observer, "ST?", args.commandTimeout,
                writeWithResponse=True)).get("v", "")
            normalisedTx = parse_fields(await waitBleCommand(
                client, observer, "TX?", args.commandTimeout)).get("v", "")
            normalisedBtx = parse_fields(await waitBleCommand(
                client, observer, "BTX?", args.commandTimeout)).get("v", "")
            if (normalisedStream.lower() != "ser" or
                    normalisedTx.lower() != "full" or
                    normalisedBtx.upper() != "FAST"):
                raise HilFailure(
                    "serial baseline did not apply ST/TX/BTX=%s/%s/%s" %
                    (normalisedStream, normalisedTx, normalisedBtx))
            await waitBleCommand(client, observer, "ST=AUTO", args.commandTimeout)
            await waitBleCommand(client, observer, "TX=FULL", args.commandTimeout)
            await waitBleCommand(client, observer, "BTX=FAST", args.commandTimeout)
            await setBleRows(client, observer, 8, args.modeTimeout,
                             sidecar=sidecar)
            railCalibration = None
            if args.railAvddUv is not None:
                railCalibration = await setBleRailCalibration(
                    client, observer, args.railAvddUv, args.railAvssUv,
                    args.modeTimeout, sidecar=sidecar)
                summary["railCalibration"] = railCalibration
            await switchBleMode(client, observer, "CAP", 2,
                                args.modeTimeout, logSubscribed=True,
                                sidecar=sidecar)
            validateBleWindow(observer, strictSequence=False)
            await stopSubscriptions(client, {"C", "D", "L"})

            if "matrix" in args.phases:
                matrix = []
                for channels in args.subscriptionCases:
                    onlyData = channels == {"D"}
                    logOnlyRes = args.profile == "full" and channels == {"L"}
                    dataFrames = (profile.ff20OnlyFrames if onlyData else
                                  profile.caseDataFrames)
                    minimumSeconds = (
                        profile.connectedIdleSeconds if not channels else
                        profile.ff20OnlySeconds if onlyData else 0.0)
                    logModeEvidence = None
                    if logOnlyRes:
                        _, logModeEvidence = switchSerialMode(
                            sidecar, "RES", 2, args.modeTimeout)
                    try:
                        result = await runSubscriptionCase(
                            client, observer, sidecar, channels, args.query,
                            dataFrames, profile.caseLogMessages,
                            minimumSeconds,
                            max(profile.caseTimeout, minimumSeconds + 30.0), detector,
                            args.minimumLogStackBytes, args.minimumOtherStackBytes,
                            args.heapLeakToleranceBytes,
                            requireFirmwareTelemetry=(args.profile == "full" and
                                                      channels == {"D"}),
                            requiredLogTags=requiredLogTagsForSubscription(
                                args.profile, channels))
                    finally:
                        if logOnlyRes:
                            switchSerialMode(sidecar, "CAP", 2, args.modeTimeout)
                    if logModeEvidence is not None:
                        result["measurementMode"] = "RES"
                        result["modeEvidence"] = logModeEvidence
                    matrix.append(result)
                    output.emit("BLE_MATRIX_PASS,sub=%s,frames=%d,logs=%d,seconds=%.3f" %
                                (result["subscriptions"],
                                 result["wire"]["frames"],
                                 result["wire"]["messages"]["L"],
                                 result["seconds"]))
                summary["phases"]["matrix"] = matrix

            if "tx-modes" in args.phases:
                result = await runTxModePhase(
                    client, observer, profile, detector, args.commandTimeout)
                summary["phases"]["txModes"] = result
                output.emit("BLE_TX_MODES_PASS,sequence=FAST-SAFE-FAST")

            if "mode-stress" in args.phases:
                result = await runModeStressPhase(
                    client, observer, profile, detector, args.modeTimeout,
                    args.knownResistor, sidecar)
                summary["phases"]["modeStress"] = result
                output.emit("BLE_MODE_STRESS_PASS,cycles=%d,transitions=%d" %
                            (result["cycles"], len(result["transitions"])))

            if "subscribe-stress" in args.phases:
                result = await runSubscribeStressPhase(
                    client, observer, profile, detector, sidecar,
                    args.minimumLogStackBytes, args.minimumOtherStackBytes,
                    args.heapLeakToleranceBytes,
                    requireTelemetry=args.profile == "full")
                summary["phases"]["subscribeStress"] = result
                output.emit("BLE_SUBSCRIBE_STRESS_PASS,cycles=%d,windows=%d" %
                            (result["cycles"], result["subscriptionWindows"]))

            if "long-run" in args.phases:
                result = await runBleLongRunPhase(
                    client, observer, sidecar, profile, detector,
                    args.commandTimeout, args.frameTimeout,
                    args.queryInterval, args.knownResistor,
                    args.minimumLogStackBytes, args.minimumOtherStackBytes,
                    args.heapLeakToleranceBytes,
                    requireTelemetry=args.profile == "full")
                summary["phases"]["longRun"] = result
                output.emit("BLE_LONG_RUN_PASS,mode=RES,frames=%d,seconds=%.3f,fps=%.3f" %
                            (result["frames"], result["seconds"], result["fps"]))

            # Restore documented defaults and the original runtime selection
            # where a public setter exists.  CAP is intentionally the final
            # mode requested by both acceptance sequences.
            await resetBleWindowAfterUnsubscribe(observer)
            await startSubscriptions(client, observer, {"C", "D", "L"})
            await switchBleMode(client, observer, "CAP", 1,
                                args.modeTimeout, logSubscribed=True,
                                sidecar=sidecar)
            txRestore = {"rel": "REL", "short": "SHORT", "full": "FULL"}.get(
                initialTx.lower())
            if txRestore:
                await waitBleCommand(client, observer, "TX=%s" % txRestore,
                                     args.commandTimeout)
            streamRestore = {
                "auto": "AUTO", "ser": "SER", "ble": "BLE",
                "wifi": "WIFI", "all": "ALL",
            }.get(initialStream.lower())
            if streamRestore:
                await waitBleCommand(client, observer, "ST=%s" % streamRestore,
                                     args.commandTimeout)
            validateBleWindow(observer, strictSequence=False)
            await stopSubscriptions(client, {"C", "D", "L"})
            btxRestore = initialBtx.upper()
            if btxRestore == "SAFE":
                await resetBleWindowAfterUnsubscribe(observer)
                await startSubscriptions(client, observer, {"C"},
                                         forceIndicate=True)
                await waitBleCommand(client, observer, "BTX=SAFE",
                                     args.commandTimeout)
                await waitBleCommand(client, observer, "BTX?",
                                     args.commandTimeout)
                validateBleWindow(observer)
                await stopSubscriptions(client, {"C"})
            elif btxRestore == "FAST":
                # Setup and every stress phase leave the device in FAST.
                pass
            detector.assertHealthy()
        finally:
            if getattr(client, "is_connected", False):
                await client.disconnect()

        if "reconnect" in args.phases:
            reconnect = await runReconnectStress(
                device, BleakClient, output, detector, profile,
                args.commandTimeout, initialStream, initialTx, initialBtx,
                sidecar, args.minimumLogStackBytes,
                args.minimumOtherStackBytes, args.heapLeakToleranceBytes,
                requireTelemetry=args.profile == "full")
            summary["phases"]["reconnect"] = reconnect
            output.emit("BLE_RECONNECT_PASS,cycles=%d" % reconnect["cycles"])

        detector.assertHealthy()
        nonAsciiDelta = sidecar.nonAsciiLines - baselineNonAsciiLines
        crcDelta = sidecar.protocol.counters.crc_errors - baselineCrcErrors
        malformedDelta = sidecar.protocol.counters.malformed - baselineMalformed
        if nonAsciiDelta or crcDelta or malformedDelta:
            raise HilFailure(
                "serial sidecar wire errors after ready nonAscii=%d crc=%d malformed=%d" %
                (nonAsciiDelta, crcDelta, malformedDelta))
        summary["postReadySerialWireErrors"] = {
            "nonAscii": nonAsciiDelta,
            "crc": crcDelta,
            "malformed": malformedDelta,
        }
        summary["passed"] = True
        summary["unexpectedFaults"] = []
        summary["disconnectCallbacks"] = len(disconnectEvents)
        output.emit("BLE_HIL_PASS,phases=%s,resets=0,panics=0" %
                    "+".join(sorted(args.phases)))
        writeSummary(outputDirectory, "ble_hil", summary)
        return EXIT_PASS
    except HilSkipped as error:
        summary["skipReason"] = str(error)
        output.emit("BLE_HIL_NOT_RUN,reason=%s" % _safeField(str(error)))
        writeSummary(outputDirectory, "ble_hil", summary)
        return EXIT_SKIPPED
    except Exception as error:
        summary["failure"] = "%s: %s" % (type(error).__name__, error)
        summary["disconnectCallbacks"] = len(disconnectEvents)
        if sidecar:
            summary["unexpectedFaults"] = [dataclasses.asdict(item)
                                             for item in sidecar.detector.events()]
        output.emit("BLE_HIL_FAIL,reason=%s,detail=%s" %
                    (type(error).__name__, _safeField(str(error))))
        writeSummary(outputDirectory, "ble_hil", summary)
        return EXIT_FAIL
    finally:
        if sidecar:
            sidecar.close()
        output.close()


def addCommonArguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--profile", choices=("smoke", "full"), default="smoke",
                        help="full selects the requested long-run/stress acceptance counts")
    parser.add_argument("--output-directory", dest="outputDirectory",
                        help="write a full wire log and machine-readable summary")
    parser.add_argument("--startup-timeout", dest="startupTimeout", type=float,
                        default=60.0)
    parser.add_argument("--command-timeout", dest="commandTimeout", type=float,
                        default=10.0)
    parser.add_argument("--mode-timeout", dest="modeTimeout", type=float,
                        default=90.0)
    parser.add_argument("--frame-timeout", dest="frameTimeout", type=float,
                        default=30.0)
    parser.add_argument("--query-interval", dest="queryInterval", type=float,
                        default=30.0)
    parser.add_argument("--minimum-log-stack-bytes", dest="minimumLogStackBytes",
                        type=int, default=2048)
    parser.add_argument("--minimum-other-stack-bytes", dest="minimumOtherStackBytes",
                        type=int, default=512)
    parser.add_argument("--heap-leak-tolerance-bytes", dest="heapLeakToleranceBytes",
                        type=int, default=4096)
    parser.add_argument(
        "--rail-avdd-uv", dest="railAvddUv", type=int,
        help="explicit positive AVDD-to-GND calibration required for VOLT HIL")
    parser.add_argument(
        "--rail-avss-uv", dest="railAvssUv", type=int,
        help="explicit negative AVSS-to-GND calibration required for VOLT HIL")
    parser.add_argument(
        "--known-resistor", dest="knownResistor", action="append",
        type=parseKnownResistance,
        default=None,
        help="wide HIL sanity range, for example S1D1:5000:20000")


def buildArgumentParser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SensorArray serial/BLE control, data, log and stability HIL")
    subparsers = parser.add_subparsers(dest="transport", required=True)

    serialParser = subparsers.add_parser("serial", help="serial control/data HIL")
    addCommonArguments(serialParser)
    serialParser.add_argument("--port",
                              help="overrides SENSORARRAY_SERIAL_PORT")
    serialParser.add_argument("--baud", type=int, default=115200)
    serialParser.add_argument("--query", action="append",
                              default=list(DEFAULT_QUERY_COMMANDS))
    serialParser.add_argument("--modes", type=parseModes,
                              default=parseModes("CAP,RES,VOLT,CAP"))
    serialParser.add_argument("--mode-cycles", dest="modeCycles", type=int,
                              default=1)
    serialParser.add_argument("--frames-per-mode", dest="framesPerMode", type=int,
                              default=10)
    serialParser.add_argument("--long-run-mode", dest="longRunMode",
                              choices=("CAP", "RES", "VOLT"), default="RES")
    serialParser.add_argument("--long-run-frames", dest="longRunFrames", type=int)
    serialParser.add_argument("--long-run-seconds", dest="longRunSeconds", type=float)
    serialParser.add_argument("--long-run-maximum-seconds",
                              dest="longRunMaximumSeconds", type=float)

    bleParser = subparsers.add_parser("ble", help="BLE matrix/stress HIL with serial sidecar")
    addCommonArguments(bleParser)
    bleParser.add_argument("--serial-port", dest="serialPort",
                           help="serial panic observer; overrides SENSORARRAY_SERIAL_PORT")
    bleParser.add_argument("--serial-baud", dest="serialBaud", type=int,
                           default=115200)
    bleParser.add_argument("--address",
                           help="overrides SENSORARRAY_BLE_ADDRESS")
    bleParser.add_argument("--name-prefix", dest="namePrefix", default="CscArray_")
    bleParser.add_argument("--scan-seconds", dest="scanSeconds", type=float,
                           default=10.0)
    bleParser.add_argument("--query", action="append",
                           default=list(DEFAULT_QUERY_COMMANDS))
    bleParser.add_argument(
        "--phases", type=parsePhases,
        default=parsePhases("matrix,tx-modes,mode-stress,subscribe-stress,reconnect,long-run"))
    bleParser.add_argument(
        "--subscription-cases", dest="subscriptionCases", type=parseSubscriptionCases,
        default=parseSubscriptionCases("none,11,20,30,11+20,11+30,20+30,11+20+30"))
    for option, destination, valueType in (
            ("--connected-idle-seconds", "connectedIdleSeconds", float),
            ("--case-data-frames", "caseDataFrames", int),
            ("--ff20-only-frames", "ff20OnlyFrames", int),
            ("--ff20-only-seconds", "ff20OnlySeconds", float),
            ("--case-log-messages", "caseLogMessages", int),
            ("--case-timeout", "caseTimeout", float),
            ("--tx-mode-frames", "txModeFrames", int),
            ("--mode-cycles", "modeCycles", int),
            ("--frames-per-mode", "framesPerMode", int),
            ("--subscribe-cycles", "subscribeCycles", int),
            ("--reconnect-cycles", "reconnectCycles", int),
            ("--long-run-frames", "longRunFrames", int),
            ("--long-run-seconds", "longRunSeconds", float),
            ("--long-run-maximum-seconds", "longRunMaximumSeconds", float)):
        bleParser.add_argument(option, dest=destination, type=valueType)
    return parser


def validateArguments(parser: argparse.ArgumentParser,
                      args: argparse.Namespace) -> None:
    if args.knownResistor is None:
        args.knownResistor = defaultKnownResistances()
    positiveFields = ("startupTimeout", "commandTimeout", "modeTimeout",
                      "frameTimeout", "minimumLogStackBytes",
                      "minimumOtherStackBytes")
    for fieldName in positiveFields:
        if getattr(args, fieldName) <= 0.0:
            parser.error("--%s must be positive" % fieldName)
    if args.heapLeakToleranceBytes < 0:
        parser.error("--heap-leak-tolerance-bytes must be non-negative")
    if (args.railAvddUv is None) != (args.railAvssUv is None):
        parser.error("--rail-avdd-uv and --rail-avss-uv must be supplied together")
    if args.railAvddUv is not None and (
            args.railAvddUv <= 0 or args.railAvssUv >= 0):
        parser.error("rail calibration requires positive AVDD and negative AVSS")
    if args.transport == "serial":
        if args.modeCycles < 1 or args.framesPerMode < 1:
            parser.error("serial mode cycles and frames per mode must be positive")
        for fieldName in ("longRunFrames", "longRunSeconds",
                          "longRunMaximumSeconds"):
            value = getattr(args, fieldName)
            if value is not None and value < 0:
                parser.error("serial long-run values must be non-negative")
    else:
        numericFields = (
            "connectedIdleSeconds", "caseDataFrames", "ff20OnlyFrames",
            "ff20OnlySeconds", "caseLogMessages", "caseTimeout", "txModeFrames",
            "modeCycles", "framesPerMode", "subscribeCycles", "reconnectCycles",
            "longRunFrames", "longRunSeconds", "longRunMaximumSeconds",
        )
        for fieldName in numericFields:
            value = getattr(args, fieldName)
            if value is not None and value < 0:
                parser.error("BLE profile overrides must be non-negative")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = buildArgumentParser()
    args = parser.parse_args(argv)
    validateArguments(parser, args)
    if args.transport == "serial":
        return runSerialHil(args)
    return asyncio.run(runBleHil(args))


if __name__ == "__main__":
    raise SystemExit(main())
