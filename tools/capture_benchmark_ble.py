#!/usr/bin/env python3
"""Receive-only BLE notification journal, synchronized by host monotonic clock.

No serial port or application GATT command is opened/sent. Only the three CCCD
subscriptions are requested. The existing TX/BTX configuration is left intact.
Run before the serial benchmark; wait for <label>.ble.ready.json, then launch
the serial suite. Finish with --stop-file (create that file), SIGINT or SIGTERM.
The duration is a finite total session ceiling, including connection/cleanup.
No reconnect is attempted. Offline analysis must distinguish unconnected time
from connected zero-frame windows and callback time from over-air reception.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
from collections import Counter
from contextlib import ExitStack
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import signal
import time

from receive_ble_text import SERVICE, CTRL_TX, DATA_TX, LOG_TX

CHANNELS = {"C": CTRL_TX, "D": DATA_TX, "L": LOG_TX}


@dataclass(frozen=True)
class Options:
    address: str
    output_directory: Path
    label: str
    name: str | None = None
    adapter: str = "hci0"
    duration: float = 4500.0
    stop_file: Path | None = None
    scan_seconds: float = 10.0
    connect_seconds: float = 15.0
    operation_seconds: float = 5.0
    cleanup_seconds: float = 10.0
    progress_seconds: float = 20.0


class Journal:
    """At most 64 KiB file buffering; no unbounded decoded-frame queue."""
    def __init__(self, stream):
        self.stream = stream
        self.started_ns = time.monotonic_ns()
        self.sha = hashlib.sha256()
        self.bytes = self.unflushed = 0
        self.last_flush = time.monotonic()

    def emit(self, kind, *, monotonic_ns=None, **fields):
        ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        record = {"schema_version": 1, "type": kind, "monotonic_ns": ns,
                  "t_seconds": (ns - self.started_ns) / 1e9, **fields}
        payload = (json.dumps(record, separators=(",", ":")) + "\n").encode("utf-8")
        written = self.stream.write(payload)
        if written != len(payload):
            raise OSError("journal_partial_write")
        self.sha.update(payload)
        self.bytes += written
        self.unflushed += written
        if self.unflushed >= 65536 or time.monotonic() - self.last_flush >= 0.25:
            self.flush()

    def flush(self):
        self.stream.flush()
        self.unflushed = 0
        self.last_flush = time.monotonic()


class Recorder:
    def __init__(self, options, journal, ready_path):
        self.options, self.journal, self.ready_path = options, journal, ready_path
        self.started = time.monotonic()
        self.deadline = self.started + options.duration
        self.stop = asyncio.Event()
        self.stop_reason = None
        self.phase = "prepare"
        self.callback_error = None
        self.connected_ns = self.ready_ns = self.disconnected_ns = self.finished_ns = None
        self.unexpected_disconnect = False
        self.counts, self.payload_bytes = Counter(), Counter()
        self.notification_index = 0
        self.maximum_notification_bytes = 0
        self.first_notify_ns = self.last_notify_ns = None

    def request_stop(self, reason):
        if self.stop_reason is None:
            self.stop_reason = reason
        self.stop.set()

    def callback(self, channel):
        def received(_sender, data):
            ns = time.monotonic_ns()
            payload = bytes(data)  # Copy before a backend can reuse its buffer.
            try:
                self.notification_index += 1
                self.journal.emit("notification", monotonic_ns=ns, index=self.notification_index,
                                  channel=channel, characteristic_uuid=CHANNELS[channel],
                                  length=len(payload), payload_base64=base64.b64encode(payload).decode("ascii"))
                self.counts[channel] += 1
                self.payload_bytes[channel] += len(payload)
                self.maximum_notification_bytes = max(self.maximum_notification_bytes, len(payload))
                self.first_notify_ns = self.first_notify_ns or ns
                self.last_notify_ns = ns
            except Exception as error:
                self.callback_error = repr(error)
                self.request_stop("journal_error")
        return received

    def on_disconnect(self, _client):
        ns = time.monotonic_ns()
        self.disconnected_ns = ns
        unexpected = self.phase != "cleanup"
        self.unexpected_disconnect |= unexpected
        try:
            self.journal.emit("disconnected", monotonic_ns=ns, unexpected=unexpected,
                              reconnect_attempted=False)
        except Exception as error:
            self.callback_error = repr(error)
        if unexpected:
            self.request_stop("runtime_disconnect")

    async def bounded(self, awaitable, seconds, *, cleanup=False):
        limit = self.deadline if cleanup else self.deadline - self.options.cleanup_seconds
        remaining = min(seconds, limit - time.monotonic())
        if remaining <= 0:
            if hasattr(awaitable, "close"):
                awaitable.close()
            raise TimeoutError("session_deadline_expired")
        return await asyncio.wait_for(awaitable, remaining)

    async def run(self, scanner, client_class):
        client, subscribed, failure = None, [], None
        cleanup_errors = []
        mtu = None
        selected = None
        handlers = []
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, self.request_stop, sig.name)
                handlers.append(sig)
            except (NotImplementedError, RuntimeError):
                pass
        self.journal.emit("metadata", collector="capture_benchmark_ble", address_requested=self.options.address,
                          name_requested=self.options.name, service_uuid=SERVICE, channels=CHANNELS,
                          adapter=self.options.adapter, application_commands_sent=0, serial_opened=False,
                          clock_origin={"kind": "time.monotonic_ns", "origin_monotonic_ns": self.journal.started_ns,
                                        "wall_utc": datetime.now(timezone.utc).isoformat(),
                                        "scope": "same_host_monotonic_clock_as_serial_runner",
                                        "timestamp_semantics": "host_notification_callback_not_over_air"},
                          python=platform.python_version(), system=platform.system(),
                          duration_total_ceiling_seconds=self.options.duration,
                          buffering_max_bytes=65536, flush_interval_seconds=0.25)
        try:
            device = await self.bounded(scanner.find_device_by_address(self.options.address,
                timeout=self.options.scan_seconds, adapter=self.options.adapter), self.options.scan_seconds + 1)
            if device is None or device.address.upper() != self.options.address.upper():
                raise RuntimeError("exact_address_not_found")
            if self.options.name and device.name != self.options.name:
                raise RuntimeError("exact_address_name_mismatch:" + str(device.name))
            selected = {"address": device.address, "name": device.name}
            self.journal.emit("device_selected", **selected)
            client = client_class(device, disconnected_callback=self.on_disconnect,
                                  timeout=self.options.connect_seconds)
            await self.bounded(client.connect(), self.options.connect_seconds)
            self.connected_ns = time.monotonic_ns()
            self.journal.emit("connected", monotonic_ns=self.connected_ns, **selected)
            if SERVICE not in {service.uuid.lower() for service in client.services}:
                raise RuntimeError("expected_service_missing")
            for channel, uuid in CHANNELS.items():
                await self.bounded(client.start_notify(uuid, self.callback(channel)), self.options.operation_seconds)
                subscribed.append(channel)
                self.journal.emit("subscribed", channel=channel, characteristic_uuid=uuid)
            try:
                mtu = getattr(client, "mtu_size", None)
            except Exception:
                mtu = None
            self.ready_ns = time.monotonic_ns()
            self.phase = "recording"
            ready = {"schema_version": 1, "monotonic_ns": self.ready_ns, "address": device.address,
                     "all_channels_subscribed": subscribed, "service_uuid": SERVICE,
                     "mtu_reported_by_backend": mtu, "negotiated_mtu_verified": False}
            self.journal.emit("subscriptions_ready", **{k: v for k, v in ready.items() if k != "schema_version"})
            self.journal.flush()
            with self.ready_path.open("x", encoding="utf-8") as ready_file:
                ready_file.write(json.dumps(ready, indent=2) + "\n")
            print(json.dumps({"type": "BLE_READY", "ready_file": str(self.ready_path), **ready}), flush=True)
            progress = time.monotonic() + self.options.progress_seconds
            while not self.stop.is_set():
                now = time.monotonic()
                if now >= self.deadline - self.options.cleanup_seconds:
                    self.request_stop("duration_ceiling")
                    break
                if self.options.stop_file and self.options.stop_file.exists():
                    self.request_stop("stop_file")
                    break
                if self.callback_error:
                    raise RuntimeError("notification_journal_error:" + self.callback_error)
                if now - self.journal.last_flush >= 0.25:
                    self.journal.flush()
                if now >= progress:
                    status = {"type": "BLE_PROGRESS", "elapsed_seconds": now - self.started,
                              "notifications": dict(self.counts), "payload_bytes": dict(self.payload_bytes)}
                    print(json.dumps(status), flush=True)
                    progress = now + self.options.progress_seconds
                try:
                    await asyncio.wait_for(self.stop.wait(), min(0.1, self.deadline - self.options.cleanup_seconds - now))
                except asyncio.TimeoutError:
                    pass
            if self.unexpected_disconnect:
                raise RuntimeError("BLE_disconnected_no_reconnect")
            if self.callback_error:
                raise RuntimeError("notification_journal_error:" + self.callback_error)
        except (Exception, asyncio.CancelledError) as error:
            failure = repr(error)
            try:
                self.journal.emit("error", error=failure, phase=self.phase)
            except OSError:
                pass
        finally:
            self.finished_ns = time.monotonic_ns()
            self.phase = "cleanup"
            # Closing the capture interval precedes unsubscribe/disconnect.
            # Notifications delivered during cleanup remain in the journal.
            try:
                self.journal.emit("recording_end", monotonic_ns=self.finished_ns,
                                  stop_reason=self.stop_reason, error=failure)
            except OSError:
                pass
            if client is not None:
                for channel in subscribed:
                    try:
                        await self.bounded(client.stop_notify(CHANNELS[channel]), min(1.0, self.options.operation_seconds), cleanup=True)
                    except (Exception, asyncio.CancelledError) as error:
                        cleanup_errors.append(f"stop_{channel}:{error!r}")
                try:
                    await self.bounded(client.disconnect(), self.options.operation_seconds, cleanup=True)
                except (Exception, asyncio.CancelledError) as error:
                    cleanup_errors.append(f"disconnect:{error!r}")
            for sig in handlers:
                loop.remove_signal_handler(sig)
            self.journal.flush()
        if cleanup_errors:
            failure = (failure + ";" if failure else "") + "cleanup_errors"
        return {"schema_version": 1, "status": "ERROR" if failure else "RECORDED", "error": failure,
                "stop_reason": self.stop_reason, "cleanup_errors": cleanup_errors,
                "selected_device": selected, "address_requested": self.options.address,
                "service_uuid": SERVICE, "characteristics": CHANNELS, "subscribed_channels": subscribed,
                "connected_monotonic_ns": self.connected_ns, "ready_monotonic_ns": self.ready_ns,
                "recording_end_monotonic_ns": self.finished_ns, "disconnected_monotonic_ns": self.disconnected_ns,
                "unexpected_disconnect": self.unexpected_disconnect, "reconnect_attempted": False,
                "notification_counts": dict(self.counts), "notification_payload_bytes": dict(self.payload_bytes),
                "maximum_observed_notification_bytes": self.maximum_notification_bytes,
                "first_notification_monotonic_ns": self.first_notify_ns, "last_notification_monotonic_ns": self.last_notify_ns,
                "mtu_reported_by_backend": mtu, "negotiated_mtu_verified": False,
                "mtu_note": "Backend value may be 23 on BlueZ without proving negotiated MTU; unknown remains null.",
                "application_commands_sent": 0, "serial_opened": False, "mode_or_rate_settings_written": False,
                "recording_seconds": (self.finished_ns - self.ready_ns) / 1e9 if self.ready_ns else 0,
                "elapsed_seconds": time.monotonic() - self.started,
                "measurement_acceptance": "NOT_EVALUATED",
                "unconnected_windows": "NOT_RUN_not_zero_FPS",
                "journal_cost": "base64 + buffered file write in callback; no frame duplication or fsync; host I/O can affect callback latency"}


async def capture(options, *, backend=None):
    if not re.fullmatch(r"[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}", options.address):
        raise ValueError("explicit Bluetooth address required")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", options.label):
        raise ValueError("invalid label")
    if not math.isfinite(options.duration) or not 0 < options.duration <= 7200:
        raise ValueError("duration must be finite and at most 7200 seconds")
    if options.duration <= options.cleanup_seconds:
        raise ValueError("duration must exceed cleanup budget")
    if options.stop_file and options.stop_file.exists():
        raise FileExistsError("stop file already exists; choose a new stop signal path")
    options.output_directory.mkdir(parents=True, exist_ok=True)
    paths = {"journal": options.output_directory / (options.label + ".ble.notifications.jsonl"),
             "summary": options.output_directory / (options.label + ".ble.summary.json"),
             "ready": options.output_directory / (options.label + ".ble.ready.json")}
    if any(path.exists() for path in paths.values()):
        raise FileExistsError("BLE evidence output exists; choose a new --label")
    if backend is None:
        from bleak import BleakClient, BleakScanner
        backend = BleakScanner, BleakClient
    with ExitStack() as stack:
        stream = stack.enter_context(paths["journal"].open("xb", buffering=65536))
        summary = stack.enter_context(paths["summary"].open("x", encoding="utf-8"))
        journal = Journal(stream)
        result = await Recorder(options, journal, paths["ready"]).run(*backend)
        journal.flush()
        result.update(journal_file=str(paths["journal"]), journal_sha256=journal.sha.hexdigest(),
                      journal_bytes=journal.bytes, ready_file=str(paths["ready"]) if paths["ready"].exists() else None)
        summary.write(json.dumps(result, indent=2) + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", required=True)
    parser.add_argument("--name")
    parser.add_argument("--adapter", default="hci0")
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--duration", type=float, default=4500.0)
    parser.add_argument("--stop-file", type=Path)
    try:
        result = asyncio.run(capture(Options(**vars(parser.parse_args(argv)))))
    except (OSError, ValueError, ImportError) as error:
        parser.exit(2, f"BLE_CAPTURE_ERROR: {error}\n")
    print(json.dumps({key: result.get(key) for key in ("status", "error", "stop_reason", "notification_counts", "journal_file", "journal_sha256")}), flush=True)
    return 0 if result["status"] == "RECORDED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
