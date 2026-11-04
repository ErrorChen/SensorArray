#!/usr/bin/env python3
"""Bounded receive-only SensorArray BLE recorder. Never opens Serial or FF10.

Root should additionally bound the process with timeout --kill-after=5s 200s.
CCCD subscriptions are expected GATT writes; no application command is sent.
"""
from __future__ import annotations
import argparse
import asyncio
from collections import Counter
from contextlib import ExitStack
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, "/home/csc/esp/workspace/SensorArray/tools")
from text_protocol import FragmentReassembler, TextProtocolParser, parse_fields
from reference_fixture import analyse_capture

SERVICE = "000000ff-0000-1000-8000-00805f9b34fb"
CHANNELS = {"C": "0000ff11-0000-1000-8000-00805f9b34fb",
            "D": "0000ff20-0000-1000-8000-00805f9b34fb",
            "L": "0000ff30-0000-1000-8000-00805f9b34fb"}


@dataclass
class Options:
    address: str
    output_directory: Path
    label: str
    name: str = "CscArray_CDD9AD"
    duration: float = 140.0
    adapter: str = "hci0"
    scan_seconds: float = 10.0
    connect_seconds: float = 15.0
    operation_seconds: float = 5.0
    cleanup_seconds: float = 10.0
    progress_seconds: float = 20.0


class Recorder:
    def __init__(self, options, streams):
        self.options, self.streams = options, streams
        self.started = time.monotonic()
        self.deadline = self.started + options.duration + 50.0
        self.reassembler, self.parser = FragmentReassembler(), TextProtocolParser()
        self.line_buffers = {key: bytearray() for key in CHANNELS}
        self.notify_count, self.notify_bytes = Counter(), Counter()
        self.max_notify_bytes = 0
        self.disconnected = False
        self.phase = "prepare"
        self.capture_start = self.capture_end = None
        self.first_notify = self.last_notify = None
        self.faults, self.incomplete, self.ble_status = [], [], []
        self.non_ascii = 0
        self.callback_error = None
        self.app_writes = 0

    def event(self, kind, **values):
        self.streams["events"].write(json.dumps({"type": kind, "host_monotonic_ns": time.monotonic_ns(),
            "elapsed_seconds": time.monotonic() - self.started, "phase": self.phase, **values}) + "\n")
        self.streams["events"].flush()

    def expire(self, final=False):
        now = time.monotonic()
        items = self.reassembler._messages
        for key, item in list(items.items()):
            if final or now - item["created"] > 5.0 or len(items) > 128:
                n, parts = int(item["n"]), item["parts"]
                record = {"channel": key[0], "message_id": key[1], "fragment_count": n,
                          "received_indices": sorted(parts), "missing_count": n - len(parts),
                          "reason": "capture_end" if final else "age_or_inflight_bound"}
                self.incomplete.append(record)
                self.event("incomplete_envelope", **record)
                del items[key]

    def callback(self, fallback):
        def received(_sender, data):
            try:
                payload = bytes(data)
                ns, now = time.monotonic_ns(), time.monotonic()
                self.first_notify = self.first_notify or now
                self.last_notify = now
                self.notify_count[fallback] += 1
                self.notify_bytes[fallback] += len(payload)
                self.max_notify_bytes = max(self.max_notify_bytes, len(payload))
                self.streams["notifications"].write(json.dumps({"host_monotonic_ns": ns,
                    "elapsed_seconds": now - self.started, "phase": self.phase,
                    "channel": fallback, "characteristic": CHANNELS[fallback],
                    "length": len(payload), "raw_hex": payload.hex()}) + "\n")
                self.streams["notifications"].flush()
                for channel, message in self.reassembler.feed(fallback, payload):
                    self.streams["logical"].write(json.dumps({"host_monotonic_ns": ns,
                        "channel": channel, "length": len(message), "raw_hex": message.hex()}) + "\n")
                    self.streams["logical"].flush()
                    if channel == "D":
                        self.streams["data"].write(message)
                        self.streams["data"].flush()
                    buffer = self.line_buffers[channel]
                    buffer.extend(message)
                    while b"\n" in buffer:
                        wire, _, remainder = buffer.partition(b"\n")
                        buffer[:] = remainder
                        try:
                            line = wire.rstrip(b"\r").decode("ascii")
                        except UnicodeDecodeError:
                            self.non_ascii += 1
                            continue
                        if channel == "D":
                            self.parser.feed_line(line)
                        if line.startswith("BL50,"):
                            self.ble_status.append({"host_monotonic_ns": ns, "fields": parse_fields(line)})
                        if line.startswith(("MFAULT,", "MERR,", "RMERR,", "APP_FATAL", "FDC_FATAL", "RST,")) or any(
                                marker in line.lower() for marker in ("guru meditation", "task watchdog got triggered", "interrupt wdt timeout", "rst:0x")):
                            self.faults.append(line)
                            self.event("firmware_fault_or_reset", line=line)
                    if len(buffer) > 65536:
                        raise RuntimeError("logical_line_exceeds_65536_bytes")
                self.expire()
            except Exception as error:
                self.callback_error = repr(error)
        return received

    async def bounded(self, awaitable, seconds):
        remaining = min(seconds, self.deadline - time.monotonic())
        if remaining <= 0:
            if hasattr(awaitable, "close"):
                awaitable.close()
            raise TimeoutError("total_deadline_expired")
        return await asyncio.wait_for(awaitable, remaining)

    def on_disconnect(self, _client):
        self.disconnected = True
        self.event("disconnected")

    async def run(self, scanner, client_class):
        client, subscribed, failure, mtu = None, [], None, None
        cleanup_errors = []
        self.event("begin", address=self.options.address, application_commands_sent=0, serial_opened=False)
        try:
            device = await self.bounded(scanner.find_device_by_address(self.options.address,
                timeout=self.options.scan_seconds, adapter=self.options.adapter), self.options.scan_seconds + 1)
            if device is None or device.address.upper() != self.options.address.upper():
                raise RuntimeError("exact_address_not_found")
            if self.options.name and device.name != self.options.name:
                raise RuntimeError(f"exact_address_name_mismatch:{device.name}")
            self.event("device_selected", address=device.address, name=device.name)
            client = client_class(device, disconnected_callback=self.on_disconnect,
                                  timeout=self.options.connect_seconds)
            await self.bounded(client.connect(), self.options.connect_seconds)
            if SERVICE not in {service.uuid.lower() for service in client.services}:
                raise RuntimeError("expected_service_missing")
            for channel, uuid in CHANNELS.items():
                await self.bounded(client.start_notify(uuid, self.callback(channel)), self.options.operation_seconds)
                subscribed.append(channel)
                self.event("subscribed", channel=channel, characteristic=uuid)
            mtu = getattr(client, "mtu_size", None)
            self.phase = "capture"
            self.capture_start = time.monotonic()
            self.event("capture_start", mtu_reported_by_backend=mtu, negotiated_mtu_verified=False)
            end = min(self.capture_start + self.options.duration, self.deadline - self.options.cleanup_seconds)
            progress_at = self.capture_start + self.options.progress_seconds
            while time.monotonic() < end:
                if self.disconnected:
                    raise RuntimeError("BLE_disconnected")
                if self.callback_error:
                    raise RuntimeError("callback_error:" + self.callback_error)
                if self.faults:
                    raise RuntimeError("firmware_fault_or_reset")
                now = time.monotonic()
                if now >= progress_at:
                    status = {"type": "BLE_PROGRESS", "capture_seconds": now - self.capture_start,
                              "notifications": sum(self.notify_count.values()),
                              "payload_bytes": sum(self.notify_bytes.values()), "parser": asdict(self.parser.counters)}
                    print(json.dumps(status), flush=True)
                    self.event("progress", **{key: value for key, value in status.items() if key != "type"})
                    self.expire()
                    progress_at = now + self.options.progress_seconds
                await asyncio.sleep(min(0.1, max(0, end - now)))
            self.capture_end = time.monotonic()
            if self.capture_end - self.capture_start < self.options.duration - 0.01:
                raise TimeoutError("capture_window_shortened_by_total_deadline")
            if not sum(getattr(self.parser.counters, key) for key in ("cap_frames", "voltage_frames", "resistance_frames", "mixed_frames")):
                raise RuntimeError("no_complete_scientific_frames")
        except (Exception, asyncio.CancelledError) as error:
            failure = repr(error)
            self.event("error", error=failure)
        finally:
            self.capture_end = self.capture_end or (time.monotonic() if self.capture_start else None)
            self.phase = "cleanup"
            self.event("capture_end")
            if client is not None:
                for channel in subscribed:
                    try:
                        await self.bounded(client.stop_notify(CHANNELS[channel]), min(1.0, self.options.operation_seconds))
                    except (Exception, asyncio.CancelledError) as error:
                        cleanup_errors.append(f"stop_{channel}:{error!r}")
                try:
                    await self.bounded(client.disconnect(), self.options.operation_seconds)
                except (Exception, asyncio.CancelledError) as error:
                    cleanup_errors.append(f"disconnect:{error!r}")
            self.expire(final=True)
        if cleanup_errors:
            failure = (failure + ";" if failure else "") + "cleanup_errors"
        duration = self.capture_end - self.capture_start if self.capture_start else 0.0
        return {"status": "ERROR" if failure else "CAPTURED", "error": failure,
                "measurement_acceptance": "NOT_EVALUATED", "cleanup_errors": cleanup_errors,
                "address_requested": self.options.address, "name_requested": self.options.name,
                "capture_seconds": duration, "requested_capture_seconds": self.options.duration,
                "elapsed_seconds": time.monotonic() - self.started,
                "application_commands_sent": 0, "serial_opened": False, "cccd_subscriptions": subscribed,
                "mtu_reported_by_backend": mtu, "negotiated_mtu_verified": False,
                "mtu_note": "Bleak BlueZ may report 23 irrespective of negotiated MTU; compare firmware BL50.mtu. No private MTU workaround is used.",
                "notification_counts": dict(self.notify_count), "notification_payload_bytes": dict(self.notify_bytes),
                "maximum_observed_notification_bytes": self.max_notify_bytes,
                "first_to_last_notify_seconds": (self.last_notify - self.first_notify) if self.first_notify else None,
                "live_parser_counters": asdict(self.parser.counters),
                "live_parser_sequence_scope": "global; use offline identity groups for boot/revision boundaries",
                "fragment_stats": {key: asdict(value) for key, value in self.reassembler.stats.items()},
                "incomplete_envelopes": self.incomplete, "non_ascii_logical_lines": self.non_ascii,
                "unfinished_line_bytes": {key: len(value) for key, value in self.line_buffers.items()},
                "firmware_faults_or_resets": self.faults, "firmware_BL50_records": self.ble_status,
                "throughput_scope": "Host callback payload bytes include envelope headers and all three channels; not over-air bitrate or capture FPS",
                "notification_logging_cost": "Synchronous buffered writes/flush in callbacks; no fsync. Host logging can affect receive throughput."}


async def capture(options, backend=None):
    if not re.fullmatch(r"[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}", options.address):
        raise ValueError("explicit Bluetooth address required")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", options.label) or not 0 < options.duration <= 3600:
        raise ValueError("invalid label or duration")
    suffixes = {"notifications": ".notifications.jsonl", "logical": ".logical.jsonl", "data": ".data.rx.bin",
                "events": ".events.jsonl", "summary": ".summary.json"}
    options.output_directory.mkdir(parents=True, exist_ok=True)
    paths = {key: options.output_directory / (options.label + suffix) for key, suffix in suffixes.items()}
    if any(path.exists() for path in paths.values()):
        raise FileExistsError("output exists; choose a new --label")
    if backend is None:
        from bleak import BleakClient, BleakScanner
        backend = BleakScanner, BleakClient
    with ExitStack() as stack:
        streams = {key: stack.enter_context(path.open("xb" if key == "data" else "x", **({} if key == "data" else {"encoding": "utf-8"}))) for key, path in paths.items()}
        receiver = Recorder(options, streams)
        result = await receiver.run(*backend)
        streams["data"].flush()
        payload = paths["data"].read_bytes()
        analysed = analyse_capture(payload.decode("ascii", errors="surrogateescape").splitlines())
        result["offline_protocol"] = {"parser_counters": analysed.counters,
            "truncated_frames": analysed.truncated_frames, "malformed_frames": analysed.malformed_frames,
            "identity_errors": analysed.identity_errors, "non_ascii_lines": analysed.non_ascii_lines,
            "groups": [{"identity": group.identity, "identity_status": group.identity_status,
                        "frame_type": group.frame_type, "rows": group.rows, "frames": group.frames,
                        "first_sequence": group.first_sequence, "last_sequence": group.last_sequence,
                        "timestamp_span_seconds": group.duration(), "sequence_gaps": group.sequence_gaps,
                        "missing_sequences": group.missing_sequences, "duplicate_or_old_frames": group.duplicate_or_old_frames}
                       for group in analysed.groups]}
        result["first_data_identity"] = analysed.groups[0].identity if analysed.groups else None
        result["last_data_identity"] = analysed.groups[-1].identity if analysed.groups else None
        result["identity_query_commands_sent"] = False
        result["files"] = {key: {"path": str(path), "bytes": path.stat().st_size,
                                 "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                           for key, path in paths.items() if key != "summary"}
        streams["summary"].write(json.dumps(result, indent=2) + "\n")
        streams["summary"].flush()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", required=True)
    parser.add_argument("--name", default="CscArray_CDD9AD")
    parser.add_argument("--duration", type=float, default=140.0)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--adapter", default="hci0")
    options = Options(**vars(parser.parse_args()))
    try:
        result = asyncio.run(capture(options))
    except (OSError, ValueError, ImportError) as error:
        parser.exit(2, f"BLE_CAPTURE_ERROR: {error}\n")
    print(json.dumps({"status": result["status"], "error": result["error"],
                      "capture_seconds": result["capture_seconds"],
                      "notifications": result["notification_counts"],
                      "summary_label": options.label}), flush=True)
    return 0 if result["status"] == "CAPTURED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
