#!/usr/bin/env python3
"""BLE journal regressions use mocked GATT only: no RF, serial or device writes."""
import asyncio
import base64
import contextlib
from dataclasses import replace
import hashlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from capture_benchmark_ble import CHANNELS, Options, SERVICE, capture

ADDRESS = "80:B5:4E:CD:D9:AE"
PAYLOADS = {key: bytes((0, 255, ord(key))) + b"\r\nG,D,1,0,1\nraw" for key in CHANNELS}


def backend(*, found=True, fail_connect=False, disconnect=False, service=True, stall_cleanup=False):
    clients = []
    class Scanner:
        @staticmethod
        async def find_device_by_address(address, **kwargs):
            assert address == ADDRESS and kwargs["adapter"] == "hci0"
            return SimpleNamespace(address=ADDRESS, name="synthetic") if found else None
    class Client:
        def __init__(self, device, disconnected_callback, **kwargs):
            assert device.address == ADDRESS
            self.on_disconnect = disconnected_callback
            self.services = [SimpleNamespace(uuid=SERVICE)] if service else []
            self.mtu_size = 23
            self.subscribed, self.stopped = [], []
            self.app_writes = 0
            self.pending = None
            clients.append(self)
        async def connect(self):
            if fail_connect:
                await asyncio.sleep(1)
        async def start_notify(self, uuid, callback):
            channel = next(key for key, value in CHANNELS.items() if value == uuid)
            self.subscribed.append(channel)
            data = bytearray(PAYLOADS[channel])
            callback(SimpleNamespace(uuid=uuid), data)
            data[:] = b"mutated_backend_buffer"
            if disconnect and channel == "L":
                async def later():
                    await asyncio.sleep(0.015)
                    self.on_disconnect(self)
                self.pending = asyncio.create_task(later())
        async def stop_notify(self, uuid):
            self.stopped.append(uuid)
            if stall_cleanup:
                await asyncio.sleep(1)
        async def disconnect(self):
            if self.pending:
                await self.pending
            self.on_disconnect(self)
        async def write_gatt_char(self, *_args, **_kwargs):
            self.app_writes += 1
            raise AssertionError("application writes are forbidden")
    return (Scanner, Client), clients


class BleJournalTests(unittest.IsolatedAsyncioTestCase):
    def options(self, directory, **changes):
        options = Options(ADDRESS, Path(directory), "synthetic", name="synthetic",
                          duration=0.14, scan_seconds=0.01, connect_seconds=0.015,
                          operation_seconds=0.01, cleanup_seconds=0.035, progress_seconds=0.02)
        return replace(options, **changes)

    async def invoke(self, options, mocked):
        with contextlib.redirect_stdout(io.StringIO()):
            return await capture(options, backend=mocked)

    async def test_exact_bytes_clock_hash_no_writes_and_exclusive_outputs(self):
        mocked, clients = backend()
        with tempfile.TemporaryDirectory() as directory:
            options = self.options(directory)
            result = await self.invoke(options, mocked)
            self.assertEqual(result["status"], "RECORDED", result)
            self.assertEqual(result["stop_reason"], "duration_ceiling")
            self.assertEqual(result["notification_counts"], {"C": 1, "D": 1, "L": 1})
            self.assertEqual(clients[0].app_writes, 0)
            self.assertEqual(clients[0].subscribed, ["C", "D", "L"])
            self.assertFalse(result["serial_opened"])
            self.assertFalse(result["negotiated_mtu_verified"])
            self.assertEqual(result["mtu_reported_by_backend"], 23)
            payload = Path(result["journal_file"]).read_bytes()
            self.assertEqual(len(payload), result["journal_bytes"])
            self.assertEqual(hashlib.sha256(payload).hexdigest(), result["journal_sha256"])
            events = [json.loads(line) for line in payload.splitlines()]
            self.assertTrue(all(event["schema_version"] == 1 for event in events))
            self.assertTrue(all(isinstance(event["monotonic_ns"], int) for event in events))
            for event in events:
                if event["type"] == "notification":
                    raw = base64.b64decode(event["payload_base64"], validate=True)
                    self.assertEqual(raw, PAYLOADS[event["channel"]])
                    self.assertEqual(len(raw), event["length"])
                    self.assertEqual(event["characteristic_uuid"], CHANNELS[event["channel"]])
            ready = json.loads(Path(result["ready_file"]).read_text())
            self.assertEqual(ready["all_channels_subscribed"], ["C", "D", "L"])
            self.assertEqual(ready["monotonic_ns"], result["ready_monotonic_ns"])
            self.assertLess(result["ready_monotonic_ns"], result["recording_end_monotonic_ns"])
            with self.assertRaises(FileExistsError):
                await self.invoke(options, mocked)
            self.assertEqual(Path(result["journal_file"]).read_bytes(), payload)

    async def test_not_connected_has_no_ready_or_zero_fps_claim(self):
        mocked, clients = backend(found=False)
        with tempfile.TemporaryDirectory() as directory:
            result = await self.invoke(self.options(directory), mocked)
            self.assertEqual(result["status"], "ERROR")
            self.assertIsNone(result["connected_monotonic_ns"])
            self.assertIsNone(result["ready_file"])
            self.assertEqual(result["notification_counts"], {})
            self.assertEqual(result["unconnected_windows"], "NOT_RUN_not_zero_FPS")
            self.assertEqual(len(clients), 0)

    async def test_connect_timeout_is_bounded_no_reconnect(self):
        mocked, clients = backend(fail_connect=True)
        with tempfile.TemporaryDirectory() as directory:
            result = await self.invoke(self.options(directory), mocked)
            self.assertEqual(result["status"], "ERROR")
            self.assertIsNone(result["connected_monotonic_ns"])
            self.assertFalse(result["reconnect_attempted"])
            self.assertEqual(len(clients), 1)
            self.assertLess(result["elapsed_seconds"], 0.14)

    async def test_runtime_disconnect_is_explicit_and_never_reconnected(self):
        mocked, clients = backend(disconnect=True)
        with tempfile.TemporaryDirectory() as directory:
            result = await self.invoke(self.options(directory), mocked)
            self.assertEqual(result["status"], "ERROR")
            self.assertEqual(result["stop_reason"], "runtime_disconnect")
            self.assertTrue(result["unexpected_disconnect"])
            self.assertEqual(len(clients), 1)
            events = [json.loads(line) for line in Path(result["journal_file"]).read_text().splitlines()]
            self.assertTrue(any(e["type"] == "disconnected" and e["unexpected"] for e in events))

    async def test_signal_file_stops_after_ready_and_preserves_summary(self):
        mocked, _ = backend()
        with tempfile.TemporaryDirectory() as directory:
            stop = Path(directory) / "root.stop"
            options = self.options(directory, duration=1, stop_file=stop)
            async def signal_when_ready():
                while not (Path(directory) / "synthetic.ble.ready.json").exists():
                    await asyncio.sleep(0.002)
                stop.touch()
            task = asyncio.create_task(signal_when_ready())
            result = await self.invoke(options, mocked)
            await task
            self.assertEqual(result["status"], "RECORDED", result)
            self.assertEqual(result["stop_reason"], "stop_file")
            self.assertLess(result["elapsed_seconds"], 1)
            summary = json.loads((Path(directory) / "synthetic.ble.summary.json").read_text())
            self.assertEqual(summary["journal_sha256"], result["journal_sha256"])

    async def test_missing_service_never_publishes_ready(self):
        mocked, clients = backend(service=False)
        with tempfile.TemporaryDirectory() as directory:
            result = await self.invoke(self.options(directory), mocked)
            self.assertEqual(result["status"], "ERROR")
            self.assertIsNone(result["ready_file"])
            self.assertEqual(clients[0].subscribed, [])

    async def test_cleanup_timeout_reported_without_reusing_connection(self):
        mocked, clients = backend(stall_cleanup=True)
        with tempfile.TemporaryDirectory() as directory:
            result = await self.invoke(self.options(directory), mocked)
            self.assertEqual(result["status"], "ERROR")
            self.assertEqual(len(result["cleanup_errors"]), 3)
            self.assertEqual(len(clients), 1)
            self.assertLess(result["elapsed_seconds"], 0.18)

    async def test_existing_stop_file_does_not_start_connection(self):
        mocked, clients = backend()
        with tempfile.TemporaryDirectory() as directory:
            stop = Path(directory) / "exists.stop"
            stop.touch()
            with self.assertRaises(FileExistsError):
                await self.invoke(self.options(directory, stop_file=stop), mocked)
            self.assertEqual(clients, [])


if __name__ == "__main__":
    unittest.main()
