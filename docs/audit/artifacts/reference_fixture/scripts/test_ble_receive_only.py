"""Synthetic mocks only: no BLE adapter, D-Bus or Serial access."""
import asyncio
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import zlib
import sys
sys.path.insert(0, "/tmp")
sys.path.insert(0, "/home/csc/esp/workspace/SensorArray/tools")
from sensorarray_ble_capture import Options, SERVICE, CHANNELS, capture
from test_reference_fixture import resistance

ADDRESS = "80:B5:4E:CD:D9:AE"


class Scanner:
    called = 0
    @classmethod
    async def find_device_by_address(cls, address, **kwargs):
        cls.called += 1
        assert address == ADDRESS
        return SimpleNamespace(address=ADDRESS, name="CscArray_CDD9AD")


class Client:
    writes = 0
    blocked = False
    corrupt_envelope = False
    def __init__(self, device, **kwargs):
        assert device.address == ADDRESS
        self.services = [SimpleNamespace(uuid=SERVICE)]
        self.mtu_size = 23
        self.callbacks = {}
        self.task = None
    async def connect(self):
        if self.blocked:
            await asyncio.sleep(10)
    async def write_gatt_char(self, *args, **kwargs):
        Client.writes += 1
        raise AssertionError("Application write forbidden")
    async def start_notify(self, uuid, callback):
        assert uuid in CHANNELS.values()
        self.callbacks[uuid] = callback
        if uuid == CHANNELS["L"]:
            self.task = asyncio.create_task(self.emit())
    async def emit(self):
        await asyncio.sleep(0.005)
        callback = self.callbacks[CHANNELS["D"]]
        for seq in (1, 3):
            payload = ("\n".join(resistance(rows=1, seq=seq, timestamp=seq * 1000)) + "\n").encode()
            crc = zlib.crc32(payload) & 0xFFFFFFFF
            if self.corrupt_envelope and seq == 3:
                crc ^= 1
            parts = [payload[i:i+180] for i in range(0, len(payload), 180)]
            for i, part in enumerate(parts):
                header = f"G,D,{seq},{i},{len(parts)},{len(part)},{len(payload)},{crc:08X}\n".encode()
                callback(None, bytearray(header + part))
                await asyncio.sleep(0.001)
        callback(None, bytearray(b"G,D,99,0,2,3,6,00000000\nabc"))
    async def stop_notify(self, uuid):
        pass
    async def disconnect(self):
        if self.task:
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)


class RecorderTests(unittest.IsolatedAsyncioTestCase):
    def options(self, root):
        return Options(ADDRESS, Path(root), "synthetic", duration=0.08,
                       scan_seconds=0.02, connect_seconds=0.02, operation_seconds=0.02,
                       cleanup_seconds=0.05)
    async def test_raw_reassembly_crc_gap_and_no_overwrite_or_application_write(self):
        with tempfile.TemporaryDirectory() as root:
            result = await capture(self.options(root), (Scanner, Client))
            self.assertEqual(result["status"], "CAPTURED", result)
            self.assertEqual(Client.writes, 0)
            self.assertFalse(result["serial_opened"])
            self.assertEqual(result["cccd_subscriptions"], ["C", "D", "L"])
            self.assertEqual(result["offline_protocol"]["parser_counters"]["resistance_frames"], 2)
            self.assertEqual(result["offline_protocol"]["groups"][0]["missing_sequences"], 1)
            self.assertEqual(result["fragment_stats"]["D"]["crc_fail"], 0)
            self.assertEqual(result["incomplete_envelopes"][0]["missing_count"], 1)
            notifications = [json.loads(line) for line in Path(root,"synthetic.notifications.jsonl").read_text().splitlines()]
            self.assertTrue(all(len(bytes.fromhex(row["raw_hex"])) == row["length"] for row in notifications))
            self.assertTrue(all(row["host_monotonic_ns"] > 0 for row in notifications))
            raw = Path(root,"synthetic.data.rx.bin").read_bytes()
            self.assertTrue(raw.startswith(b"R,seq=1,"))
            self.assertNotIn(b"G,D,", raw)
            called = Scanner.called
            with self.assertRaises(FileExistsError):
                await capture(self.options(root), (Scanner, Client))
            self.assertEqual(Scanner.called, called)
            self.assertEqual(Path(root,"synthetic.data.rx.bin").read_bytes(), raw)
    async def test_corrupt_envelope_retained_raw_but_not_scientific_data(self):
        class Corrupt(Client):
            corrupt_envelope = True
        with tempfile.TemporaryDirectory() as root:
            result = await capture(self.options(root), (Scanner, Corrupt))
            self.assertEqual(result["fragment_stats"]["D"]["crc_fail"], 1)
            self.assertEqual(result["offline_protocol"]["parser_counters"]["resistance_frames"], 1)
            self.assertIn('"raw_hex"', Path(root,"synthetic.notifications.jsonl").read_text())
    async def test_connection_timeout_and_wrong_address_fail_bounded(self):
        class Blocked(Client):
            blocked = True
        with tempfile.TemporaryDirectory() as root:
            result = await capture(self.options(root), (Scanner, Blocked))
            self.assertEqual(result["status"], "ERROR")
            self.assertLess(result["elapsed_seconds"], 0.3)
        class Wrong:
            @staticmethod
            async def find_device_by_address(*args, **kwargs):
                return SimpleNamespace(address="00:11:22:33:44:55", name="CscArray_CDD9AD")
        with tempfile.TemporaryDirectory() as root:
            result = await capture(self.options(root), (Wrong, Client))
            self.assertEqual(result["status"], "ERROR")
            self.assertIn("exact_address_not_found", result["error"])


if __name__ == "__main__":
    unittest.main()
