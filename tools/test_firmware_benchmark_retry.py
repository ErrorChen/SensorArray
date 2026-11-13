#!/usr/bin/env python3
"""Execute the separate retry runner on synthetic PTYs; never access hardware."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import time
import unittest

from run_firmware_benchmark_retry import REQUIRED, run_benchmark
import test_firmware_benchmark as benchmark_tests
from test_firmware_benchmark import Firmware
from text_protocol import parse_fields


class QueryFirmware(Firmware):
    def __init__(self, *, drop_query=None, drop_numbers=(), truncate_query=None, truncate_numbers=(),
                 change_config=False, **kwargs):
        self.query_counts = {}
        self.drop_query, self.drop_numbers = drop_query, set(drop_numbers)
        self.truncate_query, self.truncate_numbers = truncate_query, set(truncate_numbers)
        self.change_config = change_config
        self.current_query = None
        super().__init__(**kwargs)

    def reply(self, command):
        self.current_query = command if command.endswith("?") else None
        if self.current_query:
            count = self.query_counts[command] = self.query_counts.get(command, 0) + 1
            if command == self.drop_query and (not self.drop_numbers or count in self.drop_numbers):
                self.commands.append(command)
                return
            # A real authority revision change during the pre-capture snapshot
            # must remain a failure, even though both replies are complete.
            if self.change_config and command == "CONFIG?" and count == 5:
                self.rev += 1
        super().reply(command)

    def send(self, value):
        command = self.current_query
        expected_tag = {"RAIL?": "ARL", "BAT?": "ABAT"}.get(command, (command or "")[:-1])
        if command in REQUIRED and value.startswith(expected_tag + ","):
            fields = parse_fields(value.strip())
            defaults = {"project": "SensorArray", "wires": "ascii", "ctrlMax": "512", "dataMax": "2304",
                        "adcCrc": "crc8", "layout": "HOMOGENEOUS", "pending": "NONE", "stackUnit": "bytes",
                        "enumerated": "unknown", "primary": "present", "secondary": "present", "ads": "present",
                        "physicalSafe": "unknown", "scope": "all_candidates", "done": "1", "joined": "1", "restored": "1",
                        "gen": str(self.mgen), "rid": str(self.mrid)}
            for field in REQUIRED[command].split():
                fields.setdefault(field, defaults.get(field, "0"))
            if command == self.truncate_query and self.query_counts[command] in self.truncate_numbers:
                first = next(iter(fields))
                value = f"{expected_tag},{first}={fields[first]}\n"
            else:
                value = expected_tag + "," + ",".join(f"{key}={item}" for key, item in fields.items()) + "\n"
        super().send(value)


class QueryRetryTests(unittest.TestCase):
    def options(self, device, directory, **kwargs):
        return benchmark_tests.BenchmarkTests().options(device, directory, **kwargs)

    def invoke(self, device, options):
        with contextlib.redirect_stdout(io.StringIO()):
            index = run_benchmark(options)
        reports = [json.loads(Path(item["summary_file"]).read_text()) for item in index["results"]]
        return index, reports[0]

    def test_first_and_second_missing_readonly_reply_recover(self):
        for dropped in ({1}, {1, 2}):
            device = QueryFirmware(drop_query="PERF?", drop_numbers=dropped)
            try:
                with tempfile.TemporaryDirectory() as directory:
                    options = self.options(device, directory)
                    index, report = self.invoke(device, options)
                    self.assertEqual(index["status"], "COMPLETE", report)
                    attempts = [item for item in report["query_attempts"] if item["command"] == "PERF?" and item["phase"] == "before"]
                    self.assertEqual(len(attempts), len(dropped) + 1)
                    self.assertEqual(attempts[-1]["outcome"], "complete_reply")
                    self.assertGreaterEqual(report["capture_seconds"], options.capture_seconds)
                    self.assertFalse(report["setters_retried"])
            finally:
                device.close()

    def test_permanent_query_loss_stops_at_three_and_preserves_error(self):
        device = QueryFirmware(drop_query="SPI?")
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, report = self.invoke(device, self.options(device, directory))
                self.assertEqual(index["status"], "ERROR")
                self.assertIn("readonly_query_failed:SPI?:attempts=3", report["error"])
                self.assertEqual(report["capture_seconds"], 0)
                for phase in ("before", "after"):
                    attempts = [item for item in report["query_attempts"] if item["command"] == "SPI?" and item["phase"] == phase]
                    self.assertLessEqual(len(attempts), 3)
                self.assertEqual(report["safe_cap_recovery"], "CAP8_QUERY_VERIFIED")
        finally:
            device.close()

    def test_truncated_newline_terminated_reply_is_not_success(self):
        device = QueryFirmware(truncate_query="USBSTREAM?", truncate_numbers={1, 2})
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, report = self.invoke(device, self.options(device, directory))
                self.assertEqual(index["status"], "COMPLETE", report)
                attempts = [item for item in report["query_attempts"] if item["command"] == "USBSTREAM?" and item["phase"] == "prepare"]
                self.assertEqual(len(attempts), 3)
                self.assertIn("reply_missing_fields", attempts[0]["error"])
                raw = Path(report["raw_file"]).read_bytes()
                self.assertEqual(raw.count(b"USBSTREAM,v=DEBUG\n"), 2)
        finally:
            device.close()

    def test_retry_never_extends_original_phase_deadline(self):
        device = QueryFirmware(drop_query="BUILD?")
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory, prepare_seconds=0.07)
                start = time.monotonic()
                index, report = self.invoke(device, options)
                self.assertEqual(index["status"], "ERROR")
                attempts = [item for item in report["query_attempts"] if item["command"] == "BUILD?"]
                self.assertEqual(len(attempts), 1)
                self.assertTrue(all(item["started_monotonic"] < item["phase_deadline"] for item in attempts))
                self.assertEqual(report["capture_seconds"], 0)
                self.assertLess(time.monotonic() - start, options.prepare_seconds + options.cleanup_seconds)
        finally:
            device.close()

    def test_setter_ack_loss_never_replays_setter(self):
        device = QueryFirmware(lost_ack=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, report = self.invoke(device, self.options(device, directory))
                self.assertEqual(index["status"], "ERROR")
                self.assertIn("transition_ack_unconfirmed", report["error"])
                events = [json.loads(line) for line in Path(report["events_file"]).read_text().splitlines()]
                for phase in ("prepare", "cleanup"):
                    setters = [e for e in events if e["type"] == "tx_begin" and e["phase"] == phase and e.get("command") == "MODE=CAP"]
                    self.assertEqual(len(setters), 1)
                self.assertFalse(any("=" in a["command"] for a in report["query_attempts"]))
        finally:
            device.close()

    def test_config_authority_change_cannot_be_retried_into_false_success(self):
        device = QueryFirmware(change_config=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, report = self.invoke(device, self.options(device, directory))
                self.assertEqual(index["status"], "ERROR", report)
                self.assertIn("configuration_changed_before_capture", report["error"])
                self.assertEqual(report["capture_seconds"], 0)
        finally:
            device.close()

    def test_original_runner_source_not_modified(self):
        import hashlib
        import run_firmware_benchmark as original
        path = Path(original.__file__)
        before = path.read_bytes()
        constructor = original.BenchmarkCollector
        device = QueryFirmware()
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, report = self.invoke(device, self.options(device, directory))
                self.assertEqual(index["status"], "COMPLETE", report)
                self.assertEqual(report["base_runner_source_sha256"], hashlib.sha256(before).hexdigest())
                self.assertEqual(path.read_bytes(), before)
                self.assertIs(original.BenchmarkCollector, constructor)
        finally:
            device.close()


if __name__ == "__main__":
    unittest.main()
