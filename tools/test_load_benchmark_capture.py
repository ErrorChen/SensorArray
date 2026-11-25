#!/usr/bin/env python3
"""Exercise the load capture entry point on synthetic PTYs; no hardware."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import time
import unittest

import run_firmware_benchmark as base
import run_firmware_benchmark_retry as retry
import test_firmware_benchmark as benchmark_tests
from test_firmware_benchmark_retry import QueryFirmware

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/hil/capture_load_benchmark.py"
spec = importlib.util.spec_from_file_location("capture_load_benchmark", SCRIPT)
load = importlib.util.module_from_spec(spec)
spec.loader.exec_module(load)


class LoadFirmware(QueryFirmware):
    def __init__(self, *, scheduler=None, missing_caps=(), truncate_diagnostic=None,
                 truncate_attempts=(), drop_diagnostic=None, change_during_extra=False,
                 watchdog=False, **kwargs):
        self.scheduler = {"initError": "0x0", "registered": "1", "idleKnown": "1", "tickHz": "100",
                          "budgetTicks": "2", "maxUnits": "32", "idleGen": "47",
                          "idleAgeTicks": "0", "maxIdleGapTicks": "1",
                          **{worker: "12/1/4" for worker in load.WORKERS}}
        self.scheduler.update(scheduler or {})
        self.missing_caps = set(missing_caps)
        self.truncate_diagnostic = truncate_diagnostic
        self.truncate_attempts = set(truncate_attempts)
        self.drop_diagnostic = drop_diagnostic
        self.change_during_extra = change_during_extra
        self.watchdog = watchdog
        super().__init__(**kwargs)

    def reply(self, command):
        if command not in load.EXTRA_REQUIRED:
            return super().reply(command)
        self.current_query = command
        self.commands.append(command)
        count = self.query_counts[command] = self.query_counts.get(command, 0) + 1
        if command == self.drop_diagnostic:
            return
        if command == "SCHED?":
            if self.change_during_extra:
                self.rev += 1
            fields = dict(self.scheduler)
        else:
            fields = {key: "0" for key in load.EXTRA_REQUIRED[command].split()}
            fields.update(normalSubmitted=str(self.frames), driverTxBufferBytes="2304",
                          driverAcceptedBytes=str(len(self.sent)))
        if command == self.truncate_diagnostic and count in self.truncate_attempts:
            fields.pop(next(reversed(fields)))
        self.send(command[:-1] + "," + ",".join(f"{key}={value}" for key, value in fields.items()) + "\n")

    def send(self, value):
        if value.startswith("PROTO,"):
            caps = {"config_query_v1", "config_identity_v1", "serial_sink_v1", "work_budget_v1"} - self.missing_caps
            # A substring lookalike is deliberately not a supported capability.
            caps.update(cap + "0" for cap in self.missing_caps)
            value = "PROTO,version=2,caps=" + "/".join(sorted(caps)) + "\n"
        if self.watchdog and value.startswith("MFAULT,"):
            value = "E (12345) task_wdt: Task watchdog got triggered. synthetic=1\n"
        super().send(value)


class LoadCaptureTests(unittest.TestCase):
    def invoke(self, device, directory, **kwargs):
        options = benchmark_tests.BenchmarkTests().options(device, directory, **kwargs)
        with contextlib.redirect_stdout(io.StringIO()):
            result = load.run_benchmark(options)
        report = json.loads(Path(result["results"][0]["summary_file"]).read_text())
        return options, result, report

    def test_full_seven_worker_schema_and_query_windows(self):
        device = LoadFirmware()
        try:
            with tempfile.TemporaryDirectory() as directory:
                options, result, report = self.invoke(device, directory)
                self.assertEqual(result["status"], "COMPLETE", report)
                self.assertTrue(report["diagnostic_observation_accepted"])
                self.assertEqual(report["scheduler_workers"], ["bt", "tr", "log", "ser", "bc", "sc", "wc"])
                self.assertGreaterEqual(report["capture_seconds"], options.capture_seconds)
                self.assertGreaterEqual(report["warmup_seconds"], options.warmup_seconds)
                for name in ("before", "after"):
                    window = report["snapshots"][name]
                    self.assertEqual(window["SERIAL?"]["fields"]["driverTxBufferBytes"], "2304")
                    self.assertTrue(report["scheduler_health"][name]["usable"])
                    for command in ("CONFIG?", "SERIAL?", "SCHED?", "CONFIG?_closing"):
                        self.assertLessEqual(window[command]["started_monotonic"], window[command]["finished_monotonic"])
                    self.assertLessEqual(window["CONFIG?"]["finished_monotonic"], window["SERIAL?"]["started_monotonic"])
                    self.assertLessEqual(window["SERIAL?"]["finished_monotonic"], window["SCHED?"]["started_monotonic"])
                    self.assertLessEqual(window["SCHED?"]["finished_monotonic"], window["CONFIG?_closing"]["started_monotonic"])
                    for key in load.CONFIG_CONSISTENCY:
                        self.assertEqual(window["CONFIG?"]["fields"][key], window["CONFIG?_closing"]["fields"][key])
                self.assertLessEqual(report["snapshots"]["before"]["CONFIG?_closing"]["finished_monotonic"], report["capture_monotonic_start"])
                self.assertLessEqual(report["capture_monotonic_end"], report["snapshots"]["after"]["CONFIG?"]["started_monotonic"])
                self.assertTrue(all(cmd.split("=", 1)[0] in {"MODE", "ROWS", "ROWMODES", "USBSTREAM"}
                                    for cmd in device.commands if "=" in cmd))
                self.assertFalse(report["setters_retried"])
                self.assertIn("not CPU utilization", report["scheduler_counter_semantics"])
        finally:
            device.close()

    def test_unknown_unregistered_and_error_are_recorded_and_refused(self):
        cases = [({"idleKnown": "0", "idleGen": "0", "idleAgeTicks": "unknown", "maxIdleGapTicks": "unknown"}, "idle_progress_unknown"),
                 ({"registered": "0"}, "hook_unregistered"),
                 ({"initError": "0x105"}, "hook_init_error"),
                 ({"idleAgeTicks": "4294967295"}, "idleAgeTicks_unknown")]
        for fields, reason in cases:
            with self.subTest(fields=fields):
                device = LoadFirmware(scheduler=fields)
                try:
                    with tempfile.TemporaryDirectory() as directory:
                        _, result, report = self.invoke(device, directory)
                        self.assertEqual(result["status"], "ERROR")
                        self.assertIn(reason, report["error"])
                        self.assertEqual(report["capture_seconds"], 0)
                        self.assertFalse(report["diagnostic_observation_accepted"])
                        for key, value in fields.items():
                            self.assertEqual(report["snapshots"]["before"]["SCHED?"]["fields"][key], value)
                        attempts = [a for a in report["query_attempts"] if a["command"] == "SCHED?" and a["phase"] == "before"]
                        self.assertEqual(len(attempts), 1)  # complete unavailable state is not a lost reply
                finally:
                    device.close()

    def test_missing_capability_exact_token_rejected_before_diagnostics(self):
        for capability in ("serial_sink_v1", "work_budget_v1"):
            device = LoadFirmware(missing_caps={capability})
            try:
                with tempfile.TemporaryDirectory() as directory:
                    _, result, report = self.invoke(device, directory)
                    self.assertEqual(result["status"], "ERROR")
                    self.assertIn("load_diagnostic_capability_required:" + capability, report["error"])
                    self.assertFalse(report["diagnostic_capabilities_verified"])
                    self.assertFalse(any(cmd in load.EXTRA_REQUIRED for cmd in device.commands))
                    self.assertEqual(report["capture_seconds"], 0)
            finally:
                device.close()

    def test_truncated_diagnostic_reply_retries_with_original_deadline(self):
        for command in load.EXTRA_REQUIRED:
            device = LoadFirmware(truncate_diagnostic=command, truncate_attempts={1, 2})
            try:
                with tempfile.TemporaryDirectory() as directory:
                    _, result, report = self.invoke(device, directory)
                    self.assertEqual(result["status"], "COMPLETE", report)
                    attempts = [a for a in report["query_attempts"] if a["command"] == command and a["phase"] == "before"]
                    self.assertEqual(len(attempts), 3)
                    self.assertIn("reply_missing_fields", attempts[0]["error"])
                    self.assertEqual(len({a["phase_deadline"] for a in attempts}), 1)
                    self.assertEqual(attempts[-1]["outcome"], "complete_reply")
                    self.assertFalse(any("=" in a["command"] for a in report["query_attempts"]))
            finally:
                device.close()

    def test_new_query_cannot_extend_phase_deadline(self):
        device = LoadFirmware(drop_diagnostic="SERIAL?")
        try:
            with tempfile.TemporaryDirectory() as directory:
                started = time.monotonic()
                options, result, report = self.invoke(device, directory, prepare_seconds=0.12, warmup_seconds=0.005, command_timeout=0.4)
                self.assertEqual(result["status"], "ERROR")
                attempts = [a for a in report["query_attempts"] if a["command"] == "SERIAL?" and a["phase"] == "before"]
                self.assertEqual(len(attempts), 1)
                self.assertLess(attempts[0]["started_monotonic"], attempts[0]["phase_deadline"])
                self.assertLess(time.monotonic() - started, options.prepare_seconds + options.cleanup_seconds + 0.3)
                self.assertEqual(report["capture_seconds"], 0)
        finally:
            device.close()

    def test_revision_change_during_new_queries_cannot_form_authority(self):
        device = LoadFirmware(change_during_extra=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                _, result, report = self.invoke(device, directory)
                self.assertEqual(result["status"], "ERROR")
                self.assertIn("snapshot_window_configuration_changed:before", report["error"])
                self.assertEqual(report["capture_seconds"], 0)
                attempts = [a for a in report["diagnostic_snapshot_attempts"] if a["name"] == "before"]
                self.assertEqual(len(attempts), 2)
                for attempt in attempts:
                    self.assertEqual(attempt["outcome"], "configuration_changed")
                    self.assertNotEqual(attempt["queries"]["CONFIG?"]["fields"]["rev"],
                                        attempt["queries"]["CONFIG?_closing"]["fields"]["rev"])
        finally:
            device.close()

    def test_watchdog_remains_fail_fast(self):
        device = LoadFirmware(fault=True, watchdog=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                options, result, report = self.invoke(device, directory, capture_seconds=0.25)
                self.assertEqual(result["status"], "ERROR")
                self.assertIn("firmware_fault", report["error"])
                self.assertLess(report["capture_seconds"], options.capture_seconds)
                self.assertTrue(any("Task watchdog got triggered" in line for line in report["firmware_faults"]))
                self.assertFalse(report["diagnostic_observation_accepted"])
        finally:
            device.close()

    def test_default_windows_frozen_sources_and_process_context_restored(self):
        defaults = base.BenchmarkOptions("not_opened", Path("not_created"), "synthetic")
        self.assertEqual((defaults.capture_seconds, defaults.stability_seconds, defaults.warmup_seconds), (120, 600, 5))
        paths = [Path(base.__file__), Path(retry.__file__)]
        before = {p: p.read_bytes() for p in paths}
        previous = base.BenchmarkCollector, retry.REQUIRED, retry.validate_reply
        device = LoadFirmware()
        try:
            with tempfile.TemporaryDirectory() as directory:
                _, result, report = self.invoke(device, directory)
                self.assertEqual(result["status"], "COMPLETE", report)
                self.assertEqual(report["base_runner_source_sha256"], hashlib.sha256(before[paths[0]]).hexdigest())
                self.assertEqual(report["retry_runner_source_sha256"], hashlib.sha256(before[paths[1]]).hexdigest())
        finally:
            device.close()
        for path in paths:
            self.assertEqual(path.read_bytes(), before[path])
        self.assertIs(base.BenchmarkCollector, previous[0])
        self.assertIs(retry.REQUIRED, previous[1])
        self.assertIs(retry.validate_reply, previous[2])


if __name__ == "__main__":
    unittest.main(verbosity=2)
