#!/usr/bin/env python3
"""Run the actual collector/PTY path; these synthetic cases are explicitly not HIL."""
import contextlib
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

import capture_usb_unread as unread
from test_switch_and_query import ControlFirmware


class UnreadFirmware(ControlFirmware):
    def __init__(self, *, pause_fault=None, scheduler=None, gap_after=False, crc_in_pause=False,
                 drop_serial=False, truncate_serial_after=False, change_snapshot_after=False, **kwargs):
        self.pause_fault = pause_fault
        self.pause_fault_sent = False
        self.pause_active = self.pause_done = False
        self.gap_after, self.gap_inserted = gap_after, False
        self.crc_in_pause, self.crc_inserted = crc_in_pause, False
        self.drop_serial, self.truncate_serial_after = drop_serial, truncate_serial_after
        self.change_snapshot_after = change_snapshot_after
        self.frame_generation = []
        self.after_serial_count = 0
        self.scheduler = {"initError": "0x0", "registered": "1", "idleKnown": "1", "tickHz": "100",
                          "budgetTicks": "2", "maxUnits": "32", "idleGen": "47", "idleAgeTicks": "0",
                          "maxIdleGapTicks": "1", **{worker: "12/1/4" for worker in unread.load.WORKERS}}
        self.scheduler.update(scheduler or {})
        super().__init__(**kwargs)

    def reply(self, command):
        if command not in unread.load.EXTRA_REQUIRED:
            return super().reply(command)
        self.current_query = command
        self.commands.append(command)
        self.query_counts[command] = self.query_counts.get(command, 0) + 1
        if command == "SERIAL?" and self.drop_serial:
            return
        if command == "SCHED?":
            if self.pause_done and self.change_snapshot_after:
                self.rev += 1
            fields = dict(self.scheduler)
        else:
            fields = {key: "0" for key in unread.load.EXTRA_REQUIRED[command].split()}
            fields.update(normalSubmitted=str(self.frames), driverTxBufferBytes="2304",
                          driverAcceptedBytes=str(len(self.sent)))
            if self.pause_done:
                self.after_serial_count += 1
                if self.truncate_serial_after and self.after_serial_count == 1:
                    fields.pop("partialWrite")
        self.send(command[:-1] + "," + ",".join(f"{k}={v}" for k, v in fields.items()) + "\n")

    def send(self, value):
        if value.startswith("PROTO,"):
            value = value.rstrip() + "/serial_sink_v1/work_budget_v1\n"
        super().send(value)

    def frame(self):
        if self.pause_done and self.gap_after and not self.gap_inserted:
            self.frames += 7
            self.gap_inserted = True
        if self.pause_active and self.pause_fault and not self.pause_fault_sent:
            line = {"MFAULT": "MFAULT,seq=99,err=0x108,synthetic=1\n",
                    "WDT": "E (12345) task_wdt: Task watchdog got triggered. IDLE0 synthetic=1\n",
                    "RESET": "ESP-ROM:esp32s3-20210327 synthetic=1\n"}[self.pause_fault]
            self.send(line)
            self.pause_fault_sent = True
        previous = self.corrupt
        if self.pause_active and self.crc_in_pause and not self.crc_inserted:
            self.corrupt = "crc"
            self.crc_inserted = True
        begin = time.monotonic()
        before = len(self.sent)
        super().frame()
        self.frame_generation.append({"monotonic": begin, "sequence": self.frames,
                                      "bytes_accepted_by_pty": len(self.sent) - before,
                                      "during_application_unread": self.pause_active})
        self.corrupt = previous


class UnreadTests(unittest.TestCase):
    case_counter = 0

    @contextlib.contextmanager
    def directory(self):
        evidence = os.environ.get("SENSORARRAY_UNREAD_PTY_ARTIFACT_DIR")
        if not evidence:
            with tempfile.TemporaryDirectory() as path:
                yield path
            return
        UnreadTests.case_counter += 1
        path = Path(evidence) / f"case-{self.case_counter:02d}-{self._testMethodName}"
        path.mkdir(parents=True, exist_ok=False)
        yield str(path)

    def invoke(self, device, directory, **changes):
        options = replace(unread.Options(device.path, Path(directory), "synthetic", duration=2.5,
                          prepare_seconds=0.6, warmup_seconds=0.025, before_seconds=0.055,
                          unread_seconds=0.09, after_seconds=0.065, query_seconds=0.4,
                          cleanup_seconds=0.45, command_timeout=0.045, progress_seconds=0.2), **changes)
        observations = {"opened": [], "closed": [], "read_calls": [], "commands_during_pause": []}
        original_collector, original_read = unread.UnreadCollector, os.read

        class TrackedPort(unread.PosixPort):
            def __init__(self, path):
                super().__init__(path)
                observations["opened"].append({"fd": self.fd, "time": time.monotonic()})

            def close(self):
                observations["closed"].append({"fd": self.fd, "time": time.monotonic()})
                super().close()

        class HookCollector(original_collector):
            def event(self, kind, **fields):
                if kind == "application_unread_start":
                    device.pause_active = True
                elif kind == "application_unread_end":
                    device.pause_active, device.pause_done = False, True
                return super().event(kind, **fields)

            def send(self, command, deadline):
                if self.read_suspended:
                    observations["commands_during_pause"].append(command)
                return super().send(command, deadline)

        def tracked_read(fd, length):
            if any(item["fd"] == fd for item in observations["opened"]):
                observations["read_calls"].append({"fd": fd, "monotonic": time.monotonic()})
            return original_read(fd, length)

        with mock.patch.object(unread, "UnreadCollector", HookCollector), mock.patch.object(os, "read", tracked_read):
            with contextlib.redirect_stdout(io.StringIO()):
                report = unread.run(options, port_factory=TrackedPort, command=["PTY_SYNTHETIC_SELFTEST", "not-HIL"])
        events = [json.loads(line) for line in Path(report["events_file"]).read_text().splitlines()]
        observations["synthetic_frame_generation"] = list(device.frame_generation)
        Path(directory, "synthetic-host-observations.json").write_text(json.dumps(observations, indent=2) + "\n")
        return options, report, events, observations

    def assert_cleanup(self, report, device):
        self.assertEqual(report["safe_cap_recovery"], "CAP8_QUERY_VERIFIED", report.get("error"))
        self.assertEqual(report["usb_restore"], "VERIFIED_QUERY", report.get("error"))
        self.assertEqual((device.mode, device.rows, device.profile), ("CAP", 8, "DEBUG"))

    def test_one_open_fd_true_no_read_window_and_buffered_device_frames(self):
        device = UnreadFirmware()
        try:
            with self.directory() as directory:
                options, report, events, observed = self.invoke(device, directory)
                self.assertEqual(report["status"], "CONTROL_RECOVERY_PASS", report.get("error"))
                self.assert_cleanup(report, device)
                self.assertEqual(len(observed["opened"]), 1)
                self.assertEqual(len(observed["closed"]), 1)
                self.assertFalse(observed["commands_during_pause"])
                pause = report["unread_window"]
                self.assertTrue(pause["fd_unchanged"])
                self.assertGreaterEqual(pause["actual_seconds"], options.unread_seconds)
                self.assertEqual(pause["raw_offset_start"], pause["raw_offset_end"])
                self.assertFalse(any(pause["started_monotonic"] <= call["monotonic"] <= pause["finished_monotonic"]
                                     for call in observed["read_calls"]))
                generated = [frame for frame in observed["synthetic_frame_generation"] if frame["during_application_unread"]]
                self.assertGreater(len(generated), 3)
                self.assertTrue(all(frame["bytes_accepted_by_pty"] > 0 for frame in generated))
                after = report["receive_windows"]["receive_after"]
                self.assertGreater(after["crc_valid_identity_frames"], 0)
                generated_sequences = {frame["sequence"] for frame in generated}
                self.assertTrue(any(frame["sequence"] in generated_sequences for frame in after["first_frame_examples"]))
                self.assertEqual(after["frame_generation_wall_clock"], "UNKNOWN")
                self.assertFalse(report["absent_USB_host_tested"])
                self.assertFalse(report["physical_unplug"])
                self.assertFalse(report["BLE_tested_by_this_process"])
                for name, seconds in (("warmup", options.warmup_seconds), ("receive_before", options.before_seconds), ("receive_after", options.after_seconds)):
                    self.assertGreaterEqual(report["receive_windows"][name]["actual_seconds"], seconds)
                for name in ("before", "after"):
                    snapshot = report["snapshots"][name]
                    self.assertTrue(all(command in snapshot for command in ("CONFIG?", "SERIAL?", "SCHED?", "FRONTENDS?", "RECOVERY?", "CONFIG?_closing")))
                    self.assertTrue(unread.control.same_config(snapshot["CONFIG?"]["fields"], snapshot["CONFIG?_closing"]["fields"]))
                raw = Path(report["raw_file"]).read_bytes()
                self.assertEqual(hashlib.sha256(raw).hexdigest(), report["raw_sha256"])
                self.assertEqual(len(raw), report["rx_bytes"])
                offset = 0
                for event in events:
                    if event["type"] == "rx":
                        self.assertEqual(event["offset"], offset)
                        offset += event["length"]
                self.assertEqual(offset, len(raw))
                self.assertFalse(any(cmd.startswith(("FPS=", "CAL", "RAILCFG", "RECOVER=", "RESTART")) for cmd in device.commands))
                with self.assertRaises(FileExistsError):
                    unread.run(options)
                self.assertEqual(Path(report["raw_file"]).read_bytes(), raw)
        finally:
            device.close()

    def test_unread_fault_is_detected_on_resume_and_stays_failure_after_cleanup(self):
        for fault in ("MFAULT", "WDT", "RESET"):
            device = UnreadFirmware(pause_fault=fault)
            try:
                with self.directory() as directory:
                    options, report, _, _ = self.invoke(device, directory)
                    self.assertEqual(report["status"], "ERROR", fault)
                    self.assertFalse(report["resume_control_verified"])
                    self.assertGreaterEqual(report["unread_window"]["actual_seconds"], options.unread_seconds)
                    self.assertLess(report["receive_windows"]["receive_after"]["actual_seconds"], options.after_seconds)
                    if fault == "RESET":
                        self.assertTrue(report["reset_observations"])
                    else:
                        self.assertTrue(report["firmware_faults"])
                    self.assert_cleanup(report, device)
            finally:
                device.close()

    def test_fault_while_reading_is_fail_fast_before_unread(self):
        device = UnreadFirmware(fault=True)
        try:
            with self.directory() as directory:
                _, report, _, _ = self.invoke(device, directory, before_seconds=0.15)
                self.assertEqual(report["status"], "ERROR")
                self.assertIsNone(report["unread_window"])
                self.assertIn("firmware_fault", report["error"])
                self.assert_cleanup(report, device)
        finally:
            device.close()

    def test_data_gaps_and_rejected_crc_records_remain_visible_not_lossless_pass(self):
        device = UnreadFirmware(gap_after=True, crc_in_pause=True)
        try:
            with self.directory() as directory:
                _, report, _, _ = self.invoke(device, directory)
                self.assertEqual(report["status"], "CONTROL_RECOVERY_PASS", report.get("error"))
                after = report["receive_windows"]["receive_after"]
                self.assertGreaterEqual(after["missing_sequences"], 7)
                self.assertGreaterEqual(after["crc_rejected_records"], 1)
                self.assertGreater(after["crc_valid_identity_frames"], 0)
                self.assertIn("not required to be zero", report["sequence_gap_policy"])
                self.assertEqual(report["measurement_acceptance"], "NOT_EVALUATED")
        finally:
            device.close()

    def test_query_retries_share_deadline_and_snapshot_revision_change_is_rejected(self):
        for change in (False, True):
            device = UnreadFirmware(truncate_serial_after=not change, change_snapshot_after=change)
            try:
                with self.directory() as directory:
                    _, report, _, _ = self.invoke(device, directory)
                    attempts = [a for a in report["query_attempts"] if a["command"] == "SERIAL?" and a["phase"] == "after"]
                    if change:
                        self.assertEqual(report["status"], "ERROR")
                        self.assertIn("unread_snapshot_configuration_changed", report["error"])
                        self.assertEqual(len([a for a in report["snapshot_attempts"] if a["name"] == "after"]), 2)
                    else:
                        self.assertEqual(report["status"], "CONTROL_RECOVERY_PASS", report.get("error"))
                        self.assertEqual(len(attempts), 2)
                        self.assertIn("reply_missing_fields", attempts[0]["error"])
                        self.assertEqual(len({a["phase_deadline"] for a in attempts}), 1)
                    self.assert_cleanup(report, device)
            finally:
                device.close()

    def test_unknown_scheduler_capped_fps_or_missing_rail_are_not_changed_to_pass(self):
        cases = (({"scheduler": {"registered": "0"}}, "scheduler_evidence_unavailable"),
                 ({"fps_cap": 5}, "uncapped_FPS_required_no_FPS_written"),
                 ({"rail": False}, "existing_external_rail_required_no_calibration_written"))
        for arguments, reason in cases:
            device = UnreadFirmware(**arguments)
            try:
                with self.directory() as directory:
                    _, report, _, _ = self.invoke(device, directory)
                    self.assertEqual(report["status"], "ERROR")
                    self.assertIn(reason, report["error"])
                    self.assertIsNone(report["unread_window"])
                    self.assertFalse(any(cmd.startswith(("FPS=", "RAILCFG=", "CAL")) for cmd in device.commands))
                    self.assert_cleanup(report, device)
            finally:
                device.close()

    def test_phase_deadline_and_total_ceiling_are_real(self):
        device = UnreadFirmware(drop_serial=True)
        try:
            with self.directory() as directory:
                start = time.monotonic()
                options, report, _, _ = self.invoke(device, directory, prepare_seconds=0.08, command_timeout=0.3)
                self.assertEqual(report["status"], "ERROR")
                attempts = [a for a in report["query_attempts"] if a["command"] == "SERIAL?"]
                self.assertEqual(len(attempts), 1)
                self.assertTrue(attempts[0]["started_monotonic"] < attempts[0]["phase_deadline"])
                self.assertLess(time.monotonic() - start, options.prepare_seconds + options.cleanup_seconds + 0.1)
                self.assert_cleanup(report, device)
                for changes in ({"duration": 121}, {"unread_seconds": 3}, {"unread_seconds": float("inf")}):
                    with self.assertRaises(ValueError):
                        unread.run(replace(options, label="must-not-open", **changes))
        finally:
            device.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
