#!/usr/bin/env python3
"""Exercise the real one-reader Collector against PTY synthetic firmware, not HIL."""
import contextlib
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import re
import tempfile
import time
import unittest
import zlib

import switch_and_query as switch
from test_firmware_benchmark_retry import QueryFirmware
from text_protocol import parse_fields


class ControlFirmware(QueryFirmware):
    """Only the fixture is synthetic; serial I/O, parser and runner are real."""
    def __init__(self, *, caps=True, control_busy=False, control_boot=False,
                 mutate_bundle=False, corrupt=None, stale_frame=False, watchdog=False, late_terminal=False, **kwargs):
        self.caps, self.control_busy, self.control_boot = caps, control_busy, control_boot
        self.mutate_bundle, self.corrupt = mutate_bundle, corrupt
        self.stale_frame, self.watchdog = stale_frame, watchdog
        self.last_frame_wire = None
        self.last_frame_revision = None
        self.wdt_sent = False
        self.late_terminal, self.previous_terminal = late_terminal, None
        super().__init__(**kwargs)

    def reply(self, command):
        if command == "RESTART?" and self.mutate_bundle:
            self.rev += 1
        super().reply(command)

    def send(self, value):
        command = self.current_query
        if value.startswith(("MAPP,", "RAPP,", "RMAPP,")) and self.late_terminal:
            if self.previous_terminal:
                super().send(self.previous_terminal)
            self.previous_terminal = value
        if command in switch.EXTRA_REQUIRED and value.startswith("ERR,"):
            fields = {key: "0" for key in switch.EXTRA_REQUIRED[command].split()}
            fields.update(session="AABBCCDDEE112233", bootId="2" if self.control_boot else "1",
                          busy="1" if self.control_busy else "0", phase="idle", last="none", reason="none",
                          physicalSafe="unknown", originalError="0x0", cleanupError="0x0",
                          publishError="0x0", serialError="0x0", netError="0x0")
            # The last result's rev=0 deliberately differs from CONFIG. It is
            # not authority for the currently sampled configuration.
            if command == self.truncate_query and self.query_counts[command] in self.truncate_numbers:
                value = command[:-1] + ",session=AABBCCDDEE112233\n"
            else:
                value = command[:-1] + "," + ",".join(f"{k}={fields[k]}" for k in switch.EXTRA_REQUIRED[command].split()) + "\n"
        if value.startswith("PROTO,") and self.caps:
            value = value.rstrip() + "/recovery_query_v1/restart_query_v1\n"
        if value.startswith(("C,", "V,", "R,", "M,")) and "\nK," in value:
            if self.stale_frame and self.last_frame_wire and self.last_frame_revision != self.rev:
                super().send(self.last_frame_wire)
            self.last_frame_wire, self.last_frame_revision = value, self.rev
            lines = value.rstrip().split("\n")
            if self.corrupt == "legacy":
                lines[0] = re.sub(r"(?<=,)gen=\d+", "gen=999", lines[0])
                lines[-1] = re.sub(r"(?<=,)gen=\d+", "gen=999", lines[-1])
            elif self.corrupt == "trailer_identity":
                lines[-1] = re.sub(r"(?<=,)rev=\d+", "rev=999", lines[-1])
            elif self.corrupt == "boot":
                lines[0] = lines[0].replace("bootId=1", "bootId=2")
            elif self.corrupt == "header_identity":
                lines[0] = re.sub(r"(?<=,)rev=\d+", "rev=999", lines[0])
            elif self.corrupt == "mixed_unit" and value.startswith("M,"):
                lines = [line.replace("unit=pF,", "unit=V,") if line.startswith("MR,") else line for line in lines]
            elif self.corrupt == "wrong_tag" and value.startswith("V,"):
                lines[0] = lines[0].replace("V,", "R,", 1).replace("mode=VOLT", "mode=RES").replace("unit=V,", "unit=ohm,")
            elif self.corrupt == "before_boundary":
                lines[0] = re.sub(r"(?<=,)seq=\d+", "seq=0", lines[0])
                lines[-1] = re.sub(r"(?<=,)seq=\d+", "seq=0", lines[-1])
            if self.corrupt:
                crc = zlib.crc32("".join(line + "\n" for line in lines[:-1]).encode("ascii"))
                if self.corrupt == "crc":
                    crc ^= 1
                lines[-1] = re.sub(r"crc=[0-9A-F]+", f"crc={crc:08X}", lines[-1])
                value = "\n".join(lines) + "\n"
            if self.watchdog and not self.wdt_sent and self.frames >= 5:
                super().send("E (12345) task_wdt: Task watchdog got triggered. IDLE0\n")
                self.wdt_sent = True
        super().send(value)


class SwitchAndQueryTests(unittest.TestCase):
    case_counter = 0

    @contextlib.contextmanager
    def directory(self):
        evidence = os.environ.get("SENSORARRAY_SWITCH_PTY_ARTIFACT_DIR")
        if not evidence:
            with tempfile.TemporaryDirectory() as directory:
                yield directory
            return
        SwitchAndQueryTests.case_counter += 1
        path = Path(evidence) / f"case-{self.case_counter:02d}-{self._testMethodName}"
        path.mkdir(parents=True, exist_ok=False)
        yield str(path)

    def invoke(self, device, directory, **changes):
        options = replace(switch.Options(device.path, Path(directory), "synthetic", duration=9,
                          prepare_seconds=1.2, cleanup_seconds=1.0, step_seconds=0.25,
                          command_timeout=0.045, progress_seconds=0.2, frames_per_step=1), **changes)
        with contextlib.redirect_stdout(io.StringIO()):
            result = switch.run(options, command=["PTY_SYNTHETIC_SELFTEST", "not-HIL"])
        events = [json.loads(line) for line in Path(result["events_file"]).read_text().splitlines()]
        return options, result, events

    def assert_restored(self, report, device):
        self.assertEqual(report["safe_cap_recovery"], "CAP8_QUERY_VERIFIED", report.get("error"))
        self.assertEqual(report["usb_restore"], "VERIFIED_QUERY", report.get("error"))
        self.assertEqual((device.mode, device.rows, device.profile), ("CAP", 8, "DEBUG"))

    def test_full_50_requests_domains_mixed_one_row_raw_and_exclusive_files(self):
        device = ControlFirmware(stale_ack=True, late_terminal=True)
        try:
            with self.directory() as directory:
                options, report, events = self.invoke(device, directory)
                self.assertEqual(report["status"], "CONTROL_IDENTITY_PASS", report.get("error"))
                self.assertEqual(report["completed_requests"], 50)
                self.assert_restored(report, device)
                self.assertEqual([r["command"] for r in report["results"]], [f"{k}={v}" for k, v in switch.request_plan()])
                self.assertEqual({int(v) for k, v in switch.request_plan() if k == "ROWS"}, set(range(1, 9)))
                first, second = report["results"][:2]
                self.assertEqual(first["command"], second["command"])
                self.assertNotEqual(first["transition"]["request_id"], second["transition"]["request_id"])
                mixed1 = [r for r in report["results"] if r["transition"]["config"]["mode"] == "MIXED" and r["transition"]["config"]["rows"] == "1"]
                self.assertTrue(mixed1)
                self.assertTrue(all(f["tag"] == "M" for r in mixed1 for f in r["frames"]))
                self.assertTrue(all(r["transition"]["verification"] == "ACK_TERMINAL_CONFIG" for r in report["results"]))
                self.assertTrue(any(e["type"] == "stale_ack_ignored" for e in events))
                for command in switch.EXTRA_REQUIRED:
                    self.assertEqual(device.commands.count(command), 2)
                self.assertFalse(any(re.match(r"(?:CAL|RAILCFG|FPS|RECOVER|RESTART)=|^(?:RECOVER|RESTART)$", cmd) for cmd in device.commands))
                raw = Path(report["raw_file"]).read_bytes()
                self.assertEqual(len(raw), report["rx_bytes"])
                self.assertEqual(hashlib.sha256(raw).hexdigest(), report["raw_sha256"])
                offset = 0
                for event in events:
                    if event["type"] == "rx":
                        self.assertEqual(event["offset"], offset)
                        offset += event["length"]
                self.assertEqual(offset, len(raw))
                self.assertEqual(report["provenance"]["command"], ["PTY_SYNTHETIC_SELFTEST", "not-HIL"])
                self.assertEqual(len(report["provenance"]["workspace_head"]), 40)
                self.assertEqual(len(report["provenance"]["dirty_hash"]), 64)
                with self.assertRaises(FileExistsError):
                    switch.run(options)
                self.assertEqual(Path(report["raw_file"]).read_bytes(), raw)
        finally:
            device.close()

    def test_all_lost_terminals_rebuilt_without_replaying_setters(self):
        device = ControlFirmware(lost_terminal=True)
        try:
            with self.directory() as directory:
                _, report, events = self.invoke(device, directory)
                self.assertEqual(report["status"], "CONTROL_IDENTITY_PASS", report.get("error"))
                self.assertTrue(all(t["verification"] == "ACK_CONFIG_RECOVERED_TERMINAL" for t in report["transitions_including_setup_cleanup"]))
                requests = [e for e in events if e["type"] == "tx_begin" and e["phase"] == "switch" and "=" in e["command"]]
                self.assertEqual(len(requests), 50)
                self.assert_restored(report, device)
        finally:
            device.close()

    def test_missing_ack_is_not_invented_or_retried(self):
        device = ControlFirmware(lost_ack=True)
        try:
            with self.directory() as directory:
                _, report, events = self.invoke(device, directory)
                self.assertEqual(report["status"], "ERROR")
                self.assertIn("transition_ack_unconfirmed", report["error"])
                self.assertEqual(report["completed_requests"], 0)
                for phase in ("prepare", "cleanup"):
                    self.assertEqual(len([e for e in events if e["type"] == "tx_begin" and e["phase"] == phase and e.get("command") == "MODE=CAP"]), 1)
                self.assertEqual(device.commands.count("MODE=CAP"), 2)  # explicit setup and final CAP attempt
                self.assertFalse(any("=" in q["command"] for q in report["query_attempts"]))
        finally:
            device.close()

    def test_readonly_queries_retry_missing_and_truncated_tail(self):
        for arguments in ({"drop_query": "RECOVERY?", "drop_numbers": {1, 2}},
                          {"truncate_query": "RESTART?", "truncate_numbers": {1, 2}}):
            device = ControlFirmware(**arguments)
            try:
                with self.directory() as directory:
                    _, report, _ = self.invoke(device, directory)
                    self.assertEqual(report["status"], "CONTROL_IDENTITY_PASS", report.get("error"))
                    command = arguments.get("drop_query") or arguments["truncate_query"]
                    attempts = [q for q in report["query_attempts"] if q["command"] == command and q["phase"] == "before"]
                    self.assertEqual(len(attempts), 3)
                    self.assertEqual(attempts[-1]["outcome"], "complete_reply")
                    self.assertEqual(len({q["phase_deadline"] for q in attempts}), 1)
            finally:
                device.close()

    def test_query_unknown_busy_boot_change_and_missing_capability_fail(self):
        cases = (({"caps": False}, "capability_missing"), ({"control_busy": True}, "external_control_pending"),
                 ({"control_boot": True}, "control_query_boot_mismatch"),
                 ({"drop_query": "RESTART?"}, "readonly_query_failed:RESTART?:attempts=3"))
        for args, reason in cases:
            device = ControlFirmware(**args)
            try:
                with self.directory() as directory:
                    _, report, _ = self.invoke(device, directory)
                    self.assertEqual(report["status"], "ERROR", args)
                    self.assertIn(reason, report["error"])
                    self.assertEqual(report["completed_requests"], 0)
                    self.assert_restored(report, device)
            finally:
                device.close()

    def test_revision_change_cannot_be_assembled_into_authority(self):
        device = ControlFirmware(mutate_bundle=True)
        try:
            with self.directory() as directory:
                _, report, _ = self.invoke(device, directory)
                self.assertEqual(report["status"], "ERROR")
                self.assertIn("control_query_bundle_configuration_changed", report["error"])
                self.assertEqual(len(report["state_bundles"]), 2)
                self.assertEqual(report["completed_requests"], 0)
                self.assert_restored(report, device)
        finally:
            device.close()

    def test_complete_crc_frames_do_not_hide_wrong_domains_or_units(self):
        cases = (("legacy", "legacy_frame_identity_domain_mismatch"),
                 ("trailer_identity", "current_configuration_frame_identity_or_units"),
                 ("header_identity", "frame_unknown_configuration"), ("boot", "frame_boot_changed"),
                 ("wrong_tag", "current_configuration_frame"), ("mixed_unit", "current_configuration_frame"),
                 ("crc", "current_configuration_frame_crc_or_format"), ("before_boundary", "frame_before_applied_boundary"))
        for corrupt, reason in cases:
            device = ControlFirmware(corrupt=corrupt)
            try:
                with self.directory() as directory:
                    _, report, _ = self.invoke(device, directory)
                    self.assertEqual(report["status"], "ERROR", corrupt)
                    self.assertIn(reason, report["error"], corrupt)
                    self.assertLess(report["completed_requests"], 50)
                    self.assert_restored(report, device)
            finally:
                device.close()

    def test_previous_configuration_data_keeps_identity_and_cannot_satisfy_new_request(self):
        device = ControlFirmware(stale_frame=True)
        try:
            with self.directory() as directory:
                _, report, events = self.invoke(device, directory)
                self.assertEqual(report["status"], "CONTROL_IDENTITY_PASS", report.get("error"))
                self.assertGreater(report["queued_previous_configuration_frames"], 0)
                self.assertTrue(any(e["type"] == "queued_previous_configuration" for e in events))
                for result in report["results"]:
                    expected = result["transition"]["config"]
                    self.assertTrue(all(switch.identity(f["identity"]) == switch.identity(expected) for f in result["frames"]))
        finally:
            device.close()

    def test_watchdog_fails_even_without_reset_and_cleanup_remains_attempted(self):
        device = ControlFirmware(watchdog=True)
        try:
            with self.directory() as directory:
                _, report, _ = self.invoke(device, directory)
                self.assertEqual(report["status"], "ERROR")
                self.assertIn("firmware_fault", report["error"])
                self.assertTrue(report["firmware_faults"])
                self.assertFalse(report["reset_observations"])
                self.assert_restored(report, device)
        finally:
            device.close()

    def test_phase_deadline_bounds_missing_reply_and_rail_is_never_written(self):
        for args, changes, reason in (({"drop_query": "BUILD?"}, {"prepare_seconds": 0.07}, "readonly_query_failed"),
                                     ({"rail": False}, {}, "existing_external_rail_required")):
            device = ControlFirmware(**args)
            try:
                with self.directory() as directory:
                    start = time.monotonic()
                    options, report, _ = self.invoke(device, directory, **changes)
                    self.assertLess(time.monotonic() - start, options.prepare_seconds + options.cleanup_seconds)
                    self.assertEqual(report["status"], "ERROR")
                    self.assertIn(reason, report["error"])
                    self.assertFalse(any(cmd.startswith(("RAILCFG=", "CAL", "FPS=")) for cmd in device.commands))
                    if "drop_query" in args:
                        attempts = [q for q in report["query_attempts"] if q["command"] == "BUILD?"]
                        self.assertEqual(len(attempts), 1)
                        self.assertTrue(all(q["started_monotonic"] < q["phase_deadline"] for q in attempts))
                    self.assert_restored(report, device)
            finally:
                device.close()

    def test_finite_counter_wrap_definition(self):
        self.assertTrue(switch.at_or_after32(0, 0xFFFFFFFF))
        self.assertTrue(switch.at_or_after32(1, 0xFFFFFFFE))
        self.assertFalse(switch.at_or_after32(0xFFFFFFFE, 1))
        self.assertFalse(switch.at_or_after32(0x80000000, 0))


if __name__ == "__main__":
    unittest.main()
