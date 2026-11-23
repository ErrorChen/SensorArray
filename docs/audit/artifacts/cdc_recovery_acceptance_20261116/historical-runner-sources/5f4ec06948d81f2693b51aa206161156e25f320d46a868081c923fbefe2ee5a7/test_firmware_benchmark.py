#!/usr/bin/env python3
"""Exercise the real runner/one-reader POSIX path against synthetic PTYs only."""
import contextlib
from dataclasses import replace
import errno
import hashlib
import io
import json
import os
from pathlib import Path
import re
import select
import tempfile
import time
import unittest
import zlib

from capture_reference_fixture import PosixPort
from run_firmware_benchmark import BenchmarkOptions, configurations, run_benchmark
from test_capture_reference_fixture import Device
from test_reference_fixture import capacitance
from test_text_protocol import measurement_lines, mixed_lines
from text_protocol import parse_fields


class Firmware(Device):
    def __init__(self, *, lost_terminal=False, lost_ack=False, rail=True, fault=False,
                 wrong_identity=False, wrong_mode=False, wrong_unit=False, fps_cap=0, stale_ack=False):
        self.next_id, self.rev = 10, 1
        self.mrid = self.rrid = self.prid = 1
        self.mgen = self.rgen = self.pgen = 1
        self.lost_terminal, self.lost_ack = lost_terminal, lost_ack
        self.rail, self.fault, self.wrong_identity = rail, fault, wrong_identity
        self.wrong_mode, self.wrong_unit, self.fps_cap, self.stale_ack = wrong_mode, wrong_unit, fps_cap, stale_ack
        self.fault_sent = False
        self.full_since, self.tail_due = None, None
        self.frames = 0
        super().__init__(early_terminal=True)

    def config(self):
        mode = "MIXED" if len(set(self.rowmodes)) > 1 else self.mode
        return (f"CONFIG,session=AABBCCDDEE112233,bootId=1,rev={self.rev},boundary=1,state=CAPACITANCE,"
                f"mode={mode},rows={self.rows},profile={self.rowmodes},err=0x0,"
                f"mgen={self.mgen},mrid={self.mrid},rgen={self.rgen},rrid={self.rrid},pgen={self.pgen},prid={self.prid},"
                f"mtermId={self.mrid},mterm=applied,merr=0x0,ptermId={self.prid},pterm=applied,perr=0x0,"
                f"rtermId={self.rrid},rterm=applied,rerr=0x0")

    def reply(self, command):
        self.commands.append(command)
        if "=" in command:
            kind, value = command.split("=", 1)
            if kind == "USBSTREAM":
                self.profile = value
                self.full_since = time.monotonic() if value == "FULL" else None
                self.send(f"USBSTREAM,v={value},dataEvery={1 if value == 'FULL' else 10},diagEvery={1 if value == 'FULL' else 50},state=applied\n")
                return
            self.next_id += 1
            request_id = self.next_id
            self.rev += 1
            if kind == "MODE":
                if self.stale_ack:
                    self.send(f"MACK,id={self.mrid},new={value},state=accepted\n")
                self.mode = value
                self.rowmodes = {"CAP": "C", "VOLT": "V", "RES": "R"}[value] * 8
                self.mrid = self.prid = request_id
                self.mgen += 1
                self.pgen += 1
                ack = f"MACK,id={request_id},new={value},state=accepted"
                terminal = f"MAPP,id={request_id},new={value},state=applied"
            elif kind == "ROWS":
                self.rows = int(value)
                self.rrid, self.rgen = request_id, self.rgen + 1
                ack = f"RCMD,id={request_id},req={value},status=accepted"
                terminal = f"RAPP,id={request_id},new={value},status=applied"
            elif kind == "ROWMODES":
                self.rowmodes = value
                self.mode = {"C": "CAP", "V": "VOLT", "R": "RES"}[value[0]]
                self.prid, self.pgen = request_id, self.pgen + 1
                ack = f"RMACK,id={request_id},new={value},state=accepted"
                terminal = f"RMAPP,id={request_id},profile={value},state=applied"
            elif kind == "RAILCFG":
                self.rail = True
                avdd, avss = value.split(",")
                ack = f"RACK,id={request_id},avdd={avdd},avss={avss},source=external,state=accepted"
                terminal = f"RAPP,id={request_id},avdd={avdd},avss={avss},source=external,state=applied"
            else:
                self.send("ERR,reason=unsupported_mutation\n")
                return
            # Real ordering counterexample: terminal can precede its ACK.
            if not self.lost_terminal:
                self.send(terminal + "\n")
            if not self.lost_ack:
                self.send(ack + "\n")
            return
        replies = {
            "BUILD?": "BUILD,idf=synthetic,target=pty,proto=2",
            "PROTO?": "PROTO,version=2,caps=config_query_v1/config_identity_v1",
            "CAL?": "CAL,valid=1,source=0",
            "CONFIG?": self.config(),
            "MODE?": f"MODE,state=CAPACITANCE,active={self.mode},intref=0,refmux=24,profile={self.rowmodes},route={self.mode}",
            "RAIL?": "ARL,rv=1,rs=ok,src=external,split=external_measured,rail=5145000,avdd=3350000,avss=-1795000" if self.rail else "ARL,rv=0,rs=rail_invalid,src=unknown",
            "PERF?": f"PERF,pub={self.frames},fresh={self.frames},frames={self.frames},dropOut=0,heap=8000000,heapMin=7990000",
            "USBSTREAM?": f"USBSTREAM,v={self.profile},dataEvery={1 if self.profile == 'FULL' else 10},diagEvery={1 if self.profile == 'FULL' else 50},sinkDropped=0,driverAcceptedBytes={len(self.sent)}",
            "FPS?": f"FPS,cfcap={self.fps_cap},ofcap=0,adsgap=on",
            "FRONTENDS?": "FRONTENDS,phase=monitor,hold=0,known=7,present=7,quarantine=0",
            "SPI?": "SPI,phase=done,qualified=1,hz=900000,actualHz=909000",
            "BAT?": "ABAT,valid=0,reason=unstable,restore=ok",
        }
        self.send(replies.get(command, "ERR,reason=unsupported_query") + "\n")

    def frame(self):
        self.frames += 1
        seq, rows = self.frames, self.rows
        if self.wrong_mode:
            lines = capacitance(rows, seq, int(time.monotonic() * 1e6))[:-1]
        elif len(set(self.rowmodes)) > 1:
            lines = mixed_lines(seq, rows, self.rowmodes[:rows] + "N" * (8 - rows))[:-1]
            if self.wrong_unit:
                lines = [line.replace("unit=V,", "unit=ohm,") if line.startswith("MR,s=2,") else line for line in lines]
        elif self.mode == "CAP":
            lines = capacitance(rows, seq, int(time.monotonic() * 1e6))[:-1]
        else:
            lines = measurement_lines("V" if self.mode == "VOLT" else "R", seq, rows,
                                      ["123456"] * (rows * 8), [1] * (rows * 8))[:-1]
        tag, fields = lines[0].split(",", 1)[0], parse_fields(lines[0])
        fields.update(session="AABBCCDDEE112233", bootId="1", rev=str(self.rev),
                      mgen=str(self.mgen), mrid=str(self.mrid), rgen=str(self.rgen), rrid=str(self.rrid),
                      pgen=str(self.pgen), prid=str(self.prid), ts=str(int(time.monotonic() * 1e6)))
        if tag != "M":
            fields.update(gen=str(self.rgen if tag == "C" else self.mgen),
                          rid=str(self.rrid if tag == "C" else self.mrid))
        if self.wrong_identity:
            fields["rev"] = "0"
        lines[0] = tag + "," + ",".join(f"{key}={value}" for key, value in fields.items())
        crc = zlib.crc32("".join(line + "\n" for line in lines).encode("ascii"))
        legacy = f"rgen={self.rgen},rrid={self.rrid},pgen={self.pgen},prid={self.prid}" if tag == "M" else f"gen={fields['gen']},rid={fields['rid']}"
        lines.append(f"K,seq={seq},{legacy},crc={crc:08X},session=AABBCCDDEE112233,bootId=1,rev={fields['rev']}")
        self.send("\n".join(lines) + "\n")

    def run(self):
        pending, last_frame = bytearray(), time.monotonic()
        while not self.stopped.is_set():
            try:
                ready, _, _ = select.select([self.master], [], [], 0.002)
                if ready:
                    pending.extend(os.read(self.master, 4096))
                    while b"\n" in pending:
                        line, _, tail = pending.partition(b"\n")
                        pending = bytearray(tail)
                        if line.strip():
                            self.reply(line.decode("ascii").strip())
                now = time.monotonic()
                if self.tail_due and now >= self.tail_due:
                    self.send("ADSFAULT,stage=RESTORE_VERIFY,err=0x109,synthetic=1\n")
                    self.tail_due = None
                if self.profile == "FULL" and now - last_frame > 0.008:
                    self.frame()
                    last_frame = now
                if self.fault and not self.fault_sent and self.full_since and now - self.full_since > 0.06:
                    self.send("MFAULT,seq=100,err=0x108,synthetic=1\n")
                    self.fault_sent, self.tail_due = True, now + 0.025
            except OSError as error:
                if error.errno not in (errno.EIO, errno.EBADF, errno.EAGAIN):
                    raise
                time.sleep(0.002)


class BenchmarkTests(unittest.TestCase):
    def options(self, device, directory, **kwargs):
        base = BenchmarkOptions(device.path, Path(directory), "synthetic", mode="VOLT", rows=1,
                                capture_seconds=0.11, warmup_seconds=0.018, prepare_seconds=1.4,
                                cleanup_seconds=0.7, command_timeout=0.045, progress_seconds=0.05,
                                stability_seconds=0.14)
        return replace(base, **kwargs)

    def run_case(self, device, options):
        with contextlib.redirect_stdout(io.StringIO()):
            index = run_benchmark(options)
        reports = [json.loads(Path(item["summary_file"]).read_text()) for item in index["results"]]
        return index, reports

    def test_exact_window_query_exclusion_and_no_overwrite(self):
        device = Firmware()
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory)
                index, (report,) = self.run_case(device, options)
                self.assertEqual(index["status"], "COMPLETE", report)
                self.assertGreaterEqual(report["capture_seconds"], options.capture_seconds)
                self.assertGreaterEqual(report["warmup_seconds"], options.warmup_seconds)
                self.assertLess(report["capture_seconds"], report["elapsed_seconds"])
                self.assertGreater(report["capture_crc_valid_frames"], 0)
                self.assertEqual(report["safe_cap_recovery"], "CAP8_QUERY_VERIFIED")
                self.assertEqual(report["usb_restore"], "VERIFIED_QUERY")
                self.assertEqual((device.mode, device.rows, device.profile), ("CAP", 8, "DEBUG"))
                self.assertTrue(all(item["verification"] == "ACK_TERMINAL_CONFIG" for item in report["transitions"]))
                raw = Path(report["raw_file"]).read_bytes()
                self.assertEqual(hashlib.sha256(raw).hexdigest(), report["raw_sha256"])
                self.assertEqual(len(raw), report["rx_bytes"])
                events = [json.loads(line) for line in Path(report["events_file"]).read_text().splitlines()]
                offset = 0
                for event in events:
                    if event["type"] == "rx":
                        self.assertEqual(event["offset"], offset)
                        offset += event["length"]
                self.assertEqual(offset, len(raw))
                before, after = report["snapshots"]["before"], report["snapshots"]["after"]
                self.assertLessEqual(max(item["monotonic"] for item in before.values()), report["capture_monotonic_start"])
                self.assertGreaterEqual(min(item["monotonic"] for item in after.values()), report["capture_monotonic_end"])
                self.assertFalse(any(event["type"] == "frame" for event in events))
                with self.assertRaises(FileExistsError):
                    run_benchmark(options)
                self.assertEqual(Path(report["raw_file"]).read_bytes(), raw)
                self.assertFalse(any(re.search(r"CAL=|RESTART|RECOVER|FPS=|ADSSET", cmd) for cmd in device.commands))
        finally:
            device.close()

    def test_lost_terminal_recovers_exact_request_using_config(self):
        device = Firmware(lost_terminal=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, (report,) = self.run_case(device, self.options(device, directory))
                self.assertEqual(index["status"], "COMPLETE", report)
                self.assertTrue(all(t["verification"] == "ACK_CONFIG_RECOVERED_TERMINAL" for t in report["transitions"]))
        finally:
            device.close()

    def test_missing_ack_is_bounded_and_never_fabricated(self):
        device = Firmware(lost_ack=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory)
                start = time.monotonic()
                index, (report,) = self.run_case(device, options)
                self.assertEqual(index["status"], "ERROR")
                self.assertIn("ack_unconfirmed", report["error"])
                self.assertEqual(report["capture_seconds"], 0)
                self.assertLess(time.monotonic() - start, 1)
        finally:
            device.close()

    def test_rail_missing_requires_explicit_values_only_in_cap(self):
        for explicit in (False, True):
            device = Firmware(rail=False)
            try:
                with tempfile.TemporaryDirectory() as directory:
                    options = self.options(device, directory)
                    if explicit:
                        options = replace(options, rail_avdd_uv=3350000, rail_avss_uv=-1795000,
                                          rail_source_note="synthetic user provenance")
                    index, (report,) = self.run_case(device, options)
                    self.assertEqual(index["status"], "COMPLETE" if explicit else "ERROR", report)
                    setters = [i for i, command in enumerate(device.commands) if command.startswith("RAILCFG=")]
                    self.assertEqual(len(setters), int(explicit))
                    if explicit:
                        last_mode = [c for c in device.commands[:setters[0]] if c.startswith("MODE=")][-1]
                        self.assertEqual(last_mode, "MODE=CAP")
                        self.assertTrue(report["volatile_rail_calibration_attempted"])
                    self.assertFalse(report["calibration_written"])
            finally:
                device.close()

    def test_fault_ends_window_preserves_tail_and_cap_recovers(self):
        device = Firmware(fault=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, (report,) = self.run_case(device, self.options(device, directory, capture_seconds=0.3))
                self.assertEqual(index["status"], "ERROR")
                self.assertLess(report["capture_seconds"], 0.3)
                self.assertFalse(report["capture_duration_satisfied"])
                raw = Path(report["raw_file"]).read_bytes()
                self.assertIn(b"MFAULT,", raw)
                self.assertIn(b"ADSFAULT,stage=RESTORE_VERIFY", raw)
                self.assertEqual(report["safe_cap_recovery"], "CAP8_QUERY_VERIFIED")
        finally:
            device.close()

    def test_wrong_frame_identity_fails_without_relabeling(self):
        device = Firmware(wrong_identity=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, (report,) = self.run_case(device, self.options(device, directory))
                self.assertEqual(index["status"], "ERROR")
                self.assertGreater(report["identity_mismatches"], 0)
                self.assertEqual(report["identity_examples"][0]["header"]["rev"], "0")
        finally:
            device.close()

    def test_wire_mode_and_mixed_row_units_cannot_borrow_correct_identity(self):
        for mode, parameters in (("VOLT", {"wrong_mode": True}), ("MIXED", {"wrong_unit": True})):
            device = Firmware(**parameters)
            try:
                with tempfile.TemporaryDirectory() as directory:
                    index, (report,) = self.run_case(device, self.options(device, directory, mode=mode, rows=2))
                    self.assertEqual(index["status"], "ERROR", report)
                    self.assertGreater(report["wire_contract_mismatches"], 0)
                    self.assertEqual(report["identity_mismatches"], 0)
                    self.assertIn("mode_or_units", report["error"])
            finally:
                device.close()

    def test_nonzero_existing_fps_cap_is_reported_not_silently_changed(self):
        device = Firmware(fps_cap=5)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, (report,) = self.run_case(device, self.options(device, directory))
                self.assertEqual(index["status"], "ERROR")
                self.assertEqual(report["capture_seconds"], 0)
                self.assertIn("unlimited_FPS", report["error"])
                self.assertFalse(any("FPS=" in command for command in device.commands))
        finally:
            device.close()

    def test_old_ack_cannot_confirm_new_same_mode_request(self):
        device = Firmware(lost_ack=True, stale_ack=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                index, (report,) = self.run_case(device, self.options(device, directory))
                self.assertEqual(index["status"], "ERROR")
                self.assertIn("ack_unconfirmed", report["error"])
                events = Path(report["events_file"]).read_text()
                self.assertIn('"type": "stale_ack_ignored"', events)
        finally:
            device.close()

    def test_suite_all_32_one_port_and_mixed1_active_cap_uses_mixed_wire(self):
        device = Firmware()
        opens = []
        def factory(path):
            opens.append(path)
            return PosixPort(path)
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory, suite=True, capture_seconds=0.04, warmup_seconds=0.01)
                self.assertEqual(len(configurations(options)), 32)
                with contextlib.redirect_stdout(io.StringIO()):
                    index = run_benchmark(options, port_factory=factory)
                self.assertEqual(index["status"], "COMPLETE", index)
                self.assertEqual(len(opens), 1)
                self.assertEqual(len(index["results"]), 32)
                raw = b""
                for item in index["results"]:
                    report = json.loads(Path(item["summary_file"]).read_text())
                    self.assertGreaterEqual(report["capture_seconds"], 0.04)
                    raw += Path(report["raw_file"]).read_bytes()
                    if item["label"].endswith("mixed1"):
                        self.assertEqual(report["benchmark_mode"], "MIXED")
                        self.assertTrue(report["mixed_active_rows_homogeneous"])
                        self.assertFalse(report["mixed_full_profile_homogeneous"])
                        self.assertEqual(report["expected_wire_mode"], "MIXED")
                        self.assertEqual(report["mixed_active_profile"], "C")
                        self.assertGreater(report["parser_counters"]["mixed_frames"], 0)
                        self.assertEqual(report["parser_counters"]["cap_frames"], 0)
                self.assertTrue(bytes(device.sent).startswith(raw))
        finally:
            device.close()

    def test_full_homogeneous_row_profile_uses_cap_wire(self):
        device = Firmware()
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory, mode="MIXED", rows=1, profile="CCCCCCCC")
                index, (report,) = self.run_case(device, options)
                self.assertEqual(index["status"], "COMPLETE", report)
                self.assertTrue(report["mixed_active_rows_homogeneous"])
                self.assertTrue(report["mixed_full_profile_homogeneous"])
                self.assertEqual(report["expected_wire_mode"], "CAP")
                self.assertGreater(report["parser_counters"]["cap_frames"], 0)
                self.assertEqual(report["parser_counters"]["mixed_frames"], 0)
        finally:
            device.close()

    def test_stability_is_additional_full_window_not_total_budget(self):
        device = Firmware()
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory, stability=True)
                index, (report,) = self.run_case(device, options)
                self.assertEqual(index["status"], "COMPLETE", report)
                self.assertEqual(report["benchmark_kind"], "stability")
                self.assertEqual(report["capture_requested_seconds"], options.stability_seconds)
                self.assertGreaterEqual(report["capture_seconds"], options.stability_seconds)
                self.assertGreater(report["duration_total_budget_seconds"], options.stability_seconds)
        finally:
            device.close()

    def test_expected_image_is_checked_before_any_setter_and_again_before_cleanup(self):
        class BoundFirmware(Firmware):
            def reply(self, command):
                if command == "BUILD?":
                    self.commands.append(command)
                    self.send("BUILD,idf=synthetic,target=esp32s3,project=SensorArray,proto=2,"
                              "version=synthetic,elfSha256=" + "ab" * 32 + "\n")
                    return
                super().reply(command)
        for elf, status in (("ab" * 32, "COMPLETE"), ("cd" * 32, "ERROR")):
            with self.subTest(elf=elf):
                device = BoundFirmware()
                try:
                    with tempfile.TemporaryDirectory() as directory:
                        options = self.options(device, directory, stability=True, expected_elf_sha256=elf,
                                               expected_session="AABBCCDDEE112233", expected_boot_id=1)
                        index, (report,) = self.run_case(device, options)
                        self.assertEqual(index["status"], status, report["error"])
                        authority = report["image_authority"]
                        self.assertTrue(authority["required"])
                        if status == "COMPLETE":
                            self.assertEqual([b["purpose"] for b in authority["brackets"]], ["prepare", "cleanup"])
                            self.assertEqual(authority["first_mutation"]["expected"]["elfSha256"], elf)
                            first_setter = next(i for i, c in enumerate(device.commands) if "=" in c)
                            self.assertEqual(device.commands[first_setter-3:first_setter], ["CONFIG?", "BUILD?", "CONFIG?"])
                            self.assertEqual(report["safe_cap_recovery"], "CAP8_QUERY_VERIFIED")
                            self.assertGreaterEqual(report["capture_seconds"], options.stability_seconds)
                        else:
                            self.assertFalse(any("=" in c for c in device.commands))
                            self.assertIsNone(authority["first_mutation"])
                            self.assertEqual(report["safe_cap_recovery"], "NOT_ATTEMPTED_AUTHORITY_UNVERIFIED")
                            self.assertIn("expected_running_image_mismatch", report["error"])
                finally:
                    device.close()

    def test_failed_case_stops_suite_even_after_verified_cap8_cleanup(self):
        device = Firmware(wrong_identity=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                options = self.options(device, directory, suite=True)
                index, reports = self.run_case(device, options)
                self.assertEqual(index["status"], "ERROR")
                self.assertEqual(len(reports), 1)
                self.assertEqual(len(index["not_run"]), 31)
                self.assertEqual(reports[0]["safe_cap_recovery"], "CAP8_QUERY_VERIFIED")
                self.assertEqual(reports[0]["status"], "ERROR")
        finally:
            device.close()


if __name__ == "__main__":
    unittest.main()
