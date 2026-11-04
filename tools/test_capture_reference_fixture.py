#!/usr/bin/env python3
"""Actual POSIX collector against a PTY; never opens the physical device."""
import contextlib
import errno
import fcntl
import io
import json
import os
from pathlib import Path
import select
import tempfile
import termios
import threading
import time
import unittest
from unittest.mock import patch

from capture_reference_fixture import CaptureOptions, PosixPort, capture
from test_reference_fixture import capacitance, mixed, resistance


class Device:
    def __init__(self, drop=None, reject=False, early_terminal=False, proto=2):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(self.master, False)
        self.drop, self.reject, self.early_terminal, self.proto = drop, reject, early_terminal, proto
        self.mode, self.rows, self.profile = "CAP", 8, "DEBUG"
        self.rowmodes, self.profile_request_id = "CCCCCCCC", 0
        self.commands, self.sent = [], bytearray()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, value):
        data = value.encode("ascii")
        try:
            written = os.write(self.master, data)
        except OSError:
            return
        self.sent.extend(data[:written])

    def reply(self, command):
        self.commands.append(command)
        if command == self.drop:
            return
        replies = {"BUILD?": "BUILD,idf=v5.5.5,target=esp32s3,proto=2",
                   "PROTO?": f"PROTO,version={self.proto},wires=ascii",
                   "CONFIG?": "CONFIG,session=AABBCCDDEE112233,bootId=1,rev=1",
                   "MODE?": f"MODE,active={self.mode},pending=NONE",
                   "ROWS?": f"ROWS,active={self.rows},pending={self.rows},requestId=0,appliedId=8",
                   "ROWMODES?": f"ROWMODES,active={self.rowmodes},pending=none,gen=1,rid={self.profile_request_id},rows={self.rows},state=applied",
                   "CAL?": "CAL,valid=0,source=0", "RAIL?": "ARL,rv=0,src=unknown",
                   "USBSTREAM?": f"USBSTREAM,v={self.profile},dataEvery={1 if self.profile == 'FULL' else 10},diagEvery={1 if self.profile == 'FULL' else 50}"}
        if command in replies:
            self.send(replies[command] + "\n")
        elif command.startswith("MODE="):
            target = command.split("=", 1)[1]
            self.send(f"MACK,id=7,new={target},state=accepted\n" if not self.early_terminal else "")
            if self.reject:
                self.send("MERR,id=7,state=rejected,err=0x108\n")
                return
            self.mode = target
            self.rowmodes = ("C" if target == "CAP" else "R") * 8
            self.profile_request_id = 7
            self.send(f"MAPP,id=7,new={target},seq=1,state=applied\n")
            if self.early_terminal:
                self.send(f"MACK,id=7,new={target},state=accepted\n")
        elif command.startswith("ROWS="):
            self.rows = int(command.split("=", 1)[1])
            self.send(f"RCMD,id=8,req={self.rows},status=accepted\nRAPP,id=8,new={self.rows},status=applied\n")
        elif command.startswith("ROWMODES="):
            target = command.split("=", 1)[1]
            self.send(f"RMACK,id=9,old={self.rowmodes},new={target},state=accepted\n" if not self.early_terminal else "")
            if self.reject:
                self.send(f"RMERR,id=9,profile={target},state=rejected,err=0x108\n")
                return
            self.rowmodes, self.profile_request_id = target, 9
            self.send(f"RMAPP,id=9,gen=1,seq=1,profile={target},state=applied\n")
            if self.early_terminal:
                self.send(f"RMACK,id=9,old=CCCCCCCC,new={target},state=accepted\n")
        elif command.startswith("USBSTREAM="):
            self.profile = command.split("=", 1)[1]
            self.send(f"USBSTREAM,v={self.profile},dataEvery={1 if self.profile == 'FULL' else 10},diagEvery={1 if self.profile == 'FULL' else 50},state=applied\n")
        else:
            self.send("ERR,reason=unexpected_command\n")

    def run(self):
        pending = bytearray()
        last_frame = time.monotonic()
        seq = 1
        while not self.stopped.is_set():
            try:
                ready, _, _ = select.select([self.master], [], [], 0.01)
                if ready:
                    pending.extend(os.read(self.master, 4096))
                    while b"\n" in pending:
                        line, _, tail = pending.partition(b"\n")
                        pending = bytearray(tail)
                        command = line.decode("ascii").strip()
                        if command:
                            self.reply(command)
                if self.profile == "FULL" and time.monotonic() - last_frame > 0.025:
                    if self.rowmodes == "RCCCCCCR":
                        lines = mixed(seq=seq)
                    else:
                        lines = resistance(rows=self.rows, seq=seq) if self.mode == "RES" else capacitance(rows=self.rows, seq=seq)
                    self.send("\n".join(lines) + "\n")
                    seq += 1
                    last_frame = time.monotonic()
            except OSError as error:
                if error.errno not in (errno.EIO, errno.EBADF, errno.EAGAIN):
                    raise
                time.sleep(0.005)

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=1)
        os.close(self.master)


class CaptureTests(unittest.TestCase):
    def invoke(self, device, directory, label="capture", **options):
        config = CaptureOptions(device.path, 0.6, Path(directory), label,
                                command_timeout=0.1, progress_seconds=0.1, **options)
        with contextlib.redirect_stdout(io.StringIO()):
            return capture(config)

    def test_raw_events_restore_and_only_allowed_ioctl(self):
        device = Device(early_terminal=True)
        real_ioctl = fcntl.ioctl
        operations = []
        def checked_ioctl(fd, operation, *args):
            operations.append(operation)
            self.assertIn(operation, (termios.TIOCEXCL, termios.TIOCNXCL))
            return real_ioctl(fd, operation, *args)
        try:
            with tempfile.TemporaryDirectory() as tmp, patch("capture_reference_fixture.fcntl.ioctl", checked_ioctl):
                result = self.invoke(device, tmp, mode="RES", rows=8)
                self.assertEqual(result["status"], "CAPTURED", result)
                self.assertEqual(result["usb_restore"], "VERIFIED_QUERY")
                self.assertEqual(device.profile, "DEBUG")
                self.assertGreater(result["capture_frame_trailers"], 0)
                self.assertLess(result["elapsed_seconds"], 0.8)
                raw = Path(result["raw_file"]).read_bytes()
                self.assertTrue(bytes(device.sent).startswith(raw))
                self.assertNotIn(b"HOST>", raw)
                self.assertNotIn(b"USBSTREAM=", raw)
                events = [json.loads(line) for line in Path(result["events_file"]).read_text().splitlines()]
                self.assertTrue(any(event["type"] == "tx_begin" for event in events))
                self.assertEqual(sum(event["length"] for event in events if event["type"] == "rx"), len(raw))
                self.assertIn("CONFIG?", device.commands)
                self.assertFalse(any(command in {"CAL=SAVE", "RESTART", "RECOVER"} for command in device.commands))
                saved = raw
                with self.assertRaises(FileExistsError):
                    self.invoke(device, tmp)
                self.assertEqual(Path(result["raw_file"]).read_bytes(), saved)
            self.assertEqual(operations, [termios.TIOCEXCL, termios.TIOCNXCL])
        finally:
            device.close()

    def test_timeout_and_rejected_terminal_are_explicit_errors(self):
        for kwargs in ({"drop": "MODE?"}, {"reject": True}):
            device = Device(**kwargs)
            try:
                with tempfile.TemporaryDirectory() as tmp:
                    result = self.invoke(device, tmp, mode="RES")
                    self.assertEqual(result["status"], "ERROR")
                    self.assertEqual(result["measurement_acceptance"], "NOT_EVALUATED")
                    self.assertLess(result["elapsed_seconds"], 0.8)
                    self.assertNotIn("USBSTREAM=FULL", device.commands)
            finally:
                device.close()

    def test_lost_full_reply_still_restores_profile(self):
        device = Device()
        original = device.reply
        def reply(command):
            if command == "USBSTREAM=FULL":
                device.commands.append(command)
                device.profile = "FULL"
            else:
                original(command)
        device.reply = reply
        try:
            with tempfile.TemporaryDirectory() as tmp:
                result = self.invoke(device, tmp)
                self.assertEqual(result["status"], "ERROR")
                self.assertEqual(result["usb_restore"], "VERIFIED_QUERY")
                self.assertEqual(device.profile, "DEBUG")
        finally:
            device.close()

    def test_kernel_exclusion_and_no_hupcl(self):
        device = Device(proto=1)
        try:
            port = PosixPort(device.path)
            try:
                attrs = termios.tcgetattr(port.fd)
                self.assertTrue(attrs[2] & termios.CLOCAL)
                self.assertTrue(attrs[2] & termios.CREAD)
                self.assertFalse(attrs[2] & termios.HUPCL)
                with self.assertRaises(OSError):
                    PosixPort(device.path)
            finally:
                port.close()
            with tempfile.TemporaryDirectory() as tmp:
                result = self.invoke(device, tmp)
                self.assertEqual(result["status"], "CAPTURED")
                self.assertNotIn("CONFIG?", device.commands)
        finally:
            device.close()

    def test_rowmodes_applies_after_rows_and_query_verifies_its_request(self):
        device = Device(early_terminal=True)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                result = self.invoke(device, tmp, rows=8, rowmodes="RCCCCCCR")
                self.assertEqual(result["status"], "CAPTURED", result)
                self.assertEqual(result["rowmodes_requested"], "RCCCCCCR")
                self.assertEqual(result["replies"]["ROWMODES?"]["rid"], "9")
                commands = device.commands
                rows_at = commands.index("ROWS=8")
                profile_at = commands.index("ROWMODES=RCCCCCCR")
                self.assertEqual(commands[rows_at:profile_at], ["ROWS=8", "ROWS?"])
                self.assertEqual(commands[profile_at:profile_at + 3],
                                 ["ROWMODES=RCCCCCCR", "ROWMODES?", "USBSTREAM=FULL"])
                self.assertFalse(any(command.startswith("MODE=") for command in commands))
                self.assertIn(b"M,seq=", Path(result["raw_file"]).read_bytes())
                self.assertEqual(result["usb_restore"], "VERIFIED_QUERY")
        finally:
            device.close()

    def test_rowmodes_rejection_missing_terminal_and_wrong_query_fail(self):
        for failure in ("reject", "wrong_terminal_id", "wrong_query_id"):
            device = Device(reject=failure == "reject")
            original = device.reply
            def reply(command):
                if failure == "wrong_terminal_id" and command.startswith("ROWMODES="):
                    device.commands.append(command)
                    device.send("RMACK,id=9,old=CCCCCCCC,new=RCCCCCCR,state=accepted\n"
                                "RMAPP,id=8,profile=RCCCCCCR,state=applied\n")
                elif failure == "wrong_query_id" and command == "ROWMODES?" and device.profile_request_id == 9:
                    device.commands.append(command)
                    device.send("ROWMODES,active=RCCCCCCR,pending=none,gen=1,rid=8,rows=8,state=applied\n")
                else:
                    original(command)
            device.reply = reply
            try:
                with tempfile.TemporaryDirectory() as tmp:
                    result = self.invoke(device, tmp, rowmodes="RCCCCCCR")
                    self.assertEqual(result["status"], "ERROR", (failure, result))
                    self.assertNotIn("USBSTREAM=FULL", device.commands)
                    self.assertLess(result["elapsed_seconds"], 0.8)
            finally:
                device.close()

    def test_conflicting_or_invalid_profile_never_opens_port_or_outputs(self):
        with tempfile.TemporaryDirectory() as tmp, patch("capture_reference_fixture.PosixPort") as port:
            for options in ({"mode": "CAP", "rowmodes": "RCCCCCCR"},
                            {"rowmodes": "RCCCCCC"}, {"rowmodes": "VCCCCCCV"}):
                with self.assertRaises(ValueError):
                    capture(CaptureOptions("not-a-device", 1, Path(tmp), "invalid", **options))
            port.assert_not_called()
            self.assertFalse(list(Path(tmp).iterdir()))


if __name__ == "__main__":
    unittest.main()
