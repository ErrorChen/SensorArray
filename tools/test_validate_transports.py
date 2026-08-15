#!/usr/bin/env python3
"""Host-only tests for the serial -> BLE transport validation orchestration.

No hardware, serial port, BLE adapter, or C compiler is required. Command
construction, duration validation, and stage ordering are tested by mocking
subprocess execution.
"""

from __future__ import annotations

import contextlib
import io
import subprocess
import unittest
from pathlib import Path
from unittest import mock

from validate_transports import (
    DEFAULT_BLE_DURATION,
    DEFAULT_SERIAL_DURATION,
    DEFAULT_SERIAL_PORT,
    MIN_BLE_DURATION,
    TIMEOUT_OVERHEAD,
    ble_rows_arg,
    ble_stage_command,
    build_parser,
    decode_output,
    echo_captured,
    normalize_tx_mode,
    resolve_durations,
    serial_stage_command,
    should_run_serial,
    stage_plan,
    stage_timeout,
    validate_ble_output,
    validate_durations,
    validate_serial_output,
    wifi_stage_command,
)
from validate_transports import main as validate_transports_main


def parse(*argv: str):
    return build_parser().parse_args(list(argv))


SERIAL_OK = (
    "LOG,C,seq=1,rows=8\n"
    "SERIAL_DONE,lines=120,cap=10,volt=10,res=10,fps=1.00,gap=0,missing=0,"
    "crc=0,bad=0,ascii=0,sum=1,bt=na\n"
)

SERIAL_MIXED_OK = (
    "SERIAL_DONE,lines=180,cap=10,volt=10,res=10,mix=6,fps=1.00,gap=0,"
    "missing=0,crc=0,bad=0,ascii=0,sum=1,bt=na,"
    "rmack=6,rmapp=5,rmerr=1,rmbad=0\n"
)


def ble_ok(port: str = DEFAULT_SERIAL_PORT) -> str:
    return (
        f"SER_SIDE,port={port},baud=115200,status=open\n"
        "BRX,ok=25,miss=0,gap=0,crc=0,tiny=0,dup=0,fps=1.00,bytes=2048,"
        "phase=stream,mix=3,rmack=2,rmapp=2,rmerr=0,rmbad=0,rmopen=0\n"
        "BF,ch=D,ok=20,ms=0,gap=0,cf=0,tiny=0,dup=0,ooo=0,dr=0\n"
        "BF,ch=L,ok=3,ms=0,gap=0,cf=0,tiny=0,dup=0,ooo=0,dr=0\n"
        "BF,ch=C,ok=2,ms=0,gap=0,cf=0,tiny=0,dup=0,ooo=0,dr=0\n"
        "SER_SIDE_DONE,resets=0,lastRst=none,lastError=none\n"
    )


class ArgumentAndOrderingTests(unittest.TestCase):
    def test_default_serial_port_is_ubuntu_acm0(self) -> None:
        args = parse()
        self.assertEqual(args.port, DEFAULT_SERIAL_PORT)
        self.assertEqual(stage_plan(args), ["serial"])

    def test_port_can_be_overridden(self) -> None:
        self.assertEqual(parse("--port", "/dev/ttyUSB0").port, "/dev/ttyUSB0")

    def test_ble_implies_serial_stage_first(self) -> None:
        args = parse("--ble")
        self.assertTrue(should_run_serial(args))
        self.assertEqual(stage_plan(args), ["serial", "ble"])

    def test_wifi_alone_does_not_run_serial(self) -> None:
        args = parse("--wifi", "--wifi-host", "example")
        self.assertFalse(should_run_serial(args))
        self.assertEqual(stage_plan(args), ["wifi"])

    def test_serial_then_wifi_order(self) -> None:
        args = parse("--serial", "--wifi", "--wifi-host", "example")
        self.assertEqual(stage_plan(args), ["serial", "wifi"])

    def test_ble_then_wifi_order(self) -> None:
        args = parse("--ble", "--wifi", "--wifi-host", "example")
        self.assertEqual(stage_plan(args), ["serial", "ble", "wifi"])


class DurationTests(unittest.TestCase):
    def test_defaults_are_serial_120_and_ble_60(self) -> None:
        durations = resolve_durations(parse())
        self.assertEqual(durations["serial"], DEFAULT_SERIAL_DURATION)
        self.assertEqual(durations["ble"], DEFAULT_BLE_DURATION)
        self.assertIsNone(validate_durations(durations, True, False, False))

    def test_legacy_duration_sets_both_when_stage_specific_absent(self) -> None:
        durations = resolve_durations(parse("--duration", "90"))
        self.assertEqual(durations["serial"], 90.0)
        self.assertEqual(durations["ble"], 90.0)
        self.assertEqual(durations["wifi"], 90.0)

    def test_stage_specific_durations_override_legacy(self) -> None:
        args = parse("--duration", "300", "--serial-duration", "90",
                     "--ble-duration", "120")
        durations = resolve_durations(args)
        self.assertEqual(durations["serial"], 90.0)
        self.assertEqual(durations["ble"], 120.0)
        self.assertEqual(durations["wifi"], 300.0)

    def test_ble_below_minimum_is_rejected_when_requested(self) -> None:
        durations = resolve_durations(parse("--ble", "--ble-duration", "59"))
        self.assertEqual(
            validate_durations(durations, True, True, False),
            "ble_duration_below_minimum",
        )

    def test_ble_exactly_minimum_is_accepted(self) -> None:
        durations = resolve_durations(parse("--ble", "--ble-duration", "60"))
        self.assertIsNone(validate_durations(durations, True, True, False))
        self.assertEqual(durations["ble"], MIN_BLE_DURATION)

    def test_short_legacy_duration_allowed_for_serial_only(self) -> None:
        durations = resolve_durations(parse("--serial", "--duration", "30"))
        self.assertIsNone(validate_durations(durations, True, False, False))

    def test_nonpositive_serial_duration_rejected(self) -> None:
        durations = resolve_durations(parse("--serial-duration", "0"))
        self.assertEqual(
            validate_durations(durations, True, False, False),
            "serial_duration_must_be_positive",
        )

    def test_stage_timeout_uses_own_duration(self) -> None:
        self.assertEqual(stage_timeout(60.0), 60.0 + TIMEOUT_OVERHEAD)
        self.assertEqual(stage_timeout(120.0), 120.0 + TIMEOUT_OVERHEAD)


class CommandConstructionTests(unittest.TestCase):
    def test_serial_command_uses_port_duration_and_normalized_tx(self) -> None:
        args = parse("--port", "/dev/ttyACM0", "--tx", "rt",
                     "--serial-duration", "90")
        durations = resolve_durations(args)
        command = serial_stage_command(
            Path("/tools"), "python3", args, durations["serial"])
        self.assertEqual(command[command.index("--port") + 1], "/dev/ttyACM0")
        self.assertEqual(command[command.index("--duration") + 1], "90.0")
        self.assertEqual(command[command.index("--tx") + 1], "SHORT")

    def test_ble_command_receives_serial_port_sidecar(self) -> None:
        args = parse("--ble", "--port", "/dev/ttyACM0", "--ble-duration", "60")
        command = ble_stage_command(
            Path("/tools"), "python3", args, args.ble_duration)
        self.assertEqual(
            command[command.index("--serial-port") + 1], "/dev/ttyACM0")
        self.assertEqual(command[command.index("--duration") + 1], "60.0")

    def test_ble_command_uses_normalized_tx(self) -> None:
        for tx, expected in (("rel", "REL"), ("rt", "SHORT")):
            with self.subTest(tx=tx):
                args = parse("--ble", "--tx", tx)
                command = ble_stage_command(
                    Path("/tools"), "python3", args, DEFAULT_BLE_DURATION)
                self.assertEqual(command[command.index("--tx") + 1], expected)
        self.assertEqual(normalize_tx_mode("rel"), "REL")
        self.assertEqual(normalize_tx_mode("rt"), "SHORT")

    def test_ble_command_omits_unsupported_wrapper_flags(self) -> None:
        args = parse("--ble")
        command = ble_stage_command(
            Path("/tools"), "python3", args, DEFAULT_BLE_DURATION)
        self.assertNotIn("--compat-prefix", command)
        self.assertNotIn("--stream", command)

    def test_require_rowmodes_serial_command_sends_profiles(self) -> None:
        args = parse("--require-rowmodes")
        command = serial_stage_command(
            Path("/tools"), "python3", args, DEFAULT_SERIAL_DURATION)
        self.assertEqual(
            command[command.index("--rowmodes") + 1],
            "CCCCCCCC,VVVVVVVV,RRRRRRRR,CVVRRVVC,RVRCCVVR",
        )

    def test_require_rowmodes_ble_command_sends_profiles(self) -> None:
        args = parse("--require-rowmodes")
        command = ble_stage_command(
            Path("/tools"), "python3", args, DEFAULT_BLE_DURATION)
        self.assertIn("--command", command)
        self.assertIn("ROWMODES=CVVRRVVC", command)
        self.assertIn("ROWMODES=RVRCCVVR", command)

    def test_ble_rows_only_passed_as_single_valid_count(self) -> None:
        self.assertIsNone(ble_rows_arg("1,2,3,4,5,6,7,8"))
        self.assertEqual(ble_rows_arg("5"), "5")
        self.assertIsNone(ble_rows_arg("9"))
        self.assertIsNone(ble_rows_arg("not-a-number"))

    def test_wifi_command_requires_host_or_ip(self) -> None:
        args = parse("--wifi")
        self.assertIsNone(wifi_stage_command(Path("/tools"), "python3", args, 120.0))

    def test_wifi_command_uses_host_or_ip_target(self) -> None:
        args = parse("--wifi", "--wifi-host", "example.local")
        command = wifi_stage_command(Path("/tools"), "python3", args, 120.0)
        self.assertIsNotNone(command)
        self.assertTrue(any("receive_wifi_text.py" in item for item in (command or [])))
        self.assertEqual((command or [])[(command or []).index("--host") + 1],
                         "example.local")


class ValidationHelperTests(unittest.TestCase):
    def test_healthy_serial_output_accepted(self) -> None:
        self.assertIsNone(validate_serial_output(SERIAL_OK))

    def test_serial_mixed_evidence_required_when_requested(self) -> None:
        self.assertIsNone(
            validate_serial_output(SERIAL_MIXED_OK, require_rowmodes=True))
        self.assertEqual(
            validate_serial_output(SERIAL_OK, require_rowmodes=True),
            "serial_mix_zero_or_missing",
        )
        missing_terminal = SERIAL_MIXED_OK.replace(
            "rmapp=5,rmerr=1", "rmapp=0,rmerr=0")
        self.assertEqual(
            validate_serial_output(missing_terminal, require_rowmodes=True),
            "serial_rm_terminal_zero_or_missing",
        )
        lifecycle_error = SERIAL_MIXED_OK.replace("rmbad=0", "rmbad=1")
        self.assertEqual(
            validate_serial_output(lifecycle_error, require_rowmodes=True),
            "serial_rmbad_nonzero_or_missing",
        )

    def test_serial_done_missing_rejected(self) -> None:
        self.assertEqual(validate_serial_output("LOG,no summary\n"),
                         "serial_done_missing")

    def test_bad_serial_summary_fields_rejected(self) -> None:
        for field in ("crc", "bad", "ascii", "gap", "missing"):
            with self.subTest(field=field):
                output = SERIAL_OK.replace(f"{field}=0", f"{field}=1")
                self.assertEqual(
                    validate_serial_output(output),
                    f"serial_{field}_nonzero_or_missing",
                )

    def test_serial_without_frames_rejected(self) -> None:
        output = (SERIAL_OK
                  .replace("cap=10", "cap=0")
                  .replace("volt=10", "volt=0")
                  .replace("res=10", "res=0"))
        self.assertEqual(validate_serial_output(output),
                         "serial_frames_zero_or_missing")

    def test_serial_reset_marker_rejected(self) -> None:
        output = "LOG,rst:0x3 (RTC_SW_SYS_RST)\n" + SERIAL_OK
        self.assertEqual(validate_serial_output(output),
                         "reset_or_panic_marker:rst:")

    def test_serial_guru_meditation_rejected(self) -> None:
        output = "LOG,Guru Meditation Error: Core 0 panic'ed\n" + SERIAL_OK
        self.assertEqual(validate_serial_output(output),
                         "reset_or_panic_marker:guru meditation")

    def test_serial_skipped_rejected(self) -> None:
        output = "SERIAL_TEST_SKIPPED reason=port_not_found\n" + SERIAL_OK
        self.assertEqual(validate_serial_output(output), "serial_skipped")

    def test_healthy_ble_output_accepted(self) -> None:
        self.assertIsNone(validate_ble_output(ble_ok(), DEFAULT_SERIAL_PORT))
        self.assertIsNone(
            validate_ble_output(ble_ok(), DEFAULT_SERIAL_PORT,
                                require_rowmodes=True))

    def test_ble_mixed_evidence_required_when_requested(self) -> None:
        output = ble_ok().replace("mix=3", "mix=0")
        self.assertEqual(
            validate_ble_output(output, DEFAULT_SERIAL_PORT,
                                require_rowmodes=True),
            "ble_mix_zero_or_missing",
        )
        output = ble_ok().replace("rmapp=2,rmerr=0", "rmapp=0,rmerr=0")
        self.assertEqual(
            validate_ble_output(output, DEFAULT_SERIAL_PORT,
                                require_rowmodes=True),
            "ble_rm_terminal_zero_or_missing",
        )
        output = ble_ok().replace("rmbad=0", "rmbad=1")
        self.assertEqual(
            validate_ble_output(output, DEFAULT_SERIAL_PORT,
                                require_rowmodes=True),
            "ble_rmbad_nonzero_or_missing",
        )
        output = ble_ok().replace("rmopen=0", "rmopen=1")
        self.assertEqual(
            validate_ble_output(output, DEFAULT_SERIAL_PORT,
                                require_rowmodes=True),
            "ble_rmopen_nonzero_or_missing",
        )

    def test_ble_sidecar_open_missing_rejected(self) -> None:
        output = ble_ok().replace(
            f"SER_SIDE,port={DEFAULT_SERIAL_PORT},baud=115200,status=open\n", "")
        self.assertEqual(validate_ble_output(output, DEFAULT_SERIAL_PORT),
                         "serial_sidecar_open_missing")

    def test_ble_sidecar_open_error_rejected(self) -> None:
        output = ble_ok().replace(
            f"SER_SIDE,port={DEFAULT_SERIAL_PORT},baud=115200,status=open\n",
            "SER_SIDE,reason=open_failed,detail=permission_denied\n")
        self.assertEqual(validate_ble_output(output, DEFAULT_SERIAL_PORT),
                         "serial_sidecar_open_failed")

    def test_ble_sidecar_reset_rejected(self) -> None:
        output = ble_ok().replace("SER_SIDE_DONE,resets=0",
                                  "SER_SIDE_DONE,resets=1")
        self.assertEqual(validate_ble_output(output, DEFAULT_SERIAL_PORT),
                         "serial_sidecar_resets_nonzero")

    def test_ble_sidecar_error_rejected(self) -> None:
        output = ble_ok().replace("lastError=none",
                                  "lastError=open_failed:permission_denied")
        self.assertEqual(validate_ble_output(output, DEFAULT_SERIAL_PORT),
                         "serial_sidecar_error")

    def test_ble_brx_crc_or_miss_rejected(self) -> None:
        for field in ("crc", "miss"):
            with self.subTest(field=field):
                output = ble_ok().replace(f"{field}=0,", f"{field}=1,")
                self.assertEqual(
                    validate_ble_output(output, DEFAULT_SERIAL_PORT),
                    f"ble_{field}_nonzero_or_missing",
                )

    def test_ble_brx_zero_or_missing_received_rejected(self) -> None:
        cases = {
            "zero": ble_ok().replace("BRX,ok=25,", "BRX,ok=0,"),
            "missing": ble_ok().replace(",ok=25,", ","),
        }
        for label, output in cases.items():
            with self.subTest(case=label):
                self.assertEqual(
                    validate_ble_output(output, DEFAULT_SERIAL_PORT),
                    "ble_brx_ok_zero_or_missing",
                )

    def test_ble_fragment_failure_rejected(self) -> None:
        cases = {
            "cf": ble_ok().replace(
                "BF,ch=D,ok=20,ms=0,gap=0,cf=0",
                "BF,ch=D,ok=20,ms=0,gap=0,cf=1"),
            "ms": ble_ok().replace(
                "BF,ch=D,ok=20,ms=0,gap=0,cf=0",
                "BF,ch=D,ok=20,ms=1,gap=0,cf=0"),
            "dr": ble_ok().replace(
                "BF,ch=D,ok=20,ms=0,gap=0,cf=0,tiny=0,dup=0,ooo=0,dr=0",
                "BF,ch=D,ok=20,ms=0,gap=0,cf=0,tiny=0,dup=0,ooo=0,dr=1"),
        }
        for field, output in cases.items():
            with self.subTest(field=field):
                self.assertEqual(
                    validate_ble_output(output, DEFAULT_SERIAL_PORT),
                    f"ble_fragment_{field}_nonzero_or_missing",
                )

    def test_ble_skipped_never_treated_as_pass(self) -> None:
        output = "BLE_TEST_SKIPPED reason=device_not_found\n" + ble_ok()
        self.assertEqual(validate_ble_output(output, DEFAULT_SERIAL_PORT),
                         "ble_skipped")

    def test_ble_failed_marker_rejected(self) -> None:
        output = "BLE_TEST_FAILED reason=runtime_disconnect\n" + ble_ok()
        self.assertEqual(validate_ble_output(output, DEFAULT_SERIAL_PORT),
                         "ble_failed")


class ExecutionSequencingTests(unittest.TestCase):
    def _run_main(self, argv, serial=SERIAL_OK, serial_rc=0,
                  ble=ble_ok(), ble_rc=0):
        calls: list[list[str]] = []

        def fake_run(command, timeout, check=False,
                     capture_output=False, text=False):
            calls.append(command)
            joined = " ".join(command)
            if "receive_ble_text.py" in joined:
                stdout, rc = ble, ble_rc
            elif "receive_serial_text.py" in joined:
                stdout, rc = serial, serial_rc
            else:
                stdout, rc = "", 0
            return subprocess.CompletedProcess(
                command, rc, stdout=stdout, stderr="")

        captured = io.StringIO()
        with mock.patch("validate_transports.subprocess.run",
                        side_effect=fake_run):
            with contextlib.redirect_stdout(captured), \
                    contextlib.redirect_stderr(captured):
                code = validate_transports_main(argv)
        return code, calls, captured.getvalue()

    def test_no_flags_runs_healthy_serial_only(self) -> None:
        code, calls, _ = self._run_main([])
        self.assertEqual(code, 0)
        self.assertEqual(len(calls), 1)
        self.assertIn("receive_serial_text.py", calls[0][1])

    def test_serial_failure_prevents_ble_start(self) -> None:
        code, calls, _ = self._run_main(
            ["--ble", "--port", "/dev/ttyACM0"],
            serial="", serial_rc=7)
        self.assertEqual(code, 7)
        self.assertEqual(len(calls), 1)
        self.assertIn("receive_serial_text.py", calls[0][1])

    def test_serial_missing_done_blocks_ble(self) -> None:
        code, calls, captured = self._run_main(
            ["--ble"], serial="LOG,no summary\n")
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 1)
        self.assertIn("VAL,stage=serial,state=fail,reason=serial_done_missing",
                      captured)

    def test_bad_serial_summary_blocks_ble(self) -> None:
        code, calls, captured = self._run_main(
            ["--ble"], serial=SERIAL_OK.replace("crc=0", "crc=1"))
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 1)
        self.assertIn(
            "VAL,stage=serial,state=fail,reason=serial_crc_nonzero_or_missing",
            captured)

    def test_reset_marker_blocks_ble(self) -> None:
        code, calls, captured = self._run_main(
            ["--ble"],
            serial="LOG,Guru Meditation Error: Core 0 panic'ed\n" + SERIAL_OK)
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 1)
        self.assertIn(
            "VAL,stage=serial,state=fail,"
            "reason=reset_or_panic_marker:guru meditation",
            captured)

    def test_healthy_serial_then_ble_runs_both(self) -> None:
        code, calls, _ = self._run_main(["--ble", "--port", "/dev/ttyACM0"])
        self.assertEqual(code, 0)
        self.assertEqual(len(calls), 2)
        self.assertIn("receive_serial_text.py", calls[0][1])
        self.assertIn("receive_ble_text.py", calls[1][1])
        ble_command = calls[1]
        self.assertEqual(
            ble_command[ble_command.index("--serial-port") + 1],
            "/dev/ttyACM0",
        )
        self.assertGreaterEqual(
            float(ble_command[ble_command.index("--duration") + 1]),
            MIN_BLE_DURATION,
        )

    def test_ble_sidecar_error_blocks_run(self) -> None:
        bad_ble = ble_ok().replace(
            f"SER_SIDE,port={DEFAULT_SERIAL_PORT},baud=115200,status=open\n",
            "SER_SIDE,reason=open_failed,detail=permission_denied\n")
        code, calls, captured = self._run_main(["--ble"], ble=bad_ble)
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 2)
        self.assertIn(
            "VAL,stage=ble,state=fail,reason=serial_sidecar_open_failed",
            captured)

    def test_ble_crc_failure_blocks_run(self) -> None:
        code, calls, captured = self._run_main(
            ["--ble"], ble=ble_ok().replace("crc=0,", "crc=1,"))
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 2)
        self.assertIn(
            "VAL,stage=ble,state=fail,reason=ble_crc_nonzero_or_missing",
            captured)

    def test_ble_skipped_blocks_run(self) -> None:
        code, calls, captured = self._run_main(
            ["--ble"],
            ble="BLE_TEST_SKIPPED reason=device_not_found\n" + ble_ok())
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 2)
        self.assertIn("VAL,stage=ble,state=fail,reason=ble_skipped", captured)

    def test_ble_rc_failure_blocks_run(self) -> None:
        code, calls, _ = self._run_main(
            ["--ble"], ble=ble_ok(), ble_rc=2)
        self.assertEqual(code, 2)
        self.assertEqual(len(calls), 2)

    def test_ble_zero_received_blocks_run(self) -> None:
        code, calls, captured = self._run_main(
            ["--ble"], ble=ble_ok().replace("BRX,ok=25,", "BRX,ok=0,"))
        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 2)
        self.assertIn(
            "VAL,stage=ble,state=fail,reason=ble_brx_ok_zero_or_missing",
            captured,
        )

    def test_short_ble_duration_aborts_before_any_command(self) -> None:
        code, calls, _ = self._run_main(["--ble", "--ble-duration", "30"])
        self.assertEqual(code, 2)
        self.assertEqual(calls, [])

    def test_legacy_duration_cannot_bypass_ble_minimum(self) -> None:
        code, calls, _ = self._run_main(["--ble", "--duration", "30"])
        self.assertEqual(code, 2)
        self.assertEqual(calls, [])

    def test_serial_timeout_aborts_before_ble(self) -> None:
        calls: list[list[str]] = []

        def timing_out(command, timeout, check=False,
                       capture_output=False, text=False):
            calls.append(command)
            raise subprocess.TimeoutExpired(" ".join(command), timeout,
                                            output="partial serial output")

        with mock.patch("validate_transports.subprocess.run",
                        side_effect=timing_out):
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                code = validate_transports_main(["--ble"])
        self.assertEqual(code, 124)
        self.assertEqual(len(calls), 1)

    def test_serial_timeout_with_bytes_output_aborts_before_ble(self) -> None:
        calls: list[list[str]] = []

        def timing_out(command, timeout, check=False,
                       capture_output=False, text=False):
            calls.append(command)
            raise subprocess.TimeoutExpired(
                " ".join(command), timeout,
                output=b"partial serial output \xff",
                stderr=b"partial serial error \xff")

        captured = io.StringIO()
        with mock.patch("validate_transports.subprocess.run",
                        side_effect=timing_out):
            with contextlib.redirect_stdout(captured), \
                    contextlib.redirect_stderr(captured):
                code = validate_transports_main(["--ble"])
        self.assertEqual(code, 124)
        self.assertEqual(len(calls), 1)
        self.assertIn("VAL,stage=serial,state=timeout", captured.getvalue())


class CapturedOutputTests(unittest.TestCase):
    def test_decode_output_normalizes_bytes_str_none(self) -> None:
        self.assertEqual(decode_output(None), "")
        self.assertEqual(decode_output("plain"), "plain")
        self.assertEqual(decode_output(b"raw"), "raw")
        self.assertEqual(decode_output(b"\xff"), "\ufffd")

    def test_echo_captured_accepts_bytes_and_str(self) -> None:
        out = io.StringIO()
        err = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            combined = echo_captured(b"serial \xff bytes", "and str")
        self.assertEqual(combined, "serial \ufffd bytesand str")
        self.assertEqual(out.getvalue(), "serial \ufffd bytes")
        self.assertEqual(err.getvalue(), "and str")

    def test_echo_captured_none_is_empty(self) -> None:
        out = io.StringIO()
        err = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            combined = echo_captured(None, None)
        self.assertEqual(combined, "")
        self.assertEqual(out.getvalue(), "")
        self.assertEqual(err.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
