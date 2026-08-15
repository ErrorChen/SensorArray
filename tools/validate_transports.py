#!/usr/bin/env python3
"""Run SensorArray transport validation in the fixed serial -> BLE -> Wi-Fi order.

Contract:
- The Ubuntu default serial port is /dev/ttyACM0; --port overrides it.
- Requesting BLE always runs the serial stage first on the same port. A serial
  failure or timeout aborts the run before BLE starts.
- BLE runs at least 60 seconds by default and receives --serial-port so the
  same serial port acts as a reset/panic sidecar during the BLE stage.
- --duration remains a legacy alias for the old shared duration. It applies to
  both serial and BLE (and Wi-Fi) only when the stage-specific options are not
  given; --serial-duration defaults to 120 and --ble-duration defaults to 60.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_SERIAL_DURATION = 120.0
DEFAULT_BLE_DURATION = 60.0
DEFAULT_WIFI_DURATION = 120.0
MIN_BLE_DURATION = 60.0
TIMEOUT_OVERHEAD = 45.0
STAGE_TIMEOUT_EXIT = 124

FAIL_MARKERS = (
    "esp-rom",
    "rst:",
    "rst,reason=",
    "guru meditation",
    "stack canary",
    "panic'ed",
    "loadprohibited",
    "watchdog",
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT,
                        help="Serial port for both stages (Ubuntu default: "
                             f"{DEFAULT_SERIAL_PORT}).")
    parser.add_argument("--rows", default="1,2,3,4,5,6,7,8")
    parser.add_argument("--duration", type=float, default=None,
                        help="Legacy shared duration; used as the fallback for "
                             "serial/BLE/Wi-Fi when stage-specific durations are absent.")
    parser.add_argument("--serial-duration", type=float, default=None,
                        help=f"Serial stage duration in seconds (default {DEFAULT_SERIAL_DURATION:g}).")
    parser.add_argument("--ble-duration", type=float, default=None,
                        help=f"BLE stage duration in seconds, minimum {MIN_BLE_DURATION:g} "
                             f"(default {DEFAULT_BLE_DURATION:g}).")
    parser.add_argument("--serial", action="store_true")
    parser.add_argument("--ble", action="store_true")
    parser.add_argument("--wifi", action="store_true")
    parser.add_argument(
        "--require-rowmodes", dest="requireRowmodes", action="store_true",
        help="Require mixed-row/profile evidence: mix>0, RMACK>=1 and one "
             "terminal (RMAPP/RMERR) per request with no lifecycle errors.")
    parser.add_argument("--stream", choices=("data", "log", "all"), default="all")
    parser.add_argument("--tx", choices=("rel", "rt"), default="rel")
    parser.add_argument("--wifi-host")
    parser.add_argument("--wifi-ip")
    return parser


def should_run_serial(args: argparse.Namespace) -> bool:
    """Serial runs when explicitly requested, when BLE is requested, or by default."""
    return args.serial or args.ble or not args.wifi


def stage_plan(args: argparse.Namespace) -> list[str]:
    """Return the fixed execution order for the requested stages."""
    stages: list[str] = []
    if should_run_serial(args):
        stages.append("serial")
    if args.ble:
        stages.append("ble")
    if args.wifi:
        stages.append("wifi")
    return stages


def resolve_durations(args: argparse.Namespace) -> dict[str, float]:
    """Resolve stage durations, honoring stage-specific flags over --duration."""
    legacy = args.duration
    return {
        "serial": args.serial_duration if args.serial_duration is not None
        else (legacy if legacy is not None else DEFAULT_SERIAL_DURATION),
        "ble": args.ble_duration if args.ble_duration is not None
        else (legacy if legacy is not None else DEFAULT_BLE_DURATION),
        "wifi": legacy if legacy is not None else DEFAULT_WIFI_DURATION,
    }


def validate_durations(durations: dict[str, float], serial_requested: bool,
                       ble_requested: bool, wifi_requested: bool) -> str | None:
    """Return a rejection reason for invalid requested durations, or None."""
    if serial_requested and durations["serial"] <= 0:
        return "serial_duration_must_be_positive"
    if ble_requested and durations["ble"] < MIN_BLE_DURATION:
        return "ble_duration_below_minimum"
    if wifi_requested and durations["wifi"] <= 0:
        return "wifi_duration_must_be_positive"
    return None


def normalize_tx_mode(tx: str) -> str:
    """Map wrapper rel/rt values to values accepted by receive_ble_text.py."""
    return {"rel": "REL", "rt": "SHORT"}[tx]


def ble_rows_arg(rows: str) -> str | None:
    """BLE accepts one integer row count; return it or None when not applicable."""
    try:
        value = int(rows, 10)
    except ValueError:
        return None
    if 1 <= value <= 8:
        return str(value)
    return None


def stage_timeout(duration_s: float) -> float:
    return duration_s + TIMEOUT_OVERHEAD


def serial_stage_command(root: Path, python: str, args: argparse.Namespace,
                         serial_duration: float) -> list[str]:
    command = [
        python,
        str(root / "receive_serial_text.py"),
        "--port",
        args.port,
        "--rows",
        args.rows,
        "--duration",
        str(serial_duration),
        "--stream",
        args.stream,
        "--tx",
        normalize_tx_mode(args.tx),
    ]
    if args.requireRowmodes:
        command.extend(["--rowmodes", "CCCCCCCC,VVVVVVVV,RRRRRRRR,CVVRRVVC,RVRCCVVR"])
    return command


def ble_stage_command(root: Path, python: str, args: argparse.Namespace,
                      ble_duration: float) -> list[str]:
    command = [
        python,
        str(root / "receive_ble_text.py"),
        "--name-prefix",
        "CscArray_",
        "--duration",
        str(ble_duration),
        "--tx",
        normalize_tx_mode(args.tx),
        "--serial-port",
        args.port,
    ]
    rows = ble_rows_arg(args.rows)
    if rows is not None:
        command.extend(["--rows", rows])
    if args.requireRowmodes:
        command.extend([
            "--command", "ROWMODES=CVVRRVVC",
            "--command", "ROWMODES=RVRCCVVR",
        ])
    return command


def wifi_stage_command(root: Path, python: str, args: argparse.Namespace,
                       wifi_duration: float) -> list[str] | None:
    if args.wifi_ip:
        wifi_target = ["--ip", args.wifi_ip]
    elif args.wifi_host:
        wifi_target = ["--host", args.wifi_host]
    else:
        return None
    return [
        python,
        str(root / "receive_wifi_text.py"),
        *wifi_target,
        "--rows",
        args.rows,
        "--duration",
        str(wifi_duration),
        "--stream",
        args.stream,
        "--tx",
        args.tx,
    ]


def parse_kv_line(line: str) -> dict[str, str]:
    """Parse a comma-separated key=value line into a field dict."""
    fields: dict[str, str] = {}
    for part in line.split(","):
        key, separator, value = part.partition("=")
        if separator:
            fields[key.strip()] = value.strip()
    return fields


def last_summary_fields(output: str, marker: str) -> dict[str, str]:
    """Return fields from the last line starting with the given marker."""
    fields: dict[str, str] = {}
    for line in output.splitlines():
        if line.startswith(marker):
            fields = parse_kv_line(line)
    return fields


def _int_field(fields: dict[str, str], name: str) -> int | None:
    value = fields.get(name)
    if value is None:
        return None
    try:
        return int(value, 10)
    except ValueError:
        return None


def validate_serial_output(output: str, require_rowmodes: bool = False) -> str | None:
    """Return a rejection reason for a serial stage, or None when acceptable."""
    if "SERIAL_TEST_SKIPPED" in output:
        return "serial_skipped"
    lowered = output.lower()
    for marker in FAIL_MARKERS:
        if marker in lowered:
            return f"reset_or_panic_marker:{marker}"
    fields = last_summary_fields(output, "SERIAL_DONE")
    if not fields:
        return "serial_done_missing"
    lines = _int_field(fields, "lines")
    frames = sum(
        value for value in (
            _int_field(fields, "cap"),
            _int_field(fields, "volt"),
            _int_field(fields, "res"),
            _int_field(fields, "mix"),
        ) if value is not None
    )
    if lines is None or lines <= 0:
        return "serial_lines_zero_or_missing"
    if frames <= 0:
        return "serial_frames_zero_or_missing"
    for name in ("crc", "bad", "ascii", "gap", "missing"):
        value = _int_field(fields, name)
        if value is None or value != 0:
            return f"serial_{name}_nonzero_or_missing"
    if require_rowmodes:
        if (_int_field(fields, "mix") or 0) <= 0:
            return "serial_mix_zero_or_missing"
        if (_int_field(fields, "rmack") or 0) <= 0:
            return "serial_rmack_zero_or_missing"
        terminals = (_int_field(fields, "rmapp") or 0) + (
            _int_field(fields, "rmerr") or 0)
        if terminals <= 0:
            return "serial_rm_terminal_zero_or_missing"
        if (_int_field(fields, "rmbad") or 0) != 0:
            return "serial_rmbad_nonzero_or_missing"
    return None


def validate_ble_output(output: str, port: str,
                        require_rowmodes: bool = False) -> str | None:
    """Return a rejection reason for a BLE stage, or None when acceptable."""
    if "BLE_TEST_SKIPPED" in output:
        return "ble_skipped"
    if "BLE_TEST_FAILED" in output:
        return "ble_failed"
    side_open: dict[str, str] | None = None
    side_done: dict[str, str] | None = None
    for line in output.splitlines():
        if line.startswith("SER_SIDE,"):
            side_open = parse_kv_line(line)
        elif line.startswith("SER_SIDE_DONE"):
            side_done = parse_kv_line(line)
    if side_open is None:
        return "serial_sidecar_open_missing"
    if side_open.get("status") != "open":
        return "serial_sidecar_open_failed"
    if side_open.get("port") != port:
        return "serial_sidecar_port_mismatch"
    if side_done is None:
        return "serial_sidecar_done_missing"
    if side_done.get("resets") != "0":
        return "serial_sidecar_resets_nonzero"
    if side_done.get("lastError") != "none":
        return "serial_sidecar_error"
    brx_fields = last_summary_fields(output, "BRX")
    if not brx_fields:
        return "ble_brx_missing"
    received = _int_field(brx_fields, "ok")
    if received is None or received <= 0:
        return "ble_brx_ok_zero_or_missing"
    for name in ("crc", "miss"):
        value = _int_field(brx_fields, name)
        if value is None or value != 0:
            return f"ble_{name}_nonzero_or_missing"
    for line in output.splitlines():
        if line.startswith("BF,"):
            fields = parse_kv_line(line)
            for name in ("cf", "ms", "dr"):
                value = _int_field(fields, name)
                if value is None or value != 0:
                    return f"ble_fragment_{name}_nonzero_or_missing"
    if require_rowmodes:
        if (_int_field(brx_fields, "mix") or 0) <= 0:
            return "ble_mix_zero_or_missing"
        if (_int_field(brx_fields, "rmack") or 0) <= 0:
            return "ble_rmack_zero_or_missing"
        terminals = (_int_field(brx_fields, "rmapp") or 0) + (
            _int_field(brx_fields, "rmerr") or 0)
        if terminals <= 0:
            return "ble_rm_terminal_zero_or_missing"
        if (_int_field(brx_fields, "rmbad") or 0) != 0:
            return "ble_rmbad_nonzero_or_missing"
        rmopen = _int_field(brx_fields, "rmopen")
        if rmopen is None or rmopen != 0:
            return "ble_rmopen_nonzero_or_missing"
    return None


def decode_output(value: bytes | str | None) -> str:
    """Normalize captured child output (str/bytes/None) to readable str."""
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def echo_captured(stdout: str | None, stderr: str | None) -> str:
    """Echo captured child output unchanged and return it combined.

    A real subprocess.run(text=True) TimeoutExpired may still carry bytes in
    stdout/stderr, so normalize before echoing or concatenating.
    """
    stdout_text = decode_output(stdout)
    stderr_text = decode_output(stderr)
    output = stdout_text + stderr_text
    if stdout_text:
        sys.stdout.write(stdout_text)
        sys.stdout.flush()
    if stderr_text:
        sys.stderr.write(stderr_text)
        sys.stderr.flush()
    return output


def run_stage(name: str, command: list[str], timeout_s: float,
              duration_s: float, port: str) -> tuple[int, str]:
    print(f"VAL,stage={name},state=start,cmd={' '.join(command)},"
          f"duration={duration_s:g},port={port}", flush=True)
    try:
        completed = subprocess.run(command, timeout=timeout_s, check=False,
                                   capture_output=True, text=True)
    except subprocess.TimeoutExpired as error:
        output = echo_captured(error.stdout, error.stderr)
        print(f"VAL,stage={name},state=timeout,duration={duration_s:g},port={port}",
              flush=True)
        return STAGE_TIMEOUT_EXIT, output
    output = echo_captured(completed.stdout, completed.stderr)
    print(f"VAL,stage={name},state=done,rc={completed.returncode},"
          f"duration={duration_s:g},port={port}", flush=True)
    return completed.returncode, output


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    durations = resolve_durations(args)
    serial_requested = should_run_serial(args)
    reason = validate_durations(durations, serial_requested, args.ble, args.wifi)
    if reason is not None:
        print(f"VAL,stage=config,state=skip,reason={reason},"
              f"serial_duration={durations['serial']:g},"
              f"ble_duration={durations['ble']:g},"
              f"wifi_duration={durations['wifi']:g},min_ble={MIN_BLE_DURATION:g}",
              flush=True)
        return 2

    root = Path(__file__).resolve().parent
    python = sys.executable
    status = 0

    if serial_requested:
        status, serial_output = run_stage(
            "serial",
            serial_stage_command(root, python, args, durations["serial"]),
            stage_timeout(durations["serial"]),
            durations["serial"],
            args.port,
        )
        if status == 2:
            print(f"VAL,stage=serial,state=not_run,reason=serial_skipped,"
                  f"port={args.port},duration={durations['serial']:g}",
                  flush=True)
            return status
        if status != 0:
            return status
        serial_reason = validate_serial_output(
            serial_output, require_rowmodes=args.requireRowmodes)
        if serial_reason is not None:
            print(f"VAL,stage=serial,state=fail,reason={serial_reason},"
                  f"port={args.port},duration={durations['serial']:g}",
                  flush=True)
            return 1

    if args.ble:
        status, ble_output = run_stage(
            "ble",
            ble_stage_command(root, python, args, durations["ble"]),
            stage_timeout(durations["ble"]),
            durations["ble"],
            args.port,
        )
        if status == 2:
            print(f"VAL,stage=ble,state=not_run,reason=ble_skipped,"
                  f"port={args.port},duration={durations['ble']:g}",
                  flush=True)
            return status
        if status != 0:
            return status
        ble_reason = validate_ble_output(
            ble_output, args.port, require_rowmodes=args.requireRowmodes)
        if ble_reason is not None:
            print(f"VAL,stage=ble,state=fail,reason={ble_reason},"
                  f"port={args.port},duration={durations['ble']:g}",
                  flush=True)
            return 1

    if args.wifi:
        wifi_command = wifi_stage_command(root, python, args, durations["wifi"])
        if wifi_command is None:
            print("VAL,stage=wifi,state=skip,reason=host_or_ip_required,"
                  f"duration={durations['wifi']:g},port=n/a", flush=True)
            return 2
        status, _wifi_output = run_stage(
            "wifi",
            wifi_command,
            stage_timeout(durations["wifi"]),
            durations["wifi"],
            "n/a",
        )
    return status


if __name__ == "__main__":
    raise SystemExit(main())
