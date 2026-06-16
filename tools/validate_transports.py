#!/usr/bin/env python3
"""Run SensorArray transport validation in the fixed serial -> BLE -> Wi-Fi order."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def run_stage(name: str, command: list[str], timeout_s: float) -> int:
    print(f"VAL,stage={name},state=start,cmd={' '.join(command)}", flush=True)
    try:
        completed = subprocess.run(command, timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired:
        print(f"VAL,stage={name},state=timeout", flush=True)
        return 124
    print(f"VAL,stage={name},state=done,rc={completed.returncode}", flush=True)
    return completed.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM11")
    parser.add_argument("--rows", default="1,2,4,8")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--serial", action="store_true")
    parser.add_argument("--ble", action="store_true")
    parser.add_argument("--wifi", action="store_true")
    parser.add_argument("--stream", choices=("data", "log", "all"), default="all")
    parser.add_argument("--tx", choices=("rel", "rt"), default="rel")
    parser.add_argument("--wifi-host")
    parser.add_argument("--wifi-ip")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    python = sys.executable
    run_serial = args.serial or not args.ble and not args.wifi
    timeout_s = args.duration + 45.0
    status = 0

    if run_serial:
        status = run_stage(
            "serial",
            [
                python,
                str(root / "receive_serial_text.py"),
                "--port",
                args.port,
                "--rows",
                args.rows,
                "--duration",
                str(args.duration),
                "--stream",
                args.stream,
                "--tx",
                args.tx,
            ],
            timeout_s,
        )
        if status != 0:
            return status

    if args.ble:
        status = run_stage(
            "ble",
            [
                python,
                str(root / "receive_ble_text.py"),
                "--name-prefix",
                "CscArray_",
                "--compat-prefix",
                "",
                "--rows",
                args.rows,
                "--duration",
                str(args.duration),
                "--stream",
                args.stream,
                "--tx",
                args.tx,
            ],
            timeout_s,
        )
        if status != 0:
            return status

    if args.wifi:
        wifi_target: list[str] = []
        if args.wifi_ip:
            wifi_target = ["--ip", args.wifi_ip]
        elif args.wifi_host:
            wifi_target = ["--host", args.wifi_host]
        else:
            print("VAL,stage=wifi,state=skip,reason=host_or_ip_required", flush=True)
            return 2
        status = run_stage(
            "wifi",
            [
                python,
                str(root / "receive_wifi_text.py"),
                *wifi_target,
                "--rows",
                args.rows,
                "--duration",
                str(args.duration),
                "--stream",
                args.stream,
                "--tx",
                args.tx,
            ],
            timeout_s,
        )
    return status


if __name__ == "__main__":
    raise SystemExit(main())
