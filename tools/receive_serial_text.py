#!/usr/bin/env python3
"""Validate SensorArray Serial control and throttled data/log output."""

from __future__ import annotations

import argparse
import time

from text_protocol import TextProtocolParser, format_cap_preview


def write_control(connection: object, command: bytes) -> bool:
    try:
        connection.write(command)
        return True
    except Exception as error:
        print(f"SERIAL_CTRL_WRITE_FAILED command={command.decode().strip()} detail={error}",
              flush=True)
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--startup-wait", type=float, default=25.0)
    parser.add_argument("--set-rows", type=int, choices=range(1, 9))
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("SERIAL_TEST_SKIPPED reason=venv_pip_install_failed")
        return 2
    try:
        connection = serial.Serial()
        connection.port = args.port
        connection.baudrate = args.baud
        connection.timeout = 0.2
        connection.write_timeout = 1.0
        connection.dtr = False
        connection.rts = False
        connection.open()
    except FileNotFoundError:
        print("SERIAL_TEST_SKIPPED reason=port_not_found")
        return 2
    except PermissionError:
        print("SERIAL_TEST_SKIPPED reason=permission_denied")
        return 2
    except Exception as error:
        print(f"SERIAL_TEST_SKIPPED reason=port_not_found detail={error}")
        return 2
    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if args.show_cap else None)
    startup_deadline = time.monotonic() + args.startup_wait
    while time.monotonic() < startup_deadline:
        connection.readline()
    try:
        write_control(connection, b"ROWS?\n")
        if args.set_rows:
            write_control(connection, f"ROWS={args.set_rows}\n".encode())
        started = time.monotonic()
        sent_rows_5 = False
        sent_rows_8 = False
        while time.monotonic() - started < args.duration:
            elapsed = time.monotonic() - started
            if args.set_rows and not sent_rows_5 and elapsed >= args.duration / 3:
                write_control(connection, b"ROWS=5\n")
                sent_rows_5 = True
            if args.set_rows and not sent_rows_8 and elapsed >= args.duration * 2 / 3:
                write_control(connection, b"ROWS=8\n")
                sent_rows_8 = True
            raw = connection.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip()
            protocol.feed_line(line)
            if args.show_log and not line.startswith(("C,", "D", "K,")):
                print(f"LOG,{line}", flush=True)
    finally:
        connection.close()
    print(protocol.summary("SERIAL_DONE"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
