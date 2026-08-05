#!/usr/bin/env python3
"""Validate SensorArray Serial control and throttled data/log output."""

from __future__ import annotations

import argparse
import time

from text_protocol import (TextProtocolParser, format_cap_preview,
                           format_measurement_preview)


def decode_ascii_line(raw: bytes, protocol: TextProtocolParser) -> str | None:
    raw_line = raw.rstrip(b"\r\n")
    try:
        return raw_line.decode("ascii")
    except UnicodeDecodeError as error:
        value = raw_line[error.start] if error.start < len(raw_line) else 0
        protocol.note_non_ascii()
        print(f"SERIAL_ASCII_ERR,off={error.start},byte={value:02X},len={len(raw_line)}",
              flush=True)
        return None


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
    parser.add_argument("--rows", default="")
    parser.add_argument("--stream", choices=("data", "log", "all"))
    parser.add_argument("--tx", choices=("SHORT", "REL", "FULL", "short", "rel", "full", "rt"))
    parser.add_argument("--fpscap", choices=("OFF", "off"))
    parser.add_argument("--outcap", choices=("OFF", "off"))
    parser.add_argument("--adsgap", choices=("OFF", "ON", "RAIL", "BAT", "ZERO",
                                             "off", "on", "rail", "bat", "zero"))
    parser.add_argument("--command", action="append", default=[],
                        help="Additional raw control command to send after ST=SER.")
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()
    show_cap = args.show_cap or args.stream in ("data", "all")
    show_log = args.show_log or args.stream in ("log", "all")
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
        if show_cap else None,
        on_measurement_frame=(
            lambda frame: print(format_measurement_preview(frame), flush=True)
        ) if show_cap else None)
    startup_deadline = time.monotonic() + args.startup_wait
    while time.monotonic() < startup_deadline:
        connection.readline()
    try:
        write_control(connection, b"ST=ser\n")
        if args.tx:
            tx_mode = args.tx.upper()
            if tx_mode == "RT":
                tx_mode = "SHORT"
            write_control(connection, f"TX={tx_mode}\n".encode())
        if args.fpscap:
            write_control(connection, b"FPSCAP=OFF\n")
        if args.outcap:
            write_control(connection, b"OUTCAP=OFF\n")
        if args.adsgap:
            write_control(connection, f"ADSGAP={args.adsgap.upper()}\n".encode())
        for command in args.command:
            text = command if command.endswith("\n") else command + "\n"
            write_control(connection, text.encode("ascii"))
        write_control(connection, b"ROWS?\n")
        if args.set_rows:
            write_control(connection, f"ROWS={args.set_rows}\n".encode())
        started = time.monotonic()
        rows = [int(item) for item in args.rows.split(",") if item] if args.rows else []
        row_index = 0
        next_row_at = started
        sent_rows_5 = False
        sent_rows_8 = False
        while time.monotonic() - started < args.duration:
            elapsed = time.monotonic() - started
            if rows and time.monotonic() >= next_row_at:
                write_control(connection, f"ROWS={rows[row_index % len(rows)]}\n".encode())
                row_index += 1
                next_row_at = time.monotonic() + max(args.duration / max(len(rows), 1), 1.0)
            if not rows and args.set_rows and not sent_rows_5 and elapsed >= args.duration / 3:
                write_control(connection, b"ROWS=5\n")
                sent_rows_5 = True
            if not rows and args.set_rows and not sent_rows_8 and elapsed >= args.duration * 2 / 3:
                write_control(connection, b"ROWS=8\n")
                sent_rows_8 = True
            raw = connection.readline()
            if not raw:
                continue
            line = decode_ascii_line(raw, protocol)
            if line is None:
                continue
            protocol.feed_line(line)
            if show_log and not line.startswith(("C,", "V,", "R,", "D", "P", "K,")):
                print(f"LOG,{line}", flush=True)
    finally:
        connection.close()
    print(protocol.summary("SERIAL_DONE"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
