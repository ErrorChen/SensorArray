#!/usr/bin/env python3
"""Validate SensorArray Serial control and throttled data/log output."""

from __future__ import annotations

import argparse
import time

from text_protocol import (TextProtocolParser, format_cap_preview,
                           format_measurement_preview)


DEFAULT_ROWMODES = (
    "CCCCCCCC",
    "VVVVVVVV",
    "RRRRRRRR",
    "CVVRRVVC",
    "RVRCCVVR",
)


def parse_rowmodes(value: str) -> list[str]:
    """Parse comma-separated 8-character C/V/R ROWMODES profiles."""

    profiles: list[str] = []
    for item in value.split(","):
        profile = item.strip().upper()
        if not profile:
            continue
        if len(profile) != 8 or any(char not in "CVR" for char in profile):
            raise ValueError(f"invalid ROWMODES profile {profile!r}")
        profiles.append(profile)
    return profiles


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
    parser.add_argument("--rowmodes", default="",
                        help="Comma-separated ROWMODES profiles to apply and validate.")
    parser.add_argument("--require-rowmodes", action="store_true",
                        help="Require ROWMODES ACK -> exactly one RMAPP/RMERR terminal "
                             "for every sent profile and fail on lifecycle errors.")
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()
    show_cap = args.show_cap or args.stream in ("data", "all")
    show_log = args.show_log or args.stream in ("log", "all")
    try:
        rowmodes = parse_rowmodes(args.rowmodes)
    except ValueError as error:
        print(f"SERIAL_TEST_SKIPPED reason=invalid_rowmodes detail={error}")
        return 2
    if args.require_rowmodes and not rowmodes:
        rowmodes = list(DEFAULT_ROWMODES)
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
        next_profile_at = started
        sent_profiles = 0
        while time.monotonic() - started < args.duration:
            elapsed = time.monotonic() - started
            if rows and time.monotonic() >= next_row_at:
                write_control(connection, f"ROWS={rows[row_index % len(rows)]}\n".encode())
                row_index += 1
                next_row_at = time.monotonic() + max(args.duration / max(len(rows), 1), 1.0)
            if rowmodes and sent_profiles < len(rowmodes) and time.monotonic() >= next_profile_at:
                profile = rowmodes[sent_profiles]
                write_control(connection, f"ROWMODES={profile}\n".encode())
                sent_profiles += 1
                print(f"SERIAL_ROWMODES,phase=send,index={sent_profiles},"
                      f"count={len(rowmodes)},profile={profile}", flush=True)
                next_profile_at = time.monotonic() + max(
                    args.duration / max(len(rowmodes), 1), 1.0)
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
    status = 0
    if rowmodes:
        lifecycle = protocol.rowmode_lifecycle_errors()
        failure = None
        if sent_profiles < len(rowmodes):
            failure = f"not_all_sent={sent_profiles}/{len(rowmodes)}"
        elif protocol.counters.rmack <= 0:
            failure = "no_ack"
        elif lifecycle["unterminated"]:
            failure = f"missing_terminal={lifecycle['unterminated']}"
        elif lifecycle["duplicate"]:
            failure = f"duplicate_terminal={lifecycle['duplicate']}"
        elif lifecycle["terminal_without_ack"]:
            failure = f"terminal_without_ack={lifecycle['terminal_without_ack']}"
        elif protocol.counters.rmack < sent_profiles:
            failure = f"ack_count={protocol.counters.rmack}/{sent_profiles}"
        if failure is not None:
            print(f"SERIAL_ROWMODES_FAIL reason={failure}", flush=True)
            status = 1
    return status


if __name__ == "__main__":
    raise SystemExit(main())
