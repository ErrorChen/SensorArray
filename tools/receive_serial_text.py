#!/usr/bin/env python3
"""Receive SensorArray compact ASCII frames from a serial port."""

from __future__ import annotations

import argparse
import time

from text_protocol import TextProtocolParser, format_cap_preview


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="For example COM11")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--summary-seconds", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to run; zero means until Ctrl-C")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("SERIAL_TEST_SKIPPED reason=missing_pyserial")
        return 2

    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if args.show_cap else None
    )
    try:
        connection = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as error:  # Hardware and driver failures vary by platform.
        print(f"SERIAL_TEST_SKIPPED reason=open_failed detail={error}")
        return 2

    print(f"SERIAL_OPEN,port={args.port},baud={args.baud}", flush=True)
    next_summary = time.monotonic() + args.summary_seconds
    start_time = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - start_time < args.duration:
            raw_line = connection.readline()
            if raw_line:
                try:
                    line = raw_line.decode("ascii")
                except UnicodeDecodeError:
                    protocol.note_non_ascii()
                    line = raw_line.decode("ascii", errors="ignore")
                protocol.feed_line(line)
            if time.monotonic() >= next_summary:
                print(protocol.summary("SERIAL5"), flush=True)
                next_summary = time.monotonic() + args.summary_seconds
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()
    print(protocol.summary("SERIAL_DONE"), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
