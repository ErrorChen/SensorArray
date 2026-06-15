#!/usr/bin/env python3
"""Receive SensorArray compact ASCII frames over UDP."""

from __future__ import annotations

import argparse
import socket
import time

from text_protocol import TextProtocolParser, format_cap_preview


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--summary-seconds", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to run; zero means until Ctrl-C")
    args = parser.parse_args()

    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if args.show_cap else None
    )
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver.bind((args.bind, args.port))
    receiver.settimeout(0.5)
    print(f"UDP_LISTEN,bind={args.bind},port={args.port}", flush=True)

    next_summary = time.monotonic() + args.summary_seconds
    start_time = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - start_time < args.duration:
            try:
                payload, _address = receiver.recvfrom(65535)
            except socket.timeout:
                payload = b""
            if payload:
                try:
                    text = payload.decode("ascii")
                except UnicodeDecodeError:
                    protocol.note_non_ascii()
                    text = payload.decode("ascii", errors="ignore")
                for line in text.splitlines():
                    protocol.feed_line(line)
            if time.monotonic() >= next_summary:
                print(protocol.summary("UDP5"), flush=True)
                next_summary = time.monotonic() + args.summary_seconds
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
    print(protocol.summary("UDP_DONE"), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
