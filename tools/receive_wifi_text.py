#!/usr/bin/env python3
"""Validate SensorArray Wi-Fi UDP data/log/control channels."""

from __future__ import annotations

import argparse
import select
import socket
import time

from text_protocol import TextProtocolParser, format_cap_preview


def receiver(port: int) -> socket.socket:
    result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    result.bind(("0.0.0.0", port))
    result.setblocking(False)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="192.168.4.1")
    parser.add_argument("--data-port", type=int, default=3333)
    parser.add_argument("--log-port", type=int, default=3334)
    parser.add_argument("--ctrl-port", type=int, default=3335)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--set-rows", type=int, choices=range(1, 9))
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()

    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if args.show_cap else None)
    try:
        sockets = {
            receiver(args.data_port): "data",
            receiver(args.log_port): "log",
            receiver(args.ctrl_port): "ctrl",
        }
    except PermissionError as error:
        print(f"WIFI_TEST_SKIPPED reason=permission_denied detail={error}")
        return 2
    ctrl = next(sock for sock, channel in sockets.items() if channel == "ctrl")
    if args.set_rows:
        ctrl.sendto(f"ROWS={args.set_rows}\n".encode(), (args.device, args.ctrl_port))
    started = time.monotonic()
    counts = {"data": 0, "log": 0, "ctrl": 0}
    restored_rows = False
    while time.monotonic() - started < args.duration:
        if args.set_rows and not restored_rows and time.monotonic() - started >= args.duration / 2:
            ctrl.sendto(b"ROWS=8\n", (args.device, args.ctrl_port))
            restored_rows = True
        readable, _, _ = select.select(list(sockets), [], [], 0.5)
        for sock in readable:
            payload, _ = sock.recvfrom(65535)
            channel = sockets[sock]
            counts[channel] += 1
            for line in payload.decode("ascii", errors="replace").splitlines():
                if channel == "data":
                    protocol.feed_line(line)
                if channel == "ctrl" or (channel == "log" and args.show_log):
                    print(f"{channel.upper()},{line}", flush=True)
    for sock in sockets:
        sock.close()
    print(protocol.summary("WIFI_DONE") +
          f",dataDatagram={counts['data']},logDatagram={counts['log']},ctrlDatagram={counts['ctrl']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
