#!/usr/bin/env python3
"""Validate SensorArray Wi-Fi text data/log/control channels."""

from __future__ import annotations

import argparse
import select
import socket
import time

from text_protocol import FragmentReassembler, TextProtocolParser, format_cap_preview


def first_non_ascii(payload: bytes) -> tuple[int, int] | None:
    for offset, value in enumerate(payload):
        if value > 0x7F:
            return offset, value
    return None


def receiver(port: int) -> socket.socket:
    result = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    result.bind(("0.0.0.0", port))
    result.setblocking(False)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="192.168.4.1")
    parser.add_argument("--host")
    parser.add_argument("--ip")
    parser.add_argument("--data-port", type=int, default=3333)
    parser.add_argument("--log-port", type=int, default=3334)
    parser.add_argument("--ctrl-port", type=int, default=3335)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--set-rows", type=int, choices=range(1, 9))
    parser.add_argument("--rows", default="")
    parser.add_argument("--stream", choices=("data", "log", "all"))
    parser.add_argument("--tx", choices=("rel", "rt"))
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()
    target = args.ip or args.host or args.device

    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if (args.show_cap or args.stream in ("data", "all")) else None)
    show_log = args.show_log or args.stream in ("log", "all")
    reassembler = FragmentReassembler()
    buffers = {"D": bytearray(), "L": bytearray(), "C": bytearray()}

    def process_message(ch: str, payload: bytes) -> None:
        ascii_error = first_non_ascii(payload)
        if ascii_error is not None:
            offset, value = ascii_error
            protocol.note_non_ascii()
            print(f"WIFI_ASCII_ERR,ch={ch},stage=message,off={offset},byte={value:02X},len={len(payload)}",
                  flush=True)
            return
        buffers[ch].extend(payload)
        while b"\n" in buffers[ch]:
            raw, _, remainder = buffers[ch].partition(b"\n")
            buffers[ch][:] = remainder
            raw_line = raw.rstrip(b"\r")
            try:
                line = raw_line.decode("ascii")
            except UnicodeDecodeError as error:
                value = raw_line[error.start] if error.start < len(raw_line) else 0
                protocol.note_non_ascii()
                print(f"WIFI_ASCII_ERR,ch={ch},stage=line,off={error.start},"
                      f"byte={value:02X},len={len(raw_line)}", flush=True)
                continue
            if ch == "D":
                protocol.feed_line(line)
            elif ch == "C" or (ch == "L" and show_log):
                print(f"{ch},{line}", flush=True)
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
    if args.tx:
        ctrl.sendto(f"TX={args.tx}\n".encode(), (target, args.ctrl_port))
    ctrl.sendto(b"ST=wifi\n", (target, args.ctrl_port))
    if args.set_rows:
        ctrl.sendto(f"ROWS={args.set_rows}\n".encode(), (target, args.ctrl_port))
    started = time.monotonic()
    counts = {"data": 0, "log": 0, "ctrl": 0}
    rows = [int(item) for item in args.rows.split(",") if item] if args.rows else []
    row_index = 0
    next_row_at = started
    restored_rows = False
    while time.monotonic() - started < args.duration:
        if rows and time.monotonic() >= next_row_at:
            ctrl.sendto(f"ROWS={rows[row_index % len(rows)]}\n".encode(), (target, args.ctrl_port))
            row_index += 1
            next_row_at = time.monotonic() + max(args.duration / max(len(rows), 1), 1.0)
        if args.set_rows and not restored_rows and time.monotonic() - started >= args.duration / 2:
            ctrl.sendto(b"ROWS=8\n", (target, args.ctrl_port))
            restored_rows = True
        readable, _, _ = select.select(list(sockets), [], [], 0.5)
        for sock in readable:
            payload, _ = sock.recvfrom(65535)
            channel = sockets[sock]
            counts[channel] += 1
            ascii_error = first_non_ascii(payload)
            if ascii_error is not None:
                offset, value = ascii_error
                protocol.note_non_ascii()
                print(f"WIFI_ASCII_ERR,ch={channel},stage=datagram,off={offset},"
                      f"byte={value:02X},len={len(payload)}", flush=True)
            fallback = {"data": "D", "log": "L", "ctrl": "C"}[channel]
            for out_channel, message in reassembler.feed(fallback, payload):
                process_message(out_channel, message)
    for sock in sockets:
        sock.close()
    print(protocol.summary("WIFI_DONE") +
          f",dataDatagram={counts['data']},logDatagram={counts['log']},ctrlDatagram={counts['ctrl']}")
    print(reassembler.summary("D"))
    print(reassembler.summary("L"))
    print(reassembler.summary("C"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
