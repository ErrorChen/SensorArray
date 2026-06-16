#!/usr/bin/env python3
"""Receive and reassemble SensorArray BLE text streams with bleak."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import time
from pathlib import Path
from typing import TextIO

from text_protocol import FragmentReassembler, TextProtocolParser, format_cap_preview

SERVICE = "000000ff-0000-1000-8000-00805f9b34fb"
CTRL_RX = "0000ff10-0000-1000-8000-00805f9b34fb"
CTRL_TX = "0000ff11-0000-1000-8000-00805f9b34fb"
DATA_TX = "0000ff20-0000-1000-8000-00805f9b34fb"
LOG_TX = "0000ff30-0000-1000-8000-00805f9b34fb"


class BleTextReceiver:
    def __init__(self, args: argparse.Namespace, log_file: TextIO | None) -> None:
        self.args = args
        self.log_file = log_file
        self.reassembler = FragmentReassembler()
        self.parser = TextProtocolParser(on_cap_frame=self._on_cap_frame)
        self.buffers = {"D": bytearray(), "L": bytearray(), "C": bytearray()}
        self.notify_count = {"D": 0, "L": 0, "C": 0}
        self.bytes_rx = 0
        self.started = time.monotonic()

    def _write(self, line: str) -> None:
        print(line, flush=True)
        if self.log_file:
            self.log_file.write(line + "\n")
            self.log_file.flush()

    def _on_cap_frame(self, frame) -> None:  # type: ignore[no-untyped-def]
        if self.args.tail:
            self._write(format_cap_preview(frame))

    def process_message(self, channel: str, payload: bytes) -> None:
        self.buffers[channel].extend(payload)
        while b"\n" in self.buffers[channel]:
            raw, _, remainder = self.buffers[channel].partition(b"\n")
            self.buffers[channel][:] = remainder
            line = raw.rstrip(b"\r").decode("ascii", errors="replace")
            if channel == "D":
                self.parser.feed_line(line)
                if not self.args.tail:
                    self._write(line)
            elif channel == "L":
                self._write(line)
            else:
                self._write(line)

    def callback(self, fallback: str):
        def receive(_sender: object, payload: bytearray) -> None:
            payload_bytes = bytes(payload)
            self.notify_count[fallback] += 1
            self.bytes_rx += len(payload_bytes)
            if self.args.show_fragments:
                self._write(f"FRAG,ch={fallback},n={len(payload_bytes)},data={payload_bytes!r}")
            for channel, message in self.reassembler.feed(fallback, payload_bytes):
                self.process_message(channel, message)

        return receive

    def stats_line(self) -> str:
        stats = list(self.reassembler.stats.values())
        ok = sum(item.ok for item in stats)
        missing = sum(item.missing for item in stats)
        gap = sum(item.gap for item in stats)
        crc = sum(item.crc_fail for item in stats)
        tiny = sum(item.tiny for item in stats)
        duplicate = sum(item.duplicate for item in stats)
        elapsed = max(time.monotonic() - self.started, 0.001)
        fps = self.parser.counters.cap_frames / elapsed
        return (f"BRX,ok={ok},miss={missing},gap={gap},crc={crc},tiny={tiny},"
                f"dup={duplicate},fps={fps:.2f},bytes={self.bytes_rx}")


async def write_command(client, command: str) -> None:  # type: ignore[no-untyped-def]
    if not command.endswith("\n"):
        command += "\n"
    await client.write_gatt_char(CTRL_RX, command.encode("ascii"), response=True)


async def interactive_loop(client, stop_event: asyncio.Event) -> None:  # type: ignore[no-untyped-def]
    while not stop_event.is_set():
        try:
            command = await asyncio.to_thread(input)
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            return
        command = command.strip()
        if not command:
            continue
        if command.upper() in {"QUIT", "EXIT"}:
            stop_event.set()
            return
        await write_command(client, command)


async def stats_loop(receiver: BleTextReceiver, stop_event: asyncio.Event) -> None:
    while not stop_event.is_set():
        await asyncio.sleep(5.0)
        receiver._write(receiver.stats_line())


async def run(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("BLE_TEST_SKIPPED reason=missing_bleak")
        return 2

    log_file: TextIO | None = None
    if args.save_log:
        Path(args.save_log).parent.mkdir(parents=True, exist_ok=True)
        log_file = open(args.save_log, "a", encoding="utf-8")

    receiver = BleTextReceiver(args, log_file)
    try:
        devices = await BleakScanner.discover(timeout=args.scan_seconds)
    except Exception as error:
        print(f"BLE_TEST_SKIPPED reason=scan_failed detail={error}")
        if log_file:
            log_file.close()
        return 2

    prefixes = [args.name_prefix]
    device = next(
        (item for item in devices if any((item.name or "").startswith(prefix) for prefix in prefixes)),
        None,
    )
    if device is None:
        print("BLE_TEST_SKIPPED reason=device_not_found")
        if log_file:
            log_file.close()
        return 2

    receiver._write(f"BLE_DEVICE,name={device.name},address={device.address}")
    stop_event = asyncio.Event()
    try:
        async with BleakClient(device, timeout=15.0,
                               winrt={"use_cached_services": False}) as client:
            uuids = {service.uuid.lower() for service in client.services}
            if SERVICE not in uuids:
                receiver._write("BLE_TEST_SKIPPED reason=service_missing")
                return 2

            await client.start_notify(CTRL_TX, receiver.callback("C"))
            await client.start_notify(DATA_TX, receiver.callback("D"))
            await client.start_notify(LOG_TX, receiver.callback("L"))
            mtu = getattr(client, "mtu_size", 0)
            receiver._write(f"BL,mtu={mtu},sub=11/20/30,ok=1")

            await write_command(client, "ST=AUTO")
            await write_command(client, f"BTX={'SAFE' if args.safe else 'FAST'}")
            await write_command(client, f"TX={args.tx}")
            if args.rows:
                await write_command(client, f"ROWS={args.rows}")

            tasks = [asyncio.create_task(stats_loop(receiver, stop_event))]
            if args.interactive:
                tasks.append(asyncio.create_task(interactive_loop(client, stop_event)))

            deadline = time.monotonic() + args.duration if args.duration > 0 else None
            while not stop_event.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    stop_event.set()
                    break
                await asyncio.sleep(0.1)

            for task in tasks:
                task.cancel()
            for task in tasks:
                with contextlib.suppress(asyncio.CancelledError):
                    await task
    except Exception as error:
        receiver._write(f"BLE_TEST_SKIPPED reason=connect_failed detail={error}")
        return 2
    finally:
        receiver._write(receiver.stats_line())
        receiver._write(receiver.reassembler.summary("D"))
        receiver._write(receiver.reassembler.summary("L"))
        receiver._write(receiver.reassembler.summary("C"))
        if log_file:
            log_file.close()
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name-prefix", default="CscArray")
    parser.add_argument("--rows", type=int, choices=range(1, 9))
    parser.add_argument("--tx", choices=("SHORT", "REL", "FULL"), default="REL")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--tail", action="store_true")
    parser.add_argument("--show-fragments", action="store_true")
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--save-log")
    parser.add_argument("--safe", action="store_true")
    return parser.parse_args()


def main() -> int:
    return asyncio.run(run(parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
