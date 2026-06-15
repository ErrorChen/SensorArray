#!/usr/bin/env python3
"""Receive SensorArray compact ASCII notifications with bleak."""

from __future__ import annotations

import argparse
import asyncio
import time

from text_protocol import TextProtocolParser, format_cap_preview


DEFAULT_SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"
DEFAULT_STATUS_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"
DEFAULT_NOTIFY_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"


async def run(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("BLE_TEST_SKIPPED reason=missing_bleak")
        return 2

    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if args.show_cap else None
    )
    receive_buffer = bytearray()
    notify_count = 0
    start_time = time.monotonic()

    def on_notify(_sender: object, payload: bytearray) -> None:
        nonlocal notify_count
        notify_count += 1
        receive_buffer.extend(payload)
        while b"\n" in receive_buffer:
            raw_line, _, remainder = receive_buffer.partition(b"\n")
            receive_buffer[:] = remainder
            try:
                line = raw_line.rstrip(b"\r").decode("ascii")
            except UnicodeDecodeError:
                protocol.note_non_ascii()
                line = raw_line.rstrip(b"\r").decode("ascii", errors="ignore")
            if not args.quiet:
                print(line, flush=True)
            protocol.feed_line(line)

    try:
        devices = await BleakScanner.discover(timeout=args.scan_seconds)
    except Exception as error:
        print(f"BLE_TEST_SKIPPED reason=scan_failed detail={error}")
        return 2

    device = next((candidate for candidate in devices
                   if (candidate.name or "").startswith(args.name)), None)
    if device is None:
        print(f"BLE_TEST_SKIPPED reason=device_not_found name={args.name}")
        return 2

    print(f"BLE_FOUND,name={device.name},address={device.address}", flush=True)
    try:
        async with BleakClient(device, timeout=args.connect_seconds) as client:
            services = client.services
            service_uuids = {service.uuid.lower() for service in services}
            if args.service.lower() not in service_uuids:
                print(f"BLE_LIMITED reason=service_missing uuid={args.service}")
            await client.start_notify(args.notify, on_notify)
            if args.cap_period is not None:
                await client.write_gatt_char(args.status,
                                             f"BLECAP={args.cap_period}\n".encode("ascii"),
                                             response=True)
                print(f"BLE_COMMAND,name=blecap,value={args.cap_period}", flush=True)
            print(f"BLE_NOTIFY,uuid={args.notify}", flush=True)
            next_summary = time.monotonic() + args.summary_seconds
            while args.duration <= 0 or time.monotonic() - start_time < args.duration:
                await asyncio.sleep(0.2)
                if time.monotonic() >= next_summary:
                    elapsed = max(time.monotonic() - start_time, 0.001)
                    print(protocol.summary("BLE5") + f",notifyHz={notify_count / elapsed:.2f}",
                          flush=True)
                    next_summary = time.monotonic() + args.summary_seconds
            await client.stop_notify(args.notify)
    except Exception as error:
        print(f"BLE_TEST_SKIPPED reason=connect_or_notify_failed detail={error}")
        return 2

    if receive_buffer:
        protocol.counters.malformed += 1
    elapsed = max(time.monotonic() - start_time, 0.001)
    print(protocol.summary("BLE_DONE") + f",notifyHz={notify_count / elapsed:.2f}", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="SensorArray")
    parser.add_argument("--service", default=DEFAULT_SERVICE_UUID)
    parser.add_argument("--status", default=DEFAULT_STATUS_UUID)
    parser.add_argument("--notify", default=DEFAULT_NOTIFY_UUID)
    parser.add_argument("--scan-seconds", type=float, default=8.0)
    parser.add_argument("--connect-seconds", type=float, default=15.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to run; zero means until Ctrl-C")
    parser.add_argument("--summary-seconds", type=float, default=5.0)
    parser.add_argument("--cap-period", type=int,
                        help="Write BLECAP=N after connecting; zero disables cap frames")
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
