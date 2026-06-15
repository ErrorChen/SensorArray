#!/usr/bin/env python3
"""Validate SensorArray BLE control/data/log channels with bleak."""

from __future__ import annotations

import argparse
import asyncio
import time

from text_protocol import TextProtocolParser, format_cap_preview

SERVICE = "000000ff-0000-1000-8000-00805f9b34fb"
CTRL_RX = "0000ff10-0000-1000-8000-00805f9b34fb"
CTRL_TX = "0000ff11-0000-1000-8000-00805f9b34fb"
DATA_TX = "0000ff20-0000-1000-8000-00805f9b34fb"
LOG_TX = "0000ff30-0000-1000-8000-00805f9b34fb"


async def run(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("BLE_TEST_SKIPPED reason=venv_pip_install_failed")
        return 2

    protocol = TextProtocolParser(
        on_cap_frame=(lambda frame: print(format_cap_preview(frame), flush=True))
        if args.show_cap else None)
    buffers = {"data": bytearray(), "log": bytearray(), "ctrl": bytearray()}
    counts = {"data": 0, "log": 0, "ctrl": 0}

    def callback(channel: str):
        def receive(_sender: object, payload: bytearray) -> None:
            counts[channel] += 1
            buffers[channel].extend(payload)
            while b"\n" in buffers[channel]:
                raw, _, remainder = buffers[channel].partition(b"\n")
                buffers[channel][:] = remainder
                line = raw.rstrip(b"\r").decode("ascii", errors="replace")
                if channel == "data":
                    protocol.feed_line(line)
                if channel == "ctrl" or (channel == "log" and args.show_log):
                    print(f"{channel.upper()},{line}", flush=True)
        return receive

    try:
        devices = await BleakScanner.discover(timeout=args.scan_seconds)
    except Exception as error:
        reason = "no_adapter" if "adapter" in str(error).lower() else "scan_failed"
        print(f"BLE_TEST_SKIPPED reason={reason} detail={error}")
        return 2
    device = next((item for item in devices if (item.name or "").startswith(args.name)), None)
    if device is None:
        print("BLE_TEST_SKIPPED reason=device_not_found")
        return 2
    print(f"BLE_DEVICE,name={device.name},address={device.address}", flush=True)

    started = time.monotonic()
    try:
        async with BleakClient(device, timeout=15.0,
                               winrt={"use_cached_services": False}) as client:
            uuids = {service.uuid.lower() for service in client.services}
            if SERVICE not in uuids:
                services = ";".join(sorted(uuids)) or "none"
                print(f"BLE_TEST_SKIPPED reason=service_missing services={services}")
                return 2
            characteristics = {
                characteristic.uuid.lower()
                for service in client.services
                for characteristic in service.characteristics
            }
            missing = sorted({CTRL_RX, CTRL_TX, DATA_TX, LOG_TX} - characteristics)
            if missing:
                print("BLE_TEST_SKIPPED reason=characteristic_missing missing=" +
                      ";".join(missing))
                return 2
            await client.start_notify(CTRL_TX, callback("ctrl"))
            await client.start_notify(DATA_TX, callback("data"))
            await client.start_notify(LOG_TX, callback("log"))
            if args.show_cap:
                print("BLECAP,service=00FF,ctrl=FF10/FF11,data=FF20,log=FF30")
            if args.set_rows:
                await client.write_gatt_char(CTRL_RX, f"ROWS={args.set_rows}\n".encode(), response=True)
            restored_rows = False
            while time.monotonic() - started < args.duration:
                if args.set_rows and not restored_rows and time.monotonic() - started >= args.duration / 2:
                    await client.write_gatt_char(CTRL_RX, b"ROWS=8\n", response=True)
                    restored_rows = True
                await asyncio.sleep(0.2)
    except Exception as error:
        print(f"BLE_TEST_SKIPPED reason=connect_failed detail={error}")
        return 2
    elapsed = max(time.monotonic() - started, 0.001)
    print(protocol.summary("BLE_DONE") +
          f",dataNotify={counts['data']},logNotify={counts['log']},"
          f"ctrlNotify={counts['ctrl']},notifyHz={sum(counts.values()) / elapsed:.2f}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="SensorArray")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--set-rows", type=int, choices=range(1, 9))
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
