#!/usr/bin/env python3
"""Validate SensorArray BLE control/data/log channels with bleak."""

from __future__ import annotations

import argparse
import asyncio
import time

from text_protocol import FragmentReassembler, TextProtocolParser, format_cap_preview

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
        if (args.show_cap or args.stream in ("data", "all")) else None)
    show_log = args.show_log or args.stream in ("log", "all")
    reassembler = FragmentReassembler()
    buffers = {"D": bytearray(), "L": bytearray(), "C": bytearray()}
    counts = {"data": 0, "log": 0, "ctrl": 0}

    def process_message(channel: str, payload: bytes) -> None:
        buffers[channel].extend(payload)
        while b"\n" in buffers[channel]:
            raw, _, remainder = buffers[channel].partition(b"\n")
            buffers[channel][:] = remainder
            line = raw.rstrip(b"\r").decode("ascii", errors="replace")
            if channel == "D":
                protocol.feed_line(line)
            elif channel == "C" or (channel == "L" and show_log):
                print(f"{channel},{line}", flush=True)

    def callback(channel: str, fallback: str):
        def receive(_sender: object, payload: bytearray) -> None:
            counts[channel] += 1
            for out_channel, message in reassembler.feed(fallback, bytes(payload)):
                process_message(out_channel, message)
        return receive

    try:
        devices = await BleakScanner.discover(timeout=args.scan_seconds)
    except Exception as error:
        reason = "no_adapter" if "adapter" in str(error).lower() else "scan_failed"
        print(f"BLE_TEST_SKIPPED reason={reason} detail={error}")
        return 2
    prefixes = [args.name_prefix]
    if args.compat_prefix and args.compat_prefix not in prefixes:
        prefixes.append(args.compat_prefix)
    device = next(
        (item for item in devices if any((item.name or "").startswith(prefix) for prefix in prefixes)),
        None,
    )
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
            await client.start_notify(CTRL_TX, callback("ctrl", "C"))
            await client.start_notify(DATA_TX, callback("data", "D"))
            await client.start_notify(LOG_TX, callback("log", "L"))
            mtu = getattr(client, "mtu_size", 0)
            print(f"BL,mtu={mtu},phy=unk,ci=unk,sub=DLC,ok=1", flush=True)
            if args.show_cap:
                print("BLECAP,service=00FF,ctrl=FF10/FF11,data=FF20,log=FF30")
            if args.tx:
                await client.write_gatt_char(CTRL_RX, f"TX={args.tx}\n".encode(), response=True)
            await client.write_gatt_char(CTRL_RX, b"ST=ble\n", response=True)
            if args.set_rows:
                await client.write_gatt_char(CTRL_RX, f"ROWS={args.set_rows}\n".encode(), response=True)
            rows = [int(item) for item in args.rows.split(",") if item] if args.rows else []
            row_index = 0
            next_row_at = started
            restored_rows = False
            while time.monotonic() - started < args.duration:
                if rows and time.monotonic() >= next_row_at:
                    await client.write_gatt_char(CTRL_RX, f"ROWS={rows[row_index % len(rows)]}\n".encode(), response=True)
                    row_index += 1
                    next_row_at = time.monotonic() + max(args.duration / max(len(rows), 1), 1.0)
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
    print(reassembler.summary("D"))
    print(reassembler.summary("L"))
    print(reassembler.summary("C"))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name-prefix", default="CscArray_")
    parser.add_argument("--compat-prefix", default="SensorArray_")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--scan-seconds", type=float, default=10.0)
    parser.add_argument("--set-rows", type=int, choices=range(1, 9))
    parser.add_argument("--rows", default="")
    parser.add_argument("--stream", choices=("data", "log", "all"))
    parser.add_argument("--tx", choices=("rel", "rt"))
    parser.add_argument("--show-cap", action="store_true")
    parser.add_argument("--show-log", action="store_true")
    args = parser.parse_args()
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
