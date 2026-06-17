#!/usr/bin/env python3
"""Receive and reassemble SensorArray BLE text streams with bleak."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import threading
import time
from pathlib import Path
from typing import TextIO

from text_protocol import FragmentReassembler, TextProtocolParser, format_cap_preview

SERVICE = "000000ff-0000-1000-8000-00805f9b34fb"
CTRL_RX = "0000ff10-0000-1000-8000-00805f9b34fb"
CTRL_TX = "0000ff11-0000-1000-8000-00805f9b34fb"
DATA_TX = "0000ff20-0000-1000-8000-00805f9b34fb"
LOG_TX = "0000ff30-0000-1000-8000-00805f9b34fb"


def first_non_ascii(payload: bytes) -> tuple[int, int] | None:
    for offset, value in enumerate(payload):
        if value > 0x7F:
            return offset, value
    return None


def parse_fragment_identity(fallback: str, payload: bytes) -> tuple[str, int, int]:
    if not payload.startswith(b"G,"):
        return fallback, -1, -1
    header = payload.split(b"\n", 1)[0]
    try:
        fields = header.decode("ascii").split(",")
        return fields[1], int(fields[2], 10), int(fields[3], 10)
    except (IndexError, ValueError, UnicodeDecodeError):
        return fallback, -1, -1


class SerialSidecar:
    def __init__(self, port: str | None, baud: int, log_file: TextIO | None) -> None:
        self.port = port
        self.baud = baud
        self.log_file = log_file
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.reset_count = 0
        self.last_rst = ""
        self.last_line = ""
        self.last_error = ""

    def start(self) -> None:
        if not self.port:
            return
        self.thread = threading.Thread(target=self._run, name="serial-sidecar", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)

    def _write(self, line: str) -> None:
        print(line, flush=True)
        if self.log_file:
            self.log_file.write(line + "\n")
            self.log_file.flush()

    def _run(self) -> None:
        try:
            import serial
        except ImportError:
            self.last_error = "missing_pyserial"
            self._write("SER_SIDE,reason=missing_pyserial")
            return
        try:
            connection = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as error:
            self.last_error = f"open_failed:{error}"
            self._write(f"SER_SIDE,reason=open_failed,detail={error}")
            return
        self._write(f"SER_SIDE,port={self.port},baud={self.baud},status=open")
        try:
            while not self.stop_event.is_set():
                raw = connection.readline()
                if not raw:
                    continue
                try:
                    line = raw.rstrip(b"\r\n").decode("ascii")
                except UnicodeDecodeError as error:
                    value = raw[error.start] if error.start < len(raw) else 0
                    self._write(f"SER_ASCII_ERR,off={error.start},byte={value:02X}")
                    continue
                self.last_line = line
                if line.startswith("RST,"):
                    self.reset_count += 1
                    self.last_rst = line
                    self._write(f"SER,{line}")
        finally:
            connection.close()


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
        self.phase = "init"
        self.connected_once = False
        self.services_resolved = False
        self.notifications_started = False
        self.disconnected = False
        self.disconnect_at = 0.0
        self.disconnect_detail = ""
        self.last_notify_at = 0.0
        self.last_control_command = ""

    def _write(self, line: str) -> None:
        print(line, flush=True)
        if self.log_file:
            self.log_file.write(line + "\n")
            self.log_file.flush()

    def _on_cap_frame(self, frame) -> None:  # type: ignore[no-untyped-def]
        if self.args.tail:
            self._write(format_cap_preview(frame))

    def process_message(self, channel: str, payload: bytes) -> None:
        ascii_error = first_non_ascii(payload)
        if ascii_error is not None:
            offset, value = ascii_error
            self.parser.note_non_ascii()
            self._write(f"BLX,ch={channel},reason=nonascii_message,phase={self.phase},"
                        f"off={offset},byte={value:02X},len={len(payload)}")
            return
        self.buffers[channel].extend(payload)
        while b"\n" in self.buffers[channel]:
            raw, _, remainder = self.buffers[channel].partition(b"\n")
            self.buffers[channel][:] = remainder
            raw_line = raw.rstrip(b"\r")
            try:
                line = raw_line.decode("ascii")
            except UnicodeDecodeError as error:
                value = raw_line[error.start] if error.start < len(raw_line) else 0
                self.parser.note_non_ascii()
                self._write(f"BLX,ch={channel},reason=nonascii_line,phase={self.phase},"
                            f"off={error.start},byte={value:02X},len={len(raw_line)}")
                continue
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
            self.last_notify_at = time.monotonic()
            self.notify_count[fallback] += 1
            self.bytes_rx += len(payload_bytes)
            if self.args.show_fragments:
                self._write(f"FRAG,ch={fallback},n={len(payload_bytes)},data={payload_bytes!r}")
            ascii_error = first_non_ascii(payload_bytes)
            if ascii_error is not None:
                channel, mid, frag_index = parse_fragment_identity(fallback, payload_bytes)
                offset, value = ascii_error
                self.parser.note_non_ascii()
                self._write(f"BLX,ch={channel},reason=nonascii_fragment,phase={self.phase},"
                            f"mid={mid},frag={frag_index},off={offset},byte={value:02X},"
                            f"len={len(payload_bytes)}")
            for channel, message in self.reassembler.feed(fallback, payload_bytes):
                self.process_message(channel, message)

        return receive

    def on_disconnect(self, client) -> None:  # type: ignore[no-untyped-def]
        self.disconnected = True
        self.disconnect_at = time.monotonic()
        self.disconnect_detail = repr(client)

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
                f"dup={duplicate},fps={fps:.2f},bytes={self.bytes_rx},phase={self.phase}")

    def classify_error(self, error: Exception, sidecar: SerialSidecar) -> str:
        if sidecar.reset_count > 0:
            return "device_reboot"
        if self.disconnected and self.notifications_started:
            return "runtime_disconnect"
        if self.connected_once and not self.notifications_started:
            return f"{self.phase}_failed"
        if not self.connected_once:
            return "connect_failed"
        if self.last_control_command:
            return f"control_{self.last_control_command.lower()}_failed"
        return "ble_runtime_error"


async def write_command(client, receiver: BleTextReceiver, command: str) -> None:  # type: ignore[no-untyped-def]
    if not command.endswith("\n"):
        command += "\n"
    receiver.last_control_command = command.strip().split("=", 1)[0].lower()
    await client.write_gatt_char(CTRL_RX, command.encode("ascii"), response=True)
    receiver.last_control_command = ""


async def interactive_loop(client, receiver: BleTextReceiver, stop_event: asyncio.Event) -> None:  # type: ignore[no-untyped-def]
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
        await write_command(client, receiver, command)


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
    sidecar = SerialSidecar(args.serial_port, args.serial_baud, log_file)
    sidecar.start()
    try:
        receiver.phase = "scan"
        devices = await BleakScanner.discover(timeout=args.scan_seconds)
    except Exception as error:
        print(f"BLE_TEST_SKIPPED reason=scan_failed detail={error}")
        sidecar.stop()
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
        sidecar.stop()
        if log_file:
            log_file.close()
        return 2

    receiver._write(f"BLE_DEVICE,name={device.name},address={device.address}")
    stop_event = asyncio.Event()
    return_code = 0
    try:
        receiver.phase = "connect"
        async with BleakClient(device, timeout=15.0,
                               disconnected_callback=receiver.on_disconnect,
                               winrt={"use_cached_services": False}) as client:
            receiver.connected_once = True
            receiver.phase = "services"
            uuids = {service.uuid.lower() for service in client.services}
            receiver.services_resolved = True
            if SERVICE not in uuids:
                receiver._write("BLE_TEST_SKIPPED reason=service_missing")
                return_code = 2
                return 2

            receiver.phase = "subscribe_ctrl"
            await client.start_notify(CTRL_TX, receiver.callback("C"))
            receiver.phase = "subscribe_data"
            await client.start_notify(DATA_TX, receiver.callback("D"))
            receiver.phase = "subscribe_log"
            await client.start_notify(LOG_TX, receiver.callback("L"))
            receiver.notifications_started = True
            mtu = getattr(client, "mtu_size", 0)
            receiver._write(f"BL,mtu={mtu},sub=11/20/30,ok=1")

            receiver.phase = "control_st"
            await write_command(client, receiver, "ST=AUTO")
            receiver.phase = "control_btx"
            await write_command(client, receiver, f"BTX={'SAFE' if args.safe else 'FAST'}")
            receiver.phase = "control_tx"
            await write_command(client, receiver, f"TX={args.tx}")
            if args.rows:
                receiver.phase = "control_rows"
                await write_command(client, receiver, f"ROWS={args.rows}")

            tasks = [asyncio.create_task(stats_loop(receiver, stop_event))]
            if args.interactive:
                tasks.append(asyncio.create_task(interactive_loop(client, receiver, stop_event)))

            receiver.phase = "stream"
            deadline = time.monotonic() + args.duration if args.duration > 0 else None
            while not stop_event.is_set():
                if receiver.disconnected:
                    reason = receiver.classify_error(RuntimeError("disconnected"), sidecar)
                    receiver._write(f"BLE_TEST_FAILED reason={reason},phase={receiver.phase},"
                                    f"lastNotifyAge={time.monotonic() - receiver.last_notify_at:.3f},"
                                    f"serialResets={sidecar.reset_count}")
                    return_code = 2
                    stop_event.set()
                    break
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
        reason = receiver.classify_error(error, sidecar)
        receiver._write(f"BLE_TEST_SKIPPED reason={reason},phase={receiver.phase},"
                        f"connected={int(receiver.connected_once)},subscribed={int(receiver.notifications_started)},"
                        f"serialResets={sidecar.reset_count},detail={error}")
        return 2
    finally:
        receiver._write(receiver.stats_line())
        receiver._write(receiver.reassembler.summary("D"))
        receiver._write(receiver.reassembler.summary("L"))
        receiver._write(receiver.reassembler.summary("C"))
        if sidecar.port:
            receiver._write(f"SER_SIDE_DONE,resets={sidecar.reset_count},lastRst={sidecar.last_rst or 'none'},"
                            f"lastError={sidecar.last_error or 'none'}")
        sidecar.stop()
        if log_file:
            log_file.close()
    return return_code


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
    parser.add_argument("--serial-port",
                        help="Optional serial monitor sidecar used to classify BLE drops caused by device resets.")
    parser.add_argument("--serial-baud", type=int, default=115200)
    return parser.parse_args()


def main() -> int:
    return asyncio.run(run(parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
