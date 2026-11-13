#!/usr/bin/env python3
"""Exact-board bounded BLE control evidence; no rate/mode/calibration changes.

USBBOOT is sent only with the explicit --rom-download flag, after BUILD and
PROTO replies. Raw notifications are retained before fragment reassembly.
"""
import argparse
import asyncio
import base64
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
from receive_ble_text import SERVICE, CTRL_RX, CTRL_TX, LOG_TX
from text_protocol import FragmentReassembler

async def run(args):
    from bleak import BleakClient, BleakScanner
    out = Path(__file__).resolve().parents[1]
    path = out / (args.label + '.ble-control.jsonl')
    report = out / (args.label + '.ble-control.json')
    if path.exists() or report.exists():
        raise FileExistsError('Use a fresh evidence label')
    result = {'address': '80:B5:4E:CD:D9:AE', 'name': 'CscArray_CDD9AD',
              'status': 'RUNNING', 'commands': [], 'replies': {},
              'sudo_used': False, 'raw_usb_used': False,
              'rom_download_requested': args.rom_download}
    reassembler = FragmentReassembler()
    buffers = {'C': bytearray(), 'L': bytearray()}
    queue = asyncio.Queue()
    disconnected = asyncio.Event()
    started = time.monotonic()
    with path.open('x') as stream:
        def event(kind, **fields):
            stream.write(json.dumps({'type': kind, 'monotonic': time.monotonic(), **fields}) + '\n')
            stream.flush()
        def callback(channel):
            def received(_sender, payload):
                event('notification', channel=channel,
                      payload_base64=base64.b64encode(payload).decode('ascii'))
                for logical, message in reassembler.feed(channel, bytes(payload)):
                    if logical not in buffers:
                        continue
                    buffers[logical].extend(message)
                    while b'\n' in buffers[logical]:
                        line, _, rest = buffers[logical].partition(b'\n')
                        buffers[logical][:] = rest
                        text = line.rstrip(b'\r').decode('ascii', errors='backslashreplace')
                        event('line', channel=logical, line=text)
                        queue.put_nowait(text)
            return received
        def on_disconnect(_client):
            event('disconnected'); disconnected.set()
        try:
            device = await asyncio.wait_for(BleakScanner.find_device_by_address(
                result['address'], timeout=8, adapter='hci0'), 10)
            if not device or device.address.upper() != result['address'] or device.name != result['name']:
                raise RuntimeError('exact_board_not_found')
            async with BleakClient(device, disconnected_callback=on_disconnect, timeout=15) as client:
                if SERVICE not in {s.uuid.lower() for s in client.services}:
                    raise RuntimeError('service_mismatch')
                for ch, uuid in [('C', CTRL_TX), ('L', LOG_TX)]:
                    await asyncio.wait_for(client.start_notify(uuid, callback(ch)), 5)
                commands = ['BUILD?', 'PROTO?', 'CONFIG?', 'BOOT?', 'USB?', 'USBQ?', 'SPI?', 'FRONTENDS?', 'MODE?', 'BLELINK?', 'BLETX?']
                if args.rom_download:
                    commands = ['BUILD?', 'PROTO?', 'USB?', 'USBBOOT']
                for command in commands:
                    if command == 'USBBOOT':
                        proto = result['replies'].get('PROTO?', '')
                        usb = result['replies'].get('USB?', '')
                        if 'usb_rom_handoff_v1' not in proto or ',version=2,' not in proto or ',romAvailable=1,' not in usb:
                            raise RuntimeError('ROM_download_capability_not_confirmed')
                    prefix = 'RACK,cmd=RESTART,' if command == 'USBBOOT' else command.rstrip('?') + ','
                    while not queue.empty():
                        queue.get_nowait()
                    result['commands'].append(command)
                    event('tx', command=command)
                    await asyncio.wait_for(client.write_gatt_char(CTRL_RX, (command + '\n').encode(), response=True), 5)
                    deadline = time.monotonic() + 5
                    reply = None
                    while time.monotonic() < deadline:
                        line = await asyncio.wait_for(queue.get(), max(0.001, deadline - time.monotonic()))
                        if line.startswith(prefix):
                            reply = line; break
                        if line.startswith('ERR,'):
                            raise RuntimeError(command + ':' + line)
                    if reply is None:
                        raise TimeoutError('reply:' + command)
                    result['replies'][command] = reply
                    print(reply, flush=True)
                    if command == 'USBBOOT':
                        await asyncio.wait_for(disconnected.wait(), 10)
                        result['expected_disconnect_observed'] = True
                        break
                result['status'] = 'OBSERVED'
        except Exception as error:
            result.update(status='ERROR', error=repr(error))
        finally:
            event('finished', status=result['status'])
    result.update(elapsed_seconds=time.monotonic() - started,
                  journal=str(path.relative_to(ROOT)), journal_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with report.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({'status': result['status'], 'error': result.get('error'), 'report': str(report)}), flush=True)
    return 0 if result['status'] == 'OBSERVED' else 1

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    parser.add_argument('--rom-download', action='store_true')
    args = parser.parse_args()
    if not all(c.isalnum() or c in '-_' for c in args.label):
        raise ValueError('invalid label')
    return asyncio.run(asyncio.wait_for(run(args), 100))

if __name__ == '__main__':
    raise SystemExit(main())
