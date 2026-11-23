"""Request USBBOOT once from a positively identified application USJ recovery.

This is not an esptool/flash helper. A replacement USB enumeration is evidence
of a new USB epoch, not proof of ROM execution; the flasher must verify MAC.
No modem-line toggles, input flush, retry, NVS or flash write is performed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureError, CaptureOptions, Collector, PosixPort
from sensorarray_capture_identity import CaptureImageAuthority, ExpectedCaptureIdentity
from text_protocol import parse_fields
from check_software_recovery import usb_inventory, epoch_key


def validate_recovery(replies, expected):
    authority = CaptureImageAuthority(expected)
    authority.verify(replies['BUILD?'], replies['CONFIG_open'], replies['CONFIG_close'])
    usb, restart = replies['USB?'], replies['RESTART?']
    if usb.get('backend') != 'serial_jtag' or usb.get('romAvailable') != '1':
        raise CaptureError('application_USJ_ROM_recovery_required')
    if (restart.get('session', '').upper() != expected.session or
            restart.get('bootId') != str(expected.boot_id) or restart.get('busy') != '0'):
        raise CaptureError('restart_identity_or_busy_mismatch')
    return authority


def is_rom_acceptance(line):
    if not line.startswith('RACK,'):
        return False
    fields = parse_fields(line)
    request_id = fields.get('id', '')
    return (fields.get('cmd') == 'RESTART' and fields.get('state') == 'accepted' and
            fields.get('target') == 'rom_download' and request_id.isascii() and
            request_id.isdecimal() and 0 < int(request_id) < (1 << 32))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for option in ('label', 'expected-elf', 'expected-session', 'expected-boot'):
        p.add_argument('--' + option, required=True)
    a = p.parse_args()
    if not all(c.isalnum() or c in '-_' for c in a.label):
        raise ValueError('safe_label_required')
    expected = ExpectedCaptureIdentity(a.expected_elf, a.expected_session, a.expected_boot)
    out = Path(__file__).resolve().parents[1]
    paths = {k: out / (a.label + suffix) for k, suffix in
             [('raw', '.rx.bin'), ('events', '.events.jsonl'), ('summary', '.summary.json')]}
    if any(path.exists() for path in paths.values()):
        raise FileExistsError('use_new_label')
    result = {'status': 'ERROR', 'expected': expected.output(), 'action_attempts': 0,
              'ROM_execution': 'UNVERIFIED', 'flash_written': False,
              'modem_reset_requested': False, 'input_flushed': False}
    started = time.monotonic()
    deadline = started + 35
    port = None
    with paths['raw'].open('xb') as raw, paths['events'].open('x') as events:
        def event(kind, **fields):
            events.write(json.dumps({'type': kind, 'monotonic': time.monotonic(), **fields}) + '\n')
            events.flush()
        try:
            devices = usb_inventory()
            result['initial_inventory'] = devices
            matches = [d for d in devices if d['serial'] == '80B54ECDD9AC' and d['pid'] == '1001']
            if len(matches) != 1 or len(matches[0]['ttys']) != 1:
                raise CaptureError('unique_USJ_board_required')
            old = matches[0]
            port = PosixPort(old['ttys'][0])
            collector = Collector(CaptureOptions(old['ttys'][0], 35, out, a.label, command_timeout=2), port, raw, events)
            collector.total_deadline = deadline
            collector.phase = 'readonly_identity'
            replies = {}
            for key, command, tag in [('CONFIG_open', 'CONFIG?', 'CONFIG'), ('BUILD?', 'BUILD?', 'BUILD'),
                                      ('USB?', 'USB?', 'USB'), ('RESTART?', 'RESTART?', 'RESTART'),
                                      ('CONFIG_close', 'CONFIG?', 'CONFIG')]:
                replies[key] = collector.query(command, tag, min(deadline, time.monotonic() + 2))
            result['replies'] = replies
            if collector.faults or collector.reset_observations:
                raise CaptureError('fault_or_reset_during_preflight')
            validate_recovery(replies, expected)
            if not any(epoch_key(d) == epoch_key(old) for d in usb_inventory()):
                raise CaptureError('USJ_epoch_changed_before_action')
            collector.phase = 'single_USBBOOT'
            # Discard only parsed, already-recorded lines. Never flush device
            # input or carry a previous application's RACK into this action.
            collector.lines.clear()
            result['action_attempts'] = 1
            event('action_attempt', command='USBBOOT')
            collector.send('USBBOOT', min(deadline, time.monotonic() + 2))
            try:
                reply = collector.wait(is_rom_acceptance,
                                       min(deadline, time.monotonic() + 3), 'USBBOOT')
                result['local_publication'] = parse_fields(reply)
            except Exception as error:
                result['publication_observation_error'] = str(error)
            port.close()
            port = None
            event('old_reader_closed')
            while time.monotonic() < deadline:
                devices = usb_inventory()
                matches = [d for d in devices if d['serial'] == '80B54ECDD9AC']
                if len(matches) == 1 and matches[0]['pid'] == '1001' and epoch_key(matches[0]) != epoch_key(old):
                    result.update(status='NEW_USJ_EPOCH_OBSERVED_ROM_UNVERIFIED', new_device=matches[0])
                    break
                time.sleep(0.1)
            if result['status'] == 'ERROR':
                raise CaptureError('no_replacement_USJ_epoch_before_deadline')
        except Exception as error:
            result['error'] = repr(error)
        finally:
            if port:
                port.close()
            event('finished', status=result['status'], action_attempts=result['action_attempts'])
    result.update(elapsed_seconds=time.monotonic() - started,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  inputs={str(paths[k]): hashlib.sha256(paths[k].read_bytes()).hexdigest() for k in ('raw', 'events')})
    with paths['summary'].open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({'status': result['status'], 'error': result.get('error'), 'summary': str(paths['summary'])}))
    return 0 if result['status'] == 'NEW_USJ_EPOCH_OBSERVED_ROM_UNVERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
