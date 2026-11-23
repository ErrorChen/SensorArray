"""One bounded read-only probe of the AP-failure USJ recovery enumeration."""
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureOptions, Collector, PosixPort
from check_software_recovery import usb_inventory

out = Path(__file__).resolve().parents[1]
label = 'wifi-failure-usj-01'
result = {'status': 'ERROR', 'commands': [], 'setters': 0, 'modem_reset_requested': False}
port = None
try:
    devices = usb_inventory()
    result['inventory'] = devices
    matches = [d for d in devices if d['serial'] == '80B54ECDD9AC' and d['pid'] == '1001']
    assert len(matches) == 1 and len(matches[0]['ttys']) == 1, 'unique USJ board required'
    with (out / (label + '.rx.bin')).open('xb') as raw, (out / (label + '.events.jsonl')).open('x') as events:
        port = PosixPort(matches[0]['ttys'][0])
        collector = Collector(CaptureOptions(matches[0]['ttys'][0], 15, out, label, command_timeout=2), port, raw, events)
        deadline = time.monotonic() + 15
        collector.total_deadline = deadline
        collector.phase = 'readonly_recovery_probe'
        result['replies'] = {}
        for command, tag in [('BUILD?', 'BUILD'), ('BOOT?', 'BOOT'), ('CONFIG?', 'CONFIG'), ('USB?', 'USB'), ('WIFIGUARD?', 'WIFIGUARD'), ('MEM?', 'MEM')]:
            result['commands'].append(command)
            result['replies'][command] = collector.query(command, tag, min(deadline, time.monotonic() + 2))
        result['status'] = 'READONLY_RECOVERY_QUERIES_OBSERVED'
except Exception as error:
    result['error'] = repr(error)
finally:
    if port:
        port.close()
    result['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with (out / (label + '.summary.json')).open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result))
raise SystemExit(0 if result['status'] == 'READONLY_RECOVERY_QUERIES_OBSERVED' else 1)
