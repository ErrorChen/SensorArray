"""Observe an already authorized flash boot; never send a restart or setter."""
import hashlib
import json
from pathlib import Path
import sys
import time

root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "tools"))
from capture_reference_fixture import PosixPort, Collector, CaptureOptions, CaptureError

label = sys.argv[1]
rawpath = out / (label + ".rx.bin")
error = None
with rawpath.open("xb") as raw, (out / (label + ".events.jsonl")).open("x") as events:
    port = PosixPort("/dev/ttyACM0")
    collector = Collector(CaptureOptions("/dev/ttyACM0", 85, out, label), port, raw, events)
    collector.phase = "post_flash_boot_observation"
    try:
        observe_until = collector.started + 55
        while time.monotonic() < observe_until:
            collector.pump(observe_until)
            collector.lines.clear()
        for command, tag in [("BUILD?", "BUILD"), ("PROTO?", "PROTO"),
                             ("SPI?", "SPI"), ("FRONTENDS?", "FRONTENDS"),
                             ("CONFIG?", "CONFIG"), ("BOOT?", "BOOT"),
                             ("PERF?", "PERF"), ("USBSTREAM?", "USBSTREAM"),
                             ("SERIAL?", "SERIAL"), ("SCHED?", "SCHED"),
                             ("RECOVERY?", "RECOVERY"), ("RESTART?", "RESTART")]:
            fields = collector.query(command, tag, collector.total_deadline)
            print(command, json.dumps(fields), flush=True)
    except (OSError, CaptureError) as exc:
        error = str(exc)
    finally:
        port.close()
    raw.flush()
    with (out / (label + ".summary.json")).open("x") as stream:
        json.dump({"error": error, "replies": collector.replies,
                   "reset_requested": False, "modem_bit_ioctl": False,
                   "raw_sha256": hashlib.sha256(rawpath.read_bytes()).hexdigest(),
                   "firmware_faults": collector.faults,
                   "reset_observations": collector.reset_observations,
                   "calibration_written": False}, stream, indent=2)
if error:
    raise SystemExit(error)
