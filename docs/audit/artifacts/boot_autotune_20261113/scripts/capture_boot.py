"""Capture one explicitly requested firmware RESTART and read-only boot queries."""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
os.chdir(root)
sys.path.insert(0, str(root / "tools"))
from capture_reference_fixture import PosixPort, Collector, CaptureOptions, CaptureError

label = sys.argv[1]
rawpath = out / (label + ".rx.bin")
with rawpath.open("xb") as raw, (out / (label + ".events.jsonl")).open("x") as events:
    port = PosixPort("/dev/ttyACM0")
    collector = Collector(CaptureOptions("/dev/ttyACM0", 85, out, label), port, raw, events)
    collector.phase = "boot_capture"
    collector.event("reset_intent", method="ASCII_RESTART", count=1, modem_bit_ioctl=False)
    error = None
    try:
        collector.send("RESTART", collector.total_deadline)
        observe_until = collector.started + 60
        while time.monotonic() < observe_until:
            try:
                collector.pump(observe_until)
                collector.lines.clear()  # Raw bytes retained; bounded diagnostic-only parse queue.
            except (OSError, CaptureError) as exc:
                collector.event("serial_gap", error=str(exc))
                port.close()
                while time.monotonic() < observe_until:
                    try:
                        port = PosixPort("/dev/ttyACM0")
                        collector.port = port
                        collector.line_buffer.clear()  # Do not join lines across a disconnect.
                        collector.event("reopened", modem_bit_ioctl=False)
                        break
                    except OSError:
                        time.sleep(0.2)
                else:
                    raise CaptureError("reconnect_deadline")
        for command, tag in [("BUILD?", "BUILD"), ("PROTO?", "PROTO"),
                             ("SPI?", "SPI"), ("FRONTENDS?", "FRONTENDS"),
                             ("CONFIG?", "CONFIG"), ("BOOT?", "BOOT"),
                             ("PERF?", "PERF"), ("USBSTREAM?", "USBSTREAM")]:
            try:
                reply = collector.query(command, tag, collector.total_deadline)
                print(command, json.dumps(reply), flush=True)
            except (OSError, CaptureError) as exc:
                collector.event("query_error", command=command, error=str(exc))
                print(command, str(exc), flush=True)
    except (OSError, CaptureError) as exc:
        error = str(exc)
    finally:
        port.close()
    raw.flush()
    with (out / (label + ".summary.json")).open("x") as stream:
        json.dump({"error": error, "replies": collector.replies,
                   "reset_requested": "ASCII_RESTART_once", "modem_bit_ioctl": False,
                   "raw_sha256": hashlib.sha256(rawpath.read_bytes()).hexdigest(),
                   "firmware_faults": collector.faults,
                   "reset_observations": collector.reset_observations,
                   "calibration_written": False}, stream, indent=2)
    if error:
        raise SystemExit(error)
