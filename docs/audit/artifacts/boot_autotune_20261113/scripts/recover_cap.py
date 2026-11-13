"""Explicit CAP recovery and read-only state queries, without reset/calibration."""
import hashlib
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "tools"))
from capture_reference_fixture import PosixPort, Collector, CaptureOptions

label = sys.argv[1]
rawpath = out / (label + ".rx.bin")
with rawpath.open("xb") as raw, (out / (label + ".events.jsonl")).open("x") as events:
    port = PosixPort("/dev/ttyACM0")
    c = Collector(CaptureOptions("/dev/ttyACM0", 45, out, label), port, raw, events)
    error = None
    try:
        c.transition("MODE", "CAP")
        c.transition("ROWS", 8)
        for command, tag in [("CONFIG?", "CONFIG"), ("FRONTENDS?", "FRONTENDS"),
                             ("SPI?", "SPI"), ("RAIL?", "ARL"),
                             ("MODE?", "MODE"), ("BAT?", "ABAT"),
                             ("PERF?", "PERF"), ("USBSTREAM?", "USBSTREAM")]:
            print(command, json.dumps(c.query(command, tag, c.total_deadline)), flush=True)
    except Exception as exc:
        error = str(exc)
        c.event("error", reason=error)
    finally:
        port.close()
    raw.flush()
    with (out / (label + ".summary.json")).open("x") as stream:
        json.dump({"status": "ERROR" if error else "CAP_CONFIRMED", "error": error,
                   "replies": c.replies, "raw_sha256": hashlib.sha256(rawpath.read_bytes()).hexdigest(),
                   "reset_requested": False, "calibration_written": False}, stream, indent=2)
    if error:
        raise SystemExit(error)
