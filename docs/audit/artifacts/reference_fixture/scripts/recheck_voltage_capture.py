"""Bounded passive-fixture VOLT capture after verified volatile external rails.

Extends only this one-off procedure; the general passive fixture CLI retains
its CAP/RES admission. No calibration writes, flash, reset or input flush.
"""
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
from capture_reference_fixture import CaptureError, CaptureOptions, Collector, PosixPort


class VoltageCollector(Collector):
    def query(self, command, tag, phase_deadline):
        fields = super().query(command, tag, phase_deadline)
        if command == "MODE?" and fields.get("active") == "VOLT":
            if fields.get("intref") != "0" or fields.get("refmux") != "24":
                raise CaptureError("VOLT_reference_state_unverified")
        return fields

    def transition(self, kind, value):
        if kind == "MODE" and value == "VOLT":
            super().transition("MODE", "CAP")
            rail = self.query("RAIL?", "ARL", self.prepare_deadline)
            expected = {"src": "external", "rail": "5145000", "avdd": "3350000", "avss": "-1795000", "rv": "1", "rs": "ok", "split": "external_measured"}
            if any(rail.get(k) != v for k, v in expected.items()):
                raise CaptureError("VOLT_requires_verified_user_external_rails")
            ads = self.query("ADS?", "ADS", self.prepare_deadline)
            if ads.get("gap") != "on":
                raise CaptureError("unexpected_gap_policy")
        super().transition(kind, value)


label, rows = sys.argv[1], int(sys.argv[2])
if not re.fullmatch(r"[A-Za-z0-9_-]+", label) or rows not in (1, 8):
    raise SystemExit("invalid label or rows")
out = ROOT / "docs/audit/artifacts/reference_fixture"
paths = {suffix: out / (label + suffix) for suffix in (".rx.bin", ".events.jsonl", ".summary.json")}
if any(p.exists() for p in paths.values()):
    raise SystemExit("output exists; choose a new label")
port = None
with paths[".rx.bin"].open("xb") as raw, paths[".events.jsonl"].open("x") as events, paths[".summary.json"].open("x") as report:
    try:
        port = PosixPort("/dev/ttyACM0")
        options = CaptureOptions("/dev/ttyACM0", 150, out, label, mode="VOLT", rows=rows)
        result = VoltageCollector(options, port, raw, events).run()
    except (OSError, CaptureError) as error:
        result = {"status": "ERROR", "error": str(error), "calibration_written": False}
    finally:
        if port is not None:
            port.close()
    raw.flush()
    result.update(raw_file=str(paths[".rx.bin"]), events_file=str(paths[".events.jsonl"]),
                  raw_sha256=hashlib.sha256(paths[".rx.bin"].read_bytes()).hexdigest(),
                  voltage_reference_input="20261113-user-inputs.json", absolute_voltage_acceptance="NOT_ESTABLISHED")
    report.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result.get(key) for key in ("status", "error", "capture_seconds", "rx_bytes", "usb_restore", "raw_file")}), flush=True)
raise SystemExit(0 if result["status"] == "CAPTURED" else 1)
