"""One bounded RES8 contrast: external rails, periodic battery temporarily OFF.

Original battery cadence and USB profile are terminal/query restored. This does
not disable CRC, watchdog, ADC safety checks or the capture rate. No NVS writes.
"""
import hashlib
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
from capture_reference_fixture import CaptureError, CaptureOptions, Collector, PosixPort
from text_protocol import parse_fields


def set_battery(c, value, enabled, phase_deadline):
    deadline = c.command_deadline(phase_deadline)
    command = "BATPERIOD=" + value
    c.lines.clear()
    c.send(command, deadline)
    accepted, terminals = None, {}
    while time.monotonic() < deadline:
        line = c.next_line(deadline)
        tag, fields = line.split(",", 1)[0], parse_fields(line)
        if tag == "ERR":
            raise CaptureError("battery_command_error:" + line)
        if tag == "ACK" and fields.get("cmd") == "BATPERIOD":
            accepted = fields.get("id")
            c.event("battery_accepted", line=line)
        if tag == "BATPERIOD" and "id" in fields:
            terminals[fields["id"]] = fields
            c.event("battery_terminal", line=line)
        if accepted and accepted in terminals:
            terminal = terminals[accepted]
            if terminal.get("status") != "applied" or terminal.get("enabled") != enabled:
                raise CaptureError("battery_terminal_rejected")
            snapshot = c.query("BATPERIOD?", "BATPERIOD", phase_deadline)
            if snapshot.get("enabled") != enabled:
                raise CaptureError("battery_query_mismatch")
            return snapshot
    raise CaptureError("battery_terminal_timeout")


label = sys.argv[1]
if not re.fullmatch(r"[A-Za-z0-9_-]+", label):
    raise SystemExit("invalid label")
out = ROOT / "docs/audit/artifacts/boot_autotune_20261113"
paths = {suffix: out / (label + suffix) for suffix in (".rx.bin", ".events.jsonl", ".summary.json")}
if any(p.exists() for p in paths.values()):
    raise SystemExit("output exists; choose a new label")
original, restore, armed, port = None, "NOT_NEEDED", False, None
result = {"status": "ERROR", "error": "not_started"}
with paths[".rx.bin"].open("xb") as raw, paths[".events.jsonl"].open("x") as events, paths[".summary.json"].open("x") as report:
    try:
        port = PosixPort("/dev/ttyACM0")
        c = Collector(CaptureOptions("/dev/ttyACM0", 150, out, label, mode="RES", rows=8), port, raw, events)
        rail = c.query("RAIL?", "ARL", c.prepare_deadline)
        if (rail.get("src"), rail.get("rv"), rail.get("rail")) != ("external", "1", "5145000"):
            raise CaptureError("external_rails_unverified")
        ads = c.query("ADS?", "ADS", c.prepare_deadline)
        if ads.get("gap") != "on":
            raise CaptureError("unexpected_gap_policy")
        original = c.query("BATPERIOD?", "BATPERIOD", c.prepare_deadline)
        if original.get("enabled") not in {"0", "1"}:
            raise CaptureError("unknown_battery_setting")
        if original.get("enabled") == "1" and not 100 <= int(original["periodMs"]) <= 600000:
            raise CaptureError("invalid_original_battery_period")
        armed = True  # Application can succeed even when its reply is lost.
        set_battery(c, "OFF", "0", c.prepare_deadline)
        c.query("BAT?", "ABAT", c.prepare_deadline)
        result = c.run()
    except Exception as error:
        result = {"status": "ERROR", "error": str(error)}
    finally:
        if armed and port is not None:
            try:
                c.phase = "cleanup"
                wanted = original["periodMs"] if original["enabled"] == "1" else "OFF"
                snapshot = set_battery(c, wanted, original["enabled"], c.total_deadline)
                if original["enabled"] == "1" and snapshot.get("periodMs") != original["periodMs"]:
                    raise CaptureError("battery_period_restore_mismatch")
                c.query("BAT?", "ABAT", c.total_deadline)
                restore = "VERIFIED_QUERY"
            except Exception as error:
                restore = "ERROR:" + str(error)
                result["status"] = "ERROR"
                result["error"] = str(result.get("error")) + ";battery_restore:" + str(error)
        if port is not None:
            port.close()
    raw.flush()
    result.update(raw_file=str(paths[".rx.bin"]), events_file=str(paths[".events.jsonl"]),
                  raw_sha256=hashlib.sha256(paths[".rx.bin"].read_bytes()).hexdigest(),
                  original_battery=original, battery_restore=restore,
                  calibration_written=False, adc_crc_and_watchdog_unchanged=True)
    if "c" in globals():
        result["replies"] = c.replies
    report.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result.get(key) for key in ("status", "error", "capture_seconds", "rx_bytes", "usb_restore", "battery_restore", "raw_file")}), flush=True)
raise SystemExit(0 if result["status"] == "CAPTURED" else 1)
