"""Apply user-supplied representative rails in RAM, with bounded terminal/query verification.

One-off hardware procedure; no flash, reset, CAL=SAVE or raw-input overwrite.
The source ranges and limitations are in 20261113-user-inputs.json.
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

label = sys.argv[1]
if not re.fullmatch(r"[A-Za-z0-9_-]+", label):
    raise SystemExit("invalid label")
out = ROOT / "docs/audit/artifacts/boot_autotune_20261113"
paths = {suffix: out / (label + suffix) for suffix in (".rx.bin", ".events.jsonl", ".summary.json")}
if any(p.exists() for p in paths.values()):
    raise SystemExit("output exists; use a new label")
failure = None
accepted = None
applied = None
port = None
with paths[".rx.bin"].open("xb") as raw, paths[".events.jsonl"].open("x") as events, paths[".summary.json"].open("x") as report:
    try:
        port = PosixPort("/dev/ttyACM0")
        c = Collector(CaptureOptions("/dev/ttyACM0", 60, out, label), port, raw, events)
        proto = c.query("PROTO?", "PROTO", c.prepare_deadline)
        if proto.get("version") != "2":
            raise CaptureError("unexpected_protocol")
        before = c.query("CONFIG?", "CONFIG", c.prepare_deadline)
        c.query("CAL?", "CAL", c.prepare_deadline)
        c.transition("MODE", "CAP")
        mode = c.replies["MODE?"]
        if mode.get("intref") != "0" or mode.get("refmux") != "24":
            raise CaptureError("CAP_reference_state_unverified")
        ads = c.query("ADS?", "ADS", c.prepare_deadline)
        if ads.get("gap") != "on":
            raise CaptureError("unexpected_gap_policy:" + str(ads.get("gap")))
        c.query("RAIL?", "ARL", c.prepare_deadline)
        command = "RAILCFG=3350000,-1795000"
        deadline = c.command_deadline(c.prepare_deadline)
        c.lines.clear()
        c.send(command, deadline)
        terminals = {}
        while time.monotonic() < deadline:
            line = c.next_line(deadline)
            fields = parse_fields(line)
            tag = line.split(",", 1)[0]
            if tag == "ERR" or tag == "RERR":
                raise CaptureError("rail_rejected:" + line)
            if tag in {"RACK", "RAPP"} and fields.get("avdd") == "3350000" and fields.get("avss") == "-1795000":
                c.event("rail_terminal", line=line)
                if tag == "RACK":
                    accepted = fields.get("id")
                else:
                    terminals[fields.get("id")] = fields
            if accepted and accepted in terminals:
                applied = terminals[accepted]
                break
        if not applied or applied.get("state") != "applied":
            raise CaptureError("rail_terminal_timeout")
        rail = c.query("RAIL?", "ARL", c.prepare_deadline)
        expected = {"src": "external", "rail": "5145000", "avdd": "3350000", "avss": "-1795000", "rv": "1", "rs": "ok", "split": "external_measured"}
        if any(rail.get(k) != v for k, v in expected.items()):
            raise CaptureError("external_rail_query_mismatch")
        after = c.query("CONFIG?", "CONFIG", c.prepare_deadline)
        if (after.get("session"), after.get("bootId")) != (before.get("session"), before.get("bootId")):
            raise CaptureError("unexpected_boot_change")
        for cmd, tag in (("MODE?", "MODE"), ("BAT?", "ABAT"), ("CAL?", "CAL"), ("USBSTREAM?", "USBSTREAM")):
            c.query(cmd, tag, c.prepare_deadline)
        if c.replies["MODE?"].get("intref") != "0":
            raise CaptureError("INTREF_not_off_after_application")
    except Exception as error:
        failure = str(error)
    finally:
        if port is not None:
            port.close()
    raw.flush()
    result = {"status": "FAIL" if failure else "APPLIED_QUERY_VERIFIED", "error": failure,
              "accepted_id": accepted, "applied": applied, "volatile_rail_calibration_attempted": accepted is not None,
              "nonvolatile_calibration_written": False, "flash_or_reset_requested": False,
              "replies": c.replies if "c" in globals() else {},
              "raw_sha256": hashlib.sha256(paths[".rx.bin"].read_bytes()).hexdigest()}
    report.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
raise SystemExit(1 if failure else 0)
