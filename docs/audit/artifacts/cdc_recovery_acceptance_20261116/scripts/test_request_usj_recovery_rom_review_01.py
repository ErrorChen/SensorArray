"""Targeted review using the real entry/Collector and synthetic USB/time only.

This is not a hardware recording. No serial port, USB API, modem line or flash
operation is invoked. The old-ACK scenario verifies the focused review fix.
"""
from contextlib import redirect_stdout
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[5]
STAGE = ROOT / "docs/audit/artifacts/cdc_recovery_acceptance_20261116"
SOURCE = STAGE / "scripts/request_usj_recovery_rom.py"
source_bytes = SOURCE.read_bytes()
sys.path.insert(0, str(SOURCE.parent))
spec = importlib.util.spec_from_file_location("reviewed_usj_request", SOURCE)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)
recovery = json.loads((STAGE / "wifi-failure-usj-01.summary.json").read_text())["replies"]


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        value = self.now
        self.now += .00001
        return value

    def sleep(self, seconds):
        self.now += seconds


def wire(tag, fields):
    return (tag + "," + ",".join(k + "=" + str(v) for k, v in fields.items()) + "\n").encode()


def scenario(name):
    clock = Clock()
    sent, queue = [], []
    device = {"sysfs_path": "/SYNTHETIC/usb", "busnum": 3, "devnum": 37,
              "vid": "303a", "pid": "1001", "serial": "80B54ECDD9AC",
              "ttys": ["/dev/SYNTHETIC-USJ"]}
    config = copy.deepcopy(recovery["CONFIG?"])
    boot = config["bootId"]
    expected = recovery["BUILD?"]["elfSha256"]
    state = {"closed": False, "configs": 0, "late_at": None}

    class Port:
        def __init__(self, path):
            assert path == "/dev/SYNTHETIC-USJ"

        def metadata(self):
            return {"transport": "posix", "tx_acceptance": "synthetic_OS_acceptance"}

        def write_some(self, data, deadline):
            command = data.decode().strip()
            sent.append(command)
            if command == "CONFIG?":
                state["configs"] += 1
                values = dict(config)
                if name == "wrong_boot" and state["configs"] == 2:
                    values["bootId"] = str(int(boot) + 1)
                response = wire("CONFIG", values)
                if state["configs"] == 2 and name == "observed_reset":
                    response += b"RST,kind=synthetic\n"
                if state["configs"] == 2 and name == "old_application_ACK":
                    response += b"RACK,cmd=RESTART,id=77,state=accepted,target=application\n"
                queue.append(response)
            elif command == "BUILD?":
                queue.append(wire("BUILD", recovery["BUILD?"]))
            elif command == "USB?":
                queue.append(wire("USB", recovery["USB?"]))
            elif command == "RESTART?":
                queue.append(wire("RESTART", {"session": config["session"], "bootId": boot,
                    "busy": "1" if name == "restart_busy" else "0", "lastId": "77"}))
            elif command == "USBBOOT":
                if name == "fresh_ACK":
                    queue.append(b"RACK,cmd=RESTART,id=78,state=accepted,target=rom_download\n")
                elif name == "late_ACK":
                    state["late_at"] = clock.now + 1.5
                elif name == "wrong_target_fresh_ACK":
                    queue.append(b"RACK,cmd=RESTART,id=78,state=accepted,target=application\n")
            else:
                raise AssertionError("unexpected command " + command)
            return len(data)

        def read_some(self, maximum, deadline):
            if state["late_at"] is not None and clock.now >= state["late_at"]:
                queue.append(b"RACK,cmd=RESTART,id=78,state=accepted,target=rom_download\n")
                state["late_at"] = None
            if queue:
                return "data", queue.pop(0)
            clock.now += min(.25, max(0, deadline - clock.now))
            return None

        def close(self):
            state["closed"] = True

    def inventory():
        current = dict(device)
        if state["closed"] and name != "same_epoch_no_ACK":
            current["devnum"] += 1
        return [current]

    with tempfile.TemporaryDirectory(prefix="sensorarray-usj-review-") as directory:
        folder = Path(directory)
        (folder / "scripts").mkdir()
        copied = folder / "scripts/request_usj_recovery_rom.py"
        copied.write_bytes(source_bytes)
        argv = [str(copied), "--label", name, "--expected-elf", expected,
                "--expected-session", config["session"], "--expected-boot", boot]
        with patch.object(entry, "__file__", str(copied)), patch.object(entry, "PosixPort", Port), \
             patch.object(entry, "usb_inventory", inventory), patch.object(sys, "argv", argv), \
             patch.object(entry.time, "monotonic", clock.monotonic), \
             patch.object(entry.time, "sleep", clock.sleep), redirect_stdout(io.StringIO()):
            code = entry.main()
        result = json.loads((folder / (name + ".summary.json")).read_text())
    attempts = sent.count("USBBOOT")
    assert attempts <= 1 and result["action_attempts"] == attempts
    assert result["ROM_execution"] == "UNVERIFIED" and not result["flash_written"]
    if name in ("wrong_boot", "observed_reset", "restart_busy"):
        assert code == 1 and attempts == 0
    elif name == "same_epoch_no_ACK":
        assert code == 1 and attempts == 1
    else:
        assert code == 0 and attempts == 1
    if name in ("missing_ACK", "same_epoch_no_ACK", "old_application_ACK", "wrong_target_fresh_ACK"):
        assert "local_publication" not in result
    return {"scenario": name, "synthetic_only": True, "exit_code": code,
            "USBBOOT_writes": attempts, "status": result["status"],
            "ROM_execution": result["ROM_execution"],
            "local_publication": result.get("local_publication"),
            "publication_observation_error": result.get("publication_observation_error"),
            "error": result.get("error")}


def main():
    cases = [scenario(name) for name in ("fresh_ACK", "missing_ACK", "late_ACK",
        "old_application_ACK", "wrong_target_fresh_ACK", "wrong_boot", "observed_reset", "restart_busy", "same_epoch_no_ACK")]
    predicate_cases = []
    for value, accepted in (("1", True), ("4294967295", True), ("0", False),
                            ("4294967296", False), ("-1", False), ("١", False), ("", False)):
        actual = entry.is_rom_acceptance("RACK,cmd=RESTART,id=" + value + ",state=accepted,target=rom_download")
        assert actual is accepted
        predicate_cases.append({"id": value, "accepted": actual})
    report = {"status": "PASS_SOURCE_AND_SYNTHETIC_FOR_ONE_MANUAL_RECOVERY_ATTEMPT",
              "hardware_actions": [], "test_boundary": "Real main/Collector/identity with synthetic port, inventory and monotonic time",
              "source": {"path": str(SOURCE.relative_to(ROOT)), "sha256": hashlib.sha256(source_bytes).hexdigest()},
              "source_still_same": SOURCE.read_bytes() == source_bytes,
              "cases": cases, "predicate_cases": predicate_cases,
              "previous_static_finding": "Root fixed already-read RACK attribution before this execution: parsed lines are cleared without raw/input flush and the exact accepted/rom_download/nonzero-ASCII-u32 predicate rejects unrelated ACK. Original pre-fix main was inspected, not executed by this reviewer.",
              "remaining_ACK_limit": "RACK lacks session/boot/command nonce, so even matching fresh observed ACK is not authenticated causation. New USB epoch remains ROM_UNVERIFIED and downstream MAC/ROM synchronization is required.",
              "not_proven": ["physical ROM execution", "hardware reset timing", "SDK cancellation", "unobserved reboot race"]}
    destination = STAGE / "request-usj-recovery-rom-independent-review-01.json"
    with destination.open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"status": report["status"], "scenarios": len(cases), "source_still_same": report["source_still_same"]}))


if __name__ == "__main__":
    main()
