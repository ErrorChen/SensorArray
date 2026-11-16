"""Authorized app-only flash; retain NVS, calibration, bootloader and partitions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
os.chdir(root)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("label")
parser.add_argument("--port", default="/dev/ttyACM0", help="Previously identified board download tty; MAC is verified before writing")
parser.add_argument("--before", choices=("default_reset", "usb_reset", "no_reset"), default="default_reset")
options = parser.parse_args()
label = options.label
assert all(c.isalnum() or c in "-_" for c in label)
manifest = json.loads((out / (label + "-build-manifest.json")).read_text())
assert all(r["exit_code"] == 0 for r in manifest["results"])
assert not manifest["source_changed_during_run"], "build inputs changed; rebuild first"
for name, sha in manifest["inputs_after"].items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == sha, name
app = root / "build/SensorArray.bin"
assert hashlib.sha256(app.read_bytes()).hexdigest() == manifest["app_sha256"]
flasher = json.loads((root / "build/flasher_args.json").read_text())
assert flasher["app"]["file"] == app.name
assert flasher["flash_settings"] == {"flash_mode": "dio", "flash_size": "16MB", "flash_freq": "80m"}
app_offset = flasher["app"]["offset"]
base = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", options.port,
        "--baud", "460800", "--connect-attempts", "2"]
records = []
for name, args in [("identity", ["--before", options.before, "--after", "no_reset", "flash_id"]),
                   ("app", ["--before", "no_reset", "--after", "hard_reset",
                            "write_flash", "--flash_mode", "dio", "--flash_freq", "80m",
                            "--flash_size", "16MB", app_offset, str(app)])]:
    # Keep the ROM/flasher stub after identification. Booting the old OTG app
    # here would replace the programming interface before the write begins.
    log = out / (label + "-flash-" + name + ".txt")
    command = base + args
    with log.open("x") as stream:
        run = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=180)
    records.append({"command": command, "exit_code": run.returncode,
                    "log": str(log.relative_to(root))})
    print(log.read_text()[-1600:], flush=True)
    if run.returncode:
        raise RuntimeError(name + " failed")
    if name == "identity":
        assert "80:b5:4e:cd:d9:ac" in log.read_text().lower(), "device mismatch"
with (out / (label + "-flash-result.json")).open("x") as stream:
    json.dump({"head": manifest["head"], "build_version": manifest["build_version"],
               "app_sha256": manifest["app_sha256"], "app_bytes": manifest["app_bytes"],
               "commands": records, "nvs_or_calibration_written": False,
               "partition_written": False, "bootloader_written": False}, stream, indent=2)
