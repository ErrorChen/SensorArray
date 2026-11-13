"""Read-only flash backup before changing the USB interface; no NVS read/write."""
import hashlib, json, subprocess, sys, time
from pathlib import Path
root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
previous = json.loads((root / "docs/audit/artifacts/load_reliability_fix_20261113/repair-11-build-manifest.json").read_text())
path = out / "rollback-be09762-app.bin"
if path.exists(): raise RuntimeError("refuse to overwrite backup")
command = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", "/dev/ttyACM0",
           "--before", "default_reset", "--after", "hard_reset", "read_flash", "0x10000",
           str(previous["app_bytes"]), str(path)]
started = time.monotonic()
with (out / "rollback-be09762-read.txt").open("x") as log:
    result = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, timeout=180)
record = {"command": command, "exit_code": result.returncode, "elapsed_seconds": time.monotonic()-started,
          "flash_written": False, "nvs_accessed": False, "expected_app_sha256": previous["app_sha256"]}
if path.exists():
    record.update(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
record["matches_previous_flashed_app"] = record.get("sha256") == previous["app_sha256"]
with (out / "rollback-be09762-read.json").open("x") as f: json.dump(record, f, indent=2)
print(json.dumps(record, indent=2))
if result.returncode or not record["matches_previous_flashed_app"]: raise SystemExit(1)
