"""Execute one bounded evidence command, keeping its exact argv, exit and log."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time

root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--label", required=True)
parser.add_argument("--timeout", type=float, required=True)
parser.add_argument("command", nargs=argparse.REMAINDER)
args = parser.parse_args()
command = args.command[1:] if args.command[:1] == ["--"] else args.command
assert command and 0 < args.timeout <= 9000
assert all(c.isalnum() or c in "-_" for c in args.label)
report = out / (args.label + "-command.json")
log = out / (args.label + "-command.txt")
assert not report.exists() and not log.exists()
record = {"command": command, "cwd": str(root), "timeout_seconds": args.timeout,
          "head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
          "dirty_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "diff", "--binary"], cwd=root)).hexdigest(),
          "started_utc": datetime.now(timezone.utc).isoformat(),
          "started_monotonic": time.monotonic(), "log": str(log.relative_to(root))}
with log.open("x") as stream:
    try:
        result = subprocess.run(command, cwd=root, stdout=stream, stderr=subprocess.STDOUT,
                                timeout=args.timeout)
        record["exit_code"] = result.returncode
    except subprocess.TimeoutExpired:
        record.update(exit_code=124, runner_timeout=True)
        stream.write("\nEVIDENCE_COMMAND_DEADLINE\n")
record.update(finished_utc=datetime.now(timezone.utc).isoformat(),
              finished_monotonic=time.monotonic(),
              log_sha256=hashlib.sha256(log.read_bytes()).hexdigest())
with report.open("x") as stream:
    json.dump(record, stream, indent=2)
print(json.dumps({"command_record": str(report.relative_to(root)), "exit_code": record["exit_code"]}), flush=True)
raise SystemExit(record["exit_code"])
