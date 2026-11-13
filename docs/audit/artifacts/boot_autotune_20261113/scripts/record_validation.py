"""Task evidence recorder; executes the real project build or all host tests."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root = Path(__file__).resolve().parents[5]
out = Path(__file__).resolve().parents[1]
kind, label = sys.argv[1:3]
os.chdir(root)


def inputs():
    paths = {p for top in ("main", "core", "components") for p in Path(top).rglob("*")
             if p.is_file() and "__pycache__" not in p.parts}
    paths.update(Path(p) for p in ("CMakeLists.txt", "sdkconfig", "sdkconfig.defaults",
                                  "dependencies.lock", "partitions.csv"))
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths) if p.is_file()}


result = {
    "head": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
    "dirty_diff_sha256": hashlib.sha256(subprocess.check_output(["git", "diff", "--binary"])).hexdigest(),
    "test_inputs": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(Path("tools").glob("test_*.py"))},
    "status": subprocess.check_output(["git", "status", "--short"], text=True),
    "inputs_before": inputs(), "results": [],
}
manifest = out / (label + "-" + kind + "-manifest.json")
assert not manifest.exists()
if kind == "build":
    commands = [("idf", ["bash", "-lc", "source /home/csc/esp/esp-idf-v5.5.5/export.sh >/dev/null && idf.py reconfigure build"])]
elif kind == "tests":
    commands = [(p.stem, ["python3", str(p)]) for p in sorted(Path("tools").glob("test_*.py"))]
else:
    raise ValueError(kind)
for name, command in commands:
    log = out / (label + "-" + name + ".txt")
    started = time.monotonic()
    with log.open("x") as stream:
        try:
            run = subprocess.run(command, env={**os.environ, "CC": "/usr/bin/cc"},
                                 stdout=stream, stderr=subprocess.STDOUT,
                                 timeout=600 if kind == "build" else 180)
            code = run.returncode
        except subprocess.TimeoutExpired:
            code = 124
            stream.write("\nEVIDENCE_RUNNER_DEADLINE\n")
    result["results"].append({"command": command, "exit_code": code,
                              "duration_seconds": time.monotonic() - started,
                              "log": str(log.relative_to(root))})
    print(name, code, flush=True)
result["inputs_after"] = inputs()
result["source_changed_during_run"] = result["inputs_before"] != result["inputs_after"]
if kind == "build" and result["results"][0]["exit_code"] == 0:
    app = Path("build/SensorArray.bin")
    result.update(app_sha256=hashlib.sha256(app.read_bytes()).hexdigest(),
                  app_bytes=app.stat().st_size,
                  build_version=json.loads(Path("build/project_description.json").read_text())["project_version"])
manifest.write_text(json.dumps(result, indent=2) + "\n")
raise SystemExit(int(any(r["exit_code"] for r in result["results"])))
