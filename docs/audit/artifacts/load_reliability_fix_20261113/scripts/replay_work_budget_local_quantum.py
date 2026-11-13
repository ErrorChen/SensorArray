#!/usr/bin/env python3
"""Offline old-source / candidate comparison; never modifies production or HIL."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-prefix", type=Path)
    parser.add_argument("--harness", default="work-budget-local-quantum-model-01.c")
    args = parser.parse_args()
    root = next(p for p in Path(__file__).resolve().parents if (p / "components/sensorarrayWorkBudget").is_dir())
    evidence = Path(__file__).resolve().parents[1]
    harness = evidence / args.harness
    source_names = {"before": "work-budget-before-local-quantum-01.c.txt",
                    "candidate": "work-budget-local-quantum-candidate-01.c.txt"}
    result = {"evidence_type": "SYNTHETIC_PRODUCTION_FUNCTION_COUNTEREXAMPLE_AND_CANDIDATE",
              "hardware": "NOT_RUN", "production_modified_by_replay": False,
              "head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
              "note": "Candidate is an independent saved source proposal; no claim of measured throughput or CPU utilization.",
              "runs": {}}
    result["harness"] = {"path": str(harness.relative_to(root)),
                         "sha256": hashlib.sha256(harness.read_bytes()).hexdigest()}
    logs = []
    compiler = shutil.which("clang-18") or shutil.which("cc")
    assert compiler
    with tempfile.TemporaryDirectory(prefix="sensorarray-local-quantum-model-") as directory:
        temp = Path(directory)
        (temp / "sensorarrayWorkBudget.h").write_bytes((evidence / "work-budget-before-local-quantum-01.h.txt").read_bytes())
        for variant, name in source_names.items():
            source = evidence / name
            (temp / "sensorarrayWorkBudget.c").write_bytes(source.read_bytes())
            executable = temp / variant
            command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(temp),
                       "-I", str(root / "tests/host/work_budget_stubs"),
                       "-I", str(root / "tests/host/include"),
                       str(harness), "-o", str(executable)]
            compiled = subprocess.run(command, capture_output=True, text=True)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10) if compiled.returncode == 0 else None
            records = [json.loads(line) for line in run.stdout.splitlines()] if run and run.returncode == 0 else []
            result["runs"][variant] = {"source": str(source.relative_to(root)),
                                        "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                                        "compile_command": command, "compile_exit_code": compiled.returncode,
                                        "run_exit_code": run.returncode if run else None, "records": records}
            logs.append(f"{variant}\n{compiled.stdout}{compiled.stderr}{run.stdout if run else ''}{run.stderr if run else ''}")
    before, candidate = (result["runs"][key]["records"] for key in ("before", "candidate"))
    checks = {"both_compiled_and_ran": all(r["compile_exit_code"] == 0 and r["run_exit_code"] == 0 for r in result["runs"].values()),
              "old_four_unit_burst_delays_three_times": bool(before) and before[0]["yields"] == 300,
              "candidate_four_unit_burst_no_artificial_delay": bool(candidate) and candidate[0]["yields"] == 0,
              "same_real_idle_callbacks": bool(before) and bool(candidate) and before[0]["idleCallbacks"] == candidate[0]["idleCallbacks"] == 50000}
    result["checks"] = checks
    result["status"] = "PASS" if all(checks.values()) else "FAIL"
    if args.output_prefix:
        with args.output_prefix.with_suffix(".json").open("x") as f: json.dump(result, f, indent=2); f.write("\n")
        with args.output_prefix.with_suffix(".txt").open("x") as f: f.write("\n".join(logs))
    print(json.dumps({"status": result["status"], "checks": checks,
                      "models": {key: r["records"] for key, r in result["runs"].items()}}, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
