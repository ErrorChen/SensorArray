#!/usr/bin/env python3
"""Run independent BLE fragment schedules using exact production functions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))
from test_main_config_commit import function


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", help="Read immutable source from this commit")
    args = parser.parse_args()
    source_file = ROOT / "components/sensorarrayBle/sensorarrayBle.c"
    source = (subprocess.check_output(["git", "show",
        f"{args.baseline}:components/sensorarrayBle/sensorarrayBle.c"], cwd=ROOT).decode()
        if args.baseline else source_file.read_text())
    harness_file = Path(__file__).with_suffix(".c")
    harness = harness_file.read_text()
    for signature in (
        "static uint32_t sensorarrayBleCrc32(",
        "static size_t sensorarrayBleMaxPayloadSize(",
        "static bool sensorarrayBlePlanFragments(",
        "static esp_err_t sensorarrayBleSendFragment(",
        "static esp_err_t sensorarrayBleSendPayload(",
    ):
        name = signature.split("(")[0].split()[-1]
        assert harness.count(f"/* {name} */") == 1
        harness = harness.replace(f"/* {name} */", function(source, signature))
    # New constants, if kept outside the extracted function, remain production
    # definitions. Do not substitute a test value for the four-fragment limit.
    batch_constants = [line for line in source.splitlines()
        if re.match(r"#define\s+SENSORARRAY_BLE_\w*(?:BATCH|BUDGET)\w*\s", line)]
    harness = harness.replace("/* extracted batch configuration */", "\n".join(batch_constants))
    sha = lambda b: hashlib.sha256(b).hexdigest()
    print(json.dumps({"source": str(source_file), "baseline": args.baseline,
        "source_sha256": sha(source.encode()),
        "harness_sha256": sha(harness_file.read_bytes()),
        "budget_sha256": sha((ROOT / "components/sensorarrayWorkBudget/sensorarrayWorkBudget.c").read_bytes()),
        "hardware_access": False, "model": "synthetic scheduler and GATT boundary"}), flush=True)
    with tempfile.TemporaryDirectory(prefix="sa-ble-batch-independent-") as temp:
        temp = Path(temp)
        generated = temp / "review.c"
        generated.write_text(harness)
        command = [os.environ.get("CC", "/usr/bin/gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror"]
        for inc in ["tests/host/work_budget_stubs", "tests/host/include",
                    "components/sensorarrayWorkBudget/include", "components/sensorarrayWorkBudget"]:
            command += ["-I", str(ROOT / inc)]
        if args.baseline:
            command += ["-DAUDIT_BASELINE"]
        command += [str(generated), "-o", str(temp / "review")]
        subprocess.run(command, check=True)
        subprocess.run([str(temp / "review")], check=True)
    if not args.baseline:
        assert source_file.read_text() == source, "Production BLE source changed during review"
        print("BLE_BATCH_SOURCE_UNCHANGED=1", flush=True)


if __name__ == "__main__":
    main()
