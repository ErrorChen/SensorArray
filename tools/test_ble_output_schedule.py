#!/usr/bin/env python3
"""Execute the real standalone BLE output policy with deterministic evidence."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components/sensorarrayBle"


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="sensorarray-ble-output-") as temporary:
        for label, flags in (
            ("optimized", ["-O2"]),
            ("asan_ubsan", ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]),
        ):
            executable = Path(temporary) / label
            subprocess.run([
                os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                *flags, "-I", str(COMPONENT), str(COMPONENT / "sensorarrayBleOutputSchedule.c"),
                str(ROOT / "tests/host/test_ble_output_schedule.c"), "-o", str(executable),
            ], check=True)
            # The sandbox uses ptrace, which LeakSanitizer cannot inspect.
            # This module allocates nothing; keep address/UB checks enabled.
            environment = os.environ.copy()
            environment["ASAN_OPTIONS"] = environment.get("ASAN_OPTIONS", "") + ":detect_leaks=0"
            subprocess.run([str(executable)], check=True, env=environment)
            print(f"BLE_OUTPUT_SCHEDULE_VARIANT,{label}=PASS", flush=True)


if __name__ == "__main__":
    main()
