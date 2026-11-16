#!/usr/bin/env python3
"""Bounded app-only USJ flash followed by one official ESP32-S3 watchdog exit.

An exit code of zero means flash hash verification and a returned watchdog
call, not verified application boot. Independently observe enumeration/BUILD.
Start with the already identified USJ download interface. No automatic retry,
reflash, bootloader, partition-table, NVS, calibration or eFuse write is used.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parents[1]
EXPECTED_MAC = "80:b5:4e:cd:d9:ac"
APP_OFFSET = 0x10000
FLASH_BYTES = 16 * 1024 * 1024
SECTOR_BYTES = 4096


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def number(text):
    text = str(text).strip().lower()
    multiplier = 1
    if text.endswith(("k", "m")):
        multiplier = 1024 if text[-1] == "k" else 1024 * 1024
        text = text[:-1]
    return int(text, 16 if text.startswith("0x") else 10) * multiplier


def app_partition(path, app_bytes):
    partitions = []
    with path.open(newline="") as stream:
        for row in csv.reader(line for line in stream if line.strip() and not line.lstrip().startswith("#")):
            require(len(row) >= 5, "partition row requires explicit name/type/subtype/offset/size")
            name, kind, subtype, offset, size = (item.strip() for item in row[:5])
            require(bool(offset), "implicit partition offsets are not accepted by this app-only gate")
            start, length = number(offset), number(size)
            require(start >= 0 and length > 0 and start + length <= FLASH_BYTES, "partition outside 16 MB flash")
            require(start % SECTOR_BYTES == 0 and length % SECTOR_BYTES == 0, "partition must be sector aligned")
            partitions.append({"name": name, "type": kind, "subtype": subtype,
                               "offset": start, "size": length})
    ordered = sorted(partitions, key=lambda item: item["offset"])
    for left, right in zip(ordered, ordered[1:]):
        require(left["offset"] + left["size"] <= right["offset"], "overlapping partitions")
    applications = [item for item in partitions if item["type"] in {"app", "0", "0x00", "0x0"}]
    require(len(applications) == 1, "exactly one app partition is required")
    app = applications[0]
    require(app["offset"] == APP_OFFSET and app["subtype"] in {"factory", "0", "0x00", "0x0"},
            "factory app must start at 0x10000")
    erase_bytes = (app_bytes + SECTOR_BYTES - 1) // SECTOR_BYTES * SECTOR_BYTES
    require(0 < app_bytes <= app["size"] and erase_bytes <= app["size"], "app/erase range exceeds app partition capacity")
    return {**app, "erase_start": APP_OFFSET, "erase_end_exclusive": APP_OFFSET + erase_bytes}


def validate_inputs(root, out, label):
    manifest_path = out / (label + "-build-manifest.json")
    manifest = json.loads(manifest_path.read_text())
    results = manifest.get("results")
    require(isinstance(results, list) and bool(results) and
            all(type(item.get("exit_code")) is int and item["exit_code"] == 0 for item in results),
            "manifest has missing or failed build commands")
    require(manifest.get("source_changed_during_run") is False, "build inputs changed or unknown; rebuild first")
    inputs = manifest.get("inputs_after")
    require(isinstance(inputs, dict) and {"main/main.c", "sdkconfig", "partitions.csv"} <= inputs.keys(),
            "manifest lacks required source/config/partition hashes")
    for name, expected_hash in inputs.items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts, "manifest path must stay inside repository")
        path = (root / relative).resolve(strict=True)
        require(path.is_relative_to(root.resolve()), "manifest source symlink escapes repository")
        require(isinstance(expected_hash, str) and re.fullmatch(r"[0-9a-f]{64}", expected_hash) is not None,
                "invalid source hash: " + name)
        require(sha256(path) == expected_hash, "source hash mismatch: " + name)
    app = root / "build/SensorArray.bin"
    require(type(manifest.get("app_bytes")) is int and manifest["app_bytes"] > 0, "invalid manifest app_bytes")
    require(app.stat().st_size == manifest["app_bytes"], "app size differs from build manifest")
    require(sha256(app) == manifest.get("app_sha256"), "app hash mismatch")
    flasher_path = root / "build/flasher_args.json"
    flasher = json.loads(flasher_path.read_text())
    require(flasher.get("app", {}).get("file") == app.name, "unexpected app image path")
    require(number(flasher["app"]["offset"]) == APP_OFFSET, "app offset must be exactly 0x10000")
    require(flasher["app"].get("encrypted", "false") == "false", "encrypted flashing is outside this script's scope")
    require(flasher.get("flash_settings") == {"flash_mode": "dio", "flash_size": "16MB", "flash_freq": "80m"},
            "unexpected flash settings")
    if "extra_esptool_args" in flasher:
        require(flasher["extra_esptool_args"].get("chip") == "esp32s3", "flasher chip is not ESP32-S3")
    partition = app_partition(root / "partitions.csv", manifest["app_bytes"])
    return manifest, app, {"build_manifest_sha256": sha256(manifest_path),
                           "flasher_args_sha256": sha256(flasher_path),
                           "partitions_sha256": sha256(root / "partitions.csv"), "app_partition": partition}


def identity_matches(text):
    macs = re.findall(r"^MAC:\s*([0-9a-f:]{17})\s*$", text.lower(), flags=re.MULTILINE | re.IGNORECASE)
    return bool(macs) and set(macs) == {EXPECTED_MAC} and re.search(r"^Chip is ESP32-S3(?:[ (]|$)", text, re.MULTILINE) is not None


def helper_observation(text):
    requested, returned = None, False
    for line in text.splitlines():
        try:
            event = json.loads(line)
        except (ValueError, TypeError):
            continue
        if isinstance(event, dict):
            if event.get("watchdog_requested") is True:
                requested = True
            elif event.get("watchdog_requested") is False and requested is not True:
                requested = False
            returned = returned or event.get("watchdog_call_returned") is True
    return requested, returned


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("label")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--before", choices=("default_reset", "usb_reset", "no_reset"), default="no_reset",
                        help="Identity connection only; app write always uses no_reset before and after")
    args = parser.parse_args(argv)
    require(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.label) is not None, "invalid evidence label")
    require(args.port.startswith("/dev/") and not args.port.startswith("/dev/../"), "port must be an explicit /dev path")
    paths = {name: OUT / (args.label + "-flash-usj-" + name + ".txt") for name in ("identity", "app", "watchdog")}
    result_path = OUT / (args.label + "-flash-usj-result.json")
    # Reserve every output before any device operation; do not fail late after
    # writing because an old helper/result evidence path already exists.
    for path in [*paths.values(), result_path]:
        require(not path.exists(), "evidence already exists: " + str(path))
    result = {"schema": "app_only_usj_flash_v1", "status": "FAILED", "commands": [],
              "expected_mac": EXPECTED_MAC, "app_offset": APP_OFFSET,
              "flash_verified": False, "watchdog_helper_started": False,
              "watchdog_requested": False, "watchdog_call_returned": False,
              "app_boot_verified": False, "automatic_retries": 0,
              "nvs_or_calibration_written": False, "partition_written": False,
              "bootloader_written": False, "efuse_written": False,
              "script_sha256": sha256(Path(__file__)),
              "limitations": ["Application enumeration and BUILD identity require independent observation.",
                              "Official ROM connection may disable watchdog registers before ROM MAC readback; no flash/eFuse write is requested then."]}
    with ExitStack() as stack:
        streams = {name: stack.enter_context(path.open("x")) for name, path in paths.items()}
        report = stack.enter_context(result_path.open("x"))

        def execute(name, command, timeout):
            record = {"stage": name, "command": command, "timeout_seconds": timeout,
                      "log": str(paths[name].relative_to(ROOT))}
            result["commands"].append(record)
            try:
                run = subprocess.run(command, cwd=ROOT, stdout=streams[name], stderr=subprocess.STDOUT, timeout=timeout)
                record["exit_code"] = run.returncode
            except subprocess.TimeoutExpired:
                record.update(exit_code=124, timeout=True)
                raise RuntimeError(name + " timed out; no retry permitted")
            finally:
                streams[name].flush()
                record["log_sha256"] = sha256(paths[name])
            require(run.returncode == 0, name + " failed with exit " + str(run.returncode))
            return paths[name].read_text(errors="replace")

        try:
            manifest, app, validation = validate_inputs(ROOT, OUT, args.label)
            result.update(validation, head=manifest["head"], build_version=manifest["build_version"],
                          app_sha256=manifest["app_sha256"], app_bytes=manifest["app_bytes"])
            helper = Path(__file__).with_name("exit_usj_download_watchdog.py")
            result["watchdog_helper_sha256"] = sha256(helper)
            base = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", args.port,
                    "--baud", "460800", "--connect-attempts", "2"]
            identity = execute("identity", base + ["--before", args.before, "--after", "no_reset", "flash_id"], 180)
            require(identity_matches(identity), "identity MAC/chip mismatch; app not written")
            # Recheck the validated build after the bounded identity subprocess.
            _, _, repeated = validate_inputs(ROOT, OUT, args.label)
            require(repeated == validation, "validated input metadata changed before app write")
            written = execute("app", base + ["--before", "no_reset", "--after", "no_reset", "write_flash",
                                             "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "16MB",
                                             "0x10000", str(app)], 180)
            require(sum(line.strip() == "Hash of data verified." for line in written.splitlines()) == 1,
                    "write returned without the single real esptool hash verification marker")
            result["flash_verified"] = True
            require(sha256(helper) == result["watchdog_helper_sha256"], "watchdog helper changed after validation")
            result.update(watchdog_helper_started=True, watchdog_requested=None)
            try:
                execute("watchdog", [sys.executable, str(helper), "--port", args.port, "--mac", EXPECTED_MAC], 30)
            finally:
                result["watchdog_requested"], result["watchdog_call_returned"] = helper_observation(paths["watchdog"].read_text(errors="replace"))
            require(result["watchdog_requested"] is True and result["watchdog_call_returned"] is True,
                    "helper did not confirm one returned watchdog call; inspect separate boot evidence")
            result["status"] = "FLASH_VERIFIED_WATCHDOG_REQUESTED_BOOT_UNVERIFIED"
        except Exception as error:
            result["error"] = {"type": type(error).__name__, "message": str(error)}
        finally:
            json.dump(result, report, indent=2)
            report.write("\n")
            report.flush()
    print(json.dumps({"result": str(result_path), "status": result["status"],
                      "flash_verified": result["flash_verified"], "watchdog_requested": result["watchdog_requested"],
                      "app_boot_verified": False}), flush=True)
    return 0 if "error" not in result else 1


if __name__ == "__main__":
    raise SystemExit(main())
