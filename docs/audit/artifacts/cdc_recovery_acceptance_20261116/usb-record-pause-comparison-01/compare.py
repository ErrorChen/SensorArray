#!/usr/bin/env python3
"""Compare two CLOSED 650-ms formal-reader pause captures, without device I/O.

Both final command manifests and observer closures are checked before any raw
recording is opened. --write writes only this evidence bundle's own outputs.
"""
from __future__ import annotations

import argparse
import bisect
import gzip
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
STAGE = HERE.parent
sys.path.insert(0, str(HERE / "sources"))
import analyze_firmware_benchmark as analyzer
from text_protocol import parse_fields

CASES = ("bss-pause-volt2-01", "record-pause-volt2-01")


def require(ok, message):
    if not ok:
        raise ValueError(message)


def read_bytes(path):
    return path.read_bytes() if path.exists() else gzip.decompress(Path(str(path) + ".gz").read_bytes())


def load(path):
    return json.loads(read_bytes(path))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def json_value(value):
    # Match JSON's string object keys (e.g. invalid_points_per_frame bins).
    return json.loads(json.dumps(value))


def resolve(name):
    path = Path(name)
    return path if path.is_absolute() else ROOT / path


def reference(path):
    data = read_bytes(path)
    archived = Path(str(path) + ".gz")
    storage = archived if archived.exists() else path
    stored = storage.read_bytes()
    if storage == archived:
        require(gzip.decompress(stored) == data, f"archive_roundtrip_mismatch:{path}")
    return {"path": str(storage.relative_to(ROOT)), "uncompressed_bytes": len(data),
            "uncompressed_sha256": sha(data), "stored_bytes": len(stored),
            "stored_sha256": sha(stored)}


def closure(label):
    directory = STAGE / label / (label + "-volt2")
    prefix = directory / directory.name
    needed = [Path(str(prefix) + suffix) for suffix in
              (".command.json", ".gate.json", ".summary.json", ".ble.summary.json", ".debug.summary.json")]
    require(all(path.exists() for path in needed), f"NOT_CLOSED:{label}:final_files_missing")
    command = load(needed[0])
    require(command.get("child_termination_known") is True and command.get("exit_code") == 0,
            f"NOT_CLOSED:{label}:formal_process")
    observers = command.get("observer_closure", [])
    require({item.get("kind") for item in observers} == {"ble", "debug"}, f"NOT_CLOSED:{label}:observers_missing")
    require(all(item.get("termination_known") is True and item.get("exit_code") == 0
                and item.get("cleanup_errors") == [] for item in observers),
            f"NOT_CLOSED:{label}:observer_process")
    return directory, command


def check_source_hashes(gate):
    sources = [gate["analysis"]["tool_source"], *gate["analysis"]["parser_sources"]]
    out = []
    for item in sources:
        archived = HERE / "sources" / Path(item["path"]).name
        require(sha(archived.read_bytes()) == item["sha256"], "recorded_parser_or_analyzer_source_changed")
        out.append(reference(archived))
    return out


def diagnostics(summary, query, counter_fields):
    endpoints = {}
    for window in summary["diagnostic_windows"]:
        if window["name"] in ("before", "after") and query in window.get("queries", {}):
            item = window["queries"][query]
            endpoints[window["name"]] = {"receipt_monotonic": item["monotonic"],
                                         "fields": item["fields"]}
    if set(endpoints) != {"before", "after"}:
        return {"status": "UNKNOWN", "endpoints": endpoints}
    before, after = endpoints["before"], endpoints["after"]
    delta = {}
    for field in counter_fields:
        try:
            a, b = int(before["fields"][field]), int(after["fields"][field])
            delta[field] = b - a if b >= a else None
        except (KeyError, ValueError):
            delta[field] = None
    return {"status": "OBSERVED", **endpoints, "delta": delta,
            "span_seconds": after["receipt_monotonic"] - before["receipt_monotonic"],
            "scope": "query_receipts_bracketing_capture_not_exact_capture_endpoints"}


def adc_health(directory, label, start, end):
    raw = read_bytes(directory / (label + ".debug.rx.bin"))
    events = [json.loads(line) for line in read_bytes(directory / (label + ".debug.events.jsonl")).splitlines()]
    chunks = [event for event in events if event["type"] == "rx"]
    offsets = [event["offset"] for event in chunks]
    selected, offset = [], 0
    for wire in raw.splitlines(keepends=True):
        stop = offset + len(wire) - 1
        if wire.startswith(b"ADSHEALTH,") and wire.endswith(b"\n"):
            chunk = chunks[bisect.bisect_right(offsets, stop) - 1]
            require(stop < chunk["offset"] + chunk["length"], "debug_offset_not_covered")
            if start <= chunk["monotonic"] <= end:
                fields = parse_fields(wire.decode().rstrip("\r\n"))
                if all(key in fields for key in ("crcValidEven", "crcValidOdd", "crcErrors", "crcRetries")):
                    selected.append({"monotonic": chunk["monotonic"], "raw_offset": offset, "fields": fields})
        offset += len(wire)
    if len(selected) < 2:
        return {"status": "UNKNOWN", "records": len(selected)}
    first, last = selected[0], selected[-1]
    delta = {key: int(last["fields"][key]) - int(first["fields"][key])
             for key in ("crcValidEven", "crcValidOdd", "crcErrors")}
    require(all(value >= 0 for value in delta.values()), "ADC_counter_decrease")
    return {"status": "OBSERVED", "records": len(selected), "first": first, "last": last,
            "delta": delta, "checked_reads": sum(delta.values()),
            "span_seconds": last["monotonic"] - first["monotonic"],
            "scope": "first_last_complete_debug_ADSHEALTH_receipts_inside_capture; checked_reads_include_retries_not_frames",
            "crcRetries": "UNKNOWN_terminal_token_not_used"}


def compare_case(label, directory, command):
    name = directory.name
    gate_path = directory / (name + ".gate.json")
    gate, summary = load(gate_path), load(directory / (name + ".summary.json"))
    require(command["summary_sha256"] == sha(read_bytes(directory / (name + ".summary.json"))), "command_summary_hash_changed")
    sources = check_source_hashes(gate)
    pause_source = HERE / "sources/capture_cdc_short_pause.py"
    require(sha(pause_source.read_bytes()) == summary["source_sha256"], "pause_collector_source_changed")
    sources.append(reference(pause_source))
    sources.append(reference(HERE / "sources/sensorarray_bulk_diagnostics.py"))
    files = {}
    for group_name in ("formal_authority", "debug"):
        for kind in ("summary", "raw", "events"):
            item = gate[group_name][kind]
            path = resolve(item["path"])
            require(sha(read_bytes(path)) == item["sha256"], f"changed_closed_input:{path}")
            files[group_name + "_" + kind] = reference(path)
    journal_source = gate["analysis"]["ble_journal"]["source"]
    journal = resolve(journal_source["path"])
    journal_bytes = read_bytes(journal)
    require(len(journal_bytes) == journal_source["uncompressed_prefix_bytes"] and
            sha(journal_bytes) == journal_source["uncompressed_prefix_sha256"] and
            journal_source["file_grew_during_analysis"] is False, "changed_BLE_journal")
    files["BLE_journal"] = reference(journal)
    files["gate"] = reference(gate_path)
    files["command"] = reference(directory / (name + ".command.json"))
    replay = analyzer.analyse_directory(directory,
        ble_journal=journal if journal.exists() else Path(str(journal) + ".gz"),
        debug_summaries=[directory / (name + ".debug.summary.json")])
    require(len(replay["captures"]) == 1, "one_capture_required")
    capture = replay["captures"][0]
    recorded = gate["analysis"]["captures"][0]
    for key in ("capture_counts", "configuration_groups", "whole_raw_issues"):
        require(json_value(capture[key]) == recorded[key], f"replay_differs_from_original_gate:{label}:{key}")
    for key in ("capture_frame_counts", "configuration_groups", "USB_comparison"):
        require(json_value(capture["ble"][key]) == recorded["ble"][key], f"BLE_replay_differs_from_original_gate:{label}:{key}")
    require(len(capture["configuration_groups"]) == len(capture["ble"]["configuration_groups"]) == 1,
            "single_identity_required")
    usb, ble = capture["configuration_groups"][0], capture["ble"]["configuration_groups"][0]
    require(usb["rows"] == ble["rows"] == 2 and usb["wire_tag"] == ble["wire_tag"] == "V", "VOLT2_required")
    require(usb["identity"] == ble["identity"], "USB_BLE_identity_mismatch")
    events = [json.loads(line) for line in read_bytes(directory / (name + ".events.jsonl")).splitlines()]
    pauses = {kind: [event for event in events if event["type"] == kind]
              for kind in ("user_space_read_pause_start", "user_space_read_pause_end")}
    require(all(len(items) == 1 for items in pauses.values()), "exactly_one_pause_required")
    bracket = summary["formal_reader_pause_monotonic"]
    duration = bracket[1] - bracket[0]
    require(summary["formal_reader_pause_verified"] is True and duration >= .650 and
            abs(duration - pauses["user_space_read_pause_end"][0]["duration_seconds"]) < 1e-9,
            "pause_not_verified")
    require(summary["capture_monotonic_start"] <= bracket[0] < bracket[1] <= summary["capture_monotonic_end"],
            "pause_outside_capture")
    require(pauses["user_space_read_pause_start"][0]["requested_seconds"] == .650, "pause_request_changed")
    cleanup = [event for event in events if event["type"] == "reply" and event.get("phase") == "cleanup"
               and event.get("command") == "CONFIG?"]
    require(cleanup and summary["safe_cap_recovery"] == "CAP8_QUERY_VERIFIED", "CAP8_unverified")
    last = cleanup[-1]
    fields = parse_fields(last.get("line", ""))
    raw = read_bytes(directory / (name + ".rx.bin"))
    require((last.get("line", "") + "\n").encode() in raw, "CAP8_reply_not_in_original_raw")
    require(fields.get("mode") == "CAP" and fields.get("rows") == "8" and fields.get("err") == "0x0"
            and fields.get("session") == gate["expected"]["session"]
            and fields.get("bootId") == str(gate["expected"]["bootId"]),
            "CAP8_cleanup_reply_mismatch")
    counter_specs = {
        "USBQ?": ("dDrop", "cDrop", "dSubmit", "cSubmit", "expired", "cExpired", "partial", "dBytes", "cBytes"),
        "USB?": ("resets", "mounts", "unmounts", "rxDrop", "rxOver"),
        "BLETX?": ("dDequeued", "lDequeued", "cDequeued", "sdkCalls", "sdkAccepted", "sdkErrors"),
        "PERF?": ("pub", "fresh", "dropOut", "dropEvent", "usbSent", "usbDrop", "queueDrop"),
    }
    counters = {query: diagnostics(summary, query, fields) for query, fields in counter_specs.items()}
    queue = counters["USBQ?"]
    highwater = {side: int(queue[side]["fields"]["queueMaxUs"]) for side in ("before", "after")}
    missing = [seq for gap in usb["sequence"]["gap_examples"] for seq in range(gap["after"] + 1, gap["before"])]
    missing_raw = [{"seq": seq, "header_present": (b"V,seq=" + str(seq).encode() + b",") in raw,
                    "trailer_present": (b"K,seq=" + str(seq).encode() + b",") in raw} for seq in missing]
    return {"label": label, "expected_image": gate["expected"], "configuration_identity": usb["identity"],
        "gate_status": gate["status"], "gate_failures": gate["science"]["failures"],
        "overall_acceptance": gate["overall_acceptance"], "capture_seconds": summary["capture_seconds"],
        "pause": {"requested_seconds": .650, "actual_seconds": duration, "monotonic": bracket,
                  "verified_inside_capture": True, "events": pauses,
                  "scope": "formal_application_reader_only; Linux_USB_stack_may_continue_buffering"},
        "USB": {"counts": usb["counts"], "sequence": usb["sequence"], "rates": usb["rates"],
                "max_host_complete_interval_seconds": usb["host_complete_frame_intervals"]["max_seconds"],
                "raw_missing_sequence_observations": missing_raw},
        "BLE": {"counts": ble["counts"], "sequence": ble["sequence"], "rates": ble["rates"],
                "fragment_integrity": capture["ble"]["fragment_integrity"],
                "max_host_complete_interval_seconds": ble["host_complete_frame_intervals"]["max_seconds"]},
        "common_wire": capture["ble"]["USB_comparison"], "whole_raw_issues": capture["whole_raw_issues"],
        "counter_windows": counters, "queue_age": {"boot_highwater_us": highwater,
            "window_maximum_us": None, "semantics": "boot_highwater_not_reset_at_pause_or_capture; do_not_subtract_or_claim_window_max"},
        "ADC_CRC": adc_health(directory, name, summary["capture_monotonic_start"], summary["capture_monotonic_end"]),
        "CAP8_recovery": {"status": summary["safe_cap_recovery"], "actual_cleanup_reply": last,
                          "fields": fields, "reply_line_present_in_original_raw": True},
        "faults": summary["firmware_faults"], "resets": summary["reset_observations"],
        "observer_closure": [{key: item[key] for key in ("kind", "exit_code", "termination_known", "cleanup_errors")}
                             for item in command["observer_closure"]],
        "sources": sources, "files": files,
        "raw_replay_matches_original_gate": True}


def markdown(result):
    old, new = result["cases"]
    rows = []
    def row(label, a, b): rows.append(f"| {label} | {a} | {b} |")
    row("完整ELF / Full ELF", f'`{old["expected_image"]["elfSha256"]}`', f'`{new["expected_image"]["elfSha256"]}`')
    row("session / boot", f'{old["expected_image"]["session"]} / {old["expected_image"]["bootId"]}', f'{new["expected_image"]["session"]} / {new["expected_image"]["bootId"]}')
    row("Capture秒 / seconds", f'{old["capture_seconds"]:.9f}', f'{new["capture_seconds"]:.9f}')
    row("实际应用停读秒 / Actual application pause", f'{old["pause"]["actual_seconds"]:.9f}', f'{new["pause"]["actual_seconds"]:.9f}')
    for stream in ("USB", "BLE"):
        row(stream + " 完整可信帧 / trusted complete", old[stream]["counts"]["trusted_complete_frames"], new[stream]["counts"]["trusted_complete_frames"])
        row(stream + " 内部缺帧 / endpoint sequence missing", old[stream]["counts"].get("missing_frames", 0), new[stream]["counts"].get("missing_frames", 0))
        row(stream + " CRC / identity / wire errors", "/".join(str(old[stream]["counts"].get(k, 0)) for k in ("ascii_crc_errors", "identity_errors", "wire_contract_errors")), "/".join(str(new[stream]["counts"].get(k, 0)) for k in ("ascii_crc_errors", "identity_errors", "wire_contract_errors")))
        row(stream + " invalid点 / points", old[stream]["counts"]["invalid_points"], new[stream]["counts"]["invalid_points"])
    row("设备实际FPS / device sequence-timestamp FPS", f'{old["USB"]["rates"]["capture_fps"]:.9f}', f'{new["USB"]["rates"]["capture_fps"]:.9f}')
    for stream in ("USB", "BLE"):
        row(stream + " PC完整帧FPS / receipt FPS", f'{old[stream]["rates"]["host_complete_receive_fps"]:.9f}', f'{new[stream]["rates"]["host_complete_receive_fps"]:.9f}')
        row(stream + " 最大完整接收间隔秒 / maximum complete-receipt interval", f'{old[stream]["max_host_complete_interval_seconds"]:.9f}', f'{new[stream]["max_host_complete_interval_seconds"]:.9f}')
    for key in ("dDrop", "expired", "partial", "cDrop", "cExpired", "dSubmit", "cSubmit"):
        row("USBQ " + key + " Δ", old["counter_windows"]["USBQ?"]["delta"][key], new["counter_windows"]["USBQ?"]["delta"][key])
    row("queueMaxUs boot highwater 前→后", f'{old["queue_age"]["boot_highwater_us"]["before"]}→{old["queue_age"]["boot_highwater_us"]["after"]}', f'{new["queue_age"]["boot_highwater_us"]["before"]}→{new["queue_age"]["boot_highwater_us"]["after"]}')
    for key in ("dQueueCap", "dLifetimeUs"):
        row("USBQ " + key + " advertised", old["counter_windows"]["USBQ?"]["after"]["fields"].get(key, "NOT_REPORTED"), new["counter_windows"]["USBQ?"]["after"]["fields"].get(key, "NOT_REPORTED"))
    row("CAP8恢复 / recovery", old["CAP8_recovery"]["status"], new["CAP8_recovery"]["status"])
    row("原gate / original gate", old["gate_status"], new["gate_status"])
    text = "# 650 ms formal-reader pause comparison / 停读实录对比\n\n"
    text += "本比较在两次录制、BLE及debug observer全部关闭后离线重放原始数据；不改原gate、阈值或固件。 / Both recordings and observers were closed before raw replay. Original gates and thresholds remain unchanged.\n\n"
    text += "| 项目 / Measurement | 旧 / bss-pause-volt2-01 | 新 / record-pause-volt2-01 |\n| --- | --- | --- |\n" + "\n".join(rows) + "\n\n"
    text += "实际暂停只停止formal应用读取；Linux TTY/USB栈、控制器和设备仍可能缓冲，不能称为USB总线停机650 ms。设备FPS由同身份有序sequence增量/设备timestamp跨度算出；PC FPS由可信完整帧数与各自首末完成接收时间算出，均不是配置目标值。 / The injected pause affects application reads only. Device FPS and PC receipt FPS use distinct measured clocks and denominators.\n\n"
    text += "queueMaxUs是boot级最大已服务记录年龄，未在capture或pause开始清零，不能当本窗口最大值，也不能用前后差当年龄。USBQ计数来自包围capture的不同查询时刻；normalExpired包括DATA/LOG，不能把计数直接指派给某个sequence。 / Queue age is a boot highwater, not a pause-window maximum. Counter queries have separate boundaries and are not per-sequence traces.\n\n"
    text += "旧窗12个USB完整帧缺口与DATA拒绝+7、normal过期+5数值吻合；partial+1是部分发送的另一计数维度，不能再加成13。新窗未通过降低实测capture FPS消除缺口，但两个独立boot的FPS差不能单独归因于本修复。 / The old gap count matches seven DATA rejections plus five normal expiries; its one partial record is not an additional thirteenth loss. The new run's measured capture rate is higher, without establishing the cause of the rate difference.\n\n"
    for case in (old, new):
        label = case["label"]
        text += f'## {label}\n\n'
        text += f'USB sequence `{case["USB"]["sequence"]["first"]}..{case["USB"]["sequence"]["last"]}`；缺口 `{json.dumps(case["USB"]["sequence"]["gap_examples"])}`。BLE sequence `{case["BLE"]["sequence"]["first"]}..{case["BLE"]["sequence"]["last"]}`。共同原wire一致 **{case["common_wire"]["same_wire_bytes_sha256"]}**，差异 **{case["common_wire"]["different_wire_bytes_sha256"]}**；capture截断 **{case["whole_raw_issues"].get("truncated_frames_capture", 0)}**。完整帧保留invalid点；零完整帧CRC错误不证明没有截断。\n\n'
        adc = case["ADC_CRC"]
        if adc["status"] == "OBSERVED":
            text += f'独立debug ADSHEALTH窗：CRC拒绝 **{adc["delta"]["crcErrors"]}/{adc["checked_reads"]}** checked reads，首末receipt跨度 **{adc["span_seconds"]:.9f}秒**，含重试读数，非帧分母；crcRetries末端token不作推断。\n\n'
        text += f'FAULT/RESET观察：`{case["faults"]}` / `{case["resets"]}`；两observer正常关闭，CAP8实际cleanup回复保存在[comparison.json](comparison.json)。\n\n'
        text += "| 原始记录 / Original record | 未压缩字节 / Bytes | 未压缩SHA256 |\n| --- | ---: | --- |\n"
        for key in ("formal_authority_raw", "formal_authority_events", "debug_raw", "debug_events", "BLE_journal", "gate"):
            ref = case["files"][key]
            link = str(Path("../../../../..") / ref["path"])
            text += f'| [{key}]({link}) | {ref["uncompressed_bytes"]} | `{ref["uncompressed_sha256"]}` |\n'
        text += "\n"
    text += "## Reproduce / 重放\n\n```sh\npython3 /home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-record-pause-comparison-01/compare.py\n```\n\n"
    text += "脚本先验证双方闭合，再核对原始SHA并用固定版本actual parser/analyzer重放；stdout为JSON，`--write`只更新本证据目录的comparison.json和comparison.md。两次不同ELF/session的结果不能拼为同一boot；有限暂停筛查不建立所有stall情况下的交付保证、模拟精度或整体验收。 / These are separate image/session observations. They do not establish universal delivery guarantees, analog precision or overall acceptance.\n"
    return text


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--write", action="store_true")
    options = args.parse_args()
    try:
        closed = [(label, *closure(label)) for label in CASES]
        cases = [compare_case(*item) for item in closed]
        result = {"schema_version": 1, "status": "CLOSED_PAUSE_RECORDINGS_COMPARED",
                  "hardware_actions": [], "git_actions": [], "original_recordings_modified": False,
                  "overall_acceptance": "NOT_ESTABLISHED", "cases": cases}
        output = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
        if options.write:
            (HERE / "comparison.json").write_text(output)
            (HERE / "comparison.md").write_text(markdown(result))
        print(output, end="")
    except (ValueError, FileNotFoundError, json.JSONDecodeError) as error:
        print(json.dumps({"status": "NOT_READY_OR_INVALID", "reason": str(error),
                          "hardware_actions": [], "original_recordings_modified": False}))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
