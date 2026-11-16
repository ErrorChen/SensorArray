#!/usr/bin/env python3
"""Offline source/work-count review of closed screen-06/07 evidence only.

No device, build, firmware mutation or CPU timing simulation. Original evidence
is never rewritten. Counts of operations are not measurements of CPU cost.
"""
import base64
import bisect
import collections
import hashlib
import json
from pathlib import Path
import statistics
import time
import zlib

HERE = Path(__file__).resolve().parent
STAGE = HERE.parent
ROOT = next(p for p in HERE.parents if (p / "main/output/sensorarrayAsyncLog.c").is_file())
INPUTS = {}


def read(path):
    b = path.read_bytes()
    INPUTS[str(path.relative_to(ROOT))] = {"sha256": hashlib.sha256(b).hexdigest(), "bytes": len(b)}
    return b


def load(path):
    return json.loads(read(path))


def fields(line):
    out = {}
    for part in line.strip().split(","):
        if "=" in part:
            k, v = part.split("=", 1)
            out.setdefault(k, v)
    return out


def delta_query(summary, command):
    a, b = (summary["snapshots"][p][command] for p in ("before", "after"))
    numeric = {k: int(b["fields"][k])-int(v) for k, v in a["fields"].items()
               if v.isdecimal() and b["fields"].get(k, "").isdecimal()}
    return {"before": a, "after": b, "window_seconds": b["monotonic"]-a["monotonic"],
            "numeric_differences_not_all_counters": numeric}


def evidence(screen):
    d = STAGE / ("screen-" + screen)
    s = load(d / f"screen-{screen}-volt1.summary.json")
    a = load(d / f"screen-{screen}-analysis-01.json")["captures"][0]
    start, end = s["capture_monotonic_start"], s["capture_monotonic_end"]
    queries = {c: delta_query(s, c) for c in ("PERF?", "USBQ?", "BLESVC?", "SCHED?", "SCHEDWHY?")}
    svc = queries["BLESVC?"]
    sd = svc["numeric_differences_not_all_counters"]
    sdk = {"query_window_seconds": svc["window_seconds"], "completed": sd["completed"],
           "wall_us": sd["wallUs"], "mean_wall_us_per_completed": sd["wallUs"]/sd["completed"],
           "conf_ok_delta": sd["confOk"], "conf_error_delta": sd["confError"],
           "meaning": "SDK-call elapsed wall, includes preemption/wait; CONF is local event, not peer ACK."}
    cpu = {}
    for role in range(7):
        key = f"CPUOWNER?{role}"
        if key not in s["snapshots"]["before"]:
            continue
        x, y = (s["snapshots"][p][key]["fields"] for p in ("before", "after"))
        eligible = all(v[k] == "1" for v in (x, y) for k in ("enabled", "available", "known"))
        eligible &= all(v[k] == "0" for v in (x, y) for k in ("stale", "conflict"))
        eligible &= x["session"] == y["session"] and x["taskNumber"] == y["taskNumber"]
        row = {"before": x, "after": y, "eligible": eligible, "scheduled_share_percent": None}
        if eligible:
            run = int(y["runUs"])-int(x["runUs"])
            span = int(y["t1Us"])-int(x["t1Us"])
            assert run >= 0 and span > 0
            row.update(run_us=run, sample_span_us=span, scheduled_share_percent=run/span*100)
        cpu[str(role)] = row
    dbg = load(d / f"screen-{screen}-debug.debug.summary.json")
    raw = read(ROOT / dbg["raw_file"])
    assert hashlib.sha256(raw).hexdigest() == dbg["raw_sha256"]
    events = [json.loads(l) for l in read(ROOT / dbg["events_file"]).splitlines()]
    rx = [x for x in events if x["type"] == "rx"]
    ends = [x["offset"]+x["length"] for x in rx]
    tags, byte_counts, ns = collections.Counter(), collections.Counter(), collections.Counter()
    first = {}
    pos = 0
    for line in raw.splitlines(keepends=True):
        pos += len(line)
        event = rx[bisect.bisect_left(ends, pos)]
        if not start <= event["monotonic"] <= end:
            continue
        text = line.decode("ascii", errors="replace").strip()
        tag = text.split(",", 1)[0]
        tags[tag] += 1
        byte_counts[tag] += len(line)
        first.setdefault(tag, text)
        if tag == "SF50":
            ns[fields(text)["n"]] += 1
    ble_counts, ble_bytes, messages = collections.Counter(), collections.Counter(), {}
    for line in read(d / f"screen-{screen}-ble.ble.notifications.jsonl").splitlines():
        event = json.loads(line)
        if event["type"] != "notification" or not start <= event["monotonic_ns"]/1e9 <= end:
            continue
        data = base64.b64decode(event["payload_base64"], validate=True)
        channel = event["channel"]
        ble_counts[channel] += 1
        ble_bytes[channel] += len(data)
        if data.startswith(b"G,"):
            header, payload = data.split(b"\n", 1)
            _, ch, mid, index, count, length, total, crc = header.decode().split(",")
            assert len(payload) == int(length)
            key = ch, mid
            item = messages.setdefault(key, {"count": int(count), "total": int(total), "crc": int(crc, 16), "pieces": {}})
            item["pieces"][int(index)] = payload
    logical = collections.Counter()
    message_sizes = collections.defaultdict(list)
    for (ch, _), m in messages.items():
        if set(m["pieces"]) != set(range(m["count"])):
            continue
        data = b"".join(m["pieces"][n] for n in range(m["count"]))
        assert len(data) == m["total"] and zlib.crc32(data) == m["crc"]
        logical[ch] += 1
        message_sizes[ch].append(len(data))
    duration = end-start
    out = {"capture_seconds": duration, "source_count_scope": "original analyzer complete capture window",
           "USB": a["capture_counts"], "BLE": a["ble"]["capture_frame_counts"], "SDK": sdk,
           "queries": queries, "cpu_owner": cpu,
           "debug": {"line_count": sum(tags.values()), "bytes": sum(byte_counts.values()),
                     "tags": dict(tags), "bytes_by_tag": dict(byte_counts), "first_lines": first,
                     "SF50_window_frame_counts": dict(ns),
                     "time_assignment": "host receipt of chunk ending line; not device event time"},
           "ble_wire": {"notifications": dict(ble_counts), "bytes": dict(ble_bytes),
                        "all_notifications_per_second": sum(ble_counts.values())/duration,
                        "log_fraction_notifications": ble_counts["L"]/sum(ble_counts.values()),
                        "arithmetic_no_log_upper_at_same_notify_rate_fps": sum(ble_counts.values())/duration/3,
                        "complete_fragmented_messages": dict(logical),
                        "message_size_distribution": {k: dict(collections.Counter(v)) for k, v in message_sizes.items()}}}
    return out, s


def metadata_work(summary):
    raw = read(ROOT / summary["raw_file"])
    assert hashlib.sha256(raw).hexdigest() == summary["raw_sha256"]
    events = [json.loads(l) for l in read(ROOT / summary["events_file"]).splitlines()]
    lo = next(e["offset"] for e in events if e["type"] == "capture_start")
    hi = next(e["offset"] for e in events if e["type"] == "capture_end")
    keys = "seq rows cells gen rid session bootId rev mgen mrid rgen rrid pgen prid".split()
    frame_start, payload, pos = None, [], 0
    visits, end_scans, tokens, frames, lengths = 0, 0, 0, 0, []
    for line in raw.splitlines(keepends=True):
        begin = pos; pos += len(line)
        if line.startswith(b"V,"):
            frame_start, payload = begin, [line]
        elif frame_start is not None and (line.startswith(b"D") or line.startswith(b"P")):
            payload.append(line)
        elif frame_start is not None and line.startswith(b"K,"):
            if frame_start >= lo and pos <= hi:
                assert zlib.crc32(b"".join(payload)) == int(fields(line.decode())["crc"], 16)
                header = payload[0].decode().strip()
                parts = header.split(",")
                repeated = {}
                for key in keys:
                    end_scans += len(header)
                    for part in parts:
                        visits += 1
                        if part.startswith(key + "="):
                            repeated[key] = part.split("=", 1)[1]
                            break
                once = fields(header)
                assert repeated == {k: once[k] for k in keys}
                tokens += len(parts); frames += 1; lengths.append(sum(map(len, payload))+len(line))
            frame_start, payload = None, []
    assert frames == summary["capture_parsed_frames"]
    return {"frames": frames, "packet_bytes_min": min(lengths), "packet_bytes_max": max(lengths),
            "packet_bytes_mean": statistics.mean(lengths), "field_lookups_per_frame": len(keys),
            "existing_token_visits": visits, "single_token_pass_visits": tokens,
            "repeated_line_end_search_bytes": end_scans, "values_equal_on_replayed_headers": True,
            "limitation": "Python source-algorithm work count; no C/target time, cache or CPU-cost claim. Duplicate/invalid fields need production regression before any rewrite."}


def main():
    six, _ = evidence("06")
    seven, summary = evidence("07")
    result = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "scope": "closed evidence offline review; no hardware, target build or production edits by this script",
        "screen06": six, "screen07": seven, "metadata_replay": metadata_work(summary),
        "source_findings": [
            "main/main.c RunOneFrame calls AsyncLogPublishFrameSnapshot, which executes TextProtocolBuildFrame before queue publication: scientific ASCII/CRC runs on Core1 producer, not AsyncLog owner.",
            "AsyncLog PopFrame copies full frame and full 2304-byte text packet from immutable PSRAM slots into PSRAM workspace. PublishData reparses 14 header fields in caller/AsyncLog context, then copies only actual payload into transport slot; USB separately copies payload.",
            "PrecisionUpdate fixed 64-cell CAP loop ran every output frame; only reader was unreachable PREC20 and existing #if0 netStatus. Removed in narrow candidate; VOLT capValidMask=0 skips floating-point update, so do not attribute 30 s CPU to it.",
            "Compact summaries are computed at window n=15,30,45 for USB and n=50 for network; only n50 resets summary. They are not timed triggers or per-frame formatting. Each computation calls 5 linear stack high-water scans, formats integer telemetry, and reads heap stats.",
            "Text acquisition events were already formatted by Core1 before being copied to AsyncLog. AsyncLog TEXT fwrite has no second printf formatting pass. VFS assembler handles newline chunks with memchr/memcpy, not per-byte critical sections.",
            "AsyncLog outer wait blocks up to 1000ms. Each drain is bounded by event/frame capacity and checkpoints; continuously arriving frames/events can sustain readiness. WorkBudget elapsed time includes preemption; delay is not own CPU time.",
            "BLE computes payload CRC before slot allocation and again at dequeue. Rejected admission therefore pays CRC CPU work. These checks protect queued-payload integrity and must be retained; table/ROM-compatible acceleration needs byte-identical proof.",
        ],
        "candidate_order": [
            {"candidate": "remove unreachable PREC20 private statistics", "state": "implemented separately and host-equivalence tested; HIL pending", "scope": "no visible output/priority/queue/budget/CPU clock change"},
            {"candidate": "single-pass TransportParseFrameMeta", "state": "NOT_IMPLEMENTED", "validation": "same immutable identity, old first-duplicate semantics, exact key boundaries, malformed/range rejection; actual C differential then CPU/HIL comparison"},
            {"candidate": "profile PopFrame / publish / UpdateSummary / CompactSummary / event output phases", "state": "NOT_IMPLEMENTED", "validation": "bounded aggregates only; elapsed timing must not be labeled isolated CPU; no per-frame extra diagnostic output"},
            {"candidate": "-O2 keeping assertions enabled and CPU160MHz", "state": "NOT_IMPLEMENTED", "validation": "IDF Kconfig maps performance to -O2; assertions are separate. Broad independent build factor, full regression and same-clock HIL; no assumption about ADC physical fault causality or achieved BLE throughput."},
        ],
        "limits": ["CPU samples and SDK/diagnostic query windows differ from capture and from each other.",
                   "Owner runtime may include ISR and excludes currently running unswitched slice; other BT SDK tasks are not isolated.",
                   "CONF ok is local event, never peer application receipt; host notification spacing is not radio event timing.",
                   "~10% LOG notifications cannot alone explain the gap: even arithmetic removal at unchanged total notification rate stays near36FPS, not60FPS.",
                   "No phase-level CPU measurements exist yet; remaining AsyncLog 25% attribution is unresolved."]}
    for source in ("main/output/sensorarrayAsyncLog.c", "core/transport/sensorarrayTransportFrameMeta.c",
                   "components/sensorarrayBle/sensorarrayBle.c", "sdkconfig"):
        read(ROOT / source)
    result["inputs"] = INPUTS
    result["script_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    path = HERE / "screen-07-async-work-review-01.json"
    with path.open("x") as out:
        json.dump(result, out, indent=2)
    print(json.dumps({"output": str(path.relative_to(ROOT)), "metadata": result["metadata_replay"],
                      "screen07_debug_lines": seven["debug"]["line_count"],
                      "screen07_sdk": seven["SDK"], "scope": "offline_not_timing"}))


if __name__ == "__main__":
    main()
