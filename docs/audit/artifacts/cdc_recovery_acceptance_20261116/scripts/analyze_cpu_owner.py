#!/usr/bin/env python3
"""Offline CPUOWNER before/after analysis; no device access or input mutation.

Only each role's own sample brackets define its denominator. The result is a
switch-out-counter ratio, not instantaneous CPU utilization, SDK CPU time, or
blocked time. SDK/idle tasks are absent, so there is no attributed residual.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

ROLES = ["BLE_TX", "TRANSPORT", "ASYNC_LOG", "SERIAL_TX", "BLE_CTRL", "SERIAL_CTRL", "WIFI_CTRL"]
IDENTITY = ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid",
            "state", "mode", "rows", "profile")
REQUIRED = ("v", "session", "role", "enabled", "available", "known", "conflict", "stale", "reason",
            "taskNumber", "runUs", "t0Us", "t1Us", "observedAtUs", "ageUs", "sampleMinUs",
            "staleAfterUs", "scope", "isr", "sdk", "blocked")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def uint(value: object, bits: int = 64) -> int:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9]+", value):
        raise ValueError("not_unsigned_decimal")
    number = int(value)
    if number >= 1 << bits:
        raise ValueError("counter_out_of_range")
    return number


def parse_line(line: str, tag: str = "CPUOWNER") -> dict[str, str]:
    parts = line.split(",")
    if parts[0] != tag:
        raise ValueError("wrong_reply_tag")
    fields = {}
    for part in parts[1:]:
        key, separator, value = part.partition("=")
        if not separator or not key or key in fields:
            raise ValueError("malformed_or_duplicate_field")
        fields[key] = value
    return fields


def validate(fields: dict, role: int, session: str) -> dict:
    if not all(key in fields for key in REQUIRED):
        raise ValueError("incomplete_CPUOWNER_schema")
    if fields["v"] != "1" or uint(fields["role"], 32) != role:
        raise ValueError("role_or_version_mismatch")
    if not re.fullmatch(r"[0-9A-Fa-f]{16}", fields["session"]) or fields["session"].upper() != session.upper():
        raise ValueError("session_mismatch")
    for key in ("enabled", "available", "known", "conflict", "stale"):
        if fields[key] not in ("0", "1"):
            raise ValueError("invalid_boolean")
    for key, expected in (("scope", "switch_out"), ("isr", "may_include"),
                          ("sdk", "not_isolated"), ("blocked", "unknown")):
        if fields[key] != expected:
            raise ValueError("unsupported_counter_semantics")
    parsed = {key: uint(fields[key], 32 if key == "taskNumber" else 64)
              if fields[key] != "unknown" else None
              for key in ("taskNumber", "runUs", "t0Us", "t1Us", "observedAtUs", "ageUs")}
    sample_min = uint(fields["sampleMinUs"])
    stale_after = uint(fields["staleAfterUs"])
    if not sample_min or stale_after < sample_min:
        raise ValueError("invalid_sample_intervals")
    if fields["known"] == "0":
        if fields["available"] != "0" or any(v is not None for v in parsed.values()) or fields["stale"] != "1":
            raise ValueError("unknown_sample_has_numeric_values")
    else:
        if any(parsed[key] is None for key in ("taskNumber", "t0Us", "t1Us", "observedAtUs")):
            raise ValueError("known_sample_missing_bracket")
        if parsed["t1Us"] < parsed["t0Us"] or parsed["observedAtUs"] != parsed["t1Us"]:
            raise ValueError("invalid_sample_bracket")
        if parsed["ageUs"] is not None and (fields["stale"] == "1") != (parsed["ageUs"] >= stale_after):
            raise ValueError("stale_age_mismatch")
    if fields["available"] == "1":
        if fields["enabled"] != "1" or fields["known"] != "1" or fields["conflict"] != "0" or fields["reason"] != "ok":
            raise ValueError("invalid_available_state")
        if parsed["runUs"] is None or parsed["ageUs"] is None:
            raise ValueError("available_sample_unknown_runtime_or_age")
    elif parsed["runUs"] is not None:
        raise ValueError("unavailable_runtime_is_not_unknown")
    if fields["conflict"] == "1" and fields["reason"] != "writer_conflict":
        raise ValueError("conflict_reason_mismatch")
    if fields["enabled"] == "0" and fields["reason"] != "disabled":
        raise ValueError("disabled_reason_mismatch")
    if fields["known"] == "0" and fields["reason"] not in ("disabled", "not_observed", "writer_conflict", "clock_regressed"):
        raise ValueError("unknown_sample_reason_mismatch")
    return parsed


def config_identity(snapshot: dict) -> tuple[str, ...]:
    fields = snapshot["fields"]
    values = tuple(fields[key] for key in IDENTITY)
    if not re.fullmatch(r"[0-9A-Fa-f]{16}", values[0]):
        raise ValueError("invalid_CONFIG_session")
    return values


def analyze_document(summary: dict, raw_lines: set[str], event_replies: set[tuple]) -> dict:
    report = {
        "schema_version": 1,
        "status": "ANALYZED",
        "window_scope": "independent_owner_sample_brackets_from_before_after_CONFIG_queries_not_capture_endpoints",
        "current_CPU_utilization": "UNKNOWN",
        "SDK_CPU_time": "UNKNOWN",
        "blocked_time": "UNKNOWN",
        "unobserved_tasks_and_core_residual": "UNKNOWN",
        "aggregate_role_percent": None,
        "semantics": [
            "runUs is cumulative at switch-out; the currently running slice is omitted and ISR time may be included.",
            "Each role has its own device-time denominator; do not sum the ratios into a simultaneous CPU accounting table.",
            "Owner runtime includes work outside the SDK; SDK wall minus owner runtime is not blocked time.",
            "Unavailable or stale is unknown, never zero; session and taskNumber must match.",
        ],
        "roles": [],
    }
    snapshots = summary.get("snapshots", {})
    identity_error = None
    try:
        identities = [config_identity(snapshots[phase][key]) for phase in ("before", "after")
                      for key in ("CONFIG?", "CONFIG?_closing")]
        if len(set(identities)) != 1:
            raise ValueError("CONFIG_bracket_or_window_changed")
        for phase in ("before", "after"):
            for key in ("CONFIG?", "CONFIG?_closing"):
                wanted = snapshots[phase][key]["fields"]
                if not any(p == phase and cmd == "CONFIG?" and line in raw_lines and
                           parse_line(line, "CONFIG") == wanted
                           for p, cmd, line in event_replies if cmd == "CONFIG?"):
                    raise ValueError("CONFIG_bracket_not_in_raw_and_event_phase")
        expected = identities[0]
        report["configuration_identity"] = dict(zip(IDENTITY, expected))
    except (KeyError, TypeError, ValueError) as error:
        identity_error = str(error)
        report["configuration_identity_error"] = identity_error
        expected = ("",)
    for role, name in enumerate(ROLES):
        row = {"role": role, "name": name, "status": "UNKNOWN", "reason": None,
               "runtime_delta_us": None, "observation_span_us": None,
               "runtime_per_own_observation_span_percent": None,
               "before": None, "after": None}
        report["roles"].append(row)
        try:
            if identity_error:
                raise ValueError(identity_error)
            parsed = []
            for phase in ("before", "after"):
                snapshot = snapshots[phase][f"CPUOWNER?{role}"]
                fields = snapshot["fields"]
                line = snapshot["reply_line"]
                if parse_line(line) != fields:
                    raise ValueError("summary_fields_differ_from_reply_line")
                if line not in raw_lines or (phase, f"CPUOWNER?{role}", line) not in event_replies:
                    raise ValueError("reply_not_present_in_raw_and_event_phase")
                values = validate(fields, role, expected[0])
                row[phase] = {"fields": fields, "values": values,
                              "host_reply_received_monotonic": snapshot.get("reply_received_monotonic"),
                              "host_query_started_monotonic": snapshot.get("started_monotonic")}
                parsed.append(values)
            for phase in ("before", "after"):
                fields = row[phase]["fields"]
                if fields["available"] != "1" or fields["known"] != "1" or fields["conflict"] != "0":
                    raise ValueError(f"{phase}_unavailable_{fields['reason']}")
                if fields["stale"] != "0":
                    raise ValueError(f"{phase}_stale_historical_sample")
            before, after = parsed
            if before["taskNumber"] != after["taskNumber"]:
                raise ValueError("taskNumber_changed")
            if after["runUs"] < before["runUs"]:
                raise ValueError("runtime_counter_regressed")
            if after["t0Us"] <= before["t1Us"]:
                raise ValueError("overlapping_or_nonprogressing_sample_brackets")
            runtime = after["runUs"] - before["runUs"]
            span = after["observedAtUs"] - before["observedAtUs"]
            row.update(status="PAIRED", reason="same_session_task_fresh_ordered_samples",
                       taskNumber=before["taskNumber"], runtime_delta_us=runtime,
                       observation_span_us=span,
                       bracket_elapsed_min_us=after["t0Us"]-before["t1Us"],
                       bracket_elapsed_max_us=after["t1Us"]-before["t0Us"],
                       runtime_per_own_observation_span_percent=runtime*100/span)
        except (KeyError, TypeError, ValueError) as error:
            row["reason"] = str(error)
    report["paired_roles"] = [r["role"] for r in report["roles"] if r["status"] == "PAIRED"]
    report["unknown_roles"] = [r["role"] for r in report["roles"] if r["status"] != "PAIRED"]
    report["BLE_SDK_separate_query_window"] = {"status": "UNKNOWN", "reason": "not_validated"}
    try:
        if identity_error:
            raise ValueError(identity_error)
        a, b = [snapshots[p]["BLESVC?"] for p in ("before", "after")]
        x, y = a["fields"], b["fields"]
        if x["gen"] != y["gen"] or x["scope"] != "boot" or y["scope"] != "boot":
            raise ValueError("BLESVC_generation_or_scope_changed")
        for phase, snap in (("before", a), ("after", b)):
            line = snap["reply_line"]
            if parse_line(line,"BLESVC") != snap["fields"] or line not in raw_lines or (phase, "BLESVC?", line) not in event_replies:
                raise ValueError("BLESVC_reply_not_in_evidence")
        wall = uint(y["wallUs"]) - uint(x["wallUs"])
        if wall < 0:
            raise ValueError("BLESVC_wall_regressed")
        deltas = {key: (uint(y[key],32)-uint(x[key],32)) % (1<<32)
                  for key in ("started","completed","confOk","confError","confMatched","confUnmatched")}
        report["BLE_SDK_separate_query_window"] = {
            "status": "OBSERVED", "scope": "BLESVC_before_after_query_snapshots_not_owner_or_capture_boundaries",
            "before_host_receipt": a.get("reply_event_monotonic"),
            "after_host_receipt": b.get("reply_event_monotonic"),
            "before_active": x["active"], "after_active": y["active"],
            "counter_deltas_modulo_uint32": deltas, "wall_delta_us": wall,
            "wall_us_per_completed_call": wall/deltas["completed"] if deltas["completed"] else None,
            "CPU_time": "UNKNOWN", "blocked_time": "UNKNOWN", "PC_receipt": "UNKNOWN",
        }
    except (KeyError, TypeError, ValueError) as error:
        report["BLE_SDK_separate_query_window"]["reason"] = str(error)
    return report


def evidence_path(text: str, summary: Path) -> Path:
    path = Path(text)
    for candidate in (path, summary.parent/path.name):
        if candidate.is_file():
            return candidate
    raise ValueError(f"missing_evidence:{text}")


def analyze_file(summary_path: Path) -> dict:
    summary = json.loads(summary_path.read_text())
    raw = evidence_path(summary["raw_file"], summary_path)
    events = evidence_path(summary["events_file"], summary_path)
    raw_bytes = raw.read_bytes()
    actual_hash = hashlib.sha256(raw_bytes).hexdigest()
    if actual_hash != summary["raw_sha256"]:
        raise ValueError("raw_sha256_mismatch")
    # Only newline-complete diagnostic records qualify. Never repair a tail.
    complete = raw_bytes.split(b"\n")[:-1]
    raw_lines = {line.rstrip(b"\r").decode("ascii") for line in complete
                 if line.startswith((b"CPUOWNER,", b"BLESVC,", b"CONFIG,"))}
    replies = set()
    for line in events.read_text().splitlines():
        event = json.loads(line)
        if event.get("type") == "reply":
            replies.add((event.get("phase"), event.get("command"), event.get("line")))
    result = analyze_document(summary, raw_lines, replies)
    result["inputs"] = {"summary": {"path":str(summary_path),"sha256":sha(summary_path)},
                        "raw": {"path":str(raw),"sha256":actual_hash,"bytes":len(raw_bytes)},
                        "events": {"path":str(events),"sha256":sha(events)}}
    result["generator"] = {"path":str(Path(__file__)),"sha256":sha(Path(__file__))}
    result["hardware_actions"] = []
    result["original_recordings_modified"] = False
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = analyze_file(args.summary)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2); stream.write("\n")
    print(json.dumps({"output":str(args.output),"paired_roles":result["paired_roles"],
                      "unknown_roles":result["unknown_roles"],"status":result["status"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
