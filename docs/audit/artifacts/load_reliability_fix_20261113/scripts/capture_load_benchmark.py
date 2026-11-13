#!/usr/bin/env python3
"""Capture the load-reliability firmware with SERIAL/SCHED evidence.

This independent entry point reuses the frozen benchmark/retry collectors.
Defaults remain 120 s per configuration, 600 s stability, and 5 s warmup.
Only read-only queries retry, at most three times within the existing deadline.
Fault/watchdog handling, setter behavior, raw preservation and cleanup are not
overridden. No NVS save, reset or FPS setting is added.

Example (hardware access belongs to the operator):
  python3 docs/audit/artifacts/load_reliability_fix_20261113/scripts/capture_load_benchmark.py \
    --port /dev/ttyACM0 --output-directory OUT --label load-volt1-01 --mode VOLT --rows 1
"""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools"))
from capture_reference_fixture import CaptureError
import run_firmware_benchmark_retry as retry

base = retry.original
REQUIRED_CAPABILITIES = frozenset(("config_query_v1", "serial_sink_v1", "work_budget_v1"))
WORKERS = ("bt", "tr", "log", "ser", "bc", "sc", "wc")
EXTRA_REQUIRED = {
    "SERIAL?": "normalSubmitted prioritySubmitted normalExpired priorityExpired priorityRejected driverShortWrite driverWriteError txYield maxQueueAgeUs driverTxBufferBytes driverAcceptedBytes partialWrite",
    "SCHED?": "initError registered idleKnown tickHz budgetTicks maxUnits idleGen idleAgeTicks maxIdleGapTicks " + " ".join(WORKERS),
}
CONFIG_CONSISTENCY = base.IDENTITY + ("state", "mode", "rows", "profile", "boundary", "err")
BASE_VALIDATOR = retry.validate_reply
VARIANT = "load_reliability_diagnostics_v1"


def unsigned(value, command, field, bits=32):
    if not re.fullmatch(r"[0-9]+", value or "") or int(value) >= 1 << bits:
        raise retry.InvalidQueryReply(f"reply_invalid_unsigned:{command}:{field}")
    return int(value)


def validate_reply(command, fields):
    if command not in EXTRA_REQUIRED:
        return BASE_VALIDATOR(command, fields)
    missing = [key for key in EXTRA_REQUIRED[command].split() if not fields.get(key)]
    if missing:
        raise retry.InvalidQueryReply("reply_missing_fields:" + command + ":" + ",".join(missing))
    if command == "SERIAL?":
        for key in EXTRA_REQUIRED[command].split():
            unsigned(fields[key], command, key, 64 if key == "driverAcceptedBytes" else 32)
    else:
        if not re.fullmatch(r"0x[0-9A-Fa-f]{1,8}", fields["initError"]):
            raise retry.InvalidQueryReply("reply_invalid_error:SCHED?:initError")
        for key in ("registered", "idleKnown"):
            if fields[key] not in {"0", "1"}:
                raise retry.InvalidQueryReply("reply_invalid_boolean:SCHED?:" + key)
        for key in ("tickHz", "budgetTicks", "maxUnits", "idleGen"):
            unsigned(fields[key], command, key)
        for key in ("idleAgeTicks", "maxIdleGapTicks"):
            if fields[key] != "unknown":
                unsigned(fields[key], command, key)
        for worker in WORKERS:
            values = fields[worker].split("/")
            if len(values) != 3:
                raise retry.InvalidQueryReply("reply_invalid_worker_tuple:SCHED?:" + worker)
            for value in values:
                unsigned(value, command, worker)
    return fields


def scheduler_health(fields):
    reasons = []
    if int(fields["initError"], 16) != 0:
        reasons.append("hook_init_error")
    if fields["registered"] != "1":
        reasons.append("hook_unregistered")
    if fields["idleKnown"] != "1" or fields["idleGen"] == "0":
        reasons.append("idle_progress_unknown")
    for key in ("idleAgeTicks", "maxIdleGapTicks"):
        if fields[key] in {"unknown", "4294967295"}:
            reasons.append(key + "_unknown")
    if any(int(fields[key]) == 0 for key in ("tickHz", "budgetTicks", "maxUnits")):
        reasons.append("scheduler_budget_unavailable")
    return {"usable": not reasons, "reasons": reasons,
            "interpretation": "Individually sampled idle/checkpoint counters; yields are not CPU utilization or proof of watchdog safety."}


class LoadBenchmarkCollector(retry.RetryBenchmarkCollector):
    def __init__(self, *args, **kwargs):
        self.load_capabilities_verified = False
        self.load_proto = None
        self.load_query_windows = []
        self.load_snapshot_attempts = []
        self.load_scheduler_health = {}
        super().__init__(*args, **kwargs)
        self.event("load_runner_variant", variant=VARIANT,
                   required_capabilities=sorted(REQUIRED_CAPABILITIES),
                   scheduler_workers=list(WORKERS), fault_policy="unchanged_fail_fast")

    def query(self, command, tag, phase_deadline):
        entry = {"command": command, "phase": self.phase,
                 "started_monotonic": time.monotonic(), "phase_deadline": min(phase_deadline, self.total_deadline),
                 "attempt_begin": len(self.query_attempts)}
        self.load_query_windows.append(entry)
        try:
            if command in EXTRA_REQUIRED and not self.load_capabilities_verified:
                raise CaptureError("load_diagnostic_capabilities_not_verified")
            fields = super().query(command, tag, phase_deadline)
            entry["fields"] = dict(fields)
            if command == "PROTO?":
                self.load_proto = dict(fields)
                missing = REQUIRED_CAPABILITIES - set(fields.get("caps", "").split("/"))
                if fields.get("version") != "2" or missing:
                    raise CaptureError("load_diagnostic_capability_required:" + ",".join(sorted(missing or {"PROTO2"})))
                self.load_capabilities_verified = True
            entry["outcome"] = "complete_reply"
            return fields
        except CaptureError as error:
            entry.update(outcome="failed", error=str(error))
            raise
        finally:
            entry["finished_monotonic"] = time.monotonic()
            entry["attempt_end"] = len(self.query_attempts)

    def snapshot_window(self, name, deadline):
        self.phase = name
        if not self.load_capabilities_verified:
            raise CaptureError("load_diagnostic_capabilities_not_verified")
        queries = base.QUERIES + (("SERIAL?", "SERIAL"), ("SCHED?", "SCHED"))
        for attempt in range(2):
            window = {}
            evidence = {"name": name, "attempt": attempt + 1, "started_monotonic": time.monotonic(),
                        "phase_deadline": min(deadline, self.total_deadline), "queries": window}
            self.load_snapshot_attempts.append(evidence)
            try:
                for command, tag in queries + (("CONFIG?", "CONFIG"),):
                    closing = command == "CONFIG?" and "CONFIG?" in window
                    fields = self.query(command, tag, deadline)
                    timing = self.load_query_windows[-1]
                    finished = timing["finished_monotonic"]
                    record = {"fields": fields, "monotonic": finished, "t": finished - self.started,
                              "started_monotonic": timing["started_monotonic"], "finished_monotonic": finished,
                              "attempt_begin": timing["attempt_begin"], "attempt_end": timing["attempt_end"]}
                    window["CONFIG?_closing" if closing else command] = record
                opening, closing = window["CONFIG?"]["fields"], window["CONFIG?_closing"]["fields"]
                if any(closing.get(key) != opening.get(key) for key in CONFIG_CONSISTENCY):
                    evidence["outcome"] = "configuration_changed"
                    self.event("snapshot_retry", name=name, reason="configuration_revision_changed", attempt=attempt + 1)
                    continue
                self.snapshots[name] = window
                health = scheduler_health(window["SCHED?"]["fields"])
                self.load_scheduler_health[name] = health
                if not health["usable"]:
                    evidence["outcome"] = "scheduler_unavailable"
                    self.event("scheduler_evidence_unavailable", name=name, **health)
                    raise CaptureError("scheduler_evidence_unavailable:" + name + ":" + ",".join(health["reasons"]))
                evidence["outcome"] = "coherent_configuration"
                self.event("snapshot_window", name=name, revision=closing.get("rev"), attempt=attempt + 1,
                           diagnostics="SERIAL/SCHED", config_closes_diagnostics=True)
                return window
            except CaptureError as error:
                evidence["error"] = str(error)
                raise
            finally:
                evidence["finished_monotonic"] = time.monotonic()
        raise CaptureError("snapshot_window_configuration_changed:" + name)

    def run(self):
        result = super().run()
        usable = all(self.load_scheduler_health.get(name, {}).get("usable") for name in ("before", "after"))
        result.update(runner_variant=VARIANT, required_capabilities=sorted(REQUIRED_CAPABILITIES),
                      diagnostic_capabilities_verified=self.load_capabilities_verified, diagnostic_proto=self.load_proto,
                      scheduler_workers=list(WORKERS), scheduler_health=self.load_scheduler_health,
                      diagnostic_query_windows=self.load_query_windows, diagnostic_snapshot_attempts=self.load_snapshot_attempts,
                      diagnostic_observation_accepted=result.get("status") == "CAPTURED" and usable,
                      scheduler_counter_semantics="independent uint32 snapshots; wrap modulo 2^32; yields are not CPU utilization",
                      serial_counter_semantics="local queue/driver observations, not PC delivery",
                      load_runner_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      retry_runner_source_sha256=hashlib.sha256(Path(retry.__file__).read_bytes()).hexdigest(),
                      fault_policy="unchanged_fail_fast", default_capture_seconds=120, default_stability_seconds=600,
                      default_warmup_seconds=5)
        return result


@contextmanager
def load_collector_selected():
    # Process-local injection only; frozen files and other processes are intact.
    previous = base.BenchmarkCollector, retry.REQUIRED, retry.validate_reply
    base.BenchmarkCollector = LoadBenchmarkCollector
    retry.REQUIRED = {**retry.REQUIRED, **EXTRA_REQUIRED}
    retry.validate_reply = validate_reply
    try:
        yield
    finally:
        base.BenchmarkCollector, retry.REQUIRED, retry.validate_reply = previous


def run_benchmark(options, **kwargs):
    with load_collector_selected():
        return base.run_benchmark(options, **kwargs)


def main(argv=None):
    with load_collector_selected():
        return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
