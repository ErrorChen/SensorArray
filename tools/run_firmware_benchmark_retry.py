#!/usr/bin/env python3
"""Supplement incomplete benchmark windows with bounded read-only query retries.

The original suite runner remains byte-for-byte frozen while its process runs.
This separate entry point reuses it with a process-local Collector subclass.
Only documented read-only queries retry (maximum three attempts, sharing the
original phase/total deadlines). Missing or truncated replies are recorded;
setters are never replayed, and FPS, warmup and capture windows are unchanged.

Use a new --label, for example:
  python3 tools/run_firmware_benchmark_retry.py --port /dev/ttyACM0 --output-directory OUT --label supplement-volt1-01 --mode VOLT --rows 1

Motivation and exact raw offsets:
docs/audit/artifacts/full_benchmark_20261113/volt123-transport-diagnosis-01.json
The existing ERROR records remain authoritative evidence, not overwritten by
supplemental measurements. A full-duration truncated-wire window is still a
transport failure and must not be replaced by a favorable rerun.
"""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path
import re
import time

from capture_reference_fixture import CaptureError
import run_firmware_benchmark as original

MAX_QUERY_ATTEMPTS = 3
RETRY_GAP_SECONDS = 0.05

# This schema is for the currently negotiated PROTO=2 capabilities. Requiring
# end-of-record critical fields detects observed short-but-newline-terminated
# replies, without reinterpreting absent values as zero or a successful state.
REQUIRED = {
    "BUILD?": "idf target project proto",
    "PROTO?": "version wires ctrlMax dataMax caps adcCrc",
    "CAL?": "source schema valid payloadLength",
    "CONFIG?": "session bootId rev boundary state mode rows profile err mgen mrid rgen rrid pgen prid mtermId mterm merr ptermId pterm perr rtermId rterm rerr",
    "MODE?": "state active pending gen rid layout profile route intref refmux stackUnit",
    "RAIL?": "src rail avdd avss rv rs split",
    "PERF?": "pub fresh stale mixed dropOut dropEvent frames heap heapMin",
    "USBSTREAM?": "v dataEvery diagEvery sinkQueued sinkDropped priorityQueued driverAcceptedBytes rxBytes partialWrite timeout driverDrained lineDropped lineOversize discardBytes stdioBound initError bufferBytes enumerated",
    "FPS?": "cfcap ofcap adsgap",
    "FRONTENDS?": "phase hold known present primary secondary ads pErr sErr aErr pCleanup sCleanup aCleanup aHz aRestoreHz quarantine physicalSafe",
    "SPI?": "phase done joined timedOut qualified restored hz actualHz err run cleanup scope",
    "BAT?": "bt valid fresh ageMs lastGoodKnown restore reason",
}

INTEGER_FIELDS = {
    "bootId", "rev", "boundary", "rows", "mgen", "mrid", "rgen", "rrid", "pgen", "prid",
    "mtermId", "ptermId", "rtermId", "proto", "version", "ctrlMax", "dataMax", "schema",
    "valid", "payloadLength", "gen", "rid", "intref", "rail", "avdd", "avss", "rv",
    "pub", "fresh", "stale", "mixed", "dropOut", "dropEvent", "frames", "heap", "heapMin",
    "dataEvery", "diagEvery", "sinkQueued", "sinkDropped", "priorityQueued", "driverAcceptedBytes",
    "rxBytes", "partialWrite", "timeout", "driverDrained", "lineDropped", "lineOversize",
    "discardBytes", "stdioBound", "bufferBytes", "cfcap", "ofcap", "hold", "known", "present",
    "aHz", "aRestoreHz", "quarantine", "done", "joined", "timedOut", "qualified", "restored",
    "hz", "actualHz", "bt", "ageMs", "lastGoodKnown",
}


class InvalidQueryReply(CaptureError):
    pass


def validate_reply(command, fields):
    required = REQUIRED[command].split()
    missing = [key for key in required if not fields.get(key)]
    if missing:
        raise InvalidQueryReply("reply_missing_fields:" + command + ":" + ",".join(missing))
    for key in set(required) & INTEGER_FIELDS:
        try:
            int(fields[key], 10)
        except ValueError as error:
            raise InvalidQueryReply(f"reply_invalid_integer:{command}:{key}") from error
    if command == "CONFIG?":
        if not re.fullmatch(r"[0-9A-Fa-f]{16}", fields["session"]) or not re.fullmatch(r"[CVR]{8}", fields["profile"]):
            raise InvalidQueryReply("reply_invalid_config_identity")
        if int(fields["rows"]) not in range(1, 9):
            raise InvalidQueryReply("reply_invalid_config_rows")
    if command == "MODE?" and fields["stackUnit"] != "bytes":
        raise InvalidQueryReply("reply_invalid_mode_stack_unit")
    return fields


class RetryBenchmarkCollector(original.BenchmarkCollector):
    def __init__(self, *args, **kwargs):
        self.query_attempts = []
        super().__init__(*args, **kwargs)
        self.event("runner_variant", variant="bounded_readonly_query_retry_v1",
                   original_source_sha256=hashlib.sha256(Path(original.__file__).read_bytes()).hexdigest(),
                   max_query_attempts=MAX_QUERY_ATTEMPTS, setters_retried=False,
                   phase_deadlines_extended=False, capture_window_changed=False)

    def query(self, command, tag, phase_deadline):
        if command not in REQUIRED or not command.endswith("?"):
            raise CaptureError("retry_wrapper_requires_known_readonly_query:" + command)
        limit = min(phase_deadline, self.total_deadline)
        last_error = "phase_deadline_expired"
        baseline_faults = self.fault_events
        attempts = 0
        for attempt in range(1, MAX_QUERY_ATTEMPTS + 1):
            if time.monotonic() >= limit:
                break
            attempts = attempt
            entry = {"command": command, "phase": self.phase, "attempt": attempt,
                     "started_monotonic": time.monotonic(), "phase_deadline": limit}
            self.query_attempts.append(entry)
            self.event("readonly_query_attempt", command=command, attempt=attempt, phase_deadline=limit)
            try:
                fields = super().query(command, tag, limit)
                validate_reply(command, fields)
                if self.fault_events > baseline_faults:
                    raise CaptureError("firmware_fault_during_readonly_query:" + command)
                entry["outcome"] = "complete_reply"
                if attempt > 1:
                    self.event("readonly_query_recovered", command=command, attempts=attempt)
                return fields
            except CaptureError as error:
                last_error = str(error)
                entry.update(outcome="failed", error=last_error)
                retryable = (isinstance(error, InvalidQueryReply) or last_error == "reply_deadline_expired" or
                             last_error.startswith("reply_timeout:") or "reason=snapshot_busy" in last_error)
                self.event("readonly_query_failure", command=command, attempt=attempt,
                           error=last_error, retryable=retryable)
                if not retryable or self.fault_events > baseline_faults:
                    raise
                if attempt == MAX_QUERY_ATTEMPTS or time.monotonic() >= limit:
                    break
                # Drain continuously while backing off; no sleep/read gap and
                # no new per-attempt phase budget. Late replies remain raw.
                gap_end = min(limit, time.monotonic() + RETRY_GAP_SECONDS)
                while time.monotonic() < gap_end:
                    self.pump(gap_end)
                    self.lines.clear()
                if self.fault_events > baseline_faults:
                    raise CaptureError("firmware_fault_during_readonly_query:" + command)
            finally:
                entry["finished_monotonic"] = time.monotonic()
        raise CaptureError(f"readonly_query_failed:{command}:attempts={attempts}:{last_error}")

    def run(self):
        result = super().run()
        result.update(runner_variant="bounded_readonly_query_retry_v1", query_attempts=self.query_attempts,
                      query_retry_limit=MAX_QUERY_ATTEMPTS, query_retry_shared_phase_deadline=True,
                      setters_retried=False, base_runner_source_sha256=hashlib.sha256(Path(original.__file__).read_bytes()).hexdigest())
        return result


@contextmanager
def retry_collector_selected():
    # Constructor injection is confined to this new process/module instance;
    # it never changes the frozen file or another running suite's process.
    previous = original.BenchmarkCollector
    original.BenchmarkCollector = RetryBenchmarkCollector
    try:
        yield
    finally:
        original.BenchmarkCollector = previous


def run_benchmark(options, **kwargs):
    with retry_collector_selected():
        return original.run_benchmark(options, **kwargs)


def main(argv=None):
    with retry_collector_selected():
        return original.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
