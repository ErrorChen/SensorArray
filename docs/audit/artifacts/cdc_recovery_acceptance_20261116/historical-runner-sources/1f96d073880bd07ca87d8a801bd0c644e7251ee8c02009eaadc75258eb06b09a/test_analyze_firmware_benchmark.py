#!/usr/bin/env python3
"""Synthetic boundary regressions. These tests are not new HIL measurements."""
import gzip
import base64
import hashlib
import json
from pathlib import Path
import re
import tempfile
import unittest
import zlib

from analyze_firmware_benchmark import analyse_directory, analyse_capture, compact_row, write_report, PERF_FIELDS, Group
from text_protocol import TextProtocolParser, parse_fields
from plot_firmware_benchmark import extract_rates
from test_reference_fixture import capacitance, resistance, finish, mixed, identity


# Exact recorded encoder output from suite-01-mixed1.rx.bin, seq 222078.
# Full source SHA256: 9be7df2b642bc39b176e4cc8dfed9433b8dfc5f860b347336a3999621eb958c7.
# Kept unchanged, including its original wire CRC; this specimen is not synthetic.
RECORDED_MIXED1 = [
    "M,seq=222078,ts=11057675380,rows=1,cells=8,rgen=59,rrid=187,pgen=130,prid=188,profile=CNNNNNNN,expected=00000000000000FF,acquired=00000000000000FF,fmt=mix1,session=0DCB1029EC3CCA0D,bootId=7,rev=188,mgen=128,mrid=186",
    "MR,s=1,m=CAP,unit=pF,scale=-6,expected=FF,acquired=FF,valid=FF,fresh=FF,error=00,fmt=pf6,D=80382093,58020196,58927840,78683541,65596990,56986701,59853677,61666518",
    "K,seq=222078,rgen=59,rrid=187,pgen=130,prid=188,crc=39A56F62,session=0DCB1029EC3CCA0D,bootId=7,rev=188",
]

# Exact original lines from retry-01-volt1.rx.bin. The second ends mid-heap
# token and has none of the emitted tSlot/.../trunc suffix; never repair it.
RECORDED_STK50_COMPLETE = "STK50,unit=bytes,seq=233945,log=16384/12792,transport=6144/3204,usb=0/0,bleTx=4096/1948,bleCtrl=6144/5236,serialCtrl=6144/1808,heap=8377960/8341148/8336464,tSlot=1/4,ta=0,ts=0,tr=0,tq=0/0,bSlot=0/6,ba=120,bs=0,br=0,bc=0,bf=0,bg=0,bconf=0,trunc=0"
RECORDED_STK50_TRUNCATED = "STK50,unit=bytes,seq=237430,log=16384/12792,transport=6144/3204,usb=0/0,bleTx=4096/1948,bleCtrl=6144/5236,serialCtrl=6144/1808,heap=8377960/8341148/8336"


def synthetic_perf(**overrides):
    fields = {key: 0 for key in PERF_FIELDS}
    fields["usbStage"] = "queued"
    fields.update(overrides)
    return "PERF," + ",".join(f"{key}={fields[key]}" for key in PERF_FIELDS)


def mixed_config(profile="CVRCVRCV", rows=1, recorded=False):
    fields = ({"session": "0DCB1029EC3CCA0D", "bootId": "7", "rev": "188", "mgen": "128", "mrid": "186",
               "rgen": "59", "rrid": "187", "pgen": "130", "prid": "188"} if recorded else
              {"session": "AABBCCDDEE112233", "bootId": "1", "rev": "1", "mgen": "1", "mrid": "1",
               "rgen": "1", "rrid": "1", "pgen": "1", "prid": "1"})
    fields.update({"mode": "MIXED" if len(set(profile)) > 1 else {"C": "CAP", "V": "VOLT", "R": "RES"}[profile[0]],
                   "rows": str(rows), "profile": profile})
    return {"rowmodes_requested": profile,
            "snapshots": {phase: {"CONFIG?": {"fields": dict(fields)}} for phase in ["before", "after"]}}


def synthetic_benchmark_mixed(rows, seq=1):
    """Synthetic protocol regression only; never used as physical evidence."""
    profile = "CVRCVRCV"[:rows] + "N" * (8 - rows)
    mask = (1 << (rows * 8)) - 1
    lines = [f"M,seq={seq},ts={seq * 1000000},rows={rows},cells={rows * 8},rgen=1,rrid=1,pgen=1,prid=1,"
             f"profile={profile},expected={mask:X},acquired={mask:X},fmt=mix1" + identity()]
    for row, mode in enumerate(profile[:rows], 1):
        name, unit, scale, fmt = {"C": ("CAP", "pF", -6, "pf6"), "V": ("VOLT", "V", -6, "uv-x"), "R": ("RES", "ohm", -3, "mohm-x")}[mode]
        lines.append(f"MR,s={row},m={name},unit={unit},scale={scale},valid=FF,fresh=FF,error=00,expected=FF,acquired=FF,fmt={fmt},D=" + ",".join(["1000000"] * 8))
    return finish(lines, seq, mixed=True)


class BenchmarkAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def capture(self, chunks, label="res1-01", start=1.0, end=3.0, requested=2.0,
                mode="RES", rows=1, compressed=False, extra=None):
        payload = b""
        events = []
        for when, lines in chunks:
            wire = ("\n".join(lines) + "\n").encode("ascii")
            events.append({"type": "rx", "offset": len(payload), "length": len(wire),
                           "t": when, "phase": "capture" if start <= when <= end else "prepare" if when < start else "cleanup"})
            payload += wire
        events += [{"type": "capture_start", "t": start}, {"type": "capture_end", "t": end}]
        events.sort(key=lambda event: event["t"])
        raw = self.root / (label + (".rx.bin.gz" if compressed else ".rx.bin"))
        raw.write_bytes(gzip.compress(payload) if compressed else payload)
        summary = {"status": "CAPTURED", "mode_requested": mode, "rows_requested": rows,
                   "capture_requested_seconds": requested, "capture_seconds": end - start,
                   "raw_sha256": hashlib.sha256(payload).hexdigest()}
        summary.update(extra or {})
        (self.root / f"{label}.summary.json").write_text(json.dumps(summary))
        (self.root / f"{label}.events.jsonl").write_text("".join(json.dumps(x) + "\n" for x in events))
        return analyse_capture(self.root, label, summary)

    def test_warmup_cleanup_and_straddling_do_not_enter_capture_rates(self):
        boundary = resistance(rows=1, seq=2, timestamp=2000000)
        result = self.capture([(0.2, resistance(rows=1, seq=1, timestamp=1000000)),
                               (0.9, boundary[:-1]), (1.1, boundary[-1:]),
                               (1.5, resistance(rows=1, seq=3, timestamp=3000000)),
                               (2.5, resistance(rows=1, seq=5, timestamp=5000000)),
                               (3.1, resistance(rows=1, seq=6, timestamp=6000000))])
        self.assertEqual(result["window"]["complete_frame_phases"],
                         {"warmup": 1, "boundary_straddling": 1, "capture": 2, "cleanup": 1})
        group = result["configuration_groups"][0]
        self.assertEqual(group["counts"]["missing_frames"], 1)
        self.assertEqual(group["rates"]["capture_fps"], 1.0)
        self.assertEqual(group["rates"]["host_complete_receive_fps"], 1.0)

    def test_partial_crc_before_acquisition_is_invalid_not_route_fault(self):
        lines = resistance(rows=1, seq=1, timestamp=1000000, overrides={0: "X02"})
        lines[0] = re.sub(r",fresh=[^,]*", ",fresh=FE", lines[0])
        lines[0] = re.sub(r",acquired=[^,]*", ",acquired=FE", lines[0])
        lines = finish(lines[:-1], 1)
        result = self.capture([(1.2, lines), (1.3, ["ADSDISPOSITION,seq=1,boundedCrc=1,abortError=0x0,firstError=0x109,allCrcRun=0,limit=3,scope=ads_sweep"]),
                               (2.2, resistance(rows=1, seq=2, timestamp=2000000))])
        counts = result["capture_counts"]
        self.assertEqual(counts["complete_frames"], 2)
        self.assertEqual(counts["unacquired_points"], 1)
        self.assertEqual(counts["not_fresh_points"], 1)
        self.assertEqual(result["capture_invalid_reasons"]["X02"], 1)
        self.assertFalse(result["partial_freshness_is_route_fault"])
        self.assertEqual(result["faults_and_resets"]["FAULT"]["counts_by_receipt_phase"], {})

    def test_bad_crc_and_trailer_identity_are_not_usable_samples(self):
        bad_crc = resistance(rows=1, seq=1)
        bad_crc[-1] = re.sub(r"crc=[0-9A-F]+", "crc=00000000", bad_crc[-1])
        bad_id = resistance(rows=1, seq=2)
        bad_id[-1] = bad_id[-1].replace("rev=1", "rev=2")
        result = self.capture([(1.1, bad_crc), (1.2, bad_id), (2.1, resistance(rows=1, seq=3))])
        self.assertEqual(result["capture_counts"]["ascii_crc_errors"], 1)
        self.assertEqual(result["capture_counts"]["identity_errors"], 1)
        self.assertEqual(result["capture_counts"]["observed_points"], 8)

    def test_host_complete_intervals_keep_invalid_points_and_reject_corrupt_frames(self):
        bad_crc = resistance(rows=1, seq=3, timestamp=3000000)
        bad_crc[-1] = re.sub(r"crc=[0-9A-F]+", "crc=00000000", bad_crc[-1])
        bad_id = resistance(rows=1, seq=4, timestamp=4000000)
        bad_id[-1] = bad_id[-1].replace("rev=1", "rev=2")
        bad_units = resistance(rows=1, seq=5, timestamp=5000000)
        bad_units[0] = bad_units[0].replace("unit=ohm", "unit=V")
        bad_units = finish(bad_units[:-1], 5)
        result = self.capture([(1.2, resistance(rows=1, seq=1, timestamp=1000000)),
                               (1.5, resistance(rows=1, seq=2, timestamp=2000000, overrides={0: "X02"})),
                               (1.7, bad_crc), (1.8, bad_id), (1.9, bad_units),
                               (2.3, resistance(rows=1, seq=6, timestamp=6000000))])
        group = result["configuration_groups"][0]
        intervals = group["host_complete_frame_intervals"]
        self.assertTrue(intervals["known"])
        self.assertEqual(intervals["sample_pairs"], 2)
        self.assertEqual(intervals["ordered_complete_frames"], 3)
        self.assertAlmostEqual(intervals["max_seconds"], .8)
        # Bad units are rejected by the existing parser before Group; CRC and
        # trailer-identity failures reach Group's existing rejection path.
        self.assertEqual(group["counts"]["rejected_untrusted_frames"], 2)
        self.assertEqual(group["counts"]["complete_frames"], 5)
        self.assertEqual(group["counts"]["frames_with_invalid_points"], 3)
        self.assertEqual(group["invalid_reasons"]["X02"], 1)
        self.assertAlmostEqual(group["rates"]["host_complete_receive_fps"], 2 / 1.1)

    def test_host_complete_intervals_unknown_time_and_duplicate_gates(self):
        def group_from(samples):
            group = None
            for seq, timestamp, host in samples:
                lines = resistance(rows=1, seq=seq, timestamp=timestamp)
                parser = TextProtocolParser()
                frame = None
                for line in lines:
                    parsed = parser.feed_line(line)
                    if parsed is not None: frame = parsed
                self.assertIsNotNone(frame)
                header = {"tag": "R", **parse_fields(lines[0])}
                if group is None: group = Group(header, frame)
                group.add(frame, header, parse_fields(lines[-1]), {}, host, 0, 0)
            return group.output()
        for hosts, unavailable, regressed, pairs in [
                ([1., None, 3.], 1, 0, 0), ([1., 2., None, 4., 5.], 1, 0, 2),
                ([2., 1., 3.], 0, 1, 1), ([1., float('nan'), 3.], 1, 0, 0),
                ([1., float('inf'), 3.], 1, 0, 0)]:
            with self.subTest(hosts=hosts):
                result = group_from([(i+1, (i+1)*1000000, host) for i, host in enumerate(hosts)])
                intervals = result["host_complete_frame_intervals"]
                self.assertFalse(intervals["known"]); self.assertIsNone(intervals["max_seconds"])
                self.assertEqual(intervals["sample_pairs"], pairs)
                self.assertEqual(intervals["unavailable_timestamps"], unavailable)
                self.assertEqual(intervals["timestamp_regressions"], regressed)
        # A duplicate/old frame cannot add a sample or poison the timing of
        # the next accepted frame, even if its supplied host time is missing.
        result = group_from([(1, 1000000, 1.), (1, 1000000, 99.), (0, 0, None), (2, 2000000, 2.)])
        intervals = result["host_complete_frame_intervals"]
        self.assertTrue(intervals["known"]); self.assertEqual(intervals["sample_pairs"], 1)
        self.assertEqual(intervals["max_seconds"], 1.); self.assertEqual(result["counts"]["duplicate_or_old_frames"], 2)
        result = group_from([(1, 1000000, 1.), (2, 2000000, 1.)])
        self.assertTrue(result["host_complete_frame_intervals"]["known"])
        self.assertEqual(result["host_complete_frame_intervals"]["max_seconds"], 0.)

    def test_host_complete_intervals_never_cross_identity_boot_or_device_time_order(self):
        result = self.capture([(1.2, resistance(rows=1, seq=10, timestamp=1000000)),
                               (1.5, resistance(rows=1, seq=11, timestamp=2000000, rev=2)),
                               (2.2, resistance(rows=1, seq=1, timestamp=1000000, session="BBBBBBBBBBBBBBBB"))])
        self.assertEqual(len(result["configuration_groups"]), 3)
        for group in result["configuration_groups"]:
            intervals = group["host_complete_frame_intervals"]
            self.assertFalse(intervals["known"]); self.assertIsNone(intervals["max_seconds"])
            self.assertEqual(intervals["sample_pairs"], 0)
        wrapped = self.capture([(1.2, resistance(rows=1, seq=0xFFFFFFFF, timestamp=1000000)),
                                (1.4, resistance(rows=1, seq=0, timestamp=1)),
                                (2.2, resistance(rows=1, seq=1, timestamp=2000000))], label="res1-wrap")
        group = wrapped["configuration_groups"][0]
        self.assertEqual(group["counts"]["timestamp_regressions"], 1)
        self.assertEqual(group["host_complete_frame_intervals"]["sample_pairs"], 1)
        self.assertAlmostEqual(group["host_complete_frame_intervals"]["max_seconds"], 1.)

    def test_same_boot_revision_changes_are_separate_but_counter_delta_allowed(self):
        result = self.capture([(0.8, [synthetic_perf(pub=20, usbSent=19)]),
                               (1.2, resistance(rows=1, seq=200, timestamp=1000000, rev=1)),
                               (2.2, resistance(rows=1, seq=201, timestamp=2000000, rev=2)),
                               (3.2, [synthetic_perf(pub=30, usbSent=27)])])
        self.assertEqual(len(result["configuration_groups"]), 2)
        self.assertEqual(result["perf"]["counters"]["pub"]["delta"], 10)
        self.assertIsNone(result["perf"]["counters"]["emitQueued"]["delta"])
        self.assertTrue(all(g["rates"]["capture_fps"] is None for g in result["configuration_groups"]))

    def test_cross_boot_never_subtracts_counters_or_sequence(self):
        result = self.capture([(0.8, [synthetic_perf(pub=20)]),
                               (1.2, resistance(rows=1, seq=200)),
                               (2.2, resistance(rows=1, seq=1, session="BBBBBBBBBBBBBBBB")),
                               (3.2, [synthetic_perf(pub=30)])])
        self.assertFalse(result["counter_boot_continuity_observed"])
        self.assertIsNone(result["perf"]["counters"]["pub"]["delta"])
        self.assertEqual(len(result["configuration_groups"]), 2)
        self.assertFalse(any(g["counts"].get("missing_frames") for g in result["configuration_groups"]))

    def test_sequence_wrap_is_defined_without_inventing_counter_wrap(self):
        result = self.capture([(0.8, ["USBSTREAM,sinkQueued=4294967294"]),
                               (1.2, resistance(rows=1, seq=0xFFFFFFFE, timestamp=1000000)),
                               (2.2, resistance(rows=1, seq=1, timestamp=2000000)),
                               (3.2, ["USBSTREAM,sinkQueued=2"])])
        group = result["configuration_groups"][0]
        self.assertEqual(group["counts"]["missing_frames"], 2)
        self.assertEqual(group["counts"]["sequence_wraps"], 1)
        self.assertEqual(group["rates"]["capture_fps"], 3.0)
        self.assertEqual(result["usb"]["counters"]["sinkQueued"]["unavailable"], "COUNTER_DECREASE_OR_WRAP_UNRESOLVED")

    def test_usb_apply_ack_does_not_hide_actual_post_query(self):
        result = self.capture([(0.8, ["USBSTREAM,v=FULL,sinkQueued=10,lineDropped=2,driverAcceptedBytes=100"]),
                               (1.2, resistance(rows=1)),
                               (3.1, ["USBSTREAM,v=DEBUG,state=applied"]),
                               (3.2, ["USBSTREAM,v=DEBUG,sinkQueued=20,lineDropped=3,driverAcceptedBytes=900"])])
        self.assertEqual(result["usb"]["counters"]["driverAcceptedBytes"]["delta"], 800)
        self.assertAlmostEqual(result["usb"]["elapsed_seconds"], 2.4)
        self.assertEqual(result["window"]["observed_seconds"], 2.0)

    def test_recorded_mixed1_CAP_row_uses_M_wire_and_revalidates_harness_false_positive(self):
        result = self.capture([(1.2, RECORDED_MIXED1)], label="mixed1-01", mode="MIXED",
                     extra={**mixed_config(recorded=True), "status": "ERROR",
                            "error": "capture_frame_mode_or_units_mismatch", "wire_contract_mismatches": 1,
                            "capture_monotonic_start": 101.0, "capture_monotonic_end": 103.0})
        self.assertEqual(result["capture_counts"]["trusted_complete_frames"], 1)
        self.assertEqual(result["capture_counts"]["ascii_crc_errors"], 0)
        self.assertEqual(result["collector_status"], "ERROR")
        self.assertEqual(result["collector_contract_revalidation"]["classification"], "HARNESS_FALSE_POSITIVE_REVALIDATED")
        self.assertEqual(result["expected_wire_contract"]["active_wire_profile"], "CNNNNNNN")
        report = analyse_directory(self.root)
        self.assertEqual(len(report["configuration_table"]), 32)
        row = next(x for x in report["configuration_table"] if x["mode"] == "MIXED" and x["rows"] == 1)
        self.assertEqual(row["status"], "RECORDED_WITH_ERRORS")
        self.assertEqual(row["mixed1_semantics"], "one_active_row_full_eight_row_profile_controls_wire_format")
        self.assertEqual(sum(x["status"] == "NOT_RUN" for x in report["configuration_table"]), 31)
        first, last = self.fragments(RECORDED_MIXED1, mid=1)
        self.ble_journal([(101.1, first), (101.2, last)])
        with_ble = analyse_directory(self.root)["captures"][0]
        self.assertEqual(with_ble["ble"]["status"], "RECORDED")
        self.assertEqual(with_ble["ble"]["capture_frame_counts"]["trusted_complete_frames"], 1)
        self.assertEqual(with_ble["ble"]["USB_comparison"]["same_wire_bytes_sha256"], 1)

    def test_full_eight_mode_profile_and_each_active_MR_contract_for_rows_1_through_8(self):
        for rows in range(1, 9):
            with self.subTest(rows=rows):
                result = self.capture([(1.2, synthetic_benchmark_mixed(rows))], label=f"mixed{rows}-01", mode="MIXED", rows=rows,
                                      extra=mixed_config(rows=rows))
                self.assertEqual(result["status"], "RECORDED")
                self.assertEqual(result["expected_wire_contract"]["wire_tag"], "M")
                self.assertEqual(result["expected_wire_contract"]["active_wire_profile"], "CVRCVRCV"[:rows] + "N" * (8 - rows))
                self.assertEqual(result["capture_counts"]["wire_contract_errors"], 0)
                self.assertEqual(result["capture_counts"]["observed_points"], rows * 8)

    def test_mixed1_active_CAP_does_not_authorize_C_wire_for_heterogeneous_full_profile(self):
        result = self.capture([(1.2, capacitance(rows=1))], label="mixed1-01", mode="MIXED", extra=mixed_config())
        self.assertEqual(result["capture_counts"]["wire_contract_errors"], 1)
        self.assertEqual(result["capture_counts"]["trusted_complete_frames"], 0)
        self.assertFalse(result["requested_configuration_observed"])

    def test_mixed1_requires_matching_full_CONFIG_profile_and_MR_units(self):
        for defect in ["CONFIG_tail", "row_units", "active_profile"]:
            with self.subTest(defect=defect):
                lines = synthetic_benchmark_mixed(1)
                extra = mixed_config()
                if defect == "CONFIG_tail":
                    extra["snapshots"]["before"]["CONFIG?"]["fields"]["profile"] = "CCCCCCCC"
                elif defect == "row_units":
                    lines[1] = lines[1].replace("unit=pF", "unit=V")
                    lines = finish(lines[:-1], 1, mixed=True)
                else:
                    lines[0] = lines[0].replace("profile=CNNNNNNN", "profile=VNNNNNNN")
                    lines[1] = lines[1].replace("m=CAP,unit=pF", "m=VOLT,unit=V")
                    lines = finish(lines[:-1], 1, mixed=True)
                result = self.capture([(1.2, lines)], label=f"mixed1-{defect}", mode="MIXED", extra=extra)
                self.assertEqual(result["capture_counts"]["ascii_crc_errors"], 0)
                self.assertEqual(result["capture_counts"]["wire_contract_errors"], 1)
                self.assertEqual(result["capture_counts"]["trusted_complete_frames"], 0)

    def test_short_stability_never_claims_600_seconds(self):
        self.capture([(1.2, resistance(rows=1))], label="stability-res1-01", requested=600,
                     extra={"benchmark_kind": "stability"})
        report = analyse_directory(self.root)
        self.assertEqual(report["stability"]["status"], "INCOMPLETE_OR_ERROR")
        self.assertFalse(report["captures"][0]["window"]["requested_seconds_met"])

    def test_native_WDT_coalesced_into_scientific_header_is_fault_not_complete_frame_or_reset(self):
        # Synthetic byte-order reproduction of retry-01-volt1's native log
        # insertion into a truncated scientific header. No raw frame is repaired.
        corrupt_header = resistance(rows=1, seq=2, timestamp=2000000)[0].split(",bootId=", 1)[0]
        corrupt_header += ",boE (12202761) task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:"
        result = self.capture([(1.2, resistance(rows=1, seq=1, timestamp=1000000)),
                               (16.064, [corrupt_header, "E (12202761) task_wdt:  - IDLE0 (CPU 0)",
                                         "E (12202761) task_wdt: CPU 0: btController",
                                         "E (12202761) task_wdt: CPU 1: sensorarrayScan"])],
                              label="retry-01-res1", end=16.065, requested=120,
                              extra={"status": "ERROR", "error": "firmware_fault_or_reset_during_capture"})
        self.assertFalse(result["window"]["requested_seconds_met"])
        self.assertEqual(result["status"], "INCOMPLETE_OR_MISMATCH")
        self.assertEqual(result["capture_counts"]["complete_frames"], 1)
        self.assertEqual(result["capture_counts"]["trusted_complete_frames"], 1)
        self.assertEqual(result["whole_raw_issues"]["truncated_frames_capture"], 1)
        self.assertEqual(result["faults_and_resets"]["FAULT"]["counts_by_receipt_phase"], {"capture": 1})
        self.assertEqual(result["faults_and_resets"]["RESET"]["counts_by_receipt_phase"], {})
        self.assertEqual(result["faults_and_resets"]["FAULT"]["examples"][0]["line"], corrupt_header)
        self.assertIsNone(compact_row("RES", 1, [result])["selected_label"])

    def test_unfinished_summary_does_not_read_active_raw(self):
        (self.root / "volt8-01.summary.json").write_text("{")
        (self.root / "volt8-01.rx.bin").mkdir()  # Opening this as a recording would fail.
        result = analyse_directory(self.root)
        self.assertEqual(result["captures"][0]["reason"], "SUMMARY_NOT_FINALIZED")

    def test_hash_and_rx_coverage_must_match_finalized_recording(self):
        self.capture([(1.2, resistance(rows=1))])
        raw = self.root / "res1-01.rx.bin"
        raw.write_bytes(raw.read_bytes() + b"extra\n")
        result = analyse_directory(self.root)
        self.assertEqual(result["captures"][0]["status"], "INPUT_ERROR")
        self.assertIn("SHA256", result["captures"][0]["reason"])

    def test_gzip_health_boundaries_and_unknown_cpu(self):
        lines = resistance(rows=1)
        for count in [18176, 18304, 18432, 18560]:
            lines += [f"ADSHEALTH,seq=1,engineFrameCount={count},modeEntryFrameCount=18048,healthInterval=64,err=0x0"]
        result = self.capture([(1.2, lines)], compressed=True)
        health = result["ads_health"]["mode_entry_groups"]["18048"]
        self.assertEqual(set(health["entry_relative_boundaries"]), {"128", "256", "384", "512"})
        self.assertEqual(health["absolute_boundaries"], {})
        self.assertEqual(result["memory"]["CPU_utilization"], "NOT_RUN")

    def test_recorded_truncated_STK50_heap_is_rejected_as_text_not_minimum(self):
        result = self.capture([(1.2, resistance(rows=1) + [RECORDED_STK50_TRUNCATED])])
        memory = result["memory"]
        self.assertEqual(memory["heap"], {})
        self.assertEqual(memory["stack_minimum_remaining_bytes"], {})
        self.assertEqual(memory["telemetry_integrity"]["rejected_records_or_fields_by_receipt_phase"]["STK50"], {"capture": 1})
        rejected = memory["telemetry_integrity"]["rejected_examples"][0]
        self.assertEqual(rejected["raw_ascii_with_line_ending"], RECORDED_STK50_TRUNCATED + "\n")

    def test_complete_STK50_with_small_legitimate_values_is_not_size_filtered(self):
        # Synthetic small-value counterexample; original recorded line stays above.
        small = RECORDED_STK50_COMPLETE.replace("heap=8377960/8341148/8336464", "heap=100/16/8").replace("log=16384/12792", "log=64/4")
        result = self.capture([(1.2, resistance(rows=1) + [small])])
        self.assertEqual(result["memory"]["heap"], {"observed_free_bytes": 16, "firmware_lifetime_minimum_bytes": 8})
        self.assertEqual(result["memory"]["stack_minimum_remaining_bytes"]["log"], 4)

    def test_PERF_requires_full_layout_and_unframed_terminal_heapMin_stays_unknown(self):
        truncated = synthetic_perf(pub=20, heap=16, heapMin=8).split(",heap=", 1)[0]
        result = self.capture([(0.8, [truncated]), (1.2, resistance(rows=1)),
                               (3.2, [synthetic_perf(pub=30, heap=16, heapMin=8)])])
        memory = result["memory"]
        self.assertEqual(memory["telemetry_integrity"]["rejected_records_or_fields_by_receipt_phase"]["PERF"], {"warmup": 1})
        self.assertTrue(all(entry["heap"] == "16" and entry["heapMin"] is None for entry in memory["PERF_query_heap"]))
        self.assertIsNone(result["perf"]["counters"]["pub"]["delta"])
        # A numeric-looking truncated final token cannot be proven complete by LF.
        partial_last = synthetic_perf(heap=16, heapMin=8336464).replace("heapMin=8336464", "heapMin=8336")
        result = self.capture([(0.8, [partial_last]), (1.2, resistance(rows=1))], label="res1-terminal")
        self.assertIsNone(result["memory"]["PERF_query_heap"][0]["heapMin"])

    def test_BL50_window_names_observed_receipts_not_query_snapshots(self):
        result = self.capture([(1.2, resistance(rows=1) + ["BL50,mq=20,sentD=19"]),
                               (2.2, resistance(rows=1, seq=2) + ["BL50,mq=30,sentD=29"])])
        self.assertEqual(result["ble_firmware"]["window"], "first_last_BL50_receipts_not_exact_capture_boundaries")
        self.assertEqual(result["ble_firmware"]["counters"]["sentD"]["delta"], 10)

    def test_mixed_real_parser_row_acquisition_masks(self):
        result = self.capture([(1.2, mixed()), (2.2, mixed(seq=2, timestamp=1000001))],
                              label="mixed8-01", mode="MIXED", rows=8, extra=mixed_config("RCCCCCCR", 8))
        self.assertEqual(result["capture_counts"]["observed_points"], 128)
        self.assertEqual(result["capture_counts"]["unacquired_points"], 0)
        self.assertEqual(result["capture_counts"]["open_points"], 28)

    def test_payload_excludes_interleaved_diagnostics(self):
        lines = resistance(rows=1)
        original_bytes = sum(len(x) + 1 for x in lines)
        lines.insert(1, "P50,diagnostic=1")
        result = self.capture([(1.2, lines)])
        self.assertEqual(result["configuration_groups"][0]["bytes"]["frame_wire_total"], original_bytes)

    def test_output_protection_and_explicit_own_report_replacement(self):
        report = analyse_directory(self.root)
        path = self.root / "report.json"
        write_report(path, report)
        with self.assertRaisesRegex(ValueError, "output exists"):
            write_report(path, report)
        write_report(path, report, replace=True)
        raw = self.root / "recording.rx.bin"
        raw.write_text("unaltered")
        with self.assertRaisesRegex(ValueError, "non-report"):
            write_report(raw, report, replace=True)
        self.assertEqual(raw.read_text(), "unaltered")

    def ble_journal(self, messages, *, ready=100.6, disconnect=103.4, compressed=False, extra=None):
        def event(kind, seconds, **fields):
            return {"schema_version": 1, "type": kind, "monotonic_ns": round(seconds * 1e9), **fields}
        records = [event("metadata", 100.0, collector="capture_benchmark_ble"),
                   event("connected", 100.5), event("subscriptions_ready", ready, all_channels_subscribed=["C", "D", "L"])]
        for seconds, payload in messages:
            records.append(event("notification", seconds, channel="D", length=len(payload),
                                 payload_base64=base64.b64encode(payload).decode("ascii")))
        records.extend(extra or [])
        if disconnect is not None:
            records += [event("disconnected", disconnect), event("recording_end", disconnect + 0.1)]
        records.sort(key=lambda item: item["monotonic_ns"])
        data = "".join(json.dumps(item) + "\n" for item in records).encode()
        path = self.root / ("suite.ble.notifications.jsonl.gz" if compressed else "suite.ble.notifications.jsonl")
        path.write_bytes(gzip.compress(data) if compressed else data)
        return path

    def fragments(self, lines, mid=1, bad_outer_crc=False):
        data = ("\n".join(lines) + "\n").encode("ascii")
        crc = (zlib.crc32(data) ^ int(bad_outer_crc)) & 0xFFFFFFFF
        pieces = [data[:len(data)//2], data[len(data)//2:]]
        return [f"G,D,{mid},{index},2,{len(piece)},{len(data)},{crc:08X}\n".encode() + piece
                for index, piece in enumerate(pieces)]

    def serial_with_absolute_window(self, first=None, second=None, **kwargs):
        first = first or resistance(rows=1, seq=1, timestamp=1000000)
        second = second or resistance(rows=1, seq=2, timestamp=2000000)
        self.capture([(1.2, first), (2.2, second)], extra={"capture_monotonic_start": 101.0,
                      "capture_monotonic_end": 103.0}, **kwargs)
        return first, second

    def test_ble_reassembly_crc_same_bytes_and_same_host_window(self):
        first, second = self.serial_with_absolute_window()
        a, b = self.fragments(first), self.fragments(second, mid=2)
        self.ble_journal([(101.1, a[0]), (101.2, a[1]), (102.1, b[0]), (102.2, b[1])])
        report = analyse_directory(self.root)
        ble = report["captures"][0]["ble"]
        self.assertEqual(ble["status"], "RECORDED")
        self.assertEqual(ble["capture_frame_counts"]["complete_frames"], 2)
        self.assertEqual(ble["fragments_by_channel"]["D"]["ok"], 2)
        self.assertEqual(ble["USB_comparison"]["same_wire_bytes_sha256"], 2)
        self.assertEqual(ble["comparative_host_fps"], 1.0)

    def test_ble_complete_interval_uses_completing_notification_not_local_or_first_fragment(self):
        first = capacitance(rows=8, seq=1, timestamp=1000000)
        second = capacitance(rows=8, seq=101, timestamp=2000000)
        self.capture([(1.2, first), (2.5, second)], label="cap8-snapshots", mode="CAP", rows=8,
                     extra={"capture_monotonic_start": 101., "capture_monotonic_end": 103.})
        a, b, duplicate = self.fragments(first), self.fragments(second, mid=2), self.fragments(first, mid=3)
        self.ble_journal([(101.1, a[0]), (101.2, a[1]), (102.1, b[0]), (102.5, b[1]),
                          (102.8, duplicate[0]), (102.9, duplicate[1])])
        ble = analyse_directory(self.root)["captures"][0]["ble"]
        intervals = ble["configuration_groups"][0]["host_complete_frame_intervals"]
        self.assertTrue(intervals["known"]); self.assertEqual(intervals["sample_pairs"], 1)
        self.assertEqual(intervals["ordered_complete_frames"], 2)
        self.assertAlmostEqual(intervals["max_seconds"], 1.3)
        self.assertEqual(ble["capture_frame_counts"]["duplicate_or_old_frames"], 1)
        self.assertEqual(ble["capture_frame_counts"]["missing_frames"], 99)
        # A gap between full frames is not reclassified as radio loss or
        # subtracted from sequence gaps by this timing observation.
        self.assertEqual(ble["USB_comparison"]["same_wire_bytes_sha256"], 2)

    def test_ble_outer_crc_and_inner_ascii_crc_remain_distinct(self):
        first, second = self.serial_with_absolute_window()
        first[-1] = re.sub(r"crc=[0-9A-F]+", "crc=00000000", first[-1])
        a, b = self.fragments(first), self.fragments(second, mid=2, bad_outer_crc=True)
        self.ble_journal([(101.1, a[0]), (101.2, a[1]), (102.1, b[0]), (102.2, b[1])])
        ble = analyse_directory(self.root)["captures"][0]["ble"]
        self.assertEqual(ble["capture_frame_counts"]["ascii_crc_errors"], 1)
        self.assertEqual(ble["fragments_by_channel"]["D"]["crc_fail"], 1)
        self.assertEqual(ble["USB_comparison"]["common_identity_sequence_frames"], 0)

    def test_ble_fragment_boundary_straddler_is_not_warmup_data(self):
        first, second = self.serial_with_absolute_window()
        a, b = self.fragments(first), self.fragments(second, mid=2)
        self.ble_journal([(100.9, a[0]), (101.2, a[1]), (102.1, b[0]), (102.2, b[1])])
        ble = analyse_directory(self.root)["captures"][0]["ble"]
        self.assertEqual(ble["counts"]["boundary_straddling_frames_excluded"], 1)
        self.assertEqual(ble["capture_frame_counts"]["complete_frames"], 1)

    def test_ble_valid_but_different_scientific_data_is_detected(self):
        first, _ = self.serial_with_absolute_window()
        different = resistance(rows=1, seq=2, timestamp=2000000, overrides={0: "10005000"})
        a, b = self.fragments(first), self.fragments(different, mid=2)
        self.ble_journal([(101.1, a[0]), (101.2, a[1]), (102.1, b[0]), (102.2, b[1])])
        comparison = analyse_directory(self.root)["captures"][0]["ble"]["USB_comparison"]
        self.assertEqual(comparison["common_identity_sequence_frames"], 2)
        self.assertEqual(comparison["different_wire_bytes_sha256"], 1)

    def test_ble_old_revision_is_not_relabelled_as_current_configuration(self):
        self.serial_with_absolute_window()
        old = resistance(rows=1, seq=1, timestamp=1000000, rev=7)
        a = self.fragments(old)
        self.ble_journal([(101.1, a[0]), (101.2, a[1])])
        ble = analyse_directory(self.root)["captures"][0]["ble"]
        self.assertEqual(ble["status"], "CONFIGURATION_MISMATCH")
        self.assertFalse(ble["observed_identity_subset_of_USB"])
        self.assertIsNone(ble["comparative_host_fps"])

    def test_ble_disconnect_is_not_fake_zero_fps(self):
        first, _ = self.serial_with_absolute_window()
        a = self.fragments(first)
        self.ble_journal([(101.1, a[0]), (101.2, a[1])], disconnect=102.0)
        ble = analyse_directory(self.root)["captures"][0]["ble"]
        self.assertEqual(ble["status"], "NOT_RUN")
        self.assertEqual(ble["reason"], "PARTIAL_OR_DISCONNECTED_WINDOW")
        self.assertIsNone(ble["comparative_host_fps"])
        self.assertEqual(ble["capture_frame_counts"]["complete_frames"], 1)

    def test_ble_not_connected_during_stability_and_gzip_replay(self):
        self.serial_with_absolute_window(label="stability-res1-01")
        self.ble_journal([], ready=104.0, disconnect=105.0, compressed=True)
        report = analyse_directory(self.root)
        self.assertEqual(report["captures"][0]["ble"]["reason"], "NO_SUBSCRIBED_CONNECTION_DURING_WINDOW")
        self.assertEqual(report["captures"][0]["ble"]["USB_comparison"]["status"], "NOT_RUN")
        self.assertEqual(report["ble_journal"]["source"]["compression"], "gzip")

    def test_ble_prefix_incomplete_fragment_is_bounded_evidence_not_id_gap(self):
        first, second = self.serial_with_absolute_window()
        a, b = self.fragments(first, mid=10), self.fragments(second, mid=1000)
        self.ble_journal([(101.1, a[0]), (102.1, b[0]), (102.2, b[1])])
        ble = analyse_directory(self.root)["captures"][0]["ble"]
        stats = ble["fragments_by_channel"]["D"]
        self.assertEqual(stats["gap"], 0)
        self.assertEqual(stats["incomplete_messages_disconnected"], 1)
        self.assertEqual(stats["unobserved_parts_disconnected"], 1)

    def test_ble_finalized_summary_sha_is_verified(self):
        first, second = self.serial_with_absolute_window()
        a, b = self.fragments(first), self.fragments(second, mid=2)
        path = self.ble_journal([(101.1, a[0]), (101.2, a[1]), (102.1, b[0]), (102.2, b[1])])
        summary = self.root / "suite.ble.summary.json"
        summary.write_text(json.dumps({"status": "RECORDED", "journal_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "journal_bytes": path.stat().st_size}))
        report = analyse_directory(self.root)
        self.assertEqual(len(report["captures"]), 1)  # BLE summary is not a serial capture.
        self.assertEqual(report["ble_journal"]["collector_summary_verification"]["status"], "MATCH")
        data = json.loads(summary.read_text())
        data["journal_sha256"] = "0" * 64
        summary.write_text(json.dumps(data))
        report = analyse_directory(self.root)
        self.assertEqual(report["ble_journal"]["collector_summary_verification"]["status"], "MISMATCH")
        self.assertEqual(report["captures"][0]["ble"]["status"], "NOT_RUN")

    def test_ble_live_prefix_can_ignore_unfinished_last_record(self):
        first, second = self.serial_with_absolute_window()
        a, b = self.fragments(first), self.fragments(second, mid=2)
        path = self.ble_journal([(101.1, a[0]), (101.2, a[1]), (102.1, b[0]), (102.2, b[1])], disconnect=None,
                                extra=[{"schema_version": 1, "type": "progress", "monotonic_ns": 103100000000}])
        with path.open("ab") as stream:
            stream.write(b'{"schema_version":')
        report = analyse_directory(self.root)
        self.assertTrue(report["ble_journal"]["source"]["incomplete_last_line_ignored"])
        self.assertEqual(report["captures"][0]["ble"]["status"], "RECORDED")
        self.assertFalse(report["ble_journal"]["recording_end_observed"])

    def test_archived_serial_events_and_raw_need_no_plain_copy(self):
        self.capture([(1.2, resistance(rows=1))], compressed=True)
        events = self.root / "res1-01.events.jsonl"
        archive = events.with_name(events.name + ".gz")
        archive.write_bytes(gzip.compress(events.read_bytes()))
        events.unlink()  # Synthetic temporary fixture only.
        report = analyse_directory(self.root)
        self.assertEqual(report["captures"][0]["capture_counts"]["complete_frames"], 1)
        self.assertTrue(report["captures"][0]["files"]["events"]["path"].endswith(".jsonl.gz"))

    def test_crc_valid_truncated_K_is_excluded_from_trusted_FPS_quality_and_matching(self):
        first = resistance(rows=1, seq=1, timestamp=1000000)
        original = resistance(rows=1, seq=2, timestamp=1500000, overrides={0: "X02"})
        truncated = original[:-1] + [original[-1].split(",session=", 1)[0] + ",ses"]
        third = resistance(rows=1, seq=3, timestamp=2000000)
        result = self.capture([(1.2, first), (1.7, truncated), (2.2, third)],
                              extra={"capture_monotonic_start": 101.0, "capture_monotonic_end": 103.0})
        counts = result["capture_counts"]
        self.assertEqual(counts["complete_frames"], 3)
        self.assertEqual(counts["trusted_complete_frames"], 2)
        self.assertEqual(counts["ascii_crc_errors"], 0)
        self.assertEqual(counts["trailer_identity_missing"], 1)
        self.assertEqual(counts["identity_value_mismatch"], 0)
        self.assertEqual(counts["observed_points"], 16)
        self.assertNotIn("X02", result["capture_invalid_reasons"])
        group = result["configuration_groups"][0]
        self.assertAlmostEqual(group["rates"]["host_complete_receive_fps"], 1.0)
        self.assertAlmostEqual(group["rates"]["host_parsed_frame_fps"], 2.0)
        self.assertEqual(group["rejected_frame_diagnostics"]["reported_invalid_reasons"]["X02"], 1)
        self.assertTrue(group["rejected_frame_diagnostics"]["examples"][0]["raw_K_ascii_with_newline"].endswith(",ses\n"))
        messages = []
        for index, (lines, when) in enumerate([(first, 101.2), (original, 101.7), (third, 102.2)], 1):
            a, b = self.fragments(lines, mid=index)
            messages += [(when - .01, a), (when, b)]
        self.ble_journal(messages)
        comparison = analyse_directory(self.root)["captures"][0]["ble"]["USB_comparison"]
        self.assertEqual(comparison["USB_unique_integrity_valid_frames"], 2)
        self.assertEqual(comparison["common_identity_sequence_frames"], 2)

    def test_trailer_header_mismatch_is_a_text_disagreement_not_proof_of_configuration_change(self):
        lines = resistance(rows=1)
        lines[-1] = lines[-1].replace("rev=1", "rev=2")
        result = self.capture([(1.2, lines)])
        counts = result["capture_counts"]
        self.assertEqual(counts["trailer_identity_missing"], 0)
        self.assertEqual(counts["trailer_header_identity_mismatch"], 1)
        self.assertEqual(counts["identity_value_mismatch"], 1)
        self.assertEqual(counts["trusted_complete_frames"], 0)

    def test_short_session_is_invalid_identity_not_a_valid_session_value_mismatch(self):
        lines = resistance(rows=1)
        lines[-1] = lines[-1].split(",session=", 1)[0] + ",session=AABBC"
        result = self.capture([(1.2, lines)])
        counts = result["capture_counts"]
        self.assertEqual(counts["trailer_identity_missing"], 1)
        self.assertEqual(counts["trailer_identity_invalid"], 1)
        self.assertEqual(counts["trailer_header_identity_mismatch"], 0)
        self.assertEqual(counts["identity_value_mismatch"], 0)
        rejected = result["configuration_groups"][0]["rejected_frame_diagnostics"]
        self.assertEqual(rejected["invalid_trailer_fields"], {"session": 1})
        self.assertEqual(rejected["examples"][0]["invalid_trailer_identity_fields"], {"session": "AABBC"})

    def test_expected_CONFIG_domain_mismatch_is_separate_and_not_accepted_quality(self):
        expected = {"session": "AABBCCDDEE112233", "bootId": "1", "rev": "1",
                    "mgen": "1", "mrid": "1", "rgen": "1", "rrid": "1", "pgen": "1", "prid": "1"}
        result = self.capture([(1.2, resistance(rows=1)), (2.2, resistance(rows=1, seq=2, rev=2))],
                              extra={"snapshots": {"before": {"CONFIG?": {"fields": expected}}}})
        self.assertEqual(result["capture_counts"]["header_config_identity_mismatch"], 1)
        self.assertEqual(result["capture_counts"]["trailer_header_identity_mismatch"], 0)
        self.assertEqual(result["capture_counts"]["observed_points"], 8)

    def test_selection_retains_complete_initial_errors_and_only_replaces_short_attempt(self):
        def run(label, rows, complete, status, fps):
            return {"label": label, "requested": {"kind": "configuration", "mode": "VOLT", "rows": rows},
                    "status": status, "collector_status": "ERROR" if status != "RECORDED" else "CAPTURED",
                    "window": {"requested_seconds_met": complete, "capture_monotonic_start": 10 if label.startswith("suite") else 20},
                    "configuration_groups": [{"identity": {}, "rates": {"capture_fps": fps, "host_complete_receive_fps": fps - 1}, "bytes": {}}]}
        captures = [run("suite-01-volt3", 3, True, "RECORDED_WITH_ERRORS", 30),
                    run("retry-01-volt3", 3, True, "RECORDED", 100),
                    run("suite-01-volt1", 1, False, "INCOMPLETE_OR_MISMATCH", 999),
                    run("retry-01-volt1", 1, True, "RECORDED", 90)]
        table = [compact_row(mode, rows, captures) for mode in ["CAP", "VOLT", "RES", "MIXED"] for rows in range(1, 9)]
        selection = {f"{row['mode']}{row['rows']}": row['selected_label'] for row in table}
        self.assertEqual(selection["VOLT3"], "suite-01-volt3")
        self.assertEqual(selection["VOLT1"], "retry-01-volt1")
        self.assertEqual(len(next(row for row in table if row['mode']=='VOLT' and row['rows']==1)['captures']), 2)
        data, selected, skipped = extract_rates({"configuration_table": table}, selection=selection)
        self.assertEqual(data["VOLT"]["capture"][2], 30)
        self.assertEqual(data["VOLT"]["capture"][0], 90)


if __name__ == "__main__":
    unittest.main()
