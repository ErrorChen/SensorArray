#!/usr/bin/env python3
"""Offline regressions; synthetic timing/corruption is never physical evidence."""
import copy
import json
from pathlib import Path
import re
import tempfile
import unittest

import analyze_usb_unread as analyze
import test_analyze_firmware_benchmark as benchmark_tests
from test_reference_fixture import resistance
from text_protocol import parse_fields


class UnreadAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.fixture = benchmark_tests.BenchmarkAnalysisTests()
        self.fixture.root = Path(self.temp.name)
        frame = resistance(rows=1)
        self.config = {**parse_fields(frame[0]), "mode": "RES", "profile": "RRRRRRRR"}

    def tearDown(self):
        self.temp.cleanup()

    def replay(self, messages):
        journal = self.fixture.ble_journal(messages)
        before = journal.read_bytes()
        ble, evidence = analyze.analyse_ble_window(journal, 101.0, 103.0, self.config, "synthetic-unread")
        self.assertEqual(before, journal.read_bytes())
        self.assertTrue(evidence["schema_valid"])
        self.assertIsNone(ble["USB_comparison"]["USB_PC_FPS"])
        self.assertEqual(ble["USB_comparison"]["status"], "NOT_OBSERVED")
        self.assertFalse(ble["USB_comparison"]["post_hold_buffer_drain_used"])
        return ble

    def test_original_parser_excludes_both_straddlers_and_later_drain(self):
        frames = [self.fixture.fragments(resistance(rows=1, seq=i, timestamp=i*1000000), mid=i)
                  for i in range(1, 6)]
        times = [(100.9, 101.1), (101.2, 101.3), (102.2, 102.3), (102.9, 103.1), (103.2, 103.3)]
        ble = self.replay([(t, piece) for pair, pieces in zip(times, frames) for t, piece in zip(pair, pieces)])
        self.assertEqual(ble["capture_frame_counts"]["ordered_integrity_valid_frames"], 2)
        self.assertEqual(ble["counts"]["boundary_straddling_frames_excluded"], 2)
        group = ble["configuration_groups"][0]
        self.assertEqual((group["sequence"]["first"], group["sequence"]["last"]), (2, 3))
        self.assertAlmostEqual(group["rates"]["capture_fps"], 1.0)
        self.assertAlmostEqual(group["rates"]["host_complete_receive_fps"], 1.0)
        self.assertAlmostEqual(group["host_complete_frame_intervals"]["max_seconds"], 1.0)

    def test_bad_inner_crc_wrong_identity_and_invalid_samples_stay_separate(self):
        good = resistance(rows=1, seq=1, timestamp=1000000, overrides={0: "X12"})
        crc = resistance(rows=1, seq=2, timestamp=2000000)
        crc[-1] = re.sub(r"crc=[0-9A-F]+", "crc=00000000", crc[-1])
        wrong = resistance(rows=1, seq=3, timestamp=3000000, session="1122334455667788")
        last = resistance(rows=1, seq=4, timestamp=4000000)
        messages = []
        for i, lines in enumerate([good, crc, wrong, last]):
            pieces = self.fixture.fragments(lines, mid=i+1)
            messages.extend([(101.1+i*.4, pieces[0]), (101.2+i*.4, pieces[1])])
        ble = self.replay(messages)
        counts = ble["capture_frame_counts"]
        self.assertEqual(counts["ordered_integrity_valid_frames"], 2)
        self.assertEqual(counts["ascii_crc_errors"], 1)
        self.assertEqual(counts["identity_errors"], 1)
        self.assertEqual(counts["rejected_untrusted_frames"], 2)
        self.assertEqual(ble["capture_invalid_reasons"]["X12"], 1)

    def test_bad_fragment_crc_does_not_reach_science(self):
        bad = self.fixture.fragments(resistance(rows=1), bad_outer_crc=True)
        ble = self.replay([(101.1, bad[0]), (101.2, bad[1])])
        self.assertEqual(ble["status"], "NO_COMPLETE_FRAMES")
        self.assertEqual(ble["capture_frame_counts"], {})
        self.assertGreater(ble["fragments_by_channel"]["D"]["crc_fail"], 0)

    def test_query_unknown_or_decreasing_counter_is_not_zero_or_wrap_invention(self):
        self.assertIsNone(analyze.counter_delta("unknown", "10")["delta"])
        self.assertIsNone(analyze.counter_delta("20", "10")["delta"])
        self.assertEqual(analyze.counter_delta("20", "20")["delta"], 0)
        self.assertEqual(analyze.counter_delta("20", "31")["delta"], 11)

    def test_closed_recording_validates_then_rejects_fabricated_hold_evidence(self):
        path = Path(__file__).parents[1] / "usb-unread-01/usb-unread-01-hold.summary.json"
        summary = analyze.load(path)
        raw = analyze.read_bytes(analyze.resolve(summary["raw_file"], path))
        events = [json.loads(line) for line in analyze.read_bytes(analyze.resolve(summary["events_file"], path)).splitlines()]
        original = (path.read_bytes(), raw)
        config, build = analyze.validate_hold(summary, events, raw)
        self.assertEqual(config["mode"], "VOLT")
        self.assertEqual(build["project"], "SensorArray")
        changes = [
            lambda s: s["hold"].update(read_calls=1),
            lambda s: s["hold"].update(raw_offset_after=s["hold"]["raw_offset_after"]+1),
            lambda s: s["hold"].update(actual_seconds=1),
            lambda s: s["snapshots"]["after"]["queries"]["CONFIG?"]["fields"].update(bootId="2"),
            lambda s: s["snapshots"]["after"]["queries"]["USBQ?"].update(line="USBQ,fabricated=1"),
        ]
        for mutate in changes:
            changed = copy.deepcopy(summary)
            mutate(changed)
            with self.assertRaises(ValueError):
                analyze.validate_hold(changed, events, raw)
        changed_events = copy.deepcopy(events)
        i = next(i for i, item in enumerate(changed_events) if item["type"] == "unread_hold_start")
        changed_events.insert(i+1, {"type": "rx", "phase": "unread_hold", "t": changed_events[i]["t"]+.1,
                                    "offset": summary["hold"]["raw_offset_before"], "length": 1})
        with self.assertRaisesRegex(ValueError, "hold_event_IO_observed"):
            analyze.validate_hold(summary, changed_events, raw)
        self.assertEqual(original[0], path.read_bytes())
        self.assertEqual(original[1], raw)

    def recorded_hold(self):
        path = Path(__file__).parents[1] / "usb-unread-01/usb-unread-01-hold.summary.json"
        summary = analyze.load(path)
        raw = analyze.read_bytes(analyze.resolve(summary["raw_file"], path))
        events = [json.loads(line) for line in analyze.read_bytes(analyze.resolve(summary["events_file"], path)).splitlines()]
        return summary, events, raw

    def test_final_finished_missing_error_or_summary_fault_totals_rejected(self):
        summary, events, raw = self.recorded_hold()
        for mutation in ("missing", "failed", "post_finished"):
            changed = copy.deepcopy(events)
            if mutation == "missing":
                changed.pop()
            elif mutation == "failed":
                changed[-1].update(status="ERROR", error="synthetic_late_failure")
            else:
                changed.append({"t": changed[-1]["t"]+.1, "type": "unexpected_late_event"})
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError, "hold_final_finished"):
                analyze.validate_hold(summary, changed, raw)
        for key, value in (("cleanup_error", "timeout"), ("after_error", "timeout"),
                           ("error", "late_fault"), ("fault_events", 1), ("reset_event_count", 17),
                           ("frame_boot_change_events", 1), ("safe_cap_recovery", "FAILED_OR_UNKNOWN")):
            changed = copy.deepcopy(summary)
            changed[key] = value
            with self.subTest(field=key), self.assertRaisesRegex(ValueError, "hold_fault_or_cleanup"):
                analyze.validate_hold(changed, events, raw)

    def test_actual_write_event_names_inside_hold_are_rejected(self):
        summary, events, raw = self.recorded_hold()
        index = next(i for i, event in enumerate(events) if event["type"] == "unread_hold_start")
        for kind in ("tx_begin", "tx_complete"):
            changed = copy.deepcopy(events)
            changed.insert(index+1, {"t": changed[index]["t"]+.1, "type": kind,
                                    "phase": "unread_hold", "command": "CONFIG?"})
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, "hold_event_IO"):
                analyze.validate_hold(summary, changed, raw)

    def test_cleanup_CAP8_must_match_real_new_reply(self):
        summary, events, raw = self.recorded_hold()
        for mutation in (None, {**summary["cleanup_CONFIG"], "rows": "1"},
                         {**summary["cleanup_CONFIG"], "rev": "999"}):
            changed = copy.deepcopy(summary)
            changed["cleanup_CONFIG"] = mutation
            with self.subTest(cleanup=mutation), self.assertRaises(ValueError):
                analyze.validate_hold(changed, events, raw)

    def test_readonly_post_probe_status_hash_raw_fields_and_after_hold_time(self):
        hold, _, _ = self.recorded_hold()
        base = Path(__file__).parents[1]
        post = analyze.load(base / "ble-snapshot-postunread-01.summary.json")
        raw = analyze.read_bytes(base / "ble-snapshot-postunread-01.rx.bin")
        events = [json.loads(line) for line in analyze.read_bytes(base / "ble-snapshot-postunread-01.events.jsonl").splitlines()]
        config = hold["snapshots"]["before"]["queries"]["CONFIG?"]["fields"]
        build = hold["snapshots"]["before"]["queries"]["BUILD?"]["fields"]
        def validate(value):
            return analyze.validate_post_probe(value, events, raw, hold["hold"]["finished_monotonic"], config, build)
        fields, closure = validate(post)
        self.assertEqual(fields["CONFIG?"]["rows"], "8")
        self.assertFalse(closure["explicit_final_finished_event"])
        mutations = [lambda p: p.update(status="ERROR"), lambda p: p.update(raw_sha256="0"*64),
                     lambda p: p.update(rx_bytes=0), lambda p: p.update(faults=["MFAULT,synthetic=1"]),
                     lambda p: p.update(resets=["RST,synthetic=17"]),
                     lambda p: p["attempts"][0].update(monotonic=1.0),
                     lambda p: next(a for a in p["attempts"] if a["command"] == "MEM?")["fields"].update(intFree="424242")]
        for mutate in mutations:
            changed = copy.deepcopy(post)
            mutate(changed)
            with self.subTest(mutation=str(mutate)), self.assertRaises(ValueError):
                validate(changed)
        with self.assertRaises(ValueError):
            analyze.validate_post_probe(post, events[:-1], raw, hold["hold"]["finished_monotonic"], config, build)

    def test_stale_raw_reply_before_write_cannot_close_post_query(self):
        raw = b"CONFIG,mode=CAP,rows=8\n"
        events = [{"t": 0.0, "type": "rx", "offset": 0, "length": len(raw)},
                  {"t": 0.1, "type": "tx_begin", "command": "CONFIG?", "wire_ascii": "\nCONFIG?\n"},
                  {"t": 0.2, "type": "tx_complete", "command": "CONFIG?", "accepted_bytes": 9},
                  {"t": 0.3, "type": "reply", "command": "CONFIG?", "line": raw.decode().strip()}]
        with self.assertRaisesRegex(ValueError, "not_new_complete_raw"):
            analyze.verify_query_trace(events, raw)
        for raw_fault in (b"MFAULT,synthetic=1\n", b"RST,synthetic=17\n", b"Guru Meditation Error: test\n"):
            with self.assertRaisesRegex(ValueError, "fault_or_reset"):
                analyze.reject_fault_reset_text(raw_fault)


if __name__ == "__main__":
    unittest.main()
