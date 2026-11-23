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


if __name__ == "__main__":
    unittest.main()
