#!/usr/bin/env python3
"""Synthetic summary regressions; no physical measurement or device access."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

from reference_fixture import DEFAULT_FIXTURE, load_fixture
from summarize_reference_capture import summarize_capture
from test_reference_fixture import finish, resistance


def res_frame(seq, avdd=3300000, avss=-1800000, value=10000000, age=1, session="AABBCCDDEE112233"):
    lines = resistance(seq=seq, timestamp=seq * 100000, overrides={0: str(value)}, session=session)
    for key, replacement in (("avdd", avdd), ("avss", avss), ("age", age)):
        if re.search(rf",{key}=[^,]*", lines[0]):
            lines[0] = re.sub(rf",{key}=[^,]*", f",{key}={replacement}", lines[0])
        else:
            lines[0] += f",{key}={replacement}"
    return finish(lines[:-1], seq, session=session)


class CaptureSummaryTests(unittest.TestCase):
    def setUp(self):
        self.fixture = load_fixture(DEFAULT_FIXTURE)

    def prepare(self, directory, lines, compressed=False):
        data = ("\n".join(lines) + "\n").encode("ascii")
        raw = directory / ("synthetic.rx.bin.gz" if compressed else "synthetic.rx.bin")
        raw.write_bytes(gzip.compress(data) if compressed else data)
        (directory / "synthetic.summary.json").write_text(json.dumps({
            "status": "CAPTURED", "capture_seconds": 0.3, "capture_frame_trailers": 3,
            "raw_sha256": hashlib.sha256(data).hexdigest()}))
        events = [{"type": "capture_start", "t": 0},
                  {"type": "rx", "t": 0.1, "phase": "capture", "offset": 0, "length": len(data)},
                  {"type": "capture_end", "t": 0.3}]
        (directory / "synthetic.events.jsonl").write_text("\n".join(json.dumps(x) for x in events) + "\n")
        return raw

    def test_contiguous_rail_segments_usb_deltas_and_unavailable_raw_adc(self):
        lines = ["USBSTREAM,v=FULL,sinkQueued=100,sinkDropped=7,driverAcceptedBytes=1000,rxBytes=50,partialWrite=1,timeout=7,enumerated=unknown"]
        lines += res_frame(1, age=1)
        lines += res_frame(2, avdd=3250000, value=11000000, age=0)
        lines += ["ADSHEALTH,seq=2,engineFrameCount=64,healthInterval=64,err=0x0,crcValidEven=10,crcValidOdd=20,crcErrors=3,crcRetries=2"]
        lines += res_frame(3, value=12000000, age=0xFFFFFFFF)
        lines += ["ADSHEALTH,seq=3,engineFrameCount=128,healthInterval=64,err=0x108,crcValidEven=12,crcValidOdd=30,crcErrors=4,crcRetries=3",
                  "USBSTREAM,v=DEBUG,sinkQueued=110,sinkDropped=7,driverAcceptedBytes=3000,rxBytes=80,partialWrite=1,timeout=7,enumerated=unknown"]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.prepare(root, lines, compressed=True)
            result = summarize_capture(root, "synthetic", self.fixture)
            self.assertTrue(result["files"]["raw"]["summary_hash_matches"])
            self.assertEqual(result["capture_window"]["all_raw_complete_frames"], 3)
            segments = result["measurement_segments"]
            self.assertEqual(len(segments), 3)  # Returning rail is not merged.
            self.assertEqual([s["expected_reference_statistics"]["S1D1"]["statistics"]["mean"] for s in segments], [10000.0, 11000.0, 12000.0])
            self.assertEqual(segments[1]["rail_age_frames"]["first"], 0)
            self.assertIsNone(segments[2]["rail_age_frames"]["first"])
            self.assertEqual(segments[2]["S1D1_timeline"][0]["rail_age_encoding"], "4294967295")
            self.assertEqual(segments[0]["raw_adc"]["status"], "NOT_AVAILABLE")
            self.assertFalse(segments[0]["raw_adc"]["derived_raw_adc_created"])
            self.assertEqual(result["usb"]["counters"]["sinkDropped"]["delta"], 0)
            self.assertEqual(result["usb"]["counters"]["driverAcceptedBytes"]["delta"], 2000)
            self.assertEqual(result["usb"]["enumerated"], "unknown")
            self.assertEqual(result["ads_health"]["observed_engine_frame_counts"], [64, 128])
            self.assertEqual(len(result["ads_health"]["failed_records"]), 1)
            self.assertEqual(result["ads_spi"]["crc_valid_parity_and_totals_from_health"]["counters"]["crcValidOdd"]["delta"], 10)

    def test_cross_boot_never_gets_counter_delta_and_bad_crc_remains(self):
        lines = ["USBSTREAM,sinkQueued=10,sinkDropped=1"] + res_frame(1)
        bad = res_frame(2, session="BBBBBBBBBBBBBBBB")
        bad[-1] = re.sub(r"crc=[0-9A-F]+", "crc=00000000", bad[-1])
        lines += bad + ["USBSTREAM,sinkQueued=20,sinkDropped=1"]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.prepare(root, lines)
            result = summarize_capture(root, "synthetic", self.fixture)
            self.assertEqual(result["parser_counters"]["crc_errors"], 1)
            self.assertEqual(result["capture_window"]["all_raw_ascii_crc_valid_frames"], 1)
            self.assertEqual(len(result["configuration_groups"]), 2)
            self.assertIsNone(result["usb"]["counters"]["sinkDropped"]["delta"])
            self.assertEqual(result["usb"]["counters"]["sinkDropped"]["delta_unavailable_reason"], "boot_continuity_unverified")
            point = result["measurement_segments"][1]["expected_reference_statistics"]["S1D1"]
            self.assertEqual(point["statistics"]["rejected_crc"], 1)
            self.assertIsNone(result["measurement_segments"][1]["S1D1_timeline"][0]["decoded_value"])

    def test_hash_mismatch_and_output_overwrite_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw = self.prepare(root, res_frame(1))
            raw.write_bytes(raw.read_bytes() + b"added\n")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                summarize_capture(root, "synthetic", self.fixture)
            output = root / "existing.json"
            output.write_text("keep")
            result = subprocess.run([sys.executable, str(Path(__file__).with_name("summarize_reference_capture.py")),
                                     "--directory", str(root), "--capture", "synthetic", "--output", str(output)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(output.read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
