#!/usr/bin/env python3
"""Synthetic reference-fixture regressions, not physical calibration evidence."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zlib

from reference_fixture import (DEFAULT_FIXTURE, analyse_capture, describe_fixture,
                               evaluate_fixture, load_fixture, read_capture)
from test_text_protocol import measurement_lines


def finish(lines, seq, mixed=False, session="AABBCCDDEE112233", rev=1):
    crc = zlib.crc32("".join(x + "\n" for x in lines).encode("ascii")) & 0xFFFFFFFF
    legacy = "rgen=1,rrid=1,pgen=1,prid=1" if mixed else "gen=1,rid=1"
    return lines + [f"K,seq={seq},{legacy},crc={crc:08X},session={session},bootId=1,rev={rev}"]


def identity(session="AABBCCDDEE112233", rev=1):
    return f",session={session},bootId=1,rev={rev},mgen=1,mrid=1,rgen=1,rrid=1,pgen=1,prid=1"


def resistance(rows=8, seq=1, timestamp=1, overrides=None, session="AABBCCDDEE112233", rev=1):
    values = ["X0D"] * (8 * rows)
    values[0] = "10000000"
    if rows == 8:
        values[63] = "10000000"
    for index, value in (overrides or {}).items():
        values[index] = value
    lines = measurement_lines("R", seq, rows, values, [1] * len(values))[:-1]
    lines[0] = lines[0].replace("ts=18446744073709551615", f"ts={timestamp}")
    lines[0] = lines[0].replace("gen=4294967295,rid=4294967295", "gen=1,rid=1") + identity(session, rev)
    return finish(lines, seq, session=session, rev=rev)


def capacitance(rows=8, seq=1, timestamp=1, loaded=True, session="AABBCCDDEE112233", rev=1):
    values = [100_000_000] * (rows * 8)
    if loaded and rows >= 5:
        values[27] += 33_000_000
        values[36] += 33_000_000
    mask = (1 << (rows * 8)) - 1
    fresh = (1 << rows) - 1
    lines = [f"C,seq={seq},ts={timestamp},rows={rows},cells={rows * 8},gen=1,rid=1,"
             f"rf={fresh:X},pf={fresh:X},sf={fresh:X},expected={mask:X},acquired={mask:X},fmt=pf6" + identity(session, rev)]
    for chunk in range((len(values) + 15) // 16):
        lines.append(f"D{chunk}," + ",".join(str(x) for x in values[chunk * 16:chunk * 16 + 16]))
    return finish(lines, seq, session=session, rev=rev)


def mixed(seq=1, timestamp=1):
    lines = [f"M,seq={seq},ts={timestamp},rows=8,cells=64,rgen=1,rrid=1,pgen=1,prid=1,"
             "profile=RCCCCCCR,expected=FFFFFFFFFFFFFFFF,acquired=FFFFFFFFFFFFFFFF,fmt=mix1" + identity()]
    for row in range(1, 9):
        if row in (1, 8):
            values = ["X0D"] * 8
            index = 0 if row == 1 else 7
            values[index] = "10000000"
            valid, error = 1 << index, 0xFF ^ (1 << index)
            prefix = f"m=RES,unit=ohm,scale=-3,valid={valid:02X},error={error:02X},fmt=mohm-x"
        else:
            values = ["100000000"] * 8
            if row == 4:
                values[3] = "133000000"
            if row == 5:
                values[4] = "133000000"
            prefix = "m=CAP,unit=pF,scale=-6,valid=FF,error=00,fmt=pf6"
        lines.append(f"MR,s={row},{prefix},fresh=FF,expected=FF,acquired=FF,D=" + ",".join(values))
    return finish(lines, seq, mixed=True)


class ReferenceFixtureTests(unittest.TestCase):
    def setUp(self):
        self.fixture = load_fixture(DEFAULT_FIXTURE)

    def evaluate(self, lines, fixture=None, baseline=None):
        return evaluate_fixture(fixture or self.fixture, analyse_capture(lines),
                                analyse_capture(baseline) if baseline else None, "synthetic")

    def test_fixture_and_no_capture_never_pass(self):
        self.assertEqual(describe_fixture(self.fixture)["floating_count"], 60)
        self.assertEqual(describe_fixture(self.fixture)["one_row_coverage"], ["S1D1"])
        self.assertEqual(evaluate_fixture(self.fixture)["status"], "NOT_RUN")
        self.assertTrue(all(r["acceptance_abs"] is None for r in self.fixture["references"].values()))

    def test_one_row_only_observes_first_resistor(self):
        result = self.evaluate(resistance(rows=1))
        refs = result["groups"][0]["references"]
        self.assertEqual(refs["S1D1"]["accuracy"], "NEEDS_LIMITS")
        self.assertEqual(refs["S1D1"]["statistics"]["mean"], 10000.0)
        self.assertTrue(all(refs[label]["accuracy"] == "NOT_OBSERVED" for label in ("S8D8", "S4D4", "S5D5")))
        self.assertEqual(result["groups"][0]["screening"], "SCREENING_INCOMPLETE")

    def test_all_sixty_open_cells_require_actual_open(self):
        result = self.evaluate(resistance())
        self.assertEqual(result["groups"][0]["floating_res_pass"], 60)
        for wrong in ("X01", "X0E", "1000"):
            result = self.evaluate(resistance(overrides={1: wrong}))
            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["groups"][0]["floating_res"]["S1D2"]["status"], "FAIL")

    def test_bad_crc_and_stale_do_not_disappear(self):
        lines = resistance()
        lines[-1] = lines[-1].replace("crc=", "crc=0")  # Often same numeric CRC: force change instead.
        lines[-1] = lines[-1].split(",crc=", 1)[0] + ",crc=00000000,session=AABBCCDDEE112233,bootId=1,rev=1"
        result = self.evaluate(lines + resistance(seq=2))
        point = result["groups"][0]["references"]["S1D1"]
        self.assertEqual(point["statistics"]["rejected_crc"], 1)
        self.assertEqual(point["statistics"]["samples"], 1)
        self.assertEqual(point["accuracy"], "DATA_QUALITY_FAILED")
        lines = resistance()
        lines[0] = lines[0].replace("fresh=FFFFFFFFFFFFFFFF", "fresh=FFFFFFFFFFFFFFFD")
        lines = finish(lines[:-1], 1)
        result = self.evaluate(lines)
        self.assertEqual(result["groups"][0]["floating_res"]["S1D2"]["status"], "DATA_QUALITY_FAILED")

    def test_explicit_limit_tests_every_sample(self):
        fixture = deepcopy(self.fixture)
        fixture["references"]["S1D1"]["acceptance_abs"] = 1.0
        passed = self.evaluate(resistance(), fixture)
        self.assertEqual(passed["groups"][0]["references"]["S1D1"]["accuracy"], "PASS")
        failed = self.evaluate(resistance() + resistance(seq=2, overrides={0: "10002000"}), fixture)
        self.assertEqual(failed["groups"][0]["references"]["S1D1"]["accuracy"], "FAIL")
        self.assertEqual(failed["physical_calibration"], "NOT_RUN")

    def test_cap_total_is_not_33pf_and_baseline_is_same_cell(self):
        loaded = capacitance()
        result = self.evaluate(loaded)
        self.assertEqual(result["groups"][0]["references"]["S4D4"]["accuracy"], "BASELINE_REQUIRED")
        self.assertEqual(result["groups"][0]["references"]["S4D4"]["statistics"]["mean"], 133.0)
        baseline = capacitance(loaded=False, session="1111111111111111")
        result = self.evaluate(loaded, baseline=baseline)
        cap = result["groups"][0]["references"]["S4D4"]
        self.assertEqual(cap["baseline_subtracted"]["mean"], 33.0)
        self.assertEqual(cap["accuracy"], "NEEDS_LIMITS")
        fixture = deepcopy(self.fixture)
        fixture["references"]["S4D4"]["acceptance_abs"] = 0.5
        result = self.evaluate(loaded, fixture, baseline)
        self.assertEqual(result["groups"][0]["references"]["S4D4"]["accuracy"], "PASS")
        self.assertEqual(result["groups"][0]["diagnostic_only"]["S1D2/CAP"]["mean"], 100.0)

    def test_mixed_preserves_row_mode_units_and_open_tokens(self):
        result = self.evaluate(mixed(), baseline=capacitance(loaded=False))
        group = result["groups"][0]
        self.assertEqual(group["profile"], "RCCCCCCR")
        self.assertEqual(group["references"]["S8D8"]["statistics"]["mean"], 10000.0)
        self.assertEqual(group["references"]["S5D5"]["baseline_subtracted"]["mean"], 33.0)
        self.assertEqual(group["floating_res_pass"], 14)
        self.assertEqual(group["floating_res"]["S4D1"]["status"], "NOT_OBSERVED")

    def test_groups_do_not_merge_boot_revision_or_geometry(self):
        result = self.evaluate(resistance(seq=10) + resistance(seq=1, session="1111111111111111") +
                               resistance(seq=11, rev=2) + resistance(rows=1, seq=12))
        self.assertEqual(len(result["groups"]), 4)
        self.assertTrue(all(group["frames"] == 1 for group in result["groups"]))
        self.assertTrue(all(group["duplicate_or_old_frames"] == 0 for group in result["groups"]))

    def test_trailer_identity_not_covered_by_crc_is_checked(self):
        lines = resistance()
        lines[-1] = lines[-1].replace("rev=1", "rev=2")
        result = self.evaluate(lines)
        self.assertEqual(result["capture_errors"]["identity_errors"], 1)
        self.assertEqual(result["status"], "FAIL")

    def test_ambiguous_baseline_and_truncated_capture_do_not_pass(self):
        result = self.evaluate(capacitance(), baseline=capacitance(loaded=False) + capacitance(loaded=False, rev=2))
        self.assertEqual(result["groups"][0]["references"]["S4D4"]["accuracy"], "BASELINE_AMBIGUOUS")
        result = self.evaluate(resistance() + resistance(seq=2)[:-1])
        self.assertEqual(result["capture_errors"]["truncated_frames"], 1)
        self.assertEqual(result["status"], "FAIL")

    def test_time_window_wrap_and_legacy_identity(self):
        result = self.evaluate(resistance(seq=0xFFFFFFFF, timestamp=1) + resistance(seq=0, timestamp=120000001))
        self.assertEqual(result["groups"][0]["screening"], "WINDOW_SPAN_MET")
        self.assertEqual(result["groups"][0]["sequence_gaps"], 0)
        lines = resistance()
        lines[0] = lines[0].split(",session=", 1)[0]
        lines = finish(lines[:-1], 1)
        result = self.evaluate(lines)
        self.assertEqual(result["groups"][0]["references"]["S1D1"]["accuracy"], "UNVERIFIED_IDENTITY")

    def test_real_encoder_golden_is_not_fixture_measurement(self):
        root = Path(__file__).resolve().parents[1]
        fixture = deepcopy(self.fixture)
        fixture["references"]["S1D1"]["acceptance_abs"] = 1.0
        result = evaluate_fixture(fixture, read_capture(root / "tests/fixtures/config_identity/res_1.frame"), provenance="synthetic")
        self.assertEqual(result["groups"][0]["references"]["S1D1"]["accuracy"], "FAIL")
        self.assertEqual(result["status"], "FAIL")

    def test_legacy_memory_tag_is_not_a_mixed_measurement(self):
        memory = "M,stage=before_wifi_init,reason=low_heap,ih=44720,il=31732,im=43840,dh=36944,dl=31732,h8=8430864,l8=8257524"
        result = self.evaluate([memory, memory])
        self.assertEqual(result["status"], "NOT_RUN")
        self.assertEqual(result["memory_diagnostic_lines"], 2)
        self.assertEqual(result["capture_errors"]["truncated_frames"], 0)
        self.assertEqual(result["parser_counters"]["malformed"], 0)
        # Interleaved diagnostics do not alter the actual frame CRC bytes.
        lines = resistance()
        result = self.evaluate(lines[:1] + [memory] + lines[1:])
        self.assertEqual(result["parser_counters"]["crc_errors"], 0)
        # A corrupt scientific M remains visible, even with heap-like fields.
        result = self.evaluate([memory + ",seq=2,fmt=mix1"])
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["memory_diagnostic_lines"], 0)

    def test_firmware_fault_does_not_hide_behind_good_samples(self):
        result = self.evaluate(resistance() + ["MFAULT,mode=RES,err=0x108,state=DEGRADED"])
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["capture_errors"]["firmware_fault_events"], 1)
        # A telemetry value named wdt is not an actual watchdog event.
        result = self.evaluate(resistance() + ["STK50,wdt=unknown"])
        self.assertEqual(result["capture_errors"]["firmware_fault_events"], 0)

    def test_cli_refuses_overwrite_and_invalid_limits(self):
        tool = Path(__file__).with_name("reference_fixture.py")
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "report.json"
            command = [sys.executable, str(tool), "--output", str(output)]
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 0)
            content = output.read_bytes()
            self.assertEqual(subprocess.run(command, capture_output=True).returncode, 2)
            self.assertEqual(output.read_bytes(), content)
            self.assertEqual(json.loads(content)["status"], "NOT_RUN")
            fixture = deepcopy(self.fixture)
            fixture["references"]["S1D1"]["acceptance_abs"] = -1
            bad = Path(tmp) / "bad.json"
            bad.write_text(json.dumps(fixture))
            with self.assertRaises(ValueError):
                load_fixture(bad)


if __name__ == "__main__":
    unittest.main()
