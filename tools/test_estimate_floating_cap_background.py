#!/usr/bin/env python3
"""Synthetic protocol regressions only; these are not physical HIL evidence."""
from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from estimate_floating_cap_background import (FLOATING, METHODS, OCCUPIED,
                                               donor_sets, estimate_capture)
from test_reference_fixture import capacitance, finish, mixed


def synthetic_cap(values=None, rows=8, seq=1, timestamp=1,
                  session="AABBCCDDEE112233", rev=1, acquired=None, primary_fresh=None):
    lines = capacitance(rows=rows, seq=seq, timestamp=timestamp, session=session, rev=rev)[:-1]
    actual = [100_000_000] * (rows * 8)
    if rows >= 4:
        actual[27] = 133_000_000
    if rows >= 5:
        actual[36] = 133_000_000
    for label, fixed in (values or {}).items():
        actual[(int(label[1]) - 1) * 8 + int(label[3]) - 1] = fixed
    lines[1:] = [f"D{chunk}," + ",".join(map(str, actual[chunk * 16:chunk * 16 + 16]))
                 for chunk in range((len(actual) + 15) // 16)]
    if acquired is not None:
        lines[0] = lines[0].replace(f"acquired={(1 << (rows * 8)) - 1:X}", f"acquired={acquired:X}")
    if primary_fresh is not None:
        lines[0] = lines[0].replace(f"pf={(1 << rows) - 1:X}", f"pf={primary_fresh:X}")
    return finish(lines, seq, session=session, rev=rev)


class FloatingCapBackgroundTests(unittest.TestCase):
    def target(self, result, label="S4D4", group=0):
        return result["groups"][group]["targets"][label]

    def test_distinct_parasitics_report_all_methods_and_keep_negative_delta(self):
        values = {label: 120_000_000 for label in donor_sets("S4D4")["same_row_median"]}
        values.update({label: 150_000_000 for label in donor_sets("S4D4")["same_column_median"]})
        values.update(S1D1=9_999_000_000, S8D8=9_999_000_000, S5D5=80_000_000)
        result = estimate_capture(synthetic_cap(values))
        point = self.target(result)
        self.assertEqual(point["methods"]["same_row_median"]["target_minus_background_pf"]["mean"], 13)
        self.assertEqual(point["methods"]["same_column_median"]["target_minus_background_pf"]["mean"], -17)
        self.assertEqual(point["methods"]["all_floating_median"]["target_minus_background_pf"]["mean"], 33)
        self.assertEqual(point["cross_method_sensitivity"]["per_frame_estimate_range_pf"]["mean"], 50)
        self.assertEqual(self.target(result, "S5D5")["methods"]["all_floating_median"]["target_minus_background_pf"]["mean"], -20)
        self.assertEqual(point["precision_acceptance"], "NOT_EVALUATED")
        self.assertNotIn('"PASS"', json.dumps(result))
        self.assertFalse(result["calibration_written"])

    def test_all_occupied_cells_excluded_from_every_population(self):
        self.assertEqual(len(FLOATING), 60)
        for label in ("S4D4", "S5D5"):
            populations = donor_sets(label)
            self.assertEqual([len(populations[name]) for name in METHODS], [7, 7, 60])
            self.assertTrue(all(not set(donors) & set(OCCUPIED) for donors in populations.values()))
        # Bad values at the resistor ports and the OTHER capacitor do not poison S4D4.
        result = estimate_capture(synthetic_cap({"S1D1": -1_000_000,
                                                  "S8D8": -1_000_000, "S5D5": -1_000_000}))
        self.assertTrue(all(method["valid_paired_count"] == 1 for method in self.target(result)["methods"].values()))
        self.assertEqual(self.target(result, "S5D5")["target_rejection_counts_nonexclusive"]["invalid"], 1)

    def test_pair_per_frame_and_keep_outliers_without_smoothing(self):
        result = estimate_capture(synthetic_cap(seq=1, timestamp=1) +
                                  synthetic_cap({"S4D4": 233_000_000}, seq=2, timestamp=2))
        method = self.target(result)["methods"]["all_floating_median"]
        self.assertEqual(method["valid_paired_count"], 2)
        self.assertEqual(method["target_minus_background_pf"]["min"], 33)
        self.assertEqual(method["target_minus_background_pf"]["max"], 133)
        self.assertEqual(method["target_minus_background_pf"]["mean"], 83)
        self.assertEqual(method["target_minus_background_pf"]["stddev"], 50)

    def test_incomplete_fixed_set_has_no_estimate_or_changing_population(self):
        result = estimate_capture(synthetic_cap(seq=1, timestamp=1) +
                                  synthetic_cap({"S4D2": -1_000_000}, seq=2, timestamp=2))
        target = self.target(result)
        row = target["methods"]["same_row_median"]
        self.assertEqual(row["valid_paired_count"], 1)
        self.assertEqual(row["excluded_frame_count"], 1)
        self.assertEqual(row["missing_donor_counts_nonexclusive"]["S4D2"]["invalid"], 1)
        self.assertEqual(row["available_donor_count_in_trusted_frames"]["min"], 6)
        self.assertEqual(target["methods"]["same_column_median"]["valid_paired_count"], 2)
        frame = result["groups"][0]["per_frame"][1]["targets"]["S4D4"]["methods"]["same_row_median"]
        self.assertIsNone(frame["background_pf"])
        self.assertIsNone(frame["target_minus_background_pf"])

    def test_target_and_donors_cannot_be_paired_from_different_frames(self):
        result = estimate_capture(synthetic_cap({"S4D2": -1_000_000}, seq=1, timestamp=1) +
                                  synthetic_cap({"S4D4": -1_000_000}, seq=2, timestamp=2))
        target = self.target(result)
        self.assertEqual(target["target_usable_frames"], 1)
        for name in ("same_row_median", "all_floating_median"):
            self.assertEqual(target["methods"][name]["valid_paired_count"], 0)
            self.assertIsNone(target["methods"][name]["target_minus_background_pf"]["mean"])

    def test_bad_crc_stale_unacquired_and_invalid_targets_are_counted(self):
        corrupt = synthetic_cap(seq=1, timestamp=1)
        crc_token = next(token for token in corrupt[-1].split(",") if token.startswith("crc="))
        corrupt[-1] = corrupt[-1].replace(crc_token, f"crc={int(crc_token[4:], 16) ^ 1:08X}")
        stale = synthetic_cap(seq=2, timestamp=2, primary_fresh=0xF7)
        unacquired = synthetic_cap(seq=3, timestamp=3, acquired=((1 << 64) - 1) ^ (1 << 27))
        invalid = synthetic_cap({"S4D4": -1_000_000}, seq=4, timestamp=4)
        result = estimate_capture(corrupt + stale + unacquired + invalid + synthetic_cap(seq=5, timestamp=5))
        target = self.target(result)
        self.assertEqual(result["capture_quality"]["shared_parser_counters"]["crc_errors"], 1)
        for reason in ("bad_crc", "stale", "unacquired", "invalid"):
            self.assertEqual(target["target_rejection_counts_nonexclusive"][reason], 1)
        self.assertTrue(all(method["valid_paired_count"] == 1 for method in target["methods"].values()))
        self.assertEqual(result["groups"][0]["complete_frames"], 5)

    def test_sessions_revisions_geometry_and_no_cross_identity_pairing(self):
        result = estimate_capture(synthetic_cap(seq=9, timestamp=900) +
                                  synthetic_cap({"S4D4": 233_000_000}, seq=1, timestamp=1, session="1111111111111111") +
                                  synthetic_cap(seq=1, timestamp=1, rev=2) +
                                  synthetic_cap(rows=1, seq=2, timestamp=2, rev=2))
        self.assertEqual(len(result["groups"]), 4)
        self.assertEqual([group["complete_frames"] for group in result["groups"]], [1] * 4)
        self.assertTrue(all(group["duplicate_or_old_frames"] == 0 for group in result["groups"]))
        self.assertEqual(self.target(result, group=1)["methods"]["all_floating_median"]["target_minus_background_pf"]["mean"], 133)
        self.assertEqual(self.target(result, group=3)["target_rejection_counts_nonexclusive"]["not_observed"], 1)
        self.assertEqual(self.target(result, group=3)["methods"]["all_floating_median"]["valid_paired_count"], 0)

    def test_trailer_mismatch_legacy_identity_and_old_frames_are_excluded(self):
        wrong = synthetic_cap(seq=1, timestamp=1)
        wrong[-1] = wrong[-1].replace("rev=1", "rev=2")
        legacy = synthetic_cap(seq=2, timestamp=2)[:-1]
        legacy[0] = legacy[0].split(",session=", 1)[0]
        legacy = finish(legacy, 2)
        good = synthetic_cap(seq=3, timestamp=3)
        result = estimate_capture(wrong + legacy + good + good)
        self.assertEqual(result["capture_quality"]["identity_errors"], 1)
        self.assertEqual(len(result["groups"]), 2)
        self.assertEqual(self.target(result)["methods"]["all_floating_median"]["valid_paired_count"], 1)
        self.assertEqual(result["groups"][0]["duplicate_or_old_frames"], 1)
        self.assertEqual(self.target(result, group=1)["target_rejection_counts_nonexclusive"]["unverified_identity"], 1)
        self.assertEqual(self.target(result, group=1)["target_usable_frames"], 0)

    def test_sequence_wrap_within_identity_is_defined(self):
        result = estimate_capture(synthetic_cap(seq=0xFFFFFFFF, timestamp=1) + synthetic_cap(seq=0, timestamp=2))
        self.assertEqual(result["groups"][0]["sequence_gaps"], 0)
        self.assertEqual(self.target(result)["methods"]["all_floating_median"]["valid_paired_count"], 2)

    def test_mixed_only_cap_rows_are_donors_and_full_populations_stay_fixed(self):
        result = estimate_capture(mixed())
        target = self.target(result)
        self.assertEqual(target["methods"]["same_row_median"]["valid_paired_count"], 1)
        column = target["methods"]["same_column_median"]
        self.assertEqual(column["available_donor_count_in_trusted_frames"]["mean"], 5)
        self.assertEqual(column["valid_paired_count"], 0)
        global_method = target["methods"]["all_floating_median"]
        self.assertEqual(global_method["required_donor_count"], 60)
        self.assertEqual(global_method["available_donor_count_in_trusted_frames"]["mean"], 46)
        self.assertEqual(global_method["valid_paired_count"], 0)
        self.assertEqual(target["cross_method_sensitivity"]["all_methods_paired_count"], 0)

    def test_missing_frame_and_runtime_fault_are_visible(self):
        result = estimate_capture(synthetic_cap()[:-1] + ["MFAULT,seq=1,err=0x108"])
        self.assertEqual(result["status"], "NO_VALID_PAIRED_ESTIMATE")
        self.assertEqual(result["capture_quality"]["truncated_frames"], 1)
        self.assertEqual(result["capture_quality"]["runtime_fault_count"], 1)

    def test_gzip_cli_preserves_source_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            payload = ("\n".join(synthetic_cap()) + "\n").encode("ascii")
            source = directory / "synthetic.rx.bin.gz"
            source.write_bytes(gzip.compress(payload, mtime=0))
            output = directory / "synthetic.estimate.json"
            command = [sys.executable, str(Path(__file__).with_name("estimate_floating_cap_background.py")),
                       "--input", str(source), "--output", str(output)]
            completed = subprocess.run(command, capture_output=True, text=True, timeout=20)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            report = json.loads(output.read_text())
            self.assertEqual(report["source"]["raw_sha256"], hashlib.sha256(payload).hexdigest())
            self.assertTrue(report["source"]["gzip"])
            old_output = output.read_bytes()
            second = subprocess.run(command, capture_output=True, text=True, timeout=20)
            self.assertEqual(second.returncode, 2)
            self.assertIn("already exists", second.stderr)
            self.assertEqual(output.read_bytes(), old_output)
            self.assertEqual(gzip.decompress(source.read_bytes()), payload)


if __name__ == "__main__":
    unittest.main()
