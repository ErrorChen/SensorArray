#!/usr/bin/env python3
"""Synthetic counter/schema cases only; not HIL or reconstructed wire data."""
import copy
import unittest
import summarize_diagnostic_screen as m


def contiguous(first=100, last=102):
    counts = {"trusted_complete_frames": 3, "ordered_integrity_valid_frames": 3}
    return counts, [{"counts": copy.deepcopy(counts), "sequence": {
        "first": first, "last": last, "progress_including_missing": (last-first) % 2**32,
        "gap_examples": []}}]


class CounterSemantics(unittest.TestCase):
    def test_only_one_actual_continuous_stability_window_can_be_recorded(self):
        s = {"benchmark_kind":"stability", "capture_requested_seconds":600.0,
             "capture_seconds":600.25, "capture_monotonic_start":1000.0,
             "capture_monotonic_end":1600.25, "capture_duration_satisfied":True,
             "status":"CAPTURED"}
        d = m.stability_observation(s)
        self.assertEqual(d["status"], "CAPTURED_600_SECONDS_NOT_ACCEPTED")
        self.assertEqual(d["acceptance"], "NOT_EVALUATED")
        for key, value in (("capture_requested_seconds",120), ("capture_seconds",599.9),
                           ("capture_seconds",float("nan")), ("capture_seconds",True),
                           ("capture_monotonic_start",None), ("capture_monotonic_end",1600.0),
                           ("capture_duration_satisfied",False), ("status","ERROR")):
            with self.subTest(key=key,value=value):
                changed = {**s,key:value}
                self.assertEqual(m.stability_observation(changed)["status"], "INCOMPLETE_OR_UNKNOWN")
        self.assertEqual(m.stability_observation({**s,"benchmark_kind":"configuration"})["status"],
                         "NOT_REQUESTED_IN_THIS_CAPTURE")
        self.assertEqual(m.stability_observation({})["status"], "NOT_REQUESTED_IN_THIS_CAPTURE")

    def test_missing_key_requires_positive_contiguity_evidence(self):
        c, g = contiguous()
        self.assertEqual(m.missing_observations(c,g)["count"],0)
        self.assertEqual(m.missing_observations(c,g)["first"],100)

    def test_missing_key_without_proof_is_unknown(self):
        for mutation in ("absent_seq", "gap", "duplicate", "different_group_count", "multiple", "progress"):
            with self.subTest(mutation=mutation):
                c,g=contiguous()
                if mutation=="absent_seq": g[0].pop("sequence")
                elif mutation=="gap": g[0]["sequence"]["gap_examples"]=[{"missing":1}]
                elif mutation=="duplicate": c["ordered_integrity_valid_frames"]=2
                elif mutation=="different_group_count": g[0]["counts"]["trusted_complete_frames"]=2
                elif mutation=="multiple": g.append(copy.deepcopy(g[0]))
                else: g[0]["sequence"]["progress_including_missing"]=3
                self.assertIsNone(m.missing_observations(c,g)["count"])

    def test_same_identity_u32_wrap(self):
        c,g=contiguous(2**32-2,0)
        self.assertEqual(m.missing_observations(c,g)["count"],0)

    def test_explicit_nonzero_is_preserved(self):
        c,g=contiguous();c["missing_frames"]=8
        self.assertEqual(m.missing_observations(c,g)["count"],8)
        c["missing_frames"]="unknown"
        self.assertIsNone(m.missing_observations(c,g)["count"])

    def test_unknown_layer_not_zero(self):
        value=m.layer_window({})
        self.assertEqual(value["status"],"UNKNOWN")
        self.assertIsNone(value["elapsed_seconds"])
        self.assertEqual(value["counters"],{})

    def test_role_comparison_uses_ratios_not_cross_boot_counters(self):
        a={"label":"synthetic_a", "configuration_identity":{"session":"A"},"capture_fps":60,
           "USB":{"FPS":59},"BLE":{"FPS":30},"layer_windows":{},"CPU_owner_roles":[
               {"role":0,"name":"BLE_TX","status":"PAIRED","runtime_delta_us":20,"observation_span_us":100,
                "runtime_per_own_observation_span_percent":20},
               {"role":4,"name":"BLE_CTRL","status":"UNKNOWN","runtime_delta_us":None}]}
        b=copy.deepcopy(a);b["configuration_identity"]["session"]="B";b["label"]="synthetic_b"
        b["CPU_owner_roles"][0].update(runtime_delta_us=50,observation_span_us=200,runtime_per_own_observation_span_percent=25)
        d=m.compare_runs(a,b)
        self.assertEqual(d["CPU_roles"][0]["ratio_difference_percentage_points"],5)
        self.assertIsNone(d["CPU_roles"][1]["ratio_difference_percentage_points"])
        self.assertIsNone(d["CPU_ratio_sum"])
        self.assertEqual(d["blocked_time"],"UNKNOWN")


if __name__ == "__main__":
    unittest.main()
