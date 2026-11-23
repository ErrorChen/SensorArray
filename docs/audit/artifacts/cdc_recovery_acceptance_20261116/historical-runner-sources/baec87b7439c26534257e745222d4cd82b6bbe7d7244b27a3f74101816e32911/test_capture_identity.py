"""Pure synthetic identity regressions. No serial/BLE/USB access."""
import unittest
from unittest.mock import patch

from sensorarray_capture_identity import (CaptureIdentityError, CaptureImageAuthority,
    ExpectedCaptureIdentity, expected_from_options, verify_identity_evidence)


ELF = "ab" * 32
SESSION = "AABBCCDDEE112233"


def build():
    return dict(idf="synthetic", target="esp32s3", project="SensorArray", proto="2",
                version="synthetic-not-live", elfSha256=ELF)


def config():
    return dict(session=SESSION, bootId="1", rev="7", boundary="22", state="CAPACITANCE",
                mode="CAP", rows="8", profile="CCCCCCCC", err="0x0",
                mgen="4", mrid="5", rgen="2", rrid="3", pgen="4", prid="5",
                mtermId="5", mterm="applied", merr="0x0", ptermId="5", pterm="applied", perr="0x0",
                rtermId="3", rterm="applied", rerr="0x0")


class IdentityTests(unittest.TestCase):
    def setUp(self):
        self.expected = ExpectedCaptureIdentity(ELF, SESSION, 1)

    def test_observed_reset_or_changed_CONFIG_revokes_real_collector_send_fence(self):
        from run_firmware_benchmark import BenchmarkCollector, Collector, CaptureError
        for kind, fields in [('reset_observed', {'line': 'RST,bootId=2'}),
                             ('reply', {'command': 'CONFIG?', 'line': 'CONFIG,session=' + SESSION + ',bootId=2'})]:
            with self.subTest(kind=kind):
                collector = object.__new__(BenchmarkCollector)
                collector.image_authority = CaptureImageAuthority(self.expected)
                collector.image_authority.verify(build(), config(), config())
                with patch.object(Collector, 'event'), patch.object(Collector, 'send') as writer:
                    collector.event(kind, **fields)
                    with self.assertRaisesRegex(CaptureError, 'mutation_blocked'):
                        collector.send('MODE=CAP', 123.0)
                    writer.assert_not_called()
                self.assertFalse(collector.image_authority.verified)

    def test_wifi_owner_queries_are_explicitly_readonly_without_authority(self):
        authority = CaptureImageAuthority(self.expected)
        for command in ('WIFIQ?D', 'WIFIQ?L', 'WIFIQ?C', 'WIFIREQ?0', 'WIFIREQ?4294967295', 'CAPS2?'):
            authority.require_command(command)
        for command in ('WIFI=AP', 'WIFIQ?X', 'WIFIREQ?-1', 'WIFIREQ?4294967296',
                        'WIFIREQ?0\nWIFI=AP', 'WIFIREQ?1=AP'):
            with self.subTest(command=command), self.assertRaises(CaptureIdentityError):
                authority.require_command(command)

    def test_full_expected_identity_or_explicitly_unbound_legacy(self):
        self.assertIsNone(expected_from_options())
        for values in ((ELF, None, 1), (None, SESSION, 1), (ELF, SESSION, None),
                       ("abcd", SESSION, 1), (ELF, "1", 1), (ELF, SESSION, -1),
                       (ELF, SESSION, 2**32), (ELF, SESSION, True)):
            with self.subTest(values=values), self.assertRaises(CaptureIdentityError):
                expected_from_options(*values)
        self.assertEqual(expected_from_options(ELF.upper(), SESSION.lower(), "1"), self.expected)

    def test_good_bracket_copies_the_configuration_without_unit_or_domain_rewrite(self):
        opening, closing = config(), config()
        evidence = verify_identity_evidence(self.expected, build(), opening, closing)
        self.assertTrue(evidence["verified"])
        opening["mgen"] = closing["rgen"] = "999"
        self.assertEqual(evidence["CONFIG_opening"]["mgen"], "4")
        self.assertEqual(evidence["CONFIG_closing"]["rgen"], "2")
        self.assertEqual(evidence["physical_safety"], "NOT_ESTABLISHED_BY_IDENTITY")

    def test_wrong_image_missing_identity_and_wrong_target_are_rejected(self):
        for key, value in (("elfSha256", "cd"*32), ("proto", "1"), ("target", "pty"),
                           ("project", "Other"), ("version", "")):
            with self.subTest(key=key), self.assertRaises(CaptureIdentityError):
                changed = build(); changed[key] = value
                verify_identity_evidence(self.expected, changed, config(), config())

    def test_boot_session_and_each_configuration_domain_must_agree(self):
        for key in config():
            changed = config()
            if key == "session": changed[key] = "FFEEDDCCBBAA1122"
            elif key in ("state", "mode", "profile", "mterm", "pterm", "rterm"): changed[key] += "X"
            elif key.endswith("err"): changed[key] = "0x1"
            else: changed[key] = str(int(changed[key]) + 1)
            with self.subTest(key=key), self.assertRaises(CaptureIdentityError):
                verify_identity_evidence(self.expected, build(), config(), changed)

    def test_unknown_or_incomplete_configuration_does_not_authorize(self):
        for key in config():
            changed = config(); del changed[key]
            with self.subTest(key=key), self.assertRaises(CaptureIdentityError):
                verify_identity_evidence(self.expected, build(), changed, config())

    def test_cleanup_can_identify_same_faulted_image_without_claiming_safe(self):
        faulted = config(); faulted.update(state="DEGRADED", mode="NONE", err="0x108")
        with self.assertRaises(CaptureIdentityError):
            verify_identity_evidence(self.expected, build(), faulted, faulted)
        evidence = verify_identity_evidence(self.expected, build(), faulted, faulted, require_running=False)
        self.assertEqual(evidence["CONFIG_closing"]["err"], "0x108")
        self.assertEqual(evidence["physical_safety"], "NOT_ESTABLISHED_BY_IDENTITY")

    def test_mutation_fence_includes_non_equals_commands_and_failed_reverification(self):
        guard = CaptureImageAuthority(self.expected)
        for command in ("BUILD?", "CONFIG?", "CPUOWNER?0", "CPUOWNER?6", "CPUOWNER?7"):
            guard.require_command(command)
        for command in ("MODE=CAP", "ROWS=8", "TX=REL", "RAILCFG=3350000,-1795000", "USBBOOT", "RESTART", "BUILD?\nUSBBOOT"):
            with self.subTest(command=command), self.assertRaises(CaptureIdentityError):
                guard.require_command(command)
        guard.verify(build(), config(), config())
        guard.require_command("MODE=CAP")
        changed = build(); changed["elfSha256"] = "cd"*32
        with self.assertRaises(CaptureIdentityError): guard.verify(changed, config(), config())
        self.assertFalse(guard.verified)
        with self.assertRaises(CaptureIdentityError): guard.require_command("MODE=CAP")


if __name__ == "__main__": unittest.main()
