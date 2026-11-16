#!/usr/bin/env python3
"""Explicitly synthetic records; do not represent them as hardware captures."""
import unittest
from analyze_adc_read_provenance import decode_record, crc_division


LEGACY = b'ADSSPIBYTES,seq=1,bytes=68C07B73C2FD,len=6,received=FD,computed=FC,crcMode=2,statusEnabled=1\n'
SPLIT = (b'ADSSPIBYTES,seq=1,source=spi55_gpio1,wireKnown=0,bytes=unknown,'
         b'assembledBytes=68C07B73C2FD,spiPrefix=0068C07B73C2FC,gpioKnown=1,gpioBit=1,'
         b'tailStage=complete,len=6,received=FD,computed=FC,crcMode=2,statusEnabled=1\n')


class ProvenanceTests(unittest.TestCase):
    def test_legacy_is_explicit_assumption_and_real_crc_difference(self):
        r = decode_record(LEGACY)
        self.assertTrue(r['legacy_source_assumption'])
        self.assertTrue(r['wire_observed'])
        self.assertEqual(r['xor'], '01')

    def test_new_standard_requires_wire_known(self):
        line = LEGACY[:-1] + b',source=spi56,wireKnown=1\n'
        self.assertFalse(decode_record(line)['legacy_source_assumption'])
        with self.assertRaises(ValueError):
            decode_record(line.replace(b'wireKnown=1', b'wireKnown=0'))

    def test_split_observation_is_never_a_continuous_wire_capture(self):
        r = decode_record(SPLIT)
        self.assertFalse(r['wire_observed'])
        self.assertEqual(r['gpio_bit'], 1)
        self.assertEqual(r['xor'], '01')
        self.assertNotIn('wire_bytes', r)

    def test_both_actual_tail_values_preserved(self):
        line = SPLIT.replace(b'gpioBit=1', b'gpioBit=0').replace(b'73C2FD', b'73C2FC').replace(b'received=FD', b'received=FC')
        r = decode_record(line)
        self.assertEqual(r['gpio_bit'], 0)
        self.assertEqual(r['xor'], '00')

    def test_inconsistent_or_unknown_observations_rejected(self):
        for old, new in [(b'wireKnown=0', b'wireKnown=1'), (b'bytes=unknown', b'bytes=68C07B73C2FD'),
                         (b'gpioKnown=1', b'gpioKnown=0'), (b'gpioBit=1', b'gpioBit=2'),
                         (b'spiPrefix=0068C07B73C2FC', b'spiPrefix=0068C07B73C2FD'),
                         (b'assembledBytes=68C07B73C2FD', b'assembledBytes=68C07B73C2FC'),
                         (b'tailStage=complete', b'tailStage=gpio_low'),
                         (b'computed=FC', b'computed=FD')]:
            with self.subTest(old=old), self.assertRaises(ValueError):
                decode_record(SPLIT.replace(old, new))

    def test_no_truncated_record_completion_or_payload_guess(self):
        for line in (SPLIT[:-1], LEGACY.replace(b'bytes=68C07B73C2FD', b'bytes=68C07B73C2'),
                     LEGACY.replace(b'statusEnabled=1', b'statusEnabled=0'),
                     LEGACY.replace(b'crcMode=2', b'crcMode=1'),
                     SPLIT.replace(b'spi55_gpio1', b'unknown')):
            with self.subTest(line=line), self.assertRaises(ValueError):
                decode_record(line)

    def test_independent_polynomial_examples(self):
        self.assertEqual(crc_division(bytes.fromhex('C07B73C2')), 0xFC)
        self.assertEqual(crc_division(bytes.fromhex('0005146B')), 0xD5)


if __name__ == '__main__':
    unittest.main()
