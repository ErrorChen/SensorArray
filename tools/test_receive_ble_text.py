#!/usr/bin/env python3
"""Host-only tests for the BLE text receiver lifecycle parser feed.

No bleak, serial port, or BLE adapter is required.
"""

from __future__ import annotations

import argparse
import unittest

from receive_ble_text import BleTextReceiver


class BleTextReceiverLifecycleTest(unittest.TestCase):
    def makeReceiver(self):
        args = argparse.Namespace(tail=False, show_fragments=False)
        receiver = BleTextReceiver(args, None)
        receiver._write = lambda line: None
        return receiver

    def test_ctrl_ack_and_log_terminal_reach_shared_parser(self):
        receiver = self.makeReceiver()
        receiver.process_message(
            "C", b"RMACK,id=21,old=CCCCCCCC,new=CVVRRVVC,state=accepted\n")
        receiver.process_message(
            "L", b"RMAPP,id=21,gen=4,seq=31,profile=CVVRRVVC,state=applied\n")
        self.assertEqual(receiver.parser.counters.rmack, 1)
        self.assertEqual(receiver.parser.counters.rmapp, 1)
        self.assertEqual(receiver.parser.rowmode_lifecycle_errors(),
                         {"unterminated": 0, "duplicate": 0,
                          "terminal_without_ack": 0})

    def test_duplicate_terminal_is_lifecycle_error(self):
        receiver = self.makeReceiver()
        receiver.process_message(
            "C", b"RMACK,id=21,old=CCCCCCCC,new=CVVRRVVC,state=accepted\n")
        for _ in range(2):
            receiver.process_message(
                "L", b"RMAPP,id=21,gen=4,seq=31,profile=CVVRRVVC,state=applied\n")
        self.assertEqual(receiver.parser.rowmode_lifecycle_errors()["duplicate"],
                         1)

    def test_terminal_without_ack_is_lifecycle_error(self):
        receiver = self.makeReceiver()
        receiver.process_message(
            "L", b"RMAPP,id=9,gen=4,seq=31,profile=CVVRRVVC,state=applied\n")
        self.assertEqual(
            receiver.parser.rowmode_lifecycle_errors()["terminal_without_ack"],
            1)


if __name__ == "__main__":
    unittest.main()
