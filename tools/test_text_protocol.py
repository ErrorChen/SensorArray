import unittest
import zlib

from text_protocol import TextProtocolParser


class TextProtocolParserTest(unittest.TestCase):
    def test_reassembles_dynamic_values_and_validates_crc(self) -> None:
        lines = ["C,seq=7,ts=123,rows=5,cells=40,rf=1F,pf=1F,sf=1F,"
                 "bad=0/0/0,fmt=pf6,n=40"]
        values = list(range(40))
        for chunk in range(3):
            start = chunk * 16
            lines.append("D%d,%s" % (chunk, ",".join(str(value)
                                                     for value in values[start:start + 16])))
        crc_payload = "".join(line + "\n" for line in lines).encode("ascii")
        lines.append(f"K,seq=7,crc={zlib.crc32(crc_payload) & 0xFFFFFFFF:08X}")

        protocol = TextProtocolParser()
        frame = None
        for line in lines:
            frame = protocol.feed_line(line) or frame

        self.assertIsNotNone(frame)
        self.assertEqual(frame.values_fixed, values)
        self.assertEqual(frame.rows, 5)
        self.assertEqual(frame.cells, 40)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(protocol.counters.cap_frames, 1)

    def test_parses_battery_invalid_reason(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("A50,chip=1262,adc=1,bt=-1,br=rail")
        self.assertEqual(protocol.latest_battery_mv, -1)
        self.assertEqual(protocol.latest_fields["A50"]["br"], "rail")

    def test_parses_current_ab50_battery_invalid_reason(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("AB50,chip=1262,bt=-1,br=rail,rs=hold")
        self.assertEqual(protocol.latest_battery_mv, -1)
        self.assertEqual(protocol.latest_fields["AB50"]["br"], "rail")
        self.assertEqual(protocol.latest_fields["AB50"]["rs"], "hold")


if __name__ == "__main__":
    unittest.main()
