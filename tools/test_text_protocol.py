import unittest
import zlib

from text_protocol import TextProtocolParser


def measurement_lines(tag: str, sequence: int, rows: int,
                      values: list[str], gains: list[int]) -> list[str]:
    mode, unit, scale, fmt = (("VOLT", "V", -6, "uv") if tag == "V" else
                              ("RES", "ohm", -3, "mohm"))
    cells = rows * 8
    valid_mask = sum(1 << index for index, value in enumerate(values)
                     if not value.startswith("X"))
    error_mask = ((1 << cells) - 1) ^ valid_mask
    lines = [
        f"{tag},seq={sequence},ts=18446744073709551615,rows={rows},cells={cells},"
        f"gen=4294967295,rid=4294967295,mode={mode},unit={unit},scale={scale},"
        f"valid={valid_mask:016X},fresh={(1 << cells) - 1:016X},"
        f"error={error_mask:016X},ref=AVDD_AVSS,rail=1,age=4294967295,"
        f"avdd=2147483647,avss=-2147483648,vexc=2147483647,rref=4294967295,"
        f"dur=18446744073709551615,"
        f"tr=18446744073709551615,gc=4294967295,ov=4294967295,aa=4294967295,"
        f"fb=4294967295,ir=4294967295,to=4294967295,st=4294967295,spi=4294967295,"
        f"fmt={fmt}-x,n={cells},"
        f"bad={cells - bin(valid_mask).count('1')}"
    ]
    for chunk in range((cells + 15) // 16):
        start = chunk * 16
        lines.append(f"D{chunk}," + ",".join(values[start:start + 16]))
    for chunk in range((cells + 15) // 16):
        start = chunk * 16
        packed = "".join(f"{gain:02X}" for gain in gains[start:start + 16])
        lines.append(f"P{chunk},{packed}")
    crc_payload = "".join(line + "\n" for line in lines).encode("ascii")
    lines.append(f"K,seq={sequence},gen=4294967295,rid=4294967295,crc="
                 f"{zlib.crc32(crc_payload) & 0xFFFFFFFF:08X}")
    return lines


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
        protocol.feed_line(
            "AB50,bt=-1,valid=0,br=rail_invalid,bs=stale,ageMs=1001,"
            "periodMs=1000,due=1,run=2,skip=3,defer=4,boundary=1,"
            "restoreFail=0,sampleUs=800/900")
        self.assertEqual(protocol.latest_battery_mv, -1)
        self.assertEqual(protocol.latest_fields["AB50"]["br"], "rail_invalid")
        self.assertEqual(protocol.latest_fields["AB50"]["periodMs"], "1000")
        self.assertEqual(protocol.latest_fields["AB50"]["boundary"], "1")

    def test_parses_active_ads_check_and_cache_telemetry(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line(
            "ADSCHK,id=7,ok=1,chip=1262,idreg=0x03,rev=0,adc1=1,adc2=0,"
            "power=0x02,interface=0x04,mode0=0x00,mode1=0x00,mode2=0x0F,"
            "inpmux=0x01,refmux=0x24,dr=38400,filter=sinc1,chop=0,"
            "delayUs=0,pga=bypass,reference=avdd-avss,vbias=1")
        protocol.feed_line(
            "ADSCHKSTAT,id=7,samples=100,fresh=100,changed=99,"
            "periodMinUs=25,periodAvgUs=26,periodMaxUs=28,spi=0,timeout=0,"
            "stale=0,statusErr=0,reset=0,restore=ok,durationUs=3000")
        protocol.feed_line(
            "ADS50,mode=VOLT,n=50,frameUs=12000/13000,attemptsPerCell=1.02,"
            "rawConversions=3264,profileHit=3136,profileMiss=64,bypassHit=3136,"
            "gainHit=0,registerCacheHit=6300,registerWrites=3200,"
            "registerReadbacks=13,singleSampleCells=3000,tripleSampleCells=200,"
            "precisionFrame=1,precisionFrames=3,freshCells=3200")
        self.assertEqual(protocol.latest_fields["ADSCHK"]["chip"], "1262")
        self.assertEqual(protocol.latest_fields["ADSCHKSTAT"]["fresh"], "100")
        self.assertEqual(protocol.latest_fields["ADS50"]["profileHit"], "3136")
        self.assertEqual(protocol.counters.summary_lines, 3)

    def test_parses_abat_time_scheduler_fields(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line(
            "ABAT,bt=4012,valid=1,fresh=1,ageMs=12,periodMs=1000,due=0,"
            "run=12,skip=2,defer=1,boundary=4,restoreFail=0,"
            "raw=123,a8d=100,ac=2005900,a8g=2006000,ratio=2/1,rail=5200000,"
            "railState=ok,vbias=1,samples=3,sampleUs=820,restore=ok,reason=ok")
        self.assertEqual(protocol.latest_battery_mv, 4012)
        self.assertEqual(protocol.latest_fields["ABAT"]["restore"], "ok")
        self.assertEqual(protocol.latest_fields["ABAT"]["ratio"], "2/1")
        self.assertEqual(protocol.latest_fields["ABAT"]["run"], "12")
        self.assertEqual(protocol.latest_fields["ABAT"]["restoreFail"], "0")

    def test_voltage_frame_crc_scale_invalid_and_pga(self) -> None:
        values = [str(index - 4) for index in range(16)]
        values[5] = "X12"
        lines = measurement_lines("V", 8, 2, values, [1, 2, 4, 8] * 4)
        protocol = TextProtocolParser()
        frame = None
        for line in lines:
            frame = protocol.feed_line(line) or frame
        self.assertIsNotNone(frame)
        self.assertEqual(frame.mode, "VOLT")
        self.assertEqual(frame.scale, -6)
        self.assertIsNone(frame.values_fixed[5])
        self.assertEqual(frame.error_reasons[5], 0x12)
        self.assertEqual(frame.pga_gains[3], 8)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(protocol.counters.voltage_frames, 1)

    def test_resistance_frame_golden_crc_and_unit(self) -> None:
        values = [str(10_000_000 + index) for index in range(8)]
        lines = measurement_lines("R", 9, 1, values, [16] * 8)
        protocol = TextProtocolParser()
        frame = None
        for line in lines:
            frame = protocol.feed_line(line) or frame
        self.assertIsNotNone(frame)
        self.assertEqual(frame.mode, "RES")
        self.assertEqual(frame.unit, "ohm")
        self.assertAlmostEqual(frame.values_si[0], 10_000.0)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(protocol.counters.resistance_frames, 1)

    def test_measurement_crc_covers_pga_chunks(self) -> None:
        lines = measurement_lines("V", 10, 1, ["1"] * 8, [1] * 8)
        lines[-2] = "P0," + "02" * 8
        protocol = TextProtocolParser()
        frame = None
        for line in lines:
            frame = protocol.feed_line(line) or frame
        self.assertIsNotNone(frame)
        self.assertFalse(frame.crc_ok)
        self.assertEqual(protocol.counters.crc_errors, 1)

    def test_worst_case_measurement_fixture_fits_fixed_slot(self) -> None:
        values = ["999999999999"] * 64
        lines = measurement_lines("R", 4_294_967_295, 8, values, [32] * 64)
        payload = "".join(line + "\n" for line in lines).encode("ascii")
        self.assertEqual(len(payload), 1516)
        self.assertLessEqual(len(payload), 1536)


if __name__ == "__main__":
    unittest.main()
