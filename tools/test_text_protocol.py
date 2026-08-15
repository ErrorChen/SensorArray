import unittest
import zlib

from text_protocol import TextProtocolParser


def measurement_lines(tag: str, sequence: int, rows: int,
                      values: list[str], gains: list[int],
                      expected_mask: int | None = None,
                      acquired_mask: int | None = None,
                      fresh_mask: int | None = None,
                      max_diag: bool = False) -> list[str]:
    mode, unit, scale, fmt = (("VOLT", "V", -6, "uv") if tag == "V" else
                              ("RES", "ohm", -3, "mohm"))
    cells = rows * 8
    active_mask = (1 << cells) - 1
    if expected_mask is None:
        expected_mask = active_mask
    if acquired_mask is None:
        acquired_mask = expected_mask
    if fresh_mask is None:
        fresh_mask = active_mask
    valid_mask = sum(1 << index for index, value in enumerate(values)
                     if not value.startswith("X"))
    error_mask = ((1 << cells) - 1) ^ valid_mask
    lines = [
        f"{tag},seq={sequence},ts=18446744073709551615,rows={rows},cells={cells},"
        f"gen=4294967295,rid=4294967295,mode={mode},unit={unit},scale={scale},"
        f"valid={valid_mask:016X},fresh={fresh_mask:016X},"
        f"error={error_mask:016X},expected={expected_mask:016X},"
        f"acquired={acquired_mask:016X},ref=AVDD_AVSS,rail=1,"
        f"age={4294967295 if max_diag else 0},"
        f"avdd={2147483647 if max_diag else 0},"
        f"avss={-2147483648 if max_diag else 0},"
        f"vexc={2147483647 if max_diag else 0},"
        f"rref={4294967295 if max_diag else 0},"
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


def mixed_lines(sequence: int, rows: int, profile: str) -> list[str]:
    mode_names = {"C": "CAP", "V": "VOLT", "R": "RES"}
    units = {"C": "pF", "V": "V", "R": "ohm"}
    scales = {"C": -6, "V": -6, "R": -3}
    fmts = {"C": "pf6", "V": "uv-x", "R": "mohm-x"}
    cells = rows * 8
    header_expected = 0
    for row, char in enumerate(profile, start=1):
        if char != "N":
            header_expected |= 0xFF << ((row - 1) * 8)
    header_acquired = header_expected
    lines = [
        "M,seq=%d,ts=99,rows=%d,cells=%d,rgen=4,rrid=5,pgen=7,prid=8,"
        "profile=%s,expected=%016X,acquired=%016X,fmt=mix1" % (
            sequence, rows, cells, profile, header_expected, header_acquired)
    ]
    for row, char in enumerate(profile, start=1):
        if char == "N":
            continue
        lines.append(
            "MR,s=%d,m=%s,unit=%s,scale=%d,expected=FF,acquired=FF,"
            "valid=FF,fresh=FF,error=00,"
            "fmt=%s,D=" % (row, mode_names[char], units[char], scales[char],
                           fmts[char])
            + ",".join(str(row * 100 + index) for index in range(8))
        )
    crc_payload = "".join(line + "\n" for line in lines).encode("ascii")
    lines.append("K,seq=%d,rgen=4,rrid=5,pgen=7,prid=8,crc=%08X"
                 % (sequence, zlib.crc32(crc_payload) & 0xFFFFFFFF))
    return lines


def parse_lines(lines: list[str]) -> tuple[TextProtocolParser, object]:
    protocol = TextProtocolParser()
    frame = None
    for line in lines:
        frame = protocol.feed_line(line) or frame
    return protocol, frame


class TextProtocolParserTest(unittest.TestCase):
    def test_reassembles_mixed_row_frame_and_crc(self) -> None:
        profile = "CVVRRVVC"
        lines = [
            "M,seq=21,ts=99,rows=8,cells=64,rgen=4,rrid=5,pgen=7,prid=8,"
            f"profile={profile},fmt=mix1"
        ]
        for row, mode in enumerate(profile, start=1):
            unit, scale, fmt = {
                "C": ("pF", -6, "pf6"),
                "V": ("V", -6, "uv-x"),
                "R": ("ohm", -3, "mohm-x"),
            }[mode]
            lines.append(
                f"MR,s={row},m={'CAP' if mode == 'C' else ('VOLT' if mode == 'V' else 'RES')},"
                f"unit={unit},scale={scale},valid=FF,fresh=FF,error=00,fmt={fmt},D="
                + ",".join(str(row * 100 + index) for index in range(8))
            )
        crc_payload = "".join(line + "\n" for line in lines).encode("ascii")
        lines.append(
            "K,seq=21,rgen=4,rrid=5,pgen=7,prid=8,"
            f"crc={zlib.crc32(crc_payload) & 0xFFFFFFFF:08X}"
        )
        protocol = TextProtocolParser()
        frame = None
        for line in lines:
            frame = protocol.feed_line(line) or frame
        self.assertIsNotNone(frame)
        self.assertEqual(frame.profile, profile)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(frame.row_frames[2].mode, "VOLT")
        self.assertEqual(frame.row_frames[8].values_fixed[-1], 807)

    def test_reassembles_mixed_frame_with_inactive_trailing_rows(self) -> None:
        protocol, frame = parse_lines(mixed_lines(31, 3, "CVRNNNNN"))
        self.assertIsNotNone(frame)
        self.assertEqual(frame.profile, "CVRNNNNN")
        self.assertEqual(frame.rows, 3)
        self.assertEqual(frame.cells, 24)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(set(frame.row_frames), {1, 2, 3})
        self.assertEqual(frame.row_frames[1].mode, "CAP")
        self.assertEqual(frame.row_frames[3].mode, "RES")
        self.assertEqual(protocol.counters.malformed, 0)
        self.assertEqual(protocol.counters.mixed_frames, 1)

    def test_mixed_only_stream_is_not_reported_as_zero_data(self) -> None:
        protocol, _ = parse_lines(mixed_lines(41, 8, "CVVRRVVC"))
        summary = protocol.summary("SERIAL_DONE")
        self.assertIn("mix=1", summary)
        self.assertIn("fps=", summary)
        self.assertNotEqual(protocol.counters.mixed_frames, 0)

    def test_reassembles_mixed_frame_with_failed_none_row(self) -> None:
        protocol, frame = parse_lines(mixed_lines(32, 3, "NVRNNNNN"))
        self.assertIsNotNone(frame)
        self.assertEqual(frame.profile, "NVRNNNNN")
        self.assertEqual(frame.rows, 3)
        self.assertTrue(frame.crc_ok)
        self.assertEqual(set(frame.row_frames), {2, 3})
        self.assertEqual(frame.row_frames[2].mode, "VOLT")
        self.assertEqual(frame.row_frames[3].values_fixed[-1], 307)
        self.assertEqual(protocol.counters.malformed, 0)

    def test_rejects_mixed_frame_with_missing_active_row(self) -> None:
        lines = mixed_lines(33, 3, "CVRNNNNN")
        lines.remove(next(line for line in lines if line.startswith("MR,s=3,")))
        protocol, frame = parse_lines(lines)
        self.assertIsNone(frame)
        self.assertGreater(protocol.counters.malformed, 0)

    def test_rejects_mixed_row_for_inactive_profile_slot(self) -> None:
        lines = mixed_lines(34, 3, "CVRNNNNN")
        lines.insert(
            -1,
            "MR,s=4,m=CAP,unit=pF,scale=-6,valid=FF,fresh=FF,error=00,fmt=pf6,D="
            + ",".join(str(400 + index) for index in range(8)),
        )
        crc_payload = "".join(line + "\n" for line in lines[:-1]).encode("ascii")
        lines[-1] = "K,seq=34,rgen=4,rrid=5,pgen=7,prid=8,crc=%08X" % (
            zlib.crc32(crc_payload) & 0xFFFFFFFF)
        protocol, frame = parse_lines(lines)
        self.assertIsNone(frame)
        self.assertGreater(protocol.counters.malformed, 0)

    def test_mixed_parses_expected_acquired_open_and_partial(self) -> None:
        profile = "CVRNNNNN"
        header_expected = (1 << 24) - 1
        header_acquired = 0xFF | (0x0F << 8) | (0xFF << 16)
        lines = [
            "M,seq=50,ts=99,rows=3,cells=24,rgen=4,rrid=5,pgen=7,prid=8,"
            f"profile={profile},expected={header_expected:016X},"
            f"acquired={header_acquired:016X},fmt=mix1",
            "MR,s=1,m=CAP,unit=pF,scale=-6,expected=FF,acquired=FF,"
            "valid=00,fresh=FF,error=FF,fmt=pf6,D="
            + ",".join("X14" for _ in range(8)),
            "MR,s=2,m=VOLT,unit=V,scale=-6,expected=FF,acquired=0F,"
            "valid=0F,fresh=0F,error=F0,fmt=uv-x,D="
            + ",".join(str(200 + index) for index in range(4))
            + "," + ",".join("X0D" for _ in range(4)),
            "MR,s=3,m=RES,unit=ohm,scale=-3,expected=FF,acquired=FF,"
            "valid=00,fresh=FF,error=FF,fmt=mohm-x,D="
            + ",".join("X0D" for _ in range(8)),
        ]
        crc_payload = "".join(line + "\n" for line in lines).encode("ascii")
        lines.append("K,seq=50,rgen=4,rrid=5,pgen=7,prid=8,crc=%08X" %
                     (zlib.crc32(crc_payload) & 0xFFFFFFFF))
        protocol, frame = parse_lines(lines)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.expected_mask, header_expected)
        self.assertEqual(frame.acquired_mask, header_acquired)
        self.assertEqual(frame.row_frames[1].expected_mask, 0xFF)
        self.assertEqual(frame.row_frames[1].acquired_mask, 0xFF)
        self.assertEqual(frame.row_frames[1].valid_mask, 0)
        self.assertEqual(frame.row_frames[1].fresh_mask, 0xFF)
        self.assertEqual(frame.row_frames[1].error_mask, 0xFF)
        self.assertEqual(frame.row_frames[2].acquired_mask, 0x0F)
        self.assertEqual(frame.row_frames[2].fresh_mask, 0x0F)
        self.assertEqual(frame.row_frames[3].acquired_mask, 0xFF)
        self.assertEqual(protocol.counters.malformed, 0)

    def test_mixed_rejects_row_acquired_outside_expected(self) -> None:
        lines = mixed_lines(51, 3, "CVRNNNNN")
        for index, line in enumerate(lines):
            if line.startswith("MR,s=1,"):
                lines[index] = line.replace("expected=FF,acquired=FF",
                                            "expected=0F,acquired=FF")
                break
        crc_payload = "".join(line + "\n" for line in lines[:-1]).encode("ascii")
        lines[-1] = "K,seq=51,rgen=4,rrid=5,pgen=7,prid=8,crc=%08X" % (
            zlib.crc32(crc_payload) & 0xFFFFFFFF)
        protocol, frame = parse_lines(lines)
        self.assertIsNone(frame)
        self.assertGreater(protocol.counters.malformed, 0)

    def test_mixed_rejects_header_acquired_mismatch(self) -> None:
        lines = mixed_lines(52, 3, "CVRNNNNN")
        for index, line in enumerate(lines):
            if line.startswith("M,"):
                lines[index] = line.replace("acquired=0000000000FFFFFF",
                                            "acquired=0000000000000000")
                break
        crc_payload = "".join(line + "\n" for line in lines[:-1]).encode("ascii")
        lines[-1] = "K,seq=52,rgen=4,rrid=5,pgen=7,prid=8,crc=%08X" % (
            zlib.crc32(crc_payload) & 0xFFFFFFFF)
        protocol, frame = parse_lines(lines)
        self.assertIsNone(frame)
        self.assertGreater(protocol.counters.malformed, 0)

    def test_p50_summary_is_not_parsed_as_pga_chunk(self) -> None:
        protocol = TextProtocolParser()
        self.assertIsNone(
            protocol.feed_line("P50,prof=360,warn=72,res=0,bg=0"))
        self.assertEqual(protocol.counters.malformed, 0)
        self.assertEqual(protocol.counters.summary_lines, 1)
        self.assertEqual(protocol.latest_fields["P50"]["prof"], "360")

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
        self.assertEqual(frame.expected_mask, (1 << 40) - 1)
        self.assertEqual(frame.acquired_mask, 0)
        self.assertEqual(protocol.counters.cap_frames, 1)

    def test_cap_frame_parses_expected_acquired_masks(self) -> None:
        active = (1 << 40) - 1
        lines = ["C,seq=7,ts=123,rows=5,cells=40,rf=1F,pf=1F,sf=1F,"
                 f"expected={active:016X},acquired={active:016X},"
                 "bad=0/0/0,fmt=pf6,n=40"]
        values = list(range(40))
        for chunk in range(3):
            start = chunk * 16
            lines.append("D%d,%s" % (chunk, ",".join(str(value)
                                                     for value in values[start:start + 16])))
        crc_payload = "".join(line + "\n" for line in lines).encode("ascii")
        lines.append(f"K,seq=7,crc={zlib.crc32(crc_payload) & 0xFFFFFFFF:08X}")
        protocol, frame = parse_lines(lines)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.expected_mask, active)
        self.assertEqual(frame.acquired_mask, active)
        self.assertEqual(protocol.counters.malformed, 0)

    def test_parses_battery_invalid_reason(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("A50,chip=1262,adc=1,bt=-1,br=rail")
        self.assertEqual(protocol.latest_battery_mv, -1)
        self.assertEqual(protocol.latest_fields["A50"]["br"], "rail")

    def test_parses_current_ab50_battery_invalid_reason(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line(
            "AB50,bt=-1,valid=0,br=rail_invalid,bs=stale,ageMs=1001,"
            "periodMs=1000,due=1,run=2,validRun=1,invalidRun=1,skip=3,"
            "defer=4,boundary=1,restoreFail=0,retry=0/1,unstable=1,"
            "timeout=0,spreadRaw=1043571,spreadMaxRaw=1043571,"
            "sampleUs=800/900")
        self.assertEqual(protocol.latest_battery_mv, -1)
        self.assertEqual(protocol.latest_fields["AB50"]["br"], "rail_invalid")
        self.assertEqual(protocol.latest_fields["AB50"]["periodMs"], "1000")
        self.assertEqual(protocol.latest_fields["AB50"]["boundary"], "1")
        self.assertEqual(protocol.latest_fields["AB50"]["invalidRun"], "1")
        self.assertEqual(protocol.latest_fields["AB50"]["spreadRaw"], "1043571")

    def test_parses_nonblocking_battery_error_event(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line(
            "BATERR,seq=42,err=0x103,reason=range_error,valid=0,lastGoodMv=4153,"
            "lastGoodValid=1,sampleUs=620,restore=ok,action=report_continue")
        self.assertEqual(protocol.counters.malformed, 0)
        self.assertEqual(protocol.counters.summary_lines, 1)
        self.assertEqual(protocol.latest_fields["BATERR"]["reason"], "range_error")
        self.assertEqual(protocol.latest_fields["BATERR"]["action"], "report_continue")

    def test_rowmodes_lifecycle_ack_then_single_applied(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("RMACK,id=7,old=RRRRRRRR,new=CVVRRVVC,state=accepted")
        protocol.feed_line(
            "RMAPP,id=7,gen=3,seq=25,profile=CVVRRVVC,state=applied")
        self.assertEqual(protocol.counters.rmack, 1)
        self.assertEqual(protocol.counters.rmapp, 1)
        self.assertEqual(protocol.counters.rmerr, 0)
        self.assertEqual(protocol.counters.lifecycle_duplicate_terminal, 0)
        self.assertEqual(protocol.counters.lifecycle_terminal_without_ack, 0)

    def test_rowmodes_lifecycle_ack_then_single_rejected(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("RMACK,id=8,old=CCCCCCCC,new=RRRRRRRR,state=accepted")
        protocol.feed_line(
            "RMERR,id=8,gen=2,seq=26,profile=RRRRRRRR,err=0x103,"
            "state=rejected,route=SAFE")
        self.assertEqual(protocol.counters.rmack, 1)
        self.assertEqual(protocol.counters.rmapp, 0)
        self.assertEqual(protocol.counters.rmerr, 1)
        self.assertEqual(protocol.counters.lifecycle_duplicate_terminal, 0)
        self.assertEqual(protocol.counters.lifecycle_terminal_without_ack, 0)

    def test_rowmodes_duplicate_terminal_is_recorded(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("RMACK,id=9,old=CCCCCCCC,new=VVVVVVVV,state=accepted")
        protocol.feed_line(
            "RMAPP,id=9,gen=4,seq=27,profile=VVVVVVVV,state=applied")
        protocol.feed_line(
            "RMAPP,id=9,gen=4,seq=28,profile=VVVVVVVV,state=applied")
        self.assertEqual(protocol.counters.rmapp, 2)
        self.assertEqual(protocol.counters.lifecycle_duplicate_terminal, 1)
        self.assertEqual(protocol.counters.lifecycle_terminal_without_ack, 0)

    def test_rowmodes_terminal_without_ack_is_recorded(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line(
            "RMERR,id=10,gen=1,seq=29,profile=CVVRRVVC,err=0x102,"
            "state=rejected,route=SAFE")
        self.assertEqual(protocol.counters.rmerr, 1)
        self.assertEqual(protocol.counters.lifecycle_duplicate_terminal, 0)
        self.assertEqual(protocol.counters.lifecycle_terminal_without_ack, 1)

    def test_rowmodes_early_reject_is_a_terminal(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("RMACK,id=11,old=CCCCCCCC,new=RVRCCVVR,state=accepted")
        protocol.feed_line(
            "RMERR,id=11,profile=RVRCCVVR,state=rejected,err=0x102,reason=pending")
        self.assertEqual(protocol.counters.rmerr, 1)
        self.assertEqual(protocol.counters.lifecycle_duplicate_terminal, 0)
        self.assertEqual(protocol.counters.lifecycle_terminal_without_ack, 0)

    def test_rowmodes_missing_terminal_is_reported(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("RMACK,id=12,old=RRRRRRRR,new=CCCCCCCC,state=accepted")
        errors = protocol.rowmode_lifecycle_errors()
        self.assertEqual(errors["unterminated"], 1)
        self.assertEqual(errors["duplicate"], 0)
        self.assertEqual(errors["terminal_without_ack"], 0)

    def test_rowmodes_duplicate_and_without_ack_are_reported(self) -> None:
        protocol = TextProtocolParser()
        protocol.feed_line("RMACK,id=13,old=CCCCCCCC,new=RRRRRRRR,state=accepted")
        protocol.feed_line("RMAPP,id=13,gen=1,seq=1,profile=RRRRRRRR,state=applied")
        protocol.feed_line("RMAPP,id=13,gen=1,seq=2,profile=RRRRRRRR,state=applied")
        protocol.feed_line("RMERR,id=99,gen=1,seq=3,profile=CCCCCCCC,state=rejected")
        errors = protocol.rowmode_lifecycle_errors()
        self.assertEqual(errors["unterminated"], 0)
        self.assertEqual(errors["duplicate"], 1)
        self.assertEqual(errors["terminal_without_ack"], 1)

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

    def test_maximal_abat_fixture_exceeds_control_reply_guard(self) -> None:
        # Mirror sensorarrayAdsGapFormatBattery with every field at its widest
        # representable value. The C formatter must reject this instead of
        # publishing a truncated 511-byte control reply.
        fmt = (
            "ABAT,bt=%ld,valid=%u,fresh=%u,ageMs=%lu,lastGoodMv=%ld,"
            "lastGoodValid=%u,lastGoodFresh=%u,lastGoodAgeMs=%lu,"
            "lastGoodFrame=%lu,periodMs=%lu,due=%u,run=%lu,validRun=%lu,"
            "invalidRun=%lu,skip=%lu,defer=%lu,boundary=%lu,restoreFail=%lu,"
            "retry=%lu/%lu,unstable=%lu,timeout=%lu,spreadRaw=%lu,"
            "spreadMaxRaw=%lu,raw=%ld,a8d=%ld,ac=%ld,a8g=%ld,ratio=%u/%u,"
            "rail=%ld,railState=%s,vbias=%u,samples=%lu,sampleUs=%lu,"
            "restore=%s,reason=%s\n"
        )
        widest = (
            -2147483648, 1, 1, 4294967295, -2147483648, 1, 1, 4294967295,
            4294967295, 4294967295, 1, 4294967295, 4294967295, 4294967295,
            4294967295, 4294967295, 4294967295, 4294967295, 4294967295,
            4294967295, 4294967295, 4294967295, 4294967295, 4294967295,
            -2147483648, -2147483648, -2147483648, -2147483648, 4294967295,
            4294967295, -2147483648, "bad", 1, 4294967295, 4294967295, "fail",
            "reference_invalid",
        )
        line = fmt % widest
        self.assertGreater(len(line), 512)
        protocol = TextProtocolParser()
        protocol.feed_line(line.rstrip("\n"))
        self.assertEqual(protocol.counters.summary_lines, 1)
        self.assertEqual(protocol.latest_battery_mv, -2147483648)

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

    def test_measurement_expected_acquired_open_semantics(self) -> None:
        active = (1 << 16) - 1
        values = [str(index) for index in range(16)]
        values[5] = "X0D"
        lines = measurement_lines("V", 12, 2, values, [1] * 16,
                                  expected_mask=active, acquired_mask=active)
        protocol, frame = parse_lines(lines)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.expected_mask, active)
        self.assertEqual(frame.acquired_mask, active)
        self.assertEqual(frame.fresh_mask, active)
        self.assertEqual(frame.valid_mask, active & ~(1 << 5))
        self.assertEqual(frame.error_mask, 1 << 5)
        self.assertEqual(frame.error_reasons[5], 0x0D)
        self.assertEqual(protocol.counters.malformed, 0)

    def test_measurement_partial_acquired_is_not_fresh(self) -> None:
        active = (1 << 16) - 1
        acquired = active & ~1
        values = ["X04"] + [str(index) for index in range(1, 16)]
        lines = measurement_lines("R", 13, 2, values, [1] * 16,
                                  expected_mask=active,
                                  acquired_mask=acquired,
                                  fresh_mask=acquired)
        protocol, frame = parse_lines(lines)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.expected_mask, active)
        self.assertEqual(frame.acquired_mask, acquired)
        self.assertEqual(frame.fresh_mask, acquired)
        self.assertEqual(frame.valid_mask & 1, 0)
        self.assertEqual(frame.error_reasons[0], 0x04)
        self.assertEqual(protocol.counters.malformed, 0)

    def test_measurement_masks_cover_active_rows_only(self) -> None:
        for rows in range(1, 9):
            cells = rows * 8
            active = (1 << cells) - 1
            values = [str(index) for index in range(cells)]
            lines = measurement_lines("V", 20 + rows, rows, values,
                                      [1] * cells,
                                      expected_mask=active,
                                      acquired_mask=active)
            protocol, frame = parse_lines(lines)
            self.assertIsNotNone(frame)
            self.assertEqual(frame.expected_mask, active)
            self.assertEqual(frame.acquired_mask, active)
            self.assertEqual(protocol.counters.malformed, 0)

    def test_measurement_rejects_acquired_outside_expected(self) -> None:
        active = (1 << 16) - 1
        lines = measurement_lines("V", 14, 2, [str(index) for index in range(16)],
                                  [1] * 16,
                                  expected_mask=active & ~1,
                                  acquired_mask=active)
        protocol, frame = parse_lines(lines)
        self.assertIsNone(frame)
        self.assertGreater(protocol.counters.malformed, 0)

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
        lines = measurement_lines(
            "R", 4_294_967_295, 8, values, [32] * 64, max_diag=False)
        payload = "".join(line + "\n" for line in lines).encode("ascii")
        self.assertEqual(len(payload), 1522)
        self.assertLessEqual(len(payload), 1536)

    def test_full_diagnostic_measurement_fixture_exceeds_data_slot(self) -> None:
        values = ["999999999999"] * 64
        lines = measurement_lines(
            "R", 4_294_967_295, 8, values, [32] * 64, max_diag=True)
        payload = "".join(line + "\n" for line in lines).encode("ascii")
        self.assertGreater(len(payload), 1536)


if __name__ == "__main__":
    unittest.main()
