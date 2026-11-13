#!/usr/bin/env python3
"""Compare the harness against actual main/MixedRow/encoder bytes, not a PTY model.

Child acquisition is synthetic. The complete production frame dispatcher,
full-eight-row homogeneity predicate, mixed merge and encoder are compiled by
the existing CRC frame-flow harness. No hardware or capture files are changed.
"""
from contextlib import redirect_stdout
import io
from pathlib import Path
from unittest.mock import patch
import zlib

import test_ads_crc_frame_flow as production_flow
from run_firmware_benchmark import BenchmarkCollector, Configuration, IDENTITY
from text_protocol import TextProtocolParser, parse_fields


EXTENSION = r"""
int main(void)
{
    assert(originalSuite() == 0);
    const struct { const char *label, *profile; unsigned rows; } cases[] = {
        {"mixed1_cap", "CVRCVRCV", 1},
        {"mixed2_cap", "CCVCCCCC", 2},
        {"mixed1_volt", "VVRCVRCV", 1},
        {"mixed2_volt", "VVRCVRCV", 2},
        {"mixed1_res", "RRCRRRRR", 1},
        {"homogeneous_cap", "CCCCCCCC", 1},
        {"homogeneous_volt", "VVVVVVVV", 1},
        {"homogeneous_res", "RRRRRRRR", 1},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset(cases[i].profile, true, cases[i].rows);
        assert(sensorarrayRunOneFrame(&ctx) == ESP_OK);
        assert(ctx.frame.mixedProfile == !sensorarrayRowModeProfileIsHomogeneous(ctx.rowModeProfile.modes));
        assert(ctx.frame.identity.profileRequestId == 19);
        assert(sensorarrayTextProtocolBuildFrame(&ctx.frame, &outputPacket) == ESP_OK);
        printf("MIXED_WIRE_HEX,%s,%s,%u,", cases[i].label, cases[i].profile, cases[i].rows);
        for (unsigned j = 0; j < outputPacket.length; ++j) {
            printf("%02X", (unsigned char)outputPacket.data[j]);
        }
        putchar('\n');
    }
    return 0;
}
"""


def main():
    root = Path(production_flow.__file__).resolve().parents[1]
    harness_path = root / "tests/host/ads_crc_frame_flow_test.c"
    original_read = Path.read_text
    def read_source(path, *args, **kwargs):
        source = original_read(path, *args, **kwargs)
        if path == harness_path:
            assert "int main(void)\n{" in source
            source = source.replace("int main(void)\n{", "int originalSuite(void)\n{", 1) + EXTENSION
        return source
    output = io.StringIO()
    with patch.object(Path, "read_text", read_source), redirect_stdout(output):
        assert production_flow.main() == 0
    tested = 0
    for line in output.getvalue().splitlines():
        if not line.startswith("MIXED_WIRE_HEX,"):
            continue
        _, name, profile, rows, hex_data = line.split(",", 4)
        payload = bytes.fromhex(hex_data)
        lines = payload.decode("ascii").splitlines()
        header, trailer = parse_fields(lines[0]), parse_fields(lines[-1])
        assert zlib.crc32(b"".join(payload.splitlines(keepends=True)[:-1])) == int(trailer["crc"], 16)
        collector = BenchmarkCollector.__new__(BenchmarkCollector)
        collector.configuration = Configuration(name, "MIXED", int(rows), profile, 120)
        collector.phase = "capture"
        collector.parser = TextProtocolParser()
        collector.expected_identity = {key: header[key] for key in IDENTITY}
        collector.frame_header = collector.frame_tag = None
        collector.parsed_frames = collector.crc_valid_frames = collector.identity_mismatches = 0
        collector.wire_contract_mismatches = 0
        collector.identity_examples, collector.wire_contract_examples = [], []
        for wire_line in lines:
            collector.observe(wire_line)
        assert collector.parsed_frames == collector.crc_valid_frames == 1
        assert collector.identity_mismatches == 0
        assert collector.wire_contract_mismatches == 0, (name, collector.wire_contract_examples)
        mixed = len(set(profile)) > 1
        assert lines[0].startswith("M," if mixed else {"C": "C,", "V": "V,", "R": "R,"}[profile[0]])
        if mixed:
            assert header["profile"] == profile[:int(rows)] + "N" * (8 - int(rows))
        tested += 1
    assert tested == 8
    print("BENCHMARK_MIXED_WIRE,passed=8,source=actual_main+mixed+encoder,fixture=synthetic,full_profile_domain=verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
