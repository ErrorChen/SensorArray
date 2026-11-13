#!/usr/bin/env python3
"""Run real main/MIXED/output/encoder logic with synthetic child acquisitions.

ADC retry/classification/budget tests live in test_ads_crc_disposition.py. This
suite tests downstream ownership of its disposition, including an earlier CRC
error followed by a hard failure. No hardware or scientific capture is used.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import zlib
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders
from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "main/main.c").read_text()
    async_source = (root / "main/output/sensorarrayAsyncLog.c").read_text()
    publish_at = source.index("/* 本轮新增: only the acquisition owner publishes")
    publish = function(source[publish_at:], "static sensorarrayConfigIdentity_t sensorarrayPublishConfigAuthority(")
    terminal_at = source.index("static void sensorarrayEmitConfigTerminal(",
                               source.index("static esp_err_t sensorarrayApplyMeasurementMode("))
    terminal = function(source[terminal_at:], "static void sensorarrayEmitConfigTerminal(")
    functions = "\n\n".join((publish,
        function(source, "static void sensorarrayCaptureFrameIdentity("), terminal,
        function(source, "static esp_err_t sensorarrayRunOneFrame(")))
    start = async_source.index("            bool emitted = frame->freshFrame")
    end = async_source.index("            sensorarrayAsyncLogUpdateSummary(", start)
    async_gate = async_source[start:end]
    start = source.index("        if (ctx->asyncLogReady) {\n            esp_err_t snapshotErr =")
    end = source.index("        sensorarrayBootBreadcrumbMarkStableProgress(", start)
    legacy_gate = source[start:end]
    harness = (root / "tests/host/ads_crc_frame_flow_test.c").read_text()
    for key, value in (("FUNCTIONS", functions), ("ASYNC_GATE", async_gate),
                       ("LEGACY_GATE", legacy_gate)):
        harness = harness.replace(f"/* ACTUAL_{key} */", value)
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native compiler required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-crc-flow-") as directory:
        temp = Path(directory)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        (fake / "esp_timer.h").write_text("#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n")
        generated = temp / "test.c"
        generated.write_text(harness)
        executable = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                   "-DCONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES=100",
                   "-DCONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG=0"]
        for path in (fake, root / "tests/host/frame_stubs", root / "tests/host/include",
                     root / "main", root / "main/output", root / "core/measure/include",
                     root / "core/measure/ads", root / "core/measure/fdc", root / "core/measure/mixed",
                     root / "core/config", root / "core/board", root / "core/boardSupport/include",
                     root / "components/ads126xAdc/include", root / "components/fdc2214Cap/include",
                     root / "components/tmuxSwitch/include"):
            command += ["-I", str(path)]
        command += [str(generated)]
        for path in ("core/measure/mixed/sensorarrayMixedRow.c",
                     "core/measure/sensorarrayFrameBuilder.c", "core/measure/sensorarrayScanPlan.c",
                     "core/measure/sensorarrayMeasurementMode.c", "core/measure/sensorarrayRowModeProfile.c",
                     "core/measure/sensorarrayConfigIdentity.c", "core/config/sensorarrayScanConfig.c",
                     "main/output/sensorarrayTextProtocol.c"):
            command.append(str(root / path))
        command += ["-lm", "-o", str(executable)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(executable)], check=True, text=True, capture_output=True)
    fixtures = 0
    for line in result.stdout.splitlines():
        if not line.startswith("FRAME_HEX,"):
            print(line)
            continue
        _, label, hex_data = line.split(",", 2)
        packet = bytes.fromhex(hex_data)
        lines = packet.splitlines(keepends=True)
        assert lines[-1].startswith(b"K,") and lines[-1].endswith(b"\n")
        match = re.search(rb"(?:^|,)crc=([0-9A-Fa-f]{8})(?:,|\n)", lines[-1])
        assert match and int(match[1], 16) == zlib.crc32(b"".join(lines[:-1]))
        def fields(line):
            return dict(part.split(b"=", 1) for part in line.strip().split(b",") if b"=" in part)
        header, trailer = fields(lines[0]), fields(lines[-1])
        for key in (b"session", b"bootId", b"rev"):
            assert header[key] == trailer[key], (label, key)
        assert header[b"prid"] == b"19" and int(header[b"rev"]) > 0
        rows = int(header[b"rows"])
        expected = (1 << (8 * rows)) - 1
        rejected = 1 << (8 if label.startswith("mixed") else 0)
        assert int(header[b"expected"], 16) == expected
        assert int(header[b"acquired"], 16) == expected & ~rejected
        if label.startswith("mixed"):
            row = next(line for line in lines if line.startswith(b"MR,s=2,"))
            masks = fields(row)
            assert row.split(b",D=", 1)[1].startswith(b"X02,")
            for key in (b"acquired", b"valid", b"fresh"):
                assert int(masks[key], 16) == 0xFE, (label, key)
            assert int(masks[b"error"], 16) == 1
        else:
            assert lines[1].startswith(b"D0,X02,")
            for key in (b"valid", b"fresh"):
                assert int(header[key], 16) == expected & ~rejected, (label, key)
            assert int(header[b"error"], 16) == rejected
        assert b"987654321" not in packet  # Masked child payload must never escape.
        fixtures += 1
        print(f"CRC_FLOW_ENCODER,label={label},bytes={len(packet)},ascii_crc=PASS,invalid=X02,masks=PASS,identity=PASS")
    assert fixtures == 3, f"expected 3 real encoder fixtures, got {fixtures}"
    print("ADS_CRC_FRAME_FLOW,passed=1,source=actual_main+mixed+output_gates+encoder,fixture=synthetic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
