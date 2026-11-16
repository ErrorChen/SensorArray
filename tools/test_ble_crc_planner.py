#!/usr/bin/env python3
"""Run actual CRC/planner/wire bodies against frozen pre-change source.

ASan/UBSan native execution, including real encoder golden bytes. The ROM API
boundary is implemented with Espressif's frozen software reference; this does
not execute the S3 ROM or measure its timing. No SDK or fixture is rewritten.
"""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import tempfile
import zlib

from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/fixtures/ble_crc_planner"
BASELINE_SHA = "5f5331f593c5d09e285b0b1a2d1199dbc63543572fe9f83ab05207c2b9bdd327"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def checked(path, expected):
    data = path.read_bytes()
    assert sha(data) == expected, (path, sha(data))
    return data.decode()


def renamed(body, prefix):
    for name in ("sensorarrayBleCrc32", "sensorarrayTextCrc32",
                 "sensorarrayBleMaxPayloadSize", "sensorarrayBlePlanFragments"):
        body = body.replace(name, prefix + name)
    return body


def run(command, environment, timeout=45):
    subprocess.run(command, check=True, timeout=timeout, env=environment)


def test_encoder(compiler, flags, environment, temp, text, baseline, sdk_reference):
    fake = temp / "fake"
    fake.mkdir()
    writeFakeHeaders(fake)
    (fake / "esp_rom_crc.h").write_text(
        "#pragma once\n#include <stdint.h>\n"
        "uint32_t esp_rom_crc32_le(uint32_t, const uint8_t *, uint32_t);\n")
    sdk = temp / "sdk_reference.c"
    sdk.write_text(sdk_reference)
    includes = [fake, ROOT / "tests/host/frame_stubs", ROOT / "tests/host/include",
                ROOT / "main", ROOT / "main/output", ROOT / "core/measure/include",
                ROOT / "core/config", ROOT / "core/boardSupport/include",
                ROOT / "components/ads126xAdc/include", ROOT / "components/fdc2214Cap/include",
                ROOT / "components/tmuxSwitch/include"]
    args = [part for p in includes for part in ("-I", str(p))]
    common_sources = [ROOT / p for p in (
        "tests/host/test_config_identity.c", "core/measure/sensorarrayConfigIdentity.c",
        "core/measure/sensorarrayMeasurementMode.c", "core/measure/sensorarrayRowModeProfile.c",
        "core/measure/sensorarrayFrameBuilder.c")]
    old_crc = function(baseline, "static uint32_t sensorarrayTextCrc32(")
    new_crc = function(text, "static uint32_t sensorarrayTextCrc32(")
    variants = {
        "old": text.replace(new_crc, old_crc),
        "portable": text,
        "rom_api": "#define ESP_PLATFORM 1\n" + text,
    }
    outputs = {}
    for name, source in variants.items():
        path = temp / ("encoder_" + name + ".c")
        path.write_text(source)
        output = temp / name
        output.mkdir()
        binary = temp / ("run_" + name)
        run([compiler, *flags, *args, *map(str, common_sources), str(path), str(sdk),
             "-lm", "-o", str(binary)], environment)
        run([str(binary), str(output)], environment)
        outputs[name] = {p.name: p.read_bytes() for p in output.iterdir()}
    assert outputs["old"] == outputs["portable"] == outputs["rom_api"]
    count = 0
    for name, data in outputs["old"].items():
        golden = ROOT / "tests/fixtures/config_identity" / name
        assert data == golden.read_bytes(), ("existing golden differs", name)
        if name.endswith(".frame"):
            body, trailer = data.rsplit(b"K,", 1)
            crc = re.search(rb"(?:^|,)crc=([0-9A-F]{8})(?:,|\n|$)", trailer)
            assert crc and int(crc[1], 16) == zlib.crc32(body), name
            count += 1
    print(f"CRC_REAL_ENCODER_GOLDEN,frames={count},variants=old/portable/rom_api,byte_equal=1")


def main():
    provenance = json.loads((FIXTURE / "provenance.json").read_text())
    assert provenance["fixture_sha256"] == BASELINE_SHA
    baseline = checked(FIXTURE / "baseline-functions.c.txt", BASELINE_SHA)
    sdk_reference = checked(FIXTURE / "esp-idf-v5.5.5-crc32-le.c.txt",
                            provenance["sdk_crc_reference"]["fixture_sha256"])
    ble_path = ROOT / "components/sensorarrayBle/sensorarrayBle.c"
    text_path = ROOT / "main/output/sensorarrayTextProtocol.c"
    ble, text = ble_path.read_text(), text_path.read_text()
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native host execution required; compile-only is not PASS")
    flags = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
             "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    environment = dict(os.environ, ASAN_OPTIONS="detect_leaks=0")
    harness = (ROOT / "tests/host/test_ble_crc_planner.c").read_text()
    constants = ble[ble.index("#define SENSORARRAY_BLE_U32_DECIMAL_MAX_CHARS"):
                    ble.index("enum {\n    BLE_IDX_SERVICE")]
    ble_crc = function(ble, "static uint32_t sensorarrayBleCrc32(")
    text_crc = function(text, "static uint32_t sensorarrayTextCrc32(")
    chunks = []
    for name, body in (("bleChunk", ble_crc), ("textChunk", text_crc)):
        # Execute the exact narrowing/chunk selection, including >UINT32_MAX,
        # without fabricating a multi-gigabyte valid pointer in a host test.
        declaration = re.search(r"#if SIZE_MAX > UINT32_MAX\n.*?#endif", body, re.S)[0]
        chunks.append("static uint32_t " + name + "(size_t length) {\n" +
                      declaration + "\nreturn chunk; }")
    old_max = function(baseline, "static size_t sensorarrayBleMaxPayloadSize(")
    old_format = re.search(r'"G,[^"\n]+"', old_max)[0]
    replacements = {
        "/* HEADER_CONSTANTS */": constants,
        "/* TINY_TAIL_CONSTANT */": re.search(
            r"^#define SENSORARRAY_BLE_TINY_TAIL_MIN [^\n]+", ble, re.M)[0],
        "/* SDK_REFERENCE */": sdk_reference.replace("esp_rom_crc32_le", "sdk_crc32_le"),
        "/* BASELINE_FUNCTIONS */": renamed(baseline, "old_"),
        "/* PORTABLE_FUNCTIONS */": renamed(ble_crc + "\n" + text_crc, "portable_"),
        "/* ROM_FUNCTIONS */": ble_crc + "\n" + text_crc,
        "/* PLANNER_FUNCTIONS */": "\n".join(function(ble, sig) for sig in (
            "static size_t sensorarrayBleDecimalChars(",
            "static size_t sensorarrayBleMaxPayloadSize(",
            "static bool sensorarrayBlePlanFragments(")),
        "/* WIRE_FUNCTION */": function(ble, "static esp_err_t sensorarrayBleSendFragment("),
        "/* CHUNK_SELECTION */": "\n".join(chunks),
        "/* OLD_HEADER_FORMAT */": "static const char oldHeaderFormat[] = " + old_format + ";",
    }
    for marker, value in replacements.items():
        assert harness.count(marker) == 1, marker
        harness = harness.replace(marker, value)
    with tempfile.TemporaryDirectory(prefix="sensorarray-crc-planner-") as directory:
        temp = Path(directory)
        generated = temp / "test.c"
        generated.write_text(harness)
        binary = temp / "test"
        run([compiler, *flags, str(generated), "-o", str(binary)], environment)
        run([str(binary)], environment)
        test_encoder(compiler, flags, environment, temp, text, baseline, sdk_reference)
    print("CRC_PLANNER_SOURCES," + json.dumps({
        str(p.relative_to(ROOT)): sha(p.read_bytes()) for p in (ble_path, text_path)}))
    print("CRC_PLANNER_RESULT,hardware=NOT_RUN,SDK_modified=0,fixtures_modified=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
