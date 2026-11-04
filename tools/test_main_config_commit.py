#!/usr/bin/env python3
"""Compile actual main.c commit/terminal code with injected hardware boundaries."""
from pathlib import Path
import subprocess
import tempfile
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders


def function(source, name):
    start = source.index(name)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "main/main.c").read_text()
    # Exact production branches: no hand-maintained mirror of decisions.
    start = source.index("        case SENSORARRAY_COMMAND_MEASUREMENT_MODE:\n")
    end = source.index("        case SENSORARRAY_COMMAND_ROW_MODES:", start)
    mode_case = source[start:end]
    publish_start = source.index("/* 本轮新增: only the acquisition owner publishes")
    publish = function(source[publish_start:], "static sensorarrayConfigIdentity_t sensorarrayPublishConfigAuthority(")
    terminal_start = source.index("static void sensorarrayEmitConfigTerminal(", source.index("static esp_err_t sensorarrayApplyMeasurementMode("))
    terminal = function(source[terminal_start:], "static void sensorarrayEmitConfigTerminal(")
    mixed = function(source, "else if (mixedRequestFailed)")
    mixed_body = mixed[mixed.index("{"):]
    harness = (root / "tests/host/test_main_config_commit.c").read_text()
    harness = harness.replace("/* INSERT_PRODUCTION_PUBLISH */", publish)
    harness = harness.replace("/* INSERT_PRODUCTION_TERMINAL */", terminal)
    harness = harness.replace("/* INSERT_PRODUCTION_MODE_CASE */", mode_case)
    harness = harness.replace("/* INSERT_PRODUCTION_MIXED_FAILURE */", mixed_body)
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("a native host compiler is required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-main-config-") as temporary:
        temp = Path(temporary)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        generated = temp / "test.c"
        generated.write_text(harness)
        executable = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
        for include in (fake, root / "tests/host/include", root / "core/measure/include", root / "core/config"):
            command += ["-I", str(include)]
        command += [str(generated), str(root / "core/measure/sensorarrayMeasurementMode.c"),
                    str(root / "core/measure/sensorarrayRowModeProfile.c"),
                    str(root / "core/measure/sensorarrayConfigIdentity.c"),
                    str(root / "core/config/sensorarrayScanConfig.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
