#!/usr/bin/env python3
"""Execute actual main fault-fill/emission and cleanup branches on the host."""
from pathlib import Path
import subprocess
import tempfile
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders
from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "main/main.c").read_text()
    fill = function(source, "static void sensorarrayAdsFaultFillContext(")
    emit = function(source, "static void sensorarrayEmitAdsFault(")
    homogeneous = function(source, "else if (!mixedProfile && !ctx->frame.freshFrame &&")
    homogeneous = homogeneous[homogeneous.index("{"):]
    mixed_at = source.index("        if (!ctx->frame.freshFrame && !ctx->frame.boundedCrcFailure) {\n            if (!mixedRequestFailed)")
    mixed = function(source[mixed_at:], "if (!mixedRequestFailed)")
    mixed = mixed[mixed.index("{"):]
    check = function(source, "static void sensorarrayRunPendingAdsCheck(")
    check = check[check.index("        sensorarrayAdsFaultEvent_t fault = {"):check.rindex("    }")]
    harness = (root / "tests/host/test_main_fault_context.c").read_text()
    for key, value in (("FILL", fill), ("EMIT", emit), ("HOMOGENEOUS", homogeneous),
                       ("MIXED", mixed), ("CHECK", check)):
        harness = harness.replace(f"/* INSERT_PRODUCTION_{key} */", value)
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("a native compiler is required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-main-fault-") as temporary:
        temp = Path(temporary)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        generated = temp / "test.c"
        generated.write_text(harness)
        executable = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
        for path in (fake, root / "tests/host/frame_stubs", root / "tests/host/include",
                     root / "main", root / "core/measure/include", root / "core/measure/ads",
                     root / "core/config", root / "core/boardSupport/include",
                     root / "components/ads126xAdc/include", root / "components/fdc2214Cap/include",
                     root / "components/tmuxSwitch/include"):
            command += ["-I", str(path)]
        command += [str(generated), str(root / "core/measure/sensorarrayMeasurementMode.c"),
                    str(root / "core/measure/sensorarrayRowModeProfile.c"),
                    str(root / "core/config/sensorarrayScanConfig.c"),
                    str(root / "core/measure/ads/sensorarrayAdsFault.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
