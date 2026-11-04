#!/usr/bin/env python3
"""Execute real battery admission/compare/logging with injected register reads."""
from pathlib import Path
import subprocess
import tempfile

from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "core/measure/ads/sensorarrayAdsGap.c").read_text()
    harness = (root / "tests/host/test_battery_snapshot_evidence.c").read_text()
    definitions = []
    for name in ("sensorarrayAdsBatteryCompare_t", "sensorarrayAdsBatteryDiag_t"):
        end = source.index("} " + name + ";") + len("} " + name + ";")
        definitions.append(source[source.rindex("typedef struct {", 0, end):end])
    harness = harness.replace("/* INSERT_PRODUCTION_TYPES */", "\n".join(definitions))
    names = (
        "static bool sensorarrayAdsSnapshotFromRegisterCache(",
        "static uint8_t sensorarrayAdsBatteryRegisterMask(",
        "static bool sensorarrayAdsBatterySnapshotsMatch(",
        "static esp_err_t sensorarrayAdsBatterySaveVerifiedState(",
        "static void sensorarrayAdsLogBatteryRegisters(",
        "static const char *sensorarrayAdsBatteryRestoreName(",
        "static void sensorarrayAdsLogBatteryEvidence(",
    )
    harness = harness.replace("/* INSERT_PRODUCTION_FUNCTIONS */", "\n".join(function(source, name) for name in names))
    start = source.index('            printf("BATERR,')
    end = source.index(");", start) + 2
    harness = harness.replace("/* INSERT_PRODUCTION_BATERR */", source[start:end])
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native compiler required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-battery-evidence-") as temporary:
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
        subprocess.run(command + [str(generated), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
