#!/usr/bin/env python3
"""Execute the production battery classification against stale-rail admission."""
from pathlib import Path
import subprocess
import tempfile

from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "core/measure/ads/sensorarrayAdsGap.c").read_text()
    harness = (root / "tests/host/test_battery_validity.c").read_text()
    definitions = []
    for name in ("sensorarrayAdsRailCalibration_t", "sensorarrayAdsBatteryCompare_t", "sensorarrayAdsBatteryDiag_t"):
        end = source.index("} " + name + ";") + len("} " + name + ";")
        start = source.rindex("typedef struct {", 0, end)
        definitions.append(source[start:end])
    harness = harness.replace("/* INSERT_PRODUCTION_TYPES */", "\n".join(definitions))
    for key, name in (("AINCOM", "static bool sensorarrayAdsCalculateAincomGnd("),
                      ("REASON", "static sensorarrayBatteryInvalidReason_t sensorarrayAdsBatteryMathReason("),
                      ("VALIDITY", "static void sensorarrayAdsUpdateBatteryValidity(")):
        harness = harness.replace(f"/* INSERT_PRODUCTION_{key} */", function(source, name))
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native compiler required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-battery-validity-") as temporary:
        temp = Path(temporary)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        # Use production configuration fallbacks, not the unrelated ACK harness config.
        (fake / "sensorarrayConfig.h").unlink()
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
        command += [str(generated), str(root / "core/measure/ads/sensorarrayAdsMath.c"),
                    "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
