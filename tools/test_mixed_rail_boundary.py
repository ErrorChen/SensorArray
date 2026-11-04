#!/usr/bin/env python3
"""Run the complete production mixed engine with synthetic hardware boundaries."""
from pathlib import Path
import subprocess
import tempfile
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders
from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native compiler required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-mixed-boundary-") as temporary:
        temp = Path(temporary)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        (fake / "esp_timer.h").write_text("#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n")
        matrix = (root / "core/measure/ads/sensorarrayAdsMatrix.c").read_text()
        boundary = temp / "boundary.c"
        boundary.write_text('#include <string.h>\n#include "sensorarrayAdsMatrix.h"\n#include "sensorarrayAdsGap.h"\n' +
            "\n".join(function(matrix, name) for name in (
                "static uint32_t sensorarrayAdsMatrixConfigurationGeneration(",
                "static void sensorarrayAdsMatrixCaptureFault(",
                "void sensorarrayAdsMatrixEngineRecordMixedBoundaryFault(")))
        executable = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                   "-DCONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES=100",
                   "-DCONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL=128"]
        for path in (fake, root / "tests/host/frame_stubs", root / "tests/host/include",
                     root / "main", root / "main/output", root / "core/measure/include",
                     root / "core/measure/ads", root / "core/measure/fdc", root / "core/measure/mixed",
                     root / "core/config", root / "core/board", root / "core/boardSupport/include",
                     root / "components/ads126xAdc/include", root / "components/fdc2214Cap/include",
                     root / "components/tmuxSwitch/include"):
            command += ["-I", str(path)]
        command += [str(root / "tests/host/test_mixed_rail_boundary.c"),
                    str(boundary), str(root / "core/measure/ads/sensorarrayAdsFault.c"),
                    str(root / "core/measure/mixed/sensorarrayMixedRow.c"),
                    str(root / "core/measure/sensorarrayFrameBuilder.c"),
                    str(root / "core/measure/sensorarrayScanPlan.c"),
                    str(root / "core/measure/sensorarrayMeasurementMode.c"),
                    "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
