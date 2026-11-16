#!/usr/bin/env python3
"""Compile the exact production presence block and driver with DMA fault injection."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    cc = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        raise RuntimeError("host C compiler required")
    source = (root / "core/board/sensorarrayBringup.c").read_text()
    begin = source.index("/* ADS PRESENCE IMPLEMENTATION BEGIN")
    end = source.index("/* ADS PRESENCE IMPLEMENTATION END */")
    presence_source = source[begin:end]
    # The board-only TAIL capability is adjacent to the shared presence slot,
    # but is exercised with actual sensorarrayInitSpi by test_ads_tail_diagnostic.
    # Keep this compilation unit scoped to the unchanged presence functions.
    owner = function(source, "static bool sensorarrayAdsDedicatedSpiOwner(")
    presence_source = presence_source.replace(owner, "").replace(
        "static spi_device_handle_t s_adsDedicatedSpiDevice;", "")
    header = (root / "core/board/sensorarrayBringup.h").read_text()
    result_end = header.index("} sensorarrayAdsPresenceResult_t;") + len("} sensorarrayAdsPresenceResult_t;")
    result_start = header.rfind("typedef struct {", 0, result_end)
    with tempfile.TemporaryDirectory(prefix="sensorarray-ads-presence-") as temp:
        directory = Path(temp)
        (directory / "sensorarray_ads_presence_impl.inc").write_text(
            'typedef struct { ads126xAdcHandle_t ads; bool adsReady; '
            'spi_device_handle_t spiDevice; } sensorarrayState_t;\n' +
            header[result_start:result_end] + "\n" + presence_source
        )
        binary = directory / "test_ads_presence"
        subprocess.run([
            cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-I", temp, "-I", str(root / "tests/host/ads_stubs"),
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "components/ads126xAdc/include"),
            str(root / "tests/host/test_ads_presence.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
