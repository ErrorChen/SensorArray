#!/usr/bin/env python3
"""Run actual ADC1 scientific transport/physical-proof code on a fake bus."""
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
        raise RuntimeError("native C compiler required")
    board = (root / "core/board/sensorarrayBringup.c").read_text()
    bodies = "\n".join(function(board, name) for name in (
        "static bool sensorarrayAdsDedicatedSpiOwner(", "static esp_err_t sensorarrayInitSpi("))
    fixture = (root / "tests/host/test_ads_tail_diagnostic.c").read_text()
    fixture = fixture.replace('"test_ads_driver_reliability.c"', '"driver_bus.h"')
    fixture = fixture.replace("/* PRODUCTION_BOARD_OWNERSHIP */", bodies)
    fixture = fixture.replace("int main(void)", "int originalTailTests(void)")
    fixture += "\ntypedef struct { spi_device_handle_t spiDevice; ads126xAdcHandle_t ads; " \
        "bool adsAdc1Running, adsRefReady, adsRefMuxValid; } sensorarrayState_t;\n"
    fixture += "\n".join(function(board, name) for name in (
        "static gpio_num_t sensorarrayToGpio(", "esp_err_t sensorarrayBringupInitAds(",
        "esp_err_t sensorarrayBringupAttachAdsNoReset("))
    gap = (root / "core/measure/ads/sensorarrayAdsGap.c").read_text()
    fixture += "\n".join(function(gap, name) for name in (
        "static bool sensorarrayAdsCoreRegistersMatch(",
        "static esp_err_t sensorarrayAdsRestoreBatteryCoreState("))
    rail_block = function(gap, "if (railDiag.crcErrors != 0u)")
    fixture += '''
#include <stdarg.h>
static char railOutput[512];
static int captureRailPrintf(const char *format, ...) {
    va_list args; va_start(args, format);
    int n = vsnprintf(railOutput, sizeof(railOutput), format, args);
    va_end(args); assert(n > 0 && (size_t)n < sizeof(railOutput)); return n;
}
static void actualRailLog(uint32_t frameSequence, uint32_t txn, const ads126xAdcCrcFault_t *fault) {
    struct { unsigned crcErrors; ads126xAdcCrcFault_t firstCrc; } railDiag = {1u, *fault};
    struct {uint32_t id;} s_railTransaction = {txn};
#define printf captureRailPrintf
''' + rail_block + '''
#undef printf
}
'''
    fixture += (root / "tests/host/test_ads_scientific_read.c").read_text()
    with tempfile.TemporaryDirectory(prefix="sensorarray-scientific-read-") as directory:
        p = Path(directory)
        (p / "production_ads.c").write_text((root / "components/ads126xAdc/ads126xAdc.c").read_text())
        (p / "driver_bus.h").write_text((root / "tests/host/test_ads_driver_reliability.c").read_text().replace(
            '"../../components/ads126xAdc/ads126xAdc.c"', '"production_ads.c"'))
        (p / "test.c").write_text(fixture)
        cmd = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-DCONFIG_IDF_TARGET_ESP32S3=1", "-DCONFIG_BOARD_SPI_HOST=2",
            "-DCONFIG_BOARD_SPI_SCLK_GPIO=47", "-DCONFIG_BOARD_SPI_MOSI_GPIO=21",
            "-DCONFIG_BOARD_SPI_MISO_GPIO=14", "-DCONFIG_BOARD_ADS126X_CS_GPIO=-1",
            "-DCONFIG_BOARD_ADS126X_DRDY_GPIO=13", "-DCONFIG_BOARD_ADS126X_RESET_GPIO=38",
            "-DCONFIG_SENSORARRAY_ADS1262=1", "-DCONFIG_SENSORARRAY_ADS_DATA_RATE=15",
            "-DCONFIG_SENSORARRAY_SPI_USE_DMA=1", "-DCONFIG_ADS126X_SPI_CLOCK_HZ=2000000",
            "-DCONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE=1",
            "-DCONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES=64"]
        for path in (p, root / "tests/host/ads_stubs", root / "tests/host/include",
                     root / "components/ads126xAdc/include"):
            cmd += ["-I", str(path)]
        for enabled in (0, 1):
            subprocess.run(cmd + [f"-DCONFIG_SENSORARRAY_ADS_GPIO_TAIL_READ_ENABLE={enabled}",
                str(p / "test.c"), "-o", str(p / "test")], check=True)
            subprocess.run([str(p / "test")], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
