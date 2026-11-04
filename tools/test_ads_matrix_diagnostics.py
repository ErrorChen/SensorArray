#!/usr/bin/env python3
"""Execute the production ADC integrity logger against bounded synthetic sinks."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def production_function(source, name):
    start = source.index("static void " + name + "(")
    body = source.index("{", start)
    depth = 0
    for i in range(body, len(source)):
        depth += (source[i] == "{") - (source[i] == "}")
        if depth == 0:
            return source[start:i + 1]
    raise AssertionError("unclosed production function")


def main():
    root = Path(__file__).resolve().parents[1]
    function = production_function(
        (root / "core/measure/ads/sensorarrayAdsMatrix.c").read_text(),
        "sensorarrayAdsMatrixLogCrcEvidence")
    prefix = r"""
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "ads126xAdc.h"
#define SENSORARRAY_MATRIX_COLS 8u
typedef struct { ads126xAdcHandle_t ads; } state_t;
typedef struct {
    state_t *state;
    unsigned mode;
    uint32_t frameCrcErrors, frameCrcRetries, frameCrcRecovered;
    esp_err_t frameCrcTerminalError;
    uint64_t lastCrcEmitUs;
    ads126xAdcCrcFault_t frameFirstCrc;
    uint8_t frameFirstCrcCell;
} sensorarrayAdsMatrixEngine_t;
static int64_t nowUs = 1000000;
static unsigned emitted;
static size_t maximumLength;
static char records[4][384];
static int64_t esp_timer_get_time(void) { return nowUs; }
static const char *sensorarrayMeasurementModeName(unsigned mode) { (void)mode; return "RES"; }
static int sensorarrayAcqEventPrintf(const char *format, ...) {
    assert(emitted < 4u);
    va_list args; va_start(args, format);
    int count = vsnprintf(records[emitted], sizeof(records[emitted]), format, args);
    va_end(args);
    assert(count > 0 && (size_t)count < sizeof(records[emitted]));
    if ((size_t)count > maximumLength) { maximumLength = (size_t)count; }
    ++emitted; return count;
}
"""
    suffix = r"""
int main(void) {
    state_t state = {.ads.adc1CrcErrorCount=UINT32_MAX, .ads.adc1CrcRetryCount=UINT32_MAX};
    sensorarrayAdsMatrixEngine_t engine = {.state=&state,
        .frameCrcErrors=UINT32_MAX, .frameCrcRetries=UINT32_MAX,
        .frameCrcRecovered=UINT32_MAX, .frameFirstCrcCell=63u};
    engine.frameFirstCrc = (ads126xAdcCrcFault_t){.length=6u,
        .bytes={0x40u,0x12u,0x34u,0x56u,0x78u,0x20u},
        .received=0x20u,.computed=0x21u,.crcMode=2u,.statusEnabled=true,
        .configurationGeneration=UINT32_MAX,.drdyGeneration=UINT32_MAX,
        .timestampUs=UINT64_MAX};
    sensorarrayAdsMatrixLogCrcEvidence(&engine, UINT32_MAX, ESP_OK);
    assert(emitted == 2u);
    assert(strstr(records[0], "firstS=8,firstD=8,firstError=0x109,terminalError=0x0") != NULL);
    assert(strstr(records[0], "crcTotal=4294967295,retryTotal=4294967295") != NULL);
    assert(strstr(records[1], "bytes=401234567820,len=6,received=20,computed=21") != NULL);
    nowUs += 1;
    sensorarrayAdsMatrixLogCrcEvidence(&engine, UINT32_MAX, ESP_OK);
    assert(emitted == 2u); /* repeated recovered text is rate limited */
    engine.frameCrcTerminalError = ESP_ERR_TIMEOUT;
    sensorarrayAdsMatrixLogCrcEvidence(&engine, UINT32_MAX, ESP_ERR_INVALID_CRC);
    assert(emitted == 4u); /* an unrecovered error is always reported */
    assert(strstr(records[2], "terminalError=0x107,frameError=0x109") != NULL);
    printf("ADS_MATRIX_DIAGNOSTICS,passed=1,maxRecordBytes=%zu,fixture=synthetic,producer=production_function\n", maximumLength);
    return 0;
}
"""
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    with tempfile.TemporaryDirectory(prefix="sensorarray-ads-matrix-diag-") as temp:
        source = Path(temp) / "test.c"
        source.write_text(prefix + function + suffix)
        binary = Path(temp) / "test"
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(root / "tests/host/ads_stubs"),
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "components/ads126xAdc/include"),
            str(source), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
