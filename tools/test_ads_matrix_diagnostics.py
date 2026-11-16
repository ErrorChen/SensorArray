#!/usr/bin/env python3
"""Execute the production ADC integrity logger against bounded synthetic sinks."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
from test_main_config_commit import function as extract_function
from analyze_adc_read_provenance import decode_record


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
    driver = (root / "components/ads126xAdc/ads126xAdc.c").read_text()
    helpers = "\n".join(extract_function(driver, name) for name in
        ("const char *ads126xAdcReadSourceName(", "const char *ads126xAdcTailStageName("))
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
static char records[12][384];
static int64_t esp_timer_get_time(void) { return nowUs; }
static const char *sensorarrayMeasurementModeName(unsigned mode) { (void)mode; return "RES"; }
static int sensorarrayAcqEventPrintf(const char *format, ...) {
    assert(emitted < 12u);
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
        .bytes={0x68u,0xC0u,0x7Bu,0x73u,0xC2u,0xFDu},
        .received=0xFDu,.computed=0xFCu,.crcMode=2u,.statusEnabled=true,
        .readSource=ADS126X_READ_SOURCE_SPI56,.wireKnown=true,
        .configurationGeneration=UINT32_MAX,.drdyGeneration=UINT32_MAX,
        .timestampUs=UINT64_MAX};
    sensorarrayAdsMatrixLogCrcEvidence(&engine, UINT32_MAX, ESP_OK);
    assert(emitted == 2u);
    assert(strstr(records[0], "firstS=8,firstD=8,firstError=0x109,terminalError=0x0") != NULL);
    assert(strstr(records[0], "crcTotal=4294967295,retryTotal=4294967295") != NULL);
    assert(strstr(records[1], "bytes=68C07B73C2FD,len=6,received=FD,computed=FC") != NULL);
    assert(strstr(records[1], "source=spi56,wireKnown=1") != NULL);
    fputs(records[1],stdout); /* Synthetic bytes formatted by the real producer. */
    nowUs += 1;
    sensorarrayAdsMatrixLogCrcEvidence(&engine, UINT32_MAX, ESP_OK);
    assert(emitted == 2u); /* repeated recovered text is rate limited */
    engine.frameCrcTerminalError = ESP_ERR_TIMEOUT;
    sensorarrayAdsMatrixLogCrcEvidence(&engine, UINT32_MAX, ESP_ERR_INVALID_CRC);
    assert(emitted == 4u); /* an unrecovered error is always reported */
    assert(strstr(records[2], "terminalError=0x107,frameError=0x109") != NULL);
    engine.frameFirstCrc.readSource=ADS126X_READ_SOURCE_SPI55_GPIO1;
    engine.frameFirstCrc.wireKnown=false;engine.frameFirstCrc.gpioTailKnown=true;
    engine.frameFirstCrc.gpioTailBit=1u;engine.frameFirstCrc.tailStage=ADS126X_TAIL_STAGE_COMPLETE;
    const uint8_t partial[7]={0x00u,0x68u,0xC0u,0x7Bu,0x73u,0xC2u,0xFCu};
    memcpy(engine.frameFirstCrc.spiPrefix,partial,sizeof(partial));
    sensorarrayAdsMatrixLogCrcEvidence(&engine,UINT32_MAX,ESP_ERR_INVALID_CRC);
    assert(emitted==6u);
    assert(strstr(records[5],"bytes=unknown,len=6,received=FD,computed=FC"));
    assert(strstr(records[5],"source=spi55_gpio1,wireKnown=0,assembledBytes=68C07B73C2FD"));
    assert(strstr(records[5],"spiPrefix=0068C07B73C2FC,gpioKnown=1,gpioBit=1,tailStage=complete"));
    fputs(records[5],stdout);
    engine.frameFirstCrc.gpioTailKnown=false;engine.frameFirstCrc.gpioTailBit=0;
    engine.frameFirstCrc.tailStage=ADS126X_TAIL_STAGE_CLOCK_AFTER_SPI; /* Incomplete metadata/capacity only. */
    sensorarrayAdsMatrixLogCrcEvidence(&engine,UINT32_MAX,ESP_ERR_INVALID_CRC);
    assert(emitted==8u);
    assert(strstr(records[7],"assembledBytes=unknown,spiPrefix=unknown,gpioKnown=0,gpioBit=unknown"));
    assert(!strstr(records[7],"68C07B73C2"));
    engine.frameFirstCrc.readSource=UINT8_MAX;engine.frameFirstCrc.wireKnown=true;
    sensorarrayAdsMatrixLogCrcEvidence(&engine,UINT32_MAX,ESP_ERR_INVALID_CRC);
    assert(emitted==10u&&strstr(records[9],"bytes=unknown")&&strstr(records[9],"source=unknown,wireKnown=0"));
    engine.frameCrcErrors=0u;
    sensorarrayAdsMatrixLogCrcEvidence(&engine,UINT32_MAX,ESP_OK);
    assert(emitted==10u);
    printf("ADS_MATRIX_DIAGNOSTICS,passed=5,maxRecordBytes=%zu,fixture=synthetic,producer=production_function\n", maximumLength);
    return 0;
}
"""
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    with tempfile.TemporaryDirectory(prefix="sensorarray-ads-matrix-diag-") as temp:
        source = Path(temp) / "test.c"
        source.write_text(prefix + helpers + function + suffix)
        binary = Path(temp) / "test"
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(root / "tests/host/ads_stubs"),
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "components/ads126xAdc/include"),
            str(source), "-o", str(binary)], check=True)
        completed = subprocess.run([str(binary)], check=True, capture_output=True)
        decoded = [decode_record(line) for line in completed.stdout.splitlines(keepends=True)
                   if line.startswith(b"ADSSPIBYTES,")]
        assert len(decoded) == 2 and all(item["xor"] == "01" for item in decoded)
        assert decoded[0]["source"] == "spi56" and decoded[0]["wire_observed"]
        assert decoded[1]["source"] == "spi55_gpio1" and not decoded[1]["wire_observed"]
        print(completed.stdout.decode(), end="")
        print("ADS_MATRIX_PROVENANCE,actual_formatter=1,independent_CRC=PASS,records=2,fixture=synthetic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
