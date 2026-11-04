#!/usr/bin/env python3
"""Execute production rail acquisition/error branch with bounded synthetic inputs."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "core/measure/ads/sensorarrayAdsMatrix.c").read_text()
    get_rail = function(source, "static esp_err_t sensorarrayAdsMatrixGetRail(")
    start = source.index("    engine->faultContext.precisionFrame = frame->measurement.precisionFrame;")
    condition = source.index("    if (railError != ESP_OK) {", start)
    branch = source[start:condition] + function(source, "    if (railError != ESP_OK) {")
    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#define CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES 100u
#define SENSORARRAY_MEASUREMENT_MODE_VOLTAGE 2u
#define SENSORARRAY_MEASUREMENT_MODE_RESISTANCE 3u
#define SENSORARRAY_MATRIX_COLS 8u
#define SENSORARRAY_CELL_ERROR_RAIL_INVALID 13u
#define SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR 4u
typedef struct { bool valid; uint32_t ageFrames; } sensorarrayAdsRailSplit_t;
typedef struct { uint32_t railAgeFrames; bool railValid; } sensorarrayAdsGapSnapshot_t;
typedef struct { uint32_t id; esp_err_t originalError,cleanupError; } sensorarrayAdsTransactionEvidence_t;
typedef struct {
    uint32_t frameSequenceHint, frameCount;
    unsigned mode;
    int stateStorage, routeStorage, profileCache, valueCache, railFingerprint;
    int *state,*routeController;
    struct { bool precisionFrame, valid; esp_err_t originalError,cleanupError; uint8_t reg; } faultContext;
} sensorarrayAdsMatrixEngine_t;
typedef struct { bool auxiliaryTransactionsAllowed; } sensorarrayScanPlan_t;
typedef struct { uint8_t activeRows; struct { bool precisionFrame; } measurement;
    sensorarrayAdsGapSnapshot_t adsGap; } sensorarrayFrame_t;
static bool validRail, refreshSucceeds, beginTransaction;
static uint32_t railAge, receivedSequence;
static unsigned refreshCalls,safeCalls,invalidCells,finishCalls;
static esp_err_t refreshError,restoreError,safeError;
static sensorarrayAdsTransactionEvidence_t transaction;
static void sensorarrayAdsGapCopySnapshot(sensorarrayAdsGapSnapshot_t *out,uint32_t sequence)
{ receivedSequence=sequence; *out=(sensorarrayAdsGapSnapshot_t){.railAgeFrames=railAge,.railValid=validRail}; }
static bool sensorarrayAdsGapCopyRailSplit(uint32_t sequence,uint32_t max,sensorarrayAdsRailSplit_t *out)
{ receivedSequence=sequence; assert(max==100u); out->ageFrames=railAge;
  out->valid=validRail && railAge<=max; return out->valid; }
static void sensorarrayAdsGapCopyTransactionEvidence(sensorarrayAdsTransactionEvidence_t *rail,void *battery,void *check)
{ assert(!battery && !check); *rail=transaction; }
static esp_err_t sensorarrayAdsGapRefreshRailAtBoundary(int *state,uint32_t sequence)
{ assert(state); receivedSequence=sequence; refreshCalls++;
  if(beginTransaction) {transaction.id++;transaction.originalError=refreshError;transaction.cleanupError=restoreError;}
  if(refreshSucceeds) {validRail=true;railAge=0u;} return refreshError; }
static void sensorarrayAdsProfileCacheInvalidate(int *cache) {(void)cache;}
static void sensorarrayAdsValueCacheInvalidate(int *cache) {(void)cache;}
static void sensorarrayAdsRailFingerprintInit(int *cache) {(void)cache;}
static void sensorarrayAdsMatrixSetCellError(sensorarrayFrame_t *frame,size_t index,unsigned error,bool fresh)
{ assert(index<(size_t)frame->activeRows*8u && error==SENSORARRAY_CELL_ERROR_RAIL_INVALID && !fresh);invalidCells++; }
static void sensorarrayAdsMatrixCaptureFault(sensorarrayAdsMatrixEngine_t *engine,unsigned stage,
    esp_err_t original,uint8_t reg,uint8_t expected,uint8_t actual,uint8_t mask)
{ assert(stage==SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR && !expected && !actual && !mask);
  engine->faultContext.valid=true;engine->faultContext.originalError=original;engine->faultContext.reg=reg; }
static esp_err_t sensorarrayRouteControllerEnterSafe(int *route,const char *reason)
{ assert(route && !strcmp(reason,"rail_invalid"));safeCalls++;return safeError; }
static void sensorarrayAdsMatrixFinishFrame(sensorarrayFrame_t *frame,uint8_t rows,int64_t start,bool coherent)
{ assert(frame && rows==8u && start==1000 && !coherent);finishCalls++; }
'''
    wrapper = '\nstatic esp_err_t runBranch(sensorarrayAdsMatrixEngine_t *engine, sensorarrayFrame_t *frame, const sensorarrayScanPlan_t *plan) {\nint64_t frameStartUs=1000;\n' + branch + '\nreturn ESP_OK;\n}\n'
    suffix = r'''
static sensorarrayAdsMatrixEngine_t reset(void) {
    validRail=true;railAge=101u;refreshSucceeds=false;beginTransaction=true;
    refreshCalls=safeCalls=invalidCells=finishCalls=0u;
    refreshError=ESP_ERR_INVALID_CRC;restoreError=safeError=ESP_OK;
    transaction=(sensorarrayAdsTransactionEvidence_t){.id=35u};
    static int state,route;
    return (sensorarrayAdsMatrixEngine_t){.frameSequenceHint=4145u,.frameCount=3536u,
        .mode=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,.state=&state,.routeController=&route};
}
int main(void) {
    sensorarrayFrame_t frame={.activeRows=8u};
    sensorarrayScanPlan_t plan={.auxiliaryTransactionsAllowed=true};
    sensorarrayAdsMatrixEngine_t engine=reset();
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_CRC);
    assert(engine.faultContext.originalError==ESP_ERR_INVALID_CRC && engine.faultContext.cleanupError==ESP_OK);
    assert(refreshCalls==1u && safeCalls==1u && invalidCells==64u && finishCalls==1u);
    assert(receivedSequence==4145u && engine.faultContext.reg==UINT8_MAX);

    engine=reset();restoreError=ESP_ERR_TIMEOUT;safeError=ESP_ERR_INVALID_STATE;
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_CRC);
    assert(engine.faultContext.originalError==ESP_ERR_INVALID_CRC && engine.faultContext.cleanupError==ESP_ERR_TIMEOUT);
    assert(transaction.originalError==ESP_ERR_INVALID_CRC && transaction.cleanupError==ESP_ERR_TIMEOUT);

    engine=reset();safeError=ESP_FAIL;
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_CRC);
    assert(engine.faultContext.cleanupError==ESP_FAIL);

    engine=reset();beginTransaction=false;refreshError=ESP_ERR_INVALID_STATE;
    transaction.cleanupError=ESP_FAIL; /* old evidence must not attach to this failed admission */
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_STATE);
    assert(engine.faultContext.cleanupError==ESP_OK);

    engine=reset();railAge=100u;
    assert(runBranch(&engine,&frame,&plan)==ESP_OK);
    assert(refreshCalls==0u && safeCalls==0u && frame.adsGap.railAgeFrames==100u);

    engine=reset();refreshSucceeds=true;refreshError=ESP_OK;
    assert(runBranch(&engine,&frame,&plan)==ESP_OK);
    assert(refreshCalls==1u && safeCalls==0u && frame.adsGap.railAgeFrames==0u);

    engine=reset();refreshError=ESP_OK; /* nominal success without a legal value is not data */
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_STATE);
    assert(safeCalls==1u && invalidCells==64u);

    engine=reset();engine.mode=SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_STATE);
    assert(refreshCalls==0u);

    engine=reset();plan.auxiliaryTransactionsAllowed=false;
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_STATE && refreshCalls==0u);
    engine=reset();railAge=99u;
    assert(runBranch(&engine,&frame,&plan)==ESP_OK && refreshCalls==0u);

    engine=reset();plan.auxiliaryTransactionsAllowed=true;transaction.id=UINT32_MAX;restoreError=ESP_ERR_TIMEOUT;
    assert(runBranch(&engine,&frame,&plan)==ESP_ERR_INVALID_CRC);
    assert(transaction.id==0u && engine.faultContext.cleanupError==ESP_ERR_TIMEOUT);
    puts("ADS_RAIL_ERROR_CHAIN,passed=11,source=production_getrail_and_failure_branch,fixture=synthetic");
    return 0;
}
'''
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise RuntimeError("native C compiler required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-rail-error-") as directory:
        temporary = Path(directory)
        generated = temporary / "test.c"
        generated.write_text(prefix + get_rail + wrapper + suffix)
        executable = temporary / "test"
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(root / "tests/host/include"), str(generated),
                        "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
