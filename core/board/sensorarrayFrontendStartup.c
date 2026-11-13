#include "sensorarrayFrontendStartup.h"

#include <stdio.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "boardSupport.h"
#include "sensorarrayAcqEvent.h"
#include "sensorarrayAdsSpiProbe.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayLog.h"

#define printf sensorarrayAcqEventPrintf

/* 本轮新增: one boot-only ADS owner.  The job, result and state reference stay
 * alive even if the coordinator's deadline expires.  Query readers only see
 * published, mutex-protected values, never the worker's partial result. */
typedef enum {
    SENSORARRAY_BOOT_SPI_IDLE = 0,
    SENSORARRAY_BOOT_SPI_POSTED,
    SENSORARRAY_BOOT_SPI_RUNNING,
    SENSORARRAY_BOOT_SPI_DONE,
    SENSORARRAY_BOOT_SPI_CREATE_FAILED,
    SENSORARRAY_BOOT_SPI_SKIPPED,
} sensorarrayBootSpiPhase_t;

typedef struct {
    sensorarrayBootSpiPhase_t phase;
    bool timedOut;
    bool joined;
    esp_err_t runError;
    esp_err_t joinError;
    uint64_t postedUs;
    uint64_t deadlineUs;
    uint64_t startedUs;
    uint64_t endedUs;
    uint64_t i2cStartedUs;
    uint64_t i2cEndedUs;
    uint64_t joinedUs;
    uint64_t overlapUs;
    int32_t spiCore;
    int32_t i2cCore;
    uint32_t stackMinBytes;
    sensorarrayAdsSpiProbeResult_t result;
} sensorarrayBootSpiSnapshot_t;

typedef struct {
    sensorarrayState_t *state;
    TaskHandle_t task;
    sensorarrayAdsSpiProbeResult_t workResult;
    sensorarrayBootSpiSnapshot_t published;
} sensorarrayBootSpiJob_t;

static portMUX_TYPE s_bootSpiMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayBootSpiJob_t s_bootSpiJob;

static const char *sensorarrayBootSpiPhaseName(sensorarrayBootSpiPhase_t phase)
{
    switch (phase) {
    case SENSORARRAY_BOOT_SPI_POSTED: return "posted";
    case SENSORARRAY_BOOT_SPI_RUNNING: return "running";
    case SENSORARRAY_BOOT_SPI_DONE: return "done";
    case SENSORARRAY_BOOT_SPI_CREATE_FAILED: return "create_failed";
    case SENSORARRAY_BOOT_SPI_SKIPPED: return "skipped";
    default: return "idle";
    }
}

static sensorarrayBootSpiSnapshot_t sensorarrayBootSpiCopy(void)
{
    sensorarrayBootSpiSnapshot_t snapshot;
    portENTER_CRITICAL(&s_bootSpiMux);
    snapshot = s_bootSpiJob.published;
    portEXIT_CRITICAL(&s_bootSpiMux);
    return snapshot;
}

#if CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
static void sensorarrayBootSpiTask(void *arg)
{
    sensorarrayBootSpiJob_t *job = (sensorarrayBootSpiJob_t *)arg;
    uint64_t startedUs = (uint64_t)esp_timer_get_time();
    int32_t core = (int32_t)xPortGetCoreID();
    portENTER_CRITICAL(&s_bootSpiMux);
    job->published.phase = SENSORARRAY_BOOT_SPI_RUNNING;
    job->published.startedUs = startedUs;
    job->published.spiCore = core;
    sensorarrayState_t *state = job->state;
    uint64_t deadlineUs = job->published.deadlineUs;
    portEXIT_CRITICAL(&s_bootSpiMux);

    esp_err_t err = sensorarrayAdsSpiProbeRun(state, deadlineUs,
                                             &job->workResult);
    uint64_t endedUs = (uint64_t)esp_timer_get_time();
    uint32_t stackMinBytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    portENTER_CRITICAL(&s_bootSpiMux);
    job->published.result = job->workResult;
    job->published.runError = err;
    job->published.endedUs = endedUs;
    job->published.stackMinBytes = stackMinBytes;
    job->published.phase = SENSORARRAY_BOOT_SPI_DONE;
    /* A previous join timeout remains terminal even after a late DONE. */
    portEXIT_CRITICAL(&s_bootSpiMux);

    /* DRDY ISR can already have loaded this task handle on another core.
     * Keep the one-shot task and its configured stack for the entire boot;
     * never delete it or reuse it as another ADS owner. */
    for (;;) {
        vTaskSuspend(NULL);
    }
}
#endif

static esp_err_t sensorarrayBootSpiStart(sensorarrayState_t *state)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t postedUs = (uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&s_bootSpiMux);
    if (s_bootSpiJob.published.phase != SENSORARRAY_BOOT_SPI_IDLE) {
        portEXIT_CRITICAL(&s_bootSpiMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_bootSpiJob.state = state;
    s_bootSpiJob.published = (sensorarrayBootSpiSnapshot_t) {
        .phase = SENSORARRAY_BOOT_SPI_POSTED,
        .postedUs = postedUs,
        .deadlineUs = postedUs +
            (uint64_t)CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_BUDGET_MS * 1000u,
        .spiCore = -1,
        .i2cCore = -1,
    };
#if !CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
    s_bootSpiJob.published.phase = SENSORARRAY_BOOT_SPI_SKIPPED;
    portEXIT_CRITICAL(&s_bootSpiMux);
    return ESP_OK;
#else
    portEXIT_CRITICAL(&s_bootSpiMux);
    TaskHandle_t task = NULL;
    BaseType_t created = xTaskCreatePinnedToCore(
        sensorarrayBootSpiTask, "ads_spi_boot",
        CONFIG_SENSORARRAY_ADS_SPI_PROBE_TASK_STACK, &s_bootSpiJob,
        8u, &task, CONFIG_SENSORARRAY_MISC_CORE);
    portENTER_CRITICAL(&s_bootSpiMux);
    if (created != pdPASS) {
        s_bootSpiJob.published.phase = SENSORARRAY_BOOT_SPI_CREATE_FAILED;
        s_bootSpiJob.published.runError = ESP_ERR_NO_MEM;
        s_bootSpiJob.published.joinError = ESP_ERR_NO_MEM;
    } else {
        s_bootSpiJob.task = task;
    }
    portEXIT_CRITICAL(&s_bootSpiMux);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}

static void sensorarrayBootSpiNoteI2c(bool starting)
{
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    int32_t core = (int32_t)xPortGetCoreID();
    portENTER_CRITICAL(&s_bootSpiMux);
    if (starting) {
        s_bootSpiJob.published.i2cStartedUs = nowUs;
        s_bootSpiJob.published.i2cCore = core;
    } else {
        s_bootSpiJob.published.i2cEndedUs = nowUs;
    }
    portEXIT_CRITICAL(&s_bootSpiMux);
}

static esp_err_t sensorarrayBootSpiJoin(void)
{
    for (;;) {
        uint64_t nowUs = (uint64_t)esp_timer_get_time();
        portENTER_CRITICAL(&s_bootSpiMux);
        sensorarrayBootSpiSnapshot_t *p = &s_bootSpiJob.published;
        esp_err_t err = ESP_OK;
        /* Observe DONE before comparing the current time: I2C startup may
         * finish later than the SPI deadline after SPI already completed. */
        if (p->timedOut) {
            err = ESP_ERR_TIMEOUT;
        } else if (p->phase == SENSORARRAY_BOOT_SPI_DONE) {
            err = p->runError;
            if (err == ESP_OK && p->endedUs > p->deadlineUs) {
                p->timedOut = true;
                err = ESP_ERR_TIMEOUT;
            } else if (err == ESP_OK &&
                       (!p->result.qualified || !p->result.restored ||
                        p->result.selectedHz == 0u || p->result.actualHz == 0u)) {
                err = ESP_ERR_INVALID_RESPONSE;
            }
        } else if (p->phase == SENSORARRAY_BOOT_SPI_CREATE_FAILED) {
            err = p->runError;
        } else if (p->phase == SENSORARRAY_BOOT_SPI_SKIPPED) {
            err = ESP_OK;
        } else if (p->phase == SENSORARRAY_BOOT_SPI_IDLE) {
            err = ESP_ERR_INVALID_STATE;
        } else if (nowUs >= p->deadlineUs) {
            p->timedOut = true;
            err = ESP_ERR_TIMEOUT;
        } else {
            portEXIT_CRITICAL(&s_bootSpiMux);
            /* One tick rounded by FreeRTOS; never convert a sub-tick wait
             * to zero or confuse a wakeup with worker completion. */
            vTaskDelay(1u);
            continue;
        }
        p->joined = true;
        p->joinedUs = nowUs;
        p->joinError = err;
        uint64_t startUs = p->startedUs > p->i2cStartedUs ?
            p->startedUs : p->i2cStartedUs;
        uint64_t endUs = p->endedUs < p->i2cEndedUs ?
            p->endedUs : p->i2cEndedUs;
        p->overlapUs = p->startedUs && p->i2cStartedUs && endUs > startUs ?
            endUs - startUs : 0u;
        portEXIT_CRITICAL(&s_bootSpiMux);
        return err;
    }
}

esp_err_t sensorarrayFrontendStartupFormatSpi(char *response, size_t responseSize)
{
    sensorarrayBootSpiSnapshot_t s = sensorarrayBootSpiCopy();
    int written = snprintf(response, responseSize,
        "SPI,phase=%s,done=%u,joined=%u,timedOut=%u,qualified=%u,restored=%u,hz=%lu,actualHz=%lu,err=0x%lx,run=0x%lx,cleanup=0x%lx,core=%ld,stackBytes=%u,stackMinBytes=%lu,candidates=%lu,attempted=%lu,samples=%lu,even=%lu,odd=%lu,crc=%lu,status=%lu,reg=%lu,drdy=%lu,startUs=%llu,endUs=%llu,overlapUs=%llu,scope=all_candidates\n",
        sensorarrayBootSpiPhaseName(s.phase),
        s.phase == SENSORARRAY_BOOT_SPI_DONE ? 1u : 0u,
        s.joined ? 1u : 0u, s.timedOut ? 1u : 0u,
        s.result.qualified ? 1u : 0u, s.result.restored ? 1u : 0u,
        (unsigned long)s.result.selectedHz, (unsigned long)s.result.actualHz,
        (unsigned long)(uint32_t)s.joinError,
        (unsigned long)(uint32_t)s.runError,
        (unsigned long)(uint32_t)s.result.cleanupError, (long)s.spiCore,
        s.phase == SENSORARRAY_BOOT_SPI_RUNNING ||
        s.phase == SENSORARRAY_BOOT_SPI_DONE ?
            (unsigned)CONFIG_SENSORARRAY_ADS_SPI_PROBE_TASK_STACK : 0u,
        (unsigned long)s.stackMinBytes, (unsigned long)s.result.candidatesTested,
        (unsigned long)s.result.attemptedSamples,
        (unsigned long)s.result.validSamples, (unsigned long)s.result.crcValidEven,
        (unsigned long)s.result.crcValidOdd, (unsigned long)s.result.crcErrors,
        (unsigned long)s.result.statusErrors, (unsigned long)s.result.registerErrors,
        (unsigned long)s.result.drdyTimeouts,
        (unsigned long long)s.startedUs, (unsigned long long)s.endedUs,
        (unsigned long long)s.overlapUs);
    return written > 0 && (size_t)written < responseSize ?
        ESP_OK : ESP_ERR_INVALID_SIZE;
}

static void sensorarrayBootSpiLogJoin(esp_err_t err)
{
    sensorarrayBootSpiSnapshot_t s = sensorarrayBootSpiCopy();
    printf("BOOT_BUS_JOIN,spiPhase=%s,done=%u,timedOut=%u,err=0x%lx,spiCore=%ld,i2cCore=%ld,spiStartUs=%llu,spiEndUs=%llu,i2cStartUs=%llu,i2cEndUs=%llu,joinUs=%llu,deadlineUs=%llu,overlapUs=%llu\n",
        sensorarrayBootSpiPhaseName(s.phase),
        s.phase == SENSORARRAY_BOOT_SPI_DONE ? 1u : 0u,
        s.timedOut ? 1u : 0u, (unsigned long)(uint32_t)err,
        (long)s.spiCore, (long)s.i2cCore,
        (unsigned long long)s.startedUs, (unsigned long long)s.endedUs,
        (unsigned long long)s.i2cStartedUs, (unsigned long long)s.i2cEndedUs,
        (unsigned long long)s.joinedUs, (unsigned long long)s.deadlineUs,
        (unsigned long long)s.overlapUs);
}

static bool sensorarrayAppFdcIdMatches(uint16_t manufacturerId, uint16_t deviceId)
{
    return manufacturerId == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID &&
           deviceId == SENSORARRAY_FDC_EXPECTED_DEVICE_ID;
}

static esp_err_t sensorarrayProbeFdcIdsAtClock(sensorarrayFdcDeviceState_t *fdcState,
                                               uint32_t clockHz,
                                               uint32_t attempts,
                                               uint32_t *outGoodCount,
                                               uint32_t *outBadCount,
                                               uint16_t *outLastManufacturerId,
                                               uint16_t *outLastDeviceId,
                                               const char **outReason)
{
    if (outGoodCount) {
        *outGoodCount = 0u;
    }
    if (outBadCount) {
        *outBadCount = 0u;
    }
    if (outLastManufacturerId) {
        *outLastManufacturerId = 0u;
    }
    if (outLastDeviceId) {
        *outLastDeviceId = 0u;
    }
    if (outReason) {
        *outReason = "unknown";
    }
    if (!fdcState || !fdcState->i2cCtx || clockHz == 0u || attempts == 0u) {
        if (outReason) {
            *outReason = "invalid_arg";
        }
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = boardSupportSetI2cFrequency(fdcState->i2cCtx, clockHz);
    if (err != ESP_OK) {
        if (outReason) {
            *outReason = "bus_reconfigure_failed";
        }
        return err;
    }

    uint32_t goodCount = 0u;
    uint32_t badCount = 0u;
    esp_err_t firstErr = ESP_OK;
    uint16_t lastManufacturerId = 0u;
    uint16_t lastDeviceId = 0u;
    for (uint32_t i = 0u; i < attempts; ++i) {
        uint16_t manufacturerId = 0u;
        uint16_t deviceId = 0u;
        err = sensorarrayBringupReadFdcIdsRaw(fdcState->i2cCtx,
                                              fdcState->i2cAddr,
                                              &manufacturerId,
                                              &deviceId);
        lastManufacturerId = manufacturerId;
        lastDeviceId = deviceId;
        if (err == ESP_OK && sensorarrayAppFdcIdMatches(manufacturerId, deviceId)) {
            goodCount++;
        } else {
            badCount++;
            if (firstErr == ESP_OK) {
                firstErr = (err != ESP_OK) ? err : ESP_FAIL;
            }
        }
    }

    if (outGoodCount) {
        *outGoodCount = goodCount;
    }
    if (outBadCount) {
        *outBadCount = badCount;
    }
    if (outLastManufacturerId) {
        *outLastManufacturerId = lastManufacturerId;
    }
    if (outLastDeviceId) {
        *outLastDeviceId = lastDeviceId;
    }
    if (goodCount == attempts) {
        if (outReason) {
            *outReason = "ok";
        }
        return ESP_OK;
    }
    if (outReason) {
        *outReason = (firstErr == ESP_OK || firstErr == ESP_FAIL) ? "id_mismatch" :
            (firstErr == ESP_ERR_TIMEOUT) ? "timeout" : "id_read_failed";
    }
    return (firstErr != ESP_OK) ? firstErr : ESP_FAIL;
}

void sensorarrayFrontendStartupRecheckFdcClock(sensorarrayFdcDeviceState_t *fdcState,
                                               const char *mapLabel)
{
    if (!fdcState || !fdcState->i2cCtx) {
        return;
    }
    BoardSupportI2cBusInfo_t busInfo = {0};
    bool secondary = fdcState->i2cCtx == boardSupportGetI2c1Ctx();
    (void)boardSupportGetI2cBusInfo(secondary, &busInfo);
    int bus = (int)busInfo.Port;

    if (!CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE) {
        printf("ICLK,stage=disabled,bus=%d,addr=0x%02X,hz=%lu,map=%s\n",
               bus,
               fdcState->i2cAddr,
               (unsigned long)busInfo.FrequencyHz,
               mapLabel ? mapLabel : SENSORARRAY_NA);
        return;
    }

    static const uint32_t clockCandidates[] = {350000u, 337500u, 325000u, 300000u};
    const uint32_t attempts = 8u;
    esp_err_t lastErr = ESP_FAIL;
    const char *lastReason = "not_probed";
    for (size_t i = 0u; i < sizeof(clockCandidates) / sizeof(clockCandidates[0]); ++i) {
        uint32_t hz = clockCandidates[i];
        uint32_t goodCount = 0u;
        uint32_t badCount = 0u;
        uint16_t manufacturerId = 0u;
        uint16_t deviceId = 0u;
        const char *reason = "unknown";
        esp_err_t err = sensorarrayProbeFdcIdsAtClock(fdcState,
                                                      hz,
                                                      attempts,
                                                      &goodCount,
                                                      &badCount,
                                                      &manufacturerId,
                                                      &deviceId,
                                                      &reason);
        printf("ICLK,stage=probe,bus=%d,addr=0x%02X,hz=%lu,attempts=%lu,good=%lu,bad=%lu,err=0x%lx,idMfg=0x%04X,idDev=0x%04X,result=%s,map=%s\n",
               bus,
               fdcState->i2cAddr,
               (unsigned long)hz,
               (unsigned long)attempts,
               (unsigned long)goodCount,
               (unsigned long)badCount,
               (unsigned long)err,
               manufacturerId,
               deviceId,
               (err == ESP_OK) ? "ok" : reason,
               mapLabel ? mapLabel : SENSORARRAY_NA);
        if (err == ESP_OK) {
            printf("ICLK,stage=locked,bus=%d,addr=0x%02X,hz=%lu,attempts=%lu,map=%s\n",
                   bus,
                   fdcState->i2cAddr,
                   (unsigned long)hz,
                   (unsigned long)attempts,
                   mapLabel ? mapLabel : SENSORARRAY_NA);
            if (fdcState->handle) {
                Fdc2214CapCoreRegs_t regs = {0};
                esp_err_t verifyErr = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
                printf("ICLK,stage=verify,bus=%d,addr=0x%02X,hz=%lu,err=0x%lx,status=0x%04X,statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,map=%s\n",
                       bus,
                       fdcState->i2cAddr,
                       (unsigned long)hz,
                       (unsigned long)verifyErr,
                       regs.Status,
                       regs.StatusConfig,
                       regs.Config,
                       regs.MuxConfig,
                       mapLabel ? mapLabel : SENSORARRAY_NA);
            }
            return;
        }
        lastErr = err;
        lastReason = reason;
        if ((i + 1u) < (sizeof(clockCandidates) / sizeof(clockCandidates[0]))) {
            printf("ICLK,stage=fallback,bus=%d,addr=0x%02X,from=%lu,to=%lu,reason=%s,map=%s\n",
                   bus,
                   fdcState->i2cAddr,
                   (unsigned long)hz,
                   (unsigned long)clockCandidates[i + 1u],
                   reason,
                   mapLabel ? mapLabel : SENSORARRAY_NA);
        }
    }

    printf("ICLK,stage=failed,bus=%d,addr=0x%02X,err=0x%lx,reason=%s,action=continue_init_at_last_frequency,map=%s\n",
           bus,
           fdcState->i2cAddr,
           (unsigned long)lastErr,
           lastReason,
           mapLabel ? mapLabel : SENSORARRAY_NA);
}

static void sensorarraySweepFdcI2cClockUnderLoad(sensorarrayFdcDeviceState_t *fdcState,
                                                 const char *mapLabel)
{
    if (!CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE ||
        !fdcState || !fdcState->i2cCtx || !fdcState->handle) {
        return;
    }

    const uint32_t startHz = 400000u;
    const uint32_t minHz = 300000u;
    const uint32_t stepHz = 5000u;
    const uint32_t marginHz = 10000u;
    const uint32_t attempts = 3u;
    uint32_t maxStableHz = 0u;
    BoardSupportI2cBusInfo_t busInfo = {0};
    bool secondary = fdcState->i2cCtx == boardSupportGetI2c1Ctx();
    (void)boardSupportGetI2cBusInfo(secondary, &busInfo);

    for (uint32_t testHz = startHz; testHz >= minHz; testHz -= stepHz) {
        esp_err_t setErr = boardSupportSetI2cFrequency(fdcState->i2cCtx, testHz);
        uint32_t idOk = 0u;
        uint32_t statusOk = 0u;
        uint32_t orderedOk = 0u;
        uint32_t burstOk = 0u;
        Fdc2214CapResetI2cStats(fdcState->handle);
        esp_err_t firstErr = setErr;
        for (uint32_t attempt = 0u; attempt < attempts && setErr == ESP_OK; ++attempt) {
            uint16_t manufacturerId = 0u;
            uint16_t deviceId = 0u;
            esp_err_t err = Fdc2214CapReadId(fdcState->handle,
                                             &manufacturerId,
                                             &deviceId);
            if (err == ESP_OK && sensorarrayAppFdcIdMatches(manufacturerId, deviceId)) {
                idOk++;
            } else if (firstErr == ESP_OK) {
                firstErr = err != ESP_OK ? err : ESP_ERR_INVALID_RESPONSE;
            }

            Fdc2214CapStatus_t status = {0};
            err = Fdc2214CapReadStatus(fdcState->handle, &status);
            if (err == ESP_OK) {
                statusOk++;
            } else if (firstErr == ESP_OK) {
                firstErr = err;
            }

            Fdc2214CapFastChannelSample_t ordered[4] = {0};
            err = Fdc2214CapReadDataOrdered4(fdcState->handle, ordered);
            if (err == ESP_OK) {
                orderedOk++;
            } else if (firstErr == ESP_OK) {
                firstErr = err;
            }

            if (Fdc2214CapDataBurstSupported(fdcState->handle)) {
                Fdc2214CapFastChannelSample_t burst[4] = {0};
                err = Fdc2214CapReadDataBurst4(fdcState->handle, burst);
                if (err == ESP_OK) {
                    burstOk++;
                } else if (firstErr == ESP_OK) {
                    firstErr = err;
                }
            }
        }
        Fdc2214CapI2cStats_t stats = {0};
        Fdc2214CapGetI2cStats(fdcState->handle, &stats);
        bool burstRequired = Fdc2214CapDataBurstSupported(fdcState->handle);
        bool ok = setErr == ESP_OK && firstErr == ESP_OK &&
                  idOk == attempts && statusOk == attempts && orderedOk == attempts &&
                  (!burstRequired || burstOk == attempts) &&
                  stats.nackCount == 0u && stats.timeoutCount == 0u;
        const char *reason = setErr != ESP_OK ? "bus_reconfigure_failed" :
            (idOk != attempts ? "id_failed" :
             statusOk != attempts ? "status_failed" :
             orderedOk != attempts ? "ordered_failed" :
             (burstRequired && burstOk != attempts) ? "burst_failed" :
             stats.nackCount != 0u ? "nack" :
             stats.timeoutCount != 0u ? "timeout" : "ok");
        printf("I2C_SWEEP,bus=%d,testHz=%lu,ok=%u,idOk=%lu,statusOk=%lu,orderedOk=%lu,burstOk=%lu,nack=%lu,timeout=%lu,recover=%lu,reason=%s,map=%s\n",
               (int)busInfo.Port,
               (unsigned long)testHz,
               ok ? 1u : 0u,
               (unsigned long)idOk,
               (unsigned long)statusOk,
               (unsigned long)orderedOk,
               (unsigned long)burstOk,
               (unsigned long)stats.nackCount,
               (unsigned long)stats.timeoutCount,
               (unsigned long)stats.recoveryCount,
               reason,
               mapLabel ? mapLabel : SENSORARRAY_NA);
        if (ok) {
            maxStableHz = testHz;
            break;
        }
        if (testHz == minHz) {
            break;
        }
    }

    uint32_t selectedHz = maxStableHz > (minHz + marginHz) ?
        (maxStableHz - marginHz) : minHz;
    const char *selectedReason = maxStableHz == 0u ?
        "no_stable_workload_use_conservative" :
        (maxStableHz <= minHz ? "stable_only_at_safe_floor" : "max_stable_minus_margin");
    esp_err_t selectedErr = boardSupportSetI2cFrequency(fdcState->i2cCtx, selectedHz);
    printf("I2C_SELECTED,bus=%d,hz=%lu,maxStableHz=%lu,marginKHz=10,err=0x%lx,reason=%s,map=%s\n",
           (int)busInfo.Port,
           (unsigned long)selectedHz,
           (unsigned long)maxStableHz,
           (unsigned long)selectedErr,
           selectedReason,
           mapLabel ? mapLabel : SENSORARRAY_NA);
}

static void sensorarrayInitFdcDevice(sensorarrayFdcDeviceState_t *fdcState,
                                     bool addressValid,
                                     uint8_t requestedChannels,
                                     const char *mapLabel)
{
    if (!fdcState) {
        return;
    }
    if (!addressValid) {
        sensorarrayLogStartupFdc("fdc_init",
                                 fdcState,
                                 ESP_ERR_INVALID_ARG,
                                 "skip_invalid_addr_config",
                                 (int32_t)fdcState->i2cAddr,
                                 false,
                                 0,
                                 0,
                                 mapLabel);
        return;
    }
    if (!fdcState->i2cCtx) {
        sensorarrayLogStartupFdc("fdc_init",
                                 fdcState,
                                 ESP_ERR_NOT_SUPPORTED,
                                 "skip_i2c_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 mapLabel);
        return;
    }

    sensorarrayFrontendStartupRecheckFdcClock(fdcState, mapLabel);
    sensorarrayBringupProbeFdcBus(fdcState);

    sensorarrayFdcInitDiag_t diag = {0};
    esp_err_t err = sensorarrayBringupInitFdcDevice(fdcState->i2cCtx,
                                                    fdcState->i2cAddr,
                                                    requestedChannels,
                                                    &fdcState->handle,
                                                    &diag);
    sensorarrayBringupApplyFdcInitResult(fdcState, fdcState->handle, err, &diag);
    sensorarrayLogStartupFdc("fdc_init",
                             fdcState,
                             err,
                             diag.status,
                             (err == ESP_OK) ? (int32_t)requestedChannels : diag.detail,
                             diag.haveIds,
                             diag.manufacturerId,
                             diag.deviceId,
                             mapLabel);
    if (err == ESP_OK && fdcState->handle) {
        Fdc2214CapBurstProbeResult_t burstProbe = {0};
        esp_err_t burstErr = Fdc2214CapProbeDataBurst4(fdcState->handle,
                                                       5u,
                                                       &burstProbe);
        printf("BURST_PROBE,device=%s,supported=%u,trials=%lu,mismatch=%lu,err=0x%lx,reason=%s,map=%s\n",
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               burstProbe.supported ? 1u : 0u,
               (unsigned long)burstProbe.trials,
               (unsigned long)burstProbe.mismatchCount,
               (unsigned long)burstErr,
               burstProbe.reason ? burstProbe.reason : "unknown",
               mapLabel ? mapLabel : SENSORARRAY_NA);
        sensorarraySweepFdcI2cClockUnderLoad(fdcState, mapLabel);
        Fdc2214CapSequenceProbeResult_t sequenceProbe = {0};
        esp_err_t sequenceErr = ESP_ERR_NOT_SUPPORTED;
        if (CONFIG_SENSORARRAY_FDC_PRECISION_SAFE_SEQUENCE_ENABLE) {
            sequenceErr = Fdc2214CapProbeDataSequence4(fdcState->handle,
                                                       5u,
                                                       &sequenceProbe);
        } else {
            sequenceProbe.selectedMode = FDC_DATA_READ_MODE_ORDERED8;
            sequenceProbe.reason = "config_disabled";
        }
        printf("SEQ_PROBE,device=%s,seqProbeOk=%u,dataReadMode=%s,trials=%lu,fixedMismatch=%lu,dataMismatch=%lu,testedMask=0x%lX,okMask=0x%lX,seqRegsPerTxn=%u,seqTxnPerRow=%u,selectedUs=%lu,err=0x%lx,reason=%s,map=%s\n",
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               sequenceProbe.supported ? 1u : 0u,
               Fdc2214CapDataReadModeName(Fdc2214CapDataReadMode(fdcState->handle)),
               (unsigned long)sequenceProbe.trials,
               (unsigned long)sequenceProbe.fixedRegMismatchCount,
               (unsigned long)sequenceProbe.dataMismatchCount,
               (unsigned long)sequenceProbe.testedModeMask,
               (unsigned long)sequenceProbe.modeOkMask,
               (unsigned)Fdc2214CapDataReadModeRegsPerTransaction(
                   Fdc2214CapDataReadMode(fdcState->handle)),
               (unsigned)Fdc2214CapDataReadModeTransactionsPerRow(
                   Fdc2214CapDataReadMode(fdcState->handle)),
               (unsigned long)sequenceProbe.selectedElapsedUs,
               (unsigned long)sequenceErr,
               sequenceProbe.reason ? sequenceProbe.reason : "unknown",
               mapLabel ? mapLabel : SENSORARRAY_NA);
        uint16_t rcount[4] = {0};
        uint16_t settle[4] = {0};
        uint16_t clockDiv[4] = {0};
        uint16_t drive[4] = {0};
        uint16_t statusConfig = 0u;
        uint16_t config = 0u;
        uint16_t muxConfig = 0u;
        esp_err_t profileErr = ESP_OK;
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            esp_err_t readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                           (uint8_t)(0x08u + ch),
                                                           &rcount[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
            readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 (uint8_t)(0x10u + ch),
                                                 &settle[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
            readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 (uint8_t)(0x14u + ch),
                                                 &clockDiv[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
            readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 (uint8_t)(0x1Eu + ch),
                                                 &drive[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
        }
        (void)Fdc2214CapReadRawRegisters(fdcState->handle, 0x19u, &statusConfig);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle, 0x1Au, &config);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle, 0x1Bu, &muxConfig);
        printf("FDC_PROFILE,device=%s,name=precision_safe_fast,conversionProfile=%s,runtimeRcount=0x%04X,startupRcount=[%04X,%04X,%04X,%04X],settle=[%04X,%04X,%04X,%04X],clockDiv=[%04X,%04X,%04X,%04X],drive=[%04X,%04X,%04X,%04X],config=0x%04X,mux=0x%04X,statusConfig=0x%04X,precisionAffecting=0,err=0x%lx\n",
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               CONFIG_SENSORARRAY_FDC_RUNTIME_PROFILE_NAME,
               (unsigned)CONFIG_SENSORARRAY_FDC_RUNTIME_RCOUNT,
               rcount[0], rcount[1], rcount[2], rcount[3],
               settle[0], settle[1], settle[2], settle[3],
               clockDiv[0], clockDiv[1], clockDiv[2], clockDiv[3],
               drive[0], drive[1], drive[2], drive[3],
               config,
               muxConfig,
               statusConfig,
               (unsigned long)profileErr);
        static bool experimentalProfileLogged = false;
        if (!experimentalProfileLogged) {
            experimentalProfileLogged = true;
            printf("FDC_PROFILE,name=experimental_precision_affecting_profile,precisionAffecting=1,enabled=%u\n",
                   CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE ? 1u : 0u);
        }
    }
}

static void sensorarrayLogFdcParallelCfg(void)
{
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);
    bool primaryAvailable = primaryBus.Enabled && boardSupportGetI2cCtx();
    bool secondaryAvailable = secondaryBus.Enabled && boardSupportGetI2c1Ctx();
    bool sameBus = primaryAvailable &&
                   secondaryAvailable &&
                   primaryBus.Port == secondaryBus.Port;
    bool configEnabled = CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ != 0;
    bool enabled = configEnabled &&
                   primaryAvailable &&
                   secondaryAvailable &&
                   !sameBus;
    const char *reason = !configEnabled ? "config_disabled" :
        (!primaryAvailable || !secondaryAvailable) ? "bus_unavailable" :
        sameBus ? "same_bus" :
        "dual_bus_enabled";
    if (configEnabled && !enabled) {
        printf("FDC_PARALLEL_WARN,reason=config_enabled_but_worker_not_available,detail=%s\n",
               reason);
    }
    printf("FDC_PARALLEL_CFG,enabled=%u,primaryBus=%d,secondaryBus=%d,sameBus=%u,primaryCore=%d,secondaryCore=%d,workerMode=%s,reason=%s,scanCore=%d,logCore=%d,workerPrio=%d,logPrio=%d\n",
           enabled ? 1u : 0u,
           primaryAvailable ? (int)primaryBus.Port : -1,
           secondaryAvailable ? (int)secondaryBus.Port : -1,
           sameBus ? 1u : 0u,
           CONFIG_SENSORARRAY_FDC_PRIMARY_WORKER_TASK_CORE,
           CONFIG_SENSORARRAY_FDC_SECONDARY_WORKER_TASK_CORE,
           enabled ? "parallel" : "serial",
           reason,
           CONFIG_SENSORARRAY_SCAN_TASK_CORE,
           CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE,
           CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
           CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY);
}

esp_err_t sensorarrayFrontendStartupRun(sensorarrayState_t *state,
    bool primaryAddressValid, bool secondaryAddressValid, uint8_t channels)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Retain known bus contexts even if a later bus goes offline. */
    if (state->boardReady) {
        if (!state->fdcPrimary.i2cCtx) {
            state->fdcPrimary.i2cCtx = boardSupportGetI2cCtx();
        }
        if (!state->fdcSecondary.i2cCtx) {
            state->fdcSecondary.i2cCtx = boardSupportGetI2c1Ctx();
        }
    }
    esp_err_t err = sensorarrayBringupInitAds(state);
    state->adsReady = (err == ESP_OK);
    state->adsRefReady = false;
    sensorarrayLogStartup("ads", err,
                          state->adsReady ? "ok_ref_off" : "init_failed",
                          (int32_t)state->adsReady);
    sensorarrayLogSetAdsState(state->adsReady, state->adsRefReady);

    if (err != ESP_OK) {
        return err;
    }
    /* Install on the acquisition caller before Core 0 can become the first
     * DRDY ISR user. GPIO service affinity must not migrate with this probe. */
    err = sensorarrayBoardEnsureGpioIsrService();
    if (err != ESP_OK) {
        sensorarrayLogStartup("gpio_isr_service", err, "init_failed", err);
        return err;
    }
    err = sensorarrayBootSpiStart(state);
    printf("BOOT_SPI_POST,budgetMs=%u,targetCore=%d,priority=8,stackBytes=%u,enabled=%u,err=0x%lx\n",
           (unsigned)CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_BUDGET_MS,
           CONFIG_SENSORARRAY_MISC_CORE,
           (unsigned)CONFIG_SENSORARRAY_ADS_SPI_PROBE_TASK_STACK,
#if CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
           1u,
#else
           0u,
#endif
           (unsigned long)(uint32_t)err);
    if (err != ESP_OK) {
        sensorarrayLogStartup("ads_spi_probe", err, "task_create_failed", err);
        return err;
    }

    /* This interval includes both existing FDC startup scans and their logs.
     * It is measured, not covered by the independent SPI qualification budget. */
    sensorarrayBootSpiNoteI2c(true);
    if (state->boardReady) {
        sensorarrayInitFdcDevice(&state->fdcPrimary,
                                 primaryAddressValid,
                                 channels,
                                 "D1..D4_primary_ch0..ch3");
        sensorarrayInitFdcDevice(&state->fdcSecondary,
                                 secondaryAddressValid,
                                 channels,
                                 "D5..D8_secondary_ch0..ch3");
        BoardSupportI2cBusInfo_t bus0Info = {0};
        BoardSupportI2cBusInfo_t bus1Info = {0};
        (void)boardSupportGetI2cBusInfo(false, &bus0Info);
        (void)boardSupportGetI2cBusInfo(true, &bus1Info);
        printf("I2C_SELECTED,bus0Hz=%lu,bus1Hz=%lu,marginKHz=10,reason=post_handle_real_load_sweep\n",
               (unsigned long)bus0Info.FrequencyHz,
               (unsigned long)bus1Info.FrequencyHz);
        sensorarrayLogFdcParallelCfg();
    } else {
        sensorarrayLogStartupFdc("fdc_init",
                                 &state->fdcPrimary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4_primary_ch0..ch3");
        sensorarrayLogStartupFdc("fdc_init",
                                 &state->fdcSecondary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8_secondary_ch0..ch3");
    }

    sensorarrayBootSpiNoteI2c(false);
    err = sensorarrayBootSpiJoin();
    sensorarrayBootSpiLogJoin(err);
    if (err != ESP_OK) {
        /* In particular a timed-out worker may still own SPI/DMA.  Keep the
         * persistent job untouched and the early query surface installed. */
        sensorarrayLogStartup("ads_spi_probe", err, "join_failed", err);
        return err;
    }

    return ESP_OK;
}

bool sensorarrayFrontendStartupAdsOwnerQuiescent(void)
{
    sensorarrayBootSpiSnapshot_t s = sensorarrayBootSpiCopy();
    return s.phase != SENSORARRAY_BOOT_SPI_POSTED &&
           s.phase != SENSORARRAY_BOOT_SPI_RUNNING;
}
