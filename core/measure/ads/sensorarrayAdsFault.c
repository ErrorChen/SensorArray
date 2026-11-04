#include "sensorarrayAdsFault.h"

#include <stdio.h>
#include <string.h>

static const char *const s_stageNames[SENSORARRAY_ADS_FAULT_STAGE_COUNT] = {
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE] = "MATRIX_ROUTE",
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READ] = "MATRIX_READ",
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK] = "MATRIX_READBACK",
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_DRDY] = "MATRIX_DRDY",
    [SENSORARRAY_ADS_FAULT_STAGE_BATTERY_GAP] = "BATTERY_GAP",
    [SENSORARRAY_ADS_FAULT_STAGE_BATTERY_RESTORE] = "BATTERY_RESTORE",
    [SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR] = "RAIL_MONITOR",
    [SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION] = "PROFILE_TRANSITION",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_START] = "RECOVERY_START",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_ATTEMPT] = "RECOVERY_ATTEMPT",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_RESUME] = "RECOVERY_RESUME",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_FAILED] = "RECOVERY_FAILED",
    [SENSORARRAY_ADS_FAULT_STAGE_REGISTER_HEALTH_READ] = "REGISTER_HEALTH_READ",
    [SENSORARRAY_ADS_FAULT_STAGE_REGISTER_READ_UNSTABLE] = "REGISTER_READ_UNSTABLE",
    [SENSORARRAY_ADS_FAULT_STAGE_POWER_COMPARE] = "POWER_COMPARE",
    [SENSORARRAY_ADS_FAULT_STAGE_REFMUX_COMPARE] = "REFMUX_COMPARE",
    [SENSORARRAY_ADS_FAULT_STAGE_MODE_COMPARE] = "MODE_COMPARE",
    [SENSORARRAY_ADS_FAULT_STAGE_RESET_OBSERVED] = "RESET_OBSERVED",
    [SENSORARRAY_ADS_FAULT_STAGE_CACHE_COMPARE] = "CACHE_COMPARE",
    [SENSORARRAY_ADS_FAULT_STAGE_ROW_GPIO_COMPARE] = "ROW_GPIO_COMPARE",
    [SENSORARRAY_ADS_FAULT_STAGE_OWNER_ACQUIRE] = "OWNER_ACQUIRE",
    [SENSORARRAY_ADS_FAULT_STAGE_RESTORE_VERIFY] = "RESTORE_VERIFY",
    [SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN] = "UNKNOWN",
};

static const char *const s_outcomeNames[] = {
    [SENSORARRAY_ADS_FAULT_OUTCOME_NONE] = "none",
    [SENSORARRAY_ADS_FAULT_OUTCOME_STARTED] = "started",
    [SENSORARRAY_ADS_FAULT_OUTCOME_ATTEMPT] = "attempt",
    [SENSORARRAY_ADS_FAULT_OUTCOME_RESUMED] = "resumed",
    [SENSORARRAY_ADS_FAULT_OUTCOME_FAILED] = "failed",
};

#define SENSORARRAY_ADS_FAULT_OUTCOME_COUNT \
    (sizeof(s_outcomeNames) / sizeof(s_outcomeNames[0]))

static uint64_t s_lastEmitUs[SENSORARRAY_ADS_FAULT_STAGE_COUNT];

const char *sensorarrayAdsFaultStageName(sensorarrayAdsFaultStage_t stage)
{
    if ((unsigned)stage >= SENSORARRAY_ADS_FAULT_STAGE_COUNT) {
        return "UNKNOWN";
    }
    return s_stageNames[stage];
}

const char *sensorarrayAdsFaultOutcomeName(sensorarrayAdsFaultOutcome_t outcome)
{
    if (outcome < 0 || (size_t)outcome >= SENSORARRAY_ADS_FAULT_OUTCOME_COUNT) {
        return "none";
    }
    return s_outcomeNames[outcome];
}

static const char *sensorarrayAdsFaultText(const char *text)
{
    return text && text[0] != '\0' ? text : "unknown";
}

size_t sensorarrayAdsFaultFormat(const sensorarrayAdsFaultEvent_t *event,
                                 char *buffer,
                                 size_t bufferSize)
{
    if (!event || !buffer || bufferSize == 0u) {
        return 0u;
    }
    char boot[12], bootId[12];
    if (event->bootId == 0u) {
        strcpy(bootId, "unknown");
        strcpy(boot, "unknown");
    } else {
        snprintf(bootId, sizeof(bootId), "%lu", (unsigned long)event->bootId);
        snprintf(boot, sizeof(boot), "%lu", (unsigned long)event->bootCount);
    }
    const sensorarrayAdsMatrixFaultContext_t *ctx = event->context;
    bool captured = ctx && ctx->valid;
    /* Format into caller-owned storage; avoid a second 1.5 KiB stack copy. */
    char *line = buffer;
    size_t lineCapacity = bufferSize < SENSORARRAY_ADS_FAULT_LINE_MAX + 1u ?
        bufferSize : SENSORARRAY_ADS_FAULT_LINE_MAX + 1u;
    int written = snprintf(line, lineCapacity,
        "ADSFAULT,stage=%s,err=0x%lx,boot=%s,bootId=%s,seq=%lu,mode=%.16s,modeGen=%lu,profileGen=%lu,rowGen=%lu,rowReq=%lu,profile=%.8s,route=%.16s,owner=%.16s,drdyGen=%lu,cfgGen=%lu,attempt=%lu,outcome=%s\n"
        "ADSFAULTRAIL,bootId=%s,seq=%lu,railSpanUv=%ld,avddUv=%ld,avssUv=%ld,source=%.24s,valid=%u,ref=%.16s,restoreExp=%ld,restoreAct=%ld\n",
        sensorarrayAdsFaultStageName(captured ? ctx->stage : event->stage),
        (unsigned long)(uint32_t)(captured ? ctx->originalError : event->err),
        boot, bootId, (unsigned long)event->seq,
        sensorarrayAdsFaultText(captured ? ctx->mode : event->mode),
        (unsigned long)event->modeGeneration, (unsigned long)event->profileGeneration,
        (unsigned long)event->rowGeneration, (unsigned long)event->rowRequestId,
        sensorarrayAdsFaultText(event->profile),
        sensorarrayAdsFaultText(captured ? ctx->route : event->route),
        sensorarrayAdsFaultText(captured ? ctx->owner : event->owner),
        (unsigned long)event->drdyGeneration, (unsigned long)event->configGeneration,
        (unsigned long)event->attempt, sensorarrayAdsFaultOutcomeName(event->outcome),
        bootId, (unsigned long)event->seq,
        (long)event->railSpanUv, (long)event->avddUv, (long)event->avssUv,
        sensorarrayAdsFaultText(event->railSource), event->railValid ? 1u : 0u,
        sensorarrayAdsFaultText(event->reference),
        (long)(event->restoreExpectedValid ? event->restoreExpected : 0),
        (long)(event->restoreActualValid ? event->restoreActual : 0));
    if (written < 0 || (size_t)written >= lineCapacity) {
        buffer[0] = '\0';
        return 0u;
    }
    size_t used = (size_t)written;
    if (captured) {
        written = snprintf(line + used, lineCapacity - used,
            "ADSFAULTCTX,bootId=%s,seq=%lu,originalError=0x%lx,cleanupError=0x%lx,engineFrameCount=%lu,modeEntryFrameCount=%lu,healthInterval=%lu,healthReadbackDue=%u,precisionFrame=%u,attempted=%u,acquired=%u\n"
            "ADSFAULTREG,bootId=%s,seq=%lu,reg=%u,expected=%02X,actual=%02X,mask=%02X,cacheGen=%lu,physicalGen=%lu,configGen=%lu\n",
            bootId, (unsigned long)event->seq,
            (unsigned long)(uint32_t)ctx->originalError, (unsigned long)(uint32_t)ctx->cleanupError,
            (unsigned long)ctx->engineFrameCount, (unsigned long)ctx->modeEntryFrameCount,
            (unsigned long)ctx->healthInterval, ctx->healthReadbackDue ? 1u : 0u,
            ctx->precisionFrame ? 1u : 0u,
            ctx->attempted ? 1u : 0u, ctx->acquired ? 1u : 0u,
            bootId, (unsigned long)event->seq,
            ctx->registerId, ctx->expected, ctx->actual, ctx->compareMask,
            (unsigned long)ctx->cacheGeneration, (unsigned long)ctx->physicalGeneration,
            (unsigned long)ctx->configGeneration);
        if (written < 0 || (size_t)written >= lineCapacity - used) {
            buffer[0] = '\0'; return 0u;
        }
        used += (size_t)written;
        const sensorarrayAdsTransactionEvidence_t *transactions[3] = {
            &ctx->rail, &ctx->battery, &ctx->check};
        const char *names[3] = {"rail", "battery", "ADSCHK"};
        for (size_t index = 0u; index < 3u; ++index) {
            const sensorarrayAdsTransactionEvidence_t *t = transactions[index];
            written = snprintf(line + used, lineCapacity - used,
                "ADSFAULTTXN,bootId=%s,seq=%lu,kind=%s,id=%lu,startUs=%llu,completeUs=%llu,firstValidFrameUs=%llu,originalError=0x%lx,cleanupError=0x%lx\n",
                bootId, (unsigned long)event->seq, names[index], (unsigned long)t->id,
                (unsigned long long)t->startedUs, (unsigned long long)t->completedUs,
                (unsigned long long)t->firstValidFrameUs,
                (unsigned long)(uint32_t)t->originalError, (unsigned long)(uint32_t)t->cleanupError);
            if (written < 0 || (size_t)written >= lineCapacity - used) {
                buffer[0] = '\0'; return 0u;
            }
            used += (size_t)written;
        }
    }
    if (used + 1u > bufferSize) {
        buffer[0] = '\0';
        return 0u;
    }
    return used;
}

size_t sensorarrayAdsFaultEmit(const sensorarrayAdsFaultEvent_t *event,
                               uint64_t nowUs,
                               sensorarrayAdsFaultSink_t sink,
                               void *context)
{
    if (!event || !sink) {
        return 0u;
    }

    sensorarrayAdsFaultStage_t effectiveStage = event->context && event->context->valid ?
        event->context->stage : event->stage;
    size_t stageIndex = (size_t)effectiveStage;
    if (stageIndex >= SENSORARRAY_ADS_FAULT_STAGE_COUNT) {
        stageIndex = SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN;
    }
    bool recoveryStage =
        effectiveStage >= SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_START &&
        effectiveStage <= SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_FAILED;
    if (!recoveryStage &&
        s_lastEmitUs[stageIndex] != 0u &&
        nowUs >= s_lastEmitUs[stageIndex] &&
        nowUs - s_lastEmitUs[stageIndex] <
            SENSORARRAY_ADS_FAULT_RATE_LIMIT_US) {
        return 0u;
    }

    char line[SENSORARRAY_ADS_FAULT_LINE_MAX + 1u];
    size_t length = sensorarrayAdsFaultFormat(event, line, sizeof(line));
    if (length == 0u) {
        return 0u;
    }
    for (const char *start = line; *start;) {
        const char *end = strchr(start, '\n');
        if (!end || (size_t)(end - start) + 1u >= 384u) {
            return 0u;
        }
        start = end + 1u;
    }
    s_lastEmitUs[stageIndex] = nowUs;
    /* Each complete record fits the acquisition EventRing's 384-byte item.
     * Never submit a multi-record blob to a sink which could truncate it. */
    char *start = line;
    while (*start) {
        char *end = strchr(start, '\n');
        if (!end) {
            break;
        }
        char saved = end[1];
        end[1] = '\0';
        size_t partLength = (size_t)(end - start) + 1u;
        if (partLength >= 384u) {
            return 0u;
        }
        sink(start, partLength, context);
        end[1] = saved;
        start = end + 1u;
    }
    return length;
}
