#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sensorarrayMeasurementMode.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while (0)

typedef struct {
    sensorarrayMeasurementMode_t mode;
    uint32_t generation;
    uint32_t requestId;
} resolvedMetadata_t;

/* Mirrors the trailing metadata selection in sensorarrayRunOneFrame(): the
 * post-apply snapshot wins only when a pending homogeneous profile was
 * applied during this frame, otherwise the pre-frame snapshot is used. */
static resolvedMetadata_t resolveFrameMetadata(
    const sensorarrayMeasurementModeSnapshot_t *preApply,
    bool appliedPendingProfile,
    const sensorarrayMeasurementModeSnapshot_t *postApply)
{
    const sensorarrayMeasurementModeSnapshot_t *source =
        appliedPendingProfile ? postApply : preApply;
    return (resolvedMetadata_t){
        .mode = source->activeMode,
        .generation = source->generation,
        .requestId = source->appliedRequestId,
    };
}

/* Mirrors the CAP unit/decimalScale/mask branch: it must key on the resolved
 * frame mode, never on the pre-apply snapshot. */
static bool isCapUnitMaskBranch(const resolvedMetadata_t *metadata)
{
    return metadata->mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
}

#define REQUIRE(condition)                                                      \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return (sensorarrayMeasurementModeSnapshot_t){0};                   \
        }                                                                       \
    } while (0)

static sensorarrayMeasurementModeSnapshot_t applyMode(
    sensorarrayMeasurementModeContext_t *context,
    sensorarrayMeasurementMode_t target,
    uint32_t requestId)
{
    sensorarrayMeasurementModeSnapshot_t out = {0};
    REQUIRE(sensorarrayMeasurementModeAccept(context, target, requestId));
    REQUIRE(sensorarrayMeasurementModeBeginTransition(context));
    REQUIRE(sensorarrayMeasurementModeCompleteTransition(context, 100u, 0u));
    REQUIRE(sensorarrayMeasurementModeCopySnapshot(context, &out));
    return out;
}

static int testHomogeneousRowModesFirstFrameMetadata(void)
{
    sensorarrayMeasurementModeContext_t context;
    sensorarrayMeasurementModeInit(&context);

    /* Seed CAP (request id 10, generation 1). */
    sensorarrayMeasurementModeSnapshot_t applied = applyMode(
        &context, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE, 10u);
    CHECK(applied.generation == 1u);
    CHECK(applied.appliedRequestId == 10u);

    const sensorarrayMeasurementMode_t sequence[] = {
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
    };
    const uint32_t requestIds[] = {21u, 22u, 23u};
    uint32_t expectedGeneration = 2u;
    for (size_t index = 0u; index < 3u; ++index) {
        const sensorarrayMeasurementModeSnapshot_t pre = applied;
        applied = applyMode(&context, sequence[index], requestIds[index]);
        CHECK(applied.activeMode == sequence[index]);
        CHECK(applied.generation == expectedGeneration++);
        CHECK(applied.appliedRequestId == requestIds[index]);

        /* The first RMAPP frame must carry the post-apply metadata. */
        const resolvedMetadata_t resolved =
            resolveFrameMetadata(&pre, true, &applied);
        CHECK(resolved.mode == sequence[index]);
        CHECK(resolved.generation == applied.generation);
        CHECK(resolved.requestId == requestIds[index]);

        /* The pre-apply snapshot must not drive tag/unit decisions. */
        const resolvedMetadata_t stale =
            resolveFrameMetadata(&pre, false, &applied);
        CHECK(stale.mode == pre.activeMode);
        CHECK(stale.generation == pre.generation);
        CHECK(stale.mode != sequence[index]);
        CHECK(isCapUnitMaskBranch(&resolved) ==
              (sequence[index] == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE));
    }

    /* Normal MODE path: no in-frame apply, so the pre-frame snapshot stays
     * authoritative and the applied snapshot is already current. */
    const resolvedMetadata_t steady = resolveFrameMetadata(&applied, false, &applied);
    CHECK(steady.mode == applied.activeMode);
    CHECK(steady.generation == applied.generation);
    CHECK(steady.requestId == applied.appliedRequestId);
    return 0;
}

int main(void)
{
    CHECK(testHomogeneousRowModesFirstFrameMetadata() == 0);
    printf("ROWMODES_FRAME_METADATA_TESTS,passed=1\n");
    return 0;
}
