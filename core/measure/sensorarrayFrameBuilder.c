#include "sensorarrayFrameBuilder.h"

#include <string.h>

#include "sensorarrayTypes.h"

#define SENSORARRAY_FRAME_INVALID_CAP_SENTINEL_PF (-1.0)
#define SENSORARRAY_FRAME_INVALID_FREQ_SENTINEL_HZ (-1.0)

void sensorarrayFrameBuilderInitInvalid(sensorarrayFrame_t *frame)
{
    if (!frame) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->errorMask = UINT64_MAX;
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        frame->freqHz[i] = SENSORARRAY_FRAME_INVALID_FREQ_SENTINEL_HZ;
        frame->capTotalPf[i] = SENSORARRAY_FRAME_INVALID_CAP_SENTINEL_PF;
        frame->measurement.valuesFixed[i] = SENSORARRAY_MEASUREMENT_INVALID_FIXED;
        frame->measurement.errorReason[i] = SENSORARRAY_CELL_ERROR_UNSUPPORTED;
    }
}
