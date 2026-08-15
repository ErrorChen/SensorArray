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

uint64_t sensorarrayFrameBuilderActiveCellMask(uint8_t rowMask)
{
    uint64_t mask = 0u;
    for (uint8_t row = 0u; row < SENSORARRAY_MATRIX_ROWS; ++row) {
        if ((rowMask & (uint8_t)(1u << row)) != 0u) {
            mask |= UINT64_C(0xFF) << (row * SENSORARRAY_MATRIX_COLS);
        }
    }
    return mask;
}

bool sensorarrayFrameBuilderAcquisitionComplete(uint64_t acquiredMask,
                                                uint64_t expectedMask)
{
    return acquiredMask == expectedMask;
}

uint64_t sensorarrayFrameBuilderMaxGroupSkewUs(uint64_t capStartUs,
                                               uint64_t capEndUs,
                                               uint64_t voltStartUs,
                                               uint64_t voltEndUs,
                                               uint64_t resStartUs,
                                               uint64_t resEndUs)
{
    const uint64_t timestamps[] = {
        capStartUs, capEndUs, voltStartUs, voltEndUs, resStartUs, resEndUs,
    };
    uint64_t minimum = UINT64_MAX;
    uint64_t maximum = 0u;
    uint32_t present = 0u;
    for (size_t index = 0u; index < sizeof(timestamps) / sizeof(timestamps[0]);
         ++index) {
        if (timestamps[index] == 0u) {
            continue;
        }
        present++;
        if (timestamps[index] < minimum) {
            minimum = timestamps[index];
        }
        if (timestamps[index] > maximum) {
            maximum = timestamps[index];
        }
    }
    return present >= 2u ? maximum - minimum : 0u;
}
