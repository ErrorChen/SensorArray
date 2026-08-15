#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayFrameBuilder.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while (0)

static uint64_t rowCells(uint8_t rows)
{
    return rows >= SENSORARRAY_MATRIX_COLS ? UINT64_MAX :
        (UINT64_C(1) << (rows * SENSORARRAY_MATRIX_COLS)) - 1u;
}

static int testActiveCellMaskRows(void)
{
    CHECK(sensorarrayFrameBuilderActiveCellMask(0u) == 0u);
    for (uint8_t rows = 1u; rows <= SENSORARRAY_MATRIX_ROWS; ++rows) {
        uint8_t rowMask = rows >= SENSORARRAY_MATRIX_ROWS ? 0xFFu :
            (uint8_t)((1u << rows) - 1u);
        CHECK(sensorarrayFrameBuilderActiveCellMask(rowMask) == rowCells(rows));
        uint8_t singleBit = (uint8_t)(1u << (rows - 1u));
        CHECK(sensorarrayFrameBuilderActiveCellMask(singleBit) ==
              (UINT64_C(0xFF) << ((rows - 1u) * SENSORARRAY_MATRIX_COLS)));
    }
    CHECK(sensorarrayFrameBuilderActiveCellMask(0x81u) ==
          (UINT64_C(0xFF) | (UINT64_C(0xFF) << (7u * SENSORARRAY_MATRIX_COLS))));
    return 0;
}

static int testAcquisitionComplete(void)
{
    const uint64_t full = UINT64_MAX;
    CHECK(sensorarrayFrameBuilderAcquisitionComplete(0u, 0u));
    CHECK(sensorarrayFrameBuilderAcquisitionComplete(full, full));

    /* Acquisition completeness must not depend on electrical validity. A
     * frame where every expected cell completed a read is complete even when
     * most of those cells are OPEN/invalid. */
    uint64_t expected = rowCells(3u);
    uint64_t acquired = expected;
    uint64_t valid = UINT64_C(0x01) & expected;
    CHECK(acquired == expected);
    CHECK(valid != expected);
    CHECK(sensorarrayFrameBuilderAcquisitionComplete(acquired, expected));

    /* One missing acquisition makes the sweep incomplete. */
    CHECK(!sensorarrayFrameBuilderAcquisitionComplete(expected & ~UINT64_C(1),
                                                      expected));

    /* Stray bits outside the expected mask are not part of the contract. */
    CHECK(!sensorarrayFrameBuilderAcquisitionComplete(full, expected));
    return 0;
}

static int testMaxGroupSkewUs(void)
{
    CHECK(sensorarrayFrameBuilderMaxGroupSkewUs(0u, 0u, 0u, 0u, 0u, 0u) == 0u);
    CHECK(sensorarrayFrameBuilderMaxGroupSkewUs(0u, 5000u, 0u, 0u, 0u, 0u) == 0u);
    CHECK(sensorarrayFrameBuilderMaxGroupSkewUs(100u, 500u, 0u, 0u, 0u, 0u) == 400u);
    CHECK(sensorarrayFrameBuilderMaxGroupSkewUs(100u, 500u,
                                                900u, 1200u,
                                                0u, 0u) == 1100u);
    CHECK(sensorarrayFrameBuilderMaxGroupSkewUs(100u, 500u,
                                                900u, 1200u,
                                                300u, 400u) == 1100u);
    CHECK(sensorarrayFrameBuilderMaxGroupSkewUs(0u, 0u,
                                                900u, 1200u,
                                                300u, 0u) == 900u);
    return 0;
}

static int testInitInvalid(void)
{
    sensorarrayFrame_t frame;
    memset(&frame, 0xA5, sizeof(frame));
    sensorarrayFrameBuilderInitInvalid(&frame);
    CHECK(frame.errorMask == UINT64_MAX);
    CHECK(frame.acquiredMask == 0u);
    CHECK(frame.expectedMask == 0u);
    CHECK(!frame.freshFrame && !frame.stale);
    for (size_t index = 0u; index < SENSORARRAY_MATRIX_CELL_COUNT; ++index) {
        CHECK(frame.freqHz[index] == -1.0);
        CHECK(frame.capTotalPf[index] == -1.0);
        CHECK(frame.measurement.valuesFixed[index] ==
              SENSORARRAY_MEASUREMENT_INVALID_FIXED);
        CHECK(frame.measurement.errorReason[index] ==
              SENSORARRAY_CELL_ERROR_UNSUPPORTED);
        CHECK((frame.measurement.validMask & (UINT64_C(1) << index)) == 0u);
        CHECK((frame.measurement.freshMask & (UINT64_C(1) << index)) == 0u);
    }
    return 0;
}

int main(void)
{
    CHECK(testActiveCellMaskRows() == 0);
    CHECK(testAcquisitionComplete() == 0);
    CHECK(testMaxGroupSkewUs() == 0);
    CHECK(testInitInvalid() == 0);
    printf("FRAME_SEMANTICS_TESTS,passed=1\n");
    return 0;
}
