#pragma once

#include "sensorarrayFrame.h"

void sensorarrayFrameBuilderInitInvalid(sensorarrayFrame_t *frame);

/* Returns the full 64-cell acquisition mask for every set row in rowMask. */
uint64_t sensorarrayFrameBuilderActiveCellMask(uint8_t rowMask);

/* Returns true when every expected cell completed an acquisition this sweep.
 * Acquisition is independent of electrical validity: OPEN/SHORT/RANGE/
 * SATURATED cells may be acquired and fresh while still invalid. */
bool sensorarrayFrameBuilderAcquisitionComplete(uint64_t acquiredMask,
                                                uint64_t expectedMask);

/* Returns the maximum pairwise span among the non-zero group timestamps.
 * With exactly one recorded group this is that group's duration. Returns
 * zero when fewer than two group timestamps are present. */
uint64_t sensorarrayFrameBuilderMaxGroupSkewUs(uint64_t capStartUs,
                                               uint64_t capEndUs,
                                               uint64_t voltStartUs,
                                               uint64_t voltEndUs,
                                               uint64_t resStartUs,
                                               uint64_t resEndUs);
