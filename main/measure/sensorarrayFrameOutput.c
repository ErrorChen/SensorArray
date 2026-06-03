#include "sensorarrayFrameOutput.h"

#include "sensorarrayMeasure.h"

esp_err_t sensorarrayFrameOutputPrint(const sensorarrayFrame_t *frame)
{
    return sensorarrayFdcMatrixEmitFrame(frame);
}

