#include "sensorarrayFdcConvert.h"

#include "sensorarrayMeasure.h"

bool sensorarrayFdcConvertRawToCapPf(uint32_t raw28,
                                     uint16_t clockDividers,
                                     double inductorUh,
                                     double *outCapPf)
{
    double freqHz = sensorarrayMeasureFdcRawToSensorFrequencyHz(raw28, clockDividers);
    return sensorarrayMeasureFdcComputeCapacitancePf(freqHz, inductorUh, outCapPf);
}

