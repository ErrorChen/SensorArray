#pragma once

#include <stdbool.h>
#include <stdint.h>

bool sensorarrayFdcConvertRawToCapPf(uint32_t raw28,
                                     uint16_t clockDividers,
                                     double inductorUh,
                                     double *outCapPf);

