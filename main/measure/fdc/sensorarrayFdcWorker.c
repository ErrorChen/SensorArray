#include "sensorarrayFdcWorker.h"

const char *sensorarrayFdcWorkerRoleName(unsigned deviceId)
{
    return deviceId == 1u ? "secondary" : "primary";
}

