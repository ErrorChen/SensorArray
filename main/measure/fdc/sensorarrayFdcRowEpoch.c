#include "sensorarrayFdcRowEpoch.h"

const char *sensorarrayFdcRowEpochStageName(unsigned stage)
{
    switch (stage) {
    case 0:
        return "sleep_enter";
    case 1:
        return "row_switch";
    case 2:
        return "apply_cache";
    case 3:
        return "sleep_exit";
    default:
        return "unknown";
    }
}

