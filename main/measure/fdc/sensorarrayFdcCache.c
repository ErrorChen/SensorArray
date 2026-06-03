#include "sensorarrayFdcCache.h"

#include <stdbool.h>

const char *sensorarrayFdcCacheForceReasonName(bool force)
{
    return force ? "force_after_sleep_or_resync" : "diff_only";
}

