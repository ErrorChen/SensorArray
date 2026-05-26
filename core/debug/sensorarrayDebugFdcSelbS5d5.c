#include "sensorarrayDebugFdcSelbS5d5.h"

#include "sensorarrayDebugSelftest.h"

void sensorarrayDebugRunTestFdc2214SelbS5D5(sensorarrayState_t *state)
{
    // Keep legacy entry-point symbol, but route all S5D5 single-point logic to self-test layer.
    sensorarrayDebugRunS5d5CapFdcSecondaryModeImpl(state);
}
