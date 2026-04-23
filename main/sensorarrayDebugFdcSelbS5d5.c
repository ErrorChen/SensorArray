#include "sensorarrayDebugFdcSelbS5d5.h"

#include <stdio.h>

#include "sensorarrayDebugSelftest.h"

void sensorarrayDebugRunTestFdc2214SelbS5D5(sensorarrayState_t *state)
{
    // Keep legacy entry-point symbol, but route all S5D5 single-point logic to self-test layer.
    printf("DBGS5D5_MODE,stage=legacy_entry_redirect,targetImpl=sensorarrayDebugRunS5d5CapFdcSecondaryModeImpl,"
           "activeMode=%d,stateValid=%u\n",
           (int)SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY,
           state ? 1u : 0u);
    if (!state || !state->boardReady || !state->tmuxReady) {
        printf("DBGS5D5_MODE,stage=abort,reason=state_uninitialized,activeMode=%d,stateValid=%u\n",
               (int)SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY,
               state ? 1u : 0u);
        return;
    }
    sensorarrayDebugRunS5d5CapFdcSecondaryModeImpl(state);
}
