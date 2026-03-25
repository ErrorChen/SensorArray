#include "sensorarrayBoardMap.h"

#include <stdio.h>
#include <string.h>

#include "sensorarrayConfig.h"

static const sensorarrayRouteMap_t s_sensorarrayRouteMap[] = {
    // Board/application route recipes used by current debug workflows.
    { SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE, SENSORARRAY_SELA_ROUTE_ADS1263, false, "S1D1_res_sela_ads1263" },
    { SENSORARRAY_S4, SENSORARRAY_D4, SENSORARRAY_PATH_RESISTIVE, SENSORARRAY_SELA_ROUTE_ADS1263, false, "S4D4_res_sela_ads1263_selb0" },
    /*
     * Confirmed mapping (datasheets/circuit.pdf page4 + TMUX1134 truth table):
     *  - U7 (SELB device) serves D5..D8.
     *  - On TMUX1134, SEL=1 selects SxA (capacitive Cx), SEL=0 selects SxB (resistive Rx).
     * Therefore capacitive D5/D7 routes require selBLevel=true, and resistive D7 requires false.
     */
    { SENSORARRAY_S5, SENSORARRAY_D5, SENSORARRAY_PATH_CAPACITIVE, SENSORARRAY_SELA_ROUTE_FDC2214, true, "S5D5_cap_selb_fdc2214" },
    { SENSORARRAY_S8, SENSORARRAY_D7, SENSORARRAY_PATH_CAPACITIVE, SENSORARRAY_SELA_ROUTE_FDC2214, true, "S8D7_cap_selb_fdc2214_selb1" },
    { SENSORARRAY_S8, SENSORARRAY_D7, SENSORARRAY_PATH_RESISTIVE, SENSORARRAY_SELA_ROUTE_ADS1263, false, "S8D7_volt_sela_ads1263_selb0" },
    { SENSORARRAY_S8, SENSORARRAY_D8, SENSORARRAY_PATH_CAPACITIVE, SENSORARRAY_SELA_ROUTE_FDC2214, true, "S8D8_cap_sela_fdc2214_selb1" },
    { SENSORARRAY_S8, SENSORARRAY_D8, SENSORARRAY_PATH_RESISTIVE, SENSORARRAY_SELA_ROUTE_ADS1263, false, "S8D8_volt_sela_ads1263_selb0" },
};

static const sensorarrayFdcDLineMap_t s_sensorarrayFdcDLineMap[] = {
    // Canonical FDC ownership on this board: D1..D4 -> primary(SELA), D5..D8 -> secondary(SELB).
    { 1u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH0, "D1_primary_sela_ch0" },
    { 2u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH1, "D2_primary_sela_ch1" },
    { 3u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH2, "D3_primary_sela_ch2" },
    { 4u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH3, "D4_primary_sela_ch3" },
    { 5u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH0, "D5_secondary_selb_ch0" },
    { 6u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH1, "D6_secondary_selb_ch1" },
    { 7u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH2, "D7_secondary_selb_ch2" },
    { 8u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH3, "D8_secondary_selb_ch3" },
};

const char *sensorarrayBoardMapSelaRouteName(sensorarraySelaRoute_t route)
{
    switch (route) {
    case SENSORARRAY_SELA_ROUTE_ADS1263:
        return "ADS1263";
    case SENSORARRAY_SELA_ROUTE_FDC2214:
        return "FDC2214";
    default:
        return "UNKNOWN";
    }
}

bool sensorarrayBoardMapSelaRouteToGpioLevel(sensorarraySelaRoute_t route, int *outLevel)
{
    if (!outLevel) {
        return false;
    }

    /*
     * Confirmed on current hardware after scope validation:
     *   SELA GPIO 0 -> ADS1263 branch
     *   SELA GPIO 1 -> FDC2214 branch
     *
     * This is the only place that translates logical SELA paths into raw GPIO
     * levels. Callers must pass sensorarraySelaRoute_t and must not hardcode
     * assumptions such as "0 means ADS1263".
     */
    switch (route) {
    case SENSORARRAY_SELA_ROUTE_ADS1263:
        *outLevel = 0;
        return true;
    case SENSORARRAY_SELA_ROUTE_FDC2214:
        *outLevel = 1;
        return true;
    default:
        return false;
    }
}

bool sensorarrayBoardMapSelaRouteFromGpioLevel(int gpioLevel, sensorarraySelaRoute_t *outRoute)
{
    if (!outRoute) {
        return false;
    }

    switch (gpioLevel) {
    case 0:
        *outRoute = SENSORARRAY_SELA_ROUTE_ADS1263;
        return true;
    case 1:
        *outRoute = SENSORARRAY_SELA_ROUTE_FDC2214;
        return true;
    default:
        return false;
    }
}

bool sensorarrayBoardMapAdsMuxForDLine(uint8_t dLine, uint8_t *muxp, uint8_t *muxn)
{
    if (!muxp || !muxn || dLine < 1u || dLine > 8u) {
        return false;
    }

    // Canonical ADS map: D1..D8 -> AIN0..AIN7, measured against AINCOM.
    *muxp = (uint8_t)(dLine - 1u);
    *muxn = SENSORARRAY_ADS_MUX_AINCOM;
    return true;
}

const sensorarrayRouteMap_t *sensorarrayBoardMapFindRoute(uint8_t sColumn,
                                                           uint8_t dLine,
                                                           sensorarrayPath_t path)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayRouteMap) / sizeof(s_sensorarrayRouteMap[0])); ++i) {
        const sensorarrayRouteMap_t *entry = &s_sensorarrayRouteMap[i];
        if (entry->sColumn == sColumn && entry->dLine == dLine && entry->path == path) {
            return entry;
        }
    }
    return NULL;
}

const sensorarrayFdcDLineMap_t *sensorarrayBoardMapFindFdcByDLine(uint8_t dLine)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayFdcDLineMap) / sizeof(s_sensorarrayFdcDLineMap[0])); ++i) {
        if (s_sensorarrayFdcDLineMap[i].dLine == dLine) {
            return &s_sensorarrayFdcDLineMap[i];
        }
    }
    return NULL;
}

size_t sensorarrayBoardMapRouteCount(void)
{
    return sizeof(s_sensorarrayRouteMap) / sizeof(s_sensorarrayRouteMap[0]);
}

const sensorarrayRouteMap_t *sensorarrayBoardMapRouteAt(size_t index)
{
    if (index >= sensorarrayBoardMapRouteCount()) {
        return NULL;
    }
    return &s_sensorarrayRouteMap[index];
}

size_t sensorarrayBoardMapFdcCount(void)
{
    return sizeof(s_sensorarrayFdcDLineMap) / sizeof(s_sensorarrayFdcDLineMap[0]);
}

const sensorarrayFdcDLineMap_t *sensorarrayBoardMapFdcAt(size_t index)
{
    if (index >= sensorarrayBoardMapFdcCount()) {
        return NULL;
    }
    return &s_sensorarrayFdcDLineMap[index];
}

const char *sensorarrayBoardMapPathName(sensorarrayPath_t path)
{
    return (path == SENSORARRAY_PATH_CAPACITIVE) ? "cap" : "res";
}

sensorarrayDebugPath_t sensorarrayBoardMapPathToDebugPath(sensorarrayPath_t path, tmux1108Source_t swSource)
{
    if (path == SENSORARRAY_PATH_CAPACITIVE) {
        return SENSORARRAY_DEBUG_PATH_CAPACITIVE;
    }
    if (swSource == TMUX1108_SOURCE_REF) {
        return SENSORARRAY_DEBUG_PATH_VOLTAGE;
    }
    return SENSORARRAY_DEBUG_PATH_RESISTIVE;
}

tmux1108Source_t sensorarrayBoardMapDefaultSwSource(const sensorarrayRouteMap_t *route)
{
    if (!route) {
        return TMUX1108_SOURCE_GND;
    }
    if (route->path == SENSORARRAY_PATH_CAPACITIVE) {
        return TMUX1108_SOURCE_REF;
    }
    if (route->mapLabel && strstr(route->mapLabel, "volt") != NULL) {
        return TMUX1108_SOURCE_REF;
    }
    return TMUX1108_SOURCE_GND;
}

void sensorarrayBoardMapAudit(void)
{
    for (size_t i = 0; i < sensorarrayBoardMapRouteCount(); ++i) {
        const sensorarrayRouteMap_t *entry = sensorarrayBoardMapRouteAt(i);
        if (!entry) {
            continue;
        }
        int selaWriteLevel = -1;
        (void)sensorarrayBoardMapSelaRouteToGpioLevel(entry->selaRoute, &selaWriteLevel);

        printf("DBGROUTEMAP,index=%u,sColumn=%u,dLine=%u,path=%s,selaRoute=%s,selaWriteLevel=%d,selBLevel=%u,label=%s\n",
               (unsigned)i,
               (unsigned)entry->sColumn,
               (unsigned)entry->dLine,
               sensorarrayBoardMapPathName(entry->path),
               sensorarrayBoardMapSelaRouteName(entry->selaRoute),
               selaWriteLevel,
               entry->selBLevel ? 1u : 0u,
               entry->mapLabel ? entry->mapLabel : SENSORARRAY_NA);
    }

    for (size_t i = 0; i < sensorarrayBoardMapFdcCount(); ++i) {
        const sensorarrayFdcDLineMap_t *entry = sensorarrayBoardMapFdcAt(i);
        if (!entry) {
            continue;
        }
        const char *devLabel =
            (entry->devId == SENSORARRAY_FDC_DEV_PRIMARY) ? "primary_sela_side" : "secondary_selb_side";
        printf("DBGFDCMAP,index=%u,dLine=%u,fdcDev=%s,channel=%u,label=%s\n",
               (unsigned)i,
               (unsigned)entry->dLine,
               devLabel,
               (unsigned)entry->channel,
               entry->mapLabel ? entry->mapLabel : SENSORARRAY_NA);
    }
}
