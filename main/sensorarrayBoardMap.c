#include "sensorarrayBoardMap.h"

#include <stdio.h>
#include <string.h>

#include "sensorarrayConfig.h"

#ifndef CONFIG_TMUX1134_DEFAULT_ALL_OFF
#define CONFIG_TMUX1134_DEFAULT_ALL_OFF 0
#endif

/*
 * Board-control semantic contract (S5D5 debug must follow this table).
 *
 * TMUX1108:
 *   - A0/A1/A2: row binary select, active-high bits.
 *   - SW gpio: polarity is board-specific via CONFIG_TMUX1108_SW_REF_LEVEL.
 *       SW level == refLevel -> source REF
 *       SW level != refLevel -> source GND
 *     For S5D5 capacitive bring-up we intentionally command source=GND so the
 *     D branch is forced to local reference ground during route verification.
 *
 * TMUX1134:
 *   - SELA/SELB/SEL3/SEL4: active-high selects SxA, low selects SxB.
 *   - EN: if present, off level is CONFIG_TMUX1134_EN_OFF_LEVEL.
 *   - On this board SELA semantics are:
 *       SELA=0 -> ADS1263 branch
 *       SELA=1 -> FDC2214 branch
 *   - On this board SELB semantics are:
 *       SELB=0 -> resistive branch (Rx)
 *       SELB=1 -> capacitive branch (Cx, D5..D8 side)
 *
 * Default power-up state after tmuxSwitchInit():
 *   row=0 (A2..A0 = 000), SELA=0, SELB=0, SEL3=0(if present), SEL4=0(if present),
 *   SW=CONFIG_TMUX1108_DEFAULT_SOURCE, EN logical on (or hard-wired on).
 */

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

int sensorarrayBoardMapSwSourceToGpioLevel(tmux1108Source_t source)
{
    int refLevel = CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0;
    return (source == TMUX1108_SOURCE_REF) ? refLevel : (refLevel ? 0 : 1);
}

bool sensorarrayBoardMapSwSourceFromGpioLevel(int gpioLevel, tmux1108Source_t *outSource)
{
    if (!outSource) {
        return false;
    }
    if (gpioLevel != 0 && gpioLevel != 1) {
        return false;
    }
    int refLevel = CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0;
    *outSource = (gpioLevel == refLevel) ? TMUX1108_SOURCE_REF : TMUX1108_SOURCE_GND;
    return true;
}

const char *sensorarrayBoardMapSwSourceSemanticName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "REF" : "GND";
}

const char *sensorarrayBoardMapSelLevelSemanticName(int gpioLevel)
{
    if (gpioLevel < 0) {
        return "UNKNOWN";
    }
    return (gpioLevel != 0) ? "SxA" : "SxB";
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
        /*
         * Capacitive S5D5 bring-up expects D-side forced to board ground through
         * TMUX1108 SW source selection.
         */
        return TMUX1108_SOURCE_GND;
    }
    if (route->mapLabel && strstr(route->mapLabel, "volt") != NULL) {
        return TMUX1108_SOURCE_REF;
    }
    return TMUX1108_SOURCE_GND;
}

void sensorarrayBoardMapAudit(void)
{
    printf("DBGROUTECTRL,gpioName=A0,gpioNum=%d,activePolarity=active_high,logic1Meaning=row_bit0=1,"
           "defaultPowerUp=0\n",
           CONFIG_TMUX1108_A0_GPIO);
    printf("DBGROUTECTRL,gpioName=A1,gpioNum=%d,activePolarity=active_high,logic1Meaning=row_bit1=1,"
           "defaultPowerUp=0\n",
           CONFIG_TMUX1108_A1_GPIO);
    printf("DBGROUTECTRL,gpioName=A2,gpioNum=%d,activePolarity=active_high,logic1Meaning=row_bit2=1,"
           "defaultPowerUp=0\n",
           CONFIG_TMUX1108_A2_GPIO);
    printf("DBGROUTECTRL,gpioName=SW,gpioNum=%d,activePolarity=%s,logic1Meaning=%s,logic0Meaning=%s,"
           "defaultPowerUpSource=%s\n",
           CONFIG_TMUX1108_SW_GPIO,
           CONFIG_TMUX1108_SW_REF_LEVEL ? "active_high_selects_REF" : "active_high_selects_GND",
           CONFIG_TMUX1108_SW_REF_LEVEL ? "source=REF" : "source=GND",
           CONFIG_TMUX1108_SW_REF_LEVEL ? "source=GND" : "source=REF",
           sensorarrayBoardMapSwSourceSemanticName((CONFIG_TMUX1108_DEFAULT_SOURCE != 0) ? TMUX1108_SOURCE_REF
                                                                                           : TMUX1108_SOURCE_GND));
    printf("DBGROUTECTRL,gpioName=SELA,gpioNum=%d,activePolarity=active_high_selects_SxA,logic1Meaning=FDC2214,"
           "logic0Meaning=ADS1263,defaultPowerUp=0\n",
           CONFIG_TMUX1134_SEL1_GPIO);
    printf("DBGROUTECTRL,gpioName=SELB,gpioNum=%d,activePolarity=active_high_selects_SxA,logic1Meaning=capacitive_Cx_D5_D8,"
           "logic0Meaning=resistive_Rx_D5_D8,defaultPowerUp=0\n",
           CONFIG_TMUX1134_SEL2_GPIO);
    printf("DBGROUTECTRL,gpioName=SEL3,gpioNum=%d,activePolarity=active_high_selects_SxA,logic1Meaning=SxA,"
           "logic0Meaning=SxB,defaultPowerUp=%s\n",
           CONFIG_TMUX1134_SEL3_GPIO,
           (CONFIG_TMUX1134_SEL3_GPIO >= 0) ? "0" : "na");
    printf("DBGROUTECTRL,gpioName=SEL4,gpioNum=%d,activePolarity=active_high_selects_SxA,logic1Meaning=SxA,"
           "logic0Meaning=SxB,defaultPowerUp=%s\n",
           CONFIG_TMUX1134_SEL4_GPIO,
           (CONFIG_TMUX1134_SEL4_GPIO >= 0) ? "0" : "na");
    printf("DBGROUTECTRL,gpioName=EN,gpioNum=%d,activePolarity=%s,logic1Meaning=%s,logic0Meaning=%s,defaultPowerUp=%s\n",
           CONFIG_TMUX1134_EN_GPIO,
           CONFIG_TMUX1134_EN_OFF_LEVEL ? "active_low_enables" : "active_high_enables",
           CONFIG_TMUX1134_EN_OFF_LEVEL ? "disable" : "enable",
           CONFIG_TMUX1134_EN_OFF_LEVEL ? "enable" : "disable",
           (CONFIG_TMUX1134_EN_GPIO >= 0)
               ? (CONFIG_TMUX1134_DEFAULT_ALL_OFF ? (CONFIG_TMUX1134_EN_OFF_LEVEL ? "1" : "0")
                                                  : (CONFIG_TMUX1134_EN_OFF_LEVEL ? "0" : "1"))
               : "hardwired_on");

    for (size_t i = 0; i < sensorarrayBoardMapRouteCount(); ++i) {
        const sensorarrayRouteMap_t *entry = sensorarrayBoardMapRouteAt(i);
        if (!entry) {
            continue;
        }
        int selaWriteLevel = -1;
        (void)sensorarrayBoardMapSelaRouteToGpioLevel(entry->selaRoute, &selaWriteLevel);
        int swWriteLevel = sensorarrayBoardMapSwSourceToGpioLevel(sensorarrayBoardMapDefaultSwSource(entry));

        printf("DBGROUTEMAP,index=%u,sColumn=%u,dLine=%u,path=%s,selaRoute=%s,selaWriteLevel=%d,selBLevel=%u,"
               "swSource=%s,swWriteLevel=%d,label=%s\n",
               (unsigned)i,
               (unsigned)entry->sColumn,
               (unsigned)entry->dLine,
               sensorarrayBoardMapPathName(entry->path),
               sensorarrayBoardMapSelaRouteName(entry->selaRoute),
               selaWriteLevel,
               entry->selBLevel ? 1u : 0u,
               sensorarrayBoardMapSwSourceSemanticName(sensorarrayBoardMapDefaultSwSource(entry)),
               swWriteLevel,
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
