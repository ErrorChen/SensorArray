#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ads126xAdc.h"
#include "boardSupport.h"
#include "sensorarrayApp.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayTypes.h"
#include "sensorarrayVoltageScan.h"
#include "tmuxSwitch.h"

static sensorarrayState_t s_state = {0};
static uint8_t s_gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
static bool s_voltageHeaderPrinted = false;

typedef struct {
    const char *modeName;
    const char *modeNameCn;
    tmux1108Source_t swSource;
    const char *swName;
    bool skipFdcInit;
} sensorarrayVoltageReadModeConfig_t;

static const  sensorarrayVoltageReadModeConfig_t s_resistanceReadMode= {
    .modeName = "RESISTANCE_READ",
    .modeNameCn = "电阻读取",
    .swSource = TMUX1108_SOURCE_REF,
    .swName = "REF",
    .skipFdcInit = true,
};

static const  sensorarrayVoltageReadModeConfig_t s_piezoReadMode= {
    .modeName = "PIEZO_READ",
    .modeNameCn = "压电读取",
    .swSource = TMUX1108_SOURCE_GND,
    .swName = "GND",
    .skipFdcInit = true,
};

static uint8_t sensorarrayMainNormalizeGain(int configuredGain)
{
    switch (configuredGain) {
    case ADS126X_GAIN_1:
    case ADS126X_GAIN_2:
    case ADS126X_GAIN_4:
    case ADS126X_GAIN_8:
    case ADS126X_GAIN_16:
    case ADS126X_GAIN_32:
        return (uint8_t)configuredGain;
    default:
        return ADS126X_GAIN_1;
    }
}

static const char *sensorarrayMainAutoGainModeName(void)
{
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_OFF
    return "off";
#elif CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_FRAME
    return "per_frame";
#elif CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_POINT
    return "per_point";
#else
    return "bootstrap_once";
#endif
}

static void sensorarrayMainIdleAfterFatal(const char *stage, esp_err_t err)
{
    printf("VOLTSCAN_FATAL,stage=%s,err=%ld\n", stage ? stage : "unknown", (long)err);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }
}

static esp_err_t sensorarrayMainApplyVoltageTmuxDefaults(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmuxSwitchSelectRow(0u);
    if (err != ESP_OK) {
        return err;
    }

    int selaLevel = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_ADS1263, &selaLevel)) {
        return ESP_ERR_INVALID_STATE;
    }

    err = tmux1134SelectSelALevel(selaLevel != 0);
    if (err == ESP_OK) {
        err = tmux1134SelectSelBLevel(false);
    }
    if (err == ESP_OK) {
        err = tmuxSwitchSet1108Source(mode->swSource);
    }
    if (err == ESP_OK) {
        err = tmux1134SetEnLogicalState(true);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us((uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
    }
    return err;
}

static void sensorarrayMainPrintAppMode(const sensorarrayVoltageReadModeConfig_t *mode)
{
    printf("APPMODE,active=%s,cnName=%s,skipAdsInit=0,skipFdcInit=%u,sw=%s\n",
           mode->modeName,
           mode->modeNameCn,
           mode->skipFdcInit ? 1u : 0u,
           mode->swName);
}

static void sensorarrayMainPrintRoutePolicy(const sensorarrayVoltageReadModeConfig_t *mode)
{
    printf("DBGROUTEPOLICY,mode=%s,sw=%s,sela=ADS126X,fdcInitSkipped=%u\n",
           mode->modeName,
           mode->swName,
           mode->skipFdcInit ? 1u : 0u);
}

static esp_err_t sensorarrayMainInitVoltageScan(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state = (sensorarrayState_t){0};
    s_voltageHeaderPrinted = false;
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));

    sensorarrayMainPrintAppMode(mode);

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayBringupInitAds(&s_state);
    s_state.adsReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayBringupPrepareAdsRefPath(&s_state);
    s_state.adsRefReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMainApplyVoltageTmuxDefaults(mode);
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayMainPrintRoutePolicy(mode);

    if (s_state.adsAdc1Running) {
        err = ads126xAdcStopAdc1(&s_state.ads);
        if (err != ESP_OK) {
            return err;
        }
        s_state.adsAdc1Running = false;
    }

    uint8_t defaultGain = sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN);
    uint8_t dataRateDr = (uint8_t)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F);
    err = ads126xAdcConfigureVoltageMode(&s_state.ads, defaultGain, dataRateDr, false, false);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcStartAdc1(&s_state.ads);
    if (err != ESP_OK) {
        return err;
    }
    s_state.adsAdc1Running = true;
    return ESP_OK;
}

static ads126xAdcVoltageReadConfig_t sensorarrayMainAutoGainReadConfig(uint8_t muxp, uint8_t muxn)
{
    return (ads126xAdcVoltageReadConfig_t){
        .muxp = muxp,
        .muxn = muxn,
        .stopBeforeMuxChange = false,
        .startAfterMuxChange = false,
        .settleAfterMuxUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
        .discardFirst = (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0),
        .oversampleCount = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE,
        .drdyTimeoutUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_DRDY_TIMEOUT_US,
    };
}

static esp_err_t sensorarrayMainBootstrapAutoGain(const sensorarrayVoltageReadModeConfig_t *mode, bool printSummary)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t defaultGain = sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN);

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_OFF
    memset(s_gainTable, defaultGain, sizeof(s_gainTable));
    if (printSummary) {
        printf("VOLTSCAN_GAIN,mode=off,gain=%u\n", (unsigned)defaultGain);
    }
    return ESP_OK;
#else
    uint32_t errCount = 0u;
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        esp_err_t routeErr = sensorarrayVoltageScanApplyRouteFastWithSource(
            s,
            1u,
            mode->swSource,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_ROW_SETTLE_US,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
        if (routeErr != ESP_OK) {
            errCount += SENSORARRAY_VOLTAGE_SCAN_COLS;
            continue;
        }

        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            uint8_t muxp = 0u;
            uint8_t muxn = 0u;
            if (!sensorarrayBoardMapAdsMuxForDLine(d, &muxp, &muxn)) {
                s_gainTable[s - 1u][d - 1u] = defaultGain;
                errCount++;
                continue;
            }

            ads126xAdcVoltageReadConfig_t readCfg = sensorarrayMainAutoGainReadConfig(muxp, muxn);
            ads126xAdcAutoGainConfig_t gainCfg = {
                .minGain = ADS126X_GAIN_1,
                .maxGain = ADS126X_GAIN_32,
                .initialGain = defaultGain,
                .headroomPercent = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT,
                .maxIterations = 4u,
                .settleAfterGainChangeUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
                .keepConfiguredOnSuccess = true,
            };
            ads126xAdcAutoGainResult_t gainResult = {0};
            esp_err_t gainErr = ads126xAdcSelectAutoGainForVoltage(&s_state.ads, &readCfg, &gainCfg, &gainResult);
            if (gainErr == ESP_OK) {
                s_gainTable[s - 1u][d - 1u] = gainResult.selectedGain;
            } else {
                s_gainTable[s - 1u][d - 1u] = defaultGain;
                errCount++;
            }
        }
    }

    if (printSummary || CONFIG_SENSORARRAY_VOLTAGE_SCAN_VERBOSE_LOG) {
        printf("VOLTSCAN_GAIN,mode=%s,points=64,errCount=%lu,headroomPercent=%u\n",
               sensorarrayMainAutoGainModeName(),
               (unsigned long)errCount,
               (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT);
    }
    return (errCount == 0u) ? ESP_OK : ESP_FAIL;
#endif
}

static void sensorarrayMainPrintVoltageHeader(void)
{
    printf("MATV_HEADER,seq,timestamp_us,duration_us,unit");
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",S%uD%u", (unsigned)s, (unsigned)d);
        }
    }
    printf("\n");
}

static void sensorarrayMainMaybePrintVoltageHeader(uint32_t sequence)
{
    int headerEvery = CONFIG_SENSORARRAY_VOLTAGE_SCAN_PRINT_HEADER_EVERY_N_FRAMES;
    if (!s_voltageHeaderPrinted || (headerEvery > 0 && (sequence % (uint32_t)headerEvery) == 0u)) {
        sensorarrayMainPrintVoltageHeader();
        s_voltageHeaderPrinted = true;
    }
}

static bool sensorarrayMainFrameHasError(const sensorarrayVoltageFrame_t *frame)
{
    if (!frame) {
        return false;
    }
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            if (frame->err[s][d] != ESP_OK) {
                return true;
            }
        }
    }
    return false;
}

static void sensorarrayMainPrintVoltageFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    printf("MATV,%" PRIu32 ",%" PRIu64 ",%" PRIu32 ",uV",
           frame->sequence,
           frame->timestampUs,
           frame->scanDurationUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%" PRIi32, frame->microvolts[s][d]);
        }
    }
    printf("\n");
}

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_RAW
static void sensorarrayMainPrintRawFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    printf("MATV_RAW,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%" PRIi32, frame->raw[s][d]);
        }
    }
    printf("\n");
}
#endif

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_GAIN
static void sensorarrayMainPrintGainFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    printf("MATV_GAIN,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%u", (unsigned)frame->gain[s][d]);
        }
    }
    printf("\n");
}
#endif

static void sensorarrayMainPrintErrFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    printf("MATV_ERR,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%ld", (long)frame->err[s][d]);
        }
    }
    printf("\n");
}

static void sensorarrayMainPrintInitSummary(const sensorarrayVoltageReadModeConfig_t *mode)
{
    printf("VOLTSCAN_INIT,mode=%s,cnName=%s,sw=%s,fdcInitSkipped=%u,rows=8,cols=8,unit=uV,"
           "dr=%u,gainDefault=%u,autoGain=%s,discardFirst=%u,oversample=%u\n",
           mode->modeName,
           mode->modeNameCn,
           mode->swName,
           mode->skipFdcInit ? 1u : 0u,
           (unsigned)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F),
           (unsigned)sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN),
           sensorarrayMainAutoGainModeName(),
           (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0) ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE);
}

static void sensorarrayMainDelayFramePeriod(const sensorarrayVoltageFrame_t *frame)
{
    uint32_t framePeriodMs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_FRAME_PERIOD_MS;
    if (framePeriodMs == 0u || !frame) {
        return;
    }

    uint64_t targetUs = (uint64_t)framePeriodMs * 1000u;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    uint64_t elapsedUs = nowUs - frame->timestampUs;
    if (elapsedUs >= targetUs) {
        return;
    }

    uint64_t remainingUs = targetUs - elapsedUs;
    if (remainingUs >= 1000u) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((remainingUs + 999u) / 1000u)));
    } else {
        esp_rom_delay_us((uint32_t)remainingUs);
    }
}

static void sensorarrayRunVoltageReadMode(const sensorarrayVoltageReadModeConfig_t *mode)
{
    esp_err_t err = sensorarrayMainInitVoltageScan(mode);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("init", err);
    }

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_POINT
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));
    printf("VOLTSCAN_GAIN,mode=per_point,points=64,headroomPercent=%u\n",
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT);
#else
    (void)sensorarrayMainBootstrapAutoGain(mode, true);
#endif
    sensorarrayMainPrintInitSummary(mode);

    while (true) {
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_FRAME
        (void)sensorarrayMainBootstrapAutoGain(mode, false);
#endif

        sensorarrayVoltageFrame_t frame = {0};
        (void)sensorarrayVoltageScanOneFrameWithSource(&s_state.ads, s_gainTable, mode->swSource, &frame);

        sensorarrayMainMaybePrintVoltageHeader(frame.sequence);
        sensorarrayMainPrintVoltageFrameCsv(&frame);
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_RAW
        sensorarrayMainPrintRawFrameCsv(&frame);
#endif
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_GAIN
        sensorarrayMainPrintGainFrameCsv(&frame);
#endif
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_ERR
        if (sensorarrayMainFrameHasError(&frame)) {
            sensorarrayMainPrintErrFrameCsv(&frame);
        }
#endif

        sensorarrayMainDelayFramePeriod(&frame);
        if ((frame.sequence & 0x0Fu) == 0u) {
            taskYIELD();
        }
    }
}

void sensorarrayRunPiezoRead(void)
{
    sensorarrayRunVoltageReadMode(&s_piezoReadMode);
}

void sensorarrayRunResistanceRead(void)
{
    sensorarrayRunVoltageReadMode(&s_resistanceReadMode);
}

// The main application entry point.
void app_main(void)
{
#if CONFIG_SENSORARRAY_APP_MODE_DEBUG
    printf("APPMODE,active=DEBUG,cnName=Debug,skipAdsInit=1,skipFdcInit=0,sw=DEBUG\n");
    sensorarrayAppRun();
#elif CONFIG_SENSORARRAY_APP_MODE_RESISTANCE_READ
    sensorarrayRunResistanceRead();
#else
    sensorarrayRunPiezoRead();
#endif
}
