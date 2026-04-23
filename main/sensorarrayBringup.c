#include "sensorarrayBringup.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayLog.h"

#define SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK 0x8000u
#define SENSORARRAY_FDC_DRIVE_CURRENT_MASK 0xF800u

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static uint32_t sensorarrayBringupPostResetDelayMs(void)
{
#ifdef CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_POST_RESET_DELAY_MS
    return (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_POST_RESET_DELAY_MS;
#else
    return 5u;
#endif
}

static uint32_t sensorarrayBringupInitSettleDelayMs(void)
{
#ifdef CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_INIT_SETTLE_DELAY_MS
    return (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_INIT_SETTLE_DELAY_MS;
#else
    return 100u;
#endif
}

static gpio_num_t sensorarrayToGpio(int gpio)
{
    return gpio < 0 ? GPIO_NUM_NC : (gpio_num_t)gpio;
}

static esp_err_t sensorarrayBringupLogAdsCoreRegisters(sensorarrayState_t *state, const char *stage)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t power = 0u;
    uint8_t iface = 0u;
    uint8_t mode2 = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    esp_err_t err = ads126xAdcReadCoreRegisters(&state->ads, &power, &iface, &mode2, &inpmux, &refmux);

    printf("tag=ADS_CORE_READBACK,stage=%s,power=0x%02X,interface=0x%02X,mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,"
           "intref=%u,vbias=%u,err=%ld,result=%s\n",
           stage ? stage : SENSORARRAY_NA,
           power,
           iface,
           mode2,
           inpmux,
           refmux,
           (unsigned)((power & ADS126X_POWER_INTREF) != 0u),
           (unsigned)((power & ADS126X_POWER_VBIAS) != 0u),
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");

    return err;
}

static esp_err_t sensorarrayInitSpi(spi_device_handle_t *outDevice)
{
    if (!outDevice) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t busCfg = {
        .mosi_io_num = CONFIG_BOARD_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_BOARD_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_BOARD_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES,
    };

#if CONFIG_SENSORARRAY_SPI_USE_DMA
    int dmaChan = SPI_DMA_CH_AUTO;
#else
    int dmaChan = 0;
#endif

    esp_err_t err = spi_bus_initialize(CONFIG_BOARD_SPI_HOST, &busCfg, dmaChan);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = CONFIG_ADS126X_SPI_CLOCK_HZ,
        .mode = 1,
        .spics_io_num = CONFIG_BOARD_ADS126X_CS_GPIO,
        .queue_size = 1,
    };

    return spi_bus_add_device(CONFIG_BOARD_SPI_HOST, &devCfg, outDevice);
}

static Fdc2214CapRefClockSource_t sensorarrayBringupFdcRefClockSource(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return FDC2214_REF_CLOCK_EXTERNAL;
#else
    return FDC2214_REF_CLOCK_INTERNAL;
#endif
}

static const char *sensorarrayBringupFdcRefClockName(Fdc2214CapRefClockSource_t source)
{
    return (source == FDC2214_REF_CLOCK_EXTERNAL) ? "external_clkin" : "internal_oscillator";
}

static sensorarrayFdcRefClockQuality_t sensorarrayBringupFdcRefClockQuality(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_REF_CLOCK_QUALITY_EXTERNAL_ASSUMED;
#else
    return SENSORARRAY_FDC_REF_CLOCK_QUALITY_NOMINAL_INTERNAL;
#endif
}

static const char *sensorarrayBringupFdcRefClockQualityName(sensorarrayFdcRefClockQuality_t quality)
{
    switch (quality) {
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_NOMINAL_INTERNAL:
        return "nominal_internal";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_ESTIMATED:
        return "estimated";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_CALIBRATED:
        return "calibrated";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_EXTERNAL_ASSUMED:
        return "external_assumed";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN:
    default:
        return "unknown";
    }
}

static bool sensorarrayBringupFdcAutoScanForChannels(uint8_t channels)
{
    return channels > 1u;
}

static uint8_t sensorarrayBringupFdcRrSequenceForChannels(uint8_t channels)
{
    if (channels <= 2u) {
        return 0u; // CH0..CH1
    }
    if (channels == 3u) {
        return 1u; // CH0..CH2
    }
    return 2u; // CH0..CH3
}

static Fdc2214CapChannelConfig_t sensorarrayBringupFdcDebugChannelProfile(uint8_t channelIndex)
{
    (void)channelIndex;

    /*
     * Explicit single-channel debug profile for deterministic CH0 bring-up:
     * - CLOCK_DIVIDERS explicitly sets both CHx_FIN_SEL and CHx_FREF_DIVIDER.
     * - Values follow datasheet-style single-channel examples and avoid implicit defaults.
     */
    return (Fdc2214CapChannelConfig_t){
        .Rcount = SENSORARRAY_FDC_DEBUG_RCOUNT_CH0,
        .SettleCount = SENSORARRAY_FDC_DEBUG_SETTLECOUNT_CH0,
        .Offset = SENSORARRAY_FDC_DEBUG_OFFSET_CH0,
        .ClockDividers = SENSORARRAY_FDC_DEBUG_CLOCK_DIVIDERS_CH0,
        .DriveCurrent = SENSORARRAY_FDC_DEBUG_DRIVE_CURRENT_CH0,
    };
}

static uint16_t sensorarrayBringupFdcExpectedMuxConfig(uint8_t channels)
{
    bool autoScan = sensorarrayBringupFdcAutoScanForChannels(channels);
    uint8_t rrSequence = sensorarrayBringupFdcRrSequenceForChannels(channels);
    uint16_t muxConfig = 0x0208u; // Reserved bits [12:3] must be 00_0100_0001b.
    if (autoScan) {
        muxConfig |= 0x8000u;
        muxConfig |= (uint16_t)((uint16_t)rrSequence << 13);
    }
    muxConfig |= (uint16_t)FDC2214_DEGLITCH_10MHZ;
    return muxConfig;
}

static uint16_t sensorarrayBringupFdcExpectedStatusConfig(void)
{
    return SENSORARRAY_FDC_STATUS_CONFIG_DEFAULT;
}

static uint16_t sensorarrayBringupFdcBuildFinalConfig(Fdc2214CapChannel_t activeChannel)
{
    Fdc2214CapConfigOptions_t config = {
        .ActiveChannel = activeChannel,
        .SleepModeEnabled = false,
        // Keep full activation drive during sensor wake-up for strict debug reproducibility.
        .SensorActivateSelLowPower = false,
        .RefClockSource = sensorarrayBringupFdcRefClockSource(),
        // Keep INTB disabled; software polling and explicit status checks are used in debug flows.
        .IntbDisabled = true,
        .HighCurrentDrive = false,
    };
    return Fdc2214CapBuildConfig(&config);
}

static esp_err_t sensorarrayBringupReadFdcReg(Fdc2214CapDevice_t *dev, uint8_t reg, uint16_t *outValue)
{
    if (!dev || !outValue) {
        return ESP_ERR_INVALID_ARG;
    }
    return Fdc2214CapReadRawRegisters(dev, reg, outValue);
}

static esp_err_t sensorarrayBringupDumpFdcInitRegisters(const BoardSupportI2cCtx_t *i2cCtx,
                                                        uint8_t i2cAddr,
                                                        const char *fdcLabel,
                                                        const char *stage,
                                                        Fdc2214CapDevice_t *dev,
                                                        uint16_t manufacturerId,
                                                        uint16_t deviceId,
                                                        uint16_t *outClockDiv0Raw,
                                                        Fdc2214CapClockDividerInfo_t *outClockDiv0Info,
                                                        bool *outClockDiv0InfoValid)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t status = 0u;
    uint16_t statusConfig = 0u;
    uint16_t config = 0u;
    uint16_t muxConfig = 0u;
    uint16_t clockDiv0 = 0u;
    uint16_t rcount0 = 0u;
    uint16_t settle0 = 0u;
    uint16_t drive0 = 0u;

    esp_err_t err = sensorarrayBringupReadFdcReg(dev, 0x18u, &status);
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x19u, &statusConfig);
    }
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x1Au, &config);
    }
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x1Bu, &muxConfig);
    }
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x14u, &clockDiv0);
    }
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x08u, &rcount0);
    }
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x10u, &settle0);
    }
    if (err == ESP_OK) {
        err = sensorarrayBringupReadFdcReg(dev, 0x1Eu, &drive0);
    }

    bool clockDivDecodeValid = false;
    Fdc2214CapClockDividerInfo_t clockDivInfo = {0};
    if (err == ESP_OK && Fdc2214CapDecodeClockDividers(clockDiv0, &clockDivInfo) == ESP_OK) {
        clockDivDecodeValid = true;
    }

    if (outClockDiv0Raw) {
        *outClockDiv0Raw = clockDiv0;
    }
    if (outClockDiv0InfoValid) {
        *outClockDiv0InfoValid = clockDivDecodeValid;
    }
    if (outClockDiv0Info && clockDivDecodeValid) {
        *outClockDiv0Info = clockDivInfo;
    }

    sensorarrayFdcRefClockQuality_t refClockQuality = sensorarrayBringupFdcRefClockQuality();
    char calibratedHzField[24] = {0};
    if (refClockQuality == SENSORARRAY_FDC_REF_CLOCK_QUALITY_CALIBRATED) {
        (void)snprintf(calibratedHzField, sizeof(calibratedHzField), "%lu", (unsigned long)SENSORARRAY_FDC_REF_CLOCK_HZ);
    } else {
        (void)snprintf(calibratedHzField, sizeof(calibratedHzField), "%s", SENSORARRAY_NA);
    }

    printf("DBGFDCINIT,stage=%s,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,status=0x%04X,"
           "statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,clockDiv0Raw=0x%04X,finSel0=%u,finDivider0=%u,"
           "frefDivider0=%u,rcount0=0x%04X,settle0=0x%04X,drive0=0x%04X,refClockSource=%s,refClockQuality=%s,"
           "refClockHzNominal=%lu,refClockHzCalibrated=%s,err=%ld,result=%s\n",
           stage ? stage : SENSORARRAY_NA,
           fdcLabel ? fdcLabel : SENSORARRAY_NA,
           i2cCtx ? (int)i2cCtx->Port : -1,
           i2cAddr,
           manufacturerId,
           deviceId,
           status,
            statusConfig,
            config,
            muxConfig,
            clockDiv0,
            clockDivDecodeValid ? (unsigned)clockDivInfo.FinSel : 0u,
            clockDivDecodeValid ? (unsigned)clockDivInfo.FinDivider : 0u,
            clockDivDecodeValid ? (unsigned)clockDivInfo.FrefDivider : 0u,
            rcount0,
            settle0,
            drive0,
            sensorarrayBringupFdcRefClockName(sensorarrayBringupFdcRefClockSource()),
            sensorarrayBringupFdcRefClockQualityName(refClockQuality),
            (unsigned long)SENSORARRAY_FDC_REF_CLOCK_HZ,
            calibratedHzField,
            (long)err,
            (err == ESP_OK) ? "ok" : "read_error");

    return err;
}

static esp_err_t sensorarrayBringupVerifyFdcActiveState(Fdc2214CapDevice_t *dev,
                                                        uint8_t channels,
                                                        uint16_t expectedStatusConfig,
                                                        uint16_t expectedConfig,
                                                        uint16_t expectedMuxConfig,
                                                        sensorarrayFdcInitDiag_t *outDiag)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapCoreRegs_t coreRegs = {0};
    esp_err_t err = Fdc2214CapReadCoreRegs(dev, &coreRegs);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapStatus_t status = {0};
    Fdc2214CapStatusHealth_t statusHealth = {0};
    err = Fdc2214CapReadStatusDecoded(dev, &status, &statusHealth);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->transportOk = false;
            outDiag->postInitHealthy = false;
        }
        return err;
    }

    bool sleepEnabled = (coreRegs.Config & SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK) != 0u;
    bool autoScanEnabled = (coreRegs.MuxConfig & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    uint8_t activeChannel = (uint8_t)((coreRegs.Config >> 14) & 0x3u);
    uint8_t expectedActiveChannel = (uint8_t)((expectedConfig >> 14) & 0x3u);
    bool converting = (!sleepEnabled) && (autoScanEnabled || (activeChannel == expectedActiveChannel));

    bool unreadPresent = false;
    for (uint8_t ch = 0u; ch < channels && ch < 4u; ++ch) {
        unreadPresent = unreadPresent || status.UnreadConversion[ch];
    }

    bool configReadbackOk = (coreRegs.StatusConfig == expectedStatusConfig) &&
                            (coreRegs.Config == expectedConfig) &&
                            (coreRegs.MuxConfig == expectedMuxConfig);
    bool initHealthy = (!sleepEnabled) &&
                       converting &&
                       unreadPresent &&
                       !statusHealth.WatchdogFault &&
                       !statusHealth.AmplitudeFault &&
                       configReadbackOk;

    if (outDiag) {
        outDiag->transportOk = true;
        outDiag->configReadbackOk = configReadbackOk;
        outDiag->postInitConverting = converting;
        outDiag->postInitUnreadPresent = unreadPresent;
        outDiag->postInitStatusWatchdogFault = statusHealth.WatchdogFault;
        outDiag->postInitStatusAmplitudeFault = statusHealth.AmplitudeFault;
        outDiag->postInitHealthy = initHealthy;
    }

    printf("DBGFDCINIT_VERIFY,sleep=%u,autoscan=%u,activeChannel=%u,converting=%u,unreadPresent=%u,"
           "statusWatchdog=%u,statusAmplitude=%u,status=0x%04X,statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,"
           "expectedStatusConfig=0x%04X,expectedConfig=0x%04X,expectedMuxConfig=0x%04X,result=%s\n",
           sleepEnabled ? 1u : 0u,
           autoScanEnabled ? 1u : 0u,
           (unsigned)activeChannel,
           converting ? 1u : 0u,
           unreadPresent ? 1u : 0u,
           statusHealth.WatchdogFault ? 1u : 0u,
           statusHealth.AmplitudeFault ? 1u : 0u,
           coreRegs.Status,
           coreRegs.StatusConfig,
           coreRegs.Config,
           coreRegs.MuxConfig,
           expectedStatusConfig,
           expectedConfig,
           expectedMuxConfig,
           initHealthy
               ? "ok"
               : "unhealthy_or_mismatch");

    if (sleepEnabled || !converting) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!configReadbackOk) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (statusHealth.AnyFault) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t sensorarrayBringupProbeFdcAddress(const BoardSupportI2cCtx_t *i2cCtx,
                                            uint8_t i2cAddr,
                                            uint16_t *outManufacturerId)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = SENSORARRAY_FDC_REG_MANUFACTURER_ID;
    uint8_t rx[2] = {0};
    esp_err_t err = boardSupportI2cWriteRead((void *)i2cCtx, i2cAddr, &reg, sizeof(reg), rx, sizeof(rx));
    if (err == ESP_OK && outManufacturerId) {
        *outManufacturerId = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    }
    return err;
}

esp_err_t sensorarrayBringupReadFdcIdsRaw(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint16_t *outManufacturerId,
                                          uint16_t *outDeviceId)
{
    if (!i2cCtx || !outManufacturerId || !outDeviceId) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = SENSORARRAY_FDC_REG_MANUFACTURER_ID;
    uint8_t rx[2] = {0};
    esp_err_t err = boardSupportI2cWriteRead((void *)i2cCtx, i2cAddr, &reg, sizeof(reg), rx, sizeof(rx));
    if (err != ESP_OK) {
        return err;
    }
    *outManufacturerId = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);

    reg = (uint8_t)(SENSORARRAY_FDC_REG_MANUFACTURER_ID + 1u);
    err = boardSupportI2cWriteRead((void *)i2cCtx, i2cAddr, &reg, sizeof(reg), rx, sizeof(rx));
    if (err != ESP_OK) {
        return err;
    }
    *outDeviceId = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);

    return ESP_OK;
}

const char *sensorarrayBringupFdcDiscoveryStatusName(sensorarrayFdcDiscoveryStatus_t status)
{
    switch (status) {
    case SENSORARRAY_FDC_DISCOVERY_NO_ACK:
        return "no_ack";
    case SENSORARRAY_FDC_DISCOVERY_ACK_BUT_READ_FAILED:
        return "ack_but_read_failed";
    case SENSORARRAY_FDC_DISCOVERY_ID_OK:
        return "id_ok";
    case SENSORARRAY_FDC_DISCOVERY_UNEXPECTED_ID:
    default:
        return "unexpected_id";
    }
}

static const char *sensorarrayBringupStartupProbeStatus(const sensorarrayFdcDeviceState_t *fdcState,
                                                        uint8_t probedAddr,
                                                        const sensorarrayFdcProbeDiag_t *probeDiag,
                                                        esp_err_t *outErr)
{
    if (outErr) {
        *outErr = ESP_OK;
    }
    if (!fdcState || !probeDiag) {
        if (outErr) {
            *outErr = ESP_ERR_INVALID_ARG;
        }
        return "probe_invalid_args";
    }

    bool expectedAddr = (probedAddr == fdcState->i2cAddr);
    switch (probeDiag->status) {
    case SENSORARRAY_FDC_DISCOVERY_NO_ACK:
        if (expectedAddr) {
            if (outErr) {
                *outErr = probeDiag->ackErr;
            }
            return "probe_expected_addr_no_ack";
        }
        return "expected_probe_absent";

    case SENSORARRAY_FDC_DISCOVERY_ACK_BUT_READ_FAILED:
        if (outErr) {
            *outErr = probeDiag->idErr;
        }
        if (probeDiag->idErr == ESP_FAIL) {
            return expectedAddr ? "readback_nack" : "probe_unexpected_addr_readback_nack";
        }
        return expectedAddr ? "probe_expected_addr_read_failed" : "probe_unexpected_addr_read_failed";

    case SENSORARRAY_FDC_DISCOVERY_ID_OK:
        return expectedAddr ? "probe_expected_addr_ok" : "probe_unexpected_addr_present";

    case SENSORARRAY_FDC_DISCOVERY_UNEXPECTED_ID:
    default:
        if (outErr) {
            *outErr = ESP_ERR_INVALID_RESPONSE;
        }
        return expectedAddr ? "probe_expected_addr_unexpected_id" : "probe_unexpected_addr_unexpected_id";
    }
}

static void sensorarrayBringupInitFdcProbeDiag(sensorarrayFdcProbeDiag_t *diag)
{
    if (!diag) {
        return;
    }

    *diag = (sensorarrayFdcProbeDiag_t){
        .status = SENSORARRAY_FDC_DISCOVERY_NO_ACK,
        .ackErr = ESP_ERR_INVALID_STATE,
        .idErr = ESP_ERR_INVALID_STATE,
        .ack = false,
        .haveManufacturerId = false,
        .haveDeviceId = false,
        .manufacturerId = 0u,
        .deviceId = 0u,
    };
}

esp_err_t sensorarrayBringupProbeFdcCandidate(const BoardSupportI2cCtx_t *i2cCtx,
                                              uint8_t i2cAddr,
                                              sensorarrayFdcProbeDiag_t *outDiag)
{
    if (!i2cCtx || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayBringupInitFdcProbeDiag(outDiag);

    outDiag->ackErr = boardSupportI2cProbeAddress(i2cCtx, i2cAddr);
    if (outDiag->ackErr != ESP_OK) {
        outDiag->status = SENSORARRAY_FDC_DISCOVERY_NO_ACK;
        return outDiag->ackErr;
    }

    outDiag->ack = true;

    uint16_t manufacturerId = 0u;
    uint16_t deviceId = 0u;
    outDiag->idErr = sensorarrayBringupReadFdcIdsRaw(i2cCtx, i2cAddr, &manufacturerId, &deviceId);
    if (outDiag->idErr != ESP_OK) {
        outDiag->status = SENSORARRAY_FDC_DISCOVERY_ACK_BUT_READ_FAILED;
        return outDiag->idErr;
    }

    outDiag->haveManufacturerId = true;
    outDiag->haveDeviceId = true;
    outDiag->manufacturerId = manufacturerId;
    outDiag->deviceId = deviceId;

    if (manufacturerId == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID &&
        deviceId == SENSORARRAY_FDC_EXPECTED_DEVICE_ID) {
        outDiag->status = SENSORARRAY_FDC_DISCOVERY_ID_OK;
        return ESP_OK;
    }

    outDiag->status = SENSORARRAY_FDC_DISCOVERY_UNEXPECTED_ID;
    return ESP_OK;
}

uint8_t sensorarrayBringupNormalizeFdcChannels(uint8_t channels)
{
    if (channels == 0u) {
        channels = 1u;
    }
    if (channels > 4u) {
        channels = 4u;
    }
    return channels;
}

void sensorarrayBringupResetFdcState(sensorarrayFdcDeviceState_t *fdcState,
                                     const char *label,
                                     uint8_t i2cAddr)
{
    if (!fdcState) {
        return;
    }

    fdcState->label = label;
    fdcState->i2cCtx = NULL;
    fdcState->i2cAddr = i2cAddr;
    fdcState->handle = NULL;
    fdcState->ready = false;
    fdcState->haveIds = false;
    fdcState->manufacturerId = 0;
    fdcState->deviceId = 0;
    fdcState->configVerified = false;
    fdcState->refClockKnown = false;
    fdcState->refClockSource = FDC2214_REF_CLOCK_INTERNAL;
    fdcState->refClockQuality = SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN;
    fdcState->refClockIsCalibrated = false;
    fdcState->refClockHz = 0u;
    fdcState->refClockHzNominal = 0u;
    fdcState->refClockHzCalibrated = 0u;
    fdcState->channel0ClockDividersRaw = 0u;
    fdcState->channel0ClockDividerValid = false;
    fdcState->channel0ClockDividerInfo = (Fdc2214CapClockDividerInfo_t){0};
    fdcState->statusConfigReg = 0u;
    fdcState->configReg = 0u;
    fdcState->muxConfigReg = 0u;
}

bool sensorarrayBringupParseI2cAddress(uint32_t configuredAddress, uint8_t *outAddress)
{
    if (!outAddress || configuredAddress > 0x7Fu) {
        return false;
    }
    *outAddress = (uint8_t)configuredAddress;
    return true;
}

esp_err_t sensorarrayBringupAdsSetRefMux(sensorarrayState_t *state, uint8_t refmuxValue)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads126xAdcSetRefMux(&state->ads, refmuxValue);
    if (err == ESP_OK) {
        state->adsRefMux = refmuxValue;
        state->adsRefMuxValid = true;
        sensorarrayLogDbgExtraSetRefMux(refmuxValue);
    }
    return err;
}

esp_err_t sensorarrayBringupInitAds(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayInitSpi(&state->spiDevice);
    if (err != ESP_OK) {
        return err;
    }

    ads126xAdcConfig_t cfg = {0};
    cfg.spiDevice = state->spiDevice;
    cfg.drdyGpio = sensorarrayToGpio(CONFIG_BOARD_ADS126X_DRDY_GPIO);
    cfg.resetGpio = sensorarrayToGpio(CONFIG_BOARD_ADS126X_RESET_GPIO);
#if CONFIG_SENSORARRAY_ADS1262
    cfg.forcedType = ADS126X_DEVICE_ADS1262;
#elif CONFIG_SENSORARRAY_ADS1263
    cfg.forcedType = ADS126X_DEVICE_ADS1263;
#else
    cfg.forcedType = ADS126X_DEVICE_AUTO;
#endif
    cfg.crcMode = ADS126X_CRC_OFF;
    cfg.enableStatusByte = false;
    cfg.enableInternalRef = true;
    cfg.vrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV;
    cfg.pgaGain = 1;
    cfg.dataRateDr = 0;

    err = ads126xAdcInit(&state->ads, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcStartAdc1(&state->ads);
    if (err != ESP_OK) {
        return err;
    }
    state->adsAdc1Running = true;

    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(&state->ads, &raw, NULL);
}

esp_err_t sensorarrayBringupPrepareAdsRefPath(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads126xAdcConfigure(&state->ads, true, false, ADS126X_CRC_OFF, 1, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcSetVbiasEnabled(&state->ads, true);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayBringupAdsSetRefMux(state, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayDelayMs(SENSORARRAY_REF_SETTLE_MS);

    err = sensorarrayBringupLogAdsCoreRegisters(state, "prepare_ref_bias_settled");
    if (err != ESP_OK) {
        return err;
    }

    uint8_t power = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    err = ads126xAdcReadCoreRegisters(&state->ads, &power, NULL, NULL, &inpmux, &refmux);
    if (err != ESP_OK) {
        return err;
    }
    const bool intrefOk = ((power & ADS126X_POWER_INTREF) != 0u);
    const bool vbiasOk = ((power & ADS126X_POWER_VBIAS) != 0u);
    const bool refmuxOk = (refmux == 0x00u);
    printf("tag=ADS_CORE_EXPECT,stage=prepare_ref_bias_settled,intrefExpected=1,intrefReadback=%u,"
           "vbiasExpected=1,vbiasReadback=%u,refmuxExpected=0x00,refmuxReadback=0x%02X,inpmuxReadback=0x%02X,"
           "result=%s\n",
           intrefOk ? 1u : 0u,
           vbiasOk ? 1u : 0u,
           refmux,
           inpmux,
           (intrefOk && vbiasOk && refmuxOk) ? "ok" : "mismatch");
    if (!(intrefOk && vbiasOk && refmuxOk)) {
        return ESP_ERR_INVALID_STATE;
    }

    err = ads126xAdcStartAdc1(&state->ads);
    if (err != ESP_OK) {
        return err;
    }
    state->adsAdc1Running = true;

    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(&state->ads, &raw, NULL);
}

void sensorarrayBringupProbeFdcBus(const sensorarrayFdcDeviceState_t *fdcState)
{
    if (!fdcState || !fdcState->i2cCtx) {
        return;
    }

#if !CONFIG_SENSORARRAY_FDC_STARTUP_PROBE
    sensorarrayLogStartupFdc("fdc_probe",
                             fdcState,
                             ESP_OK,
                             "probe_disabled",
                             (int32_t)fdcState->i2cAddr,
                             false,
                             0u,
                             0u,
                             "probe_regs_0x7E_0x7F");
    return;
#else
    const bool probeAllAddresses = (CONFIG_SENSORARRAY_FDC_STARTUP_PROBE_ALL_ADDR != 0);
    if (!probeAllAddresses) {
        sensorarrayLogStartupFdc("fdc_probe",
                                 fdcState,
                                 ESP_OK,
                                 "probe_single_expected_addr",
                                 (int32_t)fdcState->i2cAddr,
                                 false,
                                 0u,
                                 0u,
                                 "probe_regs_0x7E_0x7F");

        sensorarrayFdcProbeDiag_t probeDiag = {0};
        (void)sensorarrayBringupProbeFdcCandidate(fdcState->i2cCtx, fdcState->i2cAddr, &probeDiag);
        esp_err_t effectiveErr = ESP_OK;
        const char *status = sensorarrayBringupStartupProbeStatus(fdcState,
                                                                  fdcState->i2cAddr,
                                                                  &probeDiag,
                                                                  &effectiveErr);
        sensorarrayLogStartupFdc("fdc_probe",
                                 fdcState,
                                 effectiveErr,
                                 status,
                                 (int32_t)fdcState->i2cAddr,
                                 probeDiag.haveManufacturerId && probeDiag.haveDeviceId,
                                 probeDiag.manufacturerId,
                                 probeDiag.deviceId,
                                 "probe_regs_0x7E_0x7F");
        return;
    }

    static const uint8_t probeAddresses[] = {
        SENSORARRAY_FDC_I2C_ADDR_LOW,
        SENSORARRAY_FDC_I2C_ADDR_HIGH,
    };

    for (size_t i = 0; i < (sizeof(probeAddresses) / sizeof(probeAddresses[0])); ++i) {
        sensorarrayFdcDeviceState_t probeState = *fdcState;
        probeState.i2cAddr = probeAddresses[i];

        sensorarrayFdcProbeDiag_t probeDiag = {0};
        (void)sensorarrayBringupProbeFdcCandidate(fdcState->i2cCtx, probeState.i2cAddr, &probeDiag);
        esp_err_t effectiveErr = ESP_OK;
        const char *probeStatus = sensorarrayBringupStartupProbeStatus(fdcState,
                                                                        probeState.i2cAddr,
                                                                        &probeDiag,
                                                                        &effectiveErr);

        sensorarrayLogStartupFdc("fdc_probe",
                                 &probeState,
                                 effectiveErr,
                                 probeStatus,
                                 (int32_t)probeState.i2cAddr,
                                 probeDiag.haveManufacturerId && probeDiag.haveDeviceId,
                                 probeDiag.manufacturerId,
                                 probeDiag.deviceId,
                                 "probe_regs_0x7E_0x7F");
    }
#endif
}

void sensorarrayBringupInitFdcDiag(sensorarrayFdcInitDiag_t *diag)
{
    if (!diag) {
        return;
    }
    diag->status = "unknown";
    diag->haveIds = false;
    diag->manufacturerId = 0;
    diag->deviceId = 0;
    diag->transportOk = false;
    diag->idOk = false;
    diag->configVerified = false;
    diag->configReadbackOk = false;
    diag->postInitConverting = false;
    diag->postInitUnreadPresent = false;
    diag->postInitStatusWatchdogFault = false;
    diag->postInitStatusAmplitudeFault = false;
    diag->postInitHealthy = false;
    diag->channel0DriveCurrentReq = 0u;
    diag->channel0DriveCurrentApplied = 0u;
    diag->channel0DriveCurrentMasked = false;
    diag->refClockKnown = false;
    diag->refClockSource = FDC2214_REF_CLOCK_INTERNAL;
    diag->refClockQuality = SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN;
    diag->refClockIsCalibrated = false;
    diag->refClockHz = 0u;
    diag->refClockHzNominal = 0u;
    diag->refClockHzCalibrated = 0u;
    diag->channel0ClockDividersRaw = 0u;
    diag->channel0ClockDividerValid = false;
    diag->channel0ClockDividerInfo = (Fdc2214CapClockDividerInfo_t){0};
    diag->statusConfigReg = 0u;
    diag->configReg = 0u;
    diag->muxConfigReg = 0u;
    diag->detail = 0;
}

void sensorarrayBringupApplyFdcInitResult(sensorarrayFdcDeviceState_t *fdcState,
                                          Fdc2214CapDevice_t *deviceHandle,
                                          esp_err_t initErr,
                                          const sensorarrayFdcInitDiag_t *diag)
{
    if (!fdcState || !diag) {
        return;
    }

    fdcState->handle = (initErr == ESP_OK) ? deviceHandle : NULL;
    fdcState->ready = (initErr == ESP_OK) && (deviceHandle != NULL);
    fdcState->haveIds = diag->haveIds;
    fdcState->manufacturerId = diag->manufacturerId;
    fdcState->deviceId = diag->deviceId;
    fdcState->configVerified = diag->configVerified;
    fdcState->refClockKnown = diag->refClockKnown;
    fdcState->refClockSource = diag->refClockSource;
    fdcState->refClockQuality = diag->refClockQuality;
    fdcState->refClockIsCalibrated = diag->refClockIsCalibrated;
    fdcState->refClockHz = diag->refClockHz;
    fdcState->refClockHzNominal = diag->refClockHzNominal;
    fdcState->refClockHzCalibrated = diag->refClockHzCalibrated;
    fdcState->channel0ClockDividersRaw = diag->channel0ClockDividersRaw;
    fdcState->channel0ClockDividerValid = diag->channel0ClockDividerValid;
    fdcState->channel0ClockDividerInfo = diag->channel0ClockDividerInfo;
    fdcState->statusConfigReg = diag->statusConfigReg;
    fdcState->configReg = diag->configReg;
    fdcState->muxConfigReg = diag->muxConfigReg;
}

esp_err_t sensorarrayBringupInitFdcDevice(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint8_t channels,
                                          Fdc2214CapDevice_t **outDev,
                                          sensorarrayFdcInitDiag_t *outDiag)
{
    sensorarrayBringupInitFdcDiag(outDiag);

#if !CONFIG_FDC2214CAP_ENABLE
    (void)i2cCtx;
    (void)i2cAddr;
    (void)channels;
    (void)outDev;
    if (outDiag) {
        outDiag->status = "component_disabled";
    }
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!i2cCtx || !outDev) {
        if (outDiag) {
            outDiag->status = "invalid_args";
        }
        return ESP_ERR_INVALID_ARG;
    }

    *outDev = NULL;

    Fdc2214CapBusConfig_t busCfg = {
        .I2cAddress7 = i2cAddr,
        .UserCtx = (void *)i2cCtx,
        .WriteRead = boardSupportI2cWriteRead,
        .Write = boardSupportI2cWrite,
        .GetBusGeneration = boardSupportI2cGetBusGeneration,
        .IntGpio = -1,
    };

    Fdc2214CapDevice_t *dev = NULL;
    esp_err_t err = Fdc2214CapCreate(&busCfg, &dev);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "create_failure";
        }
        return err;
    }

    err = Fdc2214CapReset(dev);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "reset_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }
    if (outDiag) {
        outDiag->transportOk = true;
    }
    sensorarrayDelayMs(sensorarrayBringupPostResetDelayMs());

    uint16_t manufacturer = 0;
    uint16_t deviceId = 0;
    err = Fdc2214CapReadId(dev, &manufacturer, &deviceId);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "read_id_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    if (outDiag) {
        outDiag->haveIds = true;
        outDiag->manufacturerId = manufacturer;
        outDiag->deviceId = deviceId;
        outDiag->idOk = true;
    }

    if (manufacturer != SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID ||
        deviceId != SENSORARRAY_FDC_EXPECTED_DEVICE_ID) {
        if (outDiag) {
            outDiag->status = "id_mismatch";
            outDiag->idOk = false;
        }
        Fdc2214CapDestroy(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    channels = sensorarrayBringupNormalizeFdcChannels(channels);
    const bool autoScan = sensorarrayBringupFdcAutoScanForChannels(channels);
    const uint8_t rrSequence = sensorarrayBringupFdcRrSequenceForChannels(channels);
    const uint16_t expectedStatusConfig = sensorarrayBringupFdcExpectedStatusConfig();
    const uint16_t expectedMuxConfig = sensorarrayBringupFdcExpectedMuxConfig(channels);
    const uint16_t finalConfig = sensorarrayBringupFdcBuildFinalConfig(FDC2214_CH0);
    const char *fdcLabel = (i2cAddr == SENSORARRAY_FDC_I2C_ADDR_LOW) ? "secondary_selb_side" : "primary_sela_side";
    bool driveCurrentWarning = false;

    // CONFIG must be written in sleep before channel/mux writes to avoid ambiguous run-state transitions.
    err = Fdc2214CapEnterSleep(dev, finalConfig);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "enter_sleep_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    for (uint8_t ch = 0; ch < channels; ++ch) {
        Fdc2214CapChannelConfig_t chCfg = sensorarrayBringupFdcDebugChannelProfile(ch);
        Fdc2214CapChannelConfigResult_t cfgResult = FDC2214_CHANNEL_CONFIG_RESULT_OK;
        uint16_t cfgDriveReadback = 0u;
        err = Fdc2214CapConfigureChannelWithResult(dev,
                                                   (Fdc2214CapChannel_t)ch,
                                                   &chCfg,
                                                   &cfgResult,
                                                   &cfgDriveReadback);
        if (err != ESP_OK) {
            if (outDiag) {
                outDiag->status = "channel_config_failure";
                outDiag->detail = (int32_t)ch;
            }
            Fdc2214CapDestroy(dev);
            return err;
        }
        if (cfgResult == FDC2214_CHANNEL_CONFIG_RESULT_WARN_DRIVE_CURRENT_MISMATCH) {
            driveCurrentWarning = true;
            printf("DBGFDCINIT_WARN,stage=channel_config,channel=%u,driveReq=0x%04X,driveNorm=0x%04X,driveReadback=0x%04X,"
                   "status=drive_current_effective_mismatch_continue\n",
                   (unsigned)ch,
                   chCfg.DriveCurrent,
                   (uint16_t)(chCfg.DriveCurrent & SENSORARRAY_FDC_DRIVE_CURRENT_MASK),
                   cfgDriveReadback);
        }
        if (outDiag && ch == 0u) {
            outDiag->channel0DriveCurrentReq = (uint16_t)(chCfg.DriveCurrent & SENSORARRAY_FDC_DRIVE_CURRENT_MASK);
            outDiag->channel0DriveCurrentApplied = (uint16_t)(cfgDriveReadback & SENSORARRAY_FDC_DRIVE_CURRENT_MASK);
            outDiag->channel0DriveCurrentMasked = (outDiag->channel0DriveCurrentReq != outDiag->channel0DriveCurrentApplied);
        }

        Fdc2214CapChannelVerifyResult_t verifyResult = FDC2214_CHANNEL_VERIFY_RESULT_OK;
        uint16_t verifyDriveReadback = 0u;
        err = Fdc2214CapReadbackVerifyChannelConfigWithResult(dev,
                                                              (Fdc2214CapChannel_t)ch,
                                                              &chCfg,
                                                              &verifyResult,
                                                              &verifyDriveReadback);
        if (err != ESP_OK) {
            if (outDiag) {
                outDiag->status = "channel_readback_mismatch";
                outDiag->detail = (int32_t)ch;
            }
            Fdc2214CapDestroy(dev);
            return err;
        }
        if (verifyResult == FDC2214_CHANNEL_VERIFY_RESULT_WARN_DRIVE_CURRENT_MISMATCH) {
            driveCurrentWarning = true;
            printf("DBGFDCINIT_WARN,stage=channel_readback,channel=%u,driveReq=0x%04X,driveNorm=0x%04X,"
                   "driveReadback=0x%04X,status=drive_current_effective_mismatch_continue\n",
                   (unsigned)ch,
                   chCfg.DriveCurrent,
                   (uint16_t)(chCfg.DriveCurrent & SENSORARRAY_FDC_DRIVE_CURRENT_MASK),
                   verifyDriveReadback);
        }
        if (outDiag && ch == 0u) {
            outDiag->channel0DriveCurrentApplied = (uint16_t)(verifyDriveReadback & SENSORARRAY_FDC_DRIVE_CURRENT_MASK);
            outDiag->channel0DriveCurrentMasked =
                (outDiag->channel0DriveCurrentReq != outDiag->channel0DriveCurrentApplied);
        }
    }

    err = Fdc2214CapSetStatusConfig(dev, expectedStatusConfig);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "status_config_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    err = Fdc2214CapSetMuxConfig(dev, autoScan, rrSequence, FDC2214_DEGLITCH_10MHZ);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "mux_config_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    /*
     * Final CONFIG write is intentionally last:
     * it clears SLEEP_MODE_EN and starts conversion with a known-good bit pattern.
     */
    err = Fdc2214CapExitSleep(dev, finalConfig);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "exit_sleep_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }
    sensorarrayDelayMs(sensorarrayBringupInitSettleDelayMs());

    err = sensorarrayBringupVerifyFdcActiveState(dev,
                                                 channels,
                                                 expectedStatusConfig,
                                                 finalConfig,
                                                 expectedMuxConfig,
                                                 outDiag);
    if (err != ESP_OK) {
        (void)sensorarrayBringupDumpFdcInitRegisters(i2cCtx,
                                                     i2cAddr,
                                                     fdcLabel,
                                                     "verify_failed",
                                                     dev,
                                                     manufacturer,
                                                     deviceId,
                                                     NULL,
                                                     NULL,
                                                     NULL);
        if (outDiag) {
            outDiag->status = "active_verify_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    uint16_t clockDiv0Raw = 0u;
    Fdc2214CapClockDividerInfo_t clockDiv0Info = {0};
    bool clockDiv0InfoValid = false;
    err = sensorarrayBringupDumpFdcInitRegisters(i2cCtx,
                                                 i2cAddr,
                                                 fdcLabel,
                                                 "post_init",
                                                 dev,
                                                 manufacturer,
                                                 deviceId,
                                                 &clockDiv0Raw,
                                                 &clockDiv0Info,
                                                 &clockDiv0InfoValid);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "diagnostic_dump_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    if (outDiag) {
        printf("DBGFDCINIT_HEALTH,fdcDev=%s,i2cAddr=0x%02X,transportOk=%u,idOk=%u,configReadbackOk=%u,converting=%u,"
               "unread=%u,watchdog=%u,amplitude=%u,driveReq=0x%04X,driveApplied=0x%04X,result=%s\n",
               fdcLabel,
               i2cAddr,
               outDiag->transportOk ? 1u : 0u,
               outDiag->idOk ? 1u : 0u,
               outDiag->configReadbackOk ? 1u : 0u,
               outDiag->postInitConverting ? 1u : 0u,
               outDiag->postInitUnreadPresent ? 1u : 0u,
               outDiag->postInitStatusWatchdogFault ? 1u : 0u,
               outDiag->postInitStatusAmplitudeFault ? 1u : 0u,
               outDiag->channel0DriveCurrentReq,
               outDiag->channel0DriveCurrentApplied,
               outDiag->postInitHealthy ? "healthy" : "warning");
    }

    *outDev = dev;
    if (outDiag) {
        if (!outDiag->postInitHealthy && driveCurrentWarning) {
            outDiag->status = "ok_with_post_init_warning_and_drive_warning";
        } else if (!outDiag->postInitHealthy) {
            outDiag->status = "ok_with_post_init_warning";
        } else {
            outDiag->status = driveCurrentWarning ? "ok_with_drive_current_warning" : "ok";
        }
        outDiag->transportOk = true;
        outDiag->idOk = true;
        outDiag->configVerified = true;
        outDiag->configReadbackOk = true;
        // "known" means source/path is known; exact calibrated Hz is tracked separately.
        outDiag->refClockKnown = true;
        outDiag->refClockSource = sensorarrayBringupFdcRefClockSource();
        outDiag->refClockQuality = sensorarrayBringupFdcRefClockQuality();
        outDiag->refClockIsCalibrated = (outDiag->refClockQuality == SENSORARRAY_FDC_REF_CLOCK_QUALITY_CALIBRATED);
        outDiag->refClockHz = SENSORARRAY_FDC_REF_CLOCK_HZ;
        outDiag->refClockHzNominal = SENSORARRAY_FDC_REF_CLOCK_HZ;
        outDiag->refClockHzCalibrated = 0u;
        outDiag->channel0ClockDividersRaw = clockDiv0Raw;
        outDiag->channel0ClockDividerValid = clockDiv0InfoValid;
        outDiag->channel0ClockDividerInfo = clockDiv0InfoValid ? clockDiv0Info : (Fdc2214CapClockDividerInfo_t){0};
        outDiag->statusConfigReg = expectedStatusConfig;
        outDiag->configReg = finalConfig;
        outDiag->muxConfigReg = expectedMuxConfig;
        outDiag->detail = (int32_t)channels;
    }
    return ESP_OK;
#endif
}

esp_err_t sensorarrayBringupInitFdcSingleChannel(const BoardSupportI2cCtx_t *i2cCtx,
                                                  uint8_t i2cAddr,
                                                  Fdc2214CapChannel_t channel,
                                                  Fdc2214CapDevice_t **outDev,
                                                  sensorarrayFdcInitDiag_t *outDiag)
{
    sensorarrayBringupInitFdcDiag(outDiag);

#if !CONFIG_FDC2214CAP_ENABLE
    (void)i2cCtx;
    (void)i2cAddr;
    (void)channel;
    (void)outDev;
    if (outDiag) {
        outDiag->status = "component_disabled";
    }
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!i2cCtx || !outDev || channel < FDC2214_CH0 || channel > FDC2214_CH3) {
        if (outDiag) {
            outDiag->status = "invalid_args";
        }
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Reuse the validated bring-up flow so reset/ID checks, sleep sequencing,
     * explicit channel profile, and readback verification stay centralized.
     * DRIVE_CURRENT effective-IDRIVE mismatches are warning-only in this flow
     * to preserve downstream debug visibility.
     * channel+1 ensures target CHx register is configured before forcing single-channel mode.
     */
    uint8_t channelsToConfigure = (uint8_t)channel + 1u;
    esp_err_t err = sensorarrayBringupInitFdcDevice(i2cCtx,
                                                    i2cAddr,
                                                    channelsToConfigure,
                                                    outDev,
                                                    outDiag);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapSetSingleChannelMode(*outDev, channel);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "single_channel_mode_failure";
            outDiag->detail = (int32_t)channel;
        }
        Fdc2214CapDestroy(*outDev);
        *outDev = NULL;
        return err;
    }
    sensorarrayDelayMs(sensorarrayBringupInitSettleDelayMs());

    uint16_t expectedStatusConfig = sensorarrayBringupFdcExpectedStatusConfig();
    uint16_t expectedMuxConfig = sensorarrayBringupFdcExpectedMuxConfig(1u);
    uint16_t expectedConfig = sensorarrayBringupFdcBuildFinalConfig(channel);
    err = sensorarrayBringupVerifyFdcActiveState(*outDev,
                                                 1u,
                                                 expectedStatusConfig,
                                                 expectedConfig,
                                                 expectedMuxConfig,
                                                 outDiag);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "single_channel_verify_failure";
            outDiag->detail = (int32_t)channel;
        }
        Fdc2214CapDestroy(*outDev);
        *outDev = NULL;
        return err;
    }

    if (outDiag) {
        if (!outDiag->postInitHealthy && outDiag->channel0DriveCurrentMasked) {
            outDiag->status = "ok_single_channel_with_post_init_warning_and_drive_warning";
        } else if (!outDiag->postInitHealthy) {
            outDiag->status = "ok_single_channel_with_post_init_warning";
        } else if (outDiag->channel0DriveCurrentMasked) {
            outDiag->status = "ok_single_channel_with_drive_warning";
        } else {
            outDiag->status = "ok_single_channel";
        }
        outDiag->transportOk = true;
        outDiag->idOk = true;
        outDiag->configVerified = true;
        outDiag->configReadbackOk = true;
        outDiag->statusConfigReg = expectedStatusConfig;
        outDiag->configReg = expectedConfig;
        outDiag->muxConfigReg = expectedMuxConfig;
        outDiag->detail = (int32_t)channel;
    }
    return ESP_OK;
#endif
}

esp_err_t sensorarrayBringupReinitSecondaryFdcForS5d5(sensorarrayState_t *state,
                                                       sensorarrayFdcInitDiag_t *outDiag)
{
    if (!state || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcDeviceState_t *fdcState = &state->fdcSecondary;
    sensorarrayBringupInitFdcDiag(outDiag);
    fdcState->i2cCtx = boardSupportGetI2c1Ctx();
    fdcState->i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW;

    if (!fdcState->i2cCtx) {
        outDiag->status = "i2c1_ctx_missing";
        return ESP_ERR_INVALID_STATE;
    }

    if (fdcState->handle) {
        (void)Fdc2214CapDestroy(fdcState->handle);
        fdcState->handle = NULL;
    }
    fdcState->ready = false;

    printf("DBGS5D5_FDC,stage=create,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,status=begin\n",
           fdcState->label ? fdcState->label : SENSORARRAY_NA,
           (int)fdcState->i2cCtx->Port,
           fdcState->i2cAddr);

    bool preSclHigh = true;
    bool preSdaHigh = true;
    esp_err_t precheckErr = boardSupportI2cCheckLines(fdcState->i2cCtx, &preSclHigh, &preSdaHigh);
    printf("DBGS5D5_I2C,stage=precheck,port=%d,sda=%d,scl=%d,addr=0x%02X,reason=startup_soft_reinit,sclHigh=%d,sdaHigh=%d,err=%ld\n",
           (int)fdcState->i2cCtx->Port,
           SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO,
           SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO,
           fdcState->i2cAddr,
           (precheckErr == ESP_OK) ? (preSclHigh ? 1 : 0) : -1,
           (precheckErr == ESP_OK) ? (preSdaHigh ? 1 : 0) : -1,
           (long)precheckErr);

    if (precheckErr == ESP_OK && preSclHigh && preSdaHigh) {
        /*
         * Idle-high means the bus is electrically free. Startup reinit must not
         * trigger manual pulse recovery or driver reinstall in this condition.
         */
        printf("DBGS5D5_I2C,stage=startup_recover_skip,reason=idle_high_bus,action=no_recovery\n");
    } else {
        BoardSupportI2cRecoveryReason_t recoverReason = BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT;
        BoardSupportI2cRecoveryLevel_t recoverLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
        if (precheckErr == ESP_OK && (!preSclHigh || !preSdaHigh)) {
            recoverReason = BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW;
            recoverLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
        }
        BoardSupportI2cRecoveryResult_t recoverResult = {0};
        esp_err_t recoverErr = boardSupportI2cRecover(fdcState->i2cCtx,
                                                      recoverReason,
                                                      recoverLevel,
                                                      fdcState->i2cAddr,
                                                      0u,
                                                      &recoverResult);
        if (recoverErr != ESP_OK) {
            outDiag->status = "startup_recover_failed";
            return recoverErr;
        }
    }

    Fdc2214CapDevice_t *newHandle = NULL;
    esp_err_t err = sensorarrayBringupInitFdcSingleChannel(fdcState->i2cCtx,
                                                            fdcState->i2cAddr,
                                                            FDC2214_CH0,
                                                            &newHandle,
                                                            outDiag);
    sensorarrayBringupApplyFdcInitResult(fdcState, newHandle, err, outDiag);
    printf("DBGS5D5_FDC,stage=reset,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,err=%ld,status=%s\n",
           fdcState->label ? fdcState->label : SENSORARRAY_NA,
           (int)fdcState->i2cCtx->Port,
           fdcState->i2cAddr,
           (long)err,
           (err == ESP_OK) ? "ok" : "init_failed");
    if (err != ESP_OK || !fdcState->handle || !fdcState->ready) {
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_STATE;
    }

    bool postInitGateOk = outDiag->configReadbackOk &&
                          outDiag->postInitConverting &&
                          outDiag->postInitUnreadPresent &&
                          !outDiag->postInitStatusWatchdogFault &&
                          !outDiag->postInitStatusAmplitudeFault;
    printf("DBGS5D5_FDC,stage=config,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,converting=%u,unread=%u,watchdog=%u,"
           "amplitude=%u,configReadbackOk=%u,status=%s\n",
           fdcState->label ? fdcState->label : SENSORARRAY_NA,
           (int)fdcState->i2cCtx->Port,
           fdcState->i2cAddr,
           outDiag->postInitConverting ? 1u : 0u,
           outDiag->postInitUnreadPresent ? 1u : 0u,
           outDiag->postInitStatusWatchdogFault ? 1u : 0u,
           outDiag->postInitStatusAmplitudeFault ? 1u : 0u,
           outDiag->configReadbackOk ? 1u : 0u,
           postInitGateOk ? "ok" : "post_init_gate_failed");
    if (!postInitGateOk) {
        outDiag->status = "post_init_gate_failed";
        (void)Fdc2214CapDestroy(fdcState->handle);
        fdcState->handle = NULL;
        fdcState->ready = false;
        return ESP_ERR_INVALID_STATE;
    }

    Fdc2214CapDebugSnapshot_t snapshot = {0};
    err = Fdc2214CapReadDebugSnapshot(fdcState->handle, FDC2214_CH0, &snapshot);
    printf("DBGS5D5_FDC,stage=snapshot_before,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,status=0x%04X,statusConfig=0x%04X,"
           "config=0x%04X,muxConfig=0x%04X,driveCurrent=0x%04X,watchdog=%u,amplitude=%u,unread=%u,dataMsb=0x%04X,"
           "dataLsb=0x%04X,err=%ld,failedReg=0x%02X\n",
           fdcState->label ? fdcState->label : SENSORARRAY_NA,
           (int)fdcState->i2cCtx->Port,
           fdcState->i2cAddr,
           snapshot.Status,
           snapshot.StatusConfig,
           snapshot.Config,
           snapshot.MuxConfig,
           snapshot.DriveCurrentCh0,
           (snapshot.StatusErrWatchdog || snapshot.DataErrWatchdog) ? 1u : 0u,
           (snapshot.StatusErrAmplitudeHigh || snapshot.StatusErrAmplitudeLow || snapshot.DataErrAmplitude) ? 1u : 0u,
           snapshot.UnreadConversion[FDC2214_CH0] ? 1u : 0u,
           snapshot.DataMsb,
           snapshot.DataLsb,
           (long)err,
           snapshot.FailedReg);
    if (err != ESP_OK) {
        outDiag->status = "post_init_snapshot_failed";
        (void)Fdc2214CapDestroy(fdcState->handle);
        fdcState->handle = NULL;
        fdcState->ready = false;
        return err;
    }

    if (snapshot.Config != outDiag->configReg ||
        snapshot.MuxConfig != outDiag->muxConfigReg ||
        snapshot.StatusConfig != outDiag->statusConfigReg) {
        outDiag->status = "post_init_config_mismatch";
        (void)Fdc2214CapDestroy(fdcState->handle);
        fdcState->handle = NULL;
        fdcState->ready = false;
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}
