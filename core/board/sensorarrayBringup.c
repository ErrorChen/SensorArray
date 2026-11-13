#include "sensorarrayBringup.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "sensorarrayConfig.h"
#include "sensorarrayAcqEvent.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

#define printf sensorarrayAcqEventPrintf

#define SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK 0x8000u
#define SENSORARRAY_FDC_DRIVE_CURRENT_MASK 0xF800u

/* ADS PRESENCE IMPLEMENTATION BEGIN -- exercised verbatim by host regression.
 * This slot exists for cold ADS init failures which already destroyed the
 * ordinary handle mutex. Its DMA storage is permanent for this boot. */
typedef struct {
    bool busy;
    bool pending;
    bool quarantined;
    bool restoreClockPending;
    uint32_t restoreClockHz;
    uint32_t restoreActualHz;
    spi_device_handle_t device;
    spi_transaction_t transaction;
    uint8_t *tx;
    uint8_t *rx;
} sensorarrayAdsPresenceIo_t;
static sensorarrayAdsPresenceIo_t s_adsPresenceIo;

bool sensorarrayBringupAdsPresenceBusy(void)
{
    return __atomic_load_n(&s_adsPresenceIo.busy, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&s_adsPresenceIo.pending, __ATOMIC_ACQUIRE);
}

bool sensorarrayBringupAdsPresenceQuarantined(void)
{
    return __atomic_load_n(&s_adsPresenceIo.quarantined, __ATOMIC_ACQUIRE);
}

static TickType_t sensorarrayAdsPresenceTicks(uint64_t deadlineUs)
{
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (nowUs >= deadlineUs) { return 0u; }
    uint64_t ticks = ((deadlineUs - nowUs) * configTICK_RATE_HZ + 999999u) / 1000000u;
    return ticks >= portMAX_DELAY ? portMAX_DELAY - 1u : (TickType_t)ticks;
}

static esp_err_t sensorarrayAdsPresenceReadSlot(spi_device_handle_t device,
    uint64_t deadlineUs, uint32_t clockHz, uint8_t registers[3])
{
    sensorarrayAdsPresenceIo_t *io = &s_adsPresenceIo;
    if (__atomic_load_n(&io->pending, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&io->quarantined, __ATOMIC_ACQUIRE) ||
        (io->device && io->device != device)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!io->tx) { io->tx = heap_caps_malloc(8u, MALLOC_CAP_DMA); }
    if (!io->rx) { io->rx = heap_caps_malloc(8u, MALLOC_CAP_DMA); }
    if (!io->tx || !io->rx) { return ESP_ERR_NO_MEM; }
    if ((uint64_t)esp_timer_get_time() >= deadlineUs) { return ESP_ERR_TIMEOUT; }
    io->device = device;
    /* RREG ID..INTERFACE plus the legal MODE0 guard register. The guard's
     * trailing bit is not used as any requested register's evidence. */
    memset(io->tx, 0, 8u);
    memset(io->rx, 0, 8u);
    io->tx[0] = 0x20u;
    io->tx[1] = 3u;
    io->transaction = (spi_transaction_t){
        .length = 48u, .rxlength = 48u,
        .tx_buffer = io->tx, .rx_buffer = io->rx,
        .override_freq_hz = clockHz,
    };
    esp_err_t err = spi_device_queue_trans(device, &io->transaction,
        sensorarrayAdsPresenceTicks(deadlineUs));
    if (err != ESP_OK) { return err; }
    __atomic_store_n(&io->pending, true, __ATOMIC_RELEASE);
    spi_transaction_t *completed = NULL;
    err = spi_device_get_trans_result(device, &completed,
        sensorarrayAdsPresenceTicks(deadlineUs));
    if (err != ESP_OK || completed != &io->transaction) {
        __atomic_store_n(&io->quarantined, true, __ATOMIC_RELEASE);
        return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;
    }
    __atomic_store_n(&io->pending, false, __ATOMIC_RELEASE);
    memcpy(registers, io->rx + 2u, 3u);
    return ESP_OK;
}

esp_err_t sensorarrayBringupProbeAdsPresence(sensorarrayState_t *state,
    uint64_t absoluteDeadlineUs, sensorarrayAdsPresenceResult_t *outResult)
{
    if (!outResult) { return ESP_ERR_INVALID_ARG; }
    memset(outResult, 0, sizeof(*outResult));
    if (!state || !state->spiDevice) {
        outResult->error = ESP_ERR_INVALID_STATE;
        return outResult->error;
    }
    bool expectedIdle = false;
    if (!__atomic_compare_exchange_n(&s_adsPresenceIo.busy, &expectedIdle, true,
            false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
        outResult->busy = true;
        outResult->error = ESP_ERR_INVALID_STATE;
        return outResult->error;
    }
    ads126xAdcHandle_t *ads = &state->ads;
    uint8_t first[3] = {0}, second[3] = {0};
    bool ownsTransaction = false;
    bool useRuntimeHandle = ads->mutex != NULL;
    uint32_t savedRequestedHz = 0u, savedActualHz = 0u;
    bool clockChanged = false;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    uint64_t remainingUs = absoluteDeadlineUs > nowUs ? absoluteDeadlineUs - nowUs : 0u;
    uint64_t cleanupReserveUs = remainingUs > 10000u ? 2000u : remainingUs / 4u;
    uint64_t workDeadlineUs = absoluteDeadlineUs - cleanupReserveUs;
    esp_err_t err = ESP_OK;
    if (ads126xAdcIsQuarantined(ads) ||
        __atomic_load_n(&s_adsPresenceIo.pending, __ATOMIC_ACQUIRE) ||
        sensorarrayBringupAdsPresenceQuarantined()) {
        outResult->quarantined = true;
        err = ESP_ERR_INVALID_STATE;
    } else if (ads->transactionDeadlineUs != 0u) {
        outResult->busy = true;
        err = ESP_ERR_INVALID_STATE;
    } else if ((uint64_t)esp_timer_get_time() >= absoluteDeadlineUs) {
        err = ESP_ERR_TIMEOUT;
    } else if (useRuntimeHandle) {
        /* adsReady is a frontend policy flag and may already be false while
         * the original mutex/device remains alive. Never create a second
         * owner merely because that readiness flag was cleared. */
        if (ads->spiDevice != state->spiDevice) {
            err = ESP_ERR_INVALID_STATE;
        } else {
            err = ads126xAdcBeginTransactionUntil(ads, workDeadlineUs);
            ownsTransaction = err == ESP_OK;
            if (err == ESP_OK) {
                err = ads126xAdcGetSpiClockHz(ads, &savedRequestedHz, &savedActualHz);
            }
            if (err == ESP_OK && savedRequestedHz != 0u) {
                /* A previous restore may have been rejected by IDF. Its
                 * anomalous entry actual clock must not become a new target;
                 * the retained requested clock defines the expected divider. */
                err = ads126xAdcPredictDefaultSpiClockHz(savedRequestedHz, &savedActualHz);
            }
            if (err == ESP_OK) { err = ads126xAdcRequestSpiClockHz(ads, 125000u); }
            if (err == ESP_OK) { clockChanged = true; }
            if (err == ESP_OK) { err = ads126xAdcReadRegisters(ads, 0u, first, sizeof(first)); }
            if (err == ESP_OK) { err = ads126xAdcReadRegisters(ads, 0u, second, sizeof(second)); }
            if (err == ESP_OK) { err = ads126xAdcGetSpiClockHz(ads, NULL, &outResult->actualHz); }
        }
    } else {
        if (s_adsPresenceIo.restoreClockPending) {
            savedRequestedHz = s_adsPresenceIo.restoreClockHz;
            savedActualHz = s_adsPresenceIo.restoreActualHz;
        } else {
            int beforeKhz = 0;
            err = spi_device_get_actual_freq(state->spiDevice, &beforeKhz);
            if (err == ESP_OK && beforeKhz > 0) {
                savedActualHz = (uint32_t)beforeKhz * 1000u;
                s_adsPresenceIo.restoreClockHz = savedActualHz;
                s_adsPresenceIo.restoreActualHz = savedActualHz;
            } else if (err == ESP_OK) { err = ESP_ERR_INVALID_RESPONSE; }
        }
        if (err == ESP_OK) {
            s_adsPresenceIo.restoreClockPending = true;
            clockChanged = true;
            err = sensorarrayAdsPresenceReadSlot(state->spiDevice, workDeadlineUs, 125000u, first);
        }
        if (err == ESP_OK) {
            err = sensorarrayAdsPresenceReadSlot(state->spiDevice, workDeadlineUs, 125000u, second);
        }
        if (err == ESP_OK) {
            int actualKhz = 0;
            err = spi_device_get_actual_freq(state->spiDevice, &actualKhz);
            if (err == ESP_OK && actualKhz > 0) { outResult->actualHz = (uint32_t)actualKhz * 1000u; }
            else if (err == ESP_OK) { err = ESP_ERR_INVALID_RESPONSE; }
        }
    }
    if (ownsTransaction) { ads126xAdcEndTransaction(ads); }
    outResult->id = second[0];
    outResult->power = second[1] & ADS126X_POWER_READ_MASK;
    outResult->interface = second[2] & ADS126X_INTERFACE_CONFIG_MASK;
    if (err == ESP_OK && (outResult->actualHz != 125000u ||
            first[0] != second[0] ||
            ((first[1] ^ second[1]) & ADS126X_POWER_READ_MASK) != 0u ||
            ((first[2] ^ second[2]) & ADS126X_INTERFACE_CONFIG_MASK) != 0u)) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK) {
        outResult->known = true;
        /* A successful SPI transaction with a pulled-up/down MISO is not
         * presence. ADS1262 DEV_ID=0 / ADS1263 DEV_ID=1; revision bits remain
         * evidence. An earlier trusted nonzero ID must also still match. */
        bool idMatches = second[0] != 0u && second[0] != 0xFFu &&
            (second[0] >> 5u) <= 1u &&
            (ads->idRegRaw == 0u || ads->idRegRaw == 0xFFu || second[0] == ads->idRegRaw);
        outResult->present = idMatches && (outResult->interface & 0x03u) != 0x03u;
        if (!outResult->present) { err = ESP_ERR_NOT_FOUND; }
    }
    /* Presence polling may run while all frontends are healthy. Restore the
     * selected runtime rate even after an absent/invalid ADC response: clock
     * setup is local to the host and a completed RREG need not return valid
     * ADC data to prove which host divider is active. */
    esp_err_t cleanup = ESP_OK;
    if (clockChanged) {
        if (ads126xAdcIsQuarantined(ads) ||
            __atomic_load_n(&s_adsPresenceIo.pending, __ATOMIC_ACQUIRE) ||
            sensorarrayBringupAdsPresenceQuarantined()) {
            cleanup = ESP_ERR_INVALID_STATE;
        } else if ((uint64_t)esp_timer_get_time() >= absoluteDeadlineUs) {
            cleanup = ESP_ERR_TIMEOUT;
        } else {
            uint8_t discard[3] = {0};
            uint32_t restoreHz = savedRequestedHz ? savedRequestedHz : savedActualHz;
            uint32_t restoredActualHz = 0u;
            if (useRuntimeHandle) {
                cleanup = ads126xAdcBeginTransactionUntil(ads, absoluteDeadlineUs);
                if (cleanup == ESP_OK) {
                    cleanup = ads126xAdcRequestSpiClockHz(ads, restoreHz);
                    if (cleanup == ESP_OK) { cleanup = ads126xAdcReadRegisters(ads, 0u, discard, sizeof(discard)); }
                    if (cleanup == ESP_OK) { cleanup = ads126xAdcGetSpiClockHz(ads, NULL, &restoredActualHz); }
                    ads126xAdcEndTransaction(ads);
                }
            } else {
                cleanup = sensorarrayAdsPresenceReadSlot(state->spiDevice, absoluteDeadlineUs,
                    restoreHz, discard);
                if (cleanup == ESP_OK) {
                    int restoredKhz = 0;
                    cleanup = spi_device_get_actual_freq(state->spiDevice, &restoredKhz);
                    if (cleanup == ESP_OK && restoredKhz > 0) { restoredActualHz = (uint32_t)restoredKhz * 1000u; }
                }
            }
            if (cleanup == ESP_OK && restoredActualHz != savedActualHz) { cleanup = ESP_ERR_INVALID_RESPONSE; }
            if (cleanup == ESP_OK) { outResult->restoredHz = restoredActualHz; }
            if (cleanup == ESP_OK && !useRuntimeHandle) { s_adsPresenceIo.restoreClockPending = false; }
        }
    }
    if (cleanup != ESP_OK) { outResult->known = false; outResult->present = false; }
    outResult->quarantined = ads126xAdcIsQuarantined(ads) ||
        __atomic_load_n(&s_adsPresenceIo.pending, __ATOMIC_ACQUIRE) ||
        sensorarrayBringupAdsPresenceQuarantined();
    outResult->error = err;
    outResult->cleanupError = cleanup;
    __atomic_store_n(&s_adsPresenceIo.busy, false, __ATOMIC_RELEASE);
    return err != ESP_OK ? err : cleanup;
}
/* ADS PRESENCE IMPLEMENTATION END */

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
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

    /* Bootstrap is a communication admission speed, not a qualified result.
     * The boot probe selects and verifies the runtime clock without removing
     * this device or recycling any outstanding DMA descriptor. */
#if CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
    const uint32_t initialClockHz = CONFIG_ADS126X_SPI_CLOCK_HZ < 125000u ?
        CONFIG_ADS126X_SPI_CLOCK_HZ : 125000u;
#else
    const uint32_t initialClockHz = CONFIG_ADS126X_SPI_CLOCK_HZ;
#endif
    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = (int)initialClockHz,
        .mode = 1,
        .duty_cycle_pos = ads126xAdcSpiDutyCycleForClock(initialClockHz),
        .spics_io_num = CONFIG_BOARD_ADS126X_CS_GPIO,
        .queue_size = 1,
    };

    err = spi_bus_add_device(CONFIG_BOARD_SPI_HOST, &devCfg, outDevice);
    printf("ADS_SPI_TIMING,path=board,requestedClockHz=%d,mode=%u,dutyCyclePos=%u,cs=%d,candidate=%u,err=0x%x\n",
        devCfg.clock_speed_hz, devCfg.mode, devCfg.duty_cycle_pos, devCfg.spics_io_num,
        devCfg.duty_cycle_pos == 64u ? 1u : 0u, err);
    return err;
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

static Fdc2214CapChannelConfig_t sensorarrayBringupFdcMatrixChannelProfile(uint8_t channelIndex)
{
    (void)channelIndex;

    /*
     * Explicit channel profile shared by both FDC2214 devices:
     * - CLOCK_DIVIDERS explicitly sets both CHx_FIN_SEL and CHx_FREF_DIVIDER.
     * - Values follow the stable bring-up profile and avoid implicit defaults.
     */
    return (Fdc2214CapChannelConfig_t){
        .Rcount = SENSORARRAY_FDC_RCOUNT,
        .SettleCount = SENSORARRAY_FDC_SETTLECOUNT,
        .Offset = SENSORARRAY_FDC_OFFSET,
        .ClockDividers = SENSORARRAY_FDC_CLOCK_DIVIDERS,
        .DriveCurrent = SENSORARRAY_FDC_DRIVE_CURRENT,
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
        .IntbDisabled = !SENSORARRAY_FDC_INTB_OUTPUT_ENABLE,
        .HighCurrentDrive = false,
    };
    return Fdc2214CapBuildConfig(&config);
}

static esp_err_t sensorarrayBringupDumpFdcInitRegisters(const BoardSupportI2cCtx_t *i2cCtx,
                                                        uint8_t i2cAddr,
                                                        const char *fdcLabel,
                                                        const char *stage,
                                                        Fdc2214CapDevice_t *dev,
                                                        uint16_t manufacturerId,
                                                        uint16_t deviceId)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapCoreRegs_t coreRegs = {0};
    uint16_t clockDiv0 = 0u;

    esp_err_t err = Fdc2214CapReadCoreRegs(dev, &coreRegs);
    if (err == ESP_OK) {
        err = Fdc2214CapReadClockDividers(dev, FDC2214_CH0, &clockDiv0);
    }

    printf("FDCINIT,stage=%s,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,status=0x%04X,"
           "statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,clockDiv0=0x%04X,"
           "refClock=%s,refClockHz=%lu,err=%ld,result=%s\n",
           stage ? stage : SENSORARRAY_NA,
           fdcLabel ? fdcLabel : SENSORARRAY_NA,
           i2cCtx ? (int)i2cCtx->Port : -1,
           i2cAddr,
           manufacturerId,
           deviceId,
           coreRegs.Status,
           coreRegs.StatusConfig,
           coreRegs.Config,
           coreRegs.MuxConfig,
           clockDiv0,
           sensorarrayBringupFdcRefClockName(sensorarrayBringupFdcRefClockSource()),
           (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");

    return err;
}

static void sensorarrayBringupDumpFdcClock(Fdc2214CapDevice_t *dev,
                                           const char *fdcLabel,
                                           const char *stage,
                                           Fdc2214CapChannel_t channel)
{
    uint16_t clockDividers = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfig = 0u;
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *clockStatus = "unknown";
    esp_err_t clockErr = ESP_ERR_INVALID_ARG;

    if (dev) {
        Fdc2214CapCoreRegs_t coreRegs = {0};
        esp_err_t coreErr = Fdc2214CapReadCoreRegs(dev, &coreRegs);
        if (coreErr == ESP_OK) {
            configReg = coreRegs.Config;
            muxConfig = coreRegs.MuxConfig;
        }
        clockErr = Fdc2214CapReadClockDividers(dev, channel, &clockDividers);
    }

    bool decodeOk = false;
    if (clockErr == ESP_OK) {
        decodeOk = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                            &finSelCode,
                                                            &finFactor,
                                                            &frefDivider,
                                                            &clockStatus);
    } else {
        clockStatus = "clock_read_error";
    }

    uint32_t effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz();
    double effectiveFrefHz = (decodeOk && frefDivider > 0u) ? ((double)effectiveFclkHz / (double)frefDivider) : 0.0;

    printf("FDC_CLOCK,stage=%s,device=%s,channel=%u,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,"
           "frefDivider=%u,refClockSource=%s,effectiveFclkHz=%lu,effectiveFrefHz=%.3f,configReg=0x%04X,"
           "muxConfig=0x%04X,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           fdcLabel ? fdcLabel : SENSORARRAY_NA,
           (unsigned)channel,
           clockDividers,
           (unsigned)finSelCode,
           (unsigned)finFactor,
           (unsigned)frefDivider,
           sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
           (unsigned long)effectiveFclkHz,
           effectiveFrefHz,
           configReg,
           muxConfig,
           clockStatus ? clockStatus : SENSORARRAY_NA);
}

static esp_err_t sensorarrayBringupVerifyFdcActiveState(Fdc2214CapDevice_t *dev,
                                                        uint8_t channels,
                                                        uint16_t expectedStatusConfig,
                                                        uint16_t expectedConfig,
                                                        uint16_t expectedMuxConfig)
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
    err = Fdc2214CapReadStatus(dev, &status);
    if (err != ESP_OK) {
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

    printf("FDCINIT_VERIFY,sleep=%u,autoscan=%u,activeChannel=%u,converting=%u,unreadPresent=%u,"
           "status=0x%04X,statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,expectedStatusConfig=0x%04X,"
           "expectedConfig=0x%04X,expectedMuxConfig=0x%04X,refClockSource=%s,refClockHz=%lu,result=%s\n",
           sleepEnabled ? 1u : 0u,
           autoScanEnabled ? 1u : 0u,
           (unsigned)activeChannel,
           converting ? 1u : 0u,
           unreadPresent ? 1u : 0u,
           coreRegs.Status,
           coreRegs.StatusConfig,
           coreRegs.Config,
           coreRegs.MuxConfig,
           expectedStatusConfig,
           expectedConfig,
           expectedMuxConfig,
           sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
           (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
           (!sleepEnabled &&
            converting &&
            coreRegs.StatusConfig == expectedStatusConfig &&
            coreRegs.Config == expectedConfig &&
            coreRegs.MuxConfig == expectedMuxConfig)
               ? "ok"
               : "mismatch");

    if (sleepEnabled || !converting) {
        return ESP_ERR_INVALID_STATE;
    }
    if (coreRegs.StatusConfig != expectedStatusConfig ||
        coreRegs.Config != expectedConfig ||
        coreRegs.MuxConfig != expectedMuxConfig) {
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
    fdcState->refClockHz = 0u;
    fdcState->statusConfigReg = 0u;
    fdcState->configReg = 0u;
    fdcState->muxConfigReg = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        fdcState->sweepProfile[ch] = (sensorarrayFdcSweepProfile_t){0};
    }
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
    cfg.crcMode = ADS126X_CRC_CRC8;
    cfg.enableStatusByte = false;
    cfg.enableInternalRef = false;
    cfg.vrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV;
    cfg.pgaGain = 1;
    cfg.dataRateDr = (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE;

    err = ads126xAdcInit(&state->ads, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    /* POWER.RESET powers up asserted and is mirrored in every ADC status
     * byte. Acknowledge only this deliberate bring-up reset so the matrix
     * engine can detect any reset that happens later. */
    err = ads126xAdcClearResetFlag(&state->ads);
    if (err != ESP_OK) {
        (void)ads126xAdcDeinit(&state->ads);
        return err;
    }

    state->adsAdc1Running = false;
    state->adsRefReady = false;
    state->adsRefMuxValid = false;
    return ESP_OK;
}

esp_err_t sensorarrayBringupAttachAdsNoReset(sensorarrayState_t *state)
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
    cfg.crcMode = ADS126X_CRC_CRC8;
    cfg.enableStatusByte = false;
    cfg.enableInternalRef = false;
    cfg.skipResetOnInit = true;
    cfg.skipConfigureOnInit = true;
    cfg.vrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV;
    cfg.pgaGain = 1;
    cfg.dataRateDr = 0;

    err = ads126xAdcInit(&state->ads, &cfg);
    if (err == ESP_OK) {
        state->adsAdc1Running = false;
        state->adsRefReady = false;
        state->adsRefMuxValid = false;
    }
    return err;
}

esp_err_t sensorarrayBringupPrepareAdsRefPath(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    tmuxSwitchControlState_t control = {0};
    esp_err_t controlErr = tmuxSwitchGetControlState(&control);
    if (controlErr != ESP_OK) {
        return controlErr;
    }
    if (control.cmdSource != TMUX1108_SOURCE_REF ||
        control.cmdSwLevel != 0 || control.obsSwLevel != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ads126xAdcConfigure(&state->ads, true, false, ADS126X_CRC_CRC8, 1, 0);
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
#if CONFIG_SENSORARRAY_FDC_STARTUP_PROBE
    if (!fdcState || !fdcState->i2cCtx) {
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
        esp_err_t err = sensorarrayBringupProbeFdcCandidate(fdcState->i2cCtx, probeState.i2cAddr, &probeDiag);
        if (probeDiag.status == SENSORARRAY_FDC_DISCOVERY_UNEXPECTED_ID) {
            err = ESP_OK;
        } else if (probeDiag.status == SENSORARRAY_FDC_DISCOVERY_ACK_BUT_READ_FAILED) {
            err = probeDiag.idErr;
        } else if (probeDiag.status == SENSORARRAY_FDC_DISCOVERY_NO_ACK) {
            err = probeDiag.ackErr;
        }

        sensorarrayLogStartupFdc("fdc_probe",
                                 &probeState,
                                 err,
                                 sensorarrayBringupFdcDiscoveryStatusName(probeDiag.status),
                                 (int32_t)probeState.i2cAddr,
                                 probeDiag.haveManufacturerId && probeDiag.haveDeviceId,
                                 probeDiag.manufacturerId,
                                 probeDiag.deviceId,
                                 "probe_regs_0x7E_0x7F");
    }
#else
    (void)fdcState;
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
    diag->configVerified = false;
    diag->refClockKnown = false;
    diag->refClockSource = FDC2214_REF_CLOCK_INTERNAL;
    diag->refClockHz = 0u;
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
    fdcState->refClockHz = diag->refClockHz;
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
        .readReg16Sequence = boardSupportI2cReadReg16Sequence,
        .maxSequenceRegs = 8u,
        .IntGpio = (CONFIG_SENSORARRAY_FDC_INTB_ENABLE != 0) ?
            ((i2cAddr == CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR) ?
                CONFIG_SENSORARRAY_FDC_INTB2_GPIO :
                CONFIG_SENSORARRAY_FDC_INTB1_GPIO) :
            -1,
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
    }

    if (manufacturer != SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID ||
        deviceId != SENSORARRAY_FDC_EXPECTED_DEVICE_ID) {
        if (outDiag) {
            outDiag->status = "id_mismatch";
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
        Fdc2214CapChannelConfig_t chCfg = sensorarrayBringupFdcMatrixChannelProfile(ch);
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
            printf("FDCINIT_WARN,stage=channel_config,channel=%u,driveReq=0x%04X,driveNorm=0x%04X,driveReadback=0x%04X,"
                   "status=drive_current_effective_mismatch_continue\n",
                   (unsigned)ch,
                   chCfg.DriveCurrent,
                   (uint16_t)(chCfg.DriveCurrent & SENSORARRAY_FDC_DRIVE_CURRENT_MASK),
                   cfgDriveReadback);
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
            printf("FDCINIT_WARN,stage=channel_readback,channel=%u,driveReq=0x%04X,driveNorm=0x%04X,"
                   "driveReadback=0x%04X,status=drive_current_effective_mismatch_continue\n",
                   (unsigned)ch,
                   chCfg.DriveCurrent,
                   (uint16_t)(chCfg.DriveCurrent & SENSORARRAY_FDC_DRIVE_CURRENT_MASK),
                   verifyDriveReadback);
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

    err = sensorarrayBringupVerifyFdcActiveState(dev,
                                                 channels,
                                                 expectedStatusConfig,
                                                 finalConfig,
                                                 expectedMuxConfig);
    if (err != ESP_OK) {
        (void)sensorarrayBringupDumpFdcInitRegisters(i2cCtx,
                                                     i2cAddr,
                                                     fdcLabel,
                                                     "verify_failed",
                                                     dev,
                                                     manufacturer,
                                                     deviceId);
        if (outDiag) {
            outDiag->status = "active_verify_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    err = sensorarrayBringupDumpFdcInitRegisters(i2cCtx, i2cAddr, fdcLabel, "post_init", dev, manufacturer, deviceId);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "diagnostic_dump_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }
    sensorarrayBringupDumpFdcClock(dev, fdcLabel, "init", FDC2214_CH0);

    *outDev = dev;
    if (outDiag) {
        outDiag->status = driveCurrentWarning ? "ok_with_drive_current_warning" : "ok";
        outDiag->configVerified = true;
        outDiag->refClockKnown = true;
        outDiag->refClockSource = sensorarrayBringupFdcRefClockSource();
        outDiag->refClockHz = sensorarrayMeasureFdcEffectiveFclkHz();
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

    uint16_t expectedStatusConfig = sensorarrayBringupFdcExpectedStatusConfig();
    uint16_t expectedMuxConfig = sensorarrayBringupFdcExpectedMuxConfig(1u);
    uint16_t expectedConfig = sensorarrayBringupFdcBuildFinalConfig(channel);
    err = sensorarrayBringupVerifyFdcActiveState(*outDev,
                                                 1u,
                                                 expectedStatusConfig,
                                                 expectedConfig,
                                                 expectedMuxConfig);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "single_channel_verify_failure";
            outDiag->detail = (int32_t)channel;
        }
        Fdc2214CapDestroy(*outDev);
        *outDev = NULL;
        return err;
    }

    const char *fdcLabel = (i2cAddr == SENSORARRAY_FDC_I2C_ADDR_LOW) ? "secondary_selb_side" : "primary_sela_side";
    sensorarrayBringupDumpFdcClock(*outDev, fdcLabel, "init", channel);

    if (outDiag) {
        outDiag->status = "ok_single_channel";
        outDiag->configVerified = true;
        outDiag->refClockKnown = true;
        outDiag->refClockSource = sensorarrayBringupFdcRefClockSource();
        outDiag->refClockHz = sensorarrayMeasureFdcEffectiveFclkHz();
        outDiag->statusConfigReg = expectedStatusConfig;
        outDiag->configReg = expectedConfig;
        outDiag->muxConfigReg = expectedMuxConfig;
        outDiag->detail = (int32_t)channel;
    }
    return ESP_OK;
#endif
}
