#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADS126X_ADC_DEFAULT_VREF_UV 2500000u
#define ADS126X_ADC_DEFAULT_DRDY_TIMEOUT_MS 1000u

/* POWER register bits for ads126xAdcReadCoreRegisters()/ads126xAdcSetVbiasEnabled(). */
#define ADS126X_POWER_INTREF (1u << 0)
#define ADS126X_POWER_VBIAS (1u << 1)

/* REFMUX value for the ADS126x internal 2.5 V reference. */
#define ADS126X_REFMUX_INTERNAL 0x00u

typedef enum {
    ADS126X_DEVICE_AUTO = 0,
    ADS126X_DEVICE_ADS1262,
    ADS126X_DEVICE_ADS1263,
} ads126xDeviceType_t;

typedef enum {
    ADS126X_CRC_OFF = 0,
    ADS126X_CRC_CHECKSUM = 1,
    ADS126X_CRC_CRC8 = 2,
} ads126xCrcMode_t;

typedef enum {
    ADS126X_ADC1_DR_2P5_SPS = 0x0,
    ADS126X_ADC1_DR_5_SPS = 0x1,
    ADS126X_ADC1_DR_10_SPS = 0x2,
    ADS126X_ADC1_DR_16P6_SPS = 0x3,
    ADS126X_ADC1_DR_20_SPS = 0x4,
    ADS126X_ADC1_DR_50_SPS = 0x5,
    ADS126X_ADC1_DR_60_SPS = 0x6,
    ADS126X_ADC1_DR_100_SPS = 0x7,
    ADS126X_ADC1_DR_400_SPS = 0x8,
    ADS126X_ADC1_DR_1200_SPS = 0x9,
    ADS126X_ADC1_DR_2400_SPS = 0xA,
    ADS126X_ADC1_DR_4800_SPS = 0xB,
    ADS126X_ADC1_DR_7200_SPS = 0xC,
    ADS126X_ADC1_DR_14400_SPS = 0xD,
    ADS126X_ADC1_DR_19200_SPS = 0xE,
    ADS126X_ADC1_DR_38400_SPS = 0xF,
} ads126xAdc1DataRate_t;

typedef enum {
    ADS126X_GAIN_1 = 1,
    ADS126X_GAIN_2 = 2,
    ADS126X_GAIN_4 = 4,
    ADS126X_GAIN_8 = 8,
    ADS126X_GAIN_16 = 16,
    ADS126X_GAIN_32 = 32,
} ads126xGain_t;

typedef struct {
    spi_device_handle_t spiDevice;
    gpio_num_t drdyGpio;
    gpio_num_t resetGpio;
    ads126xDeviceType_t forcedType;
    ads126xCrcMode_t crcMode;
    bool enableStatusByte;
    bool enableInternalRef;
    uint32_t vrefMicrovolts;
    uint8_t pgaGain;
    uint8_t dataRateDr;
} ads126xAdcConfig_t;

/*
 * Handle is semi-opaque. Do not modify fields after init unless noted.
 * drdyTimeoutMs can be adjusted after init if a different default is needed.
 */
typedef struct {
    spi_device_handle_t spiDevice;
    gpio_num_t drdyGpio;
    gpio_num_t resetGpio;
    ads126xDeviceType_t deviceType;
    ads126xDeviceType_t forcedType;
    ads126xCrcMode_t crcMode;
    bool enableStatusByte;
    bool enableInternalRef;
    uint32_t vrefMicrovolts;
    uint8_t pgaGain;
    uint8_t dataRateDr;
    uint8_t idRegRaw;
    uint32_t drdyTimeoutMs;
    SemaphoreHandle_t mutex;
    uint8_t *spiTxBuf;
    uint8_t *spiRxBuf;
    size_t spiBufSize;
} ads126xAdcHandle_t;

typedef struct {
    uint8_t muxp;
    uint8_t muxn;
    bool stopBeforeMuxChange;
    bool startAfterMuxChange;
    uint32_t settleAfterMuxUs;
    bool discardFirst;
    uint8_t oversampleCount;
    uint32_t drdyTimeoutUs;
} ads126xAdcVoltageReadConfig_t;

typedef struct {
    int32_t rawCode;
    int32_t microvolts;
    uint8_t gain;
    uint8_t samplesUsed;
    bool clippedOrNearFullScale;
    esp_err_t err;
} ads126xAdcVoltageSample_t;

typedef struct {
    uint8_t minGain;
    uint8_t maxGain;
    uint8_t initialGain;
    uint8_t headroomPercent;
    uint8_t maxIterations;
    uint32_t settleAfterGainChangeUs;
    bool keepConfiguredOnSuccess;
} ads126xAdcAutoGainConfig_t;

typedef struct {
    uint8_t selectedGain;
    int32_t sampleRaw;
    int32_t sampleMicrovolts;
    bool clippedOrNearFullScale;
    uint8_t iterations;
    esp_err_t err;
} ads126xAdcAutoGainResult_t;

esp_err_t ads126xAdcInit(ads126xAdcHandle_t *handle, const ads126xAdcConfig_t *cfg);
esp_err_t ads126xAdcDeinit(ads126xAdcHandle_t *handle);

esp_err_t ads126xAdcHardwareReset(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSendCommand(ads126xAdcHandle_t *handle, uint8_t cmd);

esp_err_t ads126xAdcReadRegisters(ads126xAdcHandle_t *handle, uint8_t startAddr, uint8_t *data, size_t len);
esp_err_t ads126xAdcWriteRegisters(ads126xAdcHandle_t *handle, uint8_t startAddr, const uint8_t *data, size_t len);

esp_err_t ads126xAdcGetIdRaw(ads126xAdcHandle_t *handle, uint8_t *idReg);

esp_err_t ads126xAdcConfigure(ads126xAdcHandle_t *handle,
                              bool enableInternalRef,
                              bool enableStatusByte,
                              ads126xCrcMode_t crcMode,
                              uint8_t pgaGain,
                              uint8_t dataRateDr);

esp_err_t ads126xAdcConfigureVoltageMode(ads126xAdcHandle_t *handle,
                                         uint8_t gain,
                                         uint8_t dataRateDr,
                                         bool enableStatusByte,
                                         bool enableCrc);

esp_err_t ads126xAdcSetRefMux(ads126xAdcHandle_t *handle, uint8_t refmuxValue);
esp_err_t ads126xAdcSetInputMux(ads126xAdcHandle_t *handle, uint8_t muxp, uint8_t muxn);

/*
 * Enable/disable internal AINCOM level shift (VBIAS) via POWER register bit1.
 * This performs a read-modify-write and preserves other POWER bits.
 */
esp_err_t ads126xAdcSetVbiasEnabled(ads126xAdcHandle_t *handle, bool enableVbias);

/*
 * Read core register snapshot used by debug and bringup diagnostics.
 * Any output pointer may be NULL when that field is not required.
 */
esp_err_t ads126xAdcReadCoreRegisters(ads126xAdcHandle_t *handle,
                                      uint8_t *outPower,
                                      uint8_t *outInterface,
                                      uint8_t *outMode2,
                                      uint8_t *outInpmux,
                                      uint8_t *outRefmux);

/*
 * Read a single differential ADC1 sample in microvolts:
 * 1) set INPMUX (muxp/muxn), 2) optional settle delay, 3) optional START1,
 * 4) optional discard conversions, 5) final conversion read and raw->uV conversion.
 */
esp_err_t ads126xAdcReadSingleDiffUv(ads126xAdcHandle_t *handle,
                                     uint8_t muxp,
                                     uint8_t muxn,
                                     bool start1EveryRead,
                                     uint32_t settleMs,
                                     uint8_t discardCount,
                                     int32_t *outRaw,
                                     int32_t *outUv,
                                     uint8_t *outStatus);

esp_err_t ads126xAdcStartAdc1(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcStopAdc1(ads126xAdcHandle_t *handle);

esp_err_t ads126xAdcWaitDrdy(ads126xAdcHandle_t *handle, uint32_t timeoutMs);

esp_err_t ads126xAdcWaitDrdyFastUs(ads126xAdcHandle_t *handle, uint32_t timeoutUs);

esp_err_t ads126xAdcReadAdc1Raw(ads126xAdcHandle_t *handle,
                                int32_t *rawCode,
                                uint8_t *statusByteOptional);

int32_t ads126xAdcRawToMicrovolts(const ads126xAdcHandle_t *handle, int32_t rawCode);

esp_err_t ads126xAdcReadVoltageMicrovoltsFast(ads126xAdcHandle_t *handle,
                                              const ads126xAdcVoltageReadConfig_t *cfg,
                                              ads126xAdcVoltageSample_t *outSample);

esp_err_t ads126xAdcSelectAutoGainForVoltage(ads126xAdcHandle_t *handle,
                                             const ads126xAdcVoltageReadConfig_t *readCfg,
                                             const ads126xAdcAutoGainConfig_t *gainCfg,
                                             ads126xAdcAutoGainResult_t *outResult);

esp_err_t ads126xAdcSelfOffsetCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSelfGainCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSystemOffsetCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSystemGainCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSelfCal(ads126xAdcHandle_t *handle);

esp_err_t ads126xAdcStartAdc2(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcStopAdc2(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcReadAdc2Raw(ads126xAdcHandle_t *handle,
                                int32_t *raw24,
                                uint8_t *statusOptional);

#if CONFIG_ADS126X_HELPER_CREATE_SPI
esp_err_t ads126xAdcHelperCreateSpiDevice(spi_device_handle_t *outDevice);
esp_err_t ads126xAdcHelperDestroySpiDevice(spi_device_handle_t device);
#endif

#ifdef __cplusplus
}
#endif
