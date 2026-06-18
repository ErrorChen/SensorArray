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
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADS126X_ADC_DEFAULT_VREF_UV 2500000u
#define ADS126X_ADC_DEFAULT_DRDY_TIMEOUT_MS 1000u

/* POWER register bits for ads126xAdcReadCoreRegisters()/ads126xAdcSetVbiasEnabled(). */
#define ADS126X_POWER_INTREF (1u << 0)
#define ADS126X_POWER_VBIAS (1u << 1)
#define ADS126X_REFMUX_INTERNAL 0x00u
#define ADS126X_REFMUX_AVDD_AVSS 0x24u
#define ADS126X_ADC2_REF_INTERNAL 0x00u
#define ADS126X_ADC2_REF_AVDD_AVSS 0x04u

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
    ADS126X_DEVICE_AUTO = 0,
    ADS126X_DEVICE_ADS1262,
    ADS126X_DEVICE_ADS1263,
} ads126xDeviceType_t;

typedef enum {
    ADS126X_CRC_OFF = 0,
    ADS126X_CRC_CHECKSUM = 1,
    ADS126X_CRC_CRC8 = 2,
} ads126xCrcMode_t;

typedef struct {
    spi_device_handle_t spiDevice;
    gpio_num_t drdyGpio;
    gpio_num_t resetGpio;
    ads126xDeviceType_t forcedType;
    ads126xCrcMode_t crcMode;
    bool enableStatusByte;
    bool enableInternalRef;
    bool skipResetOnInit;
    bool skipConfigureOnInit;
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
    uint8_t adc2Gain;
    uint8_t adc2DataRate;
    uint8_t adc2Reference;
    uint32_t adc2VrefMicrovolts;
    uint8_t idRegRaw;
    uint32_t drdyTimeoutMs;
    SemaphoreHandle_t mutex;
    uint8_t *spiTxBuf;
    uint8_t *spiRxBuf;
    size_t spiBufSize;
    bool spiDmaCapable;
    bool drdyNotificationReady;
    volatile TaskHandle_t drdyWaitTask;
    volatile uint32_t drdyGeneration;
    spi_transaction_t dmaTransaction;
} ads126xAdcHandle_t;

esp_err_t ads126xAdcInit(ads126xAdcHandle_t *handle, const ads126xAdcConfig_t *cfg);
esp_err_t ads126xAdcDeinit(ads126xAdcHandle_t *handle);

esp_err_t ads126xAdcHardwareReset(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSendCommand(ads126xAdcHandle_t *handle, uint8_t cmd);

esp_err_t ads126xAdcReadRegisters(ads126xAdcHandle_t *handle, uint8_t startAddr, uint8_t *data, size_t len);
esp_err_t ads126xAdcWriteRegisters(ads126xAdcHandle_t *handle, uint8_t startAddr, const uint8_t *data, size_t len);

esp_err_t ads126xAdcGetIdRaw(ads126xAdcHandle_t *handle, uint8_t *idReg);
uint8_t ads126xAdcGetDevId(const ads126xAdcHandle_t *handle);
uint8_t ads126xAdcGetRevId(const ads126xAdcHandle_t *handle);
bool ads126xAdcHasAdc2(const ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcProbeAdc2(ads126xAdcHandle_t *handle, bool *outHasAdc2);
esp_err_t ads126xAdcReadPowerRegister(ads126xAdcHandle_t *handle, uint8_t *outPower);
esp_err_t ads126xAdcWritePowerRegister(ads126xAdcHandle_t *handle, uint8_t power);
esp_err_t ads126xAdcApplyPowerPolicy(ads126xAdcHandle_t *handle,
                                     bool updateInternalRef,
                                     bool enableInternalRef,
                                     bool updateVbias,
                                     bool enableVbias,
                                     uint8_t *outPowerBefore,
                                     uint8_t *outPowerAfter);
esp_err_t ads126xAdcSetInternalReference(ads126xAdcHandle_t *handle, bool enableInternalRef);

esp_err_t ads126xAdcConfigure(ads126xAdcHandle_t *handle,
                              bool enableInternalRef,
                              bool enableStatusByte,
                              ads126xCrcMode_t crcMode,
                              uint8_t pgaGain,
                              uint8_t dataRateDr);

esp_err_t ads126xAdcSetRefMux(ads126xAdcHandle_t *handle, uint8_t refmuxValue);
esp_err_t ads126xAdcSetRefMuxWithVref(ads126xAdcHandle_t *handle,
                                      uint8_t refmuxValue,
                                      uint32_t vrefMicrovolts);
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
esp_err_t ads126xAdcEnableDrdyNotification(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcWaitDrdyNotificationUs(ads126xAdcHandle_t *handle, uint32_t timeoutUs);
void ads126xAdcClearDrdyNotifications(ads126xAdcHandle_t *handle);
uint32_t ads126xAdcGetDrdyGeneration(const ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcWaitDrdyGenerationUs(ads126xAdcHandle_t *handle,
                                         uint32_t startGeneration,
                                         uint32_t timeoutUs,
                                         uint32_t *outGeneration);
bool ads126xAdcStatusByteHasAdc1NewData(const ads126xAdcHandle_t *handle, uint8_t statusByte);
esp_err_t ads126xAdcSetInputMuxFast(ads126xAdcHandle_t *handle, uint8_t muxp, uint8_t muxn);
esp_err_t ads126xAdcReadAdc1RawDma(ads126xAdcHandle_t *handle,
                                   uint32_t drdyTimeoutUs,
                                   int32_t *rawCode,
                                   uint8_t *statusByteOptional,
                                   uint32_t *outReadUs);
esp_err_t ads126xAdcReadAdc1RawDmaReady(ads126xAdcHandle_t *handle,
                                        int32_t *rawCode,
                                        uint8_t *statusByteOptional,
                                        uint32_t *outReadUs);
uint32_t ads126xAdcDataRateCodeToSps(uint8_t drCode);
uint32_t ads126xAdcExpectedConversionPeriodUs(uint8_t drCode);

esp_err_t ads126xAdcReadAdc1Raw(ads126xAdcHandle_t *handle,
                                int32_t *rawCode,
                                uint8_t *statusByteOptional);

int32_t ads126xAdcRawToMicrovolts(const ads126xAdcHandle_t *handle, int32_t rawCode);

esp_err_t ads126xAdcSelfOffsetCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSelfGainCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSystemOffsetCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSystemGainCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSelfCal(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcReadCalibrationRegisters(ads126xAdcHandle_t *handle,
                                             uint8_t offsetCal[3],
                                             uint8_t fullScaleCal[3]);

esp_err_t ads126xAdcStartAdc2(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcStopAdc2(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcSetAdc2Config(ads126xAdcHandle_t *handle,
                                  uint8_t dataRate,
                                  uint8_t reference,
                                  uint8_t gain,
                                  uint32_t vrefMicrovolts);
esp_err_t ads126xAdcSetAdc2InputMux(ads126xAdcHandle_t *handle,
                                    uint8_t muxp,
                                    uint8_t muxn);
esp_err_t ads126xAdcReadAdc2Raw(ads126xAdcHandle_t *handle,
                                int32_t *raw24,
                                uint8_t *statusOptional);
esp_err_t ads126xAdcReadAdc2RawDma(ads126xAdcHandle_t *handle,
                                   uint32_t drdyTimeoutUs,
                                   int32_t *raw24,
                                   uint8_t *statusOptional,
                                   uint32_t *outReadUs);
esp_err_t ads126xAdcReadAdc2RawDmaReady(ads126xAdcHandle_t *handle,
                                        int32_t *raw24,
                                        uint8_t *statusOptional,
                                        uint32_t *outReadUs);
uint32_t ads126xAdcAdc2ExpectedConversionPeriodUs(uint8_t dataRate);
int32_t ads126xAdcAdc2RawToMicrovolts(const ads126xAdcHandle_t *handle,
                                      int32_t rawCode);
esp_err_t ads126xAdcSystemOffsetCalAdc2(ads126xAdcHandle_t *handle);
esp_err_t ads126xAdcReadAdc2CalibrationRegisters(ads126xAdcHandle_t *handle,
                                                 uint8_t offsetCal[2],
                                                 uint8_t fullScaleCal[2]);

#if CONFIG_ADS126X_HELPER_CREATE_SPI
esp_err_t ads126xAdcHelperCreateSpiDevice(spi_device_handle_t *outDevice);
esp_err_t ads126xAdcHelperDestroySpiDevice(spi_device_handle_t device);
#endif

#ifdef __cplusplus
}
#endif
