#ifndef FDC2214CAP_H
#define FDC2214CAP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Fdc2214CapDevice Fdc2214CapDevice_t;

typedef enum {
    FDC2214_CH0 = 0,
    FDC2214_CH1 = 1,
    FDC2214_CH2 = 2,
    FDC2214_CH3 = 3,
} Fdc2214CapChannel_t;

typedef enum {
    FDC2214_REF_CLOCK_INTERNAL = 0,
    FDC2214_REF_CLOCK_EXTERNAL = 1,
} Fdc2214CapRefClockSource_t;

typedef esp_err_t (*Fdc2214I2cWriteReadFn)(void* userCtx,
                                          uint8_t addr7,
                                          const uint8_t* tx,
                                          size_t txLen,
                                          uint8_t* rx,
                                          size_t rxLen);

typedef esp_err_t (*Fdc2214I2cWriteFn)(void* userCtx,
                                      uint8_t addr7,
                                      const uint8_t* tx,
                                      size_t txLen);

typedef esp_err_t (*Fdc2214CapReadReg16SequenceFn)(void *userCtx,
                                                   uint8_t addr7,
                                                   const uint8_t *regs,
                                                   size_t regCount,
                                                   uint16_t *outValues,
                                                   uint32_t timeoutMs);

typedef struct {
    uint8_t I2cAddress7;
    void* UserCtx;
    Fdc2214I2cWriteReadFn WriteRead;
    Fdc2214I2cWriteFn Write;
    Fdc2214CapReadReg16SequenceFn readReg16Sequence;
    size_t maxSequenceRegs;
    int IntGpio;
} Fdc2214CapBusConfig_t;

typedef enum {
    FDC_DATA_READ_MODE_ORDERED8 = 0,
    FDC_DATA_READ_MODE_SEQ_PAIR4,
    FDC_DATA_READ_MODE_SEQ_QUAD2,
    FDC_DATA_READ_MODE_SEQ_ALL1,
} FdcDataReadMode;

typedef enum {
    FDC2214_DEGLITCH_1MHZ = 0b001,
    FDC2214_DEGLITCH_3P3MHZ = 0b100,
    FDC2214_DEGLITCH_10MHZ = 0b101,
    FDC2214_DEGLITCH_33MHZ = 0b111,
} Fdc2214CapDeglitch_t;

typedef enum {
    FDC2214_RR_SEQUENCE_CH0_CH1 = 0,
    FDC2214_RR_SEQUENCE_CH0_CH1_CH2 = 1,
    FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3 = 2,
} Fdc2214CapRrSequence_t;

typedef struct {
    uint16_t Rcount;
    uint16_t SettleCount;
    uint16_t Offset;
    uint16_t ClockDividers;
    uint16_t DriveCurrent;
} Fdc2214CapChannelConfig_t;

typedef enum {
    FDC2214_CHANNEL_CONFIG_RESULT_OK = 0,
    FDC2214_CHANNEL_CONFIG_RESULT_WARN_DRIVE_CURRENT_MISMATCH,
} Fdc2214CapChannelConfigResult_t;

typedef enum {
    FDC2214_CHANNEL_VERIFY_RESULT_OK = 0,
    FDC2214_CHANNEL_VERIFY_RESULT_WARN_DRIVE_CURRENT_MISMATCH,
} Fdc2214CapChannelVerifyResult_t;

typedef struct {
    Fdc2214CapChannel_t ActiveChannel;
    bool SleepModeEnabled;
    bool SensorActivateSelLowPower;
    Fdc2214CapRefClockSource_t RefClockSource;
    bool IntbDisabled;
    bool HighCurrentDrive;
} Fdc2214CapConfigOptions_t;

typedef struct {
    uint16_t Raw;
    uint8_t ErrorChannel;
    bool ErrWatchdog;
    bool ErrAmplitudeHigh;
    bool ErrAmplitudeLow;
    bool DataReady;
    bool UnreadConversion[4];
} Fdc2214CapStatus_t;

#define FDC2214CAP_STATUS_ERR_CHAN_MASK        0xC000u
#define FDC2214CAP_STATUS_ERR_CHAN_SHIFT       14u
#define FDC2214CAP_STATUS_ERR_WD_MASK          0x0800u
#define FDC2214CAP_STATUS_ERR_AHW_MASK         0x0400u
#define FDC2214CAP_STATUS_ERR_ALW_MASK         0x0200u
#define FDC2214CAP_STATUS_DRDY_MASK            0x0040u
#define FDC2214CAP_STATUS_CH0_UNREAD_MASK      0x0008u
#define FDC2214CAP_STATUS_CH1_UNREAD_MASK      0x0004u
#define FDC2214CAP_STATUS_CH2_UNREAD_MASK      0x0002u
#define FDC2214CAP_STATUS_CH3_UNREAD_MASK      0x0001u
#define FDC2214CAP_STATUS_AMP_MASK \
    (FDC2214CAP_STATUS_ERR_AHW_MASK | FDC2214CAP_STATUS_ERR_ALW_MASK)

typedef struct {
    uint16_t Status;
    uint16_t StatusConfig;
    uint16_t Config;
    uint16_t MuxConfig;
} Fdc2214CapCoreRegs_t;

typedef struct {
    uint16_t Status;
    uint16_t StatusConfig;
    uint16_t Config;
    uint16_t MuxConfig;
    uint16_t RcountCh0;
    uint16_t SettleCountCh0;
    uint16_t ClockDividersCh0;
    uint16_t DriveCurrentCh0;
    Fdc2214CapChannel_t DataChannel;
    uint16_t DataMsb;
    uint16_t DataLsb;
    uint32_t DataRaw28;
    bool DataErrWatchdog;
    bool DataErrAmplitude;
    uint8_t ErrorChannel;
    bool StatusErrWatchdog;
    bool StatusErrAmplitudeHigh;
    bool StatusErrAmplitudeLow;
    bool DataReady;
    bool UnreadConversion[4];
    Fdc2214CapChannel_t ActiveChannel;
    bool SleepModeEnabled;
    bool AutoScanEnabled;
    bool Converting;
} Fdc2214CapDebugSnapshot_t;

typedef enum {
    FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN = 0,
    FDC2214_SAMPLE_STATUS_SAMPLE_VALID,
    FDC2214_SAMPLE_STATUS_STILL_SLEEPING,
    FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING,
    FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION,
    FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID,
    FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT,
    FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT,
} Fdc2214CapSampleStatus_t;

typedef struct {
    uint32_t Raw28;
    bool ErrWatchdog;
    bool ErrAmplitude;
    uint16_t StatusRaw;
    uint16_t ConfigRaw;
    uint16_t MuxRaw;
    bool SleepModeEnabled;
    bool AutoScanEnabled;
    bool Converting;
    bool UnreadConversionPresent;
    bool DataReady;
    Fdc2214CapChannel_t ActiveChannel;
    Fdc2214CapRefClockSource_t RefClockSource;
    bool SampleValid;
    Fdc2214CapSampleStatus_t SampleStatus;
} Fdc2214CapSample_t;

typedef struct {
    uint32_t raw28;
    uint16_t status;
    bool valid;
} Fdc2214CapChannelSample_t;

#define FDC2214CAP_FAST_ERROR_I2C (1u << 0)
#define FDC2214CAP_FAST_ERROR_WATCHDOG (1u << 1)
#define FDC2214CAP_FAST_ERROR_AMPLITUDE (1u << 2)
#define FDC2214CAP_FAST_ERROR_ZERO_RAW (1u << 3)
#define FDC2214CAP_FAST_ERROR_NO_UNREAD (1u << 4)
#define FDC2214CAP_FAST_ERROR_STATUS_FAULT (1u << 5)

typedef struct {
    uint32_t raw28;
    uint16_t dataMsb;
    uint16_t dataLsb;
    uint16_t statusRaw;
    bool errWatchdog;
    bool errAmplitude;
    bool unreadConversion;
    bool dataReady;
    bool valid;
    Fdc2214CapSampleStatus_t sampleStatus;
    uint32_t errorMask;
} Fdc2214CapFastChannelSample_t;

typedef struct {
    uint32_t writeCount;
    uint32_t readCount;
    uint32_t verifyReadCount;
    uint32_t writeBytes;
    uint32_t readBytes;
    uint64_t totalUs;
    uint32_t retryCount;
    uint32_t nackCount;
    uint32_t timeoutCount;
    uint32_t recoveryCount;
    uint32_t orderedDataReadCount;
    uint32_t burstDataReadCount;
    uint32_t burstProbeReadCount;
    uint32_t burstFallbackCount;
    uint32_t sequenceDataReadCount;
    uint32_t sequenceTransactionCount;
    uint32_t sequenceFallbackCount;
    uint32_t sequenceErrorCount;
} Fdc2214CapI2cStats_t;

typedef struct {
    bool supported;
    uint32_t trials;
    uint32_t mismatchCount;
    esp_err_t err;
    const char *reason;
} Fdc2214CapBurstProbeResult_t;

typedef struct {
    bool supported;
    FdcDataReadMode selectedMode;
    uint32_t trials;
    uint32_t fixedRegMismatchCount;
    uint32_t dataMismatchCount;
    uint32_t testedModeMask;
    uint32_t modeOkMask;
    uint32_t selectedElapsedUs;
    esp_err_t err;
    const char *reason;
} Fdc2214CapSequenceProbeResult_t;

// Create a device handle; the I2C callbacks are used for all transactions.
esp_err_t Fdc2214CapCreate(const Fdc2214CapBusConfig_t* busConfig, Fdc2214CapDevice_t** outDev);
// Destroy the device handle and release the mutex.
esp_err_t Fdc2214CapDestroy(Fdc2214CapDevice_t* dev);

// Write RESET_DEV with bit15 set to trigger device reset.
esp_err_t Fdc2214CapReset(Fdc2214CapDevice_t* dev);
// Read MANUFACTURER_ID and DEVICE_ID registers.
esp_err_t Fdc2214CapReadId(Fdc2214CapDevice_t* dev, uint16_t* manufacturerId, uint16_t* deviceId);

// Configure one channel (RCOUNT, SETTLECOUNT, OFFSET, CLOCK_DIVIDERS, DRIVE_CURRENT).
esp_err_t Fdc2214CapConfigureChannel(Fdc2214CapDevice_t* dev,
                                     Fdc2214CapChannel_t ch,
                                     const Fdc2214CapChannelConfig_t* cfg);
// Runtime path variant: writes channel registers and checks esp_err_t, without readback verify.
esp_err_t Fdc2214CapConfigureChannelWriteOnly(Fdc2214CapDevice_t* dev,
                                              Fdc2214CapChannel_t ch,
                                              const Fdc2214CapChannelConfig_t* cfg);
// Same as Fdc2214CapConfigureChannel, with non-fatal warning detail for DRIVE_CURRENT readback.
esp_err_t Fdc2214CapConfigureChannelWithResult(Fdc2214CapDevice_t* dev,
                                               Fdc2214CapChannel_t ch,
                                               const Fdc2214CapChannelConfig_t* cfg,
                                               Fdc2214CapChannelConfigResult_t* outResult,
                                               uint16_t* outDriveCurrentReadback);

// Read back configured channel registers and verify expected values.
esp_err_t Fdc2214CapReadbackVerifyChannelConfig(Fdc2214CapDevice_t* dev,
                                                Fdc2214CapChannel_t ch,
                                                const Fdc2214CapChannelConfig_t* expectedCfg);
// Same as Fdc2214CapReadbackVerifyChannelConfig, with non-fatal warning detail for DRIVE_CURRENT readback.
esp_err_t Fdc2214CapReadbackVerifyChannelConfigWithResult(Fdc2214CapDevice_t* dev,
                                                          Fdc2214CapChannel_t ch,
                                                          const Fdc2214CapChannelConfig_t* expectedCfg,
                                                          Fdc2214CapChannelVerifyResult_t* outResult,
                                                          uint16_t* outDriveCurrentReadback);

// Build a known-good CONFIG register value with required reserved-bit defaults.
uint16_t Fdc2214CapBuildConfig(const Fdc2214CapConfigOptions_t* options);

// Enter sleep mode explicitly by writing CONFIG with SLEEP_MODE_EN=1.
esp_err_t Fdc2214CapEnterSleep(Fdc2214CapDevice_t* dev, uint16_t configWithoutSleep);
// Exit sleep mode explicitly by writing CONFIG with SLEEP_MODE_EN=0.
esp_err_t Fdc2214CapExitSleep(Fdc2214CapDevice_t* dev, uint16_t configWithoutSleep);
// Runtime row-path variants: write CONFIG without an immediate readback.
esp_err_t Fdc2214CapEnterSleepWriteOnly(Fdc2214CapDevice_t* dev, uint16_t configWithoutSleep);
esp_err_t Fdc2214CapExitSleepWriteOnly(Fdc2214CapDevice_t* dev, uint16_t configWithoutSleep);

uint16_t Fdc2214CapUnreadMaskForChannel(Fdc2214CapChannel_t ch);
esp_err_t Fdc2214CapDecodeStatusRaw(uint16_t statusRaw,
                                    Fdc2214CapStatus_t *outStatus);
bool Fdc2214CapStatusHasAmplitudeFault(const Fdc2214CapStatus_t *status);
bool Fdc2214CapStatusHasWatchdogFault(const Fdc2214CapStatus_t *status);
bool Fdc2214CapStatusHasUnreadForChannel(const Fdc2214CapStatus_t *status,
                                         Fdc2214CapChannel_t ch);

// Read and decode STATUS register.
esp_err_t Fdc2214CapReadStatus(Fdc2214CapDevice_t* dev, Fdc2214CapStatus_t* outStatus);
// Clear latched STATUS/DATA error flags by reading STATUS and all DATA registers.
esp_err_t Fdc2214CapClearStatus(Fdc2214CapDevice_t* dev);
// Read key core registers used for diagnostics.
esp_err_t Fdc2214CapReadCoreRegs(Fdc2214CapDevice_t* dev, Fdc2214CapCoreRegs_t* outRegs);
// Read CLOCK_DIVIDERS_CHx for the requested channel.
esp_err_t Fdc2214CapReadClockDividers(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      uint16_t* outClockDividers);
// Write/read DRIVE_CURRENT_CHx, masking to effective CHx_IDRIVE bits.
esp_err_t Fdc2214CapWriteDriveCurrent(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      uint16_t driveCurrent);
esp_err_t Fdc2214CapReadDriveCurrent(Fdc2214CapDevice_t* dev,
                                     Fdc2214CapChannel_t ch,
                                     uint16_t* outDriveCurrent);
// Read one structured debug snapshot (core regs + CH0 config regs + DATA_CHx with decoded status fields).
esp_err_t Fdc2214CapReadDebugSnapshot(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t dataChannel,
                                      Fdc2214CapDebugSnapshot_t* outSnapshot);

// Single channel continuous conversion; CONFIG.ACTIVE_CHAN selects channel.
esp_err_t Fdc2214CapSetSingleChannelMode(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t activeCh);
// Autoscan conversion over CH0..CHn; use Fdc2214CapRrSequence_t for rrSequence.
esp_err_t Fdc2214CapSetAutoScanMode(Fdc2214CapDevice_t* dev, uint8_t rrSequence, Fdc2214CapDeglitch_t deglitch);
// Runtime path variant: writes CONFIG/MUX_CONFIG without readback verify.
esp_err_t Fdc2214CapSetAutoScanModeWriteOnly(Fdc2214CapDevice_t* dev,
                                             uint8_t rrSequence,
                                             Fdc2214CapDeglitch_t deglitch,
                                             uint16_t configTemplate,
                                             uint16_t* outConfig,
                                             uint16_t* outMuxConfig);

// Configure STATUS_CONFIG (ERROR_CONFIG) register; reserved bits are validated/sanitized.
esp_err_t Fdc2214CapSetStatusConfig(Fdc2214CapDevice_t* dev, uint16_t statusConfig);
// Configure MUX_CONFIG directly with explicit autoscan/rr/deglitch controls.
esp_err_t Fdc2214CapSetMuxConfig(Fdc2214CapDevice_t* dev,
                                 bool autoScan,
                                 uint8_t rrSequence,
                                 Fdc2214CapDeglitch_t deglitch);

// Read one 28-bit sample; ErrWatchdog/ErrAmplitude come from MSB bits and clear on read.
esp_err_t Fdc2214CapReadSample(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t ch, Fdc2214CapSample_t* outSample);
// Read one sample with relaxed bring-up validity: I2C ok + converting + raw!=0 is considered readable.
esp_err_t Fdc2214CapReadSampleRelaxed(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      Fdc2214CapSample_t* outSample);
// Read one channel's raw conversion and decoded status/error flags in one sample structure.
esp_err_t Fdc2214CapReadChannelRawWithStatus(Fdc2214CapDevice_t* dev,
                                             Fdc2214CapChannel_t ch,
                                             Fdc2214CapSample_t* outSample);
// Read CH0..CH3 selected by channelMask bit0..bit3. outSamples[0] maps to CH0.
esp_err_t Fdc2214CapReadChannelsRaw(Fdc2214CapDevice_t* dev,
                                    uint8_t channelMask,
                                    Fdc2214CapChannelSample_t* outSamples,
                                    size_t outSampleCount);
// Fast autoscan read for CH0..CH3: STATUS once, then DATA_MSB/DATA_LSB for each channel.
esp_err_t Fdc2214CapReadAutoscan4RawFast(Fdc2214CapDevice_t* dev,
                                         Fdc2214CapFastChannelSample_t outSamples[4]);
esp_err_t Fdc2214CapReadChannelsDataRegsFast(Fdc2214CapDevice_t* dev,
                                              uint8_t channelMask,
                                              Fdc2214CapFastChannelSample_t* outSamples,
                                              size_t outSampleCount);
esp_err_t Fdc2214CapReadChannelsDataRegsOnlyFast(Fdc2214CapDevice_t* dev,
                                                  uint8_t channelMask,
                                                  Fdc2214CapFastChannelSample_t* outSamples,
                                                  size_t outSampleCount);
// Read CH0..CH3 using the startup-probed block path when supported; otherwise
// use ordered DATA_CHx then DATA_LSB_CHx register transactions.
esp_err_t Fdc2214CapReadDataBurst4(Fdc2214CapDevice_t* dev,
                                   Fdc2214CapFastChannelSample_t outSamples[4]);
esp_err_t Fdc2214CapReadDataOrdered4(Fdc2214CapDevice_t* dev,
                                     Fdc2214CapFastChannelSample_t outSamples[4]);
// Compare ordered register reads with a 16-byte auto-increment candidate read.
// Burst mode is enabled only after every requested trial matches exactly.
esp_err_t Fdc2214CapProbeDataBurst4(Fdc2214CapDevice_t* dev,
                                    uint32_t trials,
                                    Fdc2214CapBurstProbeResult_t* outResult);
bool Fdc2214CapDataBurstSupported(const Fdc2214CapDevice_t* dev);
esp_err_t Fdc2214CapProbeDataSequence4(Fdc2214CapDevice_t *dev,
                                       uint32_t trials,
                                       Fdc2214CapSequenceProbeResult_t *outResult);
FdcDataReadMode Fdc2214CapDataReadMode(const Fdc2214CapDevice_t *dev);
const char *Fdc2214CapDataReadModeName(FdcDataReadMode mode);
size_t Fdc2214CapDataReadModeRegsPerTransaction(FdcDataReadMode mode);
size_t Fdc2214CapDataReadModeTransactionsPerRow(FdcDataReadMode mode);
bool Fdc2214CapForceOrderedDataRead(Fdc2214CapDevice_t *dev);

// Read a raw 16-bit register value.
esp_err_t Fdc2214CapReadRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue);
// Write a raw 16-bit register value.
esp_err_t Fdc2214CapWriteRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value);
// Human-readable semantic status for sample diagnostics.
const char* Fdc2214CapSampleStatusName(Fdc2214CapSampleStatus_t status);

void Fdc2214CapResetI2cStats(Fdc2214CapDevice_t* dev);
void Fdc2214CapGetI2cStats(Fdc2214CapDevice_t* dev, Fdc2214CapI2cStats_t* outStats);

void Fdc2214CapI2cTraceSetEnabled(bool enabled);
bool Fdc2214CapI2cTraceIsEnabled(void);
void Fdc2214CapI2cTraceClear(void);
void Fdc2214CapI2cTraceDump(void);

#ifdef __cplusplus
}
#endif

#endif // FDC2214CAP_H
