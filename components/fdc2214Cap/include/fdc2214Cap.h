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
typedef esp_err_t (*Fdc2214I2cGetBusGenerationFn)(void* userCtx, uint32_t* outGeneration);

typedef struct {
    uint8_t I2cAddress7;
    void* UserCtx;
    Fdc2214I2cWriteReadFn WriteRead;
    Fdc2214I2cWriteFn Write;
    Fdc2214I2cGetBusGenerationFn GetBusGeneration;
    int IntGpio;
} Fdc2214CapBusConfig_t;

typedef enum {
    FDC2214_DEGLITCH_1MHZ = 0b001,
    FDC2214_DEGLITCH_3P3MHZ = 0b100,
    FDC2214_DEGLITCH_10MHZ = 0b101,
    FDC2214_DEGLITCH_33MHZ = 0b111,
} Fdc2214CapDeglitch_t;

typedef struct {
    uint16_t Rcount;
    uint16_t SettleCount;
    uint16_t Offset;
    uint16_t ClockDividers;
    uint16_t DriveCurrent;
} Fdc2214CapChannelConfig_t;

#define FDC2214_CAP_DRIVE_CURRENT_MASK 0xF800u
#define FDC2214_CAP_DRIVE_STEP_MAX 31u

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

typedef struct {
    bool AnyFault;
    bool WatchdogFault;
    bool AmplitudeFault;
    bool DataReady;
    bool AnyUnreadConversion;
} Fdc2214CapStatusHealth_t;

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
    uint8_t FailedReg; // 0xFF when snapshot read completed without register-level failure.
    bool Partial;      // true when a register read failed mid-snapshot.
} Fdc2214CapDebugSnapshot_t;

typedef struct {
    uint16_t RawClockDividers;
    uint16_t NormalizedClockDividers;
    uint8_t FinSel;
    uint8_t FinDivider;
    uint16_t FrefDivider;
} Fdc2214CapClockDividerInfo_t;

typedef struct {
    double FclkHz;
    double FrefHz;
    double FinHz;
    double FsensorHz;
} Fdc2214CapFrequencyInfo_t;

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

typedef enum {
    FDC2214_TXN_OP_NONE = 0,
    FDC2214_TXN_OP_WRITE,
    FDC2214_TXN_OP_WRITE_READ,
    FDC2214_TXN_OP_REG_READ,
    FDC2214_TXN_OP_REG_WRITE,
    FDC2214_TXN_OP_SNAPSHOT,
} Fdc2214CapTxnOpKind_t;

typedef struct {
    Fdc2214CapTxnOpKind_t LastOpKind;
    uint8_t LastReg;
    size_t LastTxLen;
    size_t LastRxLen;
    esp_err_t LastErr;
    uint32_t ConsecutiveFailCount;
    uint32_t LastSuccessTick;
    uint8_t FailedReg;
} Fdc2214CapTxnDiag_t;

typedef struct {
    uint16_t ManufacturerId;
    uint16_t DeviceId;
    bool IdReadOk;
    uint16_t Status;
    uint16_t Config;
    uint16_t MuxConfig;
    uint16_t StatusConfig;
    uint16_t DriveCurrentRaw;
    uint8_t ActiveChannel;
    bool UnreadPresent;
    bool WatchdogFault;
    bool AmplitudeFault;
    bool ConfigReadbackMismatch;
    uint8_t LastFailedReg;
    esp_err_t TransportErr;
    uint32_t ConsecutiveTransportFailures;
    uint32_t BusGenerationAtCreate;
    uint32_t BusGenerationCurrent;
    uint32_t InstanceId;
    bool Stale;
} Fdc2214CapHealthSnapshot_t;

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

// Read and decode STATUS register.
esp_err_t Fdc2214CapReadStatus(Fdc2214CapDevice_t* dev, Fdc2214CapStatus_t* outStatus);
// Decode frequently used STATUS fault/readiness summary flags.
void Fdc2214CapDecodeStatusHealth(const Fdc2214CapStatus_t* status, Fdc2214CapStatusHealth_t* outHealth);
// Read STATUS and provide both raw decoded fields and summarized health flags.
esp_err_t Fdc2214CapReadStatusDecoded(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapStatus_t* outStatus,
                                      Fdc2214CapStatusHealth_t* outHealth);
// Read key core registers used for diagnostics.
esp_err_t Fdc2214CapReadCoreRegs(Fdc2214CapDevice_t* dev, Fdc2214CapCoreRegs_t* outRegs);
// Read one structured debug snapshot (core regs + CH0 config regs + DATA_CHx with decoded status fields).
esp_err_t Fdc2214CapReadDebugSnapshot(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t dataChannel,
                                      Fdc2214CapDebugSnapshot_t* outSnapshot);
esp_err_t Fdc2214CapReadHealthSnapshot(Fdc2214CapDevice_t* dev,
                                       Fdc2214CapChannel_t dataChannel,
                                       Fdc2214CapHealthSnapshot_t* outSnapshot);
// Return last transaction context for diagnostics and recovery logs.
esp_err_t Fdc2214CapGetLastTransactionDiag(Fdc2214CapDevice_t* dev, Fdc2214CapTxnDiag_t* outDiag);

// Single channel continuous conversion; CONFIG.ACTIVE_CHAN selects channel.
esp_err_t Fdc2214CapSetSingleChannelMode(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t activeCh);
// Autoscan conversion over CH0..CHn; rrSequence maps 0->CH0-CH1, 1->CH0-CH2, 2->CH0-CH3.
esp_err_t Fdc2214CapSetAutoScanMode(Fdc2214CapDevice_t* dev, uint8_t rrSequence, Fdc2214CapDeglitch_t deglitch);

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

// Read a raw 16-bit register value.
esp_err_t Fdc2214CapReadRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue);
// Write a raw 16-bit register value.
esp_err_t Fdc2214CapWriteRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value);
// Explicit drive-current helpers (canonical semantic form is logical step 0..31).
uint16_t Fdc2214CapMaskDriveCurrentRaw(uint16_t rawDriveCurrent);
bool Fdc2214CapDriveStepToRaw(uint8_t step, uint16_t* outRawDriveCurrent);
bool Fdc2214CapDriveRawToStep(uint16_t rawDriveCurrent, uint8_t* outStep, uint16_t* outMaskedRawDriveCurrent);
// Human-readable semantic status for sample diagnostics.
const char* Fdc2214CapSampleStatusName(Fdc2214CapSampleStatus_t status);

// Decode CHx_FIN_SEL and CHx_FREF_DIVIDER from a CLOCK_DIVIDERS register value.
esp_err_t Fdc2214CapDecodeClockDividers(uint16_t rawClockDividers,
                                        Fdc2214CapClockDividerInfo_t* outInfo);
// Recover physical frequencies for one raw sample:
// fRef=fClk/FREF_DIVIDER, fIn=raw/2^28*fRef, fSensor=fIn*FIN_DIVIDER.
bool Fdc2214CapComputeSensorFrequencyHz(uint32_t raw28,
                                        uint32_t fClkHz,
                                        const Fdc2214CapClockDividerInfo_t* clockDividerInfo,
                                        Fdc2214CapFrequencyInfo_t* outFrequencyInfo);

#ifdef __cplusplus
}
#endif

#endif // FDC2214CAP_H
