#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t Port;
    uint32_t TimeoutMs;
} BoardSupportI2cCtx_t;

typedef struct {
    bool Enabled;
    i2c_port_t Port;
    int SdaGpio;
    int SclGpio;
    uint32_t FrequencyHz;
} BoardSupportI2cBusInfo_t;

typedef enum {
    BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE = 0,
    BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY,
    BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY,
    BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL,
} BoardSupportI2cRecoveryLevel_t;

typedef enum {
    BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE = 0,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_SINGLE_NACK,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_REPEATED_NACK,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_TIMEOUT,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST,

    // Legacy names kept for compatibility with existing call sites.
    BOARD_SUPPORT_I2C_RECOVERY_REASON_PREOP_LINE_STUCK,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_TRANSFER_TIMEOUT,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_TRANSFER_ERROR_STREAK,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_DURING_TRANSFER,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_SCL_HELD_LOW,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_SCL_HELD_LOW_AFTER_PULSE,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_FAIL_DRIVER_REINIT,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_GPIO_CONFIG_FAILED,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_REINIT,
    BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_RECOVER,
} BoardSupportI2cRecoveryReason_t;

typedef struct {
    BoardSupportI2cRecoveryReason_t Reason;
    i2c_port_t Port;
    BoardSupportI2cRecoveryLevel_t RequestedLevel;
    BoardSupportI2cRecoveryLevel_t AttemptedLevel;
    int PreSclHigh;
    int PreSdaHigh;
    uint32_t PulsesIssued;
    bool DeleteAttempted;
    esp_err_t DeleteErr;
    bool InitAttempted;
    esp_err_t InitErr;
    int PostSclHigh;
    int PostSdaHigh;
    bool Success;
    uint32_t BusGenerationAfter;
} BoardSupportI2cRecoveryResult_t;

typedef struct {
    bool Valid;
    i2c_port_t Port;
    BoardSupportI2cRecoveryReason_t Reason;
    BoardSupportI2cRecoveryLevel_t RecoveryLevel;
    esp_err_t LastTransferErr;
    esp_err_t LastRecoverErr;
    esp_err_t LastReinitErr;
    uint32_t FailureStreak;
    bool RecoverAttempted;
    bool RecoverFailed;
    bool ReinitAttempted;
    bool ReinitSucceeded;
    bool DeleteAttempted;
    esp_err_t DeleteErr;
    bool InitAttempted;
    esp_err_t InitErr;
    uint32_t PulsesIssued;
    int SclHigh;
    int SdaHigh;
    uint32_t BusGeneration;
} BoardSupportI2cRecoveryStatus_t;

// Initialize board-level buses (I2C primary and optional secondary).
esp_err_t boardSupportInit(void);
// Deinitialize buses initialized by boardSupportInit.
esp_err_t boardSupportDeinit(void);

// Returns true if the optional second I2C bus is configured/enabled.
bool boardSupportIsI2c1Enabled(void);

// Returns the default I2C context for the primary bus.
const BoardSupportI2cCtx_t* boardSupportGetI2cCtx(void);
// Returns the default I2C context for the optional second bus, or NULL if disabled.
const BoardSupportI2cCtx_t* boardSupportGetI2c1Ctx(void);
// Returns configured bus metadata for the selected board-level I2C bus.
bool boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *outInfo);

// Convenience I2C callbacks matching Fdc2214Cap bus config signatures.
// All boardSupport I2C APIs are serialized with one mutex per I2C port.
esp_err_t boardSupportI2cWriteRead(void* userCtx,
                                  uint8_t addr7,
                                  const uint8_t* tx,
                                  size_t txLen,
                                  uint8_t* rx,
                                  size_t rxLen);

esp_err_t boardSupportI2cWrite(void* userCtx,
                              uint8_t addr7,
                              const uint8_t* tx,
                              size_t txLen);

// Read current bus line levels (true = released/high, false = low/stuck).
esp_err_t boardSupportI2cCheckLines(const BoardSupportI2cCtx_t *i2cCtx,
                                    bool *outSclHigh,
                                    bool *outSdaHigh);
// Re-install the I2C driver using the board's configured pins/frequency for this context.
esp_err_t boardSupportI2cReinit(const BoardSupportI2cCtx_t *i2cCtx, const char *reason);
// Attempt stuck-bus recovery (9 SCL pulses + STOP) and reinitialize the driver.
esp_err_t boardSupportI2cRecoverBus(const BoardSupportI2cCtx_t *i2cCtx, const char *reason);
// Explicit manual recover flow with diagnostic logging for dedicated debug mode.
esp_err_t boardSupportI2cManualRecover(const BoardSupportI2cCtx_t *i2cCtx,
                                       uint8_t addr7,
                                       const char *reason,
                                       uint32_t failureStreakHint);
esp_err_t boardSupportI2cRecover(const BoardSupportI2cCtx_t *i2cCtx,
                                 BoardSupportI2cRecoveryReason_t reason,
                                 BoardSupportI2cRecoveryLevel_t requestedLevel,
                                 uint8_t addr7,
                                 uint32_t failureStreakHint,
                                 BoardSupportI2cRecoveryResult_t *outResult);
esp_err_t boardSupportI2cGetPortGeneration(const BoardSupportI2cCtx_t *i2cCtx, uint32_t *outGeneration);
esp_err_t boardSupportI2cGetBusGeneration(void *userCtx, uint32_t *outGeneration);

// Returns latest recovery-related status captured for this bus.
bool boardSupportI2cGetLastRecoveryStatus(const BoardSupportI2cCtx_t *i2cCtx,
                                          BoardSupportI2cRecoveryStatus_t *outStatus);

// Returns text name for recovery-reason enum.
const char *boardSupportI2cRecoveryReasonName(BoardSupportI2cRecoveryReason_t reason);
const char *boardSupportI2cRecoverReasonToString(BoardSupportI2cRecoveryReason_t reason);

// Probe I2C address with a START + address byte + STOP transaction.
esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *i2cCtx, uint8_t addr7);

#ifdef __cplusplus
}
#endif
