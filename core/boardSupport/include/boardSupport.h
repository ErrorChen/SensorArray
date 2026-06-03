#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_I2C_BUS_OK = 0,
    BOARD_I2C_BUS_BUSY,
    BOARD_I2C_BUS_SDA_LOW,
    BOARD_I2C_BUS_SCL_LOW,
    BOARD_I2C_BUS_BOTH_LOW,
    BOARD_I2C_BUS_UNKNOWN,
} BoardSupportI2cBusState_t;

typedef struct {
    i2c_port_t Port;
    uint32_t TimeoutMs;
    int SdaGpio;
    int SclGpio;
    uint32_t FrequencyHz;
    bool Enabled;
    bool Installed;
    bool Offline;
    bool Recovering;
    uint32_t InstallCount;
    uint32_t DeleteCount;
    uint32_t TransactionCount;
    uint32_t NackCount;
    uint32_t TimeoutCount;
    uint32_t BusStuckCount;
    uint32_t RecoveryCount;
    uint32_t RecoveryFailCount;
    int64_t LastRecoveryUs;
    int64_t LastTransactionUs;
    SemaphoreHandle_t Mutex;
} BoardSupportI2cCtx_t;

typedef struct {
    bool Enabled;
    bool Installed;
    bool Offline;
    bool Recovering;
    i2c_port_t Port;
    int SdaGpio;
    int SclGpio;
    uint32_t FrequencyHz;
    uint32_t InstallCount;
    uint32_t DeleteCount;
    uint32_t TransactionCount;
    uint32_t NackCount;
    uint32_t TimeoutCount;
    uint32_t BusStuckCount;
    uint32_t RecoveryCount;
    uint32_t RecoveryFailCount;
} BoardSupportI2cBusInfo_t;

// Initialize board-level buses (I2C primary and optional secondary).
esp_err_t boardSupportInit(void);
// Deinitialize buses initialized by boardSupportInit.
esp_err_t boardSupportDeinit(void);

// Returns true if the optional second I2C bus has a legal enabled configuration.
bool boardSupportIsI2c1Enabled(void);

// Returns the default I2C context for the primary bus.
const BoardSupportI2cCtx_t* boardSupportGetI2cCtx(void);
// Returns the default I2C context for the optional second bus, or NULL if disabled/unavailable.
const BoardSupportI2cCtx_t* boardSupportGetI2c1Ctx(void);
// Returns configured bus metadata for the selected board-level I2C bus.
bool boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *outInfo);
// Recover an initialized board-level I2C bus after a timed-out or failed transaction.
esp_err_t boardSupportRecoverI2cBus(const BoardSupportI2cCtx_t *ctx);

// Convenience I2C callbacks matching Fdc2214Cap bus config signatures.
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

esp_err_t boardSupportI2cRead(void* userCtx,
                             uint8_t addr7,
                             uint8_t* rx,
                             size_t rxLen);

// Probe I2C address with a START + address byte + STOP transaction.
esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *i2cCtx, uint8_t addr7);

#ifdef __cplusplus
}
#endif
