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

#ifdef __cplusplus
}
#endif
