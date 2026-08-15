#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int i2c_port_t;
typedef struct i2c_cmd_t *i2c_cmd_handle_t;

#define I2C_LINK_RECOMMENDED_SIZE(transactions) (16u * (transactions))

#ifdef __cplusplus
}
#endif
