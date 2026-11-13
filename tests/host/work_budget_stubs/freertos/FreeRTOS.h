#pragma once
#include <assert.h>
#include <stdint.h>
typedef uint32_t TickType_t;
typedef unsigned UBaseType_t;
#define configTICK_RATE_HZ 100u
#define portTICK_TYPE_IS_ATOMIC 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((ms) / 10u)
#define pdTRUE 1
#define pdPASS 1
#define configASSERT(x) assert(x)
