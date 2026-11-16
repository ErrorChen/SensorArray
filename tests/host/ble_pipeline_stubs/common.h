#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef void *TaskHandle_t;
typedef int portMUX_TYPE;
typedef struct {
    uint8_t *storage;
    unsigned capacity, itemSize, count, head, tag;
} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef struct { unsigned token; int waiter; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define portTICK_PERIOD_MS 10u
#define pdMS_TO_TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#define configTICK_RATE_HZ 100u
#define portTICK_TYPE_IS_ATOMIC 1
#define configASSERT(x) assert(x)
extern unsigned pipelineLockDepth;
#define portENTER_CRITICAL(p) do { (void)(p); pipelineLockDepth++; } while (0)
#define portEXIT_CRITICAL(p) do { (void)(p); assert(pipelineLockDepth); pipelineLockDepth--; } while (0)
TickType_t xTaskGetTickCount(void);
int xPortGetCoreID(void);
void vTaskDelay(TickType_t ticks);
int64_t esp_timer_get_time(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
UBaseType_t uxTaskPriorityGet(TaskHandle_t task);
QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *storage, StaticQueue_t *queue);
int xQueueSend(QueueHandle_t queue, const void *item, TickType_t timeout);
int xQueueReceive(QueueHandle_t queue, void *out, TickType_t timeout);
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *semaphore);
int xSemaphoreGive(SemaphoreHandle_t semaphore);
int xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout);
int xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(int clear, TickType_t timeout);
