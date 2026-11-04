#!/usr/bin/env python3
"""Execute actual SerialIo queue/TX/VFS code with synthetic driver results."""
from pathlib import Path
import os
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="sensorarray-serial-io-") as temp:
    temp = Path(temp)
    (temp / "freertos").mkdir()
    (temp / "driver").mkdir()
    common = '''#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "esp_err.h"
typedef uint32_t TickType_t;
typedef uint8_t StackType_t;
typedef int BaseType_t;
typedef struct { unsigned length, count, read, write; uint8_t *data; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef void *TaskHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(n) (n)
QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *bytes, StaticQueue_t *queue);
int xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks);
int xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks);
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, unsigned stack, void *arg, unsigned prio, TaskHandle_t *task, int core);
int xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(int clear, TickType_t ticks);
void vTaskDelay(TickType_t ticks);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task);
int64_t esp_timer_get_time(void);
typedef struct { uint32_t tx_buffer_size, rx_buffer_size; } usb_serial_jtag_driver_config_t;
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *config);
bool usb_serial_jtag_is_driver_installed(void);
bool usb_serial_jtag_is_connected(void);
int usb_serial_jtag_write_bytes(const void *data, size_t length, TickType_t ticks);
int usb_serial_jtag_read_bytes(void *data, uint32_t length, TickType_t ticks);
esp_err_t usb_serial_jtag_wait_tx_done(TickType_t ticks);
#define ESP_VFS_FLAG_DEFAULT 0
struct esp_vfs {
    int flags;
    int (*open)(const char *, int, int);
    ssize_t (*write)(int, const void *, size_t);
    int (*close)(int);
    int (*fstat)(int, struct stat *);
};
typedef struct esp_vfs esp_vfs_t;
esp_err_t esp_vfs_register(const char *path, const esp_vfs_t *vfs, void *context);
'''
    (temp / "common.h").write_text(common)
    for file in ["freertos/FreeRTOS.h", "freertos/task.h", "freertos/queue.h", "driver/usb_serial_jtag.h", "esp_timer.h", "esp_vfs.h"]:
        (temp / file).write_text('#include "common.h"\n')
    (temp / "esp_heap_caps.h").write_text("#include <stddef.h>\n#include <stdint.h>\n#define MALLOC_CAP_8BIT 1u\n#define MALLOC_CAP_SPIRAM 2u\n#define MALLOC_CAP_INTERNAL 4u\nvoid *heap_caps_calloc(size_t count, size_t size, uint32_t caps);\n")
    (temp / "sdkconfig.h").write_text("#define CONFIG_SENSORARRAY_OUTPUT_TASK_CORE 0\n")
    exe = temp / "test"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-D_DEFAULT_SOURCE", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-Dfreopen=test_freopen", "-Dsetvbuf=test_setvbuf", "-Dclearerr=test_clearerr", "-I", str(temp), "-I", str(root / "tests/host/include"), "-I", str(root / "components/sensorarrayWifi/include"), "-I", str(root / "core/transport"), str(root / "tests/host/test_serial_io.c"), "-o", str(exe)], check=True)
    for allocation in ["psram", "internal", "failure_retry"]:
        subprocess.run([str(exe), allocation], check=True)
