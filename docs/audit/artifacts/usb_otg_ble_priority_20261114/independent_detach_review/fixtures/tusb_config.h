#pragma once
#include "esp_attr.h"
#include "sdkconfig.h"
#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 1
#define CFG_TUH_ENABLED 0
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_DWC2_DMA_ENABLE 1
#define CFG_TUD_DWC2_SLAVE_ENABLE 0
#define CFG_TUD_MEM_DCACHE_ENABLE 0
#define CFG_TUSB_MEM_SECTION DRAM_ATTR
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUD_TASK_EVENTS_PER_RUN 16
#define CFG_TUD_TASK_QUEUE_SZ 32
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
#define CFG_TUD_CDC 2
#define CFG_TUD_VENDOR 0
#define CFG_TUD_CDC_TX_BUFSIZE 4096
#define CFG_TUD_CDC_TX_EPSIZE 512 /* 8 FS packets per DMA transfer; MPS stays 64. */
#else
#define CFG_TUD_CDC 1
#define CFG_TUD_VENDOR 2
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_TX_EPSIZE 64
#endif
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_CDC_RX_BUFSIZE 128
#define CFG_TUD_CDC_RX_EPSIZE 64
/* DTR is not a DMA fence. Never overwrite accepted bytes on close/reopen;
 * a processed bus reset clears both FIFOs before the new epoch is served. */
#define CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED 0
#define CFG_TUD_CDC_RX_PERSISTENT 0
#define CFG_TUD_CDC_TX_PERSISTENT 0
#define CFG_TUD_VENDOR_RX_BUFSIZE 512
#define CFG_TUD_VENDOR_TX_BUFSIZE 4096
#define CFG_TUD_VENDOR_RX_EPSIZE 64
#define CFG_TUD_VENDOR_TX_EPSIZE 512 /* 8 FS packets per DMA transfer; MPS stays 64. */
