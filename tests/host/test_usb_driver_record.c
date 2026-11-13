/* Actual IDF driver write and BYTEBUF fit/copy functions are extracted by
 * test_serial_io.py. Only the RTOS wait/ISR scheduling surface is simulated. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int BaseType_t;
typedef unsigned TickType_t;
#define pdFALSE 0
#define pdTRUE 1
#define configASSERT assert
#define rbBUFFER_FULL_FLAG 1u
#define rbBYTE_BUFFER_FLAG 2u
typedef struct {
    size_t xSize, xMaxItemSize, xItemsWaiting;
    unsigned uxRingbufferFlags;
    uint8_t *pucAcquire, *pucFree, *pucHead, *pucTail, *pucWrite;
} Ringbuffer_t;
typedef Ringbuffer_t *RingbufHandle_t;
#include "idf_ring_bytebuf.h"
static TickType_t ringWait;
static BaseType_t prvSendAcquireGeneric(Ringbuffer_t *ring, const void *data,
                                       void **unused, size_t length, TickType_t ticks)
{
    (void)unused; ringWait = ticks;
    if (!prvCheckItemFitsByteBuffer(ring, length)) { return pdFALSE; }
    prvCopyItemByteBuf(ring, data, length);
    return pdTRUE;
}
#include "idf_ring_send.h"
typedef int *SemaphoreHandle_t;
static TickType_t mutexWait;
static int mux = 1, idle;
static unsigned interruptEnableCalls;
static BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks)
{
    if (semaphore == &mux) { mutexWait = ticks; }
    if (*semaphore) { *semaphore = 0; return pdTRUE; }
    return pdFALSE;
}
static void xSemaphoreGive(SemaphoreHandle_t semaphore) { *semaphore = 1; }
#define USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY 1u
static void usb_serial_jtag_ll_ena_intr_mask(unsigned mask)
{
    assert(mask == USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY); interruptEnableCalls++;
}
#define ESP_RETURN_ON_FALSE(condition, value, ...) do { if (!(condition)) { return value; } } while (0)
typedef struct { SemaphoreHandle_t tx_mux, tx_idle_sem; RingbufHandle_t tx_ring_buf; } driver_t;
static driver_t *p_usb_serial_jtag_obj;
#include "idf_usb_write.h"

int main(void)
{
    uint8_t bytes[2304], record[2304], unchanged[2304];
    for (size_t i = 0; i < sizeof(record); ++i) { record[i] = (uint8_t)(i % 251u); }
    Ringbuffer_t ring = {.xSize = sizeof(bytes), .xMaxItemSize = sizeof(bytes),
        .pucHead = bytes, .pucTail = bytes + sizeof(bytes), .pucFree = bytes,
        .pucAcquire = bytes, .pucWrite = bytes, .uxRingbufferFlags = rbBYTE_BUFFER_FLAG};
    driver_t driver = {.tx_mux = &mux, .tx_idle_sem = &idle, .tx_ring_buf = &ring};
    p_usb_serial_jtag_obj = &driver;
    assert(prvGetCurMaxSizeByteBuf(&ring) == sizeof(record));
    assert(usb_serial_jtag_write_bytes(record, sizeof(record), 1) == (int)sizeof(record));
    assert(ring.xItemsWaiting == sizeof(record) && memcmp(bytes, record, sizeof(record)) == 0);
    assert(prvGetCurMaxSizeByteBuf(&ring) == 0 && mutexWait == 1 && ringWait == 1);
    memcpy(unchanged, bytes, sizeof(bytes));
    assert(usb_serial_jtag_write_bytes("CTRL\n", 5, 1) == 0);
    assert(memcmp(bytes, unchanged, sizeof(bytes)) == 0); /* no prefix on failure */

    /* An empty byte ring with wrapped pointers still admits its entire nominal
     * capacity; no no-split item header/half-size restriction applies. */
    ring.uxRingbufferFlags = rbBYTE_BUFFER_FLAG; ring.xItemsWaiting = 0;
    ring.pucAcquire = ring.pucFree = ring.pucWrite = bytes + 127;
    assert(prvGetCurMaxSizeByteBuf(&ring) == sizeof(record));
    assert(usb_serial_jtag_write_bytes(record, sizeof(record), 1) == (int)sizeof(record));
    assert(memcmp(bytes + 127, record, sizeof(record) - 127) == 0);
    assert(memcmp(bytes, record + sizeof(record) - 127, 127) == 0);
    assert(ring.xItemsWaiting == sizeof(record));

    /* A record larger than remaining capacity is wholly rejected, even when
     * the ring has enough space for an old 64-byte chunk. */
    ring.uxRingbufferFlags = rbBYTE_BUFFER_FLAG; ring.xItemsWaiting = 0;
    ring.pucAcquire = ring.pucFree = ring.pucWrite = bytes;
    assert(usb_serial_jtag_write_bytes(record, 64, 1) == 64);
    memcpy(unchanged, bytes, sizeof(bytes));
    assert(usb_serial_jtag_write_bytes(record, sizeof(record), 1) == 0);
    assert(ring.xItemsWaiting == 64 && memcmp(bytes, unchanged, sizeof(bytes)) == 0);

    /* Reusing an unknown/smaller installed ring is not safe for whole records. */
    ring.xSize = ring.xMaxItemSize = 1024; ring.xItemsWaiting = 0;
    ring.pucTail = bytes + 1024; ring.pucAcquire = ring.pucFree = ring.pucWrite = bytes;
    assert(usb_serial_jtag_write_bytes(record, sizeof(record), 1) == 0);
    assert(ring.xItemsWaiting == 0);
    assert(interruptEnableCalls == 6 && mux == 1);
    puts("IDF_USB_DRIVER_RECORD_PASS,actualDriverWrite=1,actualByteBufferFitCopy=1,capacity=2304,emptyWrapped=1,allOrZero=1,smallerRingRejected=1");
    return 0;
}
