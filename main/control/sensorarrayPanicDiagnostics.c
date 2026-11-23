#include "sensorarrayPanicDiagnostics.h"

#include <stdio.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_private/panic_internal.h"
#include "soc/soc.h"
#include "xtensa_context.h"
#include "sdkconfig.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "Retained panic capture is pinned to the ESP32-S3 Xtensa exception frame"
#endif

#define PANIC_RECORD_VERSION 1u
#define PANIC_RECORD_MAGIC UINT32_C(0x53415031)
#define PANIC_DETAIL_BYTES 192u
#define PANIC_DETAIL_PAGE_BYTES 96u

typedef struct {
    uint32_t version, size;
    uint32_t originKnown, sessionLow, sessionHigh, bootId;
    uint8_t elfSha256[32];
    uint32_t infoKnown, core, exception, abort, pseudo;
    uint32_t frameKnown, pc, ra, sp, cause, vaddr;
    uint32_t addr, reasonPointer, detailPointer;
    uint32_t detailKnown, detailLength, detailTruncated;
    uint8_t detail[PANIC_DETAIL_BYTES];
    uint32_t crc32, inverse, commit;
} sensorarrayPanicRecord_t;

typedef struct {
    uint32_t known, sessionLow, sessionHigh, bootId;
    uint8_t elfSha256[32];
} sensorarrayPanicOrigin_t;

/* Panic writes only RTC storage. The running boot's read-only query snapshot
 * is copied once during startup; no mutex or concurrent snapshot writer is
 * introduced into the panic path. A core reset retains RTC, power loss does
 * not establish its validity. CRC is corruption detection, not authentication. */
static RTC_NOINIT_ATTR volatile sensorarrayPanicRecord_t s_retainedPanic;
static DRAM_ATTR volatile sensorarrayPanicOrigin_t s_panicOrigin;
static DRAM_ATTR volatile bool s_panicCapturedThisBoot;
static sensorarrayPanicRecord_t s_panicSnapshot;
static bool s_panicInitialized, s_panicSnapshotKnown;
static uint64_t s_panicQuerySession;
static uint32_t s_panicQueryBoot;
static esp_reset_reason_t s_panicQueryReset;

static bool IRAM_ATTR panicInternalRange(const void *pointer, size_t length,
                                         size_t alignment)
{
    uintptr_t address = (uintptr_t)pointer;
    return alignment != 0u && (alignment & (alignment - 1u)) == 0u &&
        (address & (alignment - 1u)) == 0u &&
        address >= SOC_DRAM_LOW && address < SOC_DRAM_HIGH &&
        length <= (uintptr_t)SOC_DRAM_HIGH - address;
}

static uint32_t IRAM_ATTR panicRecordCrc(const volatile sensorarrayPanicRecord_t *record)
{
    const volatile uint8_t *bytes = (const volatile uint8_t *)record;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0u; i < offsetof(sensorarrayPanicRecord_t, crc32); ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1u) ^ (UINT32_C(0xedb88320) & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static void IRAM_ATTR __attribute__((no_stack_protector))
panicCapture(const panic_info_t *info)
{
    /* The pinned SDK calls esp_panic_handler after selecting the responsible
     * core and stalling the other core. Do not add a spinlock: a panic may
     * interrupt any lock holder. Preserve the first capture if handling faults
     * recursively. All loops are compile-time bounded and use volatile access
     * so the compiler cannot replace them with flash-resident libc helpers. */
    if (s_panicCapturedThisBoot) { return; }
    s_panicCapturedThisBoot = true;
    s_retainedPanic.commit = 0u;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    volatile uint8_t *bytes = (volatile uint8_t *)&s_retainedPanic;
    for (size_t i = 0u; i < sizeof(s_retainedPanic); ++i) { bytes[i] = 0u; }
    s_retainedPanic.version = PANIC_RECORD_VERSION;
    s_retainedPanic.size = sizeof(s_retainedPanic);
    s_retainedPanic.originKnown = s_panicOrigin.known;
    s_retainedPanic.sessionLow = s_panicOrigin.sessionLow;
    s_retainedPanic.sessionHigh = s_panicOrigin.sessionHigh;
    s_retainedPanic.bootId = s_panicOrigin.bootId;
    for (size_t i = 0u; i < sizeof(s_panicOrigin.elfSha256); ++i) {
        s_retainedPanic.elfSha256[i] = s_panicOrigin.elfSha256[i];
    }
    s_retainedPanic.abort = g_panic_abort ? 1u : 0u;
    if (panicInternalRange(info, sizeof(*info), _Alignof(panic_info_t))) {
        s_retainedPanic.infoKnown = 1u;
        s_retainedPanic.core = (uint32_t)info->core;
        s_retainedPanic.exception = g_panic_abort ? PANIC_EXCEPTION_ABORT : (uint32_t)info->exception;
        s_retainedPanic.pseudo = info->pseudo_excause ? 1u : 0u;
        s_retainedPanic.addr = (uint32_t)(uintptr_t)info->addr;
        s_retainedPanic.reasonPointer = (uint32_t)(uintptr_t)info->reason;
        const XtExcFrame *frame = info->frame;
        if (panicInternalRange(frame, sizeof(*frame), _Alignof(XtExcFrame))) {
            s_retainedPanic.frameKnown = 1u;
            s_retainedPanic.pc = frame->pc;
            s_retainedPanic.ra = frame->a0;
            s_retainedPanic.sp = frame->a1;
            s_retainedPanic.cause = frame->exccause;
            s_retainedPanic.vaddr = frame->excvaddr;
        }
    }
    if (g_panic_abort) {
        const char *details = g_panic_abort_details;
        s_retainedPanic.detailPointer = (uint32_t)(uintptr_t)details;
        /* IDF newlib's assert text lives on its internal task stack. Never
         * dereference a flash/PSRAM/invalid string, walk the stack, or invoke
         * info->details: those require a stronger execution environment. */
        if (panicInternalRange(details, 1u, 1u)) {
            s_retainedPanic.detailKnown = 1u;
            s_retainedPanic.detailTruncated = 1u;
            for (size_t i = 0u; i < PANIC_DETAIL_BYTES; ++i) {
                uintptr_t address = (uintptr_t)details + i;
                if (address >= SOC_DRAM_HIGH) { break; }
                uint8_t c = *(const volatile uint8_t *)address;
                if (c == 0u) { s_retainedPanic.detailTruncated = 0u; break; }
                s_retainedPanic.detail[i] = c;
                s_retainedPanic.detailLength = (uint32_t)i + 1u;
            }
        }
    }
    s_retainedPanic.crc32 = panicRecordCrc(&s_retainedPanic);
    s_retainedPanic.inverse = ~PANIC_RECORD_MAGIC;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    s_retainedPanic.commit = PANIC_RECORD_MAGIC;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

/* The port handler references this symbol from another translation unit.
 * Keep the SDK's exact signature and return behavior (including debugger
 * paths). No reset, watchdog, console, task or peripheral behavior changes. */
void __real_esp_panic_handler(panic_info_t *info);
void IRAM_ATTR __attribute__((no_stack_protector))
__wrap_esp_panic_handler(panic_info_t *info)
{
    panicCapture(info);
    __real_esp_panic_handler(info);
}

static bool panicRecordValid(void)
{
    return s_retainedPanic.commit == PANIC_RECORD_MAGIC &&
        s_retainedPanic.inverse == ~PANIC_RECORD_MAGIC &&
        s_retainedPanic.version == PANIC_RECORD_VERSION &&
        s_retainedPanic.size == sizeof(s_retainedPanic) &&
        s_retainedPanic.originKnown <= 1u && s_retainedPanic.infoKnown <= 1u &&
        s_retainedPanic.frameKnown <= 1u && s_retainedPanic.abort <= 1u &&
        s_retainedPanic.pseudo <= 1u && s_retainedPanic.detailKnown <= 1u &&
        s_retainedPanic.detailTruncated <= 1u &&
        s_retainedPanic.detailLength <= PANIC_DETAIL_BYTES &&
        s_retainedPanic.crc32 == panicRecordCrc(&s_retainedPanic);
}

void sensorarrayPanicDiagnosticsInit(uint64_t sessionId, uint32_t bootId,
    const uint8_t elfSha256[32], esp_reset_reason_t resetReason)
{
    /* Single app_main startup writer, before any command RX is installed. */
    if (s_panicInitialized) { return; }
    s_panicQuerySession = sessionId;
    s_panicQueryBoot = bootId;
    s_panicQueryReset = resetReason;
    bool retentionAllowed = resetReason != ESP_RST_POWERON &&
        resetReason != ESP_RST_BROWNOUT && resetReason != ESP_RST_UNKNOWN;
    s_panicSnapshotKnown = retentionAllowed && panicRecordValid();
    if (s_panicSnapshotKnown) {
        uint8_t *destination = (uint8_t *)&s_panicSnapshot;
        const volatile uint8_t *source = (const volatile uint8_t *)&s_retainedPanic;
        for (size_t i = 0u; i < sizeof(s_panicSnapshot); ++i) { destination[i] = source[i]; }
    } else {
        s_retainedPanic.commit = 0u;
        s_retainedPanic.inverse = 0u;
    }
    s_panicOrigin.known = 0u;
    s_panicOrigin.sessionLow = (uint32_t)sessionId;
    s_panicOrigin.sessionHigh = (uint32_t)(sessionId >> 32u);
    s_panicOrigin.bootId = bootId;
    if (elfSha256) {
        for (size_t i = 0u; i < sizeof(s_panicOrigin.elfSha256); ++i) {
            s_panicOrigin.elfSha256[i] = elfSha256[i];
        }
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        s_panicOrigin.known = 1u;
    }
    s_panicInitialized = true;
}

bool sensorarrayPanicDiagnosticsHandles(const char *command)
{
    return command && (strcmp(command, "PANIC?") == 0 ||
        strcmp(command, "PANIC?1") == 0 || strcmp(command, "PANIC?2") == 0);
}

static void panicHex(const uint8_t *bytes, size_t length, char *output)
{
    static const char digits[] = "0123456789ABCDEF";
    for (size_t i = 0u; i < length; ++i) {
        output[2u * i] = digits[bytes[i] >> 4u];
        output[2u * i + 1u] = digits[bytes[i] & 15u];
    }
    output[2u * length] = '\0';
}

esp_err_t sensorarrayPanicDiagnosticsFormat(const char *command,
    char *response, size_t size)
{
    if (!command || !response || size == 0u) { return ESP_ERR_INVALID_ARG; }
    if (!sensorarrayPanicDiagnosticsHandles(command)) { return ESP_ERR_NOT_SUPPORTED; }
    unsigned page = command[6] ? (unsigned)(command[6] - '0') : 0u;
    int n;
    if (!s_panicSnapshotKnown) {
        n = snprintf(response, size,
            "PANIC,v=1,page=%u,session=%016llX,boot=%lu,known=0,record=unknown,reason=no_valid_retained_panic\n",
            page, (unsigned long long)s_panicQuerySession, (unsigned long)s_panicQueryBoot);
    } else if (page == 0u) {
        const sensorarrayPanicRecord_t *r = &s_panicSnapshot;
        char elf[65]; panicHex(r->elfSha256, sizeof(r->elfSha256), elf);
        n = snprintf(response, size,
            "PANIC,v=1,page=0,session=%016llX,boot=%lu,known=1,record=%08lX,originKnown=%lu,originSession=%08lX%08lX,originBoot=%lu,elf=%s,"
            "reset=%u,infoKnown=%lu,core=%lu,exception=%lu,abort=%lu,pseudo=%lu,frameKnown=%lu,pc=%08lX,ra=%08lX,sp=%08lX,cause=%08lX,vaddr=%08lX,"
            "addr=%08lX,reasonPtr=%08lX,detailPtr=%08lX,detailKnown=%lu,detailLen=%lu,truncated=%lu\n",
            (unsigned long long)s_panicQuerySession, (unsigned long)s_panicQueryBoot,
            (unsigned long)r->crc32, (unsigned long)r->originKnown,
            (unsigned long)r->sessionHigh, (unsigned long)r->sessionLow, (unsigned long)r->bootId, elf,
            (unsigned)s_panicQueryReset, (unsigned long)r->infoKnown, (unsigned long)r->core,
            (unsigned long)r->exception, (unsigned long)r->abort, (unsigned long)r->pseudo,
            (unsigned long)r->frameKnown, (unsigned long)r->pc, (unsigned long)r->ra,
            (unsigned long)r->sp, (unsigned long)r->cause, (unsigned long)r->vaddr,
            (unsigned long)r->addr, (unsigned long)r->reasonPointer, (unsigned long)r->detailPointer,
            (unsigned long)r->detailKnown, (unsigned long)r->detailLength, (unsigned long)r->detailTruncated);
    } else {
        const sensorarrayPanicRecord_t *r = &s_panicSnapshot;
        size_t offset = (page - 1u) * PANIC_DETAIL_PAGE_BYTES;
        size_t length = r->detailLength > offset ? r->detailLength - offset : 0u;
        if (length > PANIC_DETAIL_PAGE_BYTES) { length = PANIC_DETAIL_PAGE_BYTES; }
        char hex[PANIC_DETAIL_PAGE_BYTES * 2u + 1u];
        panicHex(r->detail + offset, length, hex);
        n = snprintf(response, size,
            "PANIC,v=1,page=%u,session=%016llX,boot=%lu,known=1,record=%08lX,detailKnown=%lu,offset=%u,length=%u,total=%lu,truncated=%lu,encoding=hex,text=%s\n",
            page, (unsigned long long)s_panicQuerySession, (unsigned long)s_panicQueryBoot,
            (unsigned long)r->crc32, (unsigned long)r->detailKnown, (unsigned)offset,
            (unsigned)length, (unsigned long)r->detailLength, (unsigned long)r->detailTruncated,
            r->detailKnown ? hex : "unknown");
    }
    if (n < 0 || (size_t)n >= size) {
        response[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}
