/* Actual production module, synthetic internal address range and SDK tail. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayPanicDiagnostics.c"

uintptr_t testDramLow, testDramHigh;
bool g_panic_abort;
char *g_panic_abort_details;
void *g_exc_frames[2];
static unsigned realCalls;
static panic_info_t *realInfo;
static bool reenter;
static struct {
    panic_info_t info;
    XtExcFrame frame;
    char text[256];
} arena;
static uint8_t elf[32];

void __real_esp_panic_handler(panic_info_t *info)
{
    ++realCalls;
    realInfo = info;
    if (reenter) {
        reenter = false;
        arena.frame.pc = 0x42424242u;
        __wrap_esp_panic_handler(info);
    }
}

static void simulateBoot(void)
{
    /* The fake reset preserves only the RTC_NOINIT object. */
    memset((void *)&s_panicOrigin, 0, sizeof(s_panicOrigin));
    memset(&s_panicSnapshot, 0, sizeof(s_panicSnapshot));
    s_panicCapturedThisBoot = false;
    s_panicInitialized = s_panicSnapshotKnown = false;
    s_panicQuerySession = 0u;
    s_panicQueryBoot = 0u;
    s_panicQueryReset = ESP_RST_UNKNOWN;
    g_panic_abort = false;
    g_panic_abort_details = NULL;
    reenter = false;
    realCalls = 0u;
    realInfo = NULL;
}

static void fixture(void)
{
    memset((void *)&s_retainedPanic, 0, sizeof(s_retainedPanic));
    simulateBoot();
    memset(&arena, 0, sizeof(arena));
    testDramLow = (uintptr_t)&arena;
    testDramHigh = (uintptr_t)&arena + sizeof(arena);
    for (unsigned i = 0u; i < sizeof(elf); ++i) { elf[i] = (uint8_t)i; }
    arena.frame.pc = 0x42012345u;
    arena.frame.a0 = 0x82023456u;
    arena.frame.a1 = 0x3fc81230u;
    arena.frame.exccause = 28u;
    arena.frame.excvaddr = 0x00000004u;
    arena.info = (panic_info_t){.core = 1, .exception = PANIC_EXCEPTION_FAULT,
        .reason = (const char *)(uintptr_t)0x3c123456u,
        .addr = (const void *)(uintptr_t)0x42012345u, .frame = &arena.frame};
}

static void beginOrigin(void)
{
    sensorarrayPanicDiagnosticsInit(UINT64_C(0x0123456789abcdef), 41u, elf, ESP_RST_POWERON);
}

static void queryKnown(bool known)
{
    char response[512];
    assert(sensorarrayPanicDiagnosticsFormat("PANIC?", response, sizeof(response)) == ESP_OK);
    assert(strstr(response, known ? ",known=1," : ",known=0,"));
}

static void nextBoot(esp_reset_reason_t reason)
{
    simulateBoot();
    sensorarrayPanicDiagnosticsInit(UINT64_C(0xfedcba9876543210), 42u, elf, reason);
}

static void basicAndReadOnly(void)
{
    fixture();
    assert(!sensorarrayPanicDiagnosticsHandles(NULL));
    assert(!sensorarrayPanicDiagnosticsHandles("PANIC=CRASH"));
    assert(!sensorarrayPanicDiagnosticsHandles("PANIC?0"));
    assert(!sensorarrayPanicDiagnosticsHandles("PANIC?3"));
    assert(sensorarrayPanicDiagnosticsHandles("PANIC?") &&
           sensorarrayPanicDiagnosticsHandles("PANIC?1") &&
           sensorarrayPanicDiagnosticsHandles("PANIC?2"));
    char response[512];
    assert(sensorarrayPanicDiagnosticsFormat(NULL, response, 512u) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayPanicDiagnosticsFormat("PANIC?", NULL, 512u) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayPanicDiagnosticsFormat("PANIC?", response, 0u) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayPanicDiagnosticsFormat("PANIC?3", response, 512u) == ESP_ERR_NOT_SUPPORTED);
    queryKnown(false);
    beginOrigin();
    __wrap_esp_panic_handler(&arena.info);
    assert(realCalls == 1u && realInfo == &arena.info && panicRecordValid());
    assert(s_retainedPanic.frameKnown == 1u && s_retainedPanic.pc == 0x42012345u);
    queryKnown(false); /* No running-query race with newly captured RTC data. */
    nextBoot(ESP_RST_PANIC);
    assert(s_panicSnapshotKnown);
    assert(sensorarrayPanicDiagnosticsFormat("PANIC?", response, 512u) == ESP_OK);
    assert(strstr(response, "session=FEDCBA9876543210,boot=42,known=1,"));
    assert(strstr(response, "originKnown=1,originSession=0123456789ABCDEF,originBoot=41,"));
    assert(strstr(response, "elf=000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F,"));
    assert(strstr(response, "frameKnown=1,pc=42012345,ra=82023456,sp=3FC81230,cause=0000001C,vaddr=00000004,"));
    sensorarrayPanicRecord_t retained = s_retainedPanic, snapshot = s_panicSnapshot;
    char original[512]; strcpy(original, response);
    sensorarrayPanicDiagnosticsInit(1u, 1u, NULL, ESP_RST_POWERON); /* Idempotent. */
    assert(sensorarrayPanicDiagnosticsFormat("PANIC?", response, 512u) == ESP_OK);
    assert(!strcmp(original, response));
    for (unsigned i = 0u; i < 10u; ++i) {
        assert(sensorarrayPanicDiagnosticsFormat("PANIC?1", response, 512u) == ESP_OK);
        assert(strstr(response, "detailKnown=0,") && strstr(response, "text=unknown\n"));
        assert(sensorarrayPanicDiagnosticsFormat("PANIC?2", response, 512u) == ESP_OK);
    }
    assert(!memcmp(&snapshot, &s_panicSnapshot, sizeof(snapshot)));
    assert(!memcmp(&retained, (const void *)&s_retainedPanic, sizeof(retained)));
    assert(realCalls == 0u);
    puts("PANIC_PASS basic_identity_raw_registers_readonly_idempotent");
}

static void detailsAndBounds(void)
{
    for (unsigned length = 0u; length <= 193u; ++length) {
        fixture(); beginOrigin();
        for (unsigned i = 0u; i < length; ++i) { arena.text[i] = (char)('!' + i % 80u); }
        arena.text[length] = '\0';
        g_panic_abort = true; g_panic_abort_details = arena.text;
        __wrap_esp_panic_handler(&arena.info);
        assert(realCalls == 1u && s_retainedPanic.abort == 1u);
        assert(s_retainedPanic.exception == PANIC_EXCEPTION_ABORT);
        assert(s_retainedPanic.detailKnown == 1u);
        assert(s_retainedPanic.detailLength == (length < 192u ? length : 192u));
        assert(s_retainedPanic.detailTruncated == (length >= 192u));
        assert(!memcmp((const void *)s_retainedPanic.detail, arena.text, s_retainedPanic.detailLength));
        nextBoot(ESP_RST_PANIC);
        char response[512], wanted[193];
        for (unsigned page = 1u; page <= 2u; ++page) {
            const char *command = page == 1u ? "PANIC?1" : "PANIC?2";
            assert(sensorarrayPanicDiagnosticsFormat(command, response, 512u) == ESP_OK);
            unsigned offset = (page - 1u) * 96u;
            unsigned count = s_panicSnapshot.detailLength > offset ? s_panicSnapshot.detailLength - offset : 0u;
            if (count > 96u) { count = 96u; }
            panicHex((const uint8_t *)arena.text + offset, count, wanted);
            const char *hex = strstr(response, ",text="); assert(hex);
            assert(strlen(hex + 6u) == count * 2u + 1u);
            assert(!strncmp(hex + 6u, wanted, count * 2u));
        }
    }
    fixture(); beginOrigin();
    g_panic_abort = true; g_panic_abort_details = (char *)(uintptr_t)0x3c000020u;
    __wrap_esp_panic_handler(&arena.info);
    assert(realCalls == 1u && s_retainedPanic.detailKnown == 0u);
    assert(s_retainedPanic.detailPointer == 0x3c000020u);
    fixture(); beginOrigin();
    arena.text[0] = 'A'; arena.text[1] = 'B';
    testDramHigh = (uintptr_t)arena.text + 2u;
    g_panic_abort = true; g_panic_abort_details = arena.text;
    __wrap_esp_panic_handler(&arena.info);
    assert(s_retainedPanic.detailKnown && s_retainedPanic.detailLength == 2u && s_retainedPanic.detailTruncated);
    puts("PANIC_PASS actual_detail_copy_194_lengths_hex_pages_flash_and_end_bound");
}

static void invalidFrameAndRecursion(void)
{
    for (unsigned kind = 0u; kind < 4u; ++kind) {
        fixture(); beginOrigin();
        arena.info.frame = kind == 0u ? NULL : kind == 1u ? (void *)((uintptr_t)&arena.frame + 1u) :
            kind == 2u ? (void *)(testDramHigh - 4u) : (void *)(UINTPTR_MAX - 3u);
        __wrap_esp_panic_handler(&arena.info);
        assert(realCalls == 1u && s_retainedPanic.infoKnown && !s_retainedPanic.frameKnown);
        assert(panicRecordValid());
    }
    for (unsigned kind = 0u; kind < 3u; ++kind) {
        fixture(); beginOrigin();
        panic_info_t *info = kind == 0u ? NULL : kind == 1u ?
            (void *)((uintptr_t)&arena.info + 1u) : (void *)(testDramHigh - sizeof(void *));
        __wrap_esp_panic_handler(info);
        assert(realCalls == 1u && realInfo == info && !s_retainedPanic.infoKnown && !s_retainedPanic.frameKnown);
        assert(panicRecordValid());
    }
    fixture(); beginOrigin(); reenter = true;
    __wrap_esp_panic_handler(&arena.info);
    assert(realCalls == 2u); /* Once per entry, no swallowed SDK behavior. */
    assert(s_retainedPanic.pc == 0x42012345u && panicRecordValid());
    puts("PANIC_PASS invalid_info_frame_no_dereference_first_capture_recursive_tail");
}

static void retentionAndCorruption(void)
{
    fixture();
    __wrap_esp_panic_handler(&arena.info); /* Panic before normal identity init. */
    assert(s_retainedPanic.originKnown == 0u);
    nextBoot(ESP_RST_PANIC); queryKnown(true);
    assert(s_panicSnapshot.originKnown == 0u);
    for (unsigned kind = 0u; kind < 8u; ++kind) {
        fixture(); beginOrigin(); __wrap_esp_panic_handler(&arena.info);
        switch (kind) {
        case 0u: s_retainedPanic.commit ^= 1u; break;
        case 1u: s_retainedPanic.inverse ^= 1u; break;
        case 2u: s_retainedPanic.crc32 ^= 1u; break;
        case 3u: s_retainedPanic.pc ^= 1u; break;
        case 4u: s_retainedPanic.version = 2u; break;
        case 5u: s_retainedPanic.size -= 4u; break;
        case 6u: s_retainedPanic.detailLength = 193u; s_retainedPanic.crc32 = panicRecordCrc(&s_retainedPanic); break;
        default: s_retainedPanic.frameKnown = 2u; s_retainedPanic.crc32 = panicRecordCrc(&s_retainedPanic); break;
        }
        nextBoot(ESP_RST_PANIC); queryKnown(false);
    }
    esp_reset_reason_t cold[] = {ESP_RST_POWERON, ESP_RST_BROWNOUT, ESP_RST_UNKNOWN};
    for (unsigned i = 0u; i < sizeof(cold)/sizeof(cold[0]); ++i) {
        fixture(); beginOrigin(); __wrap_esp_panic_handler(&arena.info);
        nextBoot(cold[i]); queryKnown(false);
        assert(s_retainedPanic.commit == 0u);
    }
    fixture(); beginOrigin(); __wrap_esp_panic_handler(&arena.info);
    nextBoot(ESP_RST_SW); queryKnown(true); /* Last retained panic, not current reset cause. */
    assert(s_panicQueryReset == ESP_RST_SW && s_panicSnapshot.bootId == 41u);
    puts("PANIC_PASS early_origin_cold_boot_corrupt_commit_crc_shape_prior_panic_retained");
}

static void formatterBounds(void)
{
    fixture(); beginOrigin();
    memset(arena.text, 0xff, sizeof(arena.text));
    g_panic_abort = true; g_panic_abort_details = arena.text;
    arena.info.core = -1; arena.info.exception = (panic_exception_t)UINT32_MAX;
    arena.info.pseudo_excause = true;
    arena.frame.pc = arena.frame.a0 = arena.frame.a1 = arena.frame.exccause = arena.frame.excvaddr = UINT32_MAX;
    __wrap_esp_panic_handler(&arena.info);
    s_retainedPanic.bootId = s_retainedPanic.sessionLow = s_retainedPanic.sessionHigh = UINT32_MAX;
    s_retainedPanic.exception = UINT32_MAX;
    s_retainedPanic.crc32 = panicRecordCrc(&s_retainedPanic);
    nextBoot(ESP_RST_PANIC);
    s_panicQuerySession = UINT64_MAX; s_panicQueryBoot = UINT32_MAX;
    s_panicQueryReset = (esp_reset_reason_t)UINT32_MAX;
    const char *commands[] = {"PANIC?", "PANIC?1", "PANIC?2"};
    char output[1024];
    for (unsigned page = 0u; page < 3u; ++page) {
        assert(sensorarrayPanicDiagnosticsFormat(commands[page], output, sizeof(output)) == ESP_OK);
        size_t needed = strlen(output) + 1u;
        printf("PANIC_MAX page=%u bytes=%zu includingNul=%zu capacity=512\n", page, needed-1u, needed);
        assert(needed <= 512u);
        for (size_t cap = 1u; cap < needed; ++cap) {
            memset(output, 0x5a, sizeof(output));
            assert(sensorarrayPanicDiagnosticsFormat(commands[page], output, cap) == ESP_ERR_INVALID_SIZE);
            assert(output[0] == '\0' && output[cap] == 0x5a);
        }
        assert(sensorarrayPanicDiagnosticsFormat(commands[page], output, needed) == ESP_OK);
    }
    puts("PANIC_PASS maximum_uint_fields_all_truncation_capacities_no_mutation");
}

int main(void)
{
    basicAndReadOnly(); detailsAndBounds(); invalidFrameAndRecursion();
    retentionAndCorruption(); formatterBounds();
    printf("PANIC_DIAGNOSTICS_PASS recordBytes=%zu hardware=NOT_RUN\n", sizeof(s_retainedPanic));
    return 0;
}
