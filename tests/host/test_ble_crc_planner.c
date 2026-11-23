/* Synthetic hardware boundary, actual production functions, frozen old oracle.
 * Host execution proves byte/branch equivalence, not target ROM performance. */
#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u
#define SENSORARRAY_BLE_TX_FAST 0u
/* TINY_TAIL_CONSTANT */
/* HEADER_CONSTANTS */
enum { SENSORARRAY_BLE_CH_DATA, SENSORARRAY_BLE_CH_LOG, SENSORARRAY_BLE_CH_CTRL };
typedef unsigned sensorarrayBleChannel_t;
typedef int esp_err_t;
enum { ESP_OK, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_SIZE, ESP_ERR_INVALID_ARG,
       ESP_ERR_TIMEOUT };
typedef struct { bool priority; uint16_t mtu; unsigned txMode; } sensorarrayBleSendContext_t;
static struct {
    struct { bool ready, quarantined; uint32_t rejected; } guard;
    struct { uint64_t tinyTailCount, txConnectionStaleDrop, fragmentError, fragmentSent; } stats;
    uint8_t txPacketScratch[512];
    bool ownerCapacityFailure;
} s_ble = {.guard = {.ready = true}};
static int s_bleMux;
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
static void sensorarrayBleStatsIncrement(uint64_t *p) { ++*p; }
static bool current = true, ascii = true;
static esp_err_t sdkResult;
static uint8_t captured[512];
static size_t capturedLength;
static bool sensorarrayBleSendContextMatches(sensorarrayBleChannel_t ch,
    const sensorarrayBleSendContext_t *ctx, void *unused)
{ (void)ch; (void)ctx; (void)unused; return current; }
static bool sensorarrayBleSendContextCurrent(sensorarrayBleChannel_t ch,
    const sensorarrayBleSendContext_t *ctx)
{ return sensorarrayBleSendContextMatches(ch, ctx, NULL); }
static uint16_t sensorarrayBleValueHandle(sensorarrayBleChannel_t ch) { return (uint16_t)ch; }
static bool sensorarrayBleValidateAscii(sensorarrayBleChannel_t ch,
    const uint8_t *data, size_t n, uint32_t mid, uint32_t idx)
{ (void)ch; (void)data; (void)n; (void)mid; (void)idx; return ascii; }
static esp_err_t sensorarrayBleSendGattValue(const sensorarrayBleSendContext_t *ctx,
    sensorarrayBleChannel_t ch, uint16_t handle, uint16_t n, uint8_t *data, bool confirm)
{
    (void)ctx; (void)ch; (void)handle; (void)confirm;
    assert(n <= sizeof captured); capturedLength = n; memcpy(captured, data, n);
    return sdkResult;
}
/* SDK_REFERENCE */
static uint64_t romCalls;
static uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *data, uint32_t n)
{ ++romCalls; return sdk_crc32_le(crc, data, n); }
/* BASELINE_FUNCTIONS */
/* PORTABLE_FUNCTIONS */
#define ESP_PLATFORM 1
/* ROM_FUNCTIONS */
#undef ESP_PLATFORM
/* PLANNER_FUNCTIONS */
/* WIRE_FUNCTION */
/* CHUNK_SELECTION */
/* OLD_HEADER_FORMAT */

static uint64_t crcCases, headerCases, planCases, wireCases;
static uint32_t largestCount;
static uint64_t tinyTailPlans;
static uint8_t bytes[2304];

static void testCrc(void)
{
    assert(old_sensorarrayBleCrc32((const uint8_t *)"123456789", 9) == UINT32_C(0xCBF43926));
    assert(sensorarrayBleCrc32(NULL, 0) == 0 && sensorarrayTextCrc32(NULL, 0) == 0);
    for (unsigned pattern = 0; pattern < 5; ++pattern) {
        for (size_t i = 0; i < sizeof bytes; ++i)
            bytes[i] = pattern == 0 ? 0 : pattern == 1 ? 255 :
                pattern == 2 ? (uint8_t)i : pattern == 3 ? (uint8_t)(i * 131u + 17u) :
                (uint8_t)(32u + i % 95u);
        for (size_t n = 0; n <= sizeof bytes; ++n) {
            const uint32_t expected = old_sensorarrayBleCrc32(bytes, n);
            assert(old_sensorarrayTextCrc32(bytes, n) == expected);
            assert(portable_sensorarrayBleCrc32(bytes, n) == expected);
            assert(portable_sensorarrayTextCrc32(bytes, n) == expected);
            const uint64_t before = romCalls;
            assert(sensorarrayBleCrc32(bytes, n) == expected);
            assert(sensorarrayTextCrc32(bytes, n) == expected);
            assert(romCalls - before == (n == 0 ? 0u : 2u));
            size_t cut = n / 2u;
            assert(esp_rom_crc32_le(esp_rom_crc32_le(0u, bytes, (uint32_t)cut),
                                    bytes + cut, (uint32_t)(n - cut)) == expected);
            if (n) {
                bytes[n / 2u] ^= 1u;
                assert(sensorarrayBleCrc32(bytes, n) != expected);
                bytes[n / 2u] ^= 1u;
            }
            ++crcCases;
        }
    }
    /* Execute the exact production size-selection statement without claiming
     * to have a >4 GiB valid data buffer or invoking undefined pointer math. */
    assert(bleChunk(0u) == 0u && textChunk(0u) == 0u);
    assert(bleChunk(UINT32_MAX) == UINT32_MAX);
    assert(textChunk(UINT32_MAX - 1u) == UINT32_MAX - 1u);
#if SIZE_MAX > UINT32_MAX
    assert(bleChunk((size_t)UINT32_MAX + 1u) == UINT32_MAX);
    assert(textChunk(SIZE_MAX) == UINT32_MAX);
#endif
}

static void checkHeader(char ch, uint32_t mid, uint32_t idx, uint32_t count,
                        size_t chunk, size_t length)
{
    char old[80];
    int n = snprintf(old, sizeof old, oldHeaderFormat, ch, (unsigned long)mid,
                     (unsigned long)idx, (unsigned long)count,
                     (unsigned)chunk, (unsigned)length, (unsigned long)UINT32_MAX);
    assert(n > 0 && (size_t)n < sizeof old);
    const size_t maximum[] = {0u, 20u, (size_t)n - 1u, (size_t)n,
                              (size_t)n + 1u, 244u, 512u};
    for (size_t i = 0; i < sizeof maximum / sizeof *maximum; ++i) {
        assert(sensorarrayBleMaxPayloadSize(maximum[i], ch, mid, idx, count,
                                            chunk, length, UINT32_MAX) ==
               old_sensorarrayBleMaxPayloadSize(maximum[i], ch, mid, idx, count,
                                                chunk, length, UINT32_MAX));
        ++headerCases;
    }
}

static void testHeaders(void)
{
    const uint32_t edges[] = {0u, 1u, 9u, 10u, 99u, 100u, 999u, 1000u, 9999u,
        10000u, 99999u, 100000u, 999999u, 1000000u, 9999999u, 10000000u,
        99999999u, 100000000u, 999999999u, 1000000000u, UINT32_MAX};
    const char channels[] = "CDL";
    for (size_t c = 0; c < 3; ++c)
        for (size_t i = 0; i < sizeof edges / sizeof *edges; ++i)
            for (size_t j = 0; j < sizeof edges / sizeof *edges; ++j) {
                /* Exercise each independently formatted field across every
                 * decimal-width transition and its maximum, plus all-wide. */
                checkHeader(channels[c], edges[i], edges[j], edges[i], 2304, 2304);
                checkHeader(channels[c], edges[i], edges[i], edges[j], edges[i], edges[j]);
            }
#if SIZE_MAX > UINT32_MAX
    /* Out of representable wire domain is explicitly rejected, not truncated.
     * Actual enqueue inputs are capped at 2304, before the planner is called. */
    assert(sensorarrayBleMaxPayloadSize(512, 'D', 1, 0, 1,
             (size_t)UINT32_MAX + 1u, 1, 0) == 0);
    assert(sensorarrayBleMaxPayloadSize(512, 'D', 1, 0, 1,
             1, SIZE_MAX, 0) == 0);
#endif
}

static void checkPlan(size_t length, size_t maximum, char channel, uint32_t mid)
{
    uint32_t a = UINT32_MAX, b = UINT32_MAX;
    s_ble.stats.tinyTailCount = 0;
    bool old = old_sensorarrayBlePlanFragments(length, maximum, channel, mid, 0, &a);
    uint64_t tiny = s_ble.stats.tinyTailCount;
    s_ble.stats.tinyTailCount = 0;
    bool now = sensorarrayBlePlanFragments(length, maximum, channel, mid, 0, &b);
    assert(now == old && a == b && tiny == s_ble.stats.tinyTailCount);
    if (now && b > largestCount) largestCount = b;
    tinyTailPlans += tiny;
    ++planCases;
}

static void testPlansAndWire(void)
{
    const uint32_t mids[] = {0, 1, 9, 10, 99, 100, 999, 1000, 999999999,
                            1000000000, UINT32_MAX};
    const size_t lengths[] = {0, 1, 9, 10, 19, 20, 21, 99, 100, 243, 244, 245,
        511, 512, 513, 598, 999, 1000, 2303, 2304};
    const uint16_t mtus[] = {23, 247, 517};
    const char channels[] = "DLC";
    for (size_t n = 0; n <= sizeof bytes; ++n) {
        checkPlan(n, 244, 'D', UINT32_MAX);
        checkPlan(n, 512, 'D', UINT32_MAX);
    }
    const size_t narrow[] = {29, 30, 31, 32, 33, 34, 35, 40, 41, 42, 43, 44};
    for (size_t m = 0; m < sizeof narrow / sizeof *narrow; ++m)
        for (size_t l = 0; l < sizeof lengths / sizeof *lengths; ++l) {
            checkPlan(lengths[l], narrow[m], 'D', 1);
            checkPlan(lengths[l], narrow[m], 'C', UINT32_MAX);
        }
    /* Four-digit fragment counts cannot be forced by these <=2304-byte
     * accepted inputs: the shorter three-digit plan can fit first. Exercise
     * the internal planner's representable (but enqueue-rejected) 4000-byte
     * input without reading a payload or claiming it is an accepted packet. */
    checkPlan(4000, 33, 'D', 1);
    assert(largestCount >= 1000 && tinyTailPlans > 0);
    memset(bytes, 'A', sizeof bytes);
    for (size_t c = 0; c < 3; ++c)
        for (size_t m = 0; m < sizeof mids / sizeof *mids; ++m)
            for (size_t t = 0; t < 3; ++t)
                for (size_t l = 0; l < sizeof lengths / sizeof *lengths; ++l) {
                    size_t n = lengths[l], maximum = mtus[t] - 3u;
                    if (maximum > 512) maximum = 512;
                    checkPlan(n, maximum, channels[c], mids[m]);
                    uint32_t count = 0;
                    if (!sensorarrayBlePlanFragments(n, maximum, channels[c], mids[m], 0, &count)) continue;
                    sensorarrayBleSendContext_t ctx = {.mtu = mtus[t]};
                    size_t offset = 0, base = n / count, extra = n % count;
                    uint32_t crc = old_sensorarrayBleCrc32(bytes, n);
                    for (uint32_t idx = 0; idx < count; ++idx) {
                        size_t chunk = base + (idx < extra ? 1u : 0u);
                        uint8_t expected[512];
                        int h = snprintf((char *)expected, sizeof expected, oldHeaderFormat,
                            channels[c], (unsigned long)mids[m], (unsigned long)idx,
                            (unsigned long)count, (unsigned)chunk, (unsigned)n, (unsigned long)crc);
                        assert(h > 0 && (size_t)h + chunk <= maximum);
                        memcpy(expected + h, bytes + offset, chunk);
                        assert(sensorarrayBleSendFragment(&ctx, (unsigned)c, channels[c], mids[m],
                            idx, count, bytes + offset, chunk, n, crc, false) == ESP_OK);
                        assert(capturedLength == (size_t)h + chunk);
                        assert(memcmp(captured, expected, capturedLength) == 0);
                        offset += chunk; ++wireCases;
                    }
                    assert(offset == n);
                }
    assert(!sensorarrayBlePlanFragments(1, 244, 'D', 1, 0, NULL));
    assert(!sensorarrayBlePlanFragments(1, 0, 'D', 1, 0, &(uint32_t){0}));
    sensorarrayBleSendContext_t ctx = {.mtu = 247};
    current = false;
    assert(sensorarrayBleSendFragment(&ctx, 0, 'D', 1, 0, 1, bytes, 1, 1, 0, false) == ESP_ERR_INVALID_STATE);
    current = true; ascii = false;
    assert(sensorarrayBleSendFragment(&ctx, 0, 'D', 1, 0, 1, bytes, 1, 1, 0, false) == ESP_ERR_INVALID_ARG);
    ascii = true; sdkResult = ESP_ERR_TIMEOUT;
    assert(sensorarrayBleSendFragment(&ctx, 0, 'D', 1, 0, 1, bytes, 1, 1, 0, false) == ESP_ERR_TIMEOUT);
    sdkResult = ESP_OK;
    assert(sensorarrayBleSendFragment(&ctx, 0, 'D', 1, 0, 1, bytes, 244, 244, 0, false) == ESP_ERR_INVALID_SIZE);
}

int main(void)
{
    testCrc(); testHeaders(); testPlansAndWire();
    printf("BLE_CRC_PLANNER_EQUIVALENCE,crc=%llu,header=%llu,plan=%llu,wire=%llu,largestCount=%u,tinyTailPlans=%llu,ASAN_UBSAN=1,targetROM=NOT_RUN\n",
        (unsigned long long)crcCases, (unsigned long long)headerCases,
        (unsigned long long)planCases, (unsigned long long)wireCases,
        (unsigned)largestCount, (unsigned long long)tinyTailPlans);
    return 0;
}
