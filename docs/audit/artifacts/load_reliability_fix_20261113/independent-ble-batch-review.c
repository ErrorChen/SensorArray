/* Independent synthetic scheduler/GATT boundary model. Function placeholders
 * are replaced by exact production bodies by the companion runner. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"
#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u
#define SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE 80u
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_TX_FAST 0
/* extracted batch configuration */
typedef enum { SENSORARRAY_BLE_CH_DATA, SENSORARRAY_BLE_CH_LOG,
               SENSORARRAY_BLE_CH_CTRL } sensorarrayBleChannel_t;
typedef struct { bool congested, confirm; unsigned txMode, mtu,
                 connectionGeneration; } sensorarrayBleSendContext_t;
static struct {
    uint32_t nextMessageId[3];
    uint8_t txPacketScratch[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    struct { uint32_t txConnectionStaleDrop, congestedCount, txCrcMismatch,
        fragmentError, fragmentSent, tinyTailCount; } stats;
} s_ble;
static int s_bleMux;
static unsigned lockDepth, delayCalls, sends, fragments, fragmentCount,
    generation, disconnectAt, failAt, drops, messages, mtu;
static TickType_t nowTick, sdkTicks;
static bool slotOwned, congested;
static size_t sourceLength, reconstructedLength;
static uint8_t source[2304], reconstructed[2304];
static esp_freertos_idle_cb_t idleHook;
#define portENTER_CRITICAL(m) do { (void)(m); lockDepth++; } while (0)
#define portEXIT_CRITICAL(m) do { (void)(m); assert(lockDepth); lockDepth--; } while (0)
TickType_t xTaskGetTickCount(void) { return nowTick; }
int xPortGetCoreID(void) { return 0; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1u && slotOwned && lockDepth == 0u);
    delayCalls++;
    nowTick += ticks;
    /* Deliberately do not invent an idle observation after our own delay. */
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t hook, UBaseType_t cpu)
{ assert(cpu == 0u); idleHook = hook; return ESP_OK; }
static void sensorarrayBleStatsIncrement(uint32_t *value) { (*value)++; }
static void sensorarrayBleNoteDrop(sensorarrayBleChannel_t channel)
{ (void)channel; drops++; }
static void sensorarrayBleStatsNoteMessageSent(sensorarrayBleChannel_t channel, bool plain)
{ (void)channel; assert(!plain); messages++; }
static bool sensorarrayBleSnapshotSendContext(sensorarrayBleChannel_t channel,
        uint32_t expected, sensorarrayBleSendContext_t *out)
{
    (void)channel;
    if (expected != generation) { return false; }
    *out = (sensorarrayBleSendContext_t){.mtu=mtu,
        .connectionGeneration=generation, .congested=congested};
    return true;
}
static bool sensorarrayBleSendContextMatches(sensorarrayBleChannel_t channel,
        const sensorarrayBleSendContext_t *context, bool *outCongested)
{
    (void)channel;
    if (outCongested) { *outCongested = congested; }
    return context->connectionGeneration == generation;
}
static bool sensorarrayBleSendContextCurrent(sensorarrayBleChannel_t channel,
        const sensorarrayBleSendContext_t *context)
{ return sensorarrayBleSendContextMatches(channel, context, NULL) && !congested; }
static char sensorarrayBleEnvelopeChannel(sensorarrayBleChannel_t channel)
{ (void)channel; return 'D'; }
static uint16_t sensorarrayBleValueHandle(sensorarrayBleChannel_t channel)
{ (void)channel; return 12u; }
static bool sensorarrayBleValidateAscii(sensorarrayBleChannel_t channel,
        const uint8_t *data, size_t length, uint32_t message, uint32_t fragment)
{ (void)channel; (void)message; (void)fragment; return data && length > 0u; }
/* sensorarrayBleCrc32 */
static esp_err_t sensorarrayBleSendGattValue(const sensorarrayBleSendContext_t *context,
        sensorarrayBleChannel_t channel, uint16_t handle, uint16_t length,
        uint8_t *data, bool confirm)
{
    (void)channel;
    assert(context && slotOwned && lockDepth == 0u && handle == 12u && !confirm);
    assert(length <= mtu - 3u);
    sends++;
    nowTick += sdkTicks;
    if (failAt && sends == failAt) { return ESP_FAIL; }
    char env;
    unsigned mid, index, count, chunk, whole, crc;
    int header = 0;
    assert(sscanf((char *)data, "G,%c,%u,%u,%u,%u,%u,%x\n%n", &env, &mid,
        &index, &count, &chunk, &whole, &crc, &header) == 7);
    assert(env == 'D' && mid == 1u && index == fragments && whole == sourceLength);
    assert(crc == sensorarrayBleCrc32(source, sourceLength));
    assert((unsigned)header + chunk == length && reconstructedLength + chunk <= sourceLength);
    memcpy(reconstructed + reconstructedLength, data + header, chunk);
    reconstructedLength += chunk;
    fragments++;
    fragmentCount = count;
    if (disconnectAt && sends == disconnectAt) { generation++; }
    return ESP_OK;
}
/* sensorarrayBleMaxPayloadSize */
/* sensorarrayBlePlanFragments */
/* sensorarrayBleSendFragment */
/* sensorarrayBleSendPayload */

static void reset(size_t length, unsigned packetMtu, TickType_t start, TickType_t cost)
{
    memset(&s_ble, 0, sizeof(s_ble));
    memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    lockDepth = delayCalls = sends = fragments = fragmentCount = drops = messages = 0u;
    disconnectAt = failAt = 0u;
    generation = 3u;
    slotOwned = true;
    congested = false;
    mtu = packetMtu;
    sourceLength = length;
    reconstructedLength = 0u;
    nowTick = start;
    sdkTicks = cost;
    for (size_t i = 0u; i < sizeof(source); i++) { source[i] = 'A' + i % 26u; }
    assert(sensorarrayWorkBudgetInit() == ESP_OK && idleHook);
    nowTick += 2u; /* Initialization itself has no earlier idle history. */
    /* No idle has run: both old and new policies must really block at every
     * checkpoint, without a fake health task or synthetic idle credit. */
}

static esp_err_t send(void)
{
    /* The unchanged actual TX loop applies this before each descriptor. */
    (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    return sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source,
        (uint16_t)sourceLength, sensorarrayBleCrc32(source, sourceLength), 3u);
}

static void complete(const char *name, unsigned expectedCheckpoints)
{
    assert(messages == 1u && drops == 0u && reconstructedLength == sourceLength);
    assert(memcmp(source, reconstructed, sourceLength) == 0 && slotOwned && lockDepth == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints == expectedCheckpoints);
    assert(delayCalls == expectedCheckpoints && s_idleGeneration == 0u);
    printf("BLE_BATCH_REVIEW,case=%s,bytes=%zu,mtu=%u,fragments=%u,checkpoints=%u,delays=%u,wireCrcPreserved=1,slotRetained=1\n",
           name, sourceLength, mtu, fragments, expectedCheckpoints, delayCalls);
    slotOwned = false; /* Caller releases only after the actual function returns. */
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    reset(600u, 247u, 10u, 0u);
    assert(send() == ESP_OK && fragments == 3u);
#ifdef AUDIT_BASELINE
    complete("typical600_baseline", 3u);
#else
    complete("typical600", 1u);
#endif
    reset(2304u, 247u, 10u, 0u);
    assert(send() == ESP_OK);
#ifdef AUDIT_BASELINE
    complete("maximum_baseline", fragments);
#else
    complete("maximum", 1u + (fragments - 1u) / 4u);
#endif
    reset(600u, 247u, 10u, 1u);
    assert(send() == ESP_OK);
    complete("sdk_crosses_tick", fragments);
    reset(600u, 247u, UINT32_MAX - 4u, 1u);
    assert(send() == ESP_OK);
    complete("tick_wrap", fragments);
    for (unsigned boundary = 1u; boundary <= 4u; boundary++) {
        reset(2304u, 247u, 10u, 0u);
        disconnectAt = boundary;
        assert(send() == ESP_ERR_INVALID_STATE);
        assert(fragments == boundary && sends == boundary && messages == 0u && drops == 1u);
        assert(s_ble.stats.txConnectionStaleDrop == 1u && s_ble.stats.fragmentError == 0u && slotOwned);
        printf("BLE_BATCH_REVIEW,case=disconnect_after_%u,nextFragmentBlocked=1,slotRetained=1\n", boundary);
    }
    reset(2304u, 247u, 10u, 0u);
    failAt = 5u;
    assert(send() == ESP_FAIL && fragments == 4u && sends == 5u);
    assert(messages == 0u && drops == 1u && s_ble.stats.fragmentError == 1u && slotOwned);
    puts("BLE_BATCH_REVIEW,case=failure_after_batch,firstErrorPreserved=1,slotRetained=1");
    return 0;
}
