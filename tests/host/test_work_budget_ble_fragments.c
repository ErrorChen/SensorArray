/* Actual BLE payload/planner + generated encoder frame; all timing is synthetic. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"
#include "work_budget_frame.h"
#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u
/* BLE_HEADER_CONSTANTS */
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_TX_FAST 0
#define SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT 4u

typedef enum { SENSORARRAY_BLE_CH_DATA, SENSORARRAY_BLE_CH_LOG, SENSORARRAY_BLE_CH_CTRL } sensorarrayBleChannel_t;
typedef struct { bool congested, confirm, priority; unsigned txMode, mtu, connectionGeneration, streamGeneration; } sensorarrayBleSendContext_t;
static struct {
    struct { bool ready, quarantined; uint32_t rejected; } guard;
    uint32_t nextMessageId[3];
    uint8_t txPacketScratch[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    bool ownerCapacityFailure;
    struct { uint32_t txConnectionStaleDrop, congestedCount, txCrcMismatch, fragmentError, fragmentSent, tinyTailCount; } stats;
} s_ble = {.guard = {.ready = true}};
static int s_bleMux;
static unsigned lockDepth, nowTick, delayCalls, drops, messages, sends, fragments, totalFragments;
static unsigned currentMtu, ticksPerSend, disconnectAfterSends;
static unsigned checkpointBeforeSend, fragmentsInBatch, maxFragmentsInBatch;
static bool slotOwned, connected, disconnectOnDelay, failSend, congested;
static esp_freertos_idle_cb_t idleHook;
static uint8_t source[2304], original[2304], reconstructed[2304];
static size_t messageBytes, receivedLength;
#define portENTER_CRITICAL(mux) do { (void)(mux); lockDepth++; } while (0)
#define portEXIT_CRITICAL(mux) do { (void)(mux); assert(lockDepth); lockDepth--; } while (0)
TickType_t xTaskGetTickCount(void) { return nowTick; }
int xPortGetCoreID(void) { return 0; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1u && lockDepth == 0u && slotOwned);
    assert(memcmp(source, original, messageBytes) == 0);
    delayCalls++; nowTick++;
    if (disconnectOnDelay) { connected = false; }
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu)
{ assert(cpu == 0u); idleHook = cb; return ESP_OK; }
static void sensorarrayBleStatsIncrement(uint32_t *counter) { (*counter)++; }
static void sensorarrayBleNoteDrop(sensorarrayBleChannel_t channel) { (void)channel; drops++; }
static void sensorarrayBleStatsNoteMessageSent(sensorarrayBleChannel_t channel, bool unfragmented)
{ (void)channel; assert(!unfragmented); messages++; }
static bool sensorarrayBleSnapshotSendContext(sensorarrayBleChannel_t channel, uint32_t generation,
                                              sensorarrayBleSendContext_t *out)
{
    (void)channel; assert(generation == 3u);
    *out = (sensorarrayBleSendContext_t){.mtu=currentMtu, .connectionGeneration=3u, .congested=congested};
    return connected;
}
static bool sensorarrayBleSendContextMatches(sensorarrayBleChannel_t channel,
                                            const sensorarrayBleSendContext_t *context, bool *outCongested)
{ (void)channel; if (outCongested) { *outCongested = congested; } return connected && context->connectionGeneration == 3u; }
static bool sensorarrayBleSendContextCurrent(sensorarrayBleChannel_t channel, const sensorarrayBleSendContext_t *context)
{ return sensorarrayBleSendContextMatches(channel, context, NULL) && !congested; }
static char sensorarrayBleEnvelopeChannel(sensorarrayBleChannel_t channel) { (void)channel; return 'D'; }
static uint16_t sensorarrayBleValueHandle(sensorarrayBleChannel_t channel) { (void)channel; return 12u; }
static bool sensorarrayBleValidateAscii(sensorarrayBleChannel_t channel, const uint8_t *data,
                                      size_t length, uint32_t message, uint32_t fragment)
{ (void)channel; (void)message; (void)fragment; return data && length > 0u; }
/* sensorarrayBleCrc32 */
static esp_err_t sensorarrayBleSendGattValue(const sensorarrayBleSendContext_t *context,
                                           sensorarrayBleChannel_t channel, uint16_t handle,
                                           uint16_t length, uint8_t *data, bool confirm)
{
    (void)channel;
    assert(context && slotOwned && lockDepth == 0u && handle == 12u && !confirm);
    assert(length <= currentMtu - 3u); sends++;
    unsigned checkpoints = s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints;
    if (checkpointBeforeSend != checkpoints) { fragmentsInBatch = 0u; checkpointBeforeSend = checkpoints; }
    fragmentsInBatch++;
    if (fragmentsInBatch > maxFragmentsInBatch) { maxFragmentsInBatch = fragmentsInBatch; }
    if (failSend) { return ESP_FAIL; }
    char envelope; unsigned mid, index, count, chunk, messageLength, crc; int header = 0;
    assert(sscanf((char *)data, "G,%c,%u,%u,%u,%u,%u,%x\n%n", &envelope, &mid, &index,
                  &count, &chunk, &messageLength, &crc, &header) == 7);
    assert(envelope == 'D' && mid == 1u && index == fragments && messageLength == messageBytes);
    assert(crc == sensorarrayBleCrc32(source, messageBytes) && (unsigned)header + chunk == length);
    assert(receivedLength + chunk <= sizeof(reconstructed));
    memcpy(reconstructed + receivedLength, data + header, chunk);
    receivedLength += chunk; fragments++; totalFragments = count;
    nowTick += ticksPerSend;
    if (disconnectAfterSends && sends >= disconnectAfterSends) { connected = false; }
    return ESP_OK;
}
/* sensorarrayBleDecimalChars */
/* sensorarrayBleMaxPayloadSize */
/* sensorarrayBlePlanFragments */
/* sensorarrayBleTwoFragmentFirstChunk */
/* sensorarrayBleSendFragment */
/* sensorarrayBleSendPayload */
static void reset(bool largeSynthetic, unsigned mtu)
{
    memset(&s_ble, 0, sizeof(s_ble)); s_ble.guard.ready = true; memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    lockDepth = nowTick = delayCalls = drops = messages = sends = fragments = totalFragments = 0u;
    ticksPerSend = disconnectAfterSends = 0u;
    checkpointBeforeSend = fragmentsInBatch = maxFragmentsInBatch = 0u;
    receivedLength = 0u; currentMtu = mtu;
    slotOwned = connected = true; disconnectOnDelay = failSend = congested = false;
    if (largeSynthetic) {
        messageBytes = sizeof(source);
        for (unsigned i = 0u; i < sizeof(source); ++i) { source[i] = (uint8_t)('A' + i % 26u); }
    } else {
        messageBytes = sizeof(referenceFrame);
        assert(messageBytes <= sizeof(source));
        memcpy(source, referenceFrame, messageBytes);
    }
    memcpy(original, source, messageBytes);
    assert(sensorarrayWorkBudgetInit() == ESP_OK && idleHook && idleHook());
    nowTick = 2u;
    /* The actual TX task calls its existing checkpoint before dequeuing. */
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
}
static esp_err_t sendPayload(void)
{
    return sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, (uint16_t)messageBytes,
                                    sensorarrayBleCrc32(source, messageBytes), 3u, false, 0u);
}
static void assertComplete(void)
{
    assert(slotOwned && fragments == totalFragments && fragments > 1u && messages == 1u && drops == 0u);
    assert(receivedLength == messageBytes && memcmp(reconstructed, source, messageBytes) == 0);
    assert(maxFragmentsInBatch <= SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT);
}
int main(void)
{
    reset(false, 247u); assert(sendPayload() == ESP_OK);
    assertComplete(); assert(fragments == 3u);
    assert(delayCalls == 0u); /* Existing descriptor checkpoint covers this fast 3-fragment frame. */
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints == 1u);
    slotOwned = false;

    reset(true, 247u); assert(sendPayload() == ESP_OK); assertComplete();
    assert(fragments > 4u && delayCalls == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints ==
           1u + (fragments - 1u) / SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT);

    reset(true, 64u); assert(sendPayload() == ESP_OK); assertComplete();
    assert(fragments > 32u && delayCalls == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints ==
           1u + (fragments - 1u) / SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT);

    reset(false, 23u); assert(sendPayload() == ESP_ERR_INVALID_SIZE);
    assert(sends == 0u && messages == 0u && drops == 1u && slotOwned);

    reset(false, 247u); ticksPerSend = 1u; assert(sendPayload() == ESP_OK); assertComplete();
    assert(delayCalls > 0u && maxFragmentsInBatch == 1u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints == fragments); /* One-tick elapsed guard. */

    reset(true, 247u); disconnectOnDelay = true; nowTick = 4u; /* Real idle hard backstop. */
    assert(sendPayload() == ESP_ERR_INVALID_STATE);
    assert(slotOwned && sends == 4u && fragments == 4u && delayCalls == 1u && messages == 0u && drops == 1u);
    assert(s_ble.stats.txConnectionStaleDrop == 1u && s_ble.stats.fragmentError == 0u);

    reset(false, 247u); disconnectAfterSends = 1u;
    assert(sendPayload() == ESP_ERR_INVALID_STATE);
    assert(sends == 1u && delayCalls == 0u && messages == 0u && drops == 1u);
    assert(s_ble.stats.txConnectionStaleDrop == 1u); /* Every fragment checks even within a batch. */

    reset(false, 247u); failSend = true; assert(sendPayload() == ESP_FAIL);
    assert(slotOwned && sends == 1u && delayCalls == 0u && messages == 0u && drops == 1u);
    assert(s_ble.stats.fragmentError == 1u);

    reset(false, 247u); congested = true; assert(sendPayload() == ESP_ERR_TIMEOUT);
    assert(sends == 0u && drops == 1u && slotOwned);

    reset(false, 247u);
    assert(sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, (uint16_t)messageBytes, 0u, 3u, false, 0u) == ESP_ERR_INVALID_CRC);
    assert(sends == 0u && drops == 1u && s_ble.stats.txCrcMismatch == 1u && slotOwned);

    reset(true, 247u); nowTick = UINT32_MAX;
    s_idleTick = UINT32_MAX - 2u; ticksPerSend = 1u;
    assert(sendPayload() == ESP_OK); assertComplete();
    assert(delayCalls > 0u && maxFragmentsInBatch == 1u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints == fragments); /* Tick wrap keeps elapsed bound. */
    puts("WORK_BUDGET_BLE_FRAGMENTS_PASS,scenarios=11,production_bodies=7,hardware=NOT_RUN");
    return 0;
}
