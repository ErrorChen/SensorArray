/* Exercises actual BLE payload/fragment planner around the new checkpoint. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"
#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u
#define SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE 80u
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_TX_FAST 0

typedef enum { SENSORARRAY_BLE_CH_DATA, SENSORARRAY_BLE_CH_LOG, SENSORARRAY_BLE_CH_CTRL } sensorarrayBleChannel_t;
typedef struct { bool congested, confirm; unsigned txMode, mtu, connectionGeneration; } sensorarrayBleSendContext_t;
static struct {
    uint32_t nextMessageId[3];
    uint8_t txPacketScratch[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    struct { uint32_t txConnectionStaleDrop, congestedCount, txCrcMismatch, fragmentError, fragmentSent, tinyTailCount; } stats;
} s_ble;
static int s_bleMux;
static unsigned lockDepth, nowTick, delayCalls, drops, messages, sends, fragments, totalFragments;
static bool slotOwned, connected, disconnectOnDelay, failSend, congested;
static esp_freertos_idle_cb_t idleHook;
static uint8_t source[128], reconstructed[128];
static size_t receivedLength;
#define portENTER_CRITICAL(mux) do { (void)(mux); lockDepth++; } while (0)
#define portEXIT_CRITICAL(mux) do { (void)(mux); assert(lockDepth); lockDepth--; } while (0)
TickType_t xTaskGetTickCount(void) { return nowTick; }
int xPortGetCoreID(void) { return 0; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1u && lockDepth == 0u && slotOwned);
    /* A caller retains payload ownership while the new checkpoint blocks. */
    assert(memcmp(source, "ABCDEFGHIJKLMNOPQRSTUVWXYZ", 26u) == 0);
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
    *out = (sensorarrayBleSendContext_t){.mtu=83u, .connectionGeneration=3u, .congested=congested};
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
    assert(length <= 80u); sends++;
    if (failSend) { return ESP_FAIL; }
    char envelope; unsigned mid, index, count, chunk, messageLength, crc; int header = 0;
    assert(sscanf((char *)data, "G,%c,%u,%u,%u,%u,%u,%x\n%n", &envelope, &mid, &index,
                  &count, &chunk, &messageLength, &crc, &header) == 7);
    assert(envelope == 'D' && mid == 1u && index == fragments && messageLength == sizeof(source));
    assert(crc == sensorarrayBleCrc32(source, sizeof(source)) && (unsigned)header + chunk == length);
    assert(receivedLength + chunk <= sizeof(reconstructed));
    memcpy(reconstructed + receivedLength, data + header, chunk);
    receivedLength += chunk; fragments++; totalFragments = count;
    return ESP_OK;
}
/* sensorarrayBleMaxPayloadSize */
/* sensorarrayBlePlanFragments */
/* sensorarrayBleSendFragment */
/* sensorarrayBleSendPayload */
static void reset(void)
{
    memset(&s_ble, 0, sizeof(s_ble)); memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    lockDepth = nowTick = delayCalls = drops = messages = sends = fragments = totalFragments = 0u;
    receivedLength = 0u;
    slotOwned = connected = true; disconnectOnDelay = failSend = congested = false;
    for (unsigned i = 0u; i < sizeof(source); ++i) { source[i] = (uint8_t)('A' + i % 26u); }
    assert(sensorarrayWorkBudgetInit() == ESP_OK && idleHook);
    assert(idleHook());
    nowTick = 2u; /* Other continuously-ready CPU0 work has kept idle out. */
}
int main(void)
{
    reset();
    assert(sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, sizeof(source),
                                    sensorarrayBleCrc32(source, sizeof(source)), 3u) == ESP_OK);
    assert(slotOwned && fragments == totalFragments && fragments > 1u && messages == 1u && drops == 0u);
    assert(delayCalls == fragments - 2u && receivedLength == sizeof(source));
    assert(memcmp(reconstructed, source, sizeof(source)) == 0);
    slotOwned = false; /* The TX task releases only after SendPayload returned. */

    reset(); disconnectOnDelay = true;
    assert(sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, sizeof(source),
                                    sensorarrayBleCrc32(source, sizeof(source)), 3u) == ESP_ERR_INVALID_STATE);
    assert(slotOwned && sends == 2u && fragments == 2u && delayCalls == 1u && messages == 0u && drops == 1u);
    assert(s_ble.stats.txConnectionStaleDrop == 1u && s_ble.stats.fragmentError == 0u);
    slotOwned = false;

    reset(); failSend = true;
    assert(sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, sizeof(source),
                                    sensorarrayBleCrc32(source, sizeof(source)), 3u) == ESP_FAIL);
    assert(slotOwned && sends == 1u && delayCalls == 0u && messages == 0u && drops == 1u);
    assert(s_ble.stats.fragmentError == 1u);

    reset(); congested = true;
    assert(sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, sizeof(source),
                                    sensorarrayBleCrc32(source, sizeof(source)), 3u) == ESP_ERR_TIMEOUT);
    assert(sends == 0u && drops == 1u && slotOwned);

    reset();
    assert(sensorarrayBleSendPayload(SENSORARRAY_BLE_CH_DATA, source, sizeof(source), 0u, 3u) == ESP_ERR_INVALID_CRC);
    assert(sends == 0u && drops == 1u && s_ble.stats.txCrcMismatch == 1u && slotOwned);
    puts("WORK_BUDGET_BLE_FRAGMENTS_PASS,scenarios=5,production_bodies=5,hardware=NOT_RUN");
    return 0;
}
