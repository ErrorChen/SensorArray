#include "sensorarraySerialIo.h"
#include "sensorarrayUsbOtg.h"
#include "sensorarrayTransportFrameMeta.h"
#include "sensorarrayTransportWifiControl.h"
#include "sensorarrayTransportInternal.h"

#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sensorarrayBle.h"
#include "sensorarrayScanConfig.h"
#include "sensorarrayTransportGuaranteedText.h"
#include "sensorarrayTransportPool.h"
#include "sensorarrayWifi.h"
#include "sensorarrayWorkBudget.h"

#ifndef CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT
#define CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT 0
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT
#define CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT 1
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD ""
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_TASK_CORE
#define CONFIG_SENSORARRAY_OUTPUT_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_COMM_TASK_CORE
#define CONFIG_SENSORARRAY_COMM_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_SERIAL_CTRL_TASK_STACK
#define CONFIG_SENSORARRAY_SERIAL_CTRL_TASK_STACK 6144
#endif
#ifndef CONFIG_SENSORARRAY_TRANSPORT_TASK_STACK
#define CONFIG_SENSORARRAY_TRANSPORT_TASK_STACK 6144
#endif
#ifndef CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_SHORT
#define CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_SHORT 0
#endif
#ifndef CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_FULL
#define CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_FULL 0
#endif

#define SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT 2u
#define SENSORARRAY_TRANSPORT_LOG_QUEUE_COUNT 2u
#define SENSORARRAY_TRANSPORT_LIFECYCLE_QUEUE_COUNT 1u
#define SENSORARRAY_TRANSPORT_PROTECTED_QUEUE_COUNT 2u
#define SENSORARRAY_TRANSPORT_PROTECTED_DEADLINE_US UINT64_C(1000000)
#define SENSORARRAY_TRANSPORT_PROTECTED_RETRY_US UINT64_C(10000)
_Static_assert(SENSORARRAY_TRANSPORT_PROTECTED_QUEUE_COUNT >= SENSORARRAY_TRANSPORT_POOL_PROTECTED_SLOT_COUNT,
               "protected queue must fit every reserved slot during owner restoration");
#define SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT 8u
#define SENSORARRAY_TRANSPORT_LEGACY_ITEM_BYTES 1560u
#define SENSORARRAY_TRANSPORT_LEGACY_QUEUE_STORAGE_BYTES \
    (3u * SENSORARRAY_TRANSPORT_LEGACY_ITEM_BYTES)
#define SENSORARRAY_TRANSPORT_SERIAL_COMMAND_MAX 96u
#define SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX 32u
#define SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX 48u
#define SENSORARRAY_CFG_OUTPUT_WIFI_TEXT (CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT != 0)
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT (CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT != 0)

_Static_assert(SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT >=
                   SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT +
                   SENSORARRAY_TRANSPORT_LOG_QUEUE_COUNT,
               "Transport pool must cover the combined descriptor queues");
_Static_assert(SENSORARRAY_TRANSPORT_TEXT_MAX ==
                   SENSORARRAY_TRANSPORT_POOL_TEXT_MAX,
               "Transport public and pool payload limits diverged");
_Static_assert(SENSORARRAY_TRANSPORT_TEXT_MAX ==
                   SENSORARRAY_BLE_MESSAGE_VALUE_MAX,
               "Transport and BLE maximum text sizes diverged");
_Static_assert(SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX <=
                   SENSORARRAY_TRANSPORT_TEXT_MAX,
               "CTRL replies must fit inside the transport text slot");

static StaticQueue_t s_dataQueueStruct;
static StaticQueue_t s_logQueueStruct;
static StaticQueue_t s_lifecycleQueueStruct;
static StaticQueue_t s_protectedQueueStruct;
static uint8_t s_dataQueueStorage[SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT *
                                  sizeof(sensorarrayTransportDescriptor_t)];
static uint8_t s_logQueueStorage[SENSORARRAY_TRANSPORT_LOG_QUEUE_COUNT *
                                 sizeof(sensorarrayTransportDescriptor_t)];
static uint8_t s_lifecycleQueueStorage[SENSORARRAY_TRANSPORT_LIFECYCLE_QUEUE_COUNT *
                                       sizeof(sensorarrayTransportDescriptor_t)];
static uint8_t s_protectedQueueStorage[SENSORARRAY_TRANSPORT_PROTECTED_QUEUE_COUNT *
                                       sizeof(sensorarrayTransportDescriptor_t)];
static QueueHandle_t s_dataQueue;
static QueueHandle_t s_logQueue;
static QueueHandle_t s_lifecycleQueue;
static QueueHandle_t s_protectedQueue;
static sensorarrayTransportPool_t s_pool;
/* One allocation for the entire boot, no internal-RAM fallback. State is
 * claimed/published under the existing stats mux; allocation stays outside. */
static uint8_t s_payloadStorageState; /* 0 absent, 1 claiming, 2 ready, 3 failed */
static esp_err_t s_payloadStorageError = ESP_ERR_INVALID_STATE;
static bool s_initInProgress;
static esp_err_t s_completedInitError = ESP_ERR_INVALID_STATE;
static sensorarrayTransportProtectionStats_t s_transportProtectionStats;
/* Admission obligation belongs to this layer. A successful queue admission is
 * not a BLE completion and does not retire BLE's independent obligation. */
static bool s_protectedAdmissionKnown;
static sensorarrayTransportFrameIdentity_t s_protectedAdmissionIdentity;
static uint64_t s_lastProtectedAdmissionUs;
/* Only the transport task touches retry/last-admitted state. An owned retry
 * descriptor remains in the permanent pool; producers cannot reclaim it. */
static bool s_protectedRetryActive;
static sensorarrayTransportDescriptor_t s_protectedRetry;
static bool s_lastBleFrameKnown;
static uint32_t s_lastBleConnectionGeneration;
static sensorarrayTransportFrameIdentity_t s_lastBleFrame;
static TaskHandle_t s_task;
static TaskHandle_t s_serialTask;
static bool s_started;
static char s_deviceName[32];
static char s_mdnsName[32];
static bool s_serialCmdErrPrinted;
static uint32_t s_serialCmdErrSuppressed;
static const char *s_serialCmdErrLastReason = "none";

static void sensorarrayTransportLogSerialCommandError(const uint8_t *data,
                                                      size_t length,
                                                      const char *reason)
{
    if (!data || length == 0u) {
        return;
    }
    s_serialCmdErrLastReason = reason ? reason : "unknown";
    if (s_serialCmdErrPrinted) {
        s_serialCmdErrSuppressed++;
        if ((s_serialCmdErrSuppressed % 32u) == 0u) {
            printf("CMDERR_SUM,src=serial,count=%lu,lastReason=%s\n",
                   (unsigned long)s_serialCmdErrSuppressed,
                   s_serialCmdErrLastReason);
        }
        return;
    }
    s_serialCmdErrPrinted = true;
    char hex[(SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX * 2u) + 1u];
    char ascii[SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX + 1u];
    size_t hexLen = length < SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX ?
        length : SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX;
    for (size_t i = 0u; i < hexLen; ++i) {
        (void)snprintf(&hex[i * 2u], 3u, "%02X", data[i]);
    }
    hex[hexLen * 2u] = '\0';
    size_t asciiLen = length < SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX ?
        length : SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX;
    for (size_t i = 0u; i < asciiLen; ++i) {
        uint8_t ch = data[i];
        ascii[i] = (ch >= 0x20u && ch <= 0x7Eu) ? (char)ch : '.';
    }
    ascii[asciiLen] = '\0';
    printf("CMDERR,src=serial,len=%lu,hex=%s,ascii=%s,reason=%s,trunc=%u\n",
           (unsigned long)length,
           hex,
           ascii,
           s_serialCmdErrLastReason,
           length > hexLen ? 1u : 0u);
}

static esp_err_t sensorarrayTransportNormaliseSerialCommand(const char *input,
                                                            size_t inputLength,
                                                            char *out,
                                                            size_t outSize,
                                                            const char **outReason)
{
    if (!input || !out || outSize == 0u) {
        if (outReason) {
            *outReason = "invalid_arg";
        }
        return ESP_ERR_INVALID_ARG;
    }
    size_t write = 0u;
    for (size_t read = 0u; read < inputLength; ++read) {
        unsigned char ch = (unsigned char)input[read];
        if (ch == '\0') {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            break;
        }
        if (ch == 0x1Bu) {
            read++;
            if (read < inputLength && input[read] == '[') {
                while (read + 1u < inputLength) {
                    read++;
                    unsigned char seq = (unsigned char)input[read];
                    if (seq >= 0x40u && seq <= 0x7Eu) {
                        break;
                    }
                }
            }
            continue;
        }
        if (iscntrl(ch) && !isspace(ch)) {
            if (outReason) {
                *outReason = "control";
            }
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (write + 1u >= outSize) {
            if (outReason) {
                *outReason = "too_long";
            }
            return ESP_ERR_INVALID_SIZE;
        }
        out[write++] = (char)ch;
    }
    while (write > 0u && isspace((unsigned char)out[write - 1u])) {
        write--;
    }
    out[write] = '\0';
    char *start = out;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != out) {
        memmove(out, start, strlen(start) + 1u);
        write = strlen(out);
    }
    if (write == 0u) {
        if (outReason) {
            *outReason = "empty";
        }
        return ESP_ERR_NOT_FOUND;
    }
    if (outReason) {
        *outReason = "ok";
    }
    return ESP_OK;
}

static bool sensorarrayTransportFindFieldU32(const char *line,
                                             const char *key,
                                             uint32_t *outValue)
{
    if (!line || !key || !outValue) {
        return false;
    }
    const char *match = strstr(line, key);
    while (match && match != line && match[-1] != ',') {
        match = strstr(match + 1, key);
    }
    if (!match) {
        return false;
    }
    match += strlen(key);
    char *end = NULL;
    unsigned long parsed = strtoul(match, &end, 10);
    if (end == match) {
        return false;
    }
    *outValue = (uint32_t)parsed;
    return true;
}

static void sensorarrayTransportSyncPoolStatsLocked(void)
{
    g_sensorarrayTransportStats.transportSlotUsed = s_pool.stats.used;
    g_sensorarrayTransportStats.transportSlotHighWater = s_pool.stats.highWater;
    g_sensorarrayTransportStats.transportSlotAllocFail = s_pool.stats.allocFail;
    g_sensorarrayTransportStats.transportSlotReleaseMismatch =
        s_pool.stats.releaseMismatch;
    g_sensorarrayTransportStats.transportStaleDescriptor =
        s_pool.stats.staleDescriptor;
}

static bool sensorarrayTransportAllocateDescriptor(
    sensorarrayTransportChannel_t channel,
    size_t length,
    sensorarrayTransportDescriptor_t *outDescriptor)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    bool allocated = sensorarrayTransportPoolAllocate(&s_pool,
                                                       channel,
                                                       length,
                                                       outDescriptor);
    sensorarrayTransportSyncPoolStatsLocked();
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    return allocated;
}

static bool sensorarrayTransportDescriptorCurrent(
    const sensorarrayTransportDescriptor_t *descriptor)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    bool current = sensorarrayTransportPoolValidate(&s_pool, descriptor);
    sensorarrayTransportSyncPoolStatsLocked();
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    return current;
}

static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(
    const sensorarrayTransportDescriptor_t *descriptor)
{
    return sensorarrayTransportPoolGetSlot(&s_pool, descriptor);
}

static bool sensorarrayTransportReleaseDescriptor(
    const sensorarrayTransportDescriptor_t *descriptor)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    bool released = sensorarrayTransportPoolRelease(&s_pool, descriptor);
    sensorarrayTransportSyncPoolStatsLocked();
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    return released;
}

static void sensorarrayTransportFillFrameMeta(sensorarrayTransportPayloadSlot_t *item)
{
    sensorarrayTransportParseFrameMeta(item);
}

static void sensorarrayTransportProtectionCount(uint32_t *counter)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    (*counter)++;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

void sensorarrayTransportGetProtectionStats(sensorarrayTransportProtectionStats_t *outStats)
{
    if (!outStats) { return; }
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    *outStats = s_transportProtectionStats;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

static bool sensorarrayTransportHoldProtectedRetry(
    const sensorarrayTransportDescriptor_t *descriptor,
    sensorarrayTransportPayloadSlot_t *slot, uint64_t nowUs)
{
    if (nowUs >= slot->protectedDeadlineUs) { return false; }
    if (s_protectedRetryActive) {
        sensorarrayTransportPayloadSlot_t *old = sensorarrayTransportDescriptorSlot(&s_protectedRetry);
        /* A periodic sample cannot replace an unfinished first-configuration
         * sample of the same configuration. A new configuration supersedes
         * an old pending obligation explicitly, never a BLE-owned buffer. */
        if (old && (old->protectionFlags & SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG) &&
            sensorarrayTransportFrameSameConfig(&old->frameIdentity, &slot->frameIdentity)) {
            return false;
        }
        (void)sensorarrayTransportReleaseDescriptor(&s_protectedRetry);
        sensorarrayTransportProtectionCount(&s_transportProtectionStats.superseded);
    }
    slot->protectedNextAttemptUs = nowUs + SENSORARRAY_TRANSPORT_PROTECTED_RETRY_US;
    s_protectedRetry = *descriptor;
    s_protectedRetryActive = true;
    sensorarrayTransportProtectionCount(&s_transportProtectionStats.retryAttempts);
    return true;
}

static TickType_t sensorarrayTransportWaitTicks(void)
{
    TickType_t wifiPoll = sensorarrayWifiOwnerNeedsPoll() ?
        (TickType_t)((SENSORARRAY_WIFI_OBSERVER_POLL_US * configTICK_RATE_HZ + UINT64_C(999999)) / UINT64_C(1000000)) : portMAX_DELAY;
    if (wifiPoll == 0u) { wifiPoll = 1u; }
    if (!s_protectedRetryActive) { return wifiPoll; }
    sensorarrayTransportPayloadSlot_t *slot = sensorarrayTransportDescriptorSlot(&s_protectedRetry);
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    uint64_t wakeUs = slot->protectedNextAttemptUs < slot->protectedDeadlineUs ?
        slot->protectedNextAttemptUs : slot->protectedDeadlineUs;
    if (wakeUs <= nowUs) { return 0u; }
    uint64_t ticks = ((wakeUs - nowUs) * configTICK_RATE_HZ + UINT64_C(999999)) / UINT64_C(1000000);
    TickType_t protectedPoll = (TickType_t)(ticks != 0u ? ticks : 1u);
    return protectedPoll < wifiPoll ? protectedPoll : wifiPoll;
}

static void sensorarrayTransportBuildNames(const uint8_t mac[6])
{
    snprintf(s_deviceName, sizeof(s_deviceName), "CscArray_%02X%02X%02X",
             mac[3], mac[4], mac[5]);
    snprintf(s_mdnsName, sizeof(s_mdnsName), "cscarray-%02x%02x%02x.local",
             mac[3], mac[4], mac[5]);
}

static void sensorarrayTransportBuildWifiConfig(sensorarrayWifiConfig_t *wifiConfig)
{
    if (!wifiConfig) {
        return;
    }
    *wifiConfig = (sensorarrayWifiConfig_t){
        .profile = SENSORARRAY_WIFI_PROFILE_HIGH,
        .dataPort = 3333u,
        .logPort = 3334u,
        .ctrlPort = 3335u,
    };
    snprintf(wifiConfig->ssid, sizeof(wifiConfig->ssid), "%s", s_deviceName);
    snprintf(wifiConfig->password, sizeof(wifiConfig->password), "%s",
             CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD);
}

static esp_err_t sensorarrayTransportStartWifiAp(uint32_t *requestId)
{
    sensorarrayWifiConfig_t wifiConfig;
    sensorarrayTransportBuildWifiConfig(&wifiConfig);
    return sensorarrayWifiRequestAp(&wifiConfig, requestId);
}

void sensorarrayTransportConfigureWifiBoot(uint64_t sessionId, uint32_t bootId)
{
    sensorarrayWifiConfigureBoot(sessionId, bootId);
}

void sensorarrayTransportReleaseWifiStartupGate(void)
{
    sensorarrayWifiAllowOwnerStart();
    /* Called only after the two main startup participants reserve scan/probe
     * and USB stacks. Default OFF allocates neither Wi-Fi task nor payload. */
    if (SENSORARRAY_CFG_OUTPUT_WIFI_TEXT) {
        sensorarrayWifiOwnerSnapshot_t snapshot;
        sensorarrayWifiCopyOwnerSnapshot(&snapshot);
        if (!snapshot.resourceAttempted && snapshot.requestId == 0u) {
            (void)sensorarrayTransportStartWifiAp(NULL);
        }
    }
}

static void sensorarrayTransportStatResult(sensorarrayTransportChannel_t channel,
                                           bool ble,
                                           esp_err_t err)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    uint32_t *sent = NULL;
    uint32_t *drop = NULL;
    if (ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
        sent = &g_sensorarrayTransportStats.bleDataSent;
        drop = &g_sensorarrayTransportStats.bleDataDrop;
    } else if (ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
        sent = &g_sensorarrayTransportStats.bleLogSent;
        drop = &g_sensorarrayTransportStats.bleLogDrop;
    } else if (!ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
        sent = &g_sensorarrayTransportStats.wifiDataSent;
        drop = &g_sensorarrayTransportStats.wifiDataDrop;
    } else if (!ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
        sent = &g_sensorarrayTransportStats.wifiLogSent;
        drop = &g_sensorarrayTransportStats.wifiLogDrop;
    }
    if (sent && drop) {
        if (err == ESP_OK) {
            (*sent)++;
        } else {
            (*drop)++;
        }
    }
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

static void sensorarrayTransportWifiCompleted(const sensorarrayWifiTxResult_t *result, void *context)
{
    (void)context;
    /* Legacy sent counters still mean an actual full sendto return, including
     * a late return. Neither local admission nor UDP acceptance proves receipt. */
    if (result->channel == SENSORARRAY_WIFI_CH_CTRL) {
        if (result->fullLength) {
            portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
            g_sensorarrayTransportStats.wifiCtrlTx++;
            portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        }
    } else {
        sensorarrayTransportStatResult(result->channel == SENSORARRAY_WIFI_CH_DATA ?
            SENSORARRAY_TRANSPORT_CHANNEL_DATA : SENSORARRAY_TRANSPORT_CHANNEL_LOG,
            false, result->fullLength ? ESP_OK : result->error);
    }
}

/* End a protected descriptor's local network TTL without any SDK/lwIP call.
 * Call only with a dequeued/task-owned descriptor. BLE NO_MEM retries remain
 * unadmitted; Wi-Fi records its one attempted submission separately. */
static bool sensorarrayTransportExpireOwnedProtected(
    const sensorarrayTransportDescriptor_t *descriptor)
{
    if (!sensorarrayTransportDescriptorCurrent(descriptor)) { return false; }
    const sensorarrayTransportPayloadSlot_t *slot = sensorarrayTransportDescriptorSlot(descriptor);
    bool bleExpired = slot->bleConnectionGeneration != 0u;
    bool wifiExpired = slot->protectedWifiTarget && !slot->protectedWifiDone;
    if (!sensorarrayTransportReleaseDescriptor(descriptor)) { return false; }
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    s_transportProtectionStats.expired++;
    if (bleExpired) {
        s_transportProtectionStats.bleExpired++;
        g_sensorarrayTransportStats.bleDataDrop++;
    }
    if (wifiExpired) {
        s_transportProtectionStats.wifiExpired++;
        g_sensorarrayTransportStats.wifiDataDrop++;
    }
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    return true;
}

static void sensorarrayTransportBleControl(const uint8_t *data, size_t length, void *context)
{
    (void)context;
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    g_sensorarrayTransportStats.bleCtrlRx++;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    sensorarrayTransportReplyTarget_t target = {.kind = SENSORARRAY_TRANSPORT_REPLY_BLE};
    (void)sensorarrayTransportHandleControlCommand(data, length, &target);
}

static void sensorarrayTransportWifiControl(const uint8_t *data,
                                            size_t length,
                                            const sensorarrayWifiPeer_t *peer,
                                            void *context)
{
    (void)context;
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    g_sensorarrayTransportStats.wifiCtrlRx++;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    sensorarrayTransportReplyTarget_t target = {
        .kind = SENSORARRAY_TRANSPORT_REPLY_WIFI,
        .wifiPeer = *peer,
    };
    (void)sensorarrayTransportHandleControlCommand(data, length, &target);
}

static void sensorarrayTransportSerialControlTask(void *arg)
{
    (void)arg;
    char line[SENSORARRAY_TRANSPORT_SERIAL_COMMAND_MAX];
    char command[SENSORARRAY_TRANSPORT_SERIAL_COMMAND_MAX];
    uint8_t bytes[64];
    size_t used = 0;
    uint32_t commandEpoch = 0;
    bool overflow = false;
    sensorarrayTransportReplyTarget_t target = {.kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL};
    for (;;) {
        uint32_t epoch = commandEpoch;
        int count = sensorarraySerialIoReadWithEpoch(bytes, sizeof(bytes), &epoch);
        if (epoch != commandEpoch) { used = 0; overflow = false; commandEpoch = epoch; }
        if (count <= 0) {
            /* An unbound/failed driver is also a bounded idle condition. */
            vTaskDelay(1u);
            continue;
        }
        for (int i = 0; i < count; ++i) {
            /* Acceptance fence: queued/partially read commands from a prior
             * USB connection do not become new commands after re-enumeration. */
            if (sensorarrayUsbOtgEnabled() && epoch != sensorarrayUsbOtgEpoch()) {
                used = 0; overflow = false; break;
            }
            uint8_t ch = bytes[i];
            if (ch == '\r' || ch == '\n') {
                if (overflow) {
                    sensorarraySerialIoNoteOversize();
                    (void)sensorarrayTransportPublishControlReply(&target,
                        "ERR,cmd=SERIAL,reason=too_long\n", sizeof("ERR,cmd=SERIAL,reason=too_long\n") - 1u);
                } else if (used) {
                    const char *reason = "ok";
                    line[used] = '\0';
                    esp_err_t err = sensorarrayTransportNormaliseSerialCommand(line, used,
                        command, sizeof(command), &reason);
                    if (err == ESP_OK) {
                        (void)sensorarrayTransportHandleControlCommand((const uint8_t *)command,
                            strlen(command), &target);
                    } else if (err != ESP_ERR_NOT_FOUND) {
                        sensorarrayTransportLogSerialCommandError((const uint8_t *)line, used, reason);
                    }
                }
                if (overflow || used != 0u) {
                    (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_CTRL);
                }
                used = 0;
                overflow = false;
            } else if (!overflow) {
                if (used + 1u < sizeof(line)) { line[used++] = (char)ch; }
                else { overflow = true; }
            }
        }
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_CTRL);
    }
}

static void sensorarrayTransportPrintNetInit(esp_err_t wifiErr)
{
    sensorarrayBleStats_t ble = {0};
    sensorarrayWifiStats_t wifi = {0};
    sensorarrayBleGetStats(&ble);
    sensorarrayWifiGetStats(&wifi);
    bool bleOk = ble.controllerReady && ble.hostReady && ble.gattReady && ble.advertising;
    printf("NET,ble=%s,ap=%s,mdns=%s,wifi=%s,wok=%u,werr=0x%lx,berr=0x%lx,bok=%u\n",
           s_deviceName,
           s_deviceName,
           s_mdnsName,
           sensorarrayTransportWifiModeName(sensorarrayTransportGetWifiMode()),
           wifi.ready ? 1u : 0u,
           (unsigned long)wifiErr,
           (unsigned long)ble.initError,
           bleOk ? 1u : 0u);
}

static void sensorarrayTransportProcessDescriptor(
    const sensorarrayTransportDescriptor_t *descriptor)
{
    if (!sensorarrayTransportDescriptorCurrent(descriptor)) {
        return;
    }

    sensorarrayTransportPayloadSlot_t *slot =
        sensorarrayTransportDescriptorSlot(descriptor);
    sensorarrayTransportChannel_t channel =
        (sensorarrayTransportChannel_t)descriptor->channel;
    sensorarrayBleChannel_t bleChannel =
        channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA ?
            SENSORARRAY_BLE_CH_DATA : SENSORARRAY_BLE_CH_LOG;
    sensorarrayWifiChannel_t wifiChannel =
        channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA ?
            SENSORARRAY_WIFI_CH_DATA : SENSORARRAY_WIFI_CH_LOG;

    sensorarrayTransportChannel_t sinkChannel =
        channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE ?
            SENSORARRAY_TRANSPORT_CHANNEL_LOG : channel;
    bool protectedFrame = slot && slot->trustedFullFrame && slot->protectionFlags != 0u;
    bool retainForRetry = false;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    bool obsolete = slot && slot->trustedFullFrame && s_lastBleFrameKnown &&
        slot->bleConnectionGeneration == s_lastBleConnectionGeneration &&
        sensorarrayTransportFrameOlderOrSame(&slot->frameIdentity, &s_lastBleFrame);
    bool expired = protectedFrame && nowUs >= slot->protectedDeadlineUs;
    if (expired) {
        (void)sensorarrayTransportExpireOwnedProtected(descriptor);
        return;
    }
    if (obsolete) {
        sensorarrayTransportProtectionCount(&g_sensorarrayTransportStats.bleDataObsolete);
        if (protectedFrame) { sensorarrayTransportProtectionCount(&s_transportProtectionStats.superseded); }
    } else if (slot && slot->bleConnectionGeneration != 0u &&
        SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT &&
        sensorarrayTransportBleChannelEnabled(sinkChannel)) {
        char shortData[384];
        const char *bleData = slot->data;
        size_t bleLength = slot->length;
        if (sinkChannel == SENSORARRAY_TRANSPORT_CHANNEL_DATA &&
            sensorarrayTransportGetTxMode() == SENSORARRAY_TRANSPORT_TX_SHORT &&
            sensorarrayTransportBuildPreview(slot,
                                               shortData,
                                               sizeof(shortData))) {
            bleData = shortData;
            bleLength = strlen(shortData);
        }
        esp_err_t bleErr;
        if (slot->trustedFullFrame) {
            /* The ordinary preview and full original are both available until
             * BLE atomically selects its stream epoch/protection class. That
             * synchronous API deep-copies the chosen bytes before returning. */
            sensorarrayBleFramePublication_t publication = {
                .fullData = (const uint8_t *)slot->data, .fullLength = slot->length,
                .normalData = (const uint8_t *)bleData, .normalLength = bleLength,
                .identity = {.sessionId = slot->frameIdentity.sessionId,
                    .bootId = slot->frameIdentity.bootId, .revision = slot->frameIdentity.revision,
                    .sequence = slot->frameIdentity.sequence},
                .protectionFlags = slot->protectionFlags,
            };
            bleErr = sensorarrayBleNotifyFrameForGeneration(&publication, slot->bleConnectionGeneration);
        } else {
            bleErr = channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE ?
                sensorarrayBleNotifyLifecycleForGeneration((const uint8_t *)bleData,
                    bleLength, slot->bleConnectionGeneration) :
                sensorarrayBleNotifyForGeneration(bleChannel, (const uint8_t *)bleData,
                    bleLength, slot->bleConnectionGeneration);
        }
        if (protectedFrame && bleErr == ESP_ERR_NO_MEM) {
            retainForRetry = sensorarrayTransportHoldProtectedRetry(descriptor, slot, nowUs);
        }
        if (protectedFrame && bleErr == ESP_OK) {
            /* Only a protected admission establishes this overtaking fence.
             * Ordinary admission is neither completion nor evidence that an
             * older protected obligation may be discarded. */
            s_lastBleFrame = slot->frameIdentity;
            s_lastBleConnectionGeneration = slot->bleConnectionGeneration;
            s_lastBleFrameKnown = true;
        }
        if (slot->trustedFullFrame && !protectedFrame && bleErr == ESP_ERR_NOT_ALLOWED) {
            sensorarrayTransportProtectionCount(&g_sensorarrayTransportStats.bleDataPolicySkipped);
        } else if (!retainForRetry) {
            sensorarrayTransportStatResult(sinkChannel, true, bleErr);
            if (protectedFrame) {
                sensorarrayTransportProtectionCount(bleErr == ESP_OK ?
                    &s_transportProtectionStats.bleAccepted : &s_transportProtectionStats.bleFailed);
            }
        }
    } else if (protectedFrame && slot->bleConnectionGeneration != 0u) {
        sensorarrayTransportProtectionCount(&s_transportProtectionStats.bleFailed);
        sensorarrayTransportStatResult(SENSORARRAY_TRANSPORT_CHANNEL_DATA, true, ESP_ERR_INVALID_STATE);
    }
    if (slot && !slot->protectedWifiDone && slot->wifiEpoch != 0u) {
        sensorarrayWifiPublication_t publication = {
            .channel = wifiChannel, .lifecycle = channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,
            .data = (const uint8_t *)slot->data, .length = slot->length,
            .expectedEpoch = slot->wifiEpoch, .deadlineUs = slot->wifiDeadlineUs,
            .identityKnown = slot->trustedFullFrame,
            .sessionId = slot->frameIdentity.sessionId, .bootId = slot->frameIdentity.bootId,
            .revision = slot->frameIdentity.revision, .sequence = slot->frameIdentity.sequence,
        };
        esp_err_t sendWifiErr = sensorarrayTransportWifiSinkEnabled() ?
            sensorarrayWifiSubmit(&publication) : ESP_ERR_INVALID_STATE;
        /* Accepted copies complete asynchronously in the Wi-Fi owner. Count
         * only admission failures here; completion owns the legacy sent count. */
        if (sendWifiErr != ESP_OK) { sensorarrayTransportStatResult(sinkChannel, false, sendWifiErr); }
        if (protectedFrame) { slot->protectedWifiDone = true; }
    }
    if (sensorarrayBleIsCongested()) {
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        g_sensorarrayTransportStats.bleCongested++;
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    }
    if (!retainForRetry) { (void)sensorarrayTransportReleaseDescriptor(descriptor); }
}

static void sensorarrayTransportNotifyTask(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

static void sensorarrayTransportDrainGuaranteedText(void)
{
    char text[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
    size_t length = 0u;
    if (!sensorarrayTransportGuaranteedTextTakeDrain(text, sizeof(text), &length)) {
        return;
    }

    esp_err_t serialErr = sensorarraySerialIoQueue(text, length, true);
    sensorarrayTransportNoteSerialLog(serialErr == ESP_OK);

    uint32_t bleConnectionGeneration = 0u;
    if (SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT &&
        sensorarrayTransportBleChannelEnabled(SENSORARRAY_TRANSPORT_CHANNEL_LOG) &&
        sensorarrayBleGetSendGeneration(SENSORARRAY_BLE_CH_LOG,
                                        &bleConnectionGeneration)) {
        esp_err_t bleErr = sensorarrayBleNotifyLifecycleForGeneration(
            (const uint8_t *)text, length, bleConnectionGeneration);
        sensorarrayTransportStatResult(SENSORARRAY_TRANSPORT_CHANNEL_LOG,
                                       true,
                                       bleErr);
    }
    uint32_t wifiEpoch = 0u;
    if (sensorarrayTransportWifiSinkEnabled() && sensorarrayWifiGetSendEpoch(false, &wifiEpoch)) {
        sensorarrayWifiPublication_t publication = {.channel = SENSORARRAY_WIFI_CH_LOG,
            .lifecycle = true, .data = (const uint8_t *)text, .length = length, .expectedEpoch = wifiEpoch};
        esp_err_t wifiErr = sensorarrayWifiSubmit(&publication);
        if (wifiErr != ESP_OK) {
            sensorarrayTransportStatResult(SENSORARRAY_TRANSPORT_CHANNEL_LOG, false, wifiErr);
        }
    }
}

/* 本轮新增: deadline reclamation is independent of transmission priority.
 * This descriptor is owned solely by this task after a synchronous BLE
 * NO_MEM rejection; no SDK/DMA call retains its payload. Release still checks
 * the slot generation, so an unproven owner is never reclaimed. */
static void sensorarrayTransportExpireProtectedRetry(void)
{
    if (!s_protectedRetryActive) { return; }
    sensorarrayTransportPayloadSlot_t *slot = sensorarrayTransportDescriptorSlot(&s_protectedRetry);
    if (!slot || (uint64_t)esp_timer_get_time() < slot->protectedDeadlineUs) { return; }
    if (!sensorarrayTransportExpireOwnedProtected(&s_protectedRetry)) { return; }
    s_protectedRetryActive = false;
    /* Prior NO_MEM retry evidence remains. This is local TTL disposal, never
     * cancellation of an accepted send or proof of peer delivery. */
}

/* At most the two real protected slots can be dequeued here. Dequeue is
 * the ownership transfer; never scan or reclaim a producer's WRITING slot.
 * Temporarily held live descriptors are restored to the front in reverse
 * order, preserving their FIFO order ahead of concurrent new publications. */
static void sensorarrayTransportExpireProtectedPending(void)
{
    if (!s_protectedQueue) { return; }
    sensorarrayTransportDescriptor_t retained[SENSORARRAY_TRANSPORT_POOL_PROTECTED_SLOT_COUNT];
    unsigned count = 0u;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    for (unsigned inspected = 0u; inspected < SENSORARRAY_TRANSPORT_POOL_PROTECTED_SLOT_COUNT; ++inspected) {
        sensorarrayTransportDescriptor_t descriptor;
        if (xQueueReceive(s_protectedQueue, &descriptor, 0) != pdTRUE) { break; }
        if (!sensorarrayTransportDescriptorCurrent(&descriptor)) { continue; }
        sensorarrayTransportPayloadSlot_t *slot = sensorarrayTransportDescriptorSlot(&descriptor);
        if (nowUs >= slot->protectedDeadlineUs) {
            (void)sensorarrayTransportExpireOwnedProtected(&descriptor);
        } else {
            retained[count++] = descriptor;
        }
    }
    while (count != 0u) {
        const sensorarrayTransportDescriptor_t *descriptor = &retained[--count];
        /* Each held descriptor occupies one of the only two pool slots; the
         * queue also holds two entries. Concurrent producers therefore cannot
         * fill the space needed to return an owner we still hold. */
        if (xQueueSendToFront(s_protectedQueue, descriptor, 0) != pdTRUE) {
            /* A violated capacity invariant must be counted, not silently
             * lose an owner. This local descriptor is still ours to release. */
            if (sensorarrayTransportReleaseDescriptor(descriptor)) {
                sensorarrayTransportProtectionCount(&s_transportProtectionStats.bleFailed);
                sensorarrayTransportStatResult(SENSORARRAY_TRANSPORT_CHANNEL_DATA, true, ESP_ERR_NO_MEM);
            }
        }
    }
}

static bool sensorarrayTransportReceiveNext(
    sensorarrayTransportDescriptor_t *outDescriptor,
    uint32_t *dataBudget)
{
    if (!outDescriptor || !dataBudget) {
        return false;
    }
    /* Constant bounded owner cleanup runs even under continuous control
     * or newer protected traffic. It does not send a retry ahead of control. */
    sensorarrayTransportExpireProtectedRetry();
    sensorarrayTransportExpireProtectedPending();
    /* Lifecycle wins at every descriptor boundary. DATA retains its bounded
     * throughput batch ahead of ordinary diagnostics. Neither queue activity
     * nor a notification resets the independent CPU work budget. */
    if (s_lifecycleQueue &&
        xQueueReceive(s_lifecycleQueue, outDescriptor, 0) == pdTRUE) {
        *dataBudget = SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT;
        return true;
    }
    if (s_protectedQueue && xQueueReceive(s_protectedQueue, outDescriptor, 0) == pdTRUE) {
        return true;
    }
    if (s_protectedRetryActive) {
        sensorarrayTransportPayloadSlot_t *retry = sensorarrayTransportDescriptorSlot(&s_protectedRetry);
        uint64_t nowUs = (uint64_t)esp_timer_get_time();
        if (nowUs >= retry->protectedNextAttemptUs || nowUs >= retry->protectedDeadlineUs) {
            *outDescriptor = s_protectedRetry;
            s_protectedRetryActive = false;
            return true;
        }
    }
    if (*dataBudget > 0u && s_dataQueue &&
        xQueueReceive(s_dataQueue, outDescriptor, 0) == pdTRUE) {
        (*dataBudget)--;
        return true;
    }
    if (s_logQueue && xQueueReceive(s_logQueue, outDescriptor, 0) == pdTRUE) {
        *dataBudget = SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT;
        return true;
    }
    return false;
}

static void sensorarrayTransportTask(void *arg)
{
    esp_err_t wifiErr = (esp_err_t)(intptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(750u));
    sensorarrayTransportPrintNetInit(wifiErr);
    sensorarrayTransportDescriptor_t descriptor;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, sensorarrayTransportWaitTicks());
        uint32_t dataBudget = SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT;
        unsigned processed = 0u;
        for (; processed < SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT; ++processed) {
            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT);
            /* At most one guaranteed terminal per iteration; it stays ahead
             * of ordinary data even when all queues are continuously refilled. */
            sensorarrayTransportWifiDrainTerminal();
            sensorarrayTransportDrainGuaranteedText();
            if (!sensorarrayTransportReceiveNext(&descriptor, &dataBudget)) {
                break;
            }
            sensorarrayTransportProcessDescriptor(&descriptor);
        }
        if (processed == SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT) {
            /* ulTaskNotifyTake(pdTRUE) consumed previous wakeups. Retain work
             * after a full batch without relying on a future producer wakeup. */
            sensorarrayTransportNotifyTask();
        }
    }
}

static bool sensorarrayTransportSupersedePendingProtected(
    const sensorarrayTransportFrameIdentity_t *identity)
{
    sensorarrayTransportDescriptor_t old;
    if (!s_protectedQueue || xQueueReceive(s_protectedQueue, &old, 0) != pdTRUE) { return false; }
    if (!sensorarrayTransportDescriptorCurrent(&old)) { return false; }
    sensorarrayTransportPayloadSlot_t *slot = sensorarrayTransportDescriptorSlot(&old);
    bool replace = !sensorarrayTransportFrameSameConfig(identity, &slot->frameIdentity) ||
        !(slot->protectionFlags & SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG);
    if (!replace && xQueueSend(s_protectedQueue, &old, 0) == pdTRUE) { return false; }
    /* xQueueReceive transferred ownership to this producer. Neither a
     * WRITING slot nor the consumer's retry/active slot can enter this path. */
    (void)sensorarrayTransportReleaseDescriptor(&old);
    sensorarrayTransportProtectionCount(&s_transportProtectionStats.superseded);
    return true;
}

static esp_err_t sensorarrayTransportQueueFrameInternal(sensorarrayTransportChannel_t channel,
                                           const char *data,
                                           size_t length,
                                           bool *outQueued,
                                           const sensorarrayTransportFrameIdentity_t *identity,
                                           uint32_t protectionFlags,
                                           const sensorarrayTransportFrameMetadata_t *metadata)
{
    if (outQueued) { *outQueued = false; }
    if ((channel != SENSORARRAY_TRANSPORT_CHANNEL_DATA &&
         channel != SENSORARRAY_TRANSPORT_CHANNEL_LOG &&
         channel != SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE) ||
        !data || length == 0u ||
        length > SENSORARRAY_TRANSPORT_TEXT_MAX) {
        if (length > SENSORARRAY_TRANSPORT_TEXT_MAX) {
            printf("TXDROP,ch=%u,len=%lu,max=%u,reason=payload_too_large\n",
                   (unsigned)channel,
                   (unsigned long)length,
                   (unsigned)SENSORARRAY_TRANSPORT_TEXT_MAX);
        }
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t suppliedProtection = protectionFlags;
    if (identity) {
        uint64_t nowUs = (uint64_t)esp_timer_get_time();
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        if (!s_protectedAdmissionKnown ||
            !sensorarrayTransportFrameSameConfig(identity, &s_protectedAdmissionIdentity)) {
            protectionFlags |= SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG;
        }
        if (!s_protectedAdmissionKnown || nowUs < s_lastProtectedAdmissionUs ||
            nowUs - s_lastProtectedAdmissionUs >= SENSORARRAY_TRANSPORT_PROTECTED_DEADLINE_US) {
            protectionFlags |= SENSORARRAY_FRAME_PROTECT_PERIODIC;
        }
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    }
    bool protectedFrame = identity && protectionFlags != 0u;
    QueueHandle_t queue = protectedFrame ? s_protectedQueue : channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA ?
        s_dataQueue : (channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE ?
            s_lifecycleQueue : s_logQueue);
    if (!queue) {
        return ESP_ERR_INVALID_STATE;
    }
    sensorarrayTransportChannel_t bleSinkChannel =
        channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE ?
            SENSORARRAY_TRANSPORT_CHANNEL_LOG : channel;
    sensorarrayBleChannel_t bleChannel =
        bleSinkChannel == SENSORARRAY_TRANSPORT_CHANNEL_DATA ?
            SENSORARRAY_BLE_CH_DATA : SENSORARRAY_BLE_CH_LOG;
    uint32_t bleConnectionGeneration = 0u;
    bool bleEnabled = SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT &&
                      sensorarrayTransportBleChannelEnabled(bleSinkChannel) &&
                      sensorarrayBleGetSendGeneration(
                          bleChannel, &bleConnectionGeneration);
    uint32_t wifiEpoch = 0u;
    bool wifiEnabled = sensorarrayTransportWifiSinkEnabled();
    bool wifiAvailable = wifiEnabled && sensorarrayWifiGetSendEpoch(false, &wifiEpoch);
    if (!bleEnabled && !wifiEnabled) {
        return ESP_OK;
    }

    sensorarrayTransportDescriptor_t descriptor;
    bool allocated;
    if (protectedFrame) {
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        allocated = sensorarrayTransportPoolAllocateProtected(&s_pool, length, &descriptor);
        sensorarrayTransportSyncPoolStatsLocked();
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        if (!allocated && sensorarrayTransportSupersedePendingProtected(identity)) {
            portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
            allocated = sensorarrayTransportPoolAllocateProtected(&s_pool, length, &descriptor);
            sensorarrayTransportSyncPoolStatsLocked();
            portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        }
    } else {
        allocated = sensorarrayTransportAllocateDescriptor(channel, length, &descriptor);
    }
    if (!allocated) {
        if (protectedFrame) {
            sensorarrayTransportProtectionCount(&s_transportProtectionStats.admissionMiss);
            if (protectionFlags & SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG) {
                sensorarrayTransportProtectionCount(&s_transportProtectionStats.firstCandidateMiss);
            }
        }
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        esp_err_t allocationErr = s_pool.storage ? ESP_ERR_TIMEOUT : ESP_ERR_NO_MEM;
        g_sensorarrayTransportStats.queueDrop++;
        if (channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
            g_sensorarrayTransportStats.queueDropData++;
        } else if (channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
            g_sensorarrayTransportStats.queueDropLog++;
        } else {
            g_sensorarrayTransportStats.queueDropLifecycle++;
            g_sensorarrayTransportStats.lifecycleDropped++;
        }
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return allocationErr;
    }

    sensorarrayTransportPayloadSlot_t *slot =
        sensorarrayTransportDescriptorSlot(&descriptor);
    slot->bleConnectionGeneration = bleEnabled ?
        bleConnectionGeneration : 0u;
    slot->wifiEpoch = wifiAvailable ? wifiEpoch : 0u;
    slot->wifiDeadlineUs = (uint64_t)esp_timer_get_time() + SENSORARRAY_WIFI_QUEUE_DEADLINE_US;
    memcpy(slot->data, data, length);
    slot->data[length] = '\0';
    if (metadata) {
        if (!sensorarrayTransportApplyFrameMetadata(slot, metadata)) {
            (void)sensorarrayTransportReleaseDescriptor(&descriptor);
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        sensorarrayTransportFillFrameMeta(slot);
    }
    if (identity) {
        slot->frameIdentity = *identity;
        slot->trustedFullFrame = true;
        slot->protectionFlags = protectionFlags;
        slot->protectedDeadlineUs = (uint64_t)esp_timer_get_time() + SENSORARRAY_TRANSPORT_PROTECTED_DEADLINE_US;
        slot->protectedNextAttemptUs = 0u;
        slot->protectedWifiDone = false;
        slot->protectedWifiTarget = wifiAvailable;
    }
    if (xQueueSend(queue, &descriptor, 0) != pdTRUE) {
        if (protectedFrame) {
            sensorarrayTransportProtectionCount(&s_transportProtectionStats.admissionMiss);
            if (protectionFlags & SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG) {
                sensorarrayTransportProtectionCount(&s_transportProtectionStats.firstCandidateMiss);
            }
        }
        if (slot->hasFrameMeta) {
            uint32_t profileGeneration = 0u;
            uint32_t profileRequestId = 0u;
            if (slot->data[0] == 'M') {
                (void)sensorarrayTransportFindFieldU32(slot->data,
                                                        "pgen=",
                                                        &profileGeneration);
                (void)sensorarrayTransportFindFieldU32(slot->data,
                                                        "prid=",
                                                        &profileRequestId);
            }
            printf("TXDROP,ch=%u,seq=%lu,rows=%u,cells=%u,gen=%lu,rid=%lu,pgen=%lu,prid=%lu,reason=transport_queue_full\n",
                   (unsigned)slot->channel,
                   (unsigned long)slot->frameSeq,
                   (unsigned)slot->rows,
                   (unsigned)slot->cells,
                   (unsigned long)slot->generation,
                   (unsigned long)slot->requestId,
                   (unsigned long)profileGeneration,
                   (unsigned long)profileRequestId);
        }
        (void)sensorarrayTransportReleaseDescriptor(&descriptor);
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        g_sensorarrayTransportStats.queueDrop++;
        if (channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
            g_sensorarrayTransportStats.queueDropData++;
        } else if (channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
            g_sensorarrayTransportStats.queueDropLog++;
        } else {
            g_sensorarrayTransportStats.queueDropLifecycle++;
            g_sensorarrayTransportStats.lifecycleDropped++;
        }
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return ESP_ERR_TIMEOUT;
    }
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
    if (channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE) {
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        g_sensorarrayTransportStats.lifecyclePublished++;
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    }
    if (outQueued) { *outQueued = true; }
    if (protectedFrame) {
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        s_transportProtectionStats.queued++;
        s_transportProtectionStats.derivedProtection += protectionFlags != suppliedProtection ? 1u : 0u;
        s_protectedAdmissionKnown = true;
        s_protectedAdmissionIdentity = *identity;
        s_lastProtectedAdmissionUs = (uint64_t)esp_timer_get_time();
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    }
    return ESP_OK;
}

static esp_err_t sensorarrayTransportQueue(sensorarrayTransportChannel_t channel,
    const char *data, size_t length, bool *outQueued)
{
    return sensorarrayTransportQueueFrameInternal(channel, data, length, outQueued, NULL, 0u, NULL);
}

static esp_err_t sensorarrayTransportEnsurePayloadStorage(void)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    if (s_payloadStorageState != 0u) {
        esp_err_t result = s_payloadStorageState == 1u ? ESP_ERR_INVALID_STATE : s_payloadStorageError;
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return result;
    }
    s_payloadStorageState = 1u;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    char *storage = heap_caps_malloc(SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    bool bound = storage && sensorarrayTransportPoolInit(&s_pool, storage,
        SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES);
    s_payloadStorageError = bound ? ESP_OK : storage ? ESP_ERR_INVALID_STATE : ESP_ERR_NO_MEM;
    s_payloadStorageState = bound ? 2u : 3u;
    esp_err_t result = s_payloadStorageError;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    /* Binding failure never publishes the new block. Existing pool/owned
     * pointers are untouched, so only this unreferenced allocation is freed. */
    if (storage && !bound) { heap_caps_free(storage); }
    return result;
}

static esp_err_t sensorarrayTransportInitOwned(void)
{
    esp_err_t budgetErr = sensorarrayWorkBudgetInit();
    if (budgetErr != ESP_OK) {
        printf("WORKBUDGET,owner=transport,err=0x%lx,fallback=bounded\n",
               (unsigned long)budgetErr);
    }
    if (s_started) {
        /* Network/control initialization can succeed before optional Serial
         * binding. Retry that binding and retain its error until it succeeds. */
        esp_err_t serialErr = sensorarraySerialIoInit();
        return serialErr != ESP_OK ? serialErr : s_completedInitError;
    }
    esp_err_t serialErr = sensorarraySerialIoInit();
    esp_err_t storageErr = sensorarrayTransportEnsurePayloadStorage();
    /* A failed optional USB driver must not suppress BLE/Wi-Fi control. */
    (void)sensorarrayScanConfigInit();
    uint8_t mac[6] = {0};
    (void)esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    sensorarrayTransportBuildNames(mac);
    sensorarrayTransportSetStream(SENSORARRAY_TRANSPORT_STREAM_AUTO);
#if CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_SHORT
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_SHORT);
#elif CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_FULL
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_FULL);
#else
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_REL);
#endif
    sensorarrayTransportSetWifiMode(SENSORARRAY_CFG_OUTPUT_WIFI_TEXT ?
                                    SENSORARRAY_TRANSPORT_WIFI_AP :
                                    SENSORARRAY_TRANSPORT_WIFI_OFF);

    esp_err_t nvsErr = nvs_flash_init();
    /* Storage/version failure is reported without erasing existing contents.
     * Optional BLE startup may fail; command/recovery tasks still start. */
    sensorarrayBleSetControlRxCallback(sensorarrayTransportBleControl, NULL);
    sensorarrayBleConfig_t bleConfig = {
        .enableBle5 = true,
        .preferPhy2m = true,
        .preferHighTxPower = true,
        .preferredMtu = 517u,
    };
    snprintf(bleConfig.deviceName, sizeof(bleConfig.deviceName), "%s", s_deviceName);
    esp_err_t bleErr = ESP_OK;
    if (SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT) {
        bleErr = nvsErr == ESP_OK ? sensorarrayBleInit(&bleConfig) : nvsErr;
    }

    sensorarrayWifiSetControlRxCallback(sensorarrayTransportWifiControl, NULL);
    sensorarrayWifiSetOwnerCallbacks(sensorarrayTransportWifiCompleted, NULL, sensorarrayTransportNotifyTask);
    /* AP startup is deliberately deferred until both main startup reservation
     * participants release the gate. This return is not a Wi-Fi applied event. */
    esp_err_t wifiErr = ESP_OK;

    if (!s_dataQueue && !s_logQueue && !s_lifecycleQueue && !s_protectedQueue) {
        s_dataQueue = xQueueCreateStatic(SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT,
                                        sizeof(sensorarrayTransportDescriptor_t),
                                        s_dataQueueStorage,
                                        &s_dataQueueStruct);
        s_logQueue = xQueueCreateStatic(SENSORARRAY_TRANSPORT_LOG_QUEUE_COUNT,
                                       sizeof(sensorarrayTransportDescriptor_t),
                                       s_logQueueStorage,
                                       &s_logQueueStruct);
        s_lifecycleQueue = xQueueCreateStatic(SENSORARRAY_TRANSPORT_LIFECYCLE_QUEUE_COUNT,
                                              sizeof(sensorarrayTransportDescriptor_t),
                                              s_lifecycleQueueStorage,
                                              &s_lifecycleQueueStruct);
        s_protectedQueue = xQueueCreateStatic(SENSORARRAY_TRANSPORT_PROTECTED_QUEUE_COUNT,
                                              sizeof(sensorarrayTransportDescriptor_t),
                                              s_protectedQueueStorage,
                                              &s_protectedQueueStruct);
    }
    if (!s_dataQueue || !s_logQueue || !s_lifecycleQueue || !s_protectedQueue) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t ok = pdPASS;
    if (!s_task) {
        ok = xTaskCreatePinnedToCore(sensorarrayTransportTask,
                                                "transport",
                                                CONFIG_SENSORARRAY_TRANSPORT_TASK_STACK,
                                                (void *)(intptr_t)wifiErr,
                                                7u,
                                                &s_task,
                                                CONFIG_SENSORARRAY_OUTPUT_TASK_CORE);
        if (ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    sensorarrayTransportGuaranteedTextSetNotify(sensorarrayTransportNotifyTask);
    if (!s_serialTask) {
        ok = xTaskCreatePinnedToCore(sensorarrayTransportSerialControlTask,
                                     "serialCtrl",
                                     CONFIG_SENSORARRAY_SERIAL_CTRL_TASK_STACK,
                                     NULL,
                                     8u,
                                     &s_serialTask,
                                     CONFIG_SENSORARRAY_COMM_TASK_CORE);
        if (ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_started = true;
    /* Serial/transport tasks remain available when an optional network sink
     * fails, but the caller must still receive the real initialization error.
     * A disabled sink is not evidence that another enabled sink succeeded. */
    s_completedInitError = ESP_OK;
    if (nvsErr != ESP_OK) {
        s_completedInitError = nvsErr;
    } else if (SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT && bleErr != ESP_OK) {
        s_completedInitError = bleErr;
    } else if (SENSORARRAY_CFG_OUTPUT_WIFI_TEXT && wifiErr != ESP_OK) {
        s_completedInitError = wifiErr;
    } else if (storageErr != ESP_OK) {
        s_completedInitError = storageErr;
    }
    /* Pool failure denies DATA/LOG/lifecycle allocation but must not prevent
     * the independent CTRL queues, command RX and recovery queries above. */
    /* Serial binding alone remains retryable. Completed network/storage
     * failures stay visible; repeated Init never claims an unstarted BLE
     * backend recovered merely because the control tasks are already alive. */
    return serialErr != ESP_OK ? serialErr : s_completedInitError;
}

esp_err_t sensorarrayTransportInit(void)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    if (s_initInProgress) {
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_initInProgress = true;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    esp_err_t result = sensorarrayTransportInitOwned();
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    s_initInProgress = false;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    return result;
}

void sensorarrayTransportGetMemoryDiagnostics(
    sensorarrayTransportMemoryDiagnostics_t *outDiagnostics)
{
    if (!outDiagnostics) {
        return;
    }
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    uint32_t externalBytes = s_payloadStorageState == 2u ?
        SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES : 0u;
    *outDiagnostics = (sensorarrayTransportMemoryDiagnostics_t){
        .legacyItemBytes = SENSORARRAY_TRANSPORT_LEGACY_ITEM_BYTES,
        .payloadSlotBytes = sizeof(sensorarrayTransportPayloadSlot_t) +
            (externalBytes ? SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE : 0u),
        .descriptorBytes = sizeof(sensorarrayTransportDescriptor_t),
        .legacyQueueStorageBytes = SENSORARRAY_TRANSPORT_LEGACY_QUEUE_STORAGE_BYTES,
        .descriptorQueueStorageBytes = sizeof(s_dataQueueStorage) +
                                       sizeof(s_logQueueStorage) +
                                       sizeof(s_lifecycleQueueStorage) + sizeof(s_protectedQueueStorage),
        .payloadPoolBytes = sizeof(s_pool.slots) + externalBytes,
        .payloadStorageBytes = SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES,
        .payloadInternalBytes = sizeof(s_pool) + sizeof(s_payloadStorageState) + sizeof(s_payloadStorageError),
        .payloadExternalBytes = externalBytes,
        .payloadStorageReady = s_payloadStorageState == 2u,
        .payloadStorageError = s_payloadStorageError,
    };
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

static uint32_t sensorarrayTransportStackMinimumRemainingBytes(TaskHandle_t task)
{
    /* ESP-IDF 5.5.1's Xtensa port defines StackType_t as uint8_t. Keep the
     * multiplication explicit so the reported field remains bytes if the
     * port type ever changes. */
    return task ? (uint32_t)uxTaskGetStackHighWaterMark(task) *
                      (uint32_t)sizeof(StackType_t) : 0u;
}

void sensorarrayTransportGetTaskStackStats(
    sensorarrayTransportTaskStackStats_t *outStats)
{
    if (!outStats) {
        return;
    }
    *outStats = (sensorarrayTransportTaskStackStats_t){
        .transportConfiguredBytes = CONFIG_SENSORARRAY_TRANSPORT_TASK_STACK,
        .transportMinimumRemainingBytes =
            sensorarrayTransportStackMinimumRemainingBytes(s_task),
        .serialCtrlConfiguredBytes = CONFIG_SENSORARRAY_SERIAL_CTRL_TASK_STACK,
        .serialCtrlMinimumRemainingBytes =
            sensorarrayTransportStackMinimumRemainingBytes(s_serialTask),
    };
}

esp_err_t sensorarrayTransportRequestWifiMode(sensorarrayTransportWifiMode_t mode, uint32_t *requestId)
{
    if (requestId) { *requestId = 0u; }
    if (mode == SENSORARRAY_TRANSPORT_WIFI_OFF) { return sensorarrayWifiRequestOff(requestId); }
    if (mode == SENSORARRAY_TRANSPORT_WIFI_AP) { return sensorarrayTransportStartWifiAp(requestId); }
    return (unsigned)mode > SENSORARRAY_TRANSPORT_WIFI_APSTA ? ESP_ERR_INVALID_ARG : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sensorarrayTransportApplyWifiMode(sensorarrayTransportWifiMode_t mode)
{
    return sensorarrayTransportRequestWifiMode(mode, NULL);
}

esp_err_t sensorarrayTransportPublishData(const char *data, size_t length)
{
    return sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_DATA, data, length, NULL);
}

esp_err_t sensorarrayTransportPublishFrame(const char *data, size_t length,
    const sensorarrayTransportFrameIdentity_t *identity, uint32_t protectionFlags)
{
    if (!identity || identity->sessionId == 0u || identity->rows < 1u || identity->rows > 8u ||
        (protectionFlags & ~SENSORARRAY_FRAME_PROTECT_MASK) != 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayTransportQueueFrameInternal(SENSORARRAY_TRANSPORT_CHANNEL_DATA,
        data, length, NULL, identity, protectionFlags, NULL);
}

esp_err_t sensorarrayTransportPublishFrameMetadata(const char *data, size_t length,
    const sensorarrayTransportFrameMetadata_t *metadata, uint32_t protectionFlags)
{
    if (!sensorarrayTransportFrameMetadataValid(metadata, data, length) ||
        (protectionFlags & ~SENSORARRAY_FRAME_PROTECT_MASK) != 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayTransportQueueFrameInternal(SENSORARRAY_TRANSPORT_CHANNEL_DATA,
        data, length, NULL, &metadata->identity, protectionFlags, metadata);
}

esp_err_t sensorarrayTransportPublishLog(const char *data, size_t length)
{
    return sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LOG, data, length, NULL);
}

esp_err_t sensorarrayTransportPublishLifecycle(const char *data, size_t length)
{
    return sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,
                                     data,
                                     length,
                                     NULL);
}

/* 本轮新增: distinguish an actual network descriptor from the established
 * ESP_OK no-op when both BLE/Wi-Fi sinks are disabled. Serial is separate. */
esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *data,
                                                        size_t length,
                                                        bool *outQueued)
{
    if (!outQueued) { return ESP_ERR_INVALID_ARG; }
    return sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,
                                     data, length, outQueued);
}

esp_err_t sensorarrayTransportPublishControlReply(
    const sensorarrayTransportReplyTarget_t *target,
    const char *data,
    size_t length)
{
    if (!target || !data || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length > SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX) {
        printf("CTRLDROP,ch=ctrl,len=%lu,max=%u,reason=reply_too_large\n",
               (unsigned long)length,
               (unsigned)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX);
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err;
    if (target->kind == SENSORARRAY_TRANSPORT_REPLY_BLE) {
        err = sensorarrayBleNotify(SENSORARRAY_BLE_CH_CTRL, (const uint8_t *)data, length);
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        if (err == ESP_OK) {
            g_sensorarrayTransportStats.bleCtrlTx++;
        }
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return err;
    }
    if (target->kind == SENSORARRAY_TRANSPORT_REPLY_WIFI) {
        uint32_t wifiEpoch = 0u;
        if (!sensorarrayWifiGetSendEpoch(true, &wifiEpoch)) { return ESP_ERR_INVALID_STATE; }
        sensorarrayWifiPublication_t publication = {.channel = SENSORARRAY_WIFI_CH_CTRL,
            .unicast = true, .peer = target->wifiPeer, .data = (const uint8_t *)data,
            .length = length, .expectedEpoch = wifiEpoch};
        return sensorarrayWifiSubmit(&publication);
    }
    return sensorarraySerialIoQueue(data, length, true);
}
