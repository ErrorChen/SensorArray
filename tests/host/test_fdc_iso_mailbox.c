#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayCommandMailbox.c"
#include "sensorarrayTransportGuaranteedText.c"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while (0)

#define TEST_QUEUE_LENGTH 8u

static sensorarrayCommand_t s_testQueue[TEST_QUEUE_LENGTH];
static size_t s_testQueueHead;
static size_t s_testQueueCount;

QueueHandle_t xQueueCreateStatic(UBaseType_t queueLength,
                                 UBaseType_t itemSize,
                                 void *storage,
                                 StaticQueue_t *queueStruct)
{
    (void)queueLength;
    (void)itemSize;
    (void)storage;
    (void)queueStruct;
    s_testQueueHead = 0u;
    s_testQueueCount = 0u;
    return (QueueHandle_t)1;
}

BaseType_t xQueueSend(QueueHandle_t queue,
                      const void *item,
                      BaseType_t ticksToWait)
{
    (void)ticksToWait;
    if (!queue || !item || s_testQueueCount >= TEST_QUEUE_LENGTH) {
        return pdFALSE;
    }
    size_t index = (s_testQueueHead + s_testQueueCount) % TEST_QUEUE_LENGTH;
    memcpy(&s_testQueue[index], item, sizeof(*s_testQueue));
    s_testQueueCount++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t queue,
                         void *item,
                         BaseType_t ticksToWait)
{
    (void)ticksToWait;
    if (!queue || !item || s_testQueueCount == 0u) {
        return pdFALSE;
    }
    memcpy(item, &s_testQueue[s_testQueueHead], sizeof(*s_testQueue));
    s_testQueueHead = (s_testQueueHead + 1u) % TEST_QUEUE_LENGTH;
    s_testQueueCount--;
    return pdTRUE;
}

UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t queue)
{
    return queue ? (UBaseType_t)(TEST_QUEUE_LENGTH - s_testQueueCount) : 0u;
}

bool sensorarrayRowModeProfileParse(
    const char *text,
    size_t length,
    sensorarrayMeasurementMode_t outModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])
{
    (void)text;
    (void)length;
    (void)outModes;
    return false;
}

bool sensorarrayMeasurementModeIsDataMode(sensorarrayMeasurementMode_t mode)
{
    return mode != SENSORARRAY_MEASUREMENT_MODE_NONE;
}

static int testFdcIsolatePostAndReceive(void)
{
    CHECK(sensorarrayCommandMailboxInit() == ESP_OK);

    uint32_t requestId = 0u;
    CHECK(sensorarrayCommandMailboxPostFdcIsolation(true, &requestId) == ESP_OK);
    CHECK(requestId != 0u);

    sensorarrayCommand_t command;
    CHECK(sensorarrayCommandMailboxTryReceive(&command));
    CHECK(command.type == SENSORARRAY_COMMAND_FDC_ISOLATE);
    CHECK(command.value == 1u);
    CHECK(command.requestId == requestId);
    CHECK(strcmp(sensorarrayCommandMailboxTypeName(command.type), "fdc_isolate") == 0);
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));

    CHECK(sensorarrayCommandMailboxPostFdcIsolation(false, &requestId) == ESP_OK);
    CHECK(sensorarrayCommandMailboxTryReceive(&command));
    CHECK(command.type == SENSORARRAY_COMMAND_FDC_ISOLATE);
    CHECK(command.value == 0u);
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));
    return 0;
}

static int testFdcIsolateQueueBound(void)
{
    CHECK(sensorarrayCommandMailboxInit() == ESP_OK);
    uint32_t requestId = 0u;
    for (size_t index = 0u; index < TEST_QUEUE_LENGTH; ++index) {
        CHECK(sensorarrayCommandMailboxPostFdcIsolation(true, &requestId) == ESP_OK);
    }
    CHECK(sensorarrayCommandMailboxPostFdcIsolation(true, &requestId) ==
          ESP_ERR_NO_MEM);
    sensorarrayCommand_t command;
    for (size_t index = 0u; index < TEST_QUEUE_LENGTH; ++index) {
        CHECK(sensorarrayCommandMailboxTryReceive(&command));
        CHECK(command.type == SENSORARRAY_COMMAND_FDC_ISOLATE);
    }
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));
    return 0;
}

int main(void)
{
    if (testFdcIsolatePostAndReceive() != 0 ||
        testFdcIsolateQueueBound() != 0) {
        return 1;
    }
    printf("FDC_ISO_MAILBOX,passed=1\n");
    return 0;
}
