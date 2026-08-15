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
    if (!text || !outModes || length == 0u ||
        length > SENSORARRAY_ROW_MODE_PROFILE_ROWS) {
        return false;
    }
    for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        outModes[row] = SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
    }
    return true;
}

bool sensorarrayMeasurementModeIsDataMode(sensorarrayMeasurementMode_t mode)
{
    return mode != SENSORARRAY_MEASUREMENT_MODE_NONE;
}

static int testAcceptedAckExactlyOneTerminal(void)
{
    uint32_t requestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("CCCCCCCC", &requestId) == ESP_OK);
    CHECK(requestId != 0u);
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 1u);
    CHECK(sensorarrayTransportGuaranteedTextIsReserved());

    /* Phase 1 only: Core 1 must not see an unacknowledged command. */
    sensorarrayCommand_t command;
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));

    CHECK(sensorarrayCommandMailboxCommitRowModesAck(requestId));
    CHECK(sensorarrayCommandMailboxTryReceive(&command));
    CHECK(command.type == SENSORARRAY_COMMAND_ROW_MODES);
    CHECK(command.requestId == requestId);
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));

    const char terminal[] = "RMAPP,id=1,state=applied\n";
    CHECK(sensorarrayCommandMailboxEmitRowModesTerminal(
              requestId, terminal, strlen(terminal)) == ESP_OK);
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 0u);
    CHECK(sensorarrayCommandMailboxEmitRowModesTerminal(
              requestId, terminal, strlen(terminal)) != ESP_OK);

    char drained[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
    size_t drainedLength = 0u;
    CHECK(sensorarrayTransportGuaranteedTextTakeDrain(
        drained, sizeof(drained), &drainedLength));
    CHECK(strcmp(drained, terminal) == 0);
    CHECK(!sensorarrayTransportGuaranteedTextTakeDrain(
        drained, sizeof(drained), &drainedLength));
    CHECK(!sensorarrayTransportGuaranteedTextIsReserved());
    return 0;
}

static int testFailedAckCancelsBeforeApplyAndReusesLane(void)
{
    uint32_t requestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("VVVVVVVV", &requestId) == ESP_OK);
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 1u);
    CHECK(sensorarrayTransportGuaranteedTextIsReserved());

    sensorarrayCommand_t command;
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));
    CHECK(sensorarrayCommandMailboxCancelRowModesAck(requestId));
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 0u);
    CHECK(!sensorarrayTransportGuaranteedTextIsReserved());
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));

    /* A canceled reservation can never be committed retroactively. */
    CHECK(!sensorarrayCommandMailboxCommitRowModesAck(requestId));

    uint32_t nextRequestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("RRRRRRRR", &nextRequestId) == ESP_OK);
    CHECK(nextRequestId != requestId);
    CHECK(sensorarrayCommandMailboxCancelRowModesAck(nextRequestId));
    return 0;
}

static int testUnknownRequestsDoNotTouchLane(void)
{
    uint32_t requestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("CCCCCCCC", &requestId) == ESP_OK);
    CHECK(!sensorarrayCommandMailboxCommitRowModesAck(requestId + 1u));
    CHECK(!sensorarrayCommandMailboxCancelRowModesAck(requestId + 1u));
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 1u);
    CHECK(sensorarrayTransportGuaranteedTextIsReserved());
    CHECK(sensorarrayCommandMailboxCancelRowModesAck(requestId));
    return 0;
}

static int testOversizeTerminalBoundedCancel(void)
{
    uint32_t requestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("CCCCCCCC", &requestId) == ESP_OK);
    CHECK(sensorarrayCommandMailboxCommitRowModesAck(requestId));
    sensorarrayCommand_t command;
    CHECK(sensorarrayCommandMailboxTryReceive(&command));

    char oversize[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
    memset(oversize, 'T', sizeof(oversize));
    CHECK(sensorarrayCommandMailboxEmitRowModesTerminal(
              requestId, oversize, sizeof(oversize)) == ESP_ERR_INVALID_SIZE);
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 0u);
    CHECK(!sensorarrayTransportGuaranteedTextIsReserved());

    /* The canceled lane is immediately reusable by the next transaction. */
    uint32_t nextRequestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("VVVVVVVV", &nextRequestId) == ESP_OK);
    CHECK(sensorarrayCommandMailboxCancelRowModesAck(nextRequestId));
    return 0;
}

static int testFullQueueFallsBackToReservationExactlyOnce(void)
{
    CHECK(uxQueueSpacesAvailable((QueueHandle_t)1) == TEST_QUEUE_LENGTH);
    for (size_t index = 0u; index < TEST_QUEUE_LENGTH; ++index) {
        CHECK(sensorarrayCommandMailboxPostText(
                  (const uint8_t *)"TRACE=1", 7u) == ESP_OK);
    }
    CHECK(uxQueueSpacesAvailable((QueueHandle_t)1) == 0u);

    uint32_t requestId = 0u;
    CHECK(sensorarrayCommandMailboxPostRowModes("CCCCCCCC", &requestId) == ESP_OK);
    CHECK(sensorarrayCommandMailboxCommitRowModesAck(requestId));

    for (size_t index = 0u; index < TEST_QUEUE_LENGTH; ++index) {
        sensorarrayCommand_t command;
        CHECK(sensorarrayCommandMailboxTryReceive(&command));
        CHECK(command.type == SENSORARRAY_COMMAND_TRACE_ENABLE);
    }

    /* The committed command is still deliverable once the shared queue is
     * drained, and only once. */
    sensorarrayCommand_t command;
    CHECK(sensorarrayCommandMailboxTryReceive(&command));
    CHECK(command.type == SENSORARRAY_COMMAND_ROW_MODES);
    CHECK(command.requestId == requestId);
    CHECK(!sensorarrayCommandMailboxTryReceive(&command));

    const char terminal[] = "RMAPP,id=9,state=applied\n";
    CHECK(sensorarrayCommandMailboxEmitRowModesTerminal(
              requestId, terminal, strlen(terminal)) == ESP_OK);
    CHECK(sensorarrayCommandMailboxRowModesOutstanding() == 0u);
    return 0;
}

int main(void)
{
    CHECK(sensorarrayCommandMailboxInit() == ESP_OK);
    CHECK(testAcceptedAckExactlyOneTerminal() == 0);
    CHECK(testFailedAckCancelsBeforeApplyAndReusesLane() == 0);
    CHECK(testUnknownRequestsDoNotTouchLane() == 0);
    CHECK(testOversizeTerminalBoundedCancel() == 0);
    CHECK(testFullQueueFallsBackToReservationExactlyOnce() == 0);
    printf("ROWMODES_ACK_GUARD,passed=1\n");
    return 0;
}
