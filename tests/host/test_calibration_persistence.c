#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sensorarrayCalibration.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while (0)

#define TEST_PAYLOAD_BYTES 40u

static uint8_t s_testBlob[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
static size_t s_testBlobLength;
static bool s_testBlobPresent;

esp_err_t nvs_flash_init_partition(const char *partitionLabel)
{
    (void)partitionLabel;
    return ESP_OK;
}

esp_err_t nvs_flash_erase_partition(const char *partName)
{
    (void)partName;
    s_testBlobPresent = false;
    s_testBlobLength = 0u;
    memset(s_testBlob, 0, sizeof(s_testBlob));
    return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char *partName,
                                  const char *namespaceName,
                                  nvs_open_mode_t openMode,
                                  nvs_handle_t *outHandle)
{
    (void)openMode;
    if (!partName || !namespaceName || !outHandle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(partName, SENSORARRAY_CALIBRATION_PARTITION_NAME) != 0 ||
        strcmp(namespaceName, SENSORARRAY_CALIBRATION_NAMESPACE) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    *outHandle = 1u;
    return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t handle,
                       const char *key,
                       const void *value,
                       size_t length)
{
    (void)handle;
    if (!key || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(key, SENSORARRAY_CALIBRATION_STORAGE_KEY) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    if (length == 0u || length > sizeof(s_testBlob)) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(s_testBlob, value, length);
    s_testBlobLength = length;
    s_testBlobPresent = true;
    return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t handle,
                       const char *key,
                       void *outValue,
                       size_t *length)
{
    (void)handle;
    if (!key || !length) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(key, SENSORARRAY_CALIBRATION_STORAGE_KEY) != 0) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!s_testBlobPresent) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (!outValue) {
        *length = s_testBlobLength;
        return ESP_OK;
    }
    if (*length < s_testBlobLength) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(outValue, s_testBlob, s_testBlobLength);
    *length = s_testBlobLength;
    return ESP_OK;
}

esp_err_t nvs_commit(nvs_handle_t handle)
{
    (void)handle;
    return ESP_OK;
}

void nvs_close(nvs_handle_t handle)
{
    (void)handle;
}

static void testResetBlob(void)
{
    s_testBlobPresent = false;
    s_testBlobLength = 0u;
    memset(s_testBlob, 0, sizeof(s_testBlob));
}

static void testWriteU32Le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8u) & 0xFFu);
    out[2] = (uint8_t)((value >> 16u) & 0xFFu);
    out[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static void testFillPayload(uint8_t *payload, size_t length, uint8_t seed)
{
    for (size_t index = 0u; index < length; ++index) {
        payload[index] = (uint8_t)(seed + (index * 3u));
    }
}

static bool testPayloadValid(const void *payload, size_t payloadLength)
{
    return payload != NULL && payloadLength == TEST_PAYLOAD_BYTES;
}

static bool testPayloadRejected(const void *payload, size_t payloadLength)
{
    (void)payload;
    (void)payloadLength;
    return false;
}

static esp_err_t testStoreValidRecord(const uint8_t *payload,
                                      size_t payloadLength)
{
    uint8_t record[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
    size_t recordLength = 0u;
    esp_err_t err = sensorarrayCalibrationRecordEncode(
        SENSORARRAY_CALIBRATION_SOURCE_PERSISTED,
        payload,
        payloadLength,
        record,
        sizeof(record),
        &recordLength);
    if (err == ESP_OK) {
        testResetBlob();
        memcpy(s_testBlob, record, recordLength);
        s_testBlobLength = recordLength;
        s_testBlobPresent = true;
    }
    return err;
}

static void testRefreshBlobCrc(void)
{
    uint32_t crc = sensorarrayCalibrationCrc32(
        s_testBlob,
        s_testBlobLength - SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES);
    testWriteU32Le(s_testBlob +
                   s_testBlobLength -
                   SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES,
                   crc);
}

static int testCrcKnownVector(void)
{
    const uint8_t data[] = "123456789";
    CHECK(sensorarrayCalibrationCrc32(data, 9u) == 0xCBF43926u);
    return 0;
}

static int testEncodeDecodeRoundTrip(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    uint8_t decoded[TEST_PAYLOAD_BYTES];
    uint8_t record[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
    size_t recordLength = 0u;
    testFillPayload(payload, sizeof(payload), 0x21u);
    memset(decoded, 0, sizeof(decoded));
    CHECK(sensorarrayCalibrationRecordEncode(
              SENSORARRAY_CALIBRATION_SOURCE_PERSISTED,
              payload,
              sizeof(payload),
              record,
              sizeof(record),
              &recordLength) == ESP_OK);
    CHECK(recordLength == SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                          sizeof(payload) +
                          SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES);
    sensorarrayCalibrationRecordInfo_t info = {0};
    CHECK(sensorarrayCalibrationRecordDecode(record,
                                             recordLength,
                                             &info,
                                             decoded,
                                             sizeof(decoded)) == ESP_OK);
    CHECK(info.magic == SENSORARRAY_CALIBRATION_MAGIC);
    CHECK(info.schemaVersion == SENSORARRAY_CALIBRATION_SCHEMA_VERSION);
    CHECK(info.boardId == SENSORARRAY_CALIBRATION_BOARD_ID);
    CHECK(info.hardwareRevision == SENSORARRAY_CALIBRATION_HARDWARE_REVISION);
    CHECK(info.source == SENSORARRAY_CALIBRATION_SOURCE_PERSISTED);
    CHECK(info.payloadLength == sizeof(payload));
    CHECK(memcmp(decoded, payload, sizeof(payload)) == 0);
    return 0;
}

static int testSaveThenLoadRoundTrip(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    uint8_t loaded[TEST_PAYLOAD_BYTES];
    size_t loadedLength = 0u;
    sensorarrayCalibrationStatus_t status = {0};
    testResetBlob();
    testFillPayload(payload, sizeof(payload), 0x44u);
    memset(loaded, 0, sizeof(loaded));
    CHECK(sensorarrayCalibrationSave(payload,
                                     sizeof(payload),
                                     &status) == ESP_OK);
    CHECK(status.valid);
    CHECK(status.source == SENSORARRAY_CALIBRATION_SOURCE_PERSISTED);
    CHECK(status.schemaVersion == SENSORARRAY_CALIBRATION_SCHEMA_VERSION);
    CHECK(status.boardId == SENSORARRAY_CALIBRATION_BOARD_ID);
    CHECK(status.hardwareRevision == SENSORARRAY_CALIBRATION_HARDWARE_REVISION);
    CHECK(status.payloadLength == sizeof(payload));
    memset(&status, 0, sizeof(status));
    CHECK(sensorarrayCalibrationLoad(testPayloadValid,
                                     loaded,
                                     sizeof(loaded),
                                     &loadedLength,
                                     &status) == ESP_OK);
    CHECK(loadedLength == sizeof(payload));
    CHECK(memcmp(loaded, payload, sizeof(payload)) == 0);
    CHECK(status.valid);
    CHECK(status.source == SENSORARRAY_CALIBRATION_SOURCE_PERSISTED);
    CHECK(status.boardId == SENSORARRAY_CALIBRATION_BOARD_ID);
    return 0;
}

static int testDefaultFallbackWhenMissing(void)
{
    uint8_t fallback[TEST_PAYLOAD_BYTES];
    uint8_t loaded[TEST_PAYLOAD_BYTES];
    size_t loadedLength = 99u;
    sensorarrayCalibrationStatus_t status = {0};
    testResetBlob();
    testFillPayload(fallback, sizeof(fallback), 0xAAu);
    memcpy(loaded, fallback, sizeof(loaded));
    CHECK(sensorarrayCalibrationLoad(testPayloadValid,
                                     loaded,
                                     sizeof(loaded),
                                     &loadedLength,
                                     &status) == ESP_ERR_NOT_FOUND);
    CHECK(memcmp(loaded, fallback, sizeof(loaded)) == 0);
    CHECK(loadedLength == 0u);
    CHECK(!status.valid);
    CHECK(status.source == SENSORARRAY_CALIBRATION_SOURCE_UNKNOWN);
    CHECK(status.schemaVersion == 0u);
    CHECK(status.boardId == 0u);
    return 0;
}

static int testCrcMismatchFallsBackToDefault(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    uint8_t fallback[TEST_PAYLOAD_BYTES];
    uint8_t loaded[TEST_PAYLOAD_BYTES];
    sensorarrayCalibrationStatus_t status = {0};
    testFillPayload(payload, sizeof(payload), 0x55u);
    CHECK(testStoreValidRecord(payload, sizeof(payload)) == ESP_OK);
    s_testBlob[SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES] ^= 0xFFu;
    testFillPayload(fallback, sizeof(fallback), 0xBBu);
    memcpy(loaded, fallback, sizeof(loaded));
    CHECK(sensorarrayCalibrationLoad(testPayloadValid,
                                     loaded,
                                     sizeof(loaded),
                                     NULL,
                                     &status) == ESP_ERR_INVALID_STATE);
    CHECK(memcmp(loaded, fallback, sizeof(loaded)) == 0);
    CHECK(!status.valid);
    CHECK(status.boardId == SENSORARRAY_CALIBRATION_BOARD_ID);
    return 0;
}

static int testSchemaMismatchFallsBackToDefault(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    uint8_t fallback[TEST_PAYLOAD_BYTES];
    uint8_t loaded[TEST_PAYLOAD_BYTES];
    sensorarrayCalibrationStatus_t status = {0};
    testFillPayload(payload, sizeof(payload), 0x66u);
    CHECK(testStoreValidRecord(payload, sizeof(payload)) == ESP_OK);
    testWriteU32Le(s_testBlob + 4u, SENSORARRAY_CALIBRATION_SCHEMA_VERSION + 99u);
    testRefreshBlobCrc();
    testFillPayload(fallback, sizeof(fallback), 0xCCu);
    memcpy(loaded, fallback, sizeof(loaded));
    CHECK(sensorarrayCalibrationLoad(testPayloadValid,
                                     loaded,
                                     sizeof(loaded),
                                     NULL,
                                     &status) == ESP_ERR_INVALID_STATE);
    CHECK(memcmp(loaded, fallback, sizeof(loaded)) == 0);
    CHECK(!status.valid);
    CHECK(status.schemaVersion ==
          SENSORARRAY_CALIBRATION_SCHEMA_VERSION + 99u);
    return 0;
}

static int testBoardMismatchFallsBackToDefault(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    uint8_t fallback[TEST_PAYLOAD_BYTES];
    uint8_t loaded[TEST_PAYLOAD_BYTES];
    sensorarrayCalibrationStatus_t status = {0};
    testFillPayload(payload, sizeof(payload), 0x77u);
    CHECK(testStoreValidRecord(payload, sizeof(payload)) == ESP_OK);
    testWriteU32Le(s_testBlob + 8u, 0xDEADBEEFu);
    testRefreshBlobCrc();
    testFillPayload(fallback, sizeof(fallback), 0xDDu);
    memcpy(loaded, fallback, sizeof(loaded));
    CHECK(sensorarrayCalibrationLoad(testPayloadValid,
                                     loaded,
                                     sizeof(loaded),
                                     NULL,
                                     &status) == ESP_ERR_INVALID_STATE);
    CHECK(memcmp(loaded, fallback, sizeof(loaded)) == 0);
    CHECK(!status.valid);
    CHECK(status.boardId == 0xDEADBEEFu);
    return 0;
}

static int testPayloadValidationFallsBackToDefault(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    uint8_t fallback[TEST_PAYLOAD_BYTES];
    uint8_t loaded[TEST_PAYLOAD_BYTES];
    sensorarrayCalibrationStatus_t status = {0};
    testFillPayload(payload, sizeof(payload), 0x88u);
    CHECK(testStoreValidRecord(payload, sizeof(payload)) == ESP_OK);
    testFillPayload(fallback, sizeof(fallback), 0xEEu);
    memcpy(loaded, fallback, sizeof(loaded));
    CHECK(sensorarrayCalibrationLoad(testPayloadRejected,
                                     loaded,
                                     sizeof(loaded),
                                     NULL,
                                     &status) == ESP_ERR_INVALID_STATE);
    CHECK(memcmp(loaded, fallback, sizeof(loaded)) == 0);
    CHECK(!status.valid);
    return 0;
}

static int testQueryStatuses(void)
{
    uint8_t payload[TEST_PAYLOAD_BYTES];
    sensorarrayCalibrationStatus_t status = {0};
    testResetBlob();
    CHECK(sensorarrayCalibrationQuery(&status) == ESP_OK);
    CHECK(!status.valid);
    CHECK(status.source == SENSORARRAY_CALIBRATION_SOURCE_UNKNOWN);
    CHECK(status.schemaVersion == 0u);
    CHECK(status.boardId == 0u);

    testFillPayload(payload, sizeof(payload), 0x99u);
    CHECK(sensorarrayCalibrationSave(payload,
                                     sizeof(payload),
                                     &status) == ESP_OK);
    CHECK(sensorarrayCalibrationQuery(&status) == ESP_OK);
    CHECK(status.valid);
    CHECK(status.source == SENSORARRAY_CALIBRATION_SOURCE_PERSISTED);
    CHECK(status.schemaVersion == SENSORARRAY_CALIBRATION_SCHEMA_VERSION);
    CHECK(status.boardId == SENSORARRAY_CALIBRATION_BOARD_ID);
    CHECK(status.payloadLength == sizeof(payload));

    testWriteU32Le(s_testBlob + 8u, 0x12345678u);
    testRefreshBlobCrc();
    CHECK(sensorarrayCalibrationQuery(&status) == ESP_OK);
    CHECK(!status.valid);
    CHECK(status.boardId == 0x12345678u);
    return 0;
}

static int testOversizeAndMalformedRejected(void)
{
    uint8_t payload[SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES + 1u];
    sensorarrayCalibrationStatus_t status = {0};
    testResetBlob();
    memset(payload, 0, sizeof(payload));
    CHECK(sensorarrayCalibrationSave(payload,
                                     sizeof(payload),
                                     &status) == ESP_ERR_INVALID_ARG);

    uint8_t record[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
    size_t recordLength = 0u;
    sensorarrayCalibrationRecordInfo_t info = {0};
    CHECK(sensorarrayCalibrationRecordEncode(
              SENSORARRAY_CALIBRATION_SOURCE_PERSISTED,
              payload,
              SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES,
              record,
              sizeof(record),
              &recordLength) == ESP_OK);
    CHECK(sensorarrayCalibrationRecordDecode(record,
                                             recordLength - 1u,
                                             &info,
                                             NULL,
                                             0u) == ESP_ERR_INVALID_SIZE);
    testWriteU32Le(record, 0x01020304u);
    memcpy(s_testBlob, record, recordLength);
    s_testBlobLength = recordLength;
    s_testBlobPresent = true;
    testRefreshBlobCrc();
    CHECK(sensorarrayCalibrationRecordDecode(s_testBlob,
                                             s_testBlobLength,
                                             &info,
                                             NULL,
                                             0u) == ESP_ERR_INVALID_STATE);
    CHECK(info.magic == 0x01020304u);
    return 0;
}

int main(void)
{
    int result = 0;
    result |= testCrcKnownVector();
    result |= testEncodeDecodeRoundTrip();
    result |= testSaveThenLoadRoundTrip();
    result |= testDefaultFallbackWhenMissing();
    result |= testCrcMismatchFallsBackToDefault();
    result |= testSchemaMismatchFallsBackToDefault();
    result |= testBoardMismatchFallsBackToDefault();
    result |= testPayloadValidationFallsBackToDefault();
    result |= testQueryStatuses();
    result |= testOversizeAndMalformedRejected();
    if (result != 0) {
        return result;
    }
    printf("CALIBRATION_TESTS,passed=1,maxPayloadBytes=%u,recordBytes=%u\n",
           (unsigned)SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES,
           (unsigned)SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES);
    return 0;
}
