#include "sensorarrayCalibration.h"

#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

static void sensorarrayCalibrationWriteU32Le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8u) & 0xFFu);
    out[2] = (uint8_t)((value >> 16u) & 0xFFu);
    out[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

static uint32_t sensorarrayCalibrationReadU32Le(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0]) |
           ((uint32_t)bytes[1] << 8u) |
           ((uint32_t)bytes[2] << 16u) |
           ((uint32_t)bytes[3] << 24u);
}

uint32_t sensorarrayCalibrationCrc32(const uint8_t *bytes, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t index = 0u; index < length; ++index) {
        crc ^= bytes[index];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1u) ^
                  ((crc & 1u) != 0u ? 0xEDB88320u : 0u);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

esp_err_t sensorarrayCalibrationRecordEncode(uint32_t source,
                                             const void *payload,
                                             size_t payloadLength,
                                             uint8_t *outBytes,
                                             size_t outCapacity,
                                             size_t *outLength)
{
    if (!payload || payloadLength == 0u ||
        payloadLength > SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES ||
        !outBytes || !outLength) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t recordLength = SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                          payloadLength +
                          SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES;
    if (outCapacity < recordLength) {
        return ESP_ERR_INVALID_SIZE;
    }

    sensorarrayCalibrationWriteU32Le(outBytes, SENSORARRAY_CALIBRATION_MAGIC);
    sensorarrayCalibrationWriteU32Le(outBytes + 4u,
                                     SENSORARRAY_CALIBRATION_SCHEMA_VERSION);
    sensorarrayCalibrationWriteU32Le(outBytes + 8u,
                                     SENSORARRAY_CALIBRATION_BOARD_ID);
    sensorarrayCalibrationWriteU32Le(outBytes + 12u,
                                     SENSORARRAY_CALIBRATION_HARDWARE_REVISION);
    sensorarrayCalibrationWriteU32Le(outBytes + 16u, source);
    sensorarrayCalibrationWriteU32Le(outBytes + 20u, (uint32_t)payloadLength);
    memcpy(outBytes + SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES,
           payload,
           payloadLength);
    sensorarrayCalibrationWriteU32Le(
        outBytes + SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES + payloadLength,
        sensorarrayCalibrationCrc32(outBytes,
                                    SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                                    payloadLength));
    *outLength = recordLength;
    return ESP_OK;
}

static bool sensorarrayCalibrationRecordReadInfo(
    const uint8_t *bytes,
    size_t length,
    sensorarrayCalibrationRecordInfo_t *outInfo,
    size_t *outPayloadLength)
{
    if (!bytes || !outInfo ||
        length < SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                 SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES) {
        return false;
    }
    uint32_t payloadLength = sensorarrayCalibrationReadU32Le(bytes + 20u);
    if (payloadLength > SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES ||
        length != SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                  payloadLength +
                  SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES) {
        return false;
    }
    *outInfo = (sensorarrayCalibrationRecordInfo_t){
        .magic = sensorarrayCalibrationReadU32Le(bytes),
        .source = sensorarrayCalibrationReadU32Le(bytes + 16u),
        .schemaVersion = sensorarrayCalibrationReadU32Le(bytes + 4u),
        .boardId = sensorarrayCalibrationReadU32Le(bytes + 8u),
        .hardwareRevision = sensorarrayCalibrationReadU32Le(bytes + 12u),
        .payloadLength = payloadLength,
    };
    if (outPayloadLength) {
        *outPayloadLength = payloadLength;
    }
    return true;
}

esp_err_t sensorarrayCalibrationRecordDecode(const uint8_t *bytes,
                                             size_t length,
                                             sensorarrayCalibrationRecordInfo_t *outInfo,
                                             void *outPayload,
                                             size_t payloadCapacity)
{
    if (!bytes || !outInfo) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(outInfo, 0, sizeof(*outInfo));

    size_t payloadLength = 0u;
    if (!sensorarrayCalibrationRecordReadInfo(bytes,
                                              length,
                                              outInfo,
                                              &payloadLength)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (outInfo->magic != SENSORARRAY_CALIBRATION_MAGIC ||
        outInfo->schemaVersion != SENSORARRAY_CALIBRATION_SCHEMA_VERSION) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t recordLength = SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                          payloadLength +
                          SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES;
    uint32_t storedCrc = sensorarrayCalibrationReadU32Le(
        bytes + recordLength - SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES);
    if (storedCrc != sensorarrayCalibrationCrc32(bytes, recordLength -
                                                 SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (outPayload) {
        if (payloadLength > payloadCapacity) {
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(outPayload,
               bytes + SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES,
               payloadLength);
    }
    return ESP_OK;
}

esp_err_t sensorarrayCalibrationInit(void)
{
    esp_err_t err = nvs_flash_init_partition(
        SENSORARRAY_CALIBRATION_PARTITION_NAME);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Recovery is scoped to the dedicated calib partition; generic
         * nvs_flash_erase() never touches this partition. */
        err = nvs_flash_erase_partition(
            SENSORARRAY_CALIBRATION_PARTITION_NAME);
        if (err == ESP_OK) {
            err = nvs_flash_init_partition(
                SENSORARRAY_CALIBRATION_PARTITION_NAME);
        }
    }
    return err;
}

static void sensorarrayCalibrationStatusFromInfo(
    sensorarrayCalibrationStatus_t *status,
    const sensorarrayCalibrationRecordInfo_t *info)
{
    if (!status || !info) {
        return;
    }
    *status = (sensorarrayCalibrationStatus_t){
        .valid = false,
        .source = info->source,
        .schemaVersion = info->schemaVersion,
        .boardId = info->boardId,
        .hardwareRevision = info->hardwareRevision,
        .payloadLength = info->payloadLength,
    };
}

static esp_err_t sensorarrayCalibrationReadRecord(uint8_t *outRecord,
                                                  size_t capacity,
                                                  size_t *outLength)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open_from_partition(
        SENSORARRAY_CALIBRATION_PARTITION_NAME,
        SENSORARRAY_CALIBRATION_NAMESPACE,
        NVS_READONLY,
        &handle);
    if (err != ESP_OK) {
        return err;
    }
    size_t length = 0u;
    err = nvs_get_blob(handle,
                       SENSORARRAY_CALIBRATION_STORAGE_KEY,
                       NULL,
                       &length);
    if (err == ESP_OK) {
        if (length == 0u || length > capacity) {
            err = ESP_ERR_INVALID_SIZE;
        } else {
            err = nvs_get_blob(handle,
                               SENSORARRAY_CALIBRATION_STORAGE_KEY,
                               outRecord,
                               &length);
        }
    }
    nvs_close(handle);
    if (outLength) {
        *outLength = err == ESP_OK ? length : 0u;
    }
    return err;
}

esp_err_t sensorarrayCalibrationSave(const void *payload,
                                     size_t payloadLength,
                                     sensorarrayCalibrationStatus_t *outStatus)
{
    if (outStatus) {
        memset(outStatus, 0, sizeof(*outStatus));
    }
    if (!payload || payloadLength == 0u ||
        payloadLength > SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = sensorarrayCalibrationInit();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t record[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
    size_t recordLength = 0u;
    err = sensorarrayCalibrationRecordEncode(
        SENSORARRAY_CALIBRATION_SOURCE_PERSISTED,
        payload,
        payloadLength,
        record,
        sizeof(record),
        &recordLength);
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open_from_partition(SENSORARRAY_CALIBRATION_PARTITION_NAME,
                                  SENSORARRAY_CALIBRATION_NAMESPACE,
                                  NVS_READWRITE,
                                  &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle,
                       SENSORARRAY_CALIBRATION_STORAGE_KEY,
                       record,
                       recordLength);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK && outStatus) {
        *outStatus = (sensorarrayCalibrationStatus_t){
            .valid = true,
            .source = SENSORARRAY_CALIBRATION_SOURCE_PERSISTED,
            .schemaVersion = SENSORARRAY_CALIBRATION_SCHEMA_VERSION,
            .boardId = SENSORARRAY_CALIBRATION_BOARD_ID,
            .hardwareRevision = SENSORARRAY_CALIBRATION_HARDWARE_REVISION,
            .payloadLength = payloadLength,
        };
    }
    return err;
}

esp_err_t sensorarrayCalibrationLoad(sensorarrayCalibrationPayloadValidateFn validatePayload,
                                     void *outPayload,
                                     size_t payloadCapacity,
                                     size_t *outPayloadLength,
                                     sensorarrayCalibrationStatus_t *outStatus)
{
    if (outStatus) {
        memset(outStatus, 0, sizeof(*outStatus));
    }
    if (outPayloadLength) {
        *outPayloadLength = 0u;
    }
    if (!validatePayload || !outPayload || payloadCapacity == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayCalibrationInit();
    if (err != ESP_OK) {
        return err;
    }
    uint8_t record[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
    size_t recordLength = 0u;
    err = sensorarrayCalibrationReadRecord(record,
                                           sizeof(record),
                                           &recordLength);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayCalibrationRecordInfo_t info = {0};
    err = sensorarrayCalibrationRecordDecode(record,
                                             recordLength,
                                             &info,
                                             NULL,
                                             0u);
    if (err != ESP_OK) {
        sensorarrayCalibrationStatusFromInfo(outStatus, &info);
        return ESP_ERR_INVALID_STATE;
    }
    if (info.boardId != SENSORARRAY_CALIBRATION_BOARD_ID ||
        info.hardwareRevision != SENSORARRAY_CALIBRATION_HARDWARE_REVISION) {
        sensorarrayCalibrationStatusFromInfo(outStatus, &info);
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t stagedPayload[SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES];
    err = sensorarrayCalibrationRecordDecode(record,
                                             recordLength,
                                             &info,
                                             stagedPayload,
                                             sizeof(stagedPayload));
    if (err != ESP_OK) {
        sensorarrayCalibrationStatusFromInfo(outStatus, &info);
        return err;
    }
    if (!validatePayload(stagedPayload, info.payloadLength)) {
        sensorarrayCalibrationStatusFromInfo(outStatus, &info);
        return ESP_ERR_INVALID_STATE;
    }
    if (info.payloadLength > payloadCapacity) {
        sensorarrayCalibrationStatusFromInfo(outStatus, &info);
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(outPayload, stagedPayload, info.payloadLength);
    if (outStatus) {
        *outStatus = (sensorarrayCalibrationStatus_t){
            .valid = true,
            .source = info.source,
            .schemaVersion = info.schemaVersion,
            .boardId = info.boardId,
            .hardwareRevision = info.hardwareRevision,
            .payloadLength = info.payloadLength,
        };
    }
    if (outPayloadLength) {
        *outPayloadLength = info.payloadLength;
    }
    return ESP_OK;
}

esp_err_t sensorarrayCalibrationQuery(sensorarrayCalibrationStatus_t *outStatus)
{
    if (!outStatus) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(outStatus, 0, sizeof(*outStatus));
    esp_err_t err = sensorarrayCalibrationInit();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t record[SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES];
    size_t recordLength = 0u;
    err = sensorarrayCalibrationReadRecord(record,
                                           sizeof(record),
                                           &recordLength);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayCalibrationRecordInfo_t info = {0};
    (void)sensorarrayCalibrationRecordDecode(record,
                                             recordLength,
                                             &info,
                                             NULL,
                                             0u);
    sensorarrayCalibrationStatusFromInfo(outStatus, &info);
    if (info.magic == SENSORARRAY_CALIBRATION_MAGIC &&
        info.schemaVersion == SENSORARRAY_CALIBRATION_SCHEMA_VERSION &&
        info.boardId == SENSORARRAY_CALIBRATION_BOARD_ID &&
        info.hardwareRevision == SENSORARRAY_CALIBRATION_HARDWARE_REVISION &&
        recordLength == SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES +
                        info.payloadLength +
                        SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES) {
        uint32_t storedCrc = sensorarrayCalibrationReadU32Le(
            record + recordLength - SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES);
        if (storedCrc == sensorarrayCalibrationCrc32(
                             record,
                             recordLength -
                             SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES)) {
            outStatus->valid = true;
        }
    }
    return ESP_OK;
}
