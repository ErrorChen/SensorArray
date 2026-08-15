#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_CALIBRATION_PARTITION_NAME "calib"
#define SENSORARRAY_CALIBRATION_NAMESPACE "calib"
#define SENSORARRAY_CALIBRATION_STORAGE_KEY "record"

/* Versioned calibration record identity.  The record is stored as:
 * magic | schemaVersion | boardId | hardwareRevision | source |
 * payloadLength | payload | CRC32.  All integers are little-endian.
 *
 * There is no board revision source on this firmware, so the record identity
 * is a fixed documented constant until a board revision API is added. */
#define SENSORARRAY_CALIBRATION_MAGIC 0x53415243u      /* "SARC" */
#define SENSORARRAY_CALIBRATION_SCHEMA_VERSION 1u
#define SENSORARRAY_CALIBRATION_BOARD_ID 0x53415231u   /* "SAR1" */
#define SENSORARRAY_CALIBRATION_HARDWARE_REVISION 1u

#define SENSORARRAY_CALIBRATION_SOURCE_UNKNOWN 0u
#define SENSORARRAY_CALIBRATION_SOURCE_PERSISTED 1u

#define SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES 512u
#define SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES 24u
#define SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES 4u
#define SENSORARRAY_CALIBRATION_RECORD_MAX_BYTES \
    (SENSORARRAY_CALIBRATION_RECORD_HEADER_BYTES + \
     SENSORARRAY_CALIBRATION_MAX_PAYLOAD_BYTES + \
     SENSORARRAY_CALIBRATION_RECORD_CRC_BYTES)

typedef struct {
    uint32_t magic;
    uint32_t source;
    uint32_t schemaVersion;
    uint32_t boardId;
    uint32_t hardwareRevision;
    uint32_t payloadLength;
} sensorarrayCalibrationRecordInfo_t;

typedef struct {
    bool valid;
    uint32_t source;
    uint32_t schemaVersion;
    uint32_t boardId;
    uint32_t hardwareRevision;
    uint32_t payloadLength;
} sensorarrayCalibrationStatus_t;

typedef bool (*sensorarrayCalibrationPayloadValidateFn)(const void *payload,
                                                        size_t payloadLength);

uint32_t sensorarrayCalibrationCrc32(const uint8_t *bytes, size_t length);

esp_err_t sensorarrayCalibrationRecordEncode(uint32_t source,
                                             const void *payload,
                                             size_t payloadLength,
                                             uint8_t *outBytes,
                                             size_t outCapacity,
                                             size_t *outLength);

/* Decodes and CRC-validates a record.  outInfo is always filled when the
 * byte length and payloadLength are structurally readable, including when
 * magic/schema/CRC validation fails, so callers can report the stored
 * identity.  outPayload may be NULL when only the header is needed. */
esp_err_t sensorarrayCalibrationRecordDecode(const uint8_t *bytes,
                                             size_t length,
                                             sensorarrayCalibrationRecordInfo_t *outInfo,
                                             void *outPayload,
                                             size_t payloadCapacity);

esp_err_t sensorarrayCalibrationInit(void);
esp_err_t sensorarrayCalibrationSave(const void *payload,
                                     size_t payloadLength,
                                     sensorarrayCalibrationStatus_t *outStatus);
esp_err_t sensorarrayCalibrationLoad(sensorarrayCalibrationPayloadValidateFn validatePayload,
                                     void *outPayload,
                                     size_t payloadCapacity,
                                     size_t *outPayloadLength,
                                     sensorarrayCalibrationStatus_t *outStatus);
esp_err_t sensorarrayCalibrationQuery(sensorarrayCalibrationStatus_t *outStatus);

#ifdef __cplusplus
}
#endif
