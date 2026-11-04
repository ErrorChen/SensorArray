/* Synthetic contexts serialized by the production fault formatter. */
#include <stdio.h>
#include <string.h>
#include "sensorarrayAdsFault.h"

int main(void)
{
    const sensorarrayAdsFaultStage_t stages[] = {
        SENSORARRAY_ADS_FAULT_STAGE_REGISTER_HEALTH_READ,
        SENSORARRAY_ADS_FAULT_STAGE_REGISTER_READ_UNSTABLE,
        SENSORARRAY_ADS_FAULT_STAGE_POWER_COMPARE,
        SENSORARRAY_ADS_FAULT_STAGE_REFMUX_COMPARE,
        SENSORARRAY_ADS_FAULT_STAGE_MODE_COMPARE,
        SENSORARRAY_ADS_FAULT_STAGE_RESET_OBSERVED,
        SENSORARRAY_ADS_FAULT_STAGE_CACHE_COMPARE,
        SENSORARRAY_ADS_FAULT_STAGE_ROW_GPIO_COMPARE,
        SENSORARRAY_ADS_FAULT_STAGE_OWNER_ACQUIRE,
        SENSORARRAY_ADS_FAULT_STAGE_RESTORE_VERIFY,
    };
    for (size_t i = 0u; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        sensorarrayAdsMatrixFaultContext_t context = {
            .valid = true, .stage = stages[i],
            .originalError = i == 0u ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_RESPONSE,
            .cleanupError = i == 2u || i == 9u ? ESP_ERR_INVALID_STATE : ESP_OK,
            .mode = "VOLT", .route = "VOLT", .owner = "MATRIX",
            .engineFrameCount = 128u, .modeEntryFrameCount = 65u,
            .healthInterval = 64u, .healthReadbackDue = true,
            .precisionFrame = true, .registerId = 0u,
            .expected = 2u, .actual = i == 5u ? 0x12u : 3u,
            .compareMask = i == 5u ? 0x10u : 0x03u,
            .cacheGeneration = 14u, .physicalGeneration = 29u, .configGeneration = 42u,
            .rail = {.id = 4u, .startedUs = 1000000u, .completedUs = 1004900u,
                     .firstValidFrameUs = 1018200u},
            .battery = {.id = 19u, .startedUs = 2000000u, .completedUs = 2005600u,
                        .originalError = ESP_ERR_INVALID_RESPONSE},
            .check = {.id = 2u, .startedUs = 3000000u, .completedUs = 3001000u},
        };
        if (i == 0u) { context.compareMask = 0u; context.expected = context.actual = 0u; }
        if (i == 1u) { context.registerId = 3u; context.expected = 0u; context.actual = 1u; context.compareMask = 0xFFu; }
        if (i == 3u || i == 9u) { context.registerId = 6u; context.expected = 0x24u; context.actual = 0u; context.compareMask = 0x3Fu; }
        if (i == 4u) { context.registerId = 2u; context.expected = 0u; context.actual = 1u; context.compareMask = 0xFFu; }
        if (i == 5u) { context.expected = 0u; }
        if (i == 6u) { context.registerId = 5u; context.expected = 0x08u; context.actual = 0x18u; context.compareMask = 0xFFu; }
        if (i == 7u) { context.registerId = 1u; context.expected = 0u; context.actual = 1u; context.compareMask = 0x07u; }
        if (i == 8u) { context.expected = 0u; context.actual = 3u; context.compareMask = 0xFFu; }
        sensorarrayAdsFaultEvent_t event = {
            /* Deliberately post-cleanup values: captured context must win. */
            .stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE,
            .err = ESP_ERR_INVALID_RESPONSE,
            .bootId = 123456u, .bootCount = 7u, .seq = 512u,
            .modeGeneration = 4u, .profileGeneration = 3u,
            .rowGeneration = 2u, .rowRequestId = 1u,
            .mode = "NONE", .route = "SAFE", .owner = "NONE",
            .profile = "VVVVVVVV", .drdyGeneration = 100u, .configGeneration = 42u,
            .railSpanUv = 5200000, .avddUv = 3400000, .avssUv = -1800000,
            .railSource = "external_calibration", .railValid = true,
            .reference = "avdd-avss", .context = &context,
        };
        char text[SENSORARRAY_ADS_FAULT_LINE_MAX + 1u];
        size_t length = sensorarrayAdsFaultFormat(&event, text, sizeof(text));
        if (!length || length != strlen(text)) { return 1; }
        printf("# fixture=%s; synthetic=1; producer=sensorarrayAdsFaultFormat; not_field_capture=1\n",
            sensorarrayAdsFaultStageName(stages[i]));
        fwrite(text, 1u, length, stdout);
    }
    return 0;
}
