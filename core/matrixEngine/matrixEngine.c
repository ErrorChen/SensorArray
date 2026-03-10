#include "matrixEngine.h"

#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef CONFIG_MATRIX_ROWS
#define CONFIG_MATRIX_ROWS 1
#endif

#ifndef CONFIG_MATRIX_COLS
#define CONFIG_MATRIX_COLS 1
#endif

#ifndef CONFIG_MATRIX_OVERSAMPLE
#define CONFIG_MATRIX_OVERSAMPLE 1
#endif

#define MATRIX_ENGINE_DEFAULT_GROUP_SIZE 4u
#define MATRIX_ENGINE_DEFAULT_CAP_GROUP_SIZE 4u
#define MATRIX_ENGINE_DEFAULT_COLS 8u

#define MATRIX_ENGINE_ADS_MUX_AIN0 0x0u
#define MATRIX_ENGINE_ADS_MUX_AINCOM 0xAu

typedef struct {
    matrixEngineConfig_t cfg;
    bool inited;
    bool useDefaultRow;
    bool useDefaultCol;
    bool useDefaultDrive;
    matrixEngineMeasure_t activeMeasure;
    SemaphoreHandle_t lock;
} matrixEngineState_t;

static matrixEngineState_t s_state = {0};

static esp_err_t matrixEngineSelectRowDefault(uint16_t row, void *userCtx)
{
    (void)userCtx;
    if (row > UINT8_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    return tmuxSwitchSelectRow((uint8_t)row);
}

static esp_err_t matrixEngineSelectColGroupDefault(uint16_t group, void *userCtx)
{
    (void)userCtx;

    bool enable = false;
    switch (s_state.activeMeasure) {
    case MATRIX_ENGINE_MEASURE_CAP_RAW:
        enable = true;
        break;
    case MATRIX_ENGINE_MEASURE_VOLTAGE_UV:
    case MATRIX_ENGINE_MEASURE_RESISTANCE_MOHM:
        enable = false;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    switch (group) {
    case 0:
        return tmuxSwitchSetSelAEnabled(enable);
    case 1:
        return tmuxSwitchSetSelBEnabled(enable);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t matrixEngineDriveDefault(tmux1108Source_t source, void *userCtx)
{
    (void)userCtx;
    return tmuxSwitchSet1108Source(source);
}

static uint16_t matrixEngineGroupSize(void)
{
    uint16_t groupSize = s_state.cfg.colGroupSize;
    if (groupSize == 0) {
        groupSize = MATRIX_ENGINE_DEFAULT_GROUP_SIZE;
    }
    return groupSize;
}

static uint8_t matrixEngineOversample(void)
{
    uint8_t oversample = s_state.cfg.oversample;
    if (oversample == 0) {
        oversample = (uint8_t)CONFIG_MATRIX_OVERSAMPLE;
    }
    return oversample == 0 ? 1 : oversample;
}

static bool matrixEngineDefaultAdcMuxForCol(uint16_t col, uint8_t *muxp, uint8_t *muxn)
{
    if (!muxp || !muxn || col >= MATRIX_ENGINE_DEFAULT_COLS) {
        return false;
    }

    // Canonical board map: D1..D8 align to AIN0..AIN7.
    *muxp = (uint8_t)(MATRIX_ENGINE_ADS_MUX_AIN0 + col);
    *muxn = MATRIX_ENGINE_ADS_MUX_AINCOM;
    return true;
}

static bool matrixEngineRegionValid(const matrixEngineRegion_t *region)
{
    if (!region || region->rows == 0 || region->cols == 0) {
        return false;
    }
    if (region->row >= CONFIG_MATRIX_ROWS || region->col >= CONFIG_MATRIX_COLS) {
        return false;
    }
    if ((uint32_t)region->row + region->rows > CONFIG_MATRIX_ROWS) {
        return false;
    }
    if ((uint32_t)region->col + region->cols > CONFIG_MATRIX_COLS) {
        return false;
    }
    return true;
}

static esp_err_t matrixEngineReadAdcUv(uint16_t col, int32_t *outUv)
{
    if (!outUv || !s_state.cfg.adc) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (s_state.cfg.adcMuxPByCol && s_state.cfg.adcMuxNByCol) {
        muxp = s_state.cfg.adcMuxPByCol[col];
        muxn = s_state.cfg.adcMuxNByCol[col];
    } else if (!matrixEngineDefaultAdcMuxForCol(col, &muxp, &muxn)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ads126xAdcSetInputMux(s_state.cfg.adc, muxp, muxn);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t oversample = matrixEngineOversample();
    int64_t sum = 0;
    for (uint8_t i = 0; i < oversample; ++i) {
        int32_t raw = 0;
        err = ads126xAdcReadAdc1Raw(s_state.cfg.adc, &raw, NULL);
        if (err != ESP_OK) {
            return err;
        }
        int32_t uv = ads126xAdcRawToMicrovolts(s_state.cfg.adc, raw);
        sum += uv;
    }

    *outUv = (int32_t)(sum / oversample);
    return ESP_OK;
}

static esp_err_t matrixEngineReadCapRaw(uint16_t col, int32_t *outRaw)
{
    if (!outRaw) {
        return ESP_ERR_INVALID_STATE;
    }

    Fdc2214CapDevice_t *cap = s_state.cfg.cap;
    if (s_state.cfg.capDevices && s_state.cfg.capDeviceCount > 0) {
        uint8_t index = (uint8_t)(col / MATRIX_ENGINE_DEFAULT_CAP_GROUP_SIZE);
        if (s_state.cfg.capDeviceIndexByCol) {
            index = s_state.cfg.capDeviceIndexByCol[col];
        }
        if (index >= s_state.cfg.capDeviceCount) {
            return ESP_ERR_INVALID_ARG;
        }
        cap = s_state.cfg.capDevices[index];
    }

    if (!cap) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ch = s_state.cfg.capChannelByCol ? s_state.cfg.capChannelByCol[col]
                                             : (uint8_t)(col % MATRIX_ENGINE_DEFAULT_CAP_GROUP_SIZE);
    if (ch > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t oversample = matrixEngineOversample();
    int64_t sum = 0;
    for (uint8_t i = 0; i < oversample; ++i) {
        Fdc2214CapSample_t sample = {0};
        esp_err_t err = Fdc2214CapReadSample(cap, (Fdc2214CapChannel_t)ch, &sample);
        if (err != ESP_OK) {
            return err;
        }
        if (sample.ErrAmplitude || sample.ErrWatchdog) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        sum += (int64_t)sample.Raw28;
    }

    *outRaw = (int32_t)(sum / oversample);
    return ESP_OK;
}

static esp_err_t matrixEngineUvToResistanceMohm(int32_t uv, int32_t *outMohm)
{
    if (!outMohm || s_state.cfg.resistanceRefOhms == 0 || s_state.cfg.resistanceExcitationUv == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (uv <= 0 || (uint32_t)uv >= s_state.cfg.resistanceExcitationUv) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t numerator = (int64_t)s_state.cfg.resistanceRefOhms * 1000 * (int64_t)uv;
    int64_t denom = (int64_t)s_state.cfg.resistanceExcitationUv - (int64_t)uv;
    if (denom == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *outMohm = (int32_t)(numerator / denom);
    return ESP_OK;
}

esp_err_t matrixEngineInit(const matrixEngineConfig_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    matrixEngineState_t next = {0};
    next.cfg = *cfg;
    next.useDefaultRow = (cfg->selectRow == NULL);
    next.useDefaultCol = (cfg->selectColGroup == NULL);
    next.useDefaultDrive = (cfg->drive == NULL);

    if (next.useDefaultRow) {
        next.cfg.selectRow = matrixEngineSelectRowDefault;
    }
    if (next.useDefaultCol) {
        next.cfg.selectColGroup = matrixEngineSelectColGroupDefault;
    }
    if (next.useDefaultDrive) {
        next.cfg.drive = matrixEngineDriveDefault;
    }

    if ((next.cfg.adcMuxPByCol && !next.cfg.adcMuxNByCol) ||
        (!next.cfg.adcMuxPByCol && next.cfg.adcMuxNByCol)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (next.cfg.capDeviceIndexByCol && (!next.cfg.capDevices || next.cfg.capDeviceCount == 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t groupSize = next.cfg.colGroupSize ? next.cfg.colGroupSize : MATRIX_ENGINE_DEFAULT_GROUP_SIZE;
    if (groupSize == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (next.useDefaultCol) {
        uint16_t groups = (CONFIG_MATRIX_COLS + groupSize - 1u) / groupSize;
        if (groups > 2) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (next.useDefaultRow || next.useDefaultCol || next.useDefaultDrive) {
        esp_err_t err = tmuxSwitchInit();
        if (err != ESP_OK) {
            return err;
        }
    }

    next.lock = xSemaphoreCreateMutex();
    if (!next.lock) {
        return ESP_ERR_NO_MEM;
    }

    if (s_state.lock) {
        vSemaphoreDelete(s_state.lock);
    }

    s_state = next;
    s_state.inited = true;
    return ESP_OK;
}

void matrixEngineDeinit(void)
{
    if (s_state.lock) {
        vSemaphoreDelete(s_state.lock);
    }
    memset(&s_state, 0, sizeof(s_state));
}

esp_err_t matrixEngineRegionIo(const matrixEngineRegion_t *region,
                               const matrixEngineRequest_t *req,
                               int32_t *outValues,
                               size_t outCount)
{
    if (!s_state.inited || !matrixEngineRegionValid(region) || !req) {
        return ESP_ERR_INVALID_ARG;
    }

    if (req->io != MATRIX_ENGINE_IO_READ && req->io != MATRIX_ENGINE_IO_WRITE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (req->io == MATRIX_ENGINE_IO_READ) {
        size_t needed = (size_t)region->rows * region->cols;
        if (!outValues || outCount < needed) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (xSemaphoreTake(s_state.lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (req->measure != MATRIX_ENGINE_MEASURE_VOLTAGE_UV &&
        req->measure != MATRIX_ENGINE_MEASURE_CAP_RAW &&
        req->measure != MATRIX_ENGINE_MEASURE_RESISTANCE_MOHM) {
        xSemaphoreGive(s_state.lock);
        return ESP_ERR_INVALID_ARG;
    }

    s_state.activeMeasure = req->measure;

    esp_err_t err = ESP_OK;
    uint16_t groupSize = matrixEngineGroupSize();
    uint16_t colStart = region->col;
    uint16_t colEnd = (uint16_t)(region->col + region->cols);
    uint16_t groupStart = (uint16_t)((colStart / groupSize) * groupSize);

    for (uint16_t row = region->row; row < (uint16_t)(region->row + region->rows); ++row) {
        if (req->io == MATRIX_ENGINE_IO_WRITE) {
            err = s_state.cfg.drive(req->driveSource, s_state.cfg.userCtx);
            if (err != ESP_OK) {
                break;
            }
        }

        err = s_state.cfg.selectRow(row, s_state.cfg.userCtx);
        if (err != ESP_OK) {
            break;
        }

        for (uint16_t groupCol = groupStart; groupCol < colEnd; groupCol = (uint16_t)(groupCol + groupSize)) {
            uint16_t group = groupCol / groupSize;
            err = s_state.cfg.selectColGroup(group, s_state.cfg.userCtx);
            if (err != ESP_OK) {
                break;
            }

            if (req->io == MATRIX_ENGINE_IO_WRITE) {
                continue;
            }

            uint16_t groupLimit = (uint16_t)(groupCol + groupSize);
            if (groupLimit > CONFIG_MATRIX_COLS) {
                groupLimit = CONFIG_MATRIX_COLS;
            }

            for (uint16_t col = groupCol; col < groupLimit; ++col) {
                if (col < colStart || col >= colEnd) {
                    continue;
                }

                int32_t value = 0;
                switch (req->measure) {
                case MATRIX_ENGINE_MEASURE_VOLTAGE_UV:
                    err = matrixEngineReadAdcUv(col, &value);
                    break;
                case MATRIX_ENGINE_MEASURE_RESISTANCE_MOHM: {
                    int32_t uv = 0;
                    err = matrixEngineReadAdcUv(col, &uv);
                    if (err == ESP_OK) {
                        err = matrixEngineUvToResistanceMohm(uv, &value);
                    }
                    break;
                }
                case MATRIX_ENGINE_MEASURE_CAP_RAW:
                    err = matrixEngineReadCapRaw(col, &value);
                    break;
                default:
                    err = ESP_ERR_INVALID_ARG;
                    break;
                }

                if (err != ESP_OK) {
                    break;
                }

                size_t index = (size_t)(row - region->row) * region->cols + (col - region->col);
                outValues[index] = value;
            }

            if (err != ESP_OK) {
                break;
            }
        }

        if (err != ESP_OK) {
            break;
        }
    }

    xSemaphoreGive(s_state.lock);
    return err;
}
