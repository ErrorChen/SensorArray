#include <string.h>

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "ads126xAdc.h"
#include "boardSupport.h"
#include "fdc2214Cap.h"
#include "matrixEngine.h"
#include "protocolUsb.h"
#include "protocolWire.h"
#include "tmuxSwitch.h"

static const char *TAG = "sensorArrayTest";

#ifndef CONFIG_MATRIX_ROWS
#define CONFIG_MATRIX_ROWS 1
#endif
#ifndef CONFIG_MATRIX_COLS
#define CONFIG_MATRIX_COLS 1
#endif
#ifndef CONFIG_MATRIX_FRAME_PERIOD_MS
#define CONFIG_MATRIX_FRAME_PERIOD_MS 50
#endif
#ifndef CONFIG_MATRIX_SCAN_TASK_CORE
#define CONFIG_MATRIX_SCAN_TASK_CORE 1
#endif
#ifndef CONFIG_MATRIX_COMM_TASK_CORE
#define CONFIG_MATRIX_COMM_TASK_CORE 0
#endif
#ifndef CONFIG_MATRIX_SCAN_TASK_STACK
#define CONFIG_MATRIX_SCAN_TASK_STACK 8192
#endif
#ifndef CONFIG_MATRIX_COMM_TASK_STACK
#define CONFIG_MATRIX_COMM_TASK_STACK 4096
#endif
#ifndef CONFIG_MATRIX_SCAN_TASK_PRIO
#define CONFIG_MATRIX_SCAN_TASK_PRIO 12
#endif
#ifndef CONFIG_MATRIX_COMM_TASK_PRIO
#define CONFIG_MATRIX_COMM_TASK_PRIO 8
#endif

#define SENSORARRAY_TEST_QUEUE_LEN 4u
#define SENSORARRAY_TEST_MAX_ROWS 8u
#define SENSORARRAY_TEST_MAX_COLS 8u
#define SENSORARRAY_TEST_RESIST_REF_OHMS 10000u
#define SENSORARRAY_TEST_RESIST_EXCITATION_UV 2500000u

typedef struct {
    protocolWireFrame_t frame;
    matrixEngineMeasure_t measure;
} sensorarrayTestFrame_t;

typedef struct {
    uint32_t parsedFrames;
} sensorarrayCommStats_t;

static QueueHandle_t s_frameQueue = NULL;
static ads126xAdcHandle_t s_adcHandle = {0};
static spi_device_handle_t s_spiDevice = NULL;
static Fdc2214CapDevice_t *s_capDevice = NULL;
static bool s_adcReady = false;
static bool s_capReady = false;
static bool s_matrixReady = false;

static gpio_num_t sensorarrayToGpio(int gpio)
{
    return gpio < 0 ? GPIO_NUM_NC : (gpio_num_t)gpio;
}

static const char *sensorarrayMeasureName(matrixEngineMeasure_t measure)
{
    switch (measure) {
    case MATRIX_ENGINE_MEASURE_VOLTAGE_UV:
        return "voltage_uv";
    case MATRIX_ENGINE_MEASURE_RESISTANCE_MOHM:
        return "resistance_mohm";
    case MATRIX_ENGINE_MEASURE_CAP_RAW:
        return "cap_raw";
    default:
        return "unknown";
    }
}

static esp_err_t sensorarrayInitSpi(spi_device_handle_t *outDevice)
{
    if (!outDevice) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t busCfg = {
        .mosi_io_num = CONFIG_BOARD_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_BOARD_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_BOARD_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES,
    };

#if CONFIG_SENSORARRAY_SPI_USE_DMA
    int dmaChan = SPI_DMA_CH_AUTO;
#else
    int dmaChan = 0;
#endif

    esp_err_t err = spi_bus_initialize(CONFIG_BOARD_SPI_HOST, &busCfg, dmaChan);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized");
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = CONFIG_ADS126X_SPI_CLOCK_HZ,
        .mode = 1,
        .spics_io_num = CONFIG_BOARD_ADS126X_CS_GPIO,
        .queue_size = 1,
    };

    return spi_bus_add_device(CONFIG_BOARD_SPI_HOST, &devCfg, outDevice);
}

static esp_err_t sensorarrayInitAdc(ads126xAdcHandle_t *handle, spi_device_handle_t *spiDevice)
{
    if (!handle || !spiDevice) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayInitSpi(spiDevice);
    if (err != ESP_OK) {
        return err;
    }

    ads126xAdcConfig_t cfg = {0};
    cfg.spiDevice = *spiDevice;
    cfg.drdyGpio = sensorarrayToGpio(CONFIG_BOARD_ADS126X_DRDY_GPIO);
    cfg.resetGpio = sensorarrayToGpio(CONFIG_BOARD_ADS126X_RESET_GPIO);
#if CONFIG_SENSORARRAY_ADS1262
    cfg.forcedType = ADS126X_DEVICE_ADS1262;
#elif CONFIG_SENSORARRAY_ADS1263
    cfg.forcedType = ADS126X_DEVICE_ADS1263;
#else
    cfg.forcedType = ADS126X_DEVICE_AUTO;
#endif
    cfg.crcMode = ADS126X_CRC_OFF;
    cfg.enableStatusByte = false;
    cfg.enableInternalRef = true;
    cfg.vrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV;
    cfg.pgaGain = 1;
    cfg.dataRateDr = 0;

    err = ads126xAdcInit(handle, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "ADS126x init ok, deviceType=%d id=0x%02X", handle->deviceType, handle->idRegRaw);
    return ESP_OK;
}

static esp_err_t sensorarrayInitCap(Fdc2214CapDevice_t **outDev)
{
#if !CONFIG_FDC2214CAP_ENABLE
    (void)outDev;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!outDev) {
        return ESP_ERR_INVALID_ARG;
    }

    const BoardSupportI2cCtx_t *i2cCtx = boardSupportGetI2cCtx();
    if (!i2cCtx) {
        return ESP_ERR_INVALID_STATE;
    }

    Fdc2214CapBusConfig_t busCfg = {
        .I2cAddress7 = CONFIG_FDC2214CAP_I2C_ADDR,
        .UserCtx = (void *)i2cCtx,
        .WriteRead = boardSupportI2cWriteRead,
        .Write = boardSupportI2cWrite,
        .IntGpio = -1,
    };

    Fdc2214CapDevice_t *dev = NULL;
    esp_err_t err = Fdc2214CapCreate(&busCfg, &dev);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapReset(dev);
    if (err != ESP_OK) {
        Fdc2214CapDestroy(dev);
        return err;
    }

    uint16_t manufacturer = 0;
    uint16_t deviceId = 0;
    err = Fdc2214CapReadId(dev, &manufacturer, &deviceId);
    if (err != ESP_OK) {
        Fdc2214CapDestroy(dev);
        return err;
    }

    Fdc2214CapChannelConfig_t chCfg = {
        .Rcount = 0xFFFF,
        .SettleCount = 0x0400,
        .Offset = 0x0000,
        .ClockDividers = 0x0001,
        .DriveCurrent = 0xA000,
    };

    uint8_t channels = (uint8_t)CONFIG_FDC2214CAP_CHANNELS;
    if (channels > 4u) {
        channels = 4u;
    }
    for (uint8_t ch = 0; ch < channels; ++ch) {
        err = Fdc2214CapConfigureChannel(dev, (Fdc2214CapChannel_t)ch, &chCfg);
        if (err != ESP_OK) {
            Fdc2214CapDestroy(dev);
            return err;
        }
    }

    if (channels > 1u) {
        err = Fdc2214CapSetAutoScanMode(dev, 0, FDC2214_DEGLITCH_10MHZ);
    } else {
        err = Fdc2214CapSetSingleChannelMode(dev, FDC2214_CH0);
    }
    if (err != ESP_OK) {
        Fdc2214CapDestroy(dev);
        return err;
    }

    ESP_LOGI(TAG, "FDC2214 init ok (0x%02X)", CONFIG_FDC2214CAP_I2C_ADDR);
    *outDev = dev;
    return ESP_OK;
#endif
}

static esp_err_t sensorarrayInitMatrix(void)
{
    matrixEngineConfig_t cfg = {0};
    cfg.adc = s_adcReady ? &s_adcHandle : NULL;
    cfg.cap = s_capReady ? s_capDevice : NULL;
    cfg.oversample = 0;
    cfg.resistanceRefOhms = SENSORARRAY_TEST_RESIST_REF_OHMS;
    cfg.resistanceExcitationUv = SENSORARRAY_TEST_RESIST_EXCITATION_UV;
    cfg.selectRow = NULL;
    cfg.selectColGroup = NULL;
    cfg.drive = NULL;
    cfg.userCtx = NULL;

    return matrixEngineInit(&cfg);
}

static void sensorarrayFillFrame(protocolWireFrame_t *frame,
                                 const int32_t *values,
                                 size_t count,
                                 uint16_t rows,
                                 uint16_t cols,
                                 matrixEngineMeasure_t measure,
                                 uint16_t seq)
{
    if (!frame || !values || rows == 0 || cols == 0) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->seq = seq;
    frame->t0 = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    size_t points = count;
    if (points > PROTOCOL_WIRE_POINT_COUNT) {
        points = PROTOCOL_WIRE_POINT_COUNT;
    }

    for (size_t idx = 0; idx < points; ++idx) {
        uint16_t row = (uint16_t)(idx / cols);
        uint16_t col = (uint16_t)(idx % cols);
        frame->validMask |= (1ULL << idx);
        frame->offset[idx] = (uint16_t)(((row & 0xFFu) << 8) | (col & 0xFFu));

        uint32_t payload = (uint32_t)values[idx] & PROTOCOL_WIRE_PAYLOAD_MASK;
        uint8_t tag = (measure == MATRIX_ENGINE_MEASURE_CAP_RAW)
                          ? PROTOCOL_WIRE_DATA_TAG_FDC2214
                          : PROTOCOL_WIRE_DATA_TAG_NONE;
        frame->data[idx] = protocolWirePackTaggedU28(tag, payload);
    }
}

static esp_err_t sensorarrayReadMatrix(matrixEngineMeasure_t measure,
                                       int32_t *values,
                                       size_t valueCount,
                                       uint16_t rows,
                                       uint16_t cols)
{
    matrixEngineRegion_t region = {
        .row = 0,
        .col = 0,
        .rows = rows,
        .cols = cols,
    };

    matrixEngineRequest_t req = {
        .io = MATRIX_ENGINE_IO_READ,
        .measure = measure,
        .driveSource = TMUX1108_SOURCE_GND,
    };

    return matrixEngineRegionIo(&region, &req, values, valueCount);
}

static void sensorarrayCommOnFrame(const uint8_t *payload, uint16_t payloadLen, void *userCtx)
{
    (void)payload;
    (void)payloadLen;
    sensorarrayCommStats_t *stats = (sensorarrayCommStats_t *)userCtx;
    if (stats) {
        stats->parsedFrames += 1u;
    }
}

static void sensorarrayScanTask(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "scan task start core=%d", (int)xPortGetCoreID());

    uint16_t rows = (uint16_t)CONFIG_MATRIX_ROWS;
    uint16_t cols = (uint16_t)CONFIG_MATRIX_COLS;
    if (rows == 0) {
        rows = 1;
    }
    if (cols == 0) {
        cols = 1;
    }
    if (rows > SENSORARRAY_TEST_MAX_ROWS) {
        rows = SENSORARRAY_TEST_MAX_ROWS;
    }
    if (cols > SENSORARRAY_TEST_MAX_COLS) {
        cols = SENSORARRAY_TEST_MAX_COLS;
    }

    size_t pointCount = (size_t)rows * cols;
    int32_t values[PROTOCOL_WIRE_POINT_COUNT] = {0};
    uint16_t seq = 0;

    while (true) {
        if (!s_matrixReady) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        matrixEngineMeasure_t measures[] = {
            MATRIX_ENGINE_MEASURE_VOLTAGE_UV,
            MATRIX_ENGINE_MEASURE_RESISTANCE_MOHM,
            MATRIX_ENGINE_MEASURE_CAP_RAW,
        };

        for (size_t i = 0; i < sizeof(measures) / sizeof(measures[0]); ++i) {
            matrixEngineMeasure_t measure = measures[i];
            if ((measure == MATRIX_ENGINE_MEASURE_CAP_RAW && !s_capReady) ||
                (measure != MATRIX_ENGINE_MEASURE_CAP_RAW && !s_adcReady)) {
                continue;
            }

            esp_err_t err = sensorarrayReadMatrix(measure, values, pointCount, rows, cols);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "matrix read %s failed: %d", sensorarrayMeasureName(measure), err);
                continue;
            }

            sensorarrayTestFrame_t frame = {0};
            frame.measure = measure;
            sensorarrayFillFrame(&frame.frame, values, pointCount, rows, cols, measure, seq++);

            if (s_frameQueue) {
                if (xQueueSend(s_frameQueue, &frame, pdMS_TO_TICKS(50)) != pdTRUE) {
                    ESP_LOGW(TAG, "frame queue full; dropping");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_MATRIX_FRAME_PERIOD_MS));
    }
}

static void sensorarrayCommTask(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "comm task start core=%d", (int)xPortGetCoreID());

    protocolUsbParser_t parser = {0};
    sensorarrayCommStats_t stats = {0};
    protocolUsbParserInit(&parser, sensorarrayCommOnFrame, &stats);

    while (true) {
        sensorarrayTestFrame_t frame = {0};
        if (!s_frameQueue) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (xQueueReceive(s_frameQueue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t wireBuf[PROTOCOL_WIRE_FRAME_BYTES] = {0};
        esp_err_t err = protocolWirePackFrame(&frame.frame, wireBuf, sizeof(wireBuf));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "pack wire failed: %d", err);
            continue;
        }

        uint8_t usbBuf[PROTOCOL_USB_HEADER_BYTES + PROTOCOL_WIRE_FRAME_BYTES] = {0};
        err = protocolUsbBuildFrame(wireBuf, sizeof(wireBuf), usbBuf, sizeof(usbBuf));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "pack usb failed: %d", err);
            continue;
        }

        size_t parsed = protocolUsbParserFeed(&parser, usbBuf, sizeof(usbBuf));
        ESP_LOGI(TAG,
                 "send %s seq=%u bytes=%u parsed=%u total=%u",
                 sensorarrayMeasureName(frame.measure),
                 frame.frame.seq,
                 (unsigned)sizeof(usbBuf),
                 (unsigned)parsed,
                 (unsigned)stats.parsedFrames);
    }
}

static void sensorarrayCoreProbeTask(void *arg)
{
    (void)arg;
    for (int i = 0; i < 5; ++i) {
        ESP_LOGI(TAG, "core probe tick=%u core=%d", (unsigned)i, (int)xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

static void sensorarrayCreateTask(const char *name,
                                  TaskFunction_t fn,
                                  uint32_t stack,
                                  void *arg,
                                  UBaseType_t prio,
                                  int coreId)
{
#if CONFIG_FREERTOS_UNICORE
    (void)coreId;
    BaseType_t ok = xTaskCreate(fn, name, stack, arg, prio, NULL);
#else
    BaseType_t ok = xTaskCreatePinnedToCore(fn, name, stack, arg, prio, NULL, coreId);
#endif
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed: %s", name);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "sensor array test app start");

    esp_err_t err = boardSupportInit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "boardSupportInit failed: %d", err);
    }

    err = tmuxSwitchInit();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "tmuxSwitchInit failed: %d", err);
    }

    err = sensorarrayInitAdc(&s_adcHandle, &s_spiDevice);
    if (err == ESP_OK) {
        s_adcReady = true;
    } else {
        ESP_LOGW(TAG, "ADS126x init skipped: %d", err);
    }

    err = sensorarrayInitCap(&s_capDevice);
    if (err == ESP_OK) {
        s_capReady = true;
    } else {
        ESP_LOGW(TAG, "FDC2214 init skipped: %d", err);
    }

    err = sensorarrayInitMatrix();
    if (err == ESP_OK) {
        s_matrixReady = true;
    } else {
        ESP_LOGW(TAG, "matrixEngineInit failed: %d", err);
    }

    s_frameQueue = xQueueCreate(SENSORARRAY_TEST_QUEUE_LEN, sizeof(sensorarrayTestFrame_t));
    if (!s_frameQueue) {
        ESP_LOGE(TAG, "frame queue create failed");
        return;
    }

    sensorarrayCreateTask("sensorScan",
                          sensorarrayScanTask,
                          CONFIG_MATRIX_SCAN_TASK_STACK,
                          NULL,
                          CONFIG_MATRIX_SCAN_TASK_PRIO,
                          CONFIG_MATRIX_SCAN_TASK_CORE);

    sensorarrayCreateTask("sensorComm",
                          sensorarrayCommTask,
                          CONFIG_MATRIX_COMM_TASK_STACK,
                          NULL,
                          CONFIG_MATRIX_COMM_TASK_PRIO,
                          CONFIG_MATRIX_COMM_TASK_CORE);

    sensorarrayCreateTask("coreProbe",
                          sensorarrayCoreProbeTask,
                          2048,
                          NULL,
                          1,
                          (CONFIG_MATRIX_SCAN_TASK_CORE == 0) ? 1 : 0);
}
