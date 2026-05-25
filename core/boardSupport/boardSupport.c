#include "boardSupport.h"

#include <stdbool.h>
#include <stdio.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "boardSupport";

#ifndef CONFIG_BOARD_I2C_FREQ_HZ
#define CONFIG_BOARD_I2C_FREQ_HZ 325000
#endif
#ifndef CONFIG_BOARD_I2C0_FREQ_HZ
#define CONFIG_BOARD_I2C0_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_BOARD_I2C1_FREQ_HZ
#define CONFIG_BOARD_I2C1_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_S5D5_I2C_RECOVER_ON_TIMEOUT
#define CONFIG_SENSORARRAY_DEBUG_S5D5_I2C_RECOVER_ON_TIMEOUT 0
#endif

#define BOARD_SUPPORT_I2C_TIMEOUT_MS 100u

static bool s_inited = false;
static bool s_i2c0_inited = false;
static bool s_i2c1_inited = false;
static bool s_i2c0_recovering = false;
static bool s_i2c1_recovering = false;
static uint32_t s_i2c0_configured_freq_hz = CONFIG_BOARD_I2C0_FREQ_HZ;
static uint32_t s_i2c1_configured_freq_hz = CONFIG_BOARD_I2C1_FREQ_HZ;

static BoardSupportI2cCtx_t s_i2c0_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
    .SdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
    .SclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
    .FrequencyHz = CONFIG_BOARD_I2C0_FREQ_HZ,
};

static BoardSupportI2cCtx_t s_i2c1_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
    .SdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
    .SclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
    .FrequencyHz = CONFIG_BOARD_I2C1_FREQ_HZ,
};

static bool boardSupportI2cPinsValid(int sda_gpio, int scl_gpio)
{
    return (sda_gpio >= 0) && (scl_gpio >= 0);
}

static TickType_t boardSupportI2cTimeoutTicks(uint32_t timeoutMs)
{
    TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
    return (ticks == 0) ? 1 : ticks;
}

static bool boardSupportI2cErrorShouldRecover(esp_err_t err)
{
#if CONFIG_SENSORARRAY_DEBUG_S5D5_I2C_RECOVER_ON_TIMEOUT
    return err == ESP_ERR_TIMEOUT;
#else
    (void)err;
    return false;
#endif
}

static bool *boardSupportI2cRecoveringFlag(i2c_port_t port)
{
    if (port == s_i2c0_ctx.Port) {
        return &s_i2c0_recovering;
    }
    if (port == s_i2c1_ctx.Port) {
        return &s_i2c1_recovering;
    }
    return NULL;
}

static bool boardSupportI2cBusAppearsStuck(const BoardSupportI2cCtx_t *ctx, int *outSda, int *outScl)
{
    if (!ctx || !boardSupportI2cPinsValid(ctx->SdaGpio, ctx->SclGpio)) {
        return false;
    }

    int sda = gpio_get_level((gpio_num_t)ctx->SdaGpio);
    int scl = gpio_get_level((gpio_num_t)ctx->SclGpio);
    if (outSda) {
        *outSda = sda;
    }
    if (outScl) {
        *outScl = scl;
    }
    return sda == 0 || scl == 0;
}

static esp_err_t boardSupportInitI2c(i2c_port_t port,
                                     int sda_gpio,
                                     int scl_gpio,
                                     uint32_t requested_freq_hz,
                                     uint32_t *out_configured_freq_hz)
{
    if (!boardSupportI2cPinsValid(sda_gpio, scl_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_config_t cfg = {0};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = sda_gpio;
    cfg.scl_io_num = scl_gpio;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = requested_freq_hz;
#ifdef SOC_I2C_SUPPORT_CLK_SRC
    cfg.clk_flags = 0;
#endif

    esp_err_t err = i2c_param_config(port, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config port %d failed: %d", (int)port, err);
        return err;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install port %d failed: %d", (int)port, err);
        return err;
    }

    if (out_configured_freq_hz) {
        *out_configured_freq_hz = cfg.master.clk_speed;
    }
    return ESP_OK;
}

esp_err_t boardSupportRecoverI2cBus(const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPinsValid(ctx->SdaGpio, ctx->SclGpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    bool *recovering = boardSupportI2cRecoveringFlag(ctx->Port);
    if (!recovering) {
        return ESP_ERR_INVALID_ARG;
    }
    if (*recovering) {
        printf("I2CRECOVER,stage=skip_already_recovering,port=%d\n", (int)ctx->Port);
        return ESP_ERR_INVALID_STATE;
    }

    *recovering = true;
    printf("I2CRECOVER,stage=begin,port=%d,sdaGpio=%d,sclGpio=%d,freqHz=%lu\n",
           (int)ctx->Port,
           ctx->SdaGpio,
           ctx->SclGpio,
           (unsigned long)ctx->FrequencyHz);

    esp_err_t deleteErr = i2c_driver_delete(ctx->Port);
    if (deleteErr != ESP_OK && deleteErr != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "i2c_driver_delete port %d during recovery returned %d", (int)ctx->Port, deleteErr);
    }

    if (ctx->Port == s_i2c0_ctx.Port) {
        s_i2c0_inited = false;
    }
    if (ctx->Port == s_i2c1_ctx.Port) {
        s_i2c1_inited = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t configuredFreqHz = ctx->FrequencyHz;
    esp_err_t initErr = boardSupportInitI2c(ctx->Port,
                                            ctx->SdaGpio,
                                            ctx->SclGpio,
                                            ctx->FrequencyHz,
                                            &configuredFreqHz);
    if (initErr == ESP_OK) {
        if (ctx->Port == s_i2c0_ctx.Port) {
            s_i2c0_inited = true;
            s_i2c0_configured_freq_hz = configuredFreqHz;
            s_i2c0_ctx.FrequencyHz = configuredFreqHz;
        }
        if (ctx->Port == s_i2c1_ctx.Port) {
            s_i2c1_inited = true;
            s_i2c1_configured_freq_hz = configuredFreqHz;
            s_i2c1_ctx.FrequencyHz = configuredFreqHz;
        }
    }

    printf("I2CRECOVER,stage=done,port=%d,err=%ld,configuredFreqHz=%lu\n",
           (int)ctx->Port,
           (long)initErr,
           (unsigned long)configuredFreqHz);
    *recovering = false;
    return initErr;
}

esp_err_t boardSupportInit(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    esp_err_t err = boardSupportInitI2c((i2c_port_t)CONFIG_BOARD_I2C_PORT,
                                        CONFIG_BOARD_I2C_SDA_GPIO,
                                        CONFIG_BOARD_I2C_SCL_GPIO,
                                        CONFIG_BOARD_I2C0_FREQ_HZ,
                                        &s_i2c0_configured_freq_hz);
    if (err != ESP_OK) {
        return err;
    }
    s_i2c0_inited = true;
    s_i2c0_ctx.FrequencyHz = s_i2c0_configured_freq_hz;
    ESP_LOGI(TAG,
             "I2C0: port=%d SDA=%d SCL=%d requestedFreqHz=%u configuredFreqHz=%u note=configured_not_measured",
             CONFIG_BOARD_I2C_PORT,
             CONFIG_BOARD_I2C_SDA_GPIO,
             CONFIG_BOARD_I2C_SCL_GPIO,
             (unsigned)CONFIG_BOARD_I2C0_FREQ_HZ,
             (unsigned)s_i2c0_configured_freq_hz);

    if (boardSupportIsI2c1Enabled()) {
        if (CONFIG_BOARD_I2C1_PORT == CONFIG_BOARD_I2C_PORT) {
            ESP_LOGE(TAG, "I2C1 port must differ from I2C0 port");
            i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
            s_i2c0_inited = false;
            return ESP_ERR_INVALID_ARG;
        }

        err = boardSupportInitI2c((i2c_port_t)CONFIG_BOARD_I2C1_PORT,
                                  CONFIG_BOARD_I2C1_SDA_GPIO,
                                  CONFIG_BOARD_I2C1_SCL_GPIO,
                                  CONFIG_BOARD_I2C1_FREQ_HZ,
                                  &s_i2c1_configured_freq_hz);
        if (err != ESP_OK) {
            i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
            s_i2c0_inited = false;
            return err;
        }
        s_i2c1_inited = true;
        s_i2c1_ctx.FrequencyHz = s_i2c1_configured_freq_hz;
        ESP_LOGI(TAG,
                 "I2C1: port=%d SDA=%d SCL=%d requestedFreqHz=%u configuredFreqHz=%u note=configured_not_measured",
                 CONFIG_BOARD_I2C1_PORT,
                 CONFIG_BOARD_I2C1_SDA_GPIO,
                 CONFIG_BOARD_I2C1_SCL_GPIO,
                 (unsigned)CONFIG_BOARD_I2C1_FREQ_HZ,
                 (unsigned)s_i2c1_configured_freq_hz);
    }

    s_inited = true;
    return ESP_OK;
}

esp_err_t boardSupportDeinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    if (s_i2c1_inited) {
        esp_err_t deinit_err = i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C1_PORT);
        if (deinit_err != ESP_OK) {
            err = deinit_err;
        }
        s_i2c1_inited = false;
    }

    if (s_i2c0_inited) {
        esp_err_t deinit_err = i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
        if (deinit_err != ESP_OK) {
            err = deinit_err;
        }
        s_i2c0_inited = false;
    }

    s_inited = false;
    return err;
}

bool boardSupportIsI2c1Enabled(void)
{
    return boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO) &&
           (CONFIG_BOARD_I2C1_PORT >= 0);
}

const BoardSupportI2cCtx_t* boardSupportGetI2cCtx(void)
{
    return &s_i2c0_ctx;
}

const BoardSupportI2cCtx_t* boardSupportGetI2c1Ctx(void)
{
    if (!boardSupportIsI2c1Enabled()) {
        return NULL;
    }
    return &s_i2c1_ctx;
}

bool boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *outInfo)
{
    if (!outInfo) {
        return false;
    }

    if (secondary) {
        *outInfo = (BoardSupportI2cBusInfo_t){
            .Enabled = boardSupportIsI2c1Enabled(),
            .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
            .SdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
            .SclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
            .FrequencyHz = s_i2c1_inited ? s_i2c1_configured_freq_hz : CONFIG_BOARD_I2C1_FREQ_HZ,
        };
        return true;
    }

    *outInfo = (BoardSupportI2cBusInfo_t){
        .Enabled = boardSupportI2cPinsValid(CONFIG_BOARD_I2C_SDA_GPIO, CONFIG_BOARD_I2C_SCL_GPIO) &&
                   (CONFIG_BOARD_I2C_PORT >= 0),
        .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
        .SdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
        .SclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
        .FrequencyHz = s_i2c0_inited ? s_i2c0_configured_freq_hz : CONFIG_BOARD_I2C0_FREQ_HZ,
    };
    return true;
}

esp_err_t boardSupportI2cWriteRead(void* userCtx,
                                  uint8_t addr7,
                                  const uint8_t* tx,
                                  size_t txLen,
                                  uint8_t* rx,
                                  size_t rxLen)
{
    if (!userCtx || !tx || txLen == 0 || !rx || rxLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    const BoardSupportI2cCtx_t* ctx = (const BoardSupportI2cCtx_t*)userCtx;
    esp_err_t err = i2c_master_write_read_device(ctx->Port,
                                                 addr7,
                                                 tx,
                                                 txLen,
                                                 rx,
                                                 rxLen,
                                                 boardSupportI2cTimeoutTicks(ctx->TimeoutMs));
    if (boardSupportI2cErrorShouldRecover(err)) {
        int sda = -1;
        int scl = -1;
        if (!boardSupportI2cBusAppearsStuck(ctx, &sda, &scl)) {
            printf("I2CRECOVER,stage=skip_lines_released,op=write_read,port=%d,addr=0x%02X,err=%ld,sda=%d,scl=%d\n",
                   (int)ctx->Port,
                   addr7,
                   (long)err,
                   sda,
                   scl);
            return err;
        }
        printf("I2CRECOVER,stage=request,op=write_read,port=%d,addr=0x%02X,err=%ld,sda=%d,scl=%d\n",
               (int)ctx->Port,
               addr7,
               (long)err,
               sda,
               scl);
#if CONFIG_SENSORARRAY_DEBUG_S5D5_I2C_RECOVER_ON_TIMEOUT
        (void)boardSupportRecoverI2cBus(ctx);
#endif
    }
    return err;
}

esp_err_t boardSupportI2cWrite(void* userCtx,
                              uint8_t addr7,
                              const uint8_t* tx,
                              size_t txLen)
{
    if (!userCtx || !tx || txLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    const BoardSupportI2cCtx_t* ctx = (const BoardSupportI2cCtx_t*)userCtx;
    esp_err_t err = i2c_master_write_to_device(ctx->Port,
                                               addr7,
                                               tx,
                                               txLen,
                                               boardSupportI2cTimeoutTicks(ctx->TimeoutMs));
    if (boardSupportI2cErrorShouldRecover(err)) {
        int sda = -1;
        int scl = -1;
        if (!boardSupportI2cBusAppearsStuck(ctx, &sda, &scl)) {
            printf("I2CRECOVER,stage=skip_lines_released,op=write,port=%d,addr=0x%02X,err=%ld,sda=%d,scl=%d\n",
                   (int)ctx->Port,
                   addr7,
                   (long)err,
                   sda,
                   scl);
            return err;
        }
        printf("I2CRECOVER,stage=request,op=write,port=%d,addr=0x%02X,err=%ld,sda=%d,scl=%d\n",
               (int)ctx->Port,
               addr7,
               (long)err,
               sda,
               scl);
#if CONFIG_SENSORARRAY_DEBUG_S5D5_I2C_RECOVER_ON_TIMEOUT
        (void)boardSupportRecoverI2cBus(ctx);
#endif
    }
    return err;
}

esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *i2cCtx, uint8_t addr7)
{
    if (!i2cCtx || addr7 > 0x7Fu) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, (uint8_t)((addr7 << 1u) | I2C_MASTER_WRITE), true);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(i2cCtx->Port, cmd, boardSupportI2cTimeoutTicks(i2cCtx->TimeoutMs));
    }

    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        printf("I2CPROBE,port=%d,addr=0x%02X,err=%ld,status=%s\n",
               (int)i2cCtx->Port,
               addr7,
               (long)err,
               (err == ESP_FAIL) ? "no_ack_no_recover" : "failed_no_recover");
    }
    return err;
}

// IMPORTANT: Do not define app_main() here.
