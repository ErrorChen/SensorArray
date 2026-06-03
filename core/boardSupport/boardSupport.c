#include "boardSupport.h"

#include <stdbool.h>
#include <stdio.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_BOARD_I2C_FREQ_HZ
#define CONFIG_BOARD_I2C_FREQ_HZ 325000
#endif
#ifndef CONFIG_BOARD_I2C0_FREQ_HZ
#define CONFIG_BOARD_I2C0_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_BOARD_I2C1_FREQ_HZ
#define CONFIG_BOARD_I2C1_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_SENSORARRAY_I2C_RECOVER_ON_TIMEOUT
#define CONFIG_SENSORARRAY_I2C_RECOVER_ON_TIMEOUT 0
#endif
#ifndef CONFIG_BOARD_I2C1_ENABLE
#define CONFIG_BOARD_I2C1_ENABLE 0
#endif

#if CONFIG_BOARD_I2C1_ENABLE
#define BOARD_SUPPORT_I2C1_ENABLE_REQUESTED 1
#else
#define BOARD_SUPPORT_I2C1_ENABLE_REQUESTED 0
#endif

#define BOARD_SUPPORT_I2C_TIMEOUT_MS 100u
#define BOARD_SUPPORT_I2C_TRACE_SLOW_MS 50u

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

static bool boardSupportI2cPortValid(int port)
{
    return port >= 0 && port < I2C_NUM_MAX;
}

static bool boardSupportI2c1EnableRequested(void)
{
    return BOARD_SUPPORT_I2C1_ENABLE_REQUESTED != 0;
}

static bool boardSupportI2c1ConfiguredByPins(void)
{
    return boardSupportI2c1EnableRequested() &&
           boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO) &&
           (CONFIG_BOARD_I2C1_PORT >= 0);
}

static bool boardSupportI2cCtxInstalled(const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx || !boardSupportI2cPortValid((int)ctx->Port)) {
        return false;
    }
    if (ctx->Port == s_i2c0_ctx.Port &&
        ctx->SdaGpio == s_i2c0_ctx.SdaGpio &&
        ctx->SclGpio == s_i2c0_ctx.SclGpio) {
        return s_i2c0_inited;
    }
    if (ctx->Port == s_i2c1_ctx.Port &&
        ctx->SdaGpio == s_i2c1_ctx.SdaGpio &&
        ctx->SclGpio == s_i2c1_ctx.SclGpio) {
        return s_i2c1_inited;
    }
    return false;
}

static TickType_t boardSupportI2cTimeoutTicks(uint32_t timeoutMs)
{
    TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
    return (ticks == 0) ? 1 : ticks;
}

static uint32_t boardSupportI2cElapsedMs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    if (elapsedUs <= 0) {
        return 0u;
    }
    return (uint32_t)((elapsedUs + 999LL) / 1000LL);
}

static bool boardSupportI2cErrorShouldRecover(esp_err_t err)
{
#if CONFIG_SENSORARRAY_I2C_RECOVER_ON_TIMEOUT
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
    if (outSda) {
        *outSda = -1;
    }
    if (outScl) {
        *outScl = -1;
    }
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

static void boardSupportI2cTraceIfNeeded(const char *op,
                                         const BoardSupportI2cCtx_t *ctx,
                                         uint8_t addr7,
                                         esp_err_t err,
                                         uint32_t elapsedMs,
                                         int sda,
                                         int scl)
{
    if (err == ESP_OK && elapsedMs <= BOARD_SUPPORT_I2C_TRACE_SLOW_MS) {
        return;
    }

    printf("I2C_TRACE,op=%s,port=%d,addr=0x%02X,err=%ld,elapsedMs=%lu,sda=%d,scl=%d\n",
           op ? op : "unknown",
           ctx ? (int)ctx->Port : -1,
           addr7,
           (long)err,
           (unsigned long)elapsedMs,
           sda,
           scl);
}

static bool boardSupportI2cExpectedSecondaryHighAddressNack(const BoardSupportI2cCtx_t *ctx, uint8_t addr7)
{
    return ctx && (int)ctx->Port == 1 && addr7 == 0x2Bu;
}

static void boardSupportI2cHandleTransactionError(const char *op,
                                                  const BoardSupportI2cCtx_t *ctx,
                                                  uint8_t addr7,
                                                  esp_err_t err,
                                                  bool stuck,
                                                  int sda,
                                                  int scl)
{
    if (err == ESP_ERR_TIMEOUT) {
        bool doRecover = stuck && boardSupportI2cErrorShouldRecover(err);
        printf("I2C_TIMEOUT,op=%s,port=%d,addr=0x%02X,sda=%d,scl=%d,action=%s\n",
               op ? op : "unknown",
               ctx ? (int)ctx->Port : -1,
               addr7,
               sda,
               scl,
               doRecover ? "recover" : "skip_recover");
        if (doRecover) {
            (void)boardSupportRecoverI2cBus(ctx);
        }
        return;
    }

    if (err == ESP_FAIL) {
        bool doRecover = stuck && (CONFIG_SENSORARRAY_I2C_RECOVER_ON_TIMEOUT != 0);
        printf("I2C_NACK,op=%s,port=%d,addr=0x%02X,sda=%d,scl=%d,recover=%u\n",
               op ? op : "unknown",
               ctx ? (int)ctx->Port : -1,
               addr7,
               sda,
               scl,
               doRecover ? 1u : 0u);
        if (doRecover) {
            (void)boardSupportRecoverI2cBus(ctx);
        }
    }
}

static esp_err_t boardSupportInitI2c(i2c_port_t port,
                                     int sda_gpio,
                                     int scl_gpio,
                                     uint32_t requested_freq_hz,
                                     uint32_t *out_configured_freq_hz)
{
    if (!boardSupportI2cPortValid((int)port)) {
        printf("BOARD_I2C_FATAL,stage=validate,port=%d,sda=%d,scl=%d,freqHz=%lu,reason=invalid_i2c_port\n",
               (int)port,
               sda_gpio,
               scl_gpio,
               (unsigned long)requested_freq_hz);
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPinsValid(sda_gpio, scl_gpio)) {
        printf("BOARD_I2C_FATAL,stage=validate,port=%d,sda=%d,scl=%d,freqHz=%lu,reason=invalid_i2c_pins\n",
               (int)port,
               sda_gpio,
               scl_gpio,
               (unsigned long)requested_freq_hz);
        return ESP_ERR_INVALID_ARG;
    }
    if (requested_freq_hz == 0u) {
        printf("BOARD_I2C_FATAL,stage=validate,port=%d,sda=%d,scl=%d,freqHz=%lu,reason=invalid_i2c_freq\n",
               (int)port,
               sda_gpio,
               scl_gpio,
               (unsigned long)requested_freq_hz);
        return ESP_ERR_INVALID_ARG;
    }

    printf("BOARD_I2C_INIT,stage=begin,port=%d,sda=%d,scl=%d,requestedFreqHz=%lu\n",
           (int)port,
           sda_gpio,
           scl_gpio,
           (unsigned long)requested_freq_hz);

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
        printf("BOARD_I2C_FATAL,stage=param_config,port=%d,sda=%d,scl=%d,requestedFreqHz=%lu,err=%ld\n",
               (int)port,
               sda_gpio,
               scl_gpio,
               (unsigned long)requested_freq_hz,
               (long)err);
        return err;
    }

    printf("BOARD_I2C_INIT,stage=driver_install,port=%d\n", (int)port);
    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        printf("BOARD_I2C_FATAL,stage=driver_install,port=%d,err=%ld\n",
               (int)port,
               (long)err);
        return err;
    }

    uint32_t configured_freq_hz = cfg.master.clk_speed;
    if (out_configured_freq_hz) {
        *out_configured_freq_hz = configured_freq_hz;
    }
    printf("BOARD_I2C_INIT,stage=installed,port=%d,sda=%d,scl=%d,requestedFreqHz=%lu,configuredFreqHz=%lu\n",
           (int)port,
           sda_gpio,
           scl_gpio,
           (unsigned long)requested_freq_hz,
           (unsigned long)configured_freq_hz);
    return ESP_OK;
}

esp_err_t boardSupportRecoverI2cBus(const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPortValid((int)ctx->Port)) {
        printf("I2CRECOVER,stage=reject,port=%d,reason=invalid_i2c_port\n",
               (int)ctx->Port);
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPinsValid(ctx->SdaGpio, ctx->SclGpio)) {
        printf("I2CRECOVER,stage=reject,port=%d,sda=%d,scl=%d,reason=invalid_i2c_pins\n",
               (int)ctx->Port,
               ctx->SdaGpio,
               ctx->SclGpio);
        return ESP_ERR_INVALID_ARG;
    }

    bool *recovering = boardSupportI2cRecoveringFlag(ctx->Port);
    if (!recovering) {
        printf("I2CRECOVER,stage=reject,port=%d,reason=no_recovering_flag\n",
               (int)ctx->Port);
        return ESP_ERR_INVALID_ARG;
    }
    if (*recovering) {
        printf("I2CRECOVER,stage=skip_already_recovering,port=%d\n", (int)ctx->Port);
        return ESP_ERR_INVALID_STATE;
    }
    if (!boardSupportI2cCtxInstalled(ctx)) {
        printf("I2CRECOVER,stage=reject,port=%d,reason=i2c_driver_not_installed\n",
               (int)ctx->Port);
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
        printf("I2CRECOVER,stage=driver_delete,port=%d,err=%ld\n",
               (int)ctx->Port,
               (long)deleteErr);
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

    s_i2c0_inited = false;
    s_i2c1_inited = false;
    s_i2c0_ctx = (BoardSupportI2cCtx_t){
        .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
        .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
        .SdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
        .SclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
        .FrequencyHz = CONFIG_BOARD_I2C0_FREQ_HZ,
    };
    s_i2c1_ctx = (BoardSupportI2cCtx_t){
        .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
        .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
        .SdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
        .SclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
        .FrequencyHz = CONFIG_BOARD_I2C1_FREQ_HZ,
    };

    printf("BOARD_I2C_CFG,primaryPort=%d,primarySda=%d,primaryScl=%d,primaryFreqHz=%lu,secondaryConfigured=%u,secondaryEnabled=%u,secondaryPort=%d,secondarySda=%d,secondaryScl=%d,secondaryFreqHz=%lu\n",
           CONFIG_BOARD_I2C_PORT,
           CONFIG_BOARD_I2C_SDA_GPIO,
           CONFIG_BOARD_I2C_SCL_GPIO,
           (unsigned long)CONFIG_BOARD_I2C0_FREQ_HZ,
           boardSupportI2c1ConfiguredByPins() ? 1u : 0u,
           boardSupportIsI2c1Enabled() ? 1u : 0u,
           CONFIG_BOARD_I2C1_PORT,
           CONFIG_BOARD_I2C1_SDA_GPIO,
           CONFIG_BOARD_I2C1_SCL_GPIO,
           (unsigned long)CONFIG_BOARD_I2C1_FREQ_HZ);

    if (!boardSupportI2cPortValid(CONFIG_BOARD_I2C_PORT)) {
        printf("BOARD_I2C_FATAL,stage=primary_validate,port=%d,sda=%d,scl=%d,reason=invalid_i2c_port\n",
               CONFIG_BOARD_I2C_PORT,
               CONFIG_BOARD_I2C_SDA_GPIO,
               CONFIG_BOARD_I2C_SCL_GPIO);
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPinsValid(CONFIG_BOARD_I2C_SDA_GPIO, CONFIG_BOARD_I2C_SCL_GPIO)) {
        printf("BOARD_I2C_FATAL,stage=primary_validate,port=%d,sda=%d,scl=%d,reason=invalid_i2c_pins\n",
               CONFIG_BOARD_I2C_PORT,
               CONFIG_BOARD_I2C_SDA_GPIO,
               CONFIG_BOARD_I2C_SCL_GPIO);
        return ESP_ERR_INVALID_ARG;
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

    if (!boardSupportI2c1EnableRequested()) {
        printf("BOARD_I2C_INFO,stage=secondary_skip,reason=not_configured\n");
    } else if (!boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO)) {
        printf("BOARD_I2C_WARN,stage=secondary_disabled,port=%d,sda=%d,scl=%d,reason=invalid_i2c_pins\n",
               CONFIG_BOARD_I2C1_PORT,
               CONFIG_BOARD_I2C1_SDA_GPIO,
               CONFIG_BOARD_I2C1_SCL_GPIO);
    } else if (!boardSupportI2cPortValid(CONFIG_BOARD_I2C1_PORT)) {
        printf("BOARD_I2C_WARN,stage=secondary_disabled,port=%d,sda=%d,scl=%d,reason=invalid_i2c_port\n",
               CONFIG_BOARD_I2C1_PORT,
               CONFIG_BOARD_I2C1_SDA_GPIO,
               CONFIG_BOARD_I2C1_SCL_GPIO);
    } else if (CONFIG_BOARD_I2C1_PORT == CONFIG_BOARD_I2C_PORT) {
        printf("BOARD_I2C_WARN,stage=secondary_disabled,port=%d,reason=same_as_primary\n",
               CONFIG_BOARD_I2C1_PORT);
    } else {
        err = boardSupportInitI2c((i2c_port_t)CONFIG_BOARD_I2C1_PORT,
                                  CONFIG_BOARD_I2C1_SDA_GPIO,
                                  CONFIG_BOARD_I2C1_SCL_GPIO,
                                  CONFIG_BOARD_I2C1_FREQ_HZ,
                                  &s_i2c1_configured_freq_hz);
        if (err != ESP_OK) {
            printf("BOARD_I2C_WARN,stage=secondary_init_failed,port=%d,err=%ld,action=primary_only\n",
                   CONFIG_BOARD_I2C1_PORT,
                   (long)err);
        } else {
            s_i2c1_inited = true;
            s_i2c1_ctx.FrequencyHz = s_i2c1_configured_freq_hz;
        }
    }

    s_inited = true;
    printf("BOARD_I2C_READY,primaryReady=%u,secondaryReady=%u,mode=%s\n",
           s_i2c0_inited ? 1u : 0u,
           s_i2c1_inited ? 1u : 0u,
           s_i2c1_inited ? "dual_i2c" : "primary_only");
    return ESP_OK;
}

esp_err_t boardSupportDeinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    if (s_i2c1_inited) {
        if (boardSupportI2cPortValid(CONFIG_BOARD_I2C1_PORT)) {
            esp_err_t deinit_err = i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C1_PORT);
            if (deinit_err != ESP_OK) {
                err = deinit_err;
            }
        } else {
            printf("BOARD_I2C_WARN,stage=deinit_skip,port=%d,reason=invalid_i2c_port\n",
                   CONFIG_BOARD_I2C1_PORT);
        }
        s_i2c1_inited = false;
    }

    if (s_i2c0_inited) {
        if (boardSupportI2cPortValid(CONFIG_BOARD_I2C_PORT)) {
            esp_err_t deinit_err = i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
            if (deinit_err != ESP_OK) {
                err = deinit_err;
            }
        } else {
            printf("BOARD_I2C_WARN,stage=deinit_skip,port=%d,reason=invalid_i2c_port\n",
                   CONFIG_BOARD_I2C_PORT);
        }
        s_i2c0_inited = false;
    }

    s_inited = false;
    return err;
}

bool boardSupportIsI2c1Enabled(void)
{
    return boardSupportI2c1EnableRequested() &&
           boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO) &&
           boardSupportI2cPortValid(CONFIG_BOARD_I2C1_PORT);
}

const BoardSupportI2cCtx_t* boardSupportGetI2cCtx(void)
{
    return &s_i2c0_ctx;
}

const BoardSupportI2cCtx_t* boardSupportGetI2c1Ctx(void)
{
    if (!boardSupportIsI2c1Enabled() || !s_i2c1_inited) {
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
                   boardSupportI2cPortValid(CONFIG_BOARD_I2C_PORT),
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
    const BoardSupportI2cCtx_t* ctx = (const BoardSupportI2cCtx_t*)userCtx;
    if (!boardSupportI2cPortValid((int)ctx->Port)) {
        printf("I2C_REJECT,op=write_read,port=%d,addr=0x%02X,reason=invalid_i2c_port\n",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!boardSupportI2cCtxInstalled(ctx)) {
        printf("I2C_REJECT,op=write_read,port=%d,addr=0x%02X,reason=i2c_driver_not_installed\n",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_STATE;
    }

    int64_t startUs = esp_timer_get_time();
    esp_err_t err = i2c_master_write_read_device(ctx->Port,
                                                 addr7,
                                                 tx,
                                                 txLen,
                                                 rx,
                                                 rxLen,
                                                 boardSupportI2cTimeoutTicks(ctx->TimeoutMs));
    uint32_t elapsedMs = boardSupportI2cElapsedMs(startUs);
    int sda = -1;
    int scl = -1;
    bool stuck = boardSupportI2cBusAppearsStuck(ctx, &sda, &scl);
    boardSupportI2cTraceIfNeeded("write_read", ctx, addr7, err, elapsedMs, sda, scl);
    if (err != ESP_OK) {
        boardSupportI2cHandleTransactionError("write_read", ctx, addr7, err, stuck, sda, scl);
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
    const BoardSupportI2cCtx_t* ctx = (const BoardSupportI2cCtx_t*)userCtx;
    if (!boardSupportI2cPortValid((int)ctx->Port)) {
        printf("I2C_REJECT,op=write,port=%d,addr=0x%02X,reason=invalid_i2c_port\n",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!boardSupportI2cCtxInstalled(ctx)) {
        printf("I2C_REJECT,op=write,port=%d,addr=0x%02X,reason=i2c_driver_not_installed\n",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_STATE;
    }

    int64_t startUs = esp_timer_get_time();
    esp_err_t err = i2c_master_write_to_device(ctx->Port,
                                               addr7,
                                               tx,
                                               txLen,
                                               boardSupportI2cTimeoutTicks(ctx->TimeoutMs));
    uint32_t elapsedMs = boardSupportI2cElapsedMs(startUs);
    int sda = -1;
    int scl = -1;
    bool stuck = boardSupportI2cBusAppearsStuck(ctx, &sda, &scl);
    boardSupportI2cTraceIfNeeded("write", ctx, addr7, err, elapsedMs, sda, scl);
    if (err != ESP_OK) {
        boardSupportI2cHandleTransactionError("write", ctx, addr7, err, stuck, sda, scl);
    }
    return err;
}

esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *i2cCtx, uint8_t addr7)
{
    if (!i2cCtx || addr7 > 0x7Fu) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPortValid((int)i2cCtx->Port)) {
        printf("I2C_REJECT,op=probe,port=%d,addr=0x%02X,reason=invalid_i2c_port\n",
               (int)i2cCtx->Port,
               addr7);
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!boardSupportI2cCtxInstalled(i2cCtx)) {
        printf("I2C_REJECT,op=probe,port=%d,addr=0x%02X,reason=i2c_driver_not_installed\n",
               (int)i2cCtx->Port,
               addr7);
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
        if (err == ESP_FAIL && boardSupportI2cExpectedSecondaryHighAddressNack(i2cCtx, addr7)) {
            printf("I2C_NACK_EXPECTED,port=%d,addr=0x%02X,reason=secondary_addr_low_0x2A\n",
                   (int)i2cCtx->Port,
                   addr7);
            return err;
        }
        printf("I2CPROBE,port=%d,addr=0x%02X,err=%ld,status=%s\n",
               (int)i2cCtx->Port,
               addr7,
               (long)err,
               (err == ESP_FAIL) ? "no_ack_no_recover" : "failed_no_recover");
    }
    return err;
}

// IMPORTANT: Do not define app_main() here.
