#include "boardSupport.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifndef CONFIG_BOARD_I2C_FREQ_HZ
#define CONFIG_BOARD_I2C_FREQ_HZ 337500
#endif
#ifndef CONFIG_BOARD_I2C0_FREQ_HZ
#define CONFIG_BOARD_I2C0_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_BOARD_I2C1_FREQ_HZ
#define CONFIG_BOARD_I2C1_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE
#define CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE 1
#endif
#ifndef CONFIG_BOARD_I2C_PRIMARY_CLK_HZ_DEFAULT
#define CONFIG_BOARD_I2C_PRIMARY_CLK_HZ_DEFAULT 350000
#endif
#ifndef CONFIG_BOARD_I2C_SECONDARY_CLK_HZ_DEFAULT
#define CONFIG_BOARD_I2C_SECONDARY_CLK_HZ_DEFAULT 350000
#endif
#ifndef CONFIG_BOARD_I2C_RUNTIME_FALLBACK_ERROR_FRAMES
#define CONFIG_BOARD_I2C_RUNTIME_FALLBACK_ERROR_FRAMES 3
#endif
#ifndef CONFIG_BOARD_I2C1_ENABLE
#define CONFIG_BOARD_I2C1_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED
#define CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED 1
#endif
#ifndef CONFIG_SENSORARRAY_I2C_RECOVERY_COOLDOWN_MS
#define CONFIG_SENSORARRAY_I2C_RECOVERY_COOLDOWN_MS 1000
#endif
#ifndef CONFIG_SENSORARRAY_I2C_RECOVERY_MAX_FAILS
#define CONFIG_SENSORARRAY_I2C_RECOVERY_MAX_FAILS 3
#endif
#ifndef CONFIG_SENSORARRAY_I2C_RECOVERY_TOGGLE_CLOCKS
#define CONFIG_SENSORARRAY_I2C_RECOVERY_TOGGLE_CLOCKS 9
#endif
#ifndef CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER
#define CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER 0
#endif

#if CONFIG_BOARD_I2C1_ENABLE
#define BOARD_SUPPORT_I2C1_ENABLE_REQUESTED 1
#else
#define BOARD_SUPPORT_I2C1_ENABLE_REQUESTED 0
#endif

#if CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE
#define BOARD_SUPPORT_I2C0_START_HZ CONFIG_BOARD_I2C_PRIMARY_CLK_HZ_DEFAULT
#define BOARD_SUPPORT_I2C1_START_HZ CONFIG_BOARD_I2C_SECONDARY_CLK_HZ_DEFAULT
#else
#define BOARD_SUPPORT_I2C0_START_HZ CONFIG_BOARD_I2C0_FREQ_HZ
#define BOARD_SUPPORT_I2C1_START_HZ CONFIG_BOARD_I2C1_FREQ_HZ
#endif

#define BOARD_SUPPORT_I2C_TIMEOUT_MS 100u
#define BOARD_SUPPORT_I2C_LOCK_TIMEOUT_MS 1000u
#define BOARD_SUPPORT_I2C_RECOVERY_PULSE_US 5u

static bool s_inited = false;
static portMUX_TYPE s_i2c_mutex_create_mux = portMUX_INITIALIZER_UNLOCKED;

static BoardSupportI2cCtx_t s_i2c0_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
    .SdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
    .SclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
    .FrequencyHz = BOARD_SUPPORT_I2C0_START_HZ,
    .Enabled = true,
};

static BoardSupportI2cCtx_t s_i2c1_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
    .SdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
    .SclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
    .FrequencyHz = BOARD_SUPPORT_I2C1_START_HZ,
    .Enabled = BOARD_SUPPORT_I2C1_ENABLE_REQUESTED != 0,
};

static bool boardSupportI2cPortValid(int port)
{
    return port == I2C_NUM_0 || port == I2C_NUM_1;
}

static bool boardSupportI2cIsGpioValid(gpio_num_t gpio)
{
    return gpio >= 0 && gpio < GPIO_NUM_MAX;
}

static bool boardSupportI2cPinsValid(int sdaGpio, int sclGpio)
{
    return boardSupportI2cIsGpioValid((gpio_num_t)sdaGpio) &&
           boardSupportI2cIsGpioValid((gpio_num_t)sclGpio) &&
           sdaGpio != sclGpio;
}

static bool boardSupportI2c1EnableRequested(void)
{
    return BOARD_SUPPORT_I2C1_ENABLE_REQUESTED != 0;
}

static bool boardSupportI2c1ConfiguredByPins(void)
{
    return boardSupportI2c1EnableRequested() &&
           boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO) &&
           boardSupportI2cPortValid(CONFIG_BOARD_I2C1_PORT);
}

static BoardSupportI2cCtx_t *boardSupportI2cWritableCtx(const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return NULL;
    }
    if (ctx == &s_i2c0_ctx ||
        (ctx->Port == s_i2c0_ctx.Port &&
         ctx->SdaGpio == s_i2c0_ctx.SdaGpio &&
         ctx->SclGpio == s_i2c0_ctx.SclGpio)) {
        return &s_i2c0_ctx;
    }
    if (ctx == &s_i2c1_ctx ||
        (ctx->Port == s_i2c1_ctx.Port &&
         ctx->SdaGpio == s_i2c1_ctx.SdaGpio &&
         ctx->SclGpio == s_i2c1_ctx.SclGpio)) {
        return &s_i2c1_ctx;
    }
    return (BoardSupportI2cCtx_t *)ctx;
}

static const char *boardSupportI2cBusStateName(BoardSupportI2cBusState_t state)
{
    switch (state) {
    case BOARD_I2C_BUS_OK:
        return "OK";
    case BOARD_I2C_BUS_BUSY:
        return "BUSY";
    case BOARD_I2C_BUS_SDA_LOW:
        return "SDA_LOW";
    case BOARD_I2C_BUS_SCL_LOW:
        return "SCL_LOW";
    case BOARD_I2C_BUS_BOTH_LOW:
        return "BOTH_LOW";
    case BOARD_I2C_BUS_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

static TickType_t boardSupportI2cMsToTicksAtLeastOne(uint32_t timeoutMs)
{
    TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
    return (ticks == 0) ? 1 : ticks;
}

static esp_err_t boardSupportI2cEnsureMutex(BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ctx->Mutex) {
        return ESP_OK;
    }

    portENTER_CRITICAL(&s_i2c_mutex_create_mux);
    if (!ctx->Mutex) {
        ctx->Mutex = xSemaphoreCreateMutex();
    }
    portEXIT_CRITICAL(&s_i2c_mutex_create_mux);

    return ctx->Mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t boardSupportI2cLock(BoardSupportI2cCtx_t *ctx, TickType_t timeoutTicks)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ctx->Mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    int64_t waitStartUs = esp_timer_get_time();
    if (xSemaphoreTake(ctx->Mutex, timeoutTicks) != pdTRUE) {
        uint64_t waitUs = (uint64_t)(esp_timer_get_time() - waitStartUs);
        ctx->BusyWaitUs += waitUs;
        ctx->LockContentionCount++;
        return ESP_ERR_TIMEOUT;
    }
    uint64_t waitUs = (uint64_t)(esp_timer_get_time() - waitStartUs);
    ctx->BusyWaitUs += waitUs;
    if (waitUs > 100u) {
        ctx->LockContentionCount++;
    }
    return ESP_OK;
}

static void boardSupportI2cUnlock(BoardSupportI2cCtx_t *ctx)
{
    if (ctx && ctx->Mutex) {
        xSemaphoreGive(ctx->Mutex);
    }
}

static esp_err_t boardSupportI2cReadPins(const BoardSupportI2cCtx_t *ctx,
                                         int *outSdaLevel,
                                         int *outSclLevel)
{
    if (outSdaLevel) {
        *outSdaLevel = -1;
    }
    if (outSclLevel) {
        *outSclLevel = -1;
    }
    if (!ctx || !boardSupportI2cPinsValid(ctx->SdaGpio, ctx->SclGpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    int sda = gpio_get_level((gpio_num_t)ctx->SdaGpio);
    int scl = gpio_get_level((gpio_num_t)ctx->SclGpio);
    if (outSdaLevel) {
        *outSdaLevel = sda;
    }
    if (outSclLevel) {
        *outSclLevel = scl;
    }
    return ESP_OK;
}

static BoardSupportI2cBusState_t boardSupportI2cClassifyBusState(const BoardSupportI2cCtx_t *ctx,
                                                                 int *outSdaLevel,
                                                                 int *outSclLevel)
{
    int sda = -1;
    int scl = -1;
    esp_err_t err = boardSupportI2cReadPins(ctx, &sda, &scl);
    if (outSdaLevel) {
        *outSdaLevel = sda;
    }
    if (outSclLevel) {
        *outSclLevel = scl;
    }
    if (err != ESP_OK) {
        return BOARD_I2C_BUS_UNKNOWN;
    }
    if (sda == 0 && scl == 0) {
        return BOARD_I2C_BUS_BOTH_LOW;
    }
    if (sda == 0) {
        return BOARD_I2C_BUS_SDA_LOW;
    }
    if (scl == 0) {
        return BOARD_I2C_BUS_SCL_LOW;
    }
    return BOARD_I2C_BUS_OK;
}

static bool boardSupportI2cBusStateStuck(BoardSupportI2cBusState_t state)
{
    return state == BOARD_I2C_BUS_SDA_LOW ||
           state == BOARD_I2C_BUS_SCL_LOW ||
           state == BOARD_I2C_BUS_BOTH_LOW;
}

static esp_err_t boardSupportI2cValidateCtx(const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPortValid((int)ctx->Port)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!boardSupportI2cPinsValid(ctx->SdaGpio, ctx->SclGpio)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ctx->FrequencyHz == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static void boardSupportI2cLogMem(const char *stage, const BoardSupportI2cCtx_t *ctx, esp_err_t err)
{
    printf("BOARD_I2C_MEM,stage=%s,port=%d,err=%ld,stackHighWaterWords=%u,freeHeap=%u,minFreeHeap=%u\n",
           stage ? stage : "na",
           ctx ? (int)ctx->Port : -1,
           (long)err,
           (unsigned)uxTaskGetStackHighWaterMark(NULL),
           (unsigned)esp_get_free_heap_size(),
           (unsigned)esp_get_minimum_free_heap_size());
}

static esp_err_t boardSupportInitI2cLocked(BoardSupportI2cCtx_t *ctx)
{
    esp_err_t err = boardSupportI2cValidateCtx(ctx);
    if (err != ESP_OK) {
        printf("BOARD_I2C_FATAL,stage=validate,port=%d,sda=%d,scl=%d,freqHz=%lu,reason=invalid_config,err=%ld\n",
               ctx ? (int)ctx->Port : -1,
               ctx ? ctx->SdaGpio : -1,
               ctx ? ctx->SclGpio : -1,
               ctx ? (unsigned long)ctx->FrequencyHz : 0ul,
               (long)err);
        return err;
    }

    printf("BOARD_I2C_INIT,stage=begin,port=%d,sda=%d,scl=%d,requestedFreqHz=%lu\n",
           (int)ctx->Port,
           ctx->SdaGpio,
           ctx->SclGpio,
           (unsigned long)ctx->FrequencyHz);

    if (ctx->Installed) {
        printf("BOARD_I2C_INIT,stage=skip_already_installed,port=%d\n",
               (int)ctx->Port);
        return ESP_OK;
    }

    i2c_config_t cfg = {0};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = ctx->SdaGpio;
    cfg.scl_io_num = ctx->SclGpio;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = ctx->FrequencyHz;
#ifdef SOC_I2C_SUPPORT_CLK_SRC
    cfg.clk_flags = 0;
#endif

    err = i2c_param_config(ctx->Port, &cfg);
    printf("BOARD_I2C_INIT,stage=param_config_done,port=%d,err=%ld\n",
           (int)ctx->Port,
           (long)err);
    if (err != ESP_OK) {
        ctx->Installed = false;
        return err;
    }

    boardSupportI2cLogMem("before_driver_install", ctx, ESP_OK);
    printf("BOARD_I2C_INIT,stage=driver_install,port=%d\n", (int)ctx->Port);
    err = i2c_driver_install(ctx->Port, I2C_MODE_MASTER, 0, 0, 0);
    boardSupportI2cLogMem("after_driver_install", ctx, err);

    if (err == ESP_OK) {
        ctx->Installed = true;
        ctx->Offline = false;
        ctx->FrequencyHz = cfg.master.clk_speed;
        ctx->InstallCount++;
        printf("BOARD_I2C_INIT,stage=done,port=%d,err=0,installCount=%lu\n",
               (int)ctx->Port,
               (unsigned long)ctx->InstallCount);
    } else {
        ctx->Installed = false;
        printf("BOARD_I2C_INIT,stage=driver_install_failed,port=%d,err=%ld\n",
               (int)ctx->Port,
               (long)err);
    }
    return err;
}

static esp_err_t boardSupportInitI2c(BoardSupportI2cCtx_t *ctx)
{
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }

    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(BOARD_SUPPORT_I2C_LOCK_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportInitI2cLocked(ctx);
    boardSupportI2cUnlock(ctx);
    return err;
}

static void boardSupportI2cMarkOffline(BoardSupportI2cCtx_t *ctx, const char *reason)
{
    if (!ctx || ctx->Offline) {
        return;
    }
    ctx->Offline = true;
    printf("BOARD_I2C_OFFLINE,port=%d,reason=%s,recoveryFailCount=%lu,timeoutCount=%lu,busStuckCount=%lu\n",
           (int)ctx->Port,
           reason ? reason : "unknown",
           (unsigned long)ctx->RecoveryFailCount,
           (unsigned long)ctx->TimeoutCount,
           (unsigned long)ctx->BusStuckCount);
}

static bool boardSupportI2cShouldRecover(BoardSupportI2cCtx_t *ctx,
                                         esp_err_t err,
                                         BoardSupportI2cBusState_t busState,
                                         const char **outAction)
{
    if (outAction) {
        *outAction = "no_recover";
    }
    if (!ctx) {
        return false;
    }
    if (!CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED) {
        if (outAction) {
            *outAction = "no_recover_config_disabled";
        }
        return false;
    }
    if (ctx->Offline) {
        if (outAction) {
            *outAction = "no_recover_offline";
        }
        return false;
    }
    if (ctx->Recovering) {
        if (outAction) {
            *outAction = "no_recover_already_recovering";
        }
        return false;
    }
    if (err != ESP_ERR_TIMEOUT) {
        if (outAction) {
            *outAction = (err == ESP_FAIL) ? "no_recover_device_nack" : "no_recover_non_timeout";
        }
        return false;
    }
    if (!boardSupportI2cBusStateStuck(busState)) {
        if (outAction) {
            *outAction = "no_recover_timeout_bus_ok";
        }
        return false;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_I2C_RECOVERY_COOLDOWN_MS * 1000LL;
    if (ctx->LastRecoveryUs != 0 && cooldownUs > 0 && (nowUs - ctx->LastRecoveryUs) < cooldownUs) {
        if (outAction) {
            *outAction = "no_recover_cooldown";
        }
        return false;
    }

    if (outAction) {
        *outAction = "recover_attempt";
    }
    return true;
}

static const char *boardSupportI2cErrKind(esp_err_t err)
{
    if (err == ESP_OK) {
        return "ok";
    }
    if (err == ESP_FAIL) {
        return "nack";
    }
    if (err == ESP_ERR_TIMEOUT) {
        return "timeout";
    }
    return "error";
}

static void boardSupportI2cHandleTransactionError(BoardSupportI2cCtx_t *ctx,
                                                  uint8_t addr7,
                                                  const char *op,
                                                  esp_err_t err,
                                                  BoardSupportI2cBusState_t busState)
{
    if (!ctx || err == ESP_OK) {
        return;
    }

    const char *action = "no_recover";
    bool doRecover = boardSupportI2cShouldRecover(ctx, err, busState, &action);
    printf("BOARD_I2C_ERR,port=%d,addr=0x%02X,op=%s,err=0x%lx,kind=%s,busState=%s,recoverAction=%s\n",
           (int)ctx->Port,
           addr7,
           op ? op : "unknown",
           (unsigned long)err,
           boardSupportI2cErrKind(err),
           boardSupportI2cBusStateName(busState),
           action);
    if (doRecover) {
        (void)boardSupportRecoverI2cBus(ctx);
    }
}

static esp_err_t boardSupportI2cCheckReadyLocked(BoardSupportI2cCtx_t *ctx,
                                                 uint8_t addr7,
                                                 const char *op)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ctx->Installed) {
        printf("I2C_REJECT,op=%s,port=%d,addr=0x%02X,reason=i2c_driver_not_installed\n",
               op ? op : "unknown",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_STATE;
    }
    if (ctx->Offline) {
        printf("I2C_REJECT,op=%s,port=%d,addr=0x%02X,reason=bus_offline\n",
               op ? op : "unknown",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_STATE;
    }
    if (ctx->Recovering) {
        printf("I2C_REJECT,op=%s,port=%d,addr=0x%02X,reason=bus_recovering\n",
               op ? op : "unknown",
               (int)ctx->Port,
               addr7);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t boardSupportI2cFinishTransaction(BoardSupportI2cCtx_t *ctx,
                                                  uint8_t addr7,
                                                  const char *op,
                                                  esp_err_t err,
                                                  int64_t startUs)
{
    int sda = -1;
    int scl = -1;
    BoardSupportI2cBusState_t busState = boardSupportI2cClassifyBusState(ctx, &sda, &scl);
    if (err == ESP_FAIL) {
        ctx->NackCount++;
    } else if (err == ESP_ERR_TIMEOUT) {
        ctx->TimeoutCount++;
    }
    if (boardSupportI2cBusStateStuck(busState)) {
        ctx->BusStuckCount++;
    }
    ctx->LastTransactionUs = esp_timer_get_time();
    int64_t elapsedUs = ctx->LastTransactionUs - startUs;
    if (CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER || err != ESP_OK) {
        printf("BOARD_I2C_XFER,stage=end,port=%d,addr=0x%02X,op=%s,err=0x%lx,elapsedUs=%lld,sda=%d,scl=%d,busState=%s,nackCount=%lu,timeoutCount=%lu,offline=%u\n",
               (int)ctx->Port,
               addr7,
               op ? op : "unknown",
               (unsigned long)err,
               (long long)elapsedUs,
               sda,
               scl,
               boardSupportI2cBusStateName(busState),
               (unsigned long)ctx->NackCount,
               (unsigned long)ctx->TimeoutCount,
               ctx->Offline ? 1u : 0u);
    }
    boardSupportI2cUnlock(ctx);
    boardSupportI2cHandleTransactionError(ctx, addr7, op, err, busState);
    return err;
}

static esp_err_t boardSupportI2cConfigureRecoveryPins(BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << (uint32_t)ctx->SdaGpio) |
                        (1ULL << (uint32_t)ctx->SclGpio),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io);
}

static void boardSupportI2cPulseRecoveryClock(BoardSupportI2cCtx_t *ctx)
{
    uint32_t toggles = (uint32_t)CONFIG_SENSORARRAY_I2C_RECOVERY_TOGGLE_CLOCKS;
    if (toggles == 0u) {
        toggles = 9u;
    }
    gpio_set_level((gpio_num_t)ctx->SdaGpio, 1);
    gpio_set_level((gpio_num_t)ctx->SclGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_PULSE_US);
    for (uint32_t i = 0; i < toggles; ++i) {
        gpio_set_level((gpio_num_t)ctx->SclGpio, 0);
        esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_PULSE_US);
        gpio_set_level((gpio_num_t)ctx->SclGpio, 1);
        esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_PULSE_US);
    }

    gpio_set_level((gpio_num_t)ctx->SdaGpio, 0);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_PULSE_US);
    gpio_set_level((gpio_num_t)ctx->SclGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_PULSE_US);
    gpio_set_level((gpio_num_t)ctx->SdaGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_PULSE_US);
}

esp_err_t boardSupportRecoverI2cBus(const BoardSupportI2cCtx_t *userCtx)
{
    BoardSupportI2cCtx_t *ctx = boardSupportI2cWritableCtx(userCtx);
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(BOARD_SUPPORT_I2C_LOCK_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    if (!CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED) {
        printf("BOARD_I2C_RECOVERY,stage=skip,port=%d,reason=config_disabled\n",
               (int)ctx->Port);
        boardSupportI2cUnlock(ctx);
        return ESP_ERR_INVALID_STATE;
    }
    if (ctx->Recovering) {
        printf("BOARD_I2C_RECOVERY,stage=skip,port=%d,reason=already_recovering\n",
               (int)ctx->Port);
        boardSupportI2cUnlock(ctx);
        return ESP_ERR_INVALID_STATE;
    }
    if (ctx->Offline) {
        printf("BOARD_I2C_RECOVERY,stage=skip,port=%d,reason=offline\n",
               (int)ctx->Port);
        boardSupportI2cUnlock(ctx);
        return ESP_ERR_INVALID_STATE;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_I2C_RECOVERY_COOLDOWN_MS * 1000LL;
    if (ctx->LastRecoveryUs != 0 && cooldownUs > 0 && (nowUs - ctx->LastRecoveryUs) < cooldownUs) {
        printf("BOARD_I2C_RECOVERY,stage=skip,port=%d,reason=cooldown,remainingUs=%lld\n",
               (int)ctx->Port,
               (long long)(cooldownUs - (nowUs - ctx->LastRecoveryUs)));
        boardSupportI2cUnlock(ctx);
        return ESP_ERR_TIMEOUT;
    }

    int sda = -1;
    int scl = -1;
    BoardSupportI2cBusState_t state = boardSupportI2cClassifyBusState(ctx, &sda, &scl);
    if (!boardSupportI2cBusStateStuck(state)) {
        printf("BOARD_I2C_RECOVERY,stage=skip,port=%d,reason=bus_not_stuck,sda=%d,scl=%d,busState=%s\n",
               (int)ctx->Port,
               sda,
               scl,
               boardSupportI2cBusStateName(state));
        boardSupportI2cUnlock(ctx);
        return ESP_OK;
    }

    ctx->Recovering = true;
    ctx->RecoveryCount++;
    ctx->LastRecoveryUs = nowUs;
    printf("BOARD_I2C_RECOVERY,stage=begin,port=%d,sda=%d,scl=%d,busState=%s,recoveryCount=%lu\n",
           (int)ctx->Port,
           sda,
           scl,
           boardSupportI2cBusStateName(state),
           (unsigned long)ctx->RecoveryCount);

    if (ctx->Installed) {
        esp_err_t deleteErr = i2c_driver_delete(ctx->Port);
        if (deleteErr == ESP_OK || deleteErr == ESP_ERR_INVALID_STATE) {
            ctx->Installed = false;
            ctx->DeleteCount++;
        }
        printf("BOARD_I2C_RECOVERY,stage=driver_delete,port=%d,err=%ld,deleteCount=%lu\n",
               (int)ctx->Port,
               (long)deleteErr,
               (unsigned long)ctx->DeleteCount);
    }

    err = boardSupportI2cConfigureRecoveryPins(ctx);
    if (err == ESP_OK) {
        boardSupportI2cPulseRecoveryClock(ctx);
    }
    state = boardSupportI2cClassifyBusState(ctx, &sda, &scl);
    if (err != ESP_OK || boardSupportI2cBusStateStuck(state)) {
        ctx->RecoveryFailCount++;
        if (ctx->RecoveryFailCount >= (uint32_t)CONFIG_SENSORARRAY_I2C_RECOVERY_MAX_FAILS) {
            boardSupportI2cMarkOffline(ctx, "recovery_failed_max");
        }
        printf("BOARD_I2C_RECOVERY,stage=failed,port=%d,err=%ld,sda=%d,scl=%d,busState=%s,recoveryFailCount=%lu\n",
               (int)ctx->Port,
               (long)err,
               sda,
               scl,
               boardSupportI2cBusStateName(state),
               (unsigned long)ctx->RecoveryFailCount);
        ctx->Recovering = false;
        boardSupportI2cUnlock(ctx);
        return (err != ESP_OK) ? err : ESP_ERR_TIMEOUT;
    }

    err = boardSupportInitI2cLocked(ctx);
    if (err == ESP_OK) {
        ctx->Offline = false;
        printf("BOARD_I2C_RECOVERY,stage=done,port=%d,err=0,installCount=%lu\n",
               (int)ctx->Port,
               (unsigned long)ctx->InstallCount);
    } else {
        ctx->RecoveryFailCount++;
        if (ctx->RecoveryFailCount >= (uint32_t)CONFIG_SENSORARRAY_I2C_RECOVERY_MAX_FAILS) {
            boardSupportI2cMarkOffline(ctx, "reinstall_failed_max");
        }
        printf("BOARD_I2C_RECOVERY,stage=reinstall_failed,port=%d,err=%ld,recoveryFailCount=%lu\n",
               (int)ctx->Port,
               (long)err,
               (unsigned long)ctx->RecoveryFailCount);
    }

    ctx->Recovering = false;
    boardSupportI2cUnlock(ctx);
    return err;
}

esp_err_t boardSupportSetI2cFrequency(const BoardSupportI2cCtx_t *userCtx, uint32_t frequencyHz)
{
    BoardSupportI2cCtx_t *ctx = boardSupportI2cWritableCtx(userCtx);
    if (!ctx || frequencyHz == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(BOARD_SUPPORT_I2C_LOCK_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    if (ctx->Installed && ctx->FrequencyHz == frequencyHz && !ctx->Offline) {
        boardSupportI2cUnlock(ctx);
        return ESP_OK;
    }

    uint32_t previousHz = ctx->FrequencyHz;
    if (ctx->Installed) {
        err = i2c_driver_delete(ctx->Port);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            printf("BOARD_I2C_FREQ,stage=driver_delete_failed,port=%d,from=%lu,to=%lu,err=%ld\n",
                   (int)ctx->Port,
                   (unsigned long)previousHz,
                   (unsigned long)frequencyHz,
                   (long)err);
            boardSupportI2cUnlock(ctx);
            return err;
        }
        ctx->Installed = false;
        ctx->DeleteCount++;
    }

    ctx->FrequencyHz = frequencyHz;
    ctx->Offline = false;
    err = boardSupportInitI2cLocked(ctx);
    printf("BOARD_I2C_FREQ,stage=reconfigure,port=%d,from=%lu,to=%lu,err=%ld,installCount=%lu,deleteCount=%lu\n",
           (int)ctx->Port,
           (unsigned long)previousHz,
           (unsigned long)frequencyHz,
           (long)err,
           (unsigned long)ctx->InstallCount,
           (unsigned long)ctx->DeleteCount);
    boardSupportI2cUnlock(ctx);
    return err;
}

esp_err_t boardSupportInit(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    s_i2c0_ctx = (BoardSupportI2cCtx_t){
        .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
        .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
        .SdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
        .SclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
        .FrequencyHz = BOARD_SUPPORT_I2C0_START_HZ,
        .Enabled = true,
        .Mutex = s_i2c0_ctx.Mutex,
    };
    s_i2c1_ctx = (BoardSupportI2cCtx_t){
        .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
        .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
        .SdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
        .SclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
        .FrequencyHz = BOARD_SUPPORT_I2C1_START_HZ,
        .Enabled = boardSupportI2c1ConfiguredByPins(),
        .Mutex = s_i2c1_ctx.Mutex,
    };

    printf("BOARD_I2C_CFG,primaryPort=%d,primarySda=%d,primaryScl=%d,primaryFreqHz=%lu,secondaryConfigured=%u,secondaryEnabled=%u,secondaryPort=%d,secondarySda=%d,secondaryScl=%d,secondaryFreqHz=%lu,autoFallback=%u,fallbackLevels=%s\n",
           CONFIG_BOARD_I2C_PORT,
           CONFIG_BOARD_I2C_SDA_GPIO,
           CONFIG_BOARD_I2C_SCL_GPIO,
           (unsigned long)BOARD_SUPPORT_I2C0_START_HZ,
           boardSupportI2c1ConfiguredByPins() ? 1u : 0u,
           boardSupportIsI2c1Enabled() ? 1u : 0u,
           CONFIG_BOARD_I2C1_PORT,
           CONFIG_BOARD_I2C1_SDA_GPIO,
           CONFIG_BOARD_I2C1_SCL_GPIO,
           (unsigned long)BOARD_SUPPORT_I2C1_START_HZ,
           CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE ? 1u : 0u,
#ifdef CONFIG_BOARD_I2C_FALLBACK_LEVELS
           CONFIG_BOARD_I2C_FALLBACK_LEVELS
#else
           "350000,337500,325000,300000"
#endif
           );

    esp_err_t err = boardSupportInitI2c(&s_i2c0_ctx);
    if (err != ESP_OK) {
        return err;
    }

    if (!boardSupportI2c1EnableRequested()) {
        printf("BOARD_I2C_INFO,stage=secondary_skip,reason=not_configured\n");
    } else if (!boardSupportI2c1ConfiguredByPins()) {
        printf("BOARD_I2C_WARN,stage=secondary_disabled,port=%d,sda=%d,scl=%d,reason=invalid_config\n",
               CONFIG_BOARD_I2C1_PORT,
               CONFIG_BOARD_I2C1_SDA_GPIO,
               CONFIG_BOARD_I2C1_SCL_GPIO);
        s_i2c1_ctx.Enabled = false;
    } else if (CONFIG_BOARD_I2C1_PORT == CONFIG_BOARD_I2C_PORT) {
        printf("BOARD_I2C_WARN,stage=secondary_disabled,port=%d,reason=same_as_primary\n",
               CONFIG_BOARD_I2C1_PORT);
        s_i2c1_ctx.Enabled = false;
    } else {
        err = boardSupportInitI2c(&s_i2c1_ctx);
        if (err != ESP_OK) {
            printf("BOARD_I2C_WARN,stage=secondary_init_failed,port=%d,err=%ld,action=primary_only\n",
                   CONFIG_BOARD_I2C1_PORT,
                   (long)err);
            s_i2c1_ctx.Enabled = false;
        }
    }

    s_inited = true;
    printf("BOARD_I2C_READY,primaryReady=%u,secondaryReady=%u,mode=%s\n",
           s_i2c0_ctx.Installed ? 1u : 0u,
           s_i2c1_ctx.Installed ? 1u : 0u,
           s_i2c1_ctx.Installed ? "dual_i2c" : "primary_only");
    return ESP_OK;
}

esp_err_t boardSupportDeinit(void)
{
    esp_err_t firstErr = ESP_OK;
    BoardSupportI2cCtx_t *ctxs[2] = {&s_i2c1_ctx, &s_i2c0_ctx};
    for (size_t i = 0; i < 2; ++i) {
        BoardSupportI2cCtx_t *ctx = ctxs[i];
        if (!ctx->Installed) {
            continue;
        }
        esp_err_t err = boardSupportI2cEnsureMutex(ctx);
        if (err == ESP_OK) {
            err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(BOARD_SUPPORT_I2C_LOCK_TIMEOUT_MS));
        }
        if (err == ESP_OK) {
            err = i2c_driver_delete(ctx->Port);
            if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
                ctx->Installed = false;
                ctx->DeleteCount++;
            }
            boardSupportI2cUnlock(ctx);
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    s_inited = false;
    return firstErr;
}

bool boardSupportIsI2c1Enabled(void)
{
    return boardSupportI2c1EnableRequested() &&
           boardSupportI2c1ConfiguredByPins() &&
           CONFIG_BOARD_I2C1_PORT != CONFIG_BOARD_I2C_PORT;
}

const BoardSupportI2cCtx_t *boardSupportGetI2cCtx(void)
{
    return &s_i2c0_ctx;
}

const BoardSupportI2cCtx_t *boardSupportGetI2c1Ctx(void)
{
    if (!s_i2c1_ctx.Enabled || !s_i2c1_ctx.Installed || s_i2c1_ctx.Offline) {
        return NULL;
    }
    return &s_i2c1_ctx;
}

bool boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *outInfo)
{
    if (!outInfo) {
        return false;
    }

    const BoardSupportI2cCtx_t *ctx = secondary ? &s_i2c1_ctx : &s_i2c0_ctx;
    *outInfo = (BoardSupportI2cBusInfo_t){
        .Enabled = ctx->Enabled,
        .Installed = ctx->Installed,
        .Offline = ctx->Offline,
        .Recovering = ctx->Recovering,
        .Port = ctx->Port,
        .SdaGpio = ctx->SdaGpio,
        .SclGpio = ctx->SclGpio,
        .FrequencyHz = ctx->FrequencyHz,
        .InstallCount = ctx->InstallCount,
        .DeleteCount = ctx->DeleteCount,
        .TransactionCount = ctx->TransactionCount,
        .NackCount = ctx->NackCount,
        .TimeoutCount = ctx->TimeoutCount,
        .BusStuckCount = ctx->BusStuckCount,
        .RecoveryCount = ctx->RecoveryCount,
        .RecoveryFailCount = ctx->RecoveryFailCount,
        .BusyWaitUs = ctx->BusyWaitUs,
        .LockContentionCount = ctx->LockContentionCount,
    };
    return true;
}

esp_err_t boardSupportI2cWriteRead(void *userCtx,
                                  uint8_t addr7,
                                  const uint8_t *tx,
                                  size_t txLen,
                                  uint8_t *rx,
                                  size_t rxLen)
{
    if (!userCtx || !tx || txLen == 0 || !rx || rxLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    BoardSupportI2cCtx_t *ctx = boardSupportI2cWritableCtx((const BoardSupportI2cCtx_t *)userCtx);
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cCheckReadyLocked(ctx, addr7, "write_read");
    if (err != ESP_OK) {
        boardSupportI2cUnlock(ctx);
        return err;
    }

    ctx->TransactionCount++;
    int64_t startUs = esp_timer_get_time();
    if (CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER) {
        printf("BOARD_I2C_XFER,stage=begin,port=%d,addr=0x%02X,op=write_read,txLen=%u,rxLen=%u,count=%lu\n",
               (int)ctx->Port,
               addr7,
               (unsigned)txLen,
               (unsigned)rxLen,
               (unsigned long)ctx->TransactionCount);
    }
    err = i2c_master_write_read_device(ctx->Port,
                                       addr7,
                                       tx,
                                       txLen,
                                       rx,
                                       rxLen,
                                       boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    return boardSupportI2cFinishTransaction(ctx, addr7, "write_read", err, startUs);
}

esp_err_t boardSupportI2cWrite(void *userCtx,
                              uint8_t addr7,
                              const uint8_t *tx,
                              size_t txLen)
{
    if (!userCtx || !tx || txLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    BoardSupportI2cCtx_t *ctx = boardSupportI2cWritableCtx((const BoardSupportI2cCtx_t *)userCtx);
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cCheckReadyLocked(ctx, addr7, "write");
    if (err != ESP_OK) {
        boardSupportI2cUnlock(ctx);
        return err;
    }

    ctx->TransactionCount++;
    int64_t startUs = esp_timer_get_time();
    if (CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER) {
        printf("BOARD_I2C_XFER,stage=begin,port=%d,addr=0x%02X,op=write,txLen=%u,rxLen=0,count=%lu\n",
               (int)ctx->Port,
               addr7,
               (unsigned)txLen,
               (unsigned long)ctx->TransactionCount);
    }
    err = i2c_master_write_to_device(ctx->Port,
                                     addr7,
                                     tx,
                                     txLen,
                                     boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    return boardSupportI2cFinishTransaction(ctx, addr7, "write", err, startUs);
}

esp_err_t boardSupportI2cRead(void *userCtx,
                             uint8_t addr7,
                             uint8_t *rx,
                             size_t rxLen)
{
    if (!userCtx || !rx || rxLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    BoardSupportI2cCtx_t *ctx = boardSupportI2cWritableCtx((const BoardSupportI2cCtx_t *)userCtx);
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cCheckReadyLocked(ctx, addr7, "read");
    if (err != ESP_OK) {
        boardSupportI2cUnlock(ctx);
        return err;
    }

    ctx->TransactionCount++;
    int64_t startUs = esp_timer_get_time();
    if (CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER) {
        printf("BOARD_I2C_XFER,stage=begin,port=%d,addr=0x%02X,op=read,txLen=0,rxLen=%u,count=%lu\n",
               (int)ctx->Port,
               addr7,
               (unsigned)rxLen,
               (unsigned long)ctx->TransactionCount);
    }
    err = i2c_master_read_from_device(ctx->Port,
                                      addr7,
                                      rx,
                                      rxLen,
                                      boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    return boardSupportI2cFinishTransaction(ctx, addr7, "read", err, startUs);
}

static bool boardSupportI2cIsDataAll1Sequence(const uint8_t *regs, size_t regCount)
{
    if (!regs || regCount != BOARD_SUPPORT_I2C_SEQUENCE_MAX_REGS) {
        return false;
    }
    for (size_t i = 0u; i < regCount; ++i) {
        if (regs[i] != (uint8_t)i) {
            return false;
        }
    }
    return true;
}

static esp_err_t boardSupportI2cBuildReg16SequenceCommand(i2c_cmd_handle_t cmd,
                                                           uint8_t addr7,
                                                           const uint8_t *regs,
                                                           size_t regCount,
                                                           uint8_t *rx)
{
    if (!cmd || !regs || !rx) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    for (size_t i = 0u; i < regCount && err == ESP_OK; ++i) {
        err = i2c_master_start(cmd);
        if (err == ESP_OK) {
            err = i2c_master_write_byte(cmd,
                                        (uint8_t)((addr7 << 1u) | I2C_MASTER_WRITE),
                                        true);
        }
        if (err == ESP_OK) {
            err = i2c_master_write_byte(cmd, regs[i], true);
        }
        if (err == ESP_OK) {
            err = i2c_master_start(cmd);
        }
        if (err == ESP_OK) {
            err = i2c_master_write_byte(cmd,
                                        (uint8_t)((addr7 << 1u) | I2C_MASTER_READ),
                                        true);
        }
        if (err == ESP_OK) {
            err = i2c_master_read(cmd, &rx[i * 2u], 2u, I2C_MASTER_LAST_NACK);
        }
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }
    return err;
}

esp_err_t boardSupportI2cReadReg16Sequence(void *userCtx,
                                          uint8_t addr7,
                                          const uint8_t *regs,
                                          size_t regCount,
                                          uint16_t *outValues,
                                          uint32_t timeoutMs)
{
    if (!userCtx || !regs || !outValues || regCount == 0u ||
        regCount > BOARD_SUPPORT_I2C_SEQUENCE_MAX_REGS || addr7 > 0x7fu) {
        return ESP_ERR_INVALID_ARG;
    }

    BoardSupportI2cCtx_t *ctx =
        boardSupportI2cWritableCtx((const BoardSupportI2cCtx_t *)userCtx);
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    uint32_t effectiveTimeoutMs = timeoutMs != 0u ? timeoutMs : ctx->TimeoutMs;
    TickType_t timeoutTicks = boardSupportI2cMsToTicksAtLeastOne(effectiveTimeoutMs);
    err = boardSupportI2cLock(ctx, timeoutTicks);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cCheckReadyLocked(ctx, addr7, "read_reg16_sequence");
    if (err != ESP_OK) {
        boardSupportI2cUnlock(ctx);
        return err;
    }

    bool cachedDataSequence = boardSupportI2cIsDataAll1Sequence(regs, regCount);
    uint8_t rx[BOARD_SUPPORT_I2C_SEQUENCE_MAX_REGS * 2u] = {0};
    uint8_t cmdBuffer[I2C_LINK_RECOMMENDED_SIZE(BOARD_SUPPORT_I2C_SEQUENCE_LINK_TRANSACTIONS)] = {0};
    uint8_t *activeRx = rx;
    i2c_cmd_handle_t cmd = NULL;
    if (cachedDataSequence) {
        if (!ctx->DataSequenceCmdReady || ctx->DataSequenceAddr7 != addr7) {
            ctx->DataSequenceCmdReady = false;
            ctx->DataSequenceCmd = NULL;
            memset(ctx->DataSequenceCmdBuffer, 0, sizeof(ctx->DataSequenceCmdBuffer));
            memset(ctx->DataSequenceRx, 0, sizeof(ctx->DataSequenceRx));
            ctx->DataSequenceCmd =
                i2c_cmd_link_create_static(ctx->DataSequenceCmdBuffer,
                                           sizeof(ctx->DataSequenceCmdBuffer));
            if (ctx->DataSequenceCmd) {
                err = boardSupportI2cBuildReg16SequenceCommand(ctx->DataSequenceCmd,
                                                               addr7,
                                                               regs,
                                                               regCount,
                                                               ctx->DataSequenceRx);
            } else {
                err = ESP_ERR_NO_MEM;
            }
            if (err == ESP_OK) {
                ctx->DataSequenceAddr7 = addr7;
                ctx->DataSequenceCmdReady = true;
            }
        }
        cmd = ctx->DataSequenceCmdReady ? ctx->DataSequenceCmd : NULL;
        activeRx = ctx->DataSequenceRx;
    } else {
        cmd = i2c_cmd_link_create_static(cmdBuffer, sizeof(cmdBuffer));
        if (cmd) {
            err = boardSupportI2cBuildReg16SequenceCommand(cmd,
                                                           addr7,
                                                           regs,
                                                           regCount,
                                                           rx);
        } else {
            err = ESP_ERR_NO_MEM;
        }
    }
    if (!cmd || err != ESP_OK) {
        if (!cachedDataSequence && cmd) {
            i2c_cmd_link_delete_static(cmd);
        }
        boardSupportI2cUnlock(ctx);
        return err != ESP_OK ? err : ESP_ERR_NO_MEM;
    }

    ctx->TransactionCount++;
    int64_t startUs = esp_timer_get_time();
    if (CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER) {
        printf("BOARD_I2C_XFER,stage=begin,port=%d,addr=0x%02X,op=read_reg16_sequence,regs=%u,rxLen=%u,cached=%u,count=%lu\n",
               (int)ctx->Port,
               addr7,
               (unsigned)regCount,
               (unsigned)(regCount * 2u),
               cachedDataSequence ? 1u : 0u,
               (unsigned long)ctx->TransactionCount);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(ctx->Port, cmd, timeoutTicks);
    }
    if (!cachedDataSequence) {
        i2c_cmd_link_delete_static(cmd);
    }

    if (err == ESP_OK) {
        for (size_t i = 0u; i < regCount; ++i) {
            outValues[i] = (uint16_t)(((uint16_t)activeRx[i * 2u] << 8u) |
                                      activeRx[i * 2u + 1u]);
        }
    }
    return boardSupportI2cFinishTransaction(ctx,
                                            addr7,
                                            "read_reg16_sequence",
                                            err,
                                            startUs);
}

static bool boardSupportI2cExpectedSecondaryHighAddressNack(const BoardSupportI2cCtx_t *ctx, uint8_t addr7)
{
    return ctx && (int)ctx->Port == 1 && addr7 == 0x2Bu;
}

esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *userCtx, uint8_t addr7)
{
    if (!userCtx || addr7 > 0x7Fu) {
        return ESP_ERR_INVALID_ARG;
    }
    BoardSupportI2cCtx_t *ctx = boardSupportI2cWritableCtx(userCtx);
    esp_err_t err = boardSupportI2cEnsureMutex(ctx);
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cLock(ctx, boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    if (err != ESP_OK) {
        return err;
    }
    err = boardSupportI2cCheckReadyLocked(ctx, addr7, "probe");
    if (err != ESP_OK) {
        boardSupportI2cUnlock(ctx);
        return err;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        boardSupportI2cUnlock(ctx);
        return ESP_ERR_NO_MEM;
    }

    ctx->TransactionCount++;
    int64_t startUs = esp_timer_get_time();
    if (CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER) {
        printf("BOARD_I2C_XFER,stage=begin,port=%d,addr=0x%02X,op=probe,txLen=1,rxLen=0,count=%lu\n",
               (int)ctx->Port,
               addr7,
               (unsigned long)ctx->TransactionCount);
    }
    err = i2c_master_start(cmd);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, (uint8_t)((addr7 << 1u) | I2C_MASTER_WRITE), true);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(ctx->Port, cmd, boardSupportI2cMsToTicksAtLeastOne(ctx->TimeoutMs));
    }
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK &&
        err == ESP_FAIL &&
        boardSupportI2cExpectedSecondaryHighAddressNack(ctx, addr7)) {
        printf("I2C_NACK_EXPECTED,port=%d,addr=0x%02X,reason=secondary_addr_low_0x2A\n",
               (int)ctx->Port,
               addr7);
    }
    return boardSupportI2cFinishTransaction(ctx, addr7, "probe", err, startUs);
}

// IMPORTANT: Do not define app_main() here.
