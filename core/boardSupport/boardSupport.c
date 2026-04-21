#include "boardSupport.h"

#include <stdbool.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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

#define BOARD_SUPPORT_I2C_TIMEOUT_MS 100u
#define BOARD_SUPPORT_I2C_RECOVERY_PULSE_COUNT 9u
#define BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US 5u
#define BOARD_SUPPORT_I2C_FAILURE_STREAK_TRIGGER 2u
#define BOARD_SUPPORT_I2C_TRANSACTION_RETRY_MAX 1u
#define BOARD_SUPPORT_I2C_MUTEX_TIMEOUT_MS 200u
#define BOARD_SUPPORT_I2C_WARN_LOG_STRIDE 8u

static bool s_inited = false;
static bool s_i2c0_inited = false;
static bool s_i2c1_inited = false;
static uint32_t s_i2c0_configured_freq_hz = CONFIG_BOARD_I2C0_FREQ_HZ;
static uint32_t s_i2c1_configured_freq_hz = CONFIG_BOARD_I2C1_FREQ_HZ;
static uint32_t s_i2c0_failure_streak = 0u;
static uint32_t s_i2c1_failure_streak = 0u;
static uint32_t s_i2c0_warn_log_counter = 0u;
static uint32_t s_i2c1_warn_log_counter = 0u;
static SemaphoreHandle_t s_i2c0_mutex = NULL;
static SemaphoreHandle_t s_i2c1_mutex = NULL;

static BoardSupportI2cCtx_t s_i2c0_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
};

static BoardSupportI2cCtx_t s_i2c1_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
};

typedef struct {
    bool valid;
    bool enabled;
    i2c_port_t port;
    int sdaGpio;
    int sclGpio;
    uint32_t requestedFreqHz;
    bool *initedFlag;
    uint32_t *configuredFreqHz;
    uint32_t *failureStreak;
    uint32_t *warnLogCounter;
    SemaphoreHandle_t *busMutex;
    const char *name;
} boardSupportI2cPortConfig_t;

typedef esp_err_t (*boardSupportI2cTransferFn_t)(const BoardSupportI2cCtx_t *ctx,
                                                 uint8_t addr7,
                                                 const uint8_t *tx,
                                                 size_t txLen,
                                                 uint8_t *rx,
                                                 size_t rxLen);

static bool boardSupportI2cPinsValid(int sda_gpio, int scl_gpio)
{
    return (sda_gpio >= 0) && (scl_gpio >= 0);
}

static bool boardSupportShouldEmitRateLimited(uint32_t *counter, uint32_t period)
{
    if (!counter) {
        return true;
    }

    (*counter)++;
    if (*counter == 1u) {
        return true;
    }
    if (period <= 1u) {
        return true;
    }
    return ((*counter % period) == 0u);
}

static const char *boardSupportI2cErrClassName(esp_err_t err)
{
    if (err == ESP_FAIL) {
        return "nack";
    }
    if (err == ESP_ERR_TIMEOUT) {
        return "timeout";
    }
    return "error";
}

static bool boardSupportI2c1Configured(void)
{
    return boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO) &&
           (CONFIG_BOARD_I2C1_PORT >= 0);
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

static bool boardSupportResolveI2cPort(const BoardSupportI2cCtx_t *i2cCtx, boardSupportI2cPortConfig_t *outCfg)
{
    if (!i2cCtx || !outCfg) {
        return false;
    }

    *outCfg = (boardSupportI2cPortConfig_t){0};
    if (i2cCtx->Port == (i2c_port_t)CONFIG_BOARD_I2C_PORT) {
        *outCfg = (boardSupportI2cPortConfig_t){
            .valid = true,
            .enabled = boardSupportI2cPinsValid(CONFIG_BOARD_I2C_SDA_GPIO, CONFIG_BOARD_I2C_SCL_GPIO) &&
                       (CONFIG_BOARD_I2C_PORT >= 0),
            .port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
            .sdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
            .sclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
            .requestedFreqHz = CONFIG_BOARD_I2C0_FREQ_HZ,
            .initedFlag = &s_i2c0_inited,
            .configuredFreqHz = &s_i2c0_configured_freq_hz,
            .failureStreak = &s_i2c0_failure_streak,
            .warnLogCounter = &s_i2c0_warn_log_counter,
            .busMutex = &s_i2c0_mutex,
            .name = "I2C0",
        };
        return true;
    }

    if (i2cCtx->Port == (i2c_port_t)CONFIG_BOARD_I2C1_PORT) {
        *outCfg = (boardSupportI2cPortConfig_t){
            .valid = true,
            .enabled = boardSupportI2c1Configured(),
            .port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
            .sdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
            .sclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
            .requestedFreqHz = CONFIG_BOARD_I2C1_FREQ_HZ,
            .initedFlag = &s_i2c1_inited,
            .configuredFreqHz = &s_i2c1_configured_freq_hz,
            .failureStreak = &s_i2c1_failure_streak,
            .warnLogCounter = &s_i2c1_warn_log_counter,
            .busMutex = &s_i2c1_mutex,
            .name = "I2C1",
        };
        return true;
    }

    return false;
}

static esp_err_t boardSupportI2cLockPort(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->busMutex || !(*cfg->busMutex)) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t timeoutTicks = pdMS_TO_TICKS(BOARD_SUPPORT_I2C_MUTEX_TIMEOUT_MS);
    if (timeoutTicks == 0) {
        timeoutTicks = 1;
    }

    if (xSemaphoreTake(*cfg->busMutex, timeoutTicks) != pdTRUE) {
        ESP_LOGE(TAG,
                 "%s lock timeout after %u ms",
                 cfg->name ? cfg->name : "I2C",
                 (unsigned)BOARD_SUPPORT_I2C_MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void boardSupportI2cUnlockPort(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->busMutex || !(*cfg->busMutex)) {
        return;
    }
    xSemaphoreGive(*cfg->busMutex);
}

static esp_err_t boardSupportI2cCheckLinesLocked(const boardSupportI2cPortConfig_t *cfg,
                                                 bool *outSclHigh,
                                                 bool *outSdaHigh)
{
    if (!cfg || !outSclHigh || !outSdaHigh || !cfg->valid || !cfg->enabled) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSclHigh = gpio_get_level((gpio_num_t)cfg->sclGpio) != 0;
    *outSdaHigh = gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0;
    return ESP_OK;
}

static esp_err_t boardSupportConfigureRecoveryPins(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->valid || !cfg->enabled) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->sdaGpio < 0 || cfg->sdaGpio >= (int)GPIO_NUM_MAX ||
        cfg->sclGpio < 0 || cfg->sclGpio >= (int)GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t mask = (1ULL << (uint32_t)cfg->sdaGpio) | (1ULL << (uint32_t)cfg->sclGpio);
    gpio_config_t ioCfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ioCfg);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level((gpio_num_t)cfg->sclGpio, 1);
    gpio_set_level((gpio_num_t)cfg->sdaGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
    return ESP_OK;
}

static esp_err_t boardSupportI2cReinitLocked(const boardSupportI2cPortConfig_t *cfg, const char *reason)
{
    if (!cfg || !cfg->valid || !cfg->enabled || !cfg->initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t deleteErr = i2c_driver_delete(cfg->port);
    if (deleteErr != ESP_OK && deleteErr != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "%s delete before reinit failed: %d", cfg->name, deleteErr);
    }
    *cfg->initedFlag = false;

    esp_err_t err = boardSupportInitI2c(cfg->port,
                                        cfg->sdaGpio,
                                        cfg->sclGpio,
                                        cfg->requestedFreqHz,
                                        cfg->configuredFreqHz);
    if (err == ESP_OK) {
        *cfg->initedFlag = true;
        if (cfg->failureStreak) {
            *cfg->failureStreak = 0u;
        }
        if (cfg->warnLogCounter) {
            *cfg->warnLogCounter = 0u;
        }
    }

    ESP_LOGW(TAG,
             "%s reinit reason=%s deleteErr=%d initErr=%d freqHz=%u status=%s",
             cfg->name,
             reason ? reason : "na",
             deleteErr,
             err,
             (unsigned)(cfg->configuredFreqHz ? *cfg->configuredFreqHz : cfg->requestedFreqHz),
             (err == ESP_OK) ? "reinit_ok" : "reinit_failed");
    return err;
}

static esp_err_t boardSupportI2cRecoverBusLocked(const boardSupportI2cPortConfig_t *cfg, const char *reason)
{
    if (!cfg || !cfg->valid || !cfg->enabled || !cfg->initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    bool sclBefore = true;
    bool sdaBefore = true;
    (void)boardSupportI2cCheckLinesLocked(cfg, &sclBefore, &sdaBefore);

    ESP_LOGW(TAG,
             "%s recover_attempt reason=%s sclBefore=%d sdaBefore=%d",
             cfg->name,
             reason ? reason : "na",
             sclBefore ? 1 : 0,
             sdaBefore ? 1 : 0);

    esp_err_t deleteErr = i2c_driver_delete(cfg->port);
    if (deleteErr != ESP_OK && deleteErr != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "%s delete before recover failed: %d", cfg->name, deleteErr);
    }
    *cfg->initedFlag = false;

    esp_err_t pinErr = boardSupportConfigureRecoveryPins(cfg);
    if (pinErr != ESP_OK) {
        ESP_LOGE(TAG, "%s recover failed: status=recover_gpio_config_failed err=%d", cfg->name, pinErr);
        return pinErr;
    }

    bool sclAfterRelease = gpio_get_level((gpio_num_t)cfg->sclGpio) != 0;
    bool sdaAfterRelease = gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0;

    // If SCL cannot be released high, no software pulse strategy can recover this bus.
    if (!sclAfterRelease) {
        esp_err_t reinitErr = boardSupportI2cReinitLocked(cfg, "recover_scl_held_low");
        ESP_LOGE(TAG,
                 "%s recover failed: status=recover_fail_scl_held_low reason=%s reinitErr=%d",
                 cfg->name,
                 reason ? reason : "na",
                 reinitErr);
        return (reinitErr == ESP_OK) ? ESP_ERR_TIMEOUT : reinitErr;
    }

    uint32_t pulsesIssued = 0u;
    if (!sdaAfterRelease) {
        for (uint32_t i = 0u; i < BOARD_SUPPORT_I2C_RECOVERY_PULSE_COUNT; ++i) {
            gpio_set_level((gpio_num_t)cfg->sclGpio, 0);
            esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
            gpio_set_level((gpio_num_t)cfg->sclGpio, 1);
            esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
            pulsesIssued = i + 1u;
            if (gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0) {
                break;
            }
        }
    }

    // Attempt STOP condition while GPIO-open-drain mode is active.
    gpio_set_level((gpio_num_t)cfg->sdaGpio, 0);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
    gpio_set_level((gpio_num_t)cfg->sclGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
    gpio_set_level((gpio_num_t)cfg->sdaGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);

    bool sclAfterPulse = gpio_get_level((gpio_num_t)cfg->sclGpio) != 0;
    bool sdaAfterPulse = gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0;

    if (!sclAfterPulse) {
        esp_err_t reinitErr = boardSupportI2cReinitLocked(cfg, "recover_scl_held_low_after_pulse");
        ESP_LOGE(TAG,
                 "%s recover failed: status=recover_fail_scl_held_low reason=%s pulses=%lu reinitErr=%d",
                 cfg->name,
                 reason ? reason : "na",
                 (unsigned long)pulsesIssued,
                 reinitErr);
        return (reinitErr == ESP_OK) ? ESP_ERR_TIMEOUT : reinitErr;
    }

    esp_err_t reinitErr = boardSupportI2cReinitLocked(cfg, reason ? reason : "bus_recover");
    if (reinitErr != ESP_OK) {
        ESP_LOGE(TAG,
                 "%s recover failed: status=recover_fail_driver_reinit reason=%s err=%d",
                 cfg->name,
                 reason ? reason : "na",
                 reinitErr);
        return reinitErr;
    }

    bool sclAfterReinit = true;
    bool sdaAfterReinit = true;
    (void)boardSupportI2cCheckLinesLocked(cfg, &sclAfterReinit, &sdaAfterReinit);
    if (!sclAfterReinit) {
        ESP_LOGE(TAG,
                 "%s recover failed: status=recover_fail_scl_held_low reason=%s pulses=%lu sdaAfterReinit=%d",
                 cfg->name,
                 reason ? reason : "na",
                 (unsigned long)pulsesIssued,
                 sdaAfterReinit ? 1 : 0);
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGW(TAG,
             "%s recover_success reason=%s pulses=%lu sclAfterRelease=%d sdaAfterRelease=%d "
             "sclAfterPulse=%d sdaAfterPulse=%d sclAfterReinit=%d sdaAfterReinit=%d",
             cfg->name,
             reason ? reason : "na",
             (unsigned long)pulsesIssued,
             sclAfterRelease ? 1 : 0,
             sdaAfterRelease ? 1 : 0,
             sclAfterPulse ? 1 : 0,
             sdaAfterPulse ? 1 : 0,
             sclAfterReinit ? 1 : 0,
             sdaAfterReinit ? 1 : 0);
    return ESP_OK;
}

static bool boardSupportI2cShouldRecover(esp_err_t err,
                                         uint32_t failureStreak,
                                         bool sclHigh,
                                         bool sdaHigh,
                                         bool allowStreakRecover)
{
    if (!sclHigh || !sdaHigh) {
        return true;
    }
    if (err == ESP_ERR_TIMEOUT) {
        return true;
    }
    if (!allowStreakRecover) {
        return false;
    }
    return failureStreak >= BOARD_SUPPORT_I2C_FAILURE_STREAK_TRIGGER;
}

static esp_err_t boardSupportI2cTransferWriteRead(const BoardSupportI2cCtx_t *ctx,
                                                  uint8_t addr7,
                                                  const uint8_t *tx,
                                                  size_t txLen,
                                                  uint8_t *rx,
                                                  size_t rxLen)
{
    return i2c_master_write_read_device(ctx->Port,
                                        addr7,
                                        tx,
                                        txLen,
                                        rx,
                                        rxLen,
                                        pdMS_TO_TICKS(ctx->TimeoutMs));
}

static esp_err_t boardSupportI2cTransferWrite(const BoardSupportI2cCtx_t *ctx,
                                              uint8_t addr7,
                                              const uint8_t *tx,
                                              size_t txLen,
                                              uint8_t *rx,
                                              size_t rxLen)
{
    (void)rx;
    (void)rxLen;
    return i2c_master_write_to_device(ctx->Port,
                                      addr7,
                                      tx,
                                      txLen,
                                      pdMS_TO_TICKS(ctx->TimeoutMs));
}

static esp_err_t boardSupportI2cTransferProbe(const BoardSupportI2cCtx_t *ctx,
                                              uint8_t addr7,
                                              const uint8_t *tx,
                                              size_t txLen,
                                              uint8_t *rx,
                                              size_t rxLen)
{
    (void)tx;
    (void)txLen;
    (void)rx;
    (void)rxLen;

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
        err = i2c_master_cmd_begin(ctx->Port, cmd, pdMS_TO_TICKS(ctx->TimeoutMs));
    }

    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t boardSupportI2cRunWithRecoveryLocked(const BoardSupportI2cCtx_t *ctx,
                                                      const boardSupportI2cPortConfig_t *cfg,
                                                      uint8_t addr7,
                                                      const uint8_t *tx,
                                                      size_t txLen,
                                                      uint8_t *rx,
                                                      size_t rxLen,
                                                      boardSupportI2cTransferFn_t transferFn,
                                                      const char *opName,
                                                      bool allowStreakRecover)
{
    if (!ctx || !cfg || !transferFn) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!cfg->valid || !cfg->enabled || !cfg->initedFlag || !(*cfg->initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    bool sclHigh = true;
    bool sdaHigh = true;
    if (boardSupportI2cCheckLinesLocked(cfg, &sclHigh, &sdaHigh) == ESP_OK && (!sclHigh || !sdaHigh)) {
        esp_err_t preRecoverErr = boardSupportI2cRecoverBusLocked(cfg, "preop_line_stuck");
        if (preRecoverErr != ESP_OK) {
            ESP_LOGE(TAG, "%s %s pre-op recover failed: %d", cfg->name, opName ? opName : "i2c", preRecoverErr);
            return preRecoverErr;
        }
    }

    esp_err_t err = ESP_FAIL;
    for (uint32_t attempt = 0u; attempt <= BOARD_SUPPORT_I2C_TRANSACTION_RETRY_MAX; ++attempt) {
        err = transferFn(ctx, addr7, tx, txLen, rx, rxLen);
        if (err == ESP_OK) {
            if (cfg->failureStreak) {
                *cfg->failureStreak = 0u;
            }
            if (cfg->warnLogCounter) {
                *cfg->warnLogCounter = 0u;
            }
            return ESP_OK;
        }

        if (cfg->failureStreak) {
            (*cfg->failureStreak)++;
        }
        bool haveLines = (boardSupportI2cCheckLinesLocked(cfg, &sclHigh, &sdaHigh) == ESP_OK);
        bool shouldRecover = boardSupportI2cShouldRecover(err,
                                                          cfg->failureStreak ? *cfg->failureStreak : 1u,
                                                          haveLines ? sclHigh : true,
                                                          haveLines ? sdaHigh : true,
                                                          allowStreakRecover);

        if (boardSupportShouldEmitRateLimited(cfg->warnLogCounter, BOARD_SUPPORT_I2C_WARN_LOG_STRIDE)) {
            ESP_LOGW(TAG,
                     "%s %s failed: addr=0x%02X attempt=%lu err=%d class=%s streak=%lu "
                     "sclHigh=%d sdaHigh=%d",
                     cfg->name,
                     opName ? opName : "i2c",
                     addr7,
                     (unsigned long)attempt,
                     err,
                     boardSupportI2cErrClassName(err),
                     (unsigned long)(cfg->failureStreak ? *cfg->failureStreak : 0u),
                     haveLines ? (sclHigh ? 1 : 0) : -1,
                     haveLines ? (sdaHigh ? 1 : 0) : -1);
        }

        if (!shouldRecover || attempt >= BOARD_SUPPORT_I2C_TRANSACTION_RETRY_MAX) {
            break;
        }

        const char *reason = (!haveLines || !sclHigh || !sdaHigh) ? "line_stuck_during_transfer"
                                                                   : ((err == ESP_ERR_TIMEOUT) ? "transfer_timeout"
                                                                                               : "transfer_error_streak");
        esp_err_t recoverErr = boardSupportI2cRecoverBusLocked(cfg, reason);
        if (recoverErr != ESP_OK) {
            ESP_LOGE(TAG, "%s %s recover failed: %d", cfg->name, opName ? opName : "i2c", recoverErr);
            return recoverErr;
        }
    }

    return err;
}

esp_err_t boardSupportInit(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    if (!s_i2c0_mutex) {
        s_i2c0_mutex = xSemaphoreCreateMutex();
        if (!s_i2c0_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (boardSupportIsI2c1Enabled() && !s_i2c1_mutex) {
        s_i2c1_mutex = xSemaphoreCreateMutex();
        if (!s_i2c1_mutex) {
            return ESP_ERR_NO_MEM;
        }
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
    s_i2c0_failure_streak = 0u;
    s_i2c0_warn_log_counter = 0u;
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
        s_i2c1_failure_streak = 0u;
        s_i2c1_warn_log_counter = 0u;
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

    s_i2c0_failure_streak = 0u;
    s_i2c1_failure_streak = 0u;
    s_i2c0_warn_log_counter = 0u;
    s_i2c1_warn_log_counter = 0u;
    if (s_i2c1_mutex) {
        vSemaphoreDelete(s_i2c1_mutex);
        s_i2c1_mutex = NULL;
    }
    if (s_i2c0_mutex) {
        vSemaphoreDelete(s_i2c0_mutex);
        s_i2c0_mutex = NULL;
    }
    s_inited = false;
    return err;
}

bool boardSupportIsI2c1Enabled(void)
{
    return boardSupportI2c1Configured();
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

esp_err_t boardSupportI2cCheckLines(const BoardSupportI2cCtx_t *i2cCtx, bool *outSclHigh, bool *outSdaHigh)
{
    if (!i2cCtx || !outSclHigh || !outSdaHigh) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cCheckLinesLocked(&cfg, outSclHigh, outSdaHigh);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cReinit(const BoardSupportI2cCtx_t *i2cCtx, const char *reason)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cReinitLocked(&cfg, reason);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cRecoverBus(const BoardSupportI2cCtx_t *i2cCtx, const char *reason)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRecoverBusLocked(&cfg, reason);
    boardSupportI2cUnlockPort(&cfg);
    return err;
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
    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(ctx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag || !(*cfg.initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRunWithRecoveryLocked(ctx,
                                                         &cfg,
                                                         addr7,
                                                         tx,
                                                         txLen,
                                                         rx,
                                                         rxLen,
                                                         boardSupportI2cTransferWriteRead,
                                                         "write_read",
                                                         true);
    boardSupportI2cUnlockPort(&cfg);
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
    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(ctx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag || !(*cfg.initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRunWithRecoveryLocked(ctx,
                                                         &cfg,
                                                         addr7,
                                                         tx,
                                                         txLen,
                                                         NULL,
                                                         0u,
                                                         boardSupportI2cTransferWrite,
                                                         "write",
                                                         true);
    boardSupportI2cUnlockPort(&cfg);
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

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag || !(*cfg.initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }

    /*
     * Probe NACKs are expected for absent addresses during diagnostics.
     * Keep pre-op stuck-line recovery enabled, but disable streak-based
     * recovery escalation for plain probe failures.
     */
    esp_err_t err = boardSupportI2cRunWithRecoveryLocked(i2cCtx,
                                                         &cfg,
                                                         addr7,
                                                         NULL,
                                                         0u,
                                                         NULL,
                                                         0u,
                                                         boardSupportI2cTransferProbe,
                                                         "probe",
                                                         false);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

// IMPORTANT: Do not define app_main() here.
