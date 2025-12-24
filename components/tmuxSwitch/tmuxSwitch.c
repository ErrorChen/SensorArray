#include "tmuxSwitch.h"

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define TMUX_GPIO_VALID(gpio) ((gpio) >= 0 && (gpio) < GPIO_NUM_MAX)
#define TMUX1134_SELA_LEVEL (CONFIG_TMUX1134_SELA_ENABLED_LEVEL ? 1 : 0)
#define TMUX1134_SELB_LEVEL (CONFIG_TMUX1134_SELB_ENABLED_LEVEL ? 1 : 0)

static portMUX_TYPE s_tmux_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_inited = false;
static uint8_t s_row = 0;
static tmux1108Source_t s_source = TMUX1108_SOURCE_GND;
static bool s_en_on = true;

static tmux1108Source_t tmux1108SourceFromConfig(int value)
{
    return value ? TMUX1108_SOURCE_REF : TMUX1108_SOURCE_GND;
}

#if CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE
static void tmuxDelayUs(void)
{
#if CONFIG_TMUX1108_SWITCH_DELAY_US > 0
    esp_rom_delay_us(CONFIG_TMUX1108_SWITCH_DELAY_US);
#endif
}
#endif

static int tmux1108SourceToLevel(tmux1108Source_t source)
{
    int ref_level = CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0;
    return (source == TMUX1108_SOURCE_REF) ? ref_level : (ref_level ? 0 : 1);
}

static void tmux1108ApplySourceNoLock(tmux1108Source_t source)
{
    gpio_set_level((gpio_num_t)CONFIG_TMUX1108_SW_GPIO, tmux1108SourceToLevel(source));
}

static void tmux1108ApplyRowNoLock(uint8_t row)
{
    gpio_set_level((gpio_num_t)CONFIG_TMUX1108_A0_GPIO, row & 0x1);
    gpio_set_level((gpio_num_t)CONFIG_TMUX1108_A1_GPIO, (row >> 1) & 0x1);
    gpio_set_level((gpio_num_t)CONFIG_TMUX1108_A2_GPIO, (row >> 2) & 0x1);
}

static void tmux1134ApplySelNoLock(int gpio, bool enabled, int enabled_level)
{
    if (gpio < 0) {
        return;
    }
    int level = enabled ? enabled_level : (enabled_level ? 0 : 1);
    gpio_set_level((gpio_num_t)gpio, level);
}

static void tmux1134SetEnStateNoLock(bool on)
{
    if (CONFIG_TMUX1134_EN_GPIO < 0) {
        s_en_on = true;
        return;
    }
    int off_level = CONFIG_TMUX1134_EN_OFF_LEVEL ? 1 : 0;
    int level = on ? (off_level ? 0 : 1) : off_level;
    gpio_set_level((gpio_num_t)CONFIG_TMUX1134_EN_GPIO, level);
    s_en_on = on;
}

static esp_err_t tmuxValidateConfig(void)
{
    if (!TMUX_GPIO_VALID(CONFIG_TMUX1108_A0_GPIO) ||
        !TMUX_GPIO_VALID(CONFIG_TMUX1108_A1_GPIO) ||
        !TMUX_GPIO_VALID(CONFIG_TMUX1108_A2_GPIO) ||
        !TMUX_GPIO_VALID(CONFIG_TMUX1108_SW_GPIO) ||
        !TMUX_GPIO_VALID(CONFIG_TMUX1134_SEL1_GPIO) ||
        !TMUX_GPIO_VALID(CONFIG_TMUX1134_SEL2_GPIO)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (CONFIG_TMUX1134_SEL3_GPIO >= 0 && !TMUX_GPIO_VALID(CONFIG_TMUX1134_SEL3_GPIO)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (CONFIG_TMUX1134_SEL4_GPIO >= 0 && !TMUX_GPIO_VALID(CONFIG_TMUX1134_SEL4_GPIO)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !TMUX_GPIO_VALID(CONFIG_TMUX1134_EN_GPIO)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t tmuxSwitchInit(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    esp_err_t err = tmuxValidateConfig();
    if (err != ESP_OK) {
        return err;
    }

    uint64_t mask = 0;
    mask |= 1ULL << CONFIG_TMUX1108_A0_GPIO;
    mask |= 1ULL << CONFIG_TMUX1108_A1_GPIO;
    mask |= 1ULL << CONFIG_TMUX1108_A2_GPIO;
    mask |= 1ULL << CONFIG_TMUX1108_SW_GPIO;
    mask |= 1ULL << CONFIG_TMUX1134_SEL1_GPIO;
    mask |= 1ULL << CONFIG_TMUX1134_SEL2_GPIO;
    if (CONFIG_TMUX1134_SEL3_GPIO >= 0) {
        mask |= 1ULL << CONFIG_TMUX1134_SEL3_GPIO;
    }
    if (CONFIG_TMUX1134_SEL4_GPIO >= 0) {
        mask |= 1ULL << CONFIG_TMUX1134_SEL4_GPIO;
    }
    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        mask |= 1ULL << CONFIG_TMUX1134_EN_GPIO;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    const tmux1108Source_t default_source = tmux1108SourceFromConfig(CONFIG_TMUX1108_DEFAULT_SOURCE);
    bool sela_default = CONFIG_TMUX1134_DEFAULT_SELA_ENABLED;
    bool selb_default = CONFIG_TMUX1134_DEFAULT_SELB_ENABLED;
    const bool all_off = CONFIG_TMUX1134_DEFAULT_ALL_OFF;

    portENTER_CRITICAL(&s_tmux_lock);

    if (all_off && CONFIG_TMUX1134_EN_GPIO < 0) {
        sela_default = false;
        selb_default = false;
    }

    // TMUX1108 EN is tied high on this board; only A[2:0] and SW are controlled.
    tmux1108ApplySourceNoLock(default_source);
    tmux1108ApplyRowNoLock(0);

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, sela_default, TMUX1134_SELA_LEVEL);
    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, selb_default, TMUX1134_SELB_LEVEL);
    if (CONFIG_TMUX1134_SEL3_GPIO >= 0) {
        gpio_set_level((gpio_num_t)CONFIG_TMUX1134_SEL3_GPIO, 0);
    }
    if (CONFIG_TMUX1134_SEL4_GPIO >= 0) {
        gpio_set_level((gpio_num_t)CONFIG_TMUX1134_SEL4_GPIO, 0);
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        tmux1134SetEnStateNoLock(!all_off);
    } else {
        s_en_on = true;
    }

    s_row = 0;
    s_source = default_source;
    s_inited = true;

    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1108SelectRow(uint8_t row)
{
    if (row > 7) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_row == row) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_OK;
    }

#if CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE
    tmux1108Source_t prev_source = s_source;
    tmux1108Source_t safe_source = tmux1108SourceFromConfig(CONFIG_TMUX1108_SAFE_SOURCE);
    bool restore_source = (safe_source != prev_source);
    if (restore_source) {
        tmux1108ApplySourceNoLock(safe_source);
        tmuxDelayUs();
    }
#endif

    tmux1108ApplyRowNoLock(row);

#if CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE
    tmuxDelayUs();
    if (restore_source) {
        tmux1108ApplySourceNoLock(prev_source);
    }
#endif

    s_row = row;
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1108GetRow(uint8_t *rowOut)
{
    if (rowOut == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }
    *rowOut = s_row;
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1108SetSource(tmux1108Source_t source)
{
    if (source != TMUX1108_SOURCE_GND && source != TMUX1108_SOURCE_REF) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_source == source) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_OK;
    }

    tmux1108ApplySourceNoLock(source);
    s_source = source;

    portEXIT_CRITICAL(&s_tmux_lock);
    return ESP_OK;
}

esp_err_t tmux1108GetSource(tmux1108Source_t *sourceOut)
{
    if (sourceOut == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }
    *sourceOut = s_source;
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SetSelAEnabled(bool enabled)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, enabled, TMUX1134_SELA_LEVEL);
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SetSelBEnabled(bool enabled)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, enabled, TMUX1134_SELB_LEVEL);
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SetAllOff(void)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        tmux1134SetEnStateNoLock(false);
    } else {
        s_en_on = true;
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, false, TMUX1134_SELA_LEVEL);
    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, false, TMUX1134_SELB_LEVEL);

    portEXIT_CRITICAL(&s_tmux_lock);
    return ESP_OK;
}

esp_err_t tmux1134SetAllOn(void)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, true, TMUX1134_SELA_LEVEL);
    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, true, TMUX1134_SELB_LEVEL);

    portEXIT_CRITICAL(&s_tmux_lock);
    return ESP_OK;
}

esp_err_t tmuxSwitchSelectRow(uint8_t row)
{
    return tmux1108SelectRow(row);
}

esp_err_t tmuxSwitchSetSelAEnabled(bool enabled)
{
    return tmux1134SetSelAEnabled(enabled);
}

esp_err_t tmuxSwitchSetSelBEnabled(bool enabled)
{
    return tmux1134SetSelBEnabled(enabled);
}

esp_err_t tmuxSwitchSet1108Source(tmux1108Source_t source)
{
    return tmux1108SetSource(source);
}
