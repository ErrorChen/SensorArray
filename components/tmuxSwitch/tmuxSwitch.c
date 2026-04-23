#include "tmuxSwitch.h"

#include <stdio.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#ifndef CONFIG_TMUX1134_DEFAULT_ALL_OFF
#define CONFIG_TMUX1134_DEFAULT_ALL_OFF 0
#endif
#ifndef CONFIG_TMUX1134_DEFAULT_SELA_ENABLED
#define CONFIG_TMUX1134_DEFAULT_SELA_ENABLED 0
#endif
#ifndef CONFIG_TMUX1134_DEFAULT_SELB_ENABLED
#define CONFIG_TMUX1134_DEFAULT_SELB_ENABLED 0
#endif

#define TMUX_GPIO_VALID(gpio) ((gpio) >= 0 && (gpio) < GPIO_NUM_MAX)
#define TMUX1134_SELA_LEVEL (CONFIG_TMUX1134_SELA_ENABLED_LEVEL ? 1 : 0)
#define TMUX1134_SELB_LEVEL (CONFIG_TMUX1134_SELB_ENABLED_LEVEL ? 1 : 0)

static portMUX_TYPE s_tmux_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_inited = false;
static uint8_t s_row = 0;
static tmux1108Source_t s_source = TMUX1108_SOURCE_GND;
static bool s_en_on = true;
static int s_cmd_sel1_level = 0;
static int s_cmd_sel2_level = 0;
static int s_cmd_sel3_level = -1;
static int s_cmd_sel4_level = -1;
static int s_cmd_en_level = -1;

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

static bool tmux1134EnabledToSelLevel(bool enabled, int enabled_level)
{
    int onLevel = enabled_level ? 1 : 0;
    return enabled ? (onLevel != 0) : (onLevel == 0);
}

static void tmuxSetCachedSelLevelNoLock(int gpio, int level)
{
    if (gpio == CONFIG_TMUX1134_SEL1_GPIO) {
        s_cmd_sel1_level = level;
    } else if (gpio == CONFIG_TMUX1134_SEL2_GPIO) {
        s_cmd_sel2_level = level;
    }
#if CONFIG_TMUX1134_SEL3_GPIO >= 0
    else if (gpio == CONFIG_TMUX1134_SEL3_GPIO) {
        s_cmd_sel3_level = level;
    }
#endif
#if CONFIG_TMUX1134_SEL4_GPIO >= 0
    else if (gpio == CONFIG_TMUX1134_SEL4_GPIO) {
        s_cmd_sel4_level = level;
    }
#endif
}

static void tmux1134ApplySelNoLock(int gpio, bool level)
{
    if (gpio < 0) {
        return;
    }
    int level_i = level ? 1 : 0;
    gpio_set_level((gpio_num_t)gpio, level_i);
    tmuxSetCachedSelLevelNoLock(gpio, level_i);
}

static int tmuxReadGpioLevelNoLock(int gpio)
{
    if (gpio < 0) {
        return -1;
    }
    return gpio_get_level((gpio_num_t)gpio);
}

static void tmux1134SetEnStateNoLock(bool on)
{
    if (CONFIG_TMUX1134_EN_GPIO < 0) {
        s_en_on = true;
        s_cmd_en_level = -1;
        return;
    }
    int off_level = CONFIG_TMUX1134_EN_OFF_LEVEL ? 1 : 0;
    int level = on ? (off_level ? 0 : 1) : off_level;
    gpio_set_level((gpio_num_t)CONFIG_TMUX1134_EN_GPIO, level);
    s_en_on = on;
    s_cmd_en_level = level;
}

static const char *tmuxStringOrNa(const char *value)
{
    return value ? value : "na";
}

static const char *tmuxFormatLevel(char *buf, size_t bufSize, int level)
{
    if (!buf || bufSize == 0u) {
        return "na";
    }
    if (level < 0) {
        (void)snprintf(buf, bufSize, "na");
        return buf;
    }
    (void)snprintf(buf, bufSize, "%d", level ? 1 : 0);
    return buf;
}

static const char *tmuxSwSemanticFromLevel(int level)
{
    if (level < 0) {
        return "source=na";
    }
    int refLevel = CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0;
    return (level == refLevel) ? "level->source=REF" : "level->source=GND";
}

static const char *tmux1134SelSemanticFromLevel(int level)
{
    if (level < 0) {
        return "select=na";
    }
    return (level != 0) ? "level->select=SxA" : "level->select=SxB";
}

static const char *tmux1134EnSemanticFromLevel(int level)
{
    if (CONFIG_TMUX1134_EN_GPIO < 0) {
        return "enabled=hardwired";
    }
    if (level < 0) {
        return "enabled=na";
    }
    int offLevel = CONFIG_TMUX1134_EN_OFF_LEVEL ? 1 : 0;
    return (level == offLevel) ? "enabled=0" : "enabled=1";
}

static void tmuxLogSnapshotLine(const tmuxSwitchSnapshotContext_t *context,
                                const char *gpioName,
                                int gpioNum,
                                int commandLevel,
                                int observedLevel,
                                const char *activePolarity,
                                const char *semanticMeaning,
                                const char *readbackReliability)
{
    char cmdBuf[8] = {0};
    char obsBuf[8] = {0};
    printf("DBGTMUXCTRL,stage=%s,routeLabel=%s,targetPath=%s,expectedSemantic=%s,gpioName=%s,gpioNum=%d,"
           "commandLevel=%s,observedLevel=%s,activePolarity=%s,semanticMeaning=%s,readbackReliability=%s\n",
           tmuxStringOrNa(context ? context->stage : NULL),
           tmuxStringOrNa(context ? context->routeLabel : NULL),
           tmuxStringOrNa(context ? context->targetPath : NULL),
           tmuxStringOrNa(context ? context->expectedSemantic : NULL),
           tmuxStringOrNa(gpioName),
           gpioNum,
           tmuxFormatLevel(cmdBuf, sizeof(cmdBuf), commandLevel),
           tmuxFormatLevel(obsBuf, sizeof(obsBuf), observedLevel),
           tmuxStringOrNa(activePolarity),
           tmuxStringOrNa(semanticMeaning),
           tmuxStringOrNa(readbackReliability));
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
#if CONFIG_TMUX1134_SEL3_GPIO >= 0
    mask |= 1ULL << CONFIG_TMUX1134_SEL3_GPIO;
#endif
#if CONFIG_TMUX1134_SEL4_GPIO >= 0
    mask |= 1ULL << CONFIG_TMUX1134_SEL4_GPIO;
#endif
#if CONFIG_TMUX1134_EN_GPIO >= 0
    mask |= 1ULL << CONFIG_TMUX1134_EN_GPIO;
#endif

    /*
     * Route diagnostics sample control pins with gpio_get_level(). For ESP-IDF,
     * that readback is only meaningful when input is enabled, so control GPIOs
     * are configured as input+output to preserve drive behavior and allow
     * MCU-side observation.
     */
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
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

    /*
     * If EN is controllable, force OFF first so row/source/SEL lines settle before
     * the final requested enable state is applied.
     */
    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        tmux1134SetEnStateNoLock(false);
    }

    // TMUX1108 EN is tied high on this board; only A[2:0] and SW are controlled.
    tmux1108ApplySourceNoLock(default_source);
    tmux1108ApplyRowNoLock(0);

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, tmux1134EnabledToSelLevel(sela_default, TMUX1134_SELA_LEVEL));
    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, tmux1134EnabledToSelLevel(selb_default, TMUX1134_SELB_LEVEL));
    if (CONFIG_TMUX1134_SEL3_GPIO >= 0) {
        gpio_set_level((gpio_num_t)CONFIG_TMUX1134_SEL3_GPIO, 0);
        s_cmd_sel3_level = 0;
    } else {
        s_cmd_sel3_level = -1;
    }
    if (CONFIG_TMUX1134_SEL4_GPIO >= 0) {
        gpio_set_level((gpio_num_t)CONFIG_TMUX1134_SEL4_GPIO, 0);
        s_cmd_sel4_level = 0;
    } else {
        s_cmd_sel4_level = -1;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        tmux1134SetEnStateNoLock(!all_off);
    } else {
        s_en_on = true;
        s_cmd_en_level = -1;
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
    *rowOut = s_row; // Cached command state; TMUX1108 row pins have no independent hardware feedback.
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
    *sourceOut = s_source; // Cached command state; not proof of external analog path selection.
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SelectSelALevel(bool level)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, level);
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SelectSelBLevel(bool level)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, level);
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SelectSel3Level(bool level)
{
    if (CONFIG_TMUX1134_SEL3_GPIO < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL3_GPIO, level);
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SelectSel4Level(bool level)
{
    if (CONFIG_TMUX1134_SEL4_GPIO < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_TMUX1134_EN_GPIO >= 0 && !s_en_on) {
        tmux1134SetEnStateNoLock(true);
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL4_GPIO, level);
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmux1134SetEnLogicalState(bool on)
{
    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        tmux1134SetEnStateNoLock(on);
    } else {
        s_en_on = true;
        s_cmd_en_level = -1;
    }
    portEXIT_CRITICAL(&s_tmux_lock);
    return ESP_OK;
}

esp_err_t tmux1134GetEnLogicalState(bool *onOut)
{
    if (onOut == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }
    *onOut = s_en_on;
    portEXIT_CRITICAL(&s_tmux_lock);
    return ESP_OK;
}

esp_err_t tmux1134SetSelAEnabled(bool enabled)
{
    return tmux1134SelectSelALevel(tmux1134EnabledToSelLevel(enabled, TMUX1134_SELA_LEVEL));
}

esp_err_t tmux1134SetSelBEnabled(bool enabled)
{
    return tmux1134SelectSelBLevel(tmux1134EnabledToSelLevel(enabled, TMUX1134_SELB_LEVEL));
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
        s_cmd_en_level = -1;
    }

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, tmux1134EnabledToSelLevel(false, TMUX1134_SELA_LEVEL));
    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, tmux1134EnabledToSelLevel(false, TMUX1134_SELB_LEVEL));

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

    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL1_GPIO, tmux1134EnabledToSelLevel(true, TMUX1134_SELA_LEVEL));
    tmux1134ApplySelNoLock(CONFIG_TMUX1134_SEL2_GPIO, tmux1134EnabledToSelLevel(true, TMUX1134_SELB_LEVEL));

    portEXIT_CRITICAL(&s_tmux_lock);
    return ESP_OK;
}

esp_err_t tmuxSwitchGetControlState(tmuxSwitchControlState_t *outState)
{
    if (outState == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_tmux_lock);
    if (!s_inited) {
        portEXIT_CRITICAL(&s_tmux_lock);
        return ESP_ERR_INVALID_STATE;
    }

    outState->inited = s_inited;
    outState->tmux1134EnControllable = (CONFIG_TMUX1134_EN_GPIO >= 0);

    outState->cmdRow = s_row;
    outState->cmdSource = s_source;
    outState->cmdTmux1134EnLogicalOn = (CONFIG_TMUX1134_EN_GPIO >= 0) ? s_en_on : true;
    outState->cmdA0Level = (int)(s_row & 0x1u);
    outState->cmdA1Level = (int)((s_row >> 1u) & 0x1u);
    outState->cmdA2Level = (int)((s_row >> 2u) & 0x1u);
    outState->cmdSwLevel = tmux1108SourceToLevel(s_source);
    outState->cmdSel1Level = s_cmd_sel1_level;
    outState->cmdSel2Level = s_cmd_sel2_level;
    outState->cmdSel3Level = (CONFIG_TMUX1134_SEL3_GPIO >= 0) ? s_cmd_sel3_level : -1;
    outState->cmdSel4Level = (CONFIG_TMUX1134_SEL4_GPIO >= 0) ? s_cmd_sel4_level : -1;
    outState->cmdEnLevel = (CONFIG_TMUX1134_EN_GPIO >= 0) ? s_cmd_en_level : -1;
    outState->cmdSelaLevel = outState->cmdSel1Level;
    outState->cmdSelbLevel = outState->cmdSel2Level;

    outState->obsA0Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1108_A0_GPIO);
    outState->obsA1Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1108_A1_GPIO);
    outState->obsA2Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1108_A2_GPIO);
    outState->obsSwLevel = tmuxReadGpioLevelNoLock(CONFIG_TMUX1108_SW_GPIO);
    outState->obsSel1Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1134_SEL1_GPIO);
    outState->obsSel2Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1134_SEL2_GPIO);
    outState->obsSel3Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1134_SEL3_GPIO);
    outState->obsSel4Level = tmuxReadGpioLevelNoLock(CONFIG_TMUX1134_SEL4_GPIO);
    if (outState->tmux1134EnControllable) {
        outState->obsEnLevel = tmuxReadGpioLevelNoLock(CONFIG_TMUX1134_EN_GPIO);
        if (outState->obsEnLevel >= 0) {
            int offLevel = CONFIG_TMUX1134_EN_OFF_LEVEL ? 1 : 0;
            outState->obsTmux1134EnLogicalOn = (outState->obsEnLevel != offLevel);
            outState->obsTmux1134EnLogicalOnValid = true;
        } else {
            outState->obsTmux1134EnLogicalOn = outState->cmdTmux1134EnLogicalOn;
            outState->obsTmux1134EnLogicalOnValid = false;
        }
    } else {
        /* EN is hard-wired on this board; expose as not-controllable in observation. */
        outState->obsEnLevel = -1;
        outState->obsTmux1134EnLogicalOn = true;
        outState->obsTmux1134EnLogicalOnValid = false;
    }
    outState->obsSelaLevel = outState->obsSel1Level;
    outState->obsSelbLevel = outState->obsSel2Level;
    portEXIT_CRITICAL(&s_tmux_lock);

    return ESP_OK;
}

esp_err_t tmuxSwitchLogControlSnapshot(const tmuxSwitchSnapshotContext_t *context)
{
    tmuxSwitchControlState_t ctrl = {0};
    esp_err_t err = tmuxSwitchGetControlState(&ctrl);
    if (err != ESP_OK) {
        printf("DBGTMUXCTRL,stage=%s,routeLabel=%s,targetPath=%s,status=ctrl_state_unavailable,err=%ld\n",
               tmuxStringOrNa(context ? context->stage : NULL),
               tmuxStringOrNa(context ? context->routeLabel : NULL),
               tmuxStringOrNa(context ? context->targetPath : NULL),
               (long)err);
        return err;
    }

    tmuxLogSnapshotLine(context,
                        "A0",
                        CONFIG_TMUX1108_A0_GPIO,
                        ctrl.cmdA0Level,
                        ctrl.obsA0Level,
                        "active_high(bit0)",
                        "row_select_bit0",
                        "low");
    tmuxLogSnapshotLine(context,
                        "A1",
                        CONFIG_TMUX1108_A1_GPIO,
                        ctrl.cmdA1Level,
                        ctrl.obsA1Level,
                        "active_high(bit1)",
                        "row_select_bit1",
                        "low");
    tmuxLogSnapshotLine(context,
                        "A2",
                        CONFIG_TMUX1108_A2_GPIO,
                        ctrl.cmdA2Level,
                        ctrl.obsA2Level,
                        "active_high(bit2)",
                        "row_select_bit2",
                        "low");
    tmuxLogSnapshotLine(context,
                        "SW",
                        CONFIG_TMUX1108_SW_GPIO,
                        ctrl.cmdSwLevel,
                        ctrl.obsSwLevel,
                        CONFIG_TMUX1108_SW_REF_LEVEL ? "active_high_selects_REF" : "active_high_selects_GND",
                        tmuxSwSemanticFromLevel(ctrl.cmdSwLevel),
                        "low");
    tmuxLogSnapshotLine(context,
                        "SELA",
                        CONFIG_TMUX1134_SEL1_GPIO,
                        ctrl.cmdSelaLevel,
                        ctrl.obsSelaLevel,
                        "active_high_selects_SxA",
                        tmux1134SelSemanticFromLevel(ctrl.cmdSelaLevel),
                        "low");
    tmuxLogSnapshotLine(context,
                        "SELB",
                        CONFIG_TMUX1134_SEL2_GPIO,
                        ctrl.cmdSelbLevel,
                        ctrl.obsSelbLevel,
                        "active_high_selects_SxA",
                        tmux1134SelSemanticFromLevel(ctrl.cmdSelbLevel),
                        "low");
    tmuxLogSnapshotLine(context,
                        "SEL3",
                        CONFIG_TMUX1134_SEL3_GPIO,
                        ctrl.cmdSel3Level,
                        ctrl.obsSel3Level,
                        "active_high_selects_SxA",
                        tmux1134SelSemanticFromLevel(ctrl.cmdSel3Level),
                        (CONFIG_TMUX1134_SEL3_GPIO >= 0) ? "low" : "not_available");
    tmuxLogSnapshotLine(context,
                        "SEL4",
                        CONFIG_TMUX1134_SEL4_GPIO,
                        ctrl.cmdSel4Level,
                        ctrl.obsSel4Level,
                        "active_high_selects_SxA",
                        tmux1134SelSemanticFromLevel(ctrl.cmdSel4Level),
                        (CONFIG_TMUX1134_SEL4_GPIO >= 0) ? "low" : "not_available");
    tmuxLogSnapshotLine(context,
                        "EN",
                        CONFIG_TMUX1134_EN_GPIO,
                        ctrl.cmdEnLevel,
                        ctrl.obsEnLevel,
                        CONFIG_TMUX1134_EN_OFF_LEVEL ? "active_low_enables" : "active_high_enables",
                        tmux1134EnSemanticFromLevel(ctrl.cmdEnLevel),
                        (CONFIG_TMUX1134_EN_GPIO >= 0) ? "low" : "not_available");

    uint32_t mismatchCount = 0u;
    const int cmdLevels[] = {
        ctrl.cmdA0Level,
        ctrl.cmdA1Level,
        ctrl.cmdA2Level,
        ctrl.cmdSwLevel,
        ctrl.cmdSelaLevel,
        ctrl.cmdSelbLevel,
        ctrl.cmdSel3Level,
        ctrl.cmdSel4Level,
        ctrl.cmdEnLevel,
    };
    const int obsLevels[] = {
        ctrl.obsA0Level,
        ctrl.obsA1Level,
        ctrl.obsA2Level,
        ctrl.obsSwLevel,
        ctrl.obsSelaLevel,
        ctrl.obsSelbLevel,
        ctrl.obsSel3Level,
        ctrl.obsSel4Level,
        ctrl.obsEnLevel,
    };
    for (size_t i = 0u; i < (sizeof(cmdLevels) / sizeof(cmdLevels[0])); ++i) {
        if (cmdLevels[i] >= 0 && obsLevels[i] >= 0 && cmdLevels[i] != obsLevels[i]) {
            mismatchCount++;
        }
    }

    printf("DBGTMUXCTRL,stage=%s,routeLabel=%s,targetPath=%s,status=%s,routeConsistency=%s,mismatchCount=%lu,"
           "note=observedLevel_is_mcu_gpio_only_not_analog_conduction_proof\n",
           tmuxStringOrNa(context ? context->stage : NULL),
           tmuxStringOrNa(context ? context->routeLabel : NULL),
           tmuxStringOrNa(context ? context->targetPath : NULL),
           (mismatchCount == 0u) ? "command_observed_match" : "command_observed_mismatch",
           (mismatchCount == 0u) ? "pass" : "fail",
           (unsigned long)mismatchCount);
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
