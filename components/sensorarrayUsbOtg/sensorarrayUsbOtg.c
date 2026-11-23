#include "sensorarrayUsbOtg.h"
#include <stdatomic.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_macros.h"
#include "esp_cpu.h"
#include "xtensa/xtensa_api.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/usb_phy.h"
#include "esp_private/system_internal.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "hal/usb_serial_jtag_hal.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/system_reg.h"
#include "soc/dport_access.h"
#include "soc/usb_dwc_struct.h"
#include "hal/usb_serial_jtag_ll.h"
#include "rom/usb/chip_usb_dw_wrapper.h"
#include "tusb.h"
#include "device/dcd.h"

/* 本轮新增. TinyUSB/DWC2 and the PHY have exactly one task owner. No uninstall,
 * task deletion, application buffer DMA, or post-timeout reuse exists here. */
#define USB_COMMAND_MAX 192u
#define USB_COMMAND_SLOTS 4u
#define USB_RECOVERY_MAGIC 0x53415552u
/* Engineering detach margin, not a claim of measured D+/D- signaling or a
 * USB-specified 20 ms minimum. Included in PrepareRestart's total deadline. */
#define USB_DETACH_DWELL_US 20000
typedef struct { uint32_t epoch; uint16_t size; char bytes[USB_COMMAND_MAX]; } usbCommand_t;
static DRAM_ATTR atomic_uint s_epoch;
static DRAM_ATTR atomic_uint s_resets;
static DRAM_ATTR atomic_uint s_processedEpoch;
static DRAM_ATTR atomic_bool s_stopRequested;
static DRAM_ATTR atomic_bool s_quiesced;
static DRAM_ATTR atomic_uint s_hardwareMayOwnDma;
static TaskHandle_t s_owner;
static usb_phy_handle_t s_phy;
static bool s_irqInstalled;
/* Owner-only final-handoff phase. A separate flag permits a real start at t=0. */
static bool s_detachStarted;
static int64_t s_detachedAtUs;
static portMUX_TYPE s_statsMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayUsbOtgStats_t s_stats;
static StaticQueue_t s_rxStorage;
static uint8_t s_rxBytes[USB_COMMAND_SLOTS * sizeof(usbCommand_t)];
static QueueHandle_t s_rxQueue;
static usbCommand_t s_buildCommand;
static bool s_rxDiscard;
static RTC_NOINIT_ATTR uint32_t s_recoveryMagic;
static RTC_NOINIT_ATTR uint32_t s_recoveryInverse;
static bool s_backendSelected;
static bool s_enabled;

bool sensorarrayUsbOtgEnabled(void)
{
    /* Called by the main startup thread before any USB/transport task exists. */
    if (!s_backendSelected) {
#ifdef CONFIG_SENSORARRAY_USB_OTG
        s_enabled = !(s_recoveryMagic == USB_RECOVERY_MAGIC &&
                      s_recoveryInverse == ~USB_RECOVERY_MAGIC);
#endif
        /* Reading the recovery latch must not consume it: an early panic in
         * this USJ boot has no OTG DMA owner to set it again. Only an explicit
         * USBBOOT with a verified armed ROM flag clears the latch below.
         * RTC retention is not guaranteed across removal of chip power. */
        s_backendSelected = true;
    }
    return s_enabled;
}

bool sensorarrayUsbOtgCdcEnabled(void)
{
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
    return sensorarrayUsbOtgEnabled();
#else
    return false;
#endif
}

uint32_t sensorarrayUsbOtgEpoch(void) { return atomic_load(&s_epoch); }

void tud_event_hook_cb(uint8_t rhport, uint32_t event, bool inIsr)
{
    (void)rhport;
    if (event == DCD_EVENT_BUS_RESET || event == DCD_EVENT_UNPLUGGED) {
        atomic_fetch_add(&s_epoch, 1u);
        if (event == DCD_EVENT_BUS_RESET) { atomic_fetch_add(&s_resets, 1u); }
    }
    if (s_owner) {
        if (inIsr) {
            BaseType_t woke = pdFALSE;
            vTaskNotifyGiveFromISR(s_owner, &woke);
            if (woke) { portYIELD_FROM_ISR(); }
        } else { xTaskNotifyGive(s_owner); }
    }
}

void tud_mount_cb(void)
{
    portENTER_CRITICAL(&s_statsMux);
    s_stats.mounted = true; s_stats.suspended = false; s_stats.mounts++;
    portEXIT_CRITICAL(&s_statsMux);
}
void tud_umount_cb(void)
{
    atomic_fetch_add(&s_epoch, 1u);
    portENTER_CRITICAL(&s_statsMux);
    s_stats.mounted = false; s_stats.unmounts++;
    memset(s_stats.cdcDtr, 0, sizeof(s_stats.cdcDtr));
    portEXIT_CRITICAL(&s_statsMux);
}
void tud_suspend_cb(bool remoteWake)
{
    (void)remoteWake;
    portENTER_CRITICAL(&s_statsMux); s_stats.suspended = true; portEXIT_CRITICAL(&s_statsMux);
}
void tud_resume_cb(void)
{
    portENTER_CRITICAL(&s_statsMux); s_stats.suspended = false; portEXIT_CRITICAL(&s_statsMux);
}
#ifndef CONFIG_SENSORARRAY_USB_OTG_CDC
void tud_vendor_tx_cb(uint8_t itf, uint32_t bytes)
{
    if (itf >= 2u) { return; }
    /* Stock vendor class discards xfer_result_t. This is deliberately called
     * callbackBytes, NOT successful bus bytes or host application receipt. */
    portENTER_CRITICAL(&s_statsMux); s_stats.callbackBytes[itf] += bytes; portEXIT_CRITICAL(&s_statsMux);
}
#endif

void tud_cdc_tx_complete_cb(uint8_t itf)
{
    if (itf >= CFG_TUD_CDC) { return; }
    /* CDC supplies neither byte count nor xfer_result_t. This count cannot
     * identify DATA versus CTRL, successful bus bytes, or host receipt. */
    portENTER_CRITICAL(&s_statsMux); s_stats.cdcTxCallbacks[itf]++; portEXIT_CRITICAL(&s_statsMux);
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)rts;
    if (itf >= CFG_TUD_CDC) { return; }
    portENTER_CRITICAL(&s_statsMux); s_stats.cdcDtr[itf] = dtr; portEXIT_CRITICAL(&s_statsMux);
    /* Ordinary DTR/baud changes never request reset or clear active DMA. */
}

static void sensorarrayUsbConsumeCommands(void)
{
    uint32_t epoch = atomic_load(&s_processedEpoch);
    if (epoch != sensorarrayUsbOtgEpoch()) { return; }
    if (s_buildCommand.epoch != epoch) {
        s_buildCommand = (usbCommand_t){.epoch = epoch};
        s_rxDiscard = false;
    }
    uint8_t bytes[64];
    /* Maximum the bounded 512-byte CTRL FIFO per owner iteration. */
    for (unsigned chunk = 0; chunk < 8u; ++chunk) {
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
        uint32_t n = tud_cdc_n_read(0, bytes, sizeof(bytes));
#else
        uint32_t n = tud_vendor_n_read(1, bytes, sizeof(bytes));
#endif
        if (epoch != sensorarrayUsbOtgEpoch()) {
            s_buildCommand.size = 0u; s_rxDiscard = false; return;
        }
        if (n == 0u) { break; }
        portENTER_CRITICAL(&s_statsMux);
        s_stats.rxBytes += n; s_stats.lastRxUs = esp_timer_get_time();
        portEXIT_CRITICAL(&s_statsMux);
        for (uint32_t i = 0; i < n; ++i) {
            uint8_t ch = bytes[i];
            if (ch == '\n' || ch == '\r') {
                if (!s_rxDiscard && s_buildCommand.size) {
                    s_buildCommand.bytes[s_buildCommand.size++] = '\n';
                    if (epoch != sensorarrayUsbOtgEpoch()) {
                        s_buildCommand.size = 0; s_rxDiscard = false; return;
                    }
                    if (xQueueSend(s_rxQueue, &s_buildCommand, 0) != pdTRUE) {
                        portENTER_CRITICAL(&s_statsMux); s_stats.rxDropped++; portEXIT_CRITICAL(&s_statsMux);
                    }
                }
                s_buildCommand.size = 0u; s_rxDiscard = false;
            } else if (!s_rxDiscard) {
                if (s_buildCommand.size < USB_COMMAND_MAX - 1u) {
                    s_buildCommand.bytes[s_buildCommand.size++] = (char)ch;
                } else {
                    s_rxDiscard = true;
                    portENTER_CRITICAL(&s_statsMux); s_stats.rxOversize++; portEXIT_CRITICAL(&s_statsMux);
                }
            }
        }
    }
    /* Debug OUT never interprets commands. Drain finite amounts so
     * an accidental debug writer cannot monopolize the USB owner. */
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
    (void)tud_cdc_n_read(1, bytes, sizeof(bytes));
#else
    (void)tud_vendor_n_read(0, bytes, sizeof(bytes));
    (void)tud_cdc_read(bytes, sizeof(bytes));
#endif
}

esp_err_t sensorarrayUsbOtgOwnerInit(void)
{
    s_owner = xTaskGetCurrentTaskHandle();
    s_rxQueue = xQueueCreateStatic(USB_COMMAND_SLOTS, sizeof(usbCommand_t), s_rxBytes, &s_rxStorage);
    sensorarrayUsbOtgInitSerialString();
    const usb_phy_config_t phy = {.controller = USB_PHY_CTRL_OTG, .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE, .otg_speed = USB_PHY_SPEED_FULL};
    esp_err_t err = usb_new_phy(&phy, &s_phy);
    if (err == ESP_OK) {
        atomic_store(&s_hardwareMayOwnDma, 1u);
        const tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
        if (!tusb_rhport_init(0, &init)) { err = ESP_FAIL; }
        else { s_irqInstalled = true; }
    }
    bool dma = err == ESP_OK && USB_DWC.gahbcfg_reg.dmaen != 0u;
    if (err == ESP_OK && !dma) { err = ESP_ERR_INVALID_STATE; }
    portENTER_CRITICAL(&s_statsMux);
    s_stats.initialized = err == ESP_OK; s_stats.initError = err; s_stats.dmaEnabled = dma;
#ifndef CONFIG_SENSORARRAY_USB_OTG_CDC
    s_stats.callbackBytesKnown[SENSORARRAY_USB_DATA] = true;
    s_stats.callbackBytesKnown[SENSORARRAY_USB_CTRL] = true;
#endif
    portEXIT_CRITICAL(&s_statsMux);
    return err;
}

static void sensorarrayUsbOtgOwnerQuiesce(void)
{
    /* Final reset only. No stock deinit loops, no deletion of active workers.
     * Owner allocated the ISR on this same core, so its ISR cannot be running
     * concurrently here. Hardware module reset stops the DMA master. */
    if (s_irqInstalled) { dcd_int_disable(0); s_irqInstalled = false; }
    if (s_phy) {
        if (!s_detachStarted) {
            /* DWC2 disconnect holds the internal PHY in its detached state.
             * Do not immediately reset USB_WRAP or select USJ's pull-up:
             * the host needs an opportunity to observe the detach. IRQ is
             * disabled, but DMA may still own its persistent buffers. */
            tud_disconnect();
            s_detachedAtUs = esp_timer_get_time();
            s_detachStarted = true;
            return;
        }
        if (esp_timer_get_time() - s_detachedAtUs < USB_DETACH_DWELL_US) { return; }
        chip_usb_set_persist_flags(0);
        REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
        PERIPH_RCC_ATOMIC() {
            DPORT_SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_USB_RST);
        }
        if ((DPORT_READ_PERI_REG(SYSTEM_PERIP_RST_EN0_REG) & SYSTEM_USB_RST) == 0u) { return; }
    }
    usb_serial_jtag_hal_phy_set_external(NULL, false);
    usb_serial_jtag_ll_phy_disable_pull_override();
    atomic_store(&s_hardwareMayOwnDma, 0u);
    portENTER_CRITICAL(&s_statsMux);
    s_stats.initialized = false; s_stats.mounted = false; s_stats.quiesced = true;
    portEXIT_CRITICAL(&s_statsMux);
    atomic_store_explicit(&s_quiesced, true, memory_order_release);
}

void sensorarrayUsbOtgOwnerPoll(void)
{
    if (atomic_load(&s_quiesced)) { return; }
    if (atomic_load(&s_stopRequested)) { sensorarrayUsbOtgOwnerQuiesce(); return; }
    if (!s_stats.initialized) { return; }
    uint32_t before = sensorarrayUsbOtgEpoch();
    tud_task_ext(0, false);
    uint32_t after = sensorarrayUsbOtgEpoch();
    if (atomic_load(&s_processedEpoch) != after) {
        s_buildCommand.size = 0u; s_rxDiscard = false;
        portENTER_CRITICAL(&s_statsMux);
        s_stats.mounted = false; memset(s_stats.cdcDtr, 0, sizeof(s_stats.cdcDtr));
        portEXIT_CRITICAL(&s_statsMux);
        /* ISR notification is not proof the queued reset was processed. The
         * reset may be event 17, beyond this iteration's 16-event budget. */
        if (before != after || tud_task_event_ready()) { return; }
        atomic_store(&s_processedEpoch, after);
    }
    if (after != sensorarrayUsbOtgEpoch()) { return; }
    portENTER_CRITICAL(&s_statsMux);
    s_stats.mounted = tud_mounted(); s_stats.suspended = tud_suspended();
    for (uint8_t itf = 0; itf < CFG_TUD_CDC; ++itf) {
        s_stats.cdcDtr[itf] = (tud_cdc_n_get_line_state(itf) & 1u) != 0u;
    }
    portEXIT_CRITICAL(&s_statsMux);
    sensorarrayUsbConsumeCommands();
}

bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane)
{
    if (atomic_load(&s_stopRequested) || atomic_load(&s_processedEpoch) != sensorarrayUsbOtgEpoch() ||
        !s_stats.initialized || !tud_ready()) { return false; }
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
    if (lane < 2u) {
        /* Formal commands/data also work for hosts that do not assert DTR.
         * Non-overwritable FIFO preserves accepted prefixes on DTR changes. */
        return tud_cdc_n_ready(0) && tud_cdc_n_write_available(0) != 0u;
    }
    return lane == SENSORARRAY_USB_DEBUG && tud_cdc_n_connected(1) &&
        tud_cdc_n_write_available(1) != 0u;
#else
    return lane < 2u ? tud_vendor_n_write_available((uint8_t)lane) != 0u :
        (lane == SENSORARRAY_USB_DEBUG && tud_cdc_connected() && tud_cdc_write_available() != 0u);
#endif
}

int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t size)
{
    if (!sensorarrayUsbOtgOwnerCanWrite(lane)) { return 0; }
    uint32_t n;
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
    uint8_t itf = lane == SENSORARRAY_USB_DEBUG ? 1u : 0u;
    n = tud_cdc_n_write(itf, data, (uint32_t)size);
    (void)tud_cdc_n_write_flush(itf);
#else
    if (lane < 2u) {
        n = tud_vendor_n_write((uint8_t)lane, data, (uint32_t)size);
        (void)tud_vendor_n_write_flush((uint8_t)lane);
    } else {
        n = tud_cdc_write(data, (uint32_t)size);
        (void)tud_cdc_write_flush();
    }
#endif
    portENTER_CRITICAL(&s_statsMux);
    s_stats.accepted[lane] += n;
    if (n) { s_stats.lastTxUs = esp_timer_get_time(); }
    portEXIT_CRITICAL(&s_statsMux);
    return (int)n;
}

int sensorarrayUsbOtgRead(void *buffer, size_t size, uint32_t *epoch)
{
    /* This static staging belongs exclusively to the one control consumer. */
    static usbCommand_t pending;
    static size_t offset;
    if (!buffer || !size || !s_rxQueue) { return 0; }
    uint32_t current = sensorarrayUsbOtgEpoch();
    if (pending.epoch != current) { pending.size = 0; offset = 0; }
    if (offset == pending.size) {
        if (xQueueReceive(s_rxQueue, &pending, 1u) != pdTRUE) { return 0; }
        offset = 0;
        if (pending.epoch != sensorarrayUsbOtgEpoch()) { pending.size = 0; return 0; }
    }
    size_t n = pending.size - offset;
    if (n > size) { n = size; }
    memcpy(buffer, pending.bytes + offset, n); offset += n;
    if (epoch) { *epoch = pending.epoch; }
    return (int)n;
}

void sensorarrayUsbOtgGetStats(sensorarrayUsbOtgStats_t *out)
{
    if (!out) { return; }
    portENTER_CRITICAL(&s_statsMux); *out = s_stats; portEXIT_CRITICAL(&s_statsMux);
    out->epoch = sensorarrayUsbOtgEpoch(); out->resets = atomic_load(&s_resets);
}

bool sensorarrayUsbOtgRomDownloadAvailable(void)
{
    return !esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_SERIAL_JTAG) &&
           !esp_efuse_read_field_bit(ESP_EFUSE_DIS_DOWNLOAD_MODE) &&
           !esp_efuse_read_field_bit(ESP_EFUSE_DIS_FORCE_DOWNLOAD) &&
           !esp_efuse_read_field_bit(ESP_EFUSE_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE);
}

esp_err_t sensorarrayUsbOtgPrepareRestart(int64_t deadlineUs, bool romDownload)
{
    if (romDownload && !sensorarrayUsbOtgRomDownloadAvailable()) { return ESP_ERR_NOT_SUPPORTED; }
    if (sensorarrayUsbOtgEnabled() && s_owner) {
        atomic_store(&s_stopRequested, true); xTaskNotifyGive(s_owner);
        while (!atomic_load_explicit(&s_quiesced, memory_order_acquire)) {
            if (esp_timer_get_time() >= deadlineUs) { return ESP_ERR_TIMEOUT; }
            vTaskDelay(1u);
        }
    }
    /* A tick or delayed caller can observe a valid but late cleanup ACK.
     * Keep that cleanup evidence; never turn it into on-time success or arm
     * ROM download after this transaction's absolute deadline. */
    if (esp_timer_get_time() >= deadlineUs) { return ESP_ERR_TIMEOUT; }
    if (romDownload) {
        REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
        if (!(REG_READ(RTC_CNTL_OPTION1_REG) & RTC_CNTL_FORCE_DOWNLOAD_BOOT)) {
            return ESP_ERR_INVALID_STATE;
        }
        /* This is the explicit recovery-release boundary, not proof of ROM
         * execution or USB enumeration. The retained flag directs even an
         * intervening CPU-reset panic to ROM; never clear the latch at boot
         * selection or an ordinary RESTART. Keep this boot's backend cached.
         * No promise is made that arbitrary hardware faults recover in SW. */
        atomic_thread_fence(memory_order_seq_cst);
        s_recoveryMagic = s_recoveryInverse = 0u;
        atomic_thread_fence(memory_order_seq_cst);
    }
    return ESP_OK;
}

/* ESP32-S3 only: TRM 7.1.3 SW_SYS_RST resets the digital system, including
 * SYSTEM_SOC_CLK_SEL (reset XTAL) and USB. Unlike an ESP32-only SDK helper,
 * this path does not retune/cache-freeze first: those helpers can wait on a
 * hardware bit or a stalled CPU's lock. Release the RTC-retained CPU stall
 * state, then request Core Reset directly. No RTOS, heap, log or TinyUSB call.
 * All state and instructions remain accessible with flash cache disabled. */
static void IRAM_ATTR sensorarrayUsbEmergencyCoreReset(void) __attribute__((noreturn));
static void IRAM_ATTR sensorarrayUsbEmergencyCoreReset(void)
{
    xt_ints_off(0xffffffffu);
    s_recoveryMagic = USB_RECOVERY_MAGIC; s_recoveryInverse = ~USB_RECOVERY_MAGIC;
    REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
    /* Core Reset preserves RTC stall registers. Releasing the other CPU is
     * required or the subsequent dual-core boot can wait forever. */
    int current = esp_cpu_get_core_id();
    for (unsigned core = 0; core < SOC_CPU_CORES_NUM; ++core) {
        if ((int)core != current) { esp_cpu_unstall((int)core); }
    }
    atomic_thread_fence(memory_order_seq_cst);
    esp_rom_software_reset_system();
    ESP_INFINITE_LOOP(); /* Terminal guard after the hardware reset request. */
}

void __real_esp_restart_noos(void) __attribute__((noreturn));
void IRAM_ATTR __wrap_esp_restart_noos(void)
{
    if (atomic_load(&s_hardwareMayOwnDma)) { sensorarrayUsbEmergencyCoreReset(); }
    __real_esp_restart_noos();
}

void sensorarrayUsbOtgRestart(bool romDownload)
{
    esp_err_t err = sensorarrayUsbOtgPrepareRestart(esp_timer_get_time() + 250000, romDownload);
    if (err == ESP_OK) { esp_restart(); }
    /* No owner ACK: no claimed cancellation or buffer reuse. A digital Core
     * Reset includes the USB peripheral (unlike the normal CPU restart).
     * Subsequent boots stay in USJ recovery until explicit USBBOOT release.
     * Never arm ROM download on this failure. */
    sensorarrayUsbEmergencyCoreReset();
}

void sensorarrayUsbOtgManualRestart(bool romDownload)
{
    esp_err_t err = sensorarrayUsbOtgPrepareRestart(esp_timer_get_time() + 250000, romDownload);
    if (err == ESP_OK) {
        /* Explicit manual escape on the pinned S3/IDF implementation only.
         * esp_restart() invokes shutdown handlers before its noos watchdog;
         * esp_wifi_stop in that list can wait indefinitely. Enter the same
         * wrapped noos terminal path directly. Its RTC watchdog is armed
         * before cache/clock/core teardown. Do not add scheduler-lock waits,
         * claim SDK cancellation, or use this entry for automatic recovery.
         * A successful ROM preparation keeps its existing release/latch. */
        esp_restart_noos();
    }
    /* Original timeout fallback: retained buffers are not reclaimed, and
     * ROM is not armed on a failed/late USB owner acknowledgement. */
    sensorarrayUsbEmergencyCoreReset();
}
