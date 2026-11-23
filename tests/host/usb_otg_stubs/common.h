#pragma once
/* Synthetic host-only SDK/RTOS surface. No USB controller or hardware access. */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdatomic.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107
#define DRAM_ATTR
#define RTC_NOINIT_ATTR
#define IRAM_ATTR
#define CONFIG_SENSORARRAY_USB_OTG 1
#define ESP_INFINITE_LOOP() do { for (;;) { atomic_signal_fence(memory_order_seq_cst); } } while (0)
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define portYIELD_FROM_ISR() ((void)0)
#define pdFALSE 0
#define pdTRUE 1
#define portTICK_PERIOD_MS 10u
#define pdMS_TO_TICKS(n) ((n) / 10u)
typedef struct { unsigned capacity, itemSize, count, read, write; uint8_t *storage; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
QueueHandle_t xQueueCreateStatic(unsigned, unsigned, uint8_t *, StaticQueue_t *);
BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void xTaskNotifyGive(TaskHandle_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *);
void vTaskDelay(TickType_t);
int64_t esp_timer_get_time(void);

typedef void *usb_phy_handle_t;
typedef struct { int controller, target, otg_mode, otg_speed; } usb_phy_config_t;
#define USB_PHY_CTRL_OTG 1
#define USB_PHY_TARGET_INT 1
#define USB_OTG_MODE_DEVICE 1
#define USB_PHY_SPEED_FULL 1
esp_err_t usb_new_phy(const usb_phy_config_t *, usb_phy_handle_t *);
void usb_serial_jtag_hal_phy_set_external(void *, bool);
void usb_serial_jtag_ll_phy_disable_pull_override(void);
esp_err_t esp_efuse_mac_get_default(uint8_t *);
enum { ESP_EFUSE_DIS_USB_SERIAL_JTAG, ESP_EFUSE_DIS_DOWNLOAD_MODE,
       ESP_EFUSE_DIS_FORCE_DOWNLOAD, ESP_EFUSE_DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE };
bool esp_efuse_read_field_bit(unsigned);
void esp_restart(void);
void esp_restart_noos(void) __attribute__((noreturn));
void __wrap_esp_restart_noos(void) __attribute__((noreturn));
void esp_rom_software_reset_system(void);
void chip_usb_set_persist_flags(uint32_t);
void xt_ints_off(uint32_t);
uint32_t clk_ll_xtal_load_freq_mhz(void);
void esp_clk_utils_mspi_speed_mode_sync_before_cpu_freq_switching(uint32_t, uint32_t);
void rtc_clk_cpu_set_to_default_config(void);
int esp_cpu_get_core_id(void);
void esp_cpu_unstall(int);
void __real_esp_restart_noos(void) __attribute__((noreturn));
#define SOC_CPU_CORES_NUM 2

enum { RTC_CNTL_USB_CONF_REG, SYSTEM_PERIP_RST_EN0_REG, RTC_CNTL_OPTION1_REG,
       USB_GAHBCFG_REG, HOST_REGISTER_COUNT };
#define RTC_CNTL_USB_RESET_DISABLE (1u << 17)
#define RTC_CNTL_FORCE_DOWNLOAD_BOOT 1u
#define SYSTEM_USB_RST (1u << 11)
#define USB_DMAEN (1u << 5)
uint32_t hostRegisterRead(unsigned);
void hostRegisterSet(unsigned, uint32_t);
void hostRegisterClear(unsigned, uint32_t);
#define REG_READ(r) hostRegisterRead(r)
#define REG_SET_BIT(r, bits) hostRegisterSet(r, bits)
#define REG_CLR_BIT(r, bits) hostRegisterClear(r, bits)
#define DPORT_SET_PERI_REG_MASK(r, bits) hostRegisterSet(r, bits)
#define DPORT_READ_PERI_REG(r) hostRegisterRead(r)
#define PERIPH_RCC_ATOMIC() for (unsigned hostRccOnce = 1; hostRccOnce; hostRccOnce = 0)
typedef struct { struct { unsigned dmaen; } gahbcfg_reg; } host_usb_dwc_t;
extern host_usb_dwc_t USB_DWC;
