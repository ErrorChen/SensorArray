#pragma once
#define _GNU_SOURCE 1
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include "esp_err.h"
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef uint8_t StackType_t;
typedef void *TaskHandle_t;
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(m) pthread_mutex_lock(m)
#define portEXIT_CRITICAL(m) pthread_mutex_unlock(m)
#define pdPASS 1
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define CONFIG_SENSORARRAY_NET_TASK_CORE 0
#define ESP_TIMER_TASK 0
#define MALLOC_CAP_SPIRAM 1u
#define MALLOC_CAP_8BIT 2u
#define SENSORARRAY_WORK_BUDGET_WIFI_CTRL 6
#define SENSORARRAY_WORK_BUDGET_WIFI_OWNER 7
int64_t esp_timer_get_time(void);
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *, uint32_t, void *, unsigned, TaskHandle_t *, int);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void xTaskNotifyGive(TaskHandle_t);
uint32_t ulTaskNotifyTake(int, uint32_t);
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t);
void *heap_caps_malloc(size_t, unsigned);
esp_err_t sensorarrayWorkBudgetInit(void);
bool sensorarrayWorkBudgetCheckpoint(int);
typedef void *esp_timer_handle_t;
typedef struct {void (*callback)(void *); void *arg; int dispatch_method; const char *name; bool skip_unhandled_events;} esp_timer_create_args_t;
esp_err_t esp_timer_create(const esp_timer_create_args_t *, esp_timer_handle_t *);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t, uint64_t);
typedef int esp_event_base_t;
#define WIFI_EVENT 1
#define ESP_EVENT_ANY_ID -1
#define WIFI_EVENT_AP_STACONNECTED 2
#define WIFI_EVENT_AP_STADISCONNECTED 3
#define WIFI_PS_NONE 0
#define WIFI_IF_AP 1
#define WIFI_MODE_AP 2
#define WIFI_PROTOCOL_11B 1
#define WIFI_PROTOCOL_11G 2
#define WIFI_PROTOCOL_11N 4
#define WIFI_BW_HT40 40
#define WIFI_BW_HT20 20
#define WIFI_AUTH_WPA2_PSK 2
#define WIFI_AUTH_OPEN 0
typedef struct {int unused;} esp_netif_t;
typedef struct {int unused;} esp_netif_config_t;
#define ESP_NETIF_DEFAULT_WIFI_AP() ((esp_netif_config_t){0})
typedef struct {int static_rx_buf_num,dynamic_rx_buf_num,dynamic_tx_buf_num,rx_ba_win,mgmt_sbuf_num;} wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() ((wifi_init_config_t){0})
typedef struct { struct {uint8_t ssid[32],password[64],ssid_len,channel,max_connection;int authmode;} ap; } wifi_config_t;
typedef struct {unsigned num;} wifi_sta_list_t;
esp_err_t esp_netif_init(void);
esp_err_t esp_event_loop_create_default(void);
esp_netif_t *esp_netif_new(const esp_netif_config_t *);
esp_err_t esp_netif_attach_wifi_ap(esp_netif_t *);
esp_err_t esp_wifi_set_default_wifi_ap_handlers(void);
esp_err_t esp_wifi_clear_default_wifi_driver_and_handlers(void *);
void esp_netif_destroy(esp_netif_t *);
esp_err_t esp_wifi_init(const wifi_init_config_t *);
esp_err_t esp_wifi_deinit(void);
esp_err_t esp_wifi_stop(void);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_set_mode(int);
esp_err_t esp_wifi_set_config(int, const wifi_config_t *);
esp_err_t esp_wifi_set_max_tx_power(int);
esp_err_t esp_wifi_set_ps(int);
esp_err_t esp_wifi_set_protocol(int,int);
esp_err_t esp_wifi_set_bandwidth(int,int);
esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t *);
typedef void (*fake_event_handler_t)(void *,esp_event_base_t,int32_t,void *);
esp_err_t esp_event_handler_register(int,int,fake_event_handler_t,void *);
esp_err_t esp_event_handler_unregister(int,int,fake_event_handler_t);
int fake_socket(int,int,int);
int fake_setsockopt(int,int,int,const void *,socklen_t);
int fake_bind(int,const struct sockaddr *,socklen_t);
int fake_close(int);
ssize_t fake_sendto(int,const void *,size_t,int,const struct sockaddr *,socklen_t);
ssize_t fake_recvfrom(int,void *,size_t,int,struct sockaddr *,socklen_t *);
#define socket fake_socket
#define setsockopt fake_setsockopt
#define bind fake_bind
#define close fake_close
#define sendto fake_sendto
#define recvfrom fake_recvfrom
