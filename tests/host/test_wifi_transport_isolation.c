/* Same actual owner/backend and persistent-slot harness, plus actual shared
 * Transport entry points. SDK/RTOS is synthetic; no hardware claims. */
#define main ownerHarnessMain
#include "test_wifi_owner.c"
#undef main
#include "sensorarrayTransport.h"
#include "sensorarrayTransportPool.h"
#include "sensorarrayTransportGuaranteedText.h"
#include "sensorarrayTransportWifiControl.h"
#include "sensorarraySerialIo.h"
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT 1
#define SENSORARRAY_BLE_CH_CTRL 2u
#define SENSORARRAY_BLE_CH_LOG 1u
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD ""
#define configTICK_RATE_HZ 100u
static char s_deviceName[]="test";
static portMUX_TYPE g_sensorarrayTransportStatsMux=portMUX_INITIALIZER_UNLOCKED;
static sensorarrayTransportStats_t g_sensorarrayTransportStats;
static bool s_protectedRetryActive;
static sensorarrayTransportDescriptor_t s_protectedRetry;
static sensorarrayTransportPayloadSlot_t unusedSlot;
static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(const sensorarrayTransportDescriptor_t *d) {return &unusedSlot;}
static unsigned usbReplies, bleReplies, serialLogs;
static bool haveGuaranteed;
static char guaranteed[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
bool sensorarrayTransportWifiSinkEnabled(void) {return sensorarrayWifiIsReady();}
bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t channel) {return true;}
static bool sensorarrayBleGetSendGeneration(unsigned channel,uint32_t *generation) {*generation=1;return true;}
static esp_err_t sensorarrayBleNotify(unsigned channel,const uint8_t *data,size_t length) {assert(data&&length);bleReplies++;return ESP_OK;}
static esp_err_t sensorarrayBleNotifyLifecycleForGeneration(const uint8_t *data,size_t length,uint32_t gen) {assert(data&&length&&gen==1);bleReplies++;return ESP_OK;}
esp_err_t sensorarraySerialIoQueue(const char *data,size_t length,bool priority) {assert(data&&length&&priority);usbReplies++;return ESP_OK;}
void sensorarrayTransportNoteSerialLog(bool accepted) {assert(accepted);serialLogs++;}
bool sensorarrayTransportGuaranteedTextTakeDrain(char *text,size_t size,size_t *length) {if(!haveGuaranteed)return false;*length=strlen(guaranteed);assert(*length<size);memcpy(text,guaranteed,*length+1);haveGuaranteed=false;return true;}
esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *data,size_t length,bool *queued) {*queued=true;assert(data&&length);return ESP_OK;}
#include "wifi_shared_functions.h"
int main(void)
{
    setup();sensorarrayWifiSetOwnerCallbacks(sensorarrayTransportWifiCompleted,NULL,wakeCallback);
    char response[512];assert(sensorarrayTransportWifiCommand("WIFI=AP",response,sizeof(response))==ESP_OK);
    assert(strstr(response,"accepted=1,pending=1")&&sdkCalls==0);
    assert(sensorarrayTransportWifiCommand("WIFI?",response,sizeof(response))==ESP_OK&&strstr(response,"pending=1"));
    step();sensorarrayWifiOwnerNoteStationEvent(true);
    sensorarrayTransportReplyTarget_t target={.kind=SENSORARRAY_TRANSPORT_REPLY_WIFI,.wifiPeer={.ipv4=123,.port=456}};
    char data[512];memset(data,'z',sizeof(data));unsigned calls=sdkCalls;
    assert(sensorarrayTransportPublishControlReply(&target,data,sizeof(data))==ESP_OK&&sdkCalls==calls&&g_sensorarrayTransportStats.wifiCtrlTx==0);
    memset(data,'x',sizeof(data));step();assert(g_sensorarrayTransportStats.wifiCtrlTx==1&&copiedLength==512&&copiedBytes[511]=='z'&&ntohs(copiedTarget.sin_port)==456);
    strcpy(guaranteed,"MFAULT,first_original=synthetic\n");haveGuaranteed=true;calls=sdkCalls;
    sensorarrayTransportDrainGuaranteedText();assert(sdkCalls==calls&&usbReplies==1&&bleReplies==1&&serialLogs==1&&g_sensorarrayTransportStats.wifiLogSent==0);
    step();assert(g_sensorarrayTransportStats.wifiLogSent==1&&copiedLength==strlen(guaranteed)&&memcmp(copiedBytes,guaranteed,copiedLength)==0);
    assert(submit(0,false,"science",7,0)==ESP_OK);blockPhase=SENSORARRAY_WIFI_PHASE_SEND;pthread_t thread=blocked();
    assert(sensorarrayTransportWaitTicks()==10u);calls=sdkCalls;
    target.kind=SENSORARRAY_TRANSPORT_REPLY_SERIAL;assert(sensorarrayTransportPublishControlReply(&target,"CONFIG",6)==ESP_OK);
    target.kind=SENSORARRAY_TRANSPORT_REPLY_BLE;assert(sensorarrayTransportPublishControlReply(&target,"CONFIG",6)==ESP_OK);
    assert(sensorarrayTransportWifiCommand("WIFIGUARD?",response,sizeof(response))==ESP_OK&&strstr(response,"active=1"));
    assert(usbReplies==2&&bleReplies==2&&sdkCalls==calls&&g_sensorarrayTransportStats.wifiDataSent==0);
    observeAfter(500000);assert(snapshot().quarantined&&snapshot().heldSlots==1);
    assert(sensorarrayTransportWifiCommand("WIFI=AP",response,sizeof(response))==ESP_ERR_INVALID_STATE&&strstr(response,"accepted=0"));
    assert(sensorarrayTransportWifiCommand("WIFI=OFF",response,sizeof(response))==ESP_OK&&strstr(response,"scope=output_gate_only"));
    sensorarrayTransportWifiDrainTerminal();sensorarrayTransportWifiDrainTerminal();assert(sdkCalls==calls&&snapshot().heldSlots==1);
    target.kind=SENSORARRAY_TRANSPORT_REPLY_WIFI;assert(sensorarrayTransportPublishControlReply(&target,"QUERY",5)==ESP_ERR_INVALID_STATE);
    target.kind=SENSORARRAY_TRANSPORT_REPLY_SERIAL;assert(sensorarrayTransportPublishControlReply(&target,"WIFIREQ",7)==ESP_OK);
    unblock(thread);assert(g_sensorarrayTransportStats.wifiDataSent==1&&snapshot().quarantined&&snapshot().completionKnown&&snapshot().lateReturns==1);
    assert(sensorarrayTransportWaitTicks()==portMAX_DELAY);
    assert(!sdkOwnerViolation);free(s_owner.payload);
    printf("WIFI_TRANSPORT_ISOLATION,actual_shared_bodies=9,queued_not_sent=PASS,CTRL512_deepcopy=PASS,lifecycle=PASS,blocked_SDK_USB_BLE_queries=PASS,deadline_poll=PASS,late_actual_send=PASS,hardware=NOT_RUN\n");return 0;
}
