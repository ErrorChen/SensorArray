#include "common.h"
#include <assert.h>
#include <limits.h>
#include "../../components/sensorarrayWifi/sensorarrayWifiOwner.c"
#include "../../components/sensorarrayWifi/sensorarrayWifi.c"
#include "../../components/sensorarrayWifi/sensorarrayWifiWatchdog.c"

static atomic_uint_fast64_t clockUs = 1000;
static _Thread_local TaskHandle_t currentTask;
static int ownerToken, rxToken, timerToken;
static unsigned allocations, taskCreates, timerCreates, timerStarts, sdkCalls, receivedResults, wakeCalls;
static unsigned capsSeen; static size_t bytesSeen;
static bool failAlloc, failTask, failTimerCreate, failTimerStart;
static sensorarrayWifiPhase_t failPhase, blockPhase;
static unsigned failHit, phaseHits[32];
static esp_err_t injectedError = ESP_ERR_NO_MEM;
static int sendResult = INT_MAX;
static sensorarrayWifiTxResult_t lastResult;
static uint8_t copiedBytes[SENSORARRAY_WIFI_DATA_MAX];
static size_t copiedLength;
static struct sockaddr_in copiedTarget;
static esp_timer_create_args_t timerArgs;
static fake_event_handler_t eventHandler;
static pthread_mutex_t blockMux = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t blockCond = PTHREAD_COND_INITIALIZER;
static bool entered, releaseCall;
static bool sdkOwnerViolation;

int64_t esp_timer_get_time(void) { return (int64_t)atomic_load(&clockUs); }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return currentTask; }
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *),const char *name,uint32_t bytes,void *arg,unsigned priority,TaskHandle_t *handle,int core)
{
    taskCreates++;
    assert(bytes == 4096 && priority == 5 && core == 0);
    if (failTask || (strcmp(name,"wifiCtrl")==0 && failPhase==SENSORARRAY_WIFI_PHASE_RX_TASK_CREATE)) { return 0; }
    *handle = strcmp(name,"wifiOwner")==0 ? &ownerToken : &rxToken;
    return pdPASS;
}
void xTaskNotifyGive(TaskHandle_t task) { assert(task); }
uint32_t ulTaskNotifyTake(int clear,uint32_t ticks) { return 0; }
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t t) { return 2048; }
void *heap_caps_malloc(size_t bytes,unsigned caps) { allocations++; bytesSeen=bytes; capsSeen=caps; return failAlloc ? NULL : calloc(1,bytes); }
esp_err_t sensorarrayWorkBudgetInit(void) { return ESP_OK; }
bool sensorarrayWorkBudgetCheckpoint(int role) { assert(role==6 || role==7); return false; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *args,esp_timer_handle_t *out) { timerCreates++;timerArgs=*args;*out=&timerToken;return failTimerCreate?ESP_ERR_NO_MEM:ESP_OK; }
esp_err_t esp_timer_start_periodic(esp_timer_handle_t t,uint64_t us) { timerStarts++;assert(us==100000 && t==&timerToken);return failTimerStart?ESP_ERR_NO_MEM:ESP_OK; }
static esp_err_t sdk(void)
{
    sensorarrayWifiOwnerSnapshot_t s; sensorarrayWifiCopyOwnerSnapshot(&s);
    sdkCalls++; phaseHits[s.phase]++;
    if (currentTask != &ownerToken || !s.sdkActive) { sdkOwnerViolation=true; }
    if (s.phase == blockPhase) {
        pthread_mutex_lock(&blockMux); entered=true; pthread_cond_broadcast(&blockCond);
        while (!releaseCall) { pthread_cond_wait(&blockCond,&blockMux); }
        pthread_mutex_unlock(&blockMux);
    }
    if (s.phase == failPhase && (!failHit || phaseHits[s.phase]==failHit)) { return injectedError; }
    return ESP_OK;
}
esp_err_t esp_netif_init(void) {return sdk();}
esp_err_t esp_event_loop_create_default(void) {return sdk();}
esp_netif_t *esp_netif_new(const esp_netif_config_t *c) {static esp_netif_t n;return sdk()==ESP_OK?&n:NULL;}
esp_err_t esp_netif_attach_wifi_ap(esp_netif_t *p) {assert(p);return sdk();}
esp_err_t esp_wifi_set_default_wifi_ap_handlers(void) {return sdk();}
esp_err_t esp_wifi_clear_default_wifi_driver_and_handlers(void *p) {assert(p);return sdk();}
void esp_netif_destroy(esp_netif_t *p) {assert(p);(void)sdk();}
esp_err_t esp_wifi_init(const wifi_init_config_t *c) {assert(c->static_rx_buf_num==8);return sdk();}
esp_err_t esp_wifi_deinit(void) {return sdk();}
esp_err_t esp_wifi_stop(void) {return sdk();}
esp_err_t esp_wifi_start(void) {return sdk();}
esp_err_t esp_wifi_set_mode(int m) {return sdk();}
esp_err_t esp_wifi_set_config(int i,const wifi_config_t *c) {assert(c->ap.ssid_len==4);return sdk();}
esp_err_t esp_wifi_set_max_tx_power(int x) {return sdk();}
esp_err_t esp_wifi_set_ps(int x) {return sdk();}
esp_err_t esp_wifi_set_protocol(int x,int y) {return sdk();}
esp_err_t esp_wifi_set_bandwidth(int x,int y) {return sdk();}
esp_err_t esp_wifi_ap_get_sta_list(wifi_sta_list_t *s) {s->num=1;return sdk();}
esp_err_t esp_event_handler_register(int x,int y,fake_event_handler_t f,void *c) {esp_err_t e=sdk();if(e==ESP_OK)eventHandler=f;return e;}
esp_err_t esp_event_handler_unregister(int x,int y,fake_event_handler_t f) {return sdk();}
int fake_socket(int a,int b,int c) {static int fd=10;return sdk()==ESP_OK?++fd:-1;}
int fake_setsockopt(int a,int b,int c,const void *d,socklen_t e) {return sdk()==ESP_OK?0:-1;}
int fake_bind(int a,const struct sockaddr *b,socklen_t c) {return sdk()==ESP_OK?0:-1;}
int fake_close(int a) {assert(a>=0);return sdk()==ESP_OK?0:-1;}
ssize_t fake_sendto(int fd,const void *data,size_t size,int flags,const struct sockaddr *address,socklen_t alen)
{
    assert(fd>=0 && size<=sizeof(copiedBytes) && flags==MSG_DONTWAIT);
    esp_err_t error=sdk();
    /* Copy after the blocking call: the owner must still retain exact bytes. */
    memcpy(copiedBytes,data,size);copiedLength=size;copiedTarget=*(const struct sockaddr_in *)address;
    return error!=ESP_OK?-1:sendResult==INT_MAX?(int)size:sendResult;
}
ssize_t fake_recvfrom(int a,void *b,size_t c,int d,struct sockaddr *e,socklen_t *f) {assert(!"RX task deliberately not run by this harness");return -1;}
static void resultCallback(const sensorarrayWifiTxResult_t *r,void *c) {receivedResults++;lastResult=*r;}
static void wakeCallback(void) {wakeCalls++;}
static sensorarrayWifiConfig_t config(void) {return (sensorarrayWifiConfig_t){.ssid="test",.profile=SENSORARRAY_WIFI_PROFILE_HIGH,.dataPort=3333,.logPort=3334,.ctrlPort=3335};}
static sensorarrayWifiOwnerSnapshot_t snapshot(void) {sensorarrayWifiOwnerSnapshot_t s;sensorarrayWifiCopyOwnerSnapshot(&s);return s;}
static void setup(void) {sensorarrayWifiConfigureBoot(UINT64_C(0x123456789abcdef0),44);sensorarrayWifiAllowOwnerStart();sensorarrayWifiSetOwnerCallbacks(resultCallback,NULL,wakeCallback);}
static uint32_t request(void) {uint32_t id; sensorarrayWifiConfig_t c=config();assert(sensorarrayWifiRequestAp(&c,&id)==ESP_OK && id);return id;}
static void step(void) {currentTask=&ownerToken;assert(sensorarrayWifiOwnerProcessOnce());currentTask=NULL;}
static void ready(void) {setup();uint32_t id=request();assert(sdkCalls==0);step();sensorarrayWifiRequestResult_t r;assert(sensorarrayWifiCopyRequestResult(id,&r)&&r.outcome==SENSORARRAY_WIFI_REQUEST_APPLIED);sensorarrayWifiOwnerNoteStationEvent(true);assert(!sdkOwnerViolation);}
static esp_err_t submit(sensorarrayWifiChannel_t channel,bool lifecycle,const void *data,size_t length,uint64_t deadline)
{
    uint32_t epoch;assert(sensorarrayWifiGetSendEpoch(channel==SENSORARRAY_WIFI_CH_CTRL,&epoch));
    sensorarrayWifiPublication_t p={.channel=channel,.lifecycle=lifecycle,.data=data,.length=length,.expectedEpoch=epoch,.deadlineUs=deadline};
    return sensorarrayWifiSubmit(&p);
}
static void *runThread(void *unused) {step();return NULL;}
static pthread_t blocked(void)
{
    pthread_t thread;assert(pthread_create(&thread,NULL,runThread,NULL)==0);
    pthread_mutex_lock(&blockMux);while(!entered)pthread_cond_wait(&blockCond,&blockMux);pthread_mutex_unlock(&blockMux);return thread;
}
static void unblock(pthread_t thread) {pthread_mutex_lock(&blockMux);releaseCall=true;pthread_cond_broadcast(&blockCond);pthread_mutex_unlock(&blockMux);assert(pthread_join(thread,NULL)==0);}
static void observeAfter(uint64_t us) {atomic_fetch_add(&clockUs,us);assert(timerArgs.callback);timerArgs.callback(timerArgs.arg);}
int main(int argc,char **argv)
{
    assert(argc>=2);const char *test=argv[1];
    if(!strcmp(test,"off")) {setup();uint32_t id;assert(sensorarrayWifiRequestOff(&id)==ESP_OK);assert(!allocations&&!taskCreates&&!sdkCalls&&!timerCreates);assert(!snapshot().outputEnabled);}
    else if(!strcmp(test,"oom")||!strcmp(test,"timer_create")||!strcmp(test,"timer_start")||!strcmp(test,"task_create")) {
        setup();failAlloc=!strcmp(test,"oom");failTimerCreate=!strcmp(test,"timer_create");failTimerStart=!strcmp(test,"timer_start");failTask=!strcmp(test,"task_create");sensorarrayWifiConfig_t c=config();uint32_t id=99;
        assert(sensorarrayWifiRequestAp(&c,&id)==ESP_ERR_NO_MEM && id==0);unsigned a=allocations,t=taskCreates,n=timerCreates;
        assert(sensorarrayWifiRequestAp(&c,&id)==ESP_ERR_NO_MEM);assert(a==allocations&&t==taskCreates&&n==timerCreates&&!sdkCalls);
        assert(snapshot().resourceAttempted&&!snapshot().resourceReady&&snapshot().resourceError==ESP_ERR_NO_MEM);
    } else if(!strcmp(test,"success")) {
        ready();assert(allocations==1&&bytesSeen==7424&&capsSeen==(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));assert(taskCreates==2&&timerCreates==1&&timerStarts==1&&timerArgs.skip_unhandled_events);
        unsigned n=sdkCalls;uint32_t id=request();sensorarrayWifiRequestResult_t r;assert(sensorarrayWifiCopyRequestResult(id,&r)&&r.outcome==SENSORARRAY_WIFI_REQUEST_APPLIED);assert(sdkCalls==n&&allocations==1&&taskCreates==2);
        uint8_t data[2304];memset(data,0xa5,sizeof(data));assert(submit(0,false,data,sizeof(data),0)==ESP_OK);memset(data,0,sizeof(data));step();assert(copiedLength==2304&&copiedBytes[2303]==0xa5&&lastResult.fullLength&&receivedResults==1);
        assert(snapshot().heldSlots==0&&snapshot().completionKnown);
    } else if(!strcmp(test,"blocked_send")||!strcmp(test,"late_send")||!strcmp(test,"off_inflight")) {
        ready();uint8_t data[2304];memset(data,0x5a,sizeof(data));assert(submit(0,false,data,sizeof(data),0)==ESP_OK);blockPhase=SENSORARRAY_WIFI_PHASE_SEND;pthread_t t=blocked();assert(snapshot().heldSlots==1&&snapshot().sdkActive);
        if(!strcmp(test,"off_inflight")){uint32_t id;assert(sensorarrayWifiRequestOff(&id)==ESP_OK);assert(snapshot().sdkActive);sensorarrayWifiConfig_t c=config();assert(sensorarrayWifiRequestAp(&c,&id)==ESP_ERR_INVALID_STATE);}
        if(strcmp(test,"late_send")){observeAfter(499999);assert(!snapshot().quarantined);observeAfter(1);assert(snapshot().quarantined&&!snapshot().completionKnown&&snapshot().heldSlots==1);}
        else {atomic_fetch_add(&clockUs,500000);}
        unsigned n=sdkCalls;unblock(t);assert(snapshot().quarantined&&snapshot().completionKnown&&!snapshot().sdkActive&&snapshot().heldSlots==0&&snapshot().lateReturns==1);
        assert(lastResult.fullLength&&lastResult.late&&copiedBytes[2303]==0x5a&&snapshot().sdkError==ESP_OK);
        sensorarrayWifiOwnerNoteStationEvent(true);uint32_t id;assert(sensorarrayWifiRequestOff(&id)==ESP_OK);sensorarrayWifiConfig_t c=config();assert(sensorarrayWifiRequestAp(&c,&id)==ESP_ERR_INVALID_STATE);assert(sdkCalls==n);
    } else if(!strcmp(test,"partial")) {ready();sendResult=2;assert(submit(0,false,"1234",4,0)==ESP_OK);step();sensorarrayWifiStats_t stats;sensorarrayWifiGetStats(&stats);assert(stats.partial[0]==1&&stats.driverAcceptedBytes[0]==2&&stats.sent[0]==0&&!lastResult.fullLength);}
    else if(!strcmp(test,"queue_epoch")) {ready();assert(submit(0,false,"old",3,0)==ESP_OK);sensorarrayWifiOwnerNoteStationEvent(false);sensorarrayWifiOwnerNoteStationEvent(true);unsigned n=sdkCalls;step();assert(sdkCalls==n&&!lastResult.fullLength);sensorarrayWifiStats_t stats;sensorarrayWifiGetStats(&stats);assert(stats.stale[0]==1);}
    else if(!strcmp(test,"priority")) {ready();assert(submit(0,false,"data",4,0)==ESP_OK);assert(submit(1,true,"life",4,0)==ESP_OK);assert(submit(2,false,"ctrl",4,0)==ESP_OK);step();assert(lastResult.channel==2);step();assert(lastResult.lifecycle);step();assert(lastResult.channel==0);}
    else if(!strcmp(test,"expiry")) {ready();assert(submit(0,false,"test",4,1200)==ESP_OK);atomic_store(&clockUs,1200);unsigned n=sdkCalls;step();assert(sdkCalls==n);sensorarrayWifiStats_t stats;sensorarrayWifiGetStats(&stats);assert(stats.expired[0]==1);}
    else if(!strcmp(test,"terminal")) {setup();uint32_t id;assert(sensorarrayWifiRequestOff(&id)==ESP_OK);sensorarrayWifiRequestResult_t r;assert(sensorarrayWifiCopyPendingTerminal(&r)&&r.id==id);for(unsigned i=0;i<3;i++){sensorarrayWifiNoteTerminalAttempt(id,false);atomic_fetch_add(&clockUs,200000);}assert(!sensorarrayWifiCopyPendingTerminal(&r));assert(sensorarrayWifiCopyRequestResult(id,&r)&&r.terminalAttempts==3&&!r.terminalQueued);uint32_t old=id;sensorarrayWifiRequestOff(&id);sensorarrayWifiRequestOff(&id);assert(!sensorarrayWifiCopyRequestResult(old,&r));assert(snapshot().terminalEvicted==1);}
    else if(!strcmp(test,"request_wrap")) {setup();s_owner.nextRequestId=UINT32_MAX;uint32_t id;sensorarrayWifiRequestOff(&id);assert(id==1);sensorarrayWifiRequestResult_t r;assert(!sensorarrayWifiCopyRequestResult(UINT32_MAX,&r));}
    else if(!strcmp(test,"init_deadline")) {setup();uint32_t id=request();blockPhase=SENSORARRAY_WIFI_PHASE_DRIVER_INIT;pthread_t t=blocked();observeAfter(5000000);sensorarrayWifiRequestResult_t r;assert(sensorarrayWifiCopyRequestResult(id,&r)&&r.outcome==SENSORARRAY_WIFI_REQUEST_QUARANTINED);unsigned n=sdkCalls;unblock(t);assert(snapshot().quarantined&&sdkCalls==n&&s_wifi.driverInitialized);assert(sensorarrayWifiCopyRequestResult(id,&r)&&r.outcome==SENSORARRAY_WIFI_REQUEST_QUARANTINED);}
    else if(!strcmp(test,"failed_cleanup")) {setup();request();failPhase=SENSORARRAY_WIFI_PHASE_DEFAULT_HANDLERS;step();assert(!snapshot().quarantined&&!s_wifi.apNetif);failPhase=SENSORARRAY_WIFI_PHASE_CLEAR_HANDLERS;s_wifi.apNetif=(esp_netif_t*)&ownerToken;s_wifi.attachAttempted=true;request();step();assert(snapshot().quarantined&&snapshot().cleanupError==ESP_ERR_NO_MEM&&s_wifi.apNetif);}
    else if(!strcmp(test,"profile_error")) {ready();sensorarrayWifiConfig_t c=config();c.profile=SENSORARRAY_WIFI_PROFILE_BLE_COMPAT;failPhase=SENSORARRAY_WIFI_PHASE_SET_BANDWIDTH;uint32_t id;assert(sensorarrayWifiRequestAp(&c,&id)==ESP_OK);step();assert(snapshot().quarantined);}
    else if(!strcmp(test,"reply_off")) {ready();uint32_t id;sensorarrayWifiRequestOff(&id);uint32_t epoch;assert(sensorarrayWifiGetSendEpoch(true,&epoch)&&!sensorarrayWifiGetSendEpoch(false,NULL));sensorarrayWifiPublication_t p={.channel=2,.unicast=true,.peer={.ipv4=123,.port=42},.data=(uint8_t*)"reply",.length=5,.expectedEpoch=epoch};assert(sensorarrayWifiSubmit(&p)==ESP_OK);step();assert(copiedTarget.sin_addr.s_addr==123&&ntohs(copiedTarget.sin_port)==42&&lastResult.fullLength);}
    else if(!strcmp(test,"callback_event")) {ready();unsigned n=sdkCalls;assert(eventHandler);eventHandler(NULL,WIFI_EVENT,WIFI_EVENT_AP_STADISCONNECTED,NULL);assert(sdkCalls==n&&!snapshot().stationKnown);step();assert(sdkCalls==n+1&&snapshot().stationKnown);}
    else if(!strcmp(test,"phase_failure")) {setup();failPhase=(sensorarrayWifiPhase_t)atoi(argv[2]);uint32_t id=request();step();sensorarrayWifiRequestResult_t r;assert(sensorarrayWifiCopyRequestResult(id,&r)&&r.outcome==SENSORARRAY_WIFI_REQUEST_FAILED);assert(r.originalError!=ESP_OK&&!snapshot().quarantined&&!s_wifi.apNetif&&!s_wifi.ctrlTask);assert(!sdkOwnerViolation);}
    else {assert(!"unknown case");}
    assert(!sdkOwnerViolation);printf("wifi_owner %s PASS\n",test);free(s_owner.payload);return 0;
}
