/* Actual observer/formatter, synthetic SDK/heap and pthread core contention.
 * Direct state setup below is host-only; firmware has no reset/injection API. */
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorarrayMemoryDiagnostics.c"
_Thread_local unsigned lockDepth;
static _Thread_local bool insideCallback;
static esp_alloc_failed_hook_t registeredHook;
static unsigned registerCalls, heapCalls, transportCalls, clockCalls;
static esp_err_t registerResult;
static bool reenterRegistration, failureDuringRegistration;
static uint8_t elf[32];
void heap_caps_get_info(multi_heap_info_t *out,unsigned caps) { (void)caps;assert(!insideCallback && !lockDepth);heapCalls++;memset(out,0,sizeof *out); }
void sensorarrayTransportGetMemoryDiagnostics(sensorarrayTransportMemoryDiagnostics_t *out) { assert(!insideCallback && !lockDepth);transportCalls++;memset(out,0,sizeof *out); }
int64_t esp_timer_get_time(void) { assert(!insideCallback && !lockDepth);clockCalls++;return 0; }
static void observe(size_t size,uint32_t caps,uintptr_t name) {
    assert(registeredHook && !insideCallback);insideCallback=true;
    registeredHook(size,caps,(const char *)name);insideCallback=false;
}
esp_err_t heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t callback) {
    assert(!insideCallback && !lockDepth && callback);registerCalls++;
    if(reenterRegistration)assert(sensorarrayMemoryDiagnosticsInit(2,3,NULL)==ESP_ERR_INVALID_STATE);
    if(registerResult==ESP_OK)registeredHook=callback;
    if(failureDuringRegistration)observe(513,0x804,0x12345678);
    return registerResult;
}
static void resetFixture(void) {
    assert(!lockDepth);memset(&s_allocationFailures,0,sizeof s_allocationFailures);
    __atomic_store_n(&s_allocationCaptureIncomplete,0,__ATOMIC_RELAXED);
    registeredHook=NULL;registerCalls=heapCalls=transportCalls=clockCalls=0;
    registerResult=ESP_OK;reenterRegistration=failureDuringRegistration=false;
    memset(elf,0xab,sizeof elf);
}
static void format(char text[512]) { assert(sensorarrayMemoryDiagnosticsFailureFormat(text,512)==ESP_OK); }
static void testUnknownAndRegistration(void) {
    resetFixture();char text[512];
    assert(sensorarrayMemoryDiagnosticsFailureFormat(NULL,512)==ESP_ERR_INVALID_ARG);
    assert(sensorarrayMemoryDiagnosticsFailureFormat(text,0)==ESP_ERR_INVALID_ARG);
    format(text);assert(strstr(text,"identityKnown=0,elfSha256=unknown,attempted=0,regKnown=0,registered=0,regErr=unknown,known=0,count=0,"));
    assert(strstr(text,"firstSize=unknown,firstCaps=unknown,firstFn=unknown,lastSize=unknown,lastCaps=unknown,lastFn=unknown,"));
    assert(!registerCalls && !heapCalls && !clockCalls && !transportCalls);
    registerResult=ESP_FAIL;assert(sensorarrayMemoryDiagnosticsInit(1,2,elf)==ESP_FAIL);
    assert(!registeredHook && registerCalls==1);format(text);
    assert(strstr(text,"attempted=1,regKnown=1,registered=0,regErr=0xFFFFFFFF,known=0,"));
    registerResult=ESP_OK;assert(sensorarrayMemoryDiagnosticsInit(99,99,NULL)==ESP_FAIL);
    assert(registerCalls==1);format(text);assert(strstr(text,"session=0000000000000001,bootId=2,"));
    resetFixture();reenterRegistration=failureDuringRegistration=true;
    assert(sensorarrayMemoryDiagnosticsInit(UINT64_MAX,UINT32_MAX,elf)==ESP_OK);
    format(text);assert(strstr(text,"registered=1,regErr=0x0,known=1,count=1,"));
    assert(strstr(text,"firstSize=513,firstCaps=0x804,firstFn=0x12345678,"));
    assert(sensorarrayMemoryDiagnosticsInit(99,99,NULL)==ESP_OK && registerCalls==1);
    assert(s_allocationFailures.first.requestedSize==513 && s_allocationFailures.sessionId==UINT64_MAX);
}
static void testFirstLastAndLimits(void) {
    resetFixture();assert(sensorarrayMemoryDiagnosticsInit(UINT64_MAX,UINT32_MAX,elf)==ESP_OK);
    observe(1600,0x80c,1);observe(SIZE_MAX,UINT32_MAX,UINTPTR_MAX);
    char text[520];format(text);
    assert(strstr(text,"count=2,saturated=0,incomplete=0,firstSize=1600,firstCaps=0x80C,firstFn=0x1,"));
    assert(s_allocationFailures.first.functionName==1); /* Invalid pointer never dereferenced. */
    observe(0,0,0);format(text);
    assert(strstr(text,"lastSize=0,lastCaps=0x0,lastFn=unknown,"));
    s_allocationFailures.count=UINT32_MAX-1u;
    observe(SIZE_MAX,UINT32_MAX,UINTPTR_MAX);assert(!s_allocationFailures.saturated);
    observe(SIZE_MAX,UINT32_MAX,UINTPTR_MAX);assert(s_allocationFailures.saturated);
    assert(s_allocationFailures.count==UINT32_MAX && s_allocationFailures.first.requestedSize==1600);
    s_allocationFailures.first=s_allocationFailures.last;
    s_allocationFailures.registrationError=(esp_err_t)UINT32_MAX;
    __atomic_store_n(&s_allocationCaptureIncomplete,1,__ATOMIC_RELAXED);
    format(text);size_t needed=strlen(text)+1u;
    assert(needed<=512);printf("MEMFAIL_MAX,bytes=%zu,capacity=512,stateBytes=%zu\n",needed-1,sizeof s_allocationFailures);
    sensorarrayAllocationFailures_t held=s_allocationFailures;
    for(size_t cap=1;cap<needed;cap++) {
        memset(text,0x5a,sizeof text);
        assert(sensorarrayMemoryDiagnosticsFailureFormat(text,cap)==ESP_ERR_INVALID_SIZE);
        assert(!text[0] && text[cap]==0x5a && !lockDepth);
    }
    assert(sensorarrayMemoryDiagnosticsFailureFormat(text,needed)==ESP_OK);
    assert(!memcmp(&held,&s_allocationFailures,sizeof held));
    assert(!heapCalls && !transportCalls && !clockCalls && registerCalls==1);
}
static void *contendedCall(void *arg) {
    char text[512];(void)arg;
    assert(sensorarrayMemoryDiagnosticsFailureFormat(text,sizeof text)==ESP_ERR_TIMEOUT && !text[0]);
    observe(222,0x808,2);return NULL;
}
static void testNonwaitingContention(void) {
    resetFixture();assert(sensorarrayMemoryDiagnosticsInit(1,2,elf)==ESP_OK);
    observe(111,0x804,1);
    portENTER_CRITICAL(&s_allocationFailureMux);
    pthread_t t;assert(!pthread_create(&t,NULL,contendedCall,NULL));
    /* Join while owner deliberately retains the lock: a waiting hook hangs
     * and is killed by the runner's deadline; a correct no-wait hook returns. */
    assert(!pthread_join(t,NULL));portEXIT_CRITICAL_ISR(&s_allocationFailureMux);
    char text[512];format(text);
    assert(strstr(text,"count=1,saturated=0,incomplete=1,"));
    assert(s_allocationFailures.first.requestedSize==111 && s_allocationFailures.last.requestedSize==111);
    observe(333,0x80c,3);format(text);
    assert(strstr(text,"count=2,saturated=0,incomplete=1,"));
    assert(s_allocationFailures.first.requestedSize==111 && s_allocationFailures.last.requestedSize==333);
}
static atomic_uint finishedWriters;
static void *writer(void *arg) {
    size_t base=(size_t)(uintptr_t)arg*10000;
    for(size_t i=1;i<=4000;i++) {size_t n=base+i;observe(n,(uint32_t)n^0xa5a5u,0x100000u+n);}
    atomic_fetch_add_explicit(&finishedWriters,1,memory_order_release);return NULL;
}
static unsigned long long number(const char *text,const char *key,int base) {
    const char *p=strstr(text,key);assert(p);return strtoull(p+strlen(key),NULL,base);
}
static void testConcurrentRecords(void) {
    resetFixture();assert(sensorarrayMemoryDiagnosticsInit(1,2,elf)==ESP_OK);
    observe(7,7^0xa5a5,0x100007);atomic_store(&finishedWriters,0);
    pthread_t t[4];for(uintptr_t i=0;i<4;i++)assert(!pthread_create(&t[i],NULL,writer,(void *)i));
    unsigned reads=0;
    do {
        char text[512];esp_err_t err=sensorarrayMemoryDiagnosticsFailureFormat(text,sizeof text);
        if(err==ESP_ERR_TIMEOUT)continue;
        assert(err==ESP_OK);reads++;
        assert(strstr(text,"firstSize=7,firstCaps=0xA5A2,firstFn=0x100007,"));
        unsigned long long n=number(text,"lastSize=",10);
        assert(number(text,"lastCaps=",0)==(n^0xa5a5u));
        assert(number(text,"lastFn=",0)==0x100000u+n);
    } while(atomic_load_explicit(&finishedWriters,memory_order_acquire)<4);
    for(unsigned i=0;i<4;i++)assert(!pthread_join(t[i],NULL));
    assert(s_allocationFailures.count>1 && s_allocationFailures.count<=16001);
    if(s_allocationFailures.count<16001)assert(__atomic_load_n(&s_allocationCaptureIncomplete,__ATOMIC_RELAXED));
    assert(s_allocationFailures.first.requestedSize==7);
    assert(!heapCalls && !transportCalls && !clockCalls);
    printf("MEMFAIL_CONCURRENT,writers=4,attempts=16000,recorded=%u,reads=%u,incomplete=%u,first_preserved=1,torn=0\n",
        s_allocationFailures.count,reads,s_allocationCaptureIncomplete);
}
int main(void) {
    testUnknownAndRegistration();testFirstLastAndLimits();testNonwaitingContention();testConcurrentRecords();
    puts("MEMFAIL_PASS,actual_callback=1,groups=4,allocation_policy_unchanged=1,opaque_pointer=1,register_sticky=1,query_readonly=1,hardware=NOT_RUN");return 0;
}
