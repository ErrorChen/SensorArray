#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorarrayTypes.h"
#include "sensorarrayFrame.h"
#include "sensorarrayTextProtocol.h"
#include "sensorarrayUsbSink.h"
#include "sensorarrayNetStatus.h"
#include "sensorarrayTransport.h"
#include "sensorarrayBle.h"

static uint32_t testNetPeriod = 50u;
#define SENSORARRAY_CFG_LOG_PERIOD_FRAMES testNetPeriod
#define CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK 16384u
#define CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR 0x2bu
#define CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR 0x2au
typedef uint8_t StackType_t;
static uint64_t nowUs;
static unsigned variant, frameNumber, formatPackets[2], stackScans[2], oldCellVisits;
static bool adsDebug;
static bool otgEnabled;
static char output[2][16384];
static size_t outputLength[2];
static char netOutput[2][8192], usbOutput[2][8192];
static size_t netLength[2], usbLength[2];
static portMUX_TYPE s_usbProfileMux;
static sensorarrayUsbSinkStats_t s_usbStats;
static sensorarrayUsbStreamProfile_t s_usbStreamProfile;
static bool s_usbInitialized;
static bool sensorarrayUsbOtgEnabled(void) { return otgEnabled; }
/* ACTUAL_USB_PROFILE */

static void record(const char *bytes, size_t length)
{
    assert(outputLength[variant] + length < sizeof(output[variant]));
    memcpy(output[variant] + outputLength[variant], bytes, length);
    outputLength[variant] += length;
}
static int capturedPrintf(const char *fmt, ...)
{
    char bytes[2048]; va_list args; va_start(args,fmt);
    int n=vsnprintf(bytes,sizeof(bytes),fmt,args); va_end(args);
    assert(n>=0 && (size_t)n<sizeof(bytes)); record(bytes,(size_t)n); return n;
}
static int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
static size_t esp_get_free_heap_size(void) { return 8294556u-frameNumber; }
static size_t esp_get_minimum_free_heap_size(void) { return 8290104u; }
static UBaseType_t uxTaskGetStackHighWaterMark(void *task)
{ (void)task;stackScans[variant]++; return 13064u; }
static bool sensorarrayCommandMailboxAdsDebugEnabled(void) { return adsDebug; }
sensorarrayUsbStreamProfile_t sensorarrayUsbSinkGetStreamProfile(void)
{ return variant == 0u ? (sensorarrayUsbStreamProfile_t){.mode=SENSORARRAY_USB_STREAM_DEBUG,.dataEvery=50,.diagEvery=15} : actualUsbProfile(); }
void sensorarrayUsbSinkGetStats(sensorarrayUsbSinkStats_t *s)
{ *s=(sensorarrayUsbSinkStats_t){.sentPackets=frameNumber+5,.sentBytes=frameNumber*598u,.droppedPackets=frameNumber/19u}; }
void sensorarrayNetGetSinkStats(sensorarrayNetSinkStats_t *s)
{ *s=(sensorarrayNetSinkStats_t){.bleSentPackets=frameNumber/2u,.bleDroppedPackets=frameNumber/3u,.bleSentBytes=frameNumber*299u}; }
void sensorarrayTransportGetStats(sensorarrayTransportStats_t *s)
{ *s=(sensorarrayTransportStats_t){.queueDrop=frameNumber/13u,.transportSlotHighWater=4,.transportSlotUsed=2}; }
void sensorarrayTransportGetTaskStackStats(sensorarrayTransportTaskStackStats_t *s)
{ stackScans[variant]+=2;*s=(sensorarrayTransportTaskStackStats_t){.transportConfiguredBytes=6144,.transportMinimumRemainingBytes=3180,.serialCtrlConfiguredBytes=6144,.serialCtrlMinimumRemainingBytes=1896}; }
void sensorarrayBleGetStats(sensorarrayBleStats_t *s)
{ stackScans[variant]+=2;*s=(sensorarrayBleStats_t){.connected=true,.mtu=247,.txPhy=2,.rxPhy=2,.txTaskConfiguredBytes=4096,.txTaskMinimumRemainingBytes=1932,.ctrlTaskConfiguredBytes=6144,.ctrlTaskMinimumRemainingBytes=5236,.messageSent=frameNumber/2u}; }
sensorarrayTransportStream_t sensorarrayTransportGetStream(void) { return SENSORARRAY_TRANSPORT_STREAM_AUTO; }
const char *sensorarrayTransportStreamName(sensorarrayTransportStream_t s) { (void)s;return "auto"; }
sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void) { return SENSORARRAY_TRANSPORT_TX_REL; }
const char *sensorarrayTransportTxModeName(sensorarrayTransportTxMode_t m) { (void)m;return "rel"; }
sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void) { return SENSORARRAY_BLE_TX_FAST; }
const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t m) { (void)m;return "fast"; }
bool sensorarrayBleIsSubscribed(sensorarrayBleChannel_t c) { (void)c;return true; }
const char *sensorarrayMeasurementModeName(sensorarrayMeasurementMode_t m)
{ return m==SENSORARRAY_MEASUREMENT_MODE_VOLTAGE?"VOLT":m==SENSORARRAY_MEASUREMENT_MODE_RESISTANCE?"RES":"CAP"; }
static const char *sensorarrayAdsRailStatusName(sensorarrayAdsRailStatus_t s)
{ (void)s;return "ok"; }
esp_err_t sensorarrayUsbSinkPublishDiagnostic(const sensorarrayTextPacket_t *p)
{ assert(usbLength[variant]+p->length<sizeof(usbOutput[variant]));memcpy(usbOutput[variant]+usbLength[variant],p->data,p->length);usbLength[variant]+=p->length;record("USB:",4);record(p->data,p->length);formatPackets[variant]++;return ESP_OK; }
esp_err_t sensorarrayNetLogPublish(const sensorarrayTextPacket_t *p)
{ assert(netLength[variant]+p->length<sizeof(netOutput[variant]));memcpy(netOutput[variant]+netLength[variant],p->data,p->length);netLength[variant]+=p->length;record("NET:",4);record(p->data,p->length);formatPackets[variant]++;return ESP_OK; }

/* ACTUAL_TYPES */
static void compareState(const sensorarrayAsyncLogSummary_t *a,const sensorarrayAsyncLogSummary_t *b)
{
    /* STATE_COMPARE_FIELDS */
}
static sensorarrayAsyncLogWorkspace_t old_workspace,new_workspace;
static sensorarrayAsyncLogWorkspace_t *old_s_asyncLogWorkspace=&old_workspace,*new_s_asyncLogWorkspace=&new_workspace;
static sensorarrayAsyncLogSharedStats_t old_s_sharedStats,new_s_sharedStats;
static portMUX_TYPE old_s_asyncLogMux,new_s_asyncLogMux;
static uint32_t old_s_initialFreeHeap=8331592,new_s_initialFreeHeap=8331592;
static uint32_t old_s_summaryTruncated,new_s_summaryTruncated;
#define printf capturedPrintf
/* ACTUAL_OLD */
/* ACTUAL_NEW */
#undef printf

int main(void)
{
    unsigned compared=0, outputRecords=0;
    const uint32_t periods[] = {0u,1u,7u,15u,50u,1000u};
    unsigned profileCases=0;
    for(unsigned otg=0;otg<2;otg++) {
        otgEnabled=otg!=0;
        for(unsigned p=0;p<sizeof(periods)/sizeof(periods[0]);p++) {
            testNetPeriod=periods[p];s_usbInitialized=false;
            assert(sensorarrayUsbSinkInit()==ESP_OK);
            sensorarrayUsbStreamProfile_t profile=actualUsbProfile();
            assert(profile.diagEvery==(otg?(testNetPeriod?testNetPeriod:20u):15u));
            assert(profile.dataEvery==50u&&profile.mode==SENSORARRAY_USB_STREAM_DEBUG);
            variant=1u;
            for(uint32_t seq=0;seq<101u;seq++) {
                assert(sensorarrayUsbSinkShouldEmitData(seq)==(otg||seq%50u==0u));
            }
            assert(sensorarrayUsbSinkShouldEmitData(UINT32_MAX)==(otg||UINT32_MAX%50u==0u));
            s_usbStats.sentPackets=123u;
            assert(sensorarrayUsbSinkInit()==ESP_OK&&s_usbStats.sentPackets==123u);
            profileCases++;
        }
    }
    testNetPeriod=50u;
    for(unsigned transport=0;transport<2;transport++) {
    otgEnabled=transport!=0;s_usbInitialized=false;
    assert(sensorarrayUsbSinkInit()==ESP_OK);
    memset(formatPackets,0,sizeof(formatPackets));memset(stackScans,0,sizeof(stackScans));
    for (unsigned scenario=0;scenario<16;scenario++) {
        memset(&old_workspace,0,sizeof(old_workspace));memset(&new_workspace,0,sizeof(new_workspace));
        memset(&old_s_sharedStats,0,sizeof(old_s_sharedStats));memset(&new_s_sharedStats,0,sizeof(new_s_sharedStats));
        old_s_summaryTruncated=new_s_summaryTruncated=0;
        old_sensorarrayAsyncLogPrecisionReset();
        adsDebug=scenario>=8;
        for (unsigned n=1;n<=151;n++) {
            frameNumber=n;nowUs=1000000u+(uint64_t)n*16700u;
            sensorarrayFrame_t frame={0};
            frame.sequence=1000u+n;frame.activeRows=(scenario&1)?8:1;
            frame.frameStartUs=nowUs;frame.emitUs=nowUs+50;
            frame.physicalSweepId=n;frame.physicalSweepUs=11300+n%31;
            frame.freshFrame=n%17!=0;frame.stale=n%17==0;frame.mixedEpoch=n%31==0;
            frame.freshCount=frame.activeRows*8u;frame.validCount=frame.freshCount;
            frame.rowFreshMask=frame.primaryFreshMask=frame.secondaryFreshMask=(1u<<frame.activeRows)-1u;
            unsigned mode=(scenario%8)/2;
            frame.mixedProfile=mode==3;
            frame.measurement.mode=mode==1?SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:mode==2?SENSORARRAY_MEASUREMENT_MODE_RESISTANCE:SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
            frame.capValidMask=(mode==0||mode==3)?UINT64_MAX:0;
            frame.freshMask=frame.capValidMask;
            for(unsigned cell=0;cell<64;cell++) {
                frame.capTotalPf[cell]=(double)(cell+n)*0.013;
                frame.raw28[cell]=cell*5000u+n*13u;
            }
            if(n%7==0) {frame.capTotalPf[2]=NAN;frame.capValidMask&=~UINT64_C(8);}
            frame.measurement.frameDurationUs=11300+n%31;frame.measurement.freshCellCount=frame.freshCount;
            frame.measurement.rawConversionCount=frame.freshCount+2;
            frame.measurement.autorangeAttemptCount=frame.freshCount;
            frame.measurement.precisionFrame=n%16==0;
            frame.adsGap.initialized=true;frame.adsGap.jobsRun=n/3u;
            frame.adsGap.ain9OffsetUv=480+(int)(n%13);frame.adsGap.railUv=5145000;
            frame.adsGap.batteryValid=n%5!=0;frame.adsGap.batteryFresh=n%3!=0;
            frame.adsGap.batteryMv=3700;frame.adsGap.batteryInvalidReason=n%5?SENSORARRAY_BATTERY_INVALID_NONE:SENSORARRAY_BATTERY_INVALID_RAIL;
            frame.adsGap.aincomGndValid=frame.adsGap.ain8GndValid=n%2==0;
            frame.adsGap.aincomGndUv=1123;frame.adsGap.ain8GndUv=2200;
            frame.adsGap.batteryLastGoodAgeMs=n%4?100:UINT32_MAX;
            frame.adsGap.batteryLastGoodFrame=n%4?n:UINT32_MAX;
            sensorarrayFrame_t saved=frame;
            old_s_sharedStats.publishedFrames=new_s_sharedStats.publishedFrames=n;
            old_s_sharedStats.droppedEventLogs=new_s_sharedStats.droppedEventLogs=n/29u;
            old_s_sharedStats.droppedOutputFrames=new_s_sharedStats.droppedOutputFrames=n/41u;
            variant=0;outputLength[0]=0;netLength[0]=usbLength[0]=0;
            old_sensorarrayAsyncLogUpdateSummary(&old_workspace.summary,&frame,12000,80,90,n%3,n%11!=0);
            variant=1;outputLength[1]=0;netLength[1]=usbLength[1]=0;
            new_sensorarrayAsyncLogUpdateSummary(&new_workspace.summary,&frame,12000,80,90,n%3,n%11!=0);
            compareState(&old_workspace.summary,&new_workspace.summary);
            variant=0;old_sensorarrayAsyncLogMaybePrintSummary(&old_workspace.summary);
            variant=1;new_sensorarrayAsyncLogMaybePrintSummary(&new_workspace.summary);
            assert(netLength[0]==netLength[1]);
            assert(memcmp(netOutput[0],netOutput[1],netLength[0])==0);
            if(!otgEnabled) {
                assert(outputLength[0]==outputLength[1]);
                assert(memcmp(output[0],output[1],outputLength[0])==0);
            } else {
                assert(usbLength[1]==netLength[1]);
                assert(memcmp(usbOutput[1],netOutput[1],netLength[1])==0);
            }
            compareState(&old_workspace.summary,&new_workspace.summary);
            assert(memcmp(&frame,&saved,sizeof(frame))==0);
            assert(old_s_summaryTruncated==new_s_summaryTruncated);
            output[0][outputLength[0]]='\0';
            assert(strstr(output[0],"PREC20")==NULL);
            compared++;outputRecords+=outputLength[0]!=0;
        }
    }
    if(!otgEnabled) {
        assert(formatPackets[0]==formatPackets[1] && formatPackets[0]>1000);
        assert(stackScans[0]==stackScans[1]);
    } else {
        assert(formatPackets[1]<formatPackets[0]);
        assert(stackScans[1]<stackScans[0]);
    }
    }
    assert(oldCellVisits==compared*64u);
    printf("USB_DEBUG_CADENCE,profile_cases=%u,transport_scenarios=32,legacy_bytes_equal=1,network_bytes_equal=1,OTG_USB_equals_complete_network_aggregate=1,ADSDBG_preserved=1\n",profileCases);
    printf("ASYNC_SUMMARY,scenarios=32,frames=%u,baseline_output_windows=%u,baseline_packets=%u,state_equal=1,input_unchanged=1,old_cell_visits=%u,new_cell_visits=0,old_window_bytes=%zu,stack_scans_only_when_summary_due=1,scope=host_synthetic_not_timing\n",compared,outputRecords,formatPackets[0],oldCellVisits,sizeof(old_s_precisionWindow));
    return 0;
}
