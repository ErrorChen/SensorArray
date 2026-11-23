#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayBleOutputSchedule.h"

static sensorarrayBleOutputSchedule_t state;
static sensorarrayBleOutputObservation_t observation;
static uint64_t nowUs;
static void reset(void)
{
    nowUs = 0u;
    memset(&observation, 0, sizeof observation);
    observation.epoch = (sensorarrayBleOutputEpoch_t){.sessionId=42u,.bootId=7u,
        .connectionGeneration=3u,.streamGeneration=1u};
    observation.active = observation.valid = true;
    sensorarrayBleOutputScheduleInit(&state, nowUs);
    assert(!sensorarrayBleOutputScheduleObserve(&state, nowUs, &observation));
}
static bool window(uint32_t attempts, uint32_t rejected, uint32_t other, uint32_t completed, uint64_t elapsed)
{
    nowUs += elapsed;
    observation.counters.admissionAttempts += attempts;
    observation.counters.capacityRejected += rejected;
    observation.counters.otherFailures += other;
    observation.counters.wholeCompleted += completed;
    return sensorarrayBleOutputScheduleObserve(&state, nowUs, &observation);
}
static void pressure(unsigned windows)
{
    for (unsigned i=0u;i<windows;i++) assert(window(100u,30u,0u,70u,1000000u));
}
static sensorarrayBleOutputTicket_t decide(uint32_t sequence, uint8_t flags)
{
    sensorarrayBleOutputFrameKey_t key={.revision=9u,.sequence=sequence};
    return sensorarrayBleOutputScheduleDecide(&state, nowUs, &observation.epoch, &key, flags);
}
static void fullAndThresholds(void)
{
    reset();
    for (uint32_t i=0u;i<10000u;i++) assert(decide(i,0u).decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY);
    assert(state.snapshot.policySkipped==0u && state.snapshot.selectedOrdinary==10000u);
    const uint32_t rejected[]={0u,1u,10u,20u};
    for(unsigned c=0u;c<4u;c++) {
        reset();
        for(unsigned i=0u;i<30u;i++) assert(window(100u,rejected[c],0u,100u-rejected[c],1000000u));
        assert(state.snapshot.ordinaryRatePermille==1000u && state.snapshot.downSteps==0u);
        assert(state.snapshot.mode==(rejected[c]>=10u?SENSORARRAY_BLE_OUTPUT_PRESSURE:SENSORARRAY_BLE_OUTPUT_FULL));
    }
    reset(); assert(window(1000u,201u,0u,799u,1000000u));
    assert(state.snapshot.pressureWindows==1u); /* Strictly >20%, despite rounded display=200. */
    assert(window(100u,20u,0u,80u,1000000u)); assert(state.snapshot.pressureWindows==0u);
    pressure(2u); assert(state.snapshot.ordinaryRatePermille==1000u);
    assert(window(100u,0u,0u,100u,1000000u)); pressure(2u);
    assert(state.snapshot.ordinaryRatePermille==1000u);
}
static void stepsAndRecovery(void)
{
    reset(); pressure(3u);
    assert(state.snapshot.ordinaryRatePermille==950u && state.snapshot.downSteps==1u && state.snapshot.lastAdjustmentUs==3000000u);
    pressure(2u); assert(state.snapshot.ordinaryRatePermille==950u);
    pressure(1u); assert(state.snapshot.ordinaryRatePermille==900u && state.snapshot.lastAdjustmentUs==6000000u);
    for(unsigned i=0u;i<9u;i++) assert(window(100u,0u,0u,100u,1000000u));
    assert(state.snapshot.ordinaryRatePermille==900u && state.snapshot.cleanUs==9000000u);
    assert(window(100u,0u,1u,99u,1000000u)); assert(state.snapshot.cleanUs==0u);
    for(unsigned i=0u;i<10u;i++) assert(window(100u,0u,0u,100u,1000000u));
    assert(state.snapshot.ordinaryRatePermille==950u && state.snapshot.upSteps==1u);
    for(unsigned i=0u;i<10u;i++) assert(window(100u,0u,0u,100u,1000000u));
    assert(state.snapshot.ordinaryRatePermille==1000u && state.snapshot.mode==SENSORARRAY_BLE_OUTPUT_FULL);
    reset(); pressure(60u); assert(state.snapshot.ordinaryRatePermille==0u && state.snapshot.downSteps==20u);
    pressure(6u); assert(state.snapshot.ordinaryRatePermille==0u && state.snapshot.downSteps==20u);
    assert(decide(0u,0u).decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY); /* Bounded recovery probe. */
    assert(decide(1u,0u).decision==SENSORARRAY_BLE_OUTPUT_SKIP_POLICY);
    assert(decide(2u,SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC).decision==SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED);
}
static void fractionalSelection(void)
{
    reset(); pressure(3u);
    unsigned sent=0u, skipped=0u;
    for(uint32_t i=0u;i<200u;i++) {
        nowUs+=10000u; /* 100Hz input, 95Hz target, never an 11ms/50Hz alias. */
        sensorarrayBleOutputTicket_t t=decide(i,0u);
        if(t.decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY)sent++;
        else {assert(t.decision==SENSORARRAY_BLE_OUTPUT_SKIP_POLICY && t.ticketId==0u);skipped++;}
    }
    assert(sent==190u && skipped==10u && state.snapshot.policySkipped==10u);
    assert(state.snapshot.reportedCompleted==0u && state.snapshot.reportedFailed==0u);
    assert(state.snapshot.eligibleFrames==state.snapshot.selectedOrdinary+state.snapshot.selectedProtected+state.snapshot.policySkipped);
    uint16_t credit=state.ordinaryCredit;
    for(unsigned i=0u;i<100u;i++) assert(decide(i,3u).decision==SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED);
    assert(state.ordinaryCredit==credit && state.snapshot.selectedProtected==100u);
}
static void missingAndAnomalousEvidence(void)
{
    reset(); pressure(2u); assert(!window(900u,300u,0u,600u,9000000u));
    assert(state.snapshot.ordinaryRatePermille==1000u && state.snapshot.pressureWindows==0u);
    pressure(2u); observation.valid=false;
    assert(!sensorarrayBleOutputScheduleObserve(&state,++nowUs,&observation));
    observation.valid=true; assert(!sensorarrayBleOutputScheduleObserve(&state,++nowUs,&observation));
    pressure(2u); assert(state.snapshot.ordinaryRatePermille==1000u);
    assert(!window(0u,0u,0u,0u,1000000u)); assert(state.snapshot.noEvidenceWindows==1u);
    assert(!window(1u,2u,0u,0u,1000000u)); assert(!state.snapshot.evidenceKnown);
    pressure(3u); assert(state.snapshot.ordinaryRatePermille==950u);
    for(unsigned i=0u;i<20u;i++) assert(window(100u,0u,0u,0u,1000000u));
    assert(state.snapshot.ordinaryRatePermille==950u && state.snapshot.cleanUs==0u); /* No completion evidence. */
    observation.counters=(sensorarrayBleOutputCounters_t){0}; nowUs+=1000000u;
    assert(!sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation));
    assert(state.snapshot.ordinaryRatePermille==950u && state.snapshot.pressureWindows==0u);
    assert(!sensorarrayBleOutputScheduleObserve(&state,nowUs,NULL));
}
static void wrapAndTime(void)
{
    reset(); observation.counters=(sensorarrayBleOutputCounters_t){UINT32_MAX-20u,UINT32_MAX-10u,UINT32_MAX,UINT32_MAX-20u};
    sensorarrayBleOutputScheduleInit(&state,0u);
    assert(!sensorarrayBleOutputScheduleObserve(&state,0u,&observation));
    pressure(3u); assert(state.snapshot.ordinaryRatePermille==950u && state.snapshot.validWindows==3u);
    sensorarrayBleOutputTicket_t old=decide(UINT32_MAX,3u);
    uint32_t generation=state.snapshot.policyGeneration;
    nowUs=2u;
    assert(!sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation));
    assert(state.snapshot.timeResets==1u && state.snapshot.policyGeneration!=generation && state.snapshot.ordinaryRatePermille==950u);
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&old,&old,true)==0u);
    nowUs=UINT64_MAX; assert(!sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation));
    nowUs=0u; assert(!sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation));
    assert(state.snapshot.timeResets==2u && state.snapshot.ordinaryRatePermille==950u);
    state.nextTicketId=UINT64_MAX;
    sensorarrayBleOutputTicket_t t=decide(0u,1u); assert(t.ticketId==1u);
    assert(decide(1u,0u).decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY);
    state.snapshot.policySkipped=UINT64_MAX; state.snapshot.ordinaryRatePermille=0u;
    assert(decide(1u,0u).decision==SENSORARRAY_BLE_OUTPUT_SKIP_POLICY && state.snapshot.policySkipped==0u);
}
static void epochAndInactive(void)
{
    reset(); pressure(3u); sensorarrayBleOutputTicket_t old=decide(1u,1u);
    observation.epoch.streamGeneration++;
    assert(!sensorarrayBleOutputScheduleObserve(&state,++nowUs,&observation));
    assert(state.snapshot.ordinaryRatePermille==1000u && !state.snapshot.evidenceKnown);
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&old,&old,true)==0u);
    sensorarrayBleOutputTicket_t current=decide(2u,1u);
    observation.active=false; assert(!sensorarrayBleOutputScheduleObserve(&state,++nowUs,&observation));
    assert(decide(3u,1u).decision==SENSORARRAY_BLE_OUTPUT_NO_STREAM);
    observation.active=true; assert(!sensorarrayBleOutputScheduleObserve(&state,++nowUs,&observation));
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&current,&current,true)==0u);
    const sensorarrayBleOutputEpoch_t previous=observation.epoch;
    observation.epoch.sessionId++; observation.epoch.bootId=0u; observation.epoch.connectionGeneration=0u;
    observation.counters=(sensorarrayBleOutputCounters_t){0};
    assert(!sensorarrayBleOutputScheduleObserve(&state,++nowUs,&observation));
    sensorarrayBleOutputFrameKey_t frame={0};
    assert(sensorarrayBleOutputScheduleDecide(&state,nowUs,&previous,&frame,3u).decision==SENSORARRAY_BLE_OUTPUT_NO_STREAM);
    assert(state.snapshot.downSteps==1u); /* Boot totals do not reset with link epoch. */
}
static void protectionCompletion(void)
{
    reset(); pressure(60u);
    sensorarrayBleOutputTicket_t first=decide(1u,3u), pending=first;
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&first,&pending,false)==0u);
    assert(pending.ticketId==first.ticketId && pending.protectionFlags==3u); /* No ownership/obligation mutation. */
    sensorarrayBleOutputTicket_t retry=decide(2u,3u); pending=retry;
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&first,&pending,true)==0u);
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&retry,&pending,true)==3u);
    assert(pending.protectionFlags==3u && state.snapshot.protectedAckCandidates==1u);
    sensorarrayBleOutputTicket_t different=pending; different.frame.revision++;
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&retry,&different,true)==0u);
    different=pending; different.frame.sequence++;
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&retry,&different,true)==0u);
    assert(decide(3u,0u).decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY);
    sensorarrayBleOutputTicket_t skipped=decide(4u,0u);
    assert(sensorarrayBleOutputScheduleNoteCompletion(&state,nowUs,&skipped,&pending,true)==0u);
    assert(state.snapshot.reportedFailed==1u && state.snapshot.ignoredCompletions==1u);
    sensorarrayBleOutputScheduleSnapshot_t before,after;
    sensorarrayBleOutputScheduleCopySnapshot(&state,&before);
    sensorarrayBleOutputScheduleCopySnapshot(&state,NULL);
    sensorarrayBleOutputScheduleCopySnapshot(&state,&after);
    assert(!memcmp(&before,&after,sizeof before));
}
static void sparseRecovery(void)
{
    /* Regression from the independently reproduced 1.2s protected-completion
     * and 12FPS/5% scenarios. Empty windows remain unknown, never healthy. */
    reset(); pressure(60u);
    uint64_t begin=nowUs, nextOffer=begin+1000000u, nextComplete=0u;
    unsigned offered=0u, complete=0u;
    for(unsigned i=1u;i<=1200u;i++) {
        nowUs=begin+(uint64_t)i*100000u;
        if(nowUs==nextOffer) {
            assert(decide(i,SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC).decision==SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED);
            observation.counters.admissionAttempts++; offered++; nextComplete=nowUs+200000u;
        }
        if(nextComplete && nowUs==nextComplete) {
            observation.counters.wholeCompleted++;complete++;
            nextOffer=nowUs+1000000u;nextComplete=0u;
        }
        (void)sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation);
    }
    assert(offered==100u && complete==100u && state.snapshot.upSteps>0u && state.snapshot.ordinaryRatePermille>0u);
    assert(state.snapshot.noEvidenceWindows>0u && state.snapshot.probeSelected==0u);
    reset(); pressure(57u); begin=nowUs;
    for(unsigned i=1u;i<=1440u;i++) {
        nowUs=begin+(uint64_t)i*83333u;
        sensorarrayBleOutputTicket_t t=decide(i,0u);
        if(t.decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY) {
            observation.counters.admissionAttempts++;observation.counters.wholeCompleted++;
        }
        (void)sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation);
    }
    assert(state.snapshot.upSteps>0u && state.snapshot.ordinaryRatePermille>50u && state.snapshot.probeSelected>0u);
    reset(); pressure(60u); begin=nowUs;
    for(unsigned i=1u;i<=1440u;i++) {
        nowUs=begin+(uint64_t)i*83333u;
        if(decide(i,0u).decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY) {
            observation.counters.admissionAttempts++;observation.counters.capacityRejected++;
        }
        (void)sensorarrayBleOutputScheduleObserve(&state,nowUs,&observation);
    }
    assert(state.snapshot.upSteps==0u && state.snapshot.ordinaryRatePermille==0u);
    assert(state.snapshot.probeSelected>=100u && state.snapshot.probeSelected<=120u);
    reset(); pressure(3u);
    assert(window(100u,0u,0u,100u,1000000u)); assert(state.snapshot.cleanUs==1000000u);
    assert(!window(0u,0u,0u,0u,1900000u));
    assert(window(1u,0u,0u,1u,1000000u)); assert(state.snapshot.cleanUs==2000000u); /* 2.9s evidence gap. */
    assert(!window(0u,0u,0u,0u,2000000u));
    assert(window(1u,0u,0u,1u,1100000u)); assert(state.snapshot.cleanUs==1100000u); /* 3.1s expires old evidence. */
    for(unsigned i=0u;i<120u;i++) assert(!window(0u,0u,0u,0u,1000000u));
    assert(state.snapshot.cleanUs==0u && state.snapshot.upSteps==0u && state.snapshot.ordinaryRatePermille==950u);
}
int main(void)
{
    fullAndThresholds(); stepsAndRecovery(); fractionalSelection(); missingAndAnomalousEvidence();
    wrapAndTime(); epochAndInactive(); protectionCompletion(); sparseRecovery();
    printf("BLE_OUTPUT_SCHEDULE_PASS,groups=8,normal_full=1,pressure_windows=3,step_permille=50,down_hold_us=3000000,recover_evidence_us=10000000,clean_ttl_us=3000000,probe_interval_us=1000000,protection_external=1,peerLoss=unknown,RSSI=unknown,stateBytes=%zu,hardware=NOT_RUN\n",sizeof state);
    return 0;
}
