/* Synthetic local-counter evidence: same two input/event loops as the saved
 * 01 reproducer, with recovery expectations changed for the repaired module.
 * This does not claim hardware/radio loss or peer receipt. */
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include "sensorarrayBleOutputSchedule.h"
static void pressured(sensorarrayBleOutputSchedule_t *s,
                      sensorarrayBleOutputObservation_t *o,
                      uint64_t *now, unsigned windows) {
    *o=(sensorarrayBleOutputObservation_t){
        .epoch={.sessionId=1,.bootId=2,.connectionGeneration=3,.streamGeneration=4},
        .active=true,.valid=true};
    *now=0;
    sensorarrayBleOutputScheduleInit(s,*now);
    (void)sensorarrayBleOutputScheduleObserve(s,*now,o);
    for(unsigned i=0;i<windows;i++) {
        *now+=1000000;
        o->counters.admissionAttempts+=100;
        o->counters.capacityRejected+=30;
        o->counters.wholeCompleted+=70;
        assert(sensorarrayBleOutputScheduleObserve(s,*now,o));
    }
}
static void zeroRateSuccessfulSnapshots(void) {
    sensorarrayBleOutputSchedule_t s;
    sensorarrayBleOutputObservation_t o;
    uint64_t now;
    pressured(&s,&o,&now,60);
    assert(s.snapshot.ordinaryRatePermille==0);
    /* Protected snapshot is due 1s after prior local completion. With 200ms
     * complete-message service time, offers recur every 1.2s. Observer stays
     * active every 100ms; therefore no missing/invalid observation interval. */
    uint64_t begin=now,nextOffer=begin+1000000,nextCompletion=0;
    uint32_t successful=0,offered=0;
    for(unsigned i=1;i<=1200;i++) {
        now=begin+(uint64_t)i*100000;
        if(now==nextOffer) {
            sensorarrayBleOutputFrameKey_t key={.revision=7,.sequence=i};
            sensorarrayBleOutputTicket_t t=sensorarrayBleOutputScheduleDecide(
                &s,now,&o.epoch,&key,SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC);
            assert(t.decision==SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED);
            o.counters.admissionAttempts++;
            offered++;
            nextCompletion=now+200000;
        }
        if(nextCompletion && now==nextCompletion) {
            o.counters.wholeCompleted++;
            successful++;
            nextOffer=now+1000000;
            nextCompletion=0;
        }
        (void)sensorarrayBleOutputScheduleObserve(&s,now,&o);
    }
    assert(successful==100 && offered==100);
    assert(s.snapshot.ordinaryRatePermille>0 && s.snapshot.upSteps>0);
    printf("RECOVERY_CONFIRMED,case=zero_rate_successful_1Hz_due_snapshots,seconds=120,offered=%u,wholeLocalCompleted=%u,capacityRejects=0,otherFailures=0,ratePermille=%u,upSteps=%" PRIu64 ",noEvidenceWindows=%" PRIu64 ",cleanUs=%" PRIu64 "\n",offered,successful,s.snapshot.ordinaryRatePermille,s.snapshot.upSteps,s.snapshot.noEvidenceWindows,s.snapshot.cleanUs);
}
static void fivePercentTwelveFps(void) {
    sensorarrayBleOutputSchedule_t s;
    sensorarrayBleOutputObservation_t o;
    uint64_t now;
    pressured(&s,&o,&now,57);
    assert(s.snapshot.ordinaryRatePermille==50);
    uint64_t begin=now;
    uint32_t sent=0;
    /* About 12fps input, every selected ordinary frame completes locally.
     * Protection counters are intentionally absent: ordinary recovery must
     * not depend on a separate protected stream manufacturing clean evidence. */
    for(unsigned i=1;i<=1440;i++) {
        now=begin+(uint64_t)i*83333;
        sensorarrayBleOutputFrameKey_t key={.revision=8,.sequence=i};
        sensorarrayBleOutputTicket_t t=sensorarrayBleOutputScheduleDecide(
            &s,now,&o.epoch,&key,0);
        if(t.decision==SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY) {
            o.counters.admissionAttempts++;
            o.counters.wholeCompleted++;
            sent++;
        }
        (void)sensorarrayBleOutputScheduleObserve(&s,now,&o);
    }
    assert(sent>72);
    assert(s.snapshot.ordinaryRatePermille>50 && s.snapshot.upSteps>0);
    printf("RECOVERY_CONFIRMED,case=five_percent_12fps,offered=1440,wholeLocalCompleted=%u,capacityRejects=0,otherFailures=0,ratePermille=%u,upSteps=%" PRIu64 ",noEvidenceWindows=%" PRIu64 ",cleanUs=%" PRIu64 "\n",sent,s.snapshot.ordinaryRatePermille,s.snapshot.upSteps,s.snapshot.noEvidenceWindows,s.snapshot.cleanUs);
}
int main(void) {
    zeroRateSuccessfulSnapshots();
    fivePercentTwelveFps();
    return 0;
}
