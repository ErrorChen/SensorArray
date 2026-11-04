#include <assert.h>
#include <stdio.h>
#include "sensorarrayFdcWorkerLifecycle.h"

int main(void)
{
    sensorarrayFdcJobLifecycle_t life = {0};
    assert(sensorarrayFdcJobReleased(&life));
    /* JOB not consumed: cancellation is state, unaffected by merged wakeups. */
    assert(sensorarrayFdcJobPost(&life));
    sensorarrayFdcJobCancel(&life);
    assert(!sensorarrayFdcJobReclaim(&life));
    assert(!sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_POSTED, SENSORARRAY_FDC_JOB_ACKED));
    sensorarrayFdcJobFinish(&life);
    assert(sensorarrayFdcJobPhase(&life) == SENSORARRAY_FDC_JOB_CANCEL_ACKED);
    assert(sensorarrayFdcJobReclaim(&life));
    /* ACK delayed; cancellation wins without entering RUN. */
    assert(sensorarrayFdcJobPost(&life));
    sensorarrayFdcJobCancel(&life);
    assert(!sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_ACKED, SENSORARRAY_FDC_JOB_RUNNING));
    sensorarrayFdcJobFinish(&life);
    assert(sensorarrayFdcJobReclaim(&life));
    /* RUN delayed, and a stale DONE wakeup cannot release or reuse storage. */
    assert(sensorarrayFdcJobPost(&life));
    assert(sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_POSTED, SENSORARRAY_FDC_JOB_ACKED));
    assert(!sensorarrayFdcJobReclaim(&life));
    sensorarrayFdcJobCancel(&life);
    sensorarrayFdcJobFinish(&life);
    assert(sensorarrayFdcJobReclaim(&life));
    /* DONE event lost: persistent release is sufficient evidence. */
    assert(sensorarrayFdcJobPost(&life));
    assert(sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_POSTED, SENSORARRAY_FDC_JOB_ACKED));
    assert(sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_ACKED, SENSORARRAY_FDC_JOB_RUNNING));
    sensorarrayFdcJobFinish(&life);
    sensorarrayFdcJobCancel(&life);
    assert(sensorarrayFdcJobPhase(&life) == SENSORARRAY_FDC_JOB_DONE);
    assert(sensorarrayFdcJobReclaim(&life));
    /* Blocked bus: quarantine preserves storage even after a late release. */
    assert(sensorarrayFdcJobPost(&life));
    assert(sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_POSTED, SENSORARRAY_FDC_JOB_ACKED));
    assert(sensorarrayFdcJobChange(&life, SENSORARRAY_FDC_JOB_ACKED, SENSORARRAY_FDC_JOB_RUNNING));
    sensorarrayFdcJobCancel(&life);
    assert(!sensorarrayFdcJobReleased(&life));
    sensorarrayFdcJobQuarantine(&life);
    assert(!sensorarrayFdcJobPost(&life));
    assert(!sensorarrayFdcJobReclaim(&life));
    sensorarrayFdcJobFinish(&life);
    assert(sensorarrayFdcJobReleased(&life));
    assert(sensorarrayFdcJobPhase(&life) == SENSORARRAY_FDC_JOB_QUARANTINED);
    assert(!sensorarrayFdcJobPost(&life));
    assert(!sensorarrayFdcJobReclaim(&life));
    puts("FDC_WORKER_LIFECYCLE_PASS,scenarios=job_unconsumed;ack_delay;run_delay;done_lost;cancel_race;blocked_bus;late_done");
    return 0;
}
