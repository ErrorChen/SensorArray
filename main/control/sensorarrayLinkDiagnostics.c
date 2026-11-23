#include "sensorarrayLinkDiagnostics.h"
#include "sensorarraySnapshotDiagnostics.h"
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"
#include "sensorarrayWorkBudget.h"

bool sensorarrayLinkDiagnosticsHandles(const char *cmd)
{
    return cmd && (sensorarraySnapshotDiagnosticsHandles(cmd) ||
                   strcmp(cmd, "BLEFLOW?") == 0 || strcmp(cmd, "BLESNAP?") == 0 ||
                   strcmp(cmd, "BLELINK?") == 0 || strcmp(cmd, "BLETX?") == 0 ||
                   strcmp(cmd, "BLESVC?") == 0 || strcmp(cmd, "SCHEDWHY?") == 0 ||
                   strcmp(cmd, "BLEGUARD?") == 0 || strcmp(cmd, "BLELOG?") == 0);
}

esp_err_t sensorarrayLinkDiagnosticsFormat(const char *cmd, char *response, size_t size)
{
    if (!cmd || !response || !size) { return ESP_ERR_INVALID_ARG; }
    if (sensorarraySnapshotDiagnosticsHandles(cmd))
        return sensorarraySnapshotDiagnosticsFormat(cmd, response, size);
    if (strcmp(cmd, "BLEFLOW?") == 0 || strcmp(cmd, "BLESNAP?") == 0) {
        sensorarrayBleFlowSnapshot_t f = {0};
        sensorarrayBleGetFlowSnapshot(&f);
        int n;
        if (strcmp(cmd, "BLEFLOW?") == 0) {
            static const char *stages[] = {"inactive", "full", "pressure", "limited"};
            char service[24];
            if (f.serviceKnown) snprintf(service, sizeof service, "%lu", (unsigned long)f.localServiceMilliFps);
            else snprintf(service, sizeof service, "unknown");
            n = snprintf(response, size,
                "BLEFLOW,v=1,scope=local_output,counters=boot,gen=%lu,stream=%lu,active=%u,stage=%s,ratePm=%u,intentionalSkip=%llu,probeSelected=%llu,attempts=%lu,capacityReject=%lu,evidence=%u,rejectPm=%u,pressureWindows=%u,cleanUs=%llu,localServiceMilliFps=%s,probeUs=1000000,cleanTtlUs=3000000,peerLoss=unknown,rssi=unknown\n",
                (unsigned long)f.connectionGeneration, (unsigned long)f.streamGeneration, f.active,
                f.stage < sizeof stages / sizeof *stages ? stages[f.stage] : "unknown", f.ordinaryRatePermille,
                (unsigned long long)f.intentionalSkip, (unsigned long long)f.probeSelected,
                (unsigned long)f.admissionAttempts, (unsigned long)f.capacityRejected, f.evidenceKnown,
                f.capacityRejectedPermille, f.pressureWindows, (unsigned long long)f.cleanEvidenceUs, service);
        } else {
            char sequence[24], revision[24], age[24], gap[24];
            if (f.lastCompleteKnown) {
                snprintf(sequence, sizeof sequence, "%lu", (unsigned long)f.lastSequence);
                snprintf(revision, sizeof revision, "%lu", (unsigned long)f.lastRevision);
            } else { snprintf(sequence, sizeof sequence, "unknown"); snprintf(revision, sizeof revision, "unknown"); }
            if (f.lastCompleteAgeKnown) snprintf(age, sizeof age, "%llu", (unsigned long long)f.lastCompleteAgeUs);
            else snprintf(age, sizeof age, "unknown");
            if (f.gapKnown) snprintf(gap, sizeof gap, "%llu", (unsigned long long)f.maxCompleteGapUs);
            else snprintf(gap, sizeof gap, "unknown");
            n = snprintf(response, size,
                "BLESNAP,v=1,scope=local_complete,counters=boot,gen=%lu,stream=%lu,queued=%lu,localComplete=%lu,failed=%lu,admitMiss=%lu,deadlineMiss=%lu,superseded=%lu,staleNormal=%lu,known=%u,lastSeq=%s,lastRev=%s,lastCompleteAgeUs=%s,gapKnown=%u,maxCompleteGapUs=%s,firstPending=%u,periodicPending=%u,dueUs=1000000,pcReceipt=unknown\n",
                (unsigned long)f.connectionGeneration, (unsigned long)f.streamGeneration,
                (unsigned long)f.protectedQueued, (unsigned long)f.protectedComplete, (unsigned long)f.protectedFailed,
                (unsigned long)f.protectedAdmissionMiss, (unsigned long)f.protectedDeadlineMiss,
                (unsigned long)f.protectedSuperseded, (unsigned long)f.staleNormal, f.lastCompleteKnown,
                sequence, revision, age, f.gapKnown, gap, f.firstPending, f.periodicPending);
        }
        return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    /* New query is its own versioned capability probe. Keep existing PROTO
     * and reply sizes unchanged; unknown firmware returns unsupported. */
    if (strcmp(cmd, "BLELOG?") == 0) {
        sensorarrayBleLogPolicySnapshot_t p = {0};
        sensorarrayBleGetLogPolicySnapshot(&p);
        int n = snprintf(response, size,
            "BLELOG,v=1,scope=boot,gen=%lu,connected=%u,mode=%s,dataCccd=%u,dataActive=%u,ordinaryAllowed=%u,policy=drop_ordinary_when_data,policyDrop=%lu,dropL=%lu,priorityBypass=1,inflightCancel=0,pcReceipt=unknown\n",
            (unsigned long)p.connectionGeneration, p.connected,
            p.txMode == SENSORARRAY_BLE_TX_SAFE ? "safe" : "fast", p.dataCccd,
            p.dataActive, p.ordinaryAllowed, (unsigned long)p.policyDrop,
            (unsigned long)p.droppedLog);
        return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    if (strcmp(cmd, "BLEGUARD?") == 0) {
        sensorarrayBleGuardSnapshot_t g = {0};
        sensorarrayBleGetGuardSnapshot(&g);
        int n = snprintf(response, size,
            "BLEGUARD,v=1,scope=send_observer,ready=%u,initErr=0x%lX,quarantined=%u,reason=%s,callKnown=%u,callId=%lu,startUs=%llu,detectedUs=%llu,ageUs=%llu,deadlineUs=%lu,pollUs=%lu,active=%u,completionKnown=%u,restartRequired=%u,rejected=%lu,recovery=manual_restart,sdkCancelled=0,pcReceipt=unknown\n",
            g.ready, (unsigned long)g.initError, g.quarantined,
            g.quarantined ? "sdk_deadline" : (g.ready ? "none" : "observer_unavailable"),
            g.callKnown, (unsigned long)g.callId, (unsigned long long)g.startUs,
            (unsigned long long)g.detectedUs, (unsigned long long)g.ageUs,
            (unsigned long)g.deadlineUs, (unsigned long)g.pollUs, g.active,
            g.completionKnown, g.restartRequired, (unsigned long)g.rejected);
        return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    if (strcmp(cmd, "SCHEDWHY?") == 0) {
        sensorarrayWorkBudgetSnapshot_t b = {0};
        sensorarrayWorkBudgetCopySnapshot(&b);
        int n = snprintf(response, size,
            "SCHEDWHY,v=1,order=quantum/backstop/units,overlap=1,init=0x%lX,registered=%u,idleKnown=%u,idleGen=%lu,bt=%lu/%lu/%lu,tr=%lu/%lu/%lu,log=%lu/%lu/%lu,ser=%lu/%lu/%lu,bc=%lu/%lu/%lu,sc=%lu/%lu/%lu,wc=%lu/%lu/%lu\n",
            (unsigned long)b.initError, b.registered, b.idleKnown, (unsigned long)b.idleGeneration,
            (unsigned long)b.workers[0].quantumDue, (unsigned long)b.workers[0].backstopDue, (unsigned long)b.workers[0].unitsDue,
            (unsigned long)b.workers[1].quantumDue, (unsigned long)b.workers[1].backstopDue, (unsigned long)b.workers[1].unitsDue,
            (unsigned long)b.workers[2].quantumDue, (unsigned long)b.workers[2].backstopDue, (unsigned long)b.workers[2].unitsDue,
            (unsigned long)b.workers[3].quantumDue, (unsigned long)b.workers[3].backstopDue, (unsigned long)b.workers[3].unitsDue,
            (unsigned long)b.workers[4].quantumDue, (unsigned long)b.workers[4].backstopDue, (unsigned long)b.workers[4].unitsDue,
            (unsigned long)b.workers[5].quantumDue, (unsigned long)b.workers[5].backstopDue, (unsigned long)b.workers[5].unitsDue,
            (unsigned long)b.workers[6].quantumDue, (unsigned long)b.workers[6].backstopDue, (unsigned long)b.workers[6].unitsDue);
        return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    sensorarrayBleLinkSnapshot_t s = {0};
    sensorarrayBleGetLinkSnapshot(&s);
    int n;
    if (strcmp(cmd, "BLELINK?") == 0) {
        n = snprintf(response, size,
            "BLELINK,gen=%lu,connected=%u,mtu=%u,phyKnown=%u,txPhy=%u,rxPhy=%u,connKnown=%u,interval1250us=%u,latency=%u,timeout10ms=%u,phyReq=0x%lX,connReq=0x%lX,dleReq=0x%lX,phyStatus=%u,connStatus=%u,dleStatus=%u,dleObserved=%u,dlePeerKnown=%u,dleTx=%u,dleRx=%u,stale=%lu\n",
            (unsigned long)s.connectionGeneration, s.connected, s.mtu, s.phyKnown, s.txPhy, s.rxPhy,
            s.connParamsKnown, s.connIntervalUnits, s.connLatency, s.supervisionTimeoutUnits,
            (unsigned long)s.phyRequestError, (unsigned long)s.connRequestError, (unsigned long)s.dataLengthRequestError,
            s.phyUpdateStatus, s.connUpdateStatus, s.dataLengthStatus, s.dataLengthObserved,
            s.dataLengthConnectionKnown, s.dataLengthTx, s.dataLengthRx, (unsigned long)s.staleLinkEvents);
    } else if (strcmp(cmd, "BLETX?") == 0) {
        n = snprintf(response, size,
            "BLETX,gen=%lu,dDequeued=%lu,lDequeued=%lu,cDequeued=%lu,dMaxAgeUs=%lu,lMaxAgeUs=%lu,cMaxAgeUs=%lu,sdkActive=%u,sdkStartUs=%llu,sdkEndUs=%llu,sdkCalls=%lu,sdkAccepted=%lu,sdkErrors=%lu,sdkMaxUs=%lu,sdkLastErr=0x%lX,txPrio=%lu,ctrlPrio=%lu,pcReceipt=unknown\n",
            (unsigned long)s.connectionGeneration, (unsigned long)s.dataDequeued, (unsigned long)s.logDequeued,
            (unsigned long)s.priorityDequeued, (unsigned long)s.queueMaxAgeUs[0], (unsigned long)s.queueMaxAgeUs[1],
            (unsigned long)s.queueMaxAgeUs[2], s.sdkCallActive, (unsigned long long)s.sdkCallStartedUs,
            (unsigned long long)s.sdkCallCompletedUs, (unsigned long)s.sdkCallCount, (unsigned long)s.sdkAccepted,
            (unsigned long)s.sdkErrors, (unsigned long)s.sdkMaxCallUs, (unsigned long)s.sdkLastError,
            (unsigned long)s.txTaskPriority, (unsigned long)s.ctrlTaskPriority);
    } else if (strcmp(cmd, "BLESVC?") == 0) {
        n = snprintf(response, size,
            "BLESVC,v=1,gen=%lu,scope=boot,active=%u,started=%lu,completed=%lu,wallUs=%llu,idleChanged=%lu,idleSame=%lu,idleUnknown=%lu,wall=elapsed,cpu=unknown,blocked=unknown,confOk=%lu,confError=%lu,confMatched=%lu,confUnmatched=%lu,confMatch=waiter_predicate,confEpoch=unknown,pcReceipt=unknown\n",
            (unsigned long)s.connectionGeneration, s.sdkCallActive, (unsigned long)s.sdkCallCount,
            (unsigned long)s.sdkCompletedCalls, (unsigned long long)s.sdkWallUs,
            (unsigned long)s.sdkIdleChanged, (unsigned long)s.sdkIdleUnchanged, (unsigned long)s.sdkIdleUnknown,
            (unsigned long)s.confOk, (unsigned long)s.confError,
            (unsigned long)s.confMatched, (unsigned long)s.confUnmatched);
    } else { return ESP_ERR_NOT_SUPPORTED; }
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
