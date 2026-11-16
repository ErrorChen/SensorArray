#include "sensorarrayLinkDiagnostics.h"
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"
#include "sensorarrayWorkBudget.h"

bool sensorarrayLinkDiagnosticsHandles(const char *cmd)
{
    return cmd && (strcmp(cmd, "BLELINK?") == 0 || strcmp(cmd, "BLETX?") == 0 ||
                   strcmp(cmd, "BLESVC?") == 0 || strcmp(cmd, "SCHEDWHY?") == 0);
}

esp_err_t sensorarrayLinkDiagnosticsFormat(const char *cmd, char *response, size_t size)
{
    if (!cmd || !response || !size) { return ESP_ERR_INVALID_ARG; }
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
            "BLESVC,v=1,gen=%lu,scope=boot,active=%u,started=%lu,completed=%lu,wallUs=%llu,idleChanged=%lu,idleSame=%lu,idleUnknown=%lu,wall=elapsed,cpu=unknown,blocked=unknown\n",
            (unsigned long)s.connectionGeneration, s.sdkCallActive, (unsigned long)s.sdkCallCount,
            (unsigned long)s.sdkCompletedCalls, (unsigned long long)s.sdkWallUs,
            (unsigned long)s.sdkIdleChanged, (unsigned long)s.sdkIdleUnchanged, (unsigned long)s.sdkIdleUnknown);
    } else { return ESP_ERR_NOT_SUPPORTED; }
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
