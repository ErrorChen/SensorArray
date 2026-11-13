#include "sensorarrayLinkDiagnostics.h"
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"
esp_err_t sensorarrayLinkDiagnosticsFormat(const char *cmd, char *response, size_t size)
{
    if (!cmd || !response || !size) { return ESP_ERR_INVALID_ARG; }
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
    } else { return ESP_ERR_NOT_SUPPORTED; }
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
