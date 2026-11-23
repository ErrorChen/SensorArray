#include "sensorarraySnapshotDiagnostics.h"
#include "sensorarrayAsyncLog.h"
#include "sensorarrayTransport.h"

#include <stdio.h>
#include <string.h>

bool sensorarraySnapshotDiagnosticsHandles(const char *command)
{
    return command && (strcmp(command, "SNAPSRC?") == 0 ||
                       strcmp(command, "SNAPNET?") == 0);
}

esp_err_t sensorarraySnapshotDiagnosticsFormat(const char *command,
                                               char *response, size_t size)
{
    if (!command || !response || size == 0u) { return ESP_ERR_INVALID_ARG; }
    int written;
    if (strcmp(command, "SNAPSRC?") == 0) {
        sensorarrayAsyncLogProtectionStats_t stats = {0};
        sensorarrayAsyncLogGetProtectionStats(&stats);
        written = snprintf(response, size,
            "SNAPSRC,v=1,scope=boot,queued=%lu,firstQueued=%lu,periodicQueued=%lu,"
            "candidateMiss=%lu,firstMiss=%lu,superseded=%lu,firstSuperseded=%lu,"
            "stage=source_queue,pcReceipt=unknown\n",
            (unsigned long)stats.queued, (unsigned long)stats.firstConfigQueued,
            (unsigned long)stats.periodicQueued, (unsigned long)stats.candidateMiss,
            (unsigned long)stats.firstCandidateMiss, (unsigned long)stats.superseded,
            (unsigned long)stats.firstSuperseded);
    } else if (strcmp(command, "SNAPNET?") == 0) {
        sensorarrayTransportProtectionStats_t protection = {0};
        sensorarrayTransportStats_t transport = {0};
        /* Separate short snapshots: do not promise a cross-counter atomic
         * instant, and never hold one layer's lock while entering another. */
        sensorarrayTransportGetProtectionStats(&protection);
        sensorarrayTransportGetStats(&transport);
        written = snprintf(response, size,
            "SNAPNET,v=1,scope=boot,readout=sequential,queued=%lu,admissionMiss=%lu,"
            "firstMiss=%lu,derived=%lu,"
            "superseded=%lu,retry=%lu,expired=%lu,bleExpired=%lu,wifiExpired=%lu,bleAccepted=%lu,bleFailed=%lu,"
            "policySkip=%lu,obsolete=%lu,stage=ble_admission,pcReceipt=unknown\n",
            (unsigned long)protection.queued, (unsigned long)protection.admissionMiss,
            (unsigned long)protection.firstCandidateMiss, (unsigned long)protection.derivedProtection,
            (unsigned long)protection.superseded, (unsigned long)protection.retryAttempts,
            (unsigned long)protection.expired,
            (unsigned long)protection.bleExpired, (unsigned long)protection.wifiExpired,
            (unsigned long)protection.bleAccepted,
            (unsigned long)protection.bleFailed, (unsigned long)transport.bleDataPolicySkipped,
            (unsigned long)transport.bleDataObsolete);
    } else {
        return ESP_ERR_NOT_FOUND;
    }
    if (written < 0 || (size_t)written >= size) {
        /* A caller must never publish a truncated diagnostic as a full reply. */
        response[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}
