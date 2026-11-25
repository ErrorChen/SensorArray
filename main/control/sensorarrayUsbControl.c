#include "sensorarrayUsbControl.h"
#include <stdio.h>
#include <string.h>
#include "sensorarrayUsbOtg.h"
#include "sensorarraySerialIo.h"
#include "sensorarrayRestartControl.h"
#include "sensorarrayConfigIdentity.h"

const char *sensorarrayUsbControlCapabilities(void)
{
    return sensorarrayUsbOtgEnabled() ? (sensorarrayUsbOtgCdcEnabled() ?
        "usb_cdc_data_v1/usb_rom_handoff_v1" : "usb_vendor_bulk_v1/usb_rom_handoff_v1") :
        "serial_sink_v1/serial_classes_v1/usb_rom_handoff_v1";
}

bool sensorarrayUsbControlHandles(const char *c)
{
    return c && (strcmp(c, "USB?") == 0 || strcmp(c, "USBQ?") == 0 || strcmp(c, "USBBOOT") == 0 ||
        (sensorarrayUsbOtgEnabled() && (strcmp(c, "SERIAL?") == 0 || strncmp(c, "USBSTREAM", 9u) == 0)));
}

esp_err_t sensorarrayUsbControlHandle(const char *command, char *response, size_t size)
{
    if (!command || !response || !size) { return ESP_ERR_INVALID_ARG; }
    int n = -1;
    if (strcmp(command, "USB?") == 0) {
        sensorarrayUsbOtgStats_t s = {0}; sensorarrayUsbOtgGetStats(&s);
        bool cdc = sensorarrayUsbOtgCdcEnabled();
        const char *dataDtr = cdc && s.initialized ? (s.cdcDtr[0] ? "1" : "0") : "unknown";
        const char *debugDtr = s.initialized ? (s.cdcDtr[cdc ? 1 : 0] ? "1" : "0") : "unknown";
        n = snprintf(response, size,
            "USB,backend=%s,init=%u,err=0x%lX,mounted=%u,suspended=%u,dma=%u,epoch=%lu,resets=%lu,mounts=%lu,unmounts=%lu,rxBytes=%llu,rxDrop=%lu,rxOver=%lu,lastRxUs=%lld,lastTxUs=%lld,romAvailable=%u,quiesced=%u,pcReceipt=unknown,dataDtr=%s,debugDtr=%s\n",
            sensorarrayUsbOtgEnabled() ? (sensorarrayUsbOtgCdcEnabled() ? "otg_cdc_v1" : "otg_bulk_v1") : "serial_jtag", s.initialized, (unsigned long)s.initError,
            s.mounted, s.suspended, s.dmaEnabled, (unsigned long)s.epoch, (unsigned long)s.resets,
            (unsigned long)s.mounts, (unsigned long)s.unmounts, (unsigned long long)s.rxBytes,
            (unsigned long)s.rxDropped, (unsigned long)s.rxOversize,
            (long long)s.lastRxUs, (long long)s.lastTxUs, sensorarrayUsbOtgRomDownloadAvailable(), s.quiesced,
            dataDtr, debugDtr);
    } else if (strcmp(command, "USBQ?") == 0) {
        sensorarrayUsbOtgStats_t u = {0}; sensorarrayUsbOtgGetStats(&u);
        sensorarraySerialIoStats_t s = {0}; sensorarraySerialIoGetStats(&s);
        char dataCallback[21], ctrlCallback[21];
        /* CDC combines DATA and CTRL in one endpoint FIFO. The stock CDC
         * callback does not identify bytes per application lane. */
        if (sensorarrayUsbOtgCdcEnabled()) {
            strcpy(dataCallback, "unknown"); strcpy(ctrlCallback, "unknown");
        } else {
            snprintf(dataCallback, sizeof(dataCallback), "%llu", (unsigned long long)u.callbackBytes[0]);
            snprintf(ctrlCallback, sizeof(ctrlCallback), "%llu", (unsigned long long)u.callbackBytes[1]);
        }
        n = snprintf(response, size,
            "USBQ,dDrop=%lu,cDrop=%lu,dSubmit=%lu,cSubmit=%lu,expired=%lu,cExpired=%lu,partial=%lu,dBytes=%llu,cBytes=%llu,dCallbackBytes=%s,cCallbackBytes=%s,queueMaxUs=%lu,stackMinBytes=%lu,callbackResult=unknown,dQueueCap=%lu,dLifetimeUs=%lu\n",
            (unsigned long)s.dataRejected, (unsigned long)s.priorityRejected,
            (unsigned long)s.dataSubmitted, (unsigned long)s.prioritySubmitted,
            (unsigned long)s.normalExpired, (unsigned long)s.priorityExpired, (unsigned long)s.partialWrite,
            (unsigned long long)u.accepted[0], (unsigned long long)u.accepted[1],
            dataCallback, ctrlCallback,
            (unsigned long)s.maxQueueAgeUs, (unsigned long)s.taskStackMinBytes,
            (unsigned long)s.dataQueueCapacity, (unsigned long)s.dataRecordLifetimeUs);
    } else if (strcmp(command, "SERIAL?") == 0 && sensorarrayUsbOtgEnabled()) {
        sensorarrayUsbOtgStats_t u = {0}; sensorarrayUsbOtgGetStats(&u);
        sensorarraySerialIoStats_t s = {0}; sensorarraySerialIoGetStats(&s);
        n = snprintf(response, size,
            "SERIAL,backend=otg_cdc,scope=debug,logRejected=%lu,logSubmitted=%lu,lineDropped=%lu,lineOversize=%lu,discardBytes=%llu,driverAcceptedBytes=%llu,rxCommands=unavailable,pcReceipt=unknown\n",
            (unsigned long)s.logRejected, (unsigned long)s.logSubmitted, (unsigned long)s.lineDropped,
            (unsigned long)s.lineOversize, (unsigned long long)s.discardBytes,
            (unsigned long long)u.accepted[2]);
    } else if (strncmp(command, "USBSTREAM", 9u) == 0 && sensorarrayUsbOtgEnabled()) {
        if (strcmp(command, "USBSTREAM?") == 0) {
            n = snprintf(response, size, "USBSTREAM,v=%s,scope=otg_data,dataEvery=1,debug=cdc,control=%s\n",
                sensorarrayUsbOtgCdcEnabled() ? "CDC" : "BULK",
                sensorarrayUsbOtgCdcEnabled() ? "shared_cdc" : "bulk_ctrl");
        } else {
            n = snprintf(response, size, "ERR,cmd=USBSTREAM,reason=legacy_usj_only,%sData=always\n",
                sensorarrayUsbOtgCdcEnabled() ? "cdc" : "bulk");
            return n > 0 && (size_t)n < size ? ESP_ERR_NOT_SUPPORTED : ESP_ERR_INVALID_SIZE;
        }
    } else if (strcmp(command, "USBBOOT") == 0) {
        /* Reserve no restart request if the largest ACK cannot be represented. */
        if (size < sizeof("RACK,cmd=RESTART,id=4294967295,state=accepted,target=rom_download\n")) {
            response[0] = '\0'; return ESP_ERR_INVALID_SIZE;
        }
        if (!sensorarrayUsbOtgRomDownloadAvailable()) {
            n = snprintf(response, size, "ERR,cmd=USBBOOT,reason=rom_download_disabled\n");
            return n > 0 && (size_t)n < size ? ESP_ERR_NOT_SUPPORTED : ESP_ERR_INVALID_SIZE;
        }
        sensorarrayConfigAuthority_t authority = {0};
        bool known = sensorarrayConfigIdentityCopy(&authority);
        uint32_t id = 0;
        esp_err_t err = sensorarrayRestartControlPostRom(&authority, known, &id);
        if (err != ESP_OK) {
            snprintf(response, size, "ERR,cmd=USBBOOT,reason=restart_pending\n"); return err;
        }
        /* Existing RACK publication guard gates this restart request as well. */
        n = snprintf(response, size, "RACK,cmd=RESTART,id=%lu,state=accepted,target=rom_download\n", (unsigned long)id);
    } else { return ESP_ERR_NOT_SUPPORTED; }
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
