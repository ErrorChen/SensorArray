#include "sensorarrayDebugFdcI2cDiscovery.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "boardSupport.h"

#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"

typedef struct {
    uint8_t busIndex;
    const BoardSupportI2cCtx_t *i2cCtx;
    BoardSupportI2cBusInfo_t busInfo;
} sensorarrayFdcScanBus_t;

static const uint8_t kFdcProbeAddresses[] = {
    SENSORARRAY_FDC_I2C_ADDR_LOW,
    SENSORARRAY_FDC_I2C_ADDR_HIGH,
};

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const char *sensorarrayFmtHex16(char *buf, size_t bufSize, bool valid, uint16_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "0x%04X", value);
    return buf;
}

static sensorarrayFdcScanBus_t sensorarrayLoadScanBus(bool secondary)
{
    sensorarrayFdcScanBus_t bus = {
        .busIndex = secondary ? 1u : 0u,
        .i2cCtx = secondary ? boardSupportGetI2c1Ctx() : boardSupportGetI2cCtx(),
        .busInfo = {0},
    };
    (void)boardSupportGetI2cBusInfo(secondary, &bus.busInfo);
    return bus;
}

static void sensorarrayLogBootBus(const sensorarrayFdcScanBus_t *bus)
{
    if (!bus) {
        return;
    }

    printf("FDC_SCAN_BOOT,bus=%u,port=%d,sda=%d,scl=%d,freqHz=%lu,enabled=%u,ctx=%s\n",
           (unsigned)bus->busIndex,
           (int)bus->busInfo.Port,
           bus->busInfo.SdaGpio,
           bus->busInfo.SclGpio,
           (unsigned long)bus->busInfo.FrequencyHz,
           bus->busInfo.Enabled ? 1u : 0u,
           bus->i2cCtx ? "ready" : "missing");
}

static void sensorarrayLogCandidate(uint32_t cycle,
                                    const sensorarrayFdcScanBus_t *bus,
                                    uint8_t i2cAddr,
                                    const sensorarrayFdcProbeDiag_t *diag)
{
    if (!bus || !diag) {
        return;
    }

    char mfgBuf[12];
    char devBuf[12];
    printf("FDC_SCAN,cycle=%lu,bus=%u,sda=%d,scl=%d,freqHz=%lu,addr=0x%02X,ack=%s,mfg=%s,dev=%s,"
           "ackErr=%ld,idErr=%ld,status=%s\n",
           (unsigned long)cycle,
           (unsigned)bus->busIndex,
           bus->busInfo.SdaGpio,
           bus->busInfo.SclGpio,
           (unsigned long)bus->busInfo.FrequencyHz,
           i2cAddr,
           diag->ack ? "ack" : "no_ack",
           sensorarrayFmtHex16(mfgBuf, sizeof(mfgBuf), diag->haveManufacturerId, diag->manufacturerId),
           sensorarrayFmtHex16(devBuf, sizeof(devBuf), diag->haveDeviceId, diag->deviceId),
           (long)diag->ackErr,
           (long)diag->idErr,
           sensorarrayBringupFdcDiscoveryStatusName(diag->status));
}

static void sensorarrayAppendFound(char *buf, size_t bufSize, uint8_t busIndex, uint8_t i2cAddr, uint32_t foundCount)
{
    if (!buf || bufSize == 0u) {
        return;
    }

    size_t used = strlen(buf);
    if (used >= (bufSize - 1u)) {
        return;
    }

    snprintf(buf + used,
             bufSize - used,
             "%si2c%u:0x%02X",
             (foundCount == 0u) ? "" : "|",
             (unsigned)busIndex,
             i2cAddr);
}

void sensorarrayDebugRunFdcI2cDiscoveryMode(sensorarrayState_t *state)
{
    const uint32_t pollDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_FDC_I2C_DISCOVERY_LOOP_DELAY_MS;
    uint32_t cycle = 0u;

    sensorarrayFdcScanBus_t bus0 = sensorarrayLoadScanBus(false);
    sensorarrayFdcScanBus_t bus1 = sensorarrayLoadScanBus(true);
    printf("FDC_SCAN_BOOT,mode=FDC_I2C_DISCOVERY,pollMs=%lu,boardReady=%u,addresses=0x2A|0x2B\n",
           (unsigned long)pollDelayMs,
           (state && state->boardReady) ? 1u : 0u);
    sensorarrayLogBootBus(&bus0);
    sensorarrayLogBootBus(&bus1);

    while (true) {
        char foundBuf[64] = {0};
        uint32_t foundCount = 0u;

        for (size_t busIdx = 0; busIdx < 2u; ++busIdx) {
            sensorarrayFdcScanBus_t bus = sensorarrayLoadScanBus(busIdx == 1u);

            for (size_t addrIdx = 0; addrIdx < (sizeof(kFdcProbeAddresses) / sizeof(kFdcProbeAddresses[0])); ++addrIdx) {
                uint8_t i2cAddr = kFdcProbeAddresses[addrIdx];
                sensorarrayFdcProbeDiag_t diag = {
                    .status = SENSORARRAY_FDC_DISCOVERY_NO_ACK,
                    .ackErr = ESP_ERR_INVALID_STATE,
                    .idErr = ESP_ERR_INVALID_STATE,
                    .ack = false,
                    .haveManufacturerId = false,
                    .haveDeviceId = false,
                    .manufacturerId = 0u,
                    .deviceId = 0u,
                };

                if (bus.busInfo.Enabled && bus.i2cCtx) {
                    (void)sensorarrayBringupProbeFdcCandidate(bus.i2cCtx, i2cAddr, &diag);
                } else if (!bus.busInfo.Enabled) {
                    diag.ackErr = ESP_ERR_NOT_SUPPORTED;
                }

                sensorarrayLogCandidate(cycle, &bus, i2cAddr, &diag);

                if (diag.status == SENSORARRAY_FDC_DISCOVERY_ID_OK) {
                    sensorarrayAppendFound(foundBuf, sizeof(foundBuf), bus.busIndex, i2cAddr, foundCount);
                    foundCount++;
                }
            }
        }

        printf("FDC_SCAN_SUMMARY,cycle=%lu,found=%s\n",
               (unsigned long)cycle,
               (foundCount == 0u) ? "none" : foundBuf);

        cycle++;
        sensorarrayDelayMs(pollDelayMs);
    }
}
