#!/usr/bin/env python3
"""Run actual guaranteed terminal slot/drain with synthetic USB/BLE/Wi-Fi boundaries."""
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(root / "tools"))
from test_main_config_commit import function

drain = function((root / "core/transport/sensorarrayTransport.c").read_text(),
                 "static void sensorarrayTransportDrainGuaranteedText(")
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayTransportGuaranteedText.c"
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT 1
#define SENSORARRAY_TRANSPORT_CHANNEL_LOG 1
#define SENSORARRAY_BLE_CH_LOG 1
#define SENSORARRAY_WIFI_CH_LOG 1
static bool bleEnabled,bleConnected,wifiEnabled,wifiReady;
static unsigned serialCalls,bleCalls,wifiCalls,serialNotes,bleNotes,wifiNotes;
static esp_err_t serialError,bleError,wifiError,bleNoted,wifiNoted;
static bool serialNoted;
static const char terminal[]="RAPP,cmd=RECOVER,id=123,state=rejected,err=0x109\n";
static void checkText(const char *s,size_t n) { assert(n==sizeof(terminal)-1 && memcmp(s,terminal,n)==0); }
static esp_err_t sensorarraySerialIoQueue(const char *s,size_t n,bool priority) {
    assert(priority);checkText(s,n);serialCalls++;return serialError;
}
static void sensorarrayTransportNoteSerialLog(bool accepted) { serialNotes++;serialNoted=accepted; }
static bool sensorarrayTransportBleChannelEnabled(unsigned ch) { assert(ch==1);return bleEnabled; }
static bool sensorarrayBleGetSendGeneration(unsigned ch,uint32_t *generation) { assert(ch==1);*generation=77;return bleConnected; }
static esp_err_t sensorarrayBleNotifyForGeneration(unsigned ch,const uint8_t *s,size_t n,uint32_t generation) {
    assert(ch==1 && generation==77);checkText((const char*)s,n);bleCalls++;return bleError;
}
static bool sensorarrayTransportWifiSinkEnabled(void) { return wifiEnabled; }
static bool sensorarrayWifiIsReady(void) { return wifiReady; }
static esp_err_t sensorarrayWifiSend(unsigned ch,const uint8_t *s,size_t n) { assert(ch==1);checkText((const char*)s,n);wifiCalls++;return wifiError; }
static void sensorarrayTransportStatResult(unsigned ch,bool isBle,esp_err_t e) {
    assert(ch==1);if(isBle){bleNotes++;bleNoted=e;}else{wifiNotes++;wifiNoted=e;}
}
/* DRAIN */
static void reset(void) {
    (void)sensorarrayTransportGuaranteedTextAbort();
    bleEnabled=bleConnected=wifiEnabled=wifiReady=false;
    serialCalls=bleCalls=wifiCalls=serialNotes=bleNotes=wifiNotes=0;
    serialError=bleError=wifiError=bleNoted=wifiNoted=ESP_OK;
    serialNoted=false;
    assert(sensorarrayTransportGuaranteedTextReserve()==ESP_OK);
}
static void ready(void) {
    assert(sensorarrayTransportGuaranteedTextPublish(terminal,sizeof(terminal)-1)==ESP_OK);
    assert(sensorarrayTransportGuaranteedTextRelease()==ESP_OK);
}
int main(void) {
    reset();ready();sensorarrayTransportDrainGuaranteedText();
    assert(serialCalls==1&&serialNotes==1&&serialNoted&&bleCalls==0&&wifiCalls==0);
    assert(!sensorarrayTransportGuaranteedTextIsReserved());
    reset();bleEnabled=bleConnected=true;serialError=ESP_ERR_NO_MEM;bleError=ESP_ERR_TIMEOUT;
    ready();sensorarrayTransportDrainGuaranteedText();
    assert(serialCalls==1&&!serialNoted&&bleCalls==1&&bleNotes==1&&bleNoted==ESP_ERR_TIMEOUT&&wifiCalls==0);
    reset();wifiEnabled=wifiReady=true;serialError=ESP_ERR_NO_MEM;wifiError=ESP_ERR_TIMEOUT;
    ready();sensorarrayTransportDrainGuaranteedText();
    assert(serialCalls==1&&!serialNoted&&wifiCalls==1&&wifiNotes==1&&wifiNoted==ESP_ERR_TIMEOUT&&bleCalls==0);
    reset();bleEnabled=bleConnected=wifiEnabled=wifiReady=true;
    serialError=bleError=wifiError=ESP_ERR_NO_MEM;ready();sensorarrayTransportDrainGuaranteedText();
    assert(serialCalls==1&&bleCalls==1&&wifiCalls==1&&!serialNoted&&bleNoted==ESP_ERR_NO_MEM&&wifiNoted==ESP_ERR_NO_MEM);
    assert(!sensorarrayTransportGuaranteedTextIsReserved());
    reset();assert(sensorarrayTransportGuaranteedTextRelease()==ESP_OK);
    sensorarrayTransportDrainGuaranteedText();assert(serialCalls==0&&sensorarrayTransportGuaranteedTextIsReserved());
    reset();assert(sensorarrayTransportGuaranteedTextPublish(terminal,sizeof(terminal)-1)==ESP_OK);
    sensorarrayTransportDrainGuaranteedText();assert(serialCalls==0&&sensorarrayTransportGuaranteedTextIsReserved());
    assert(sensorarrayTransportGuaranteedTextRelease()==ESP_OK);sensorarrayTransportDrainGuaranteedText();assert(serialCalls==1);
    puts("INDEPENDENT_GUARANTEED_SINK_PASS,scenarios=6,serial_unconditional_priority=1,ble_generation=1,wifi_actual_ready=1,each_failure_counted=1,no_early_abort=1,publish_release_barrier=1,real_slot_drain=1,sink_boundaries_synthetic=1,host_receipt_not_asserted=1");
}
'''.replace("/* DRAIN */", drain)
with tempfile.TemporaryDirectory(prefix="sa-guaranteed-review-") as name:
    tmp=Path(name);(tmp/"freertos").mkdir()
    (tmp/"freertos/FreeRTOS.h").write_text('''#pragma once
#include <pthread.h>
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) pthread_mutex_lock(p)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(p)
''')
    src=tmp/"review.c";src.write_text(harness);exe=tmp/"review"
    cmd=["/usr/bin/gcc","-std=c11","-Wall","-Wextra","-Werror","-pthread"]
    for inc in [tmp,root/"tests/host/include",root/"core/transport"]:
        cmd += ["-I",str(inc)]
    subprocess.run(cmd+[str(src),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
