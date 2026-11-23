#!/usr/bin/env python3
"""Execute the real two-core startup gate; no SDK or hardware simulation claim."""
from pathlib import Path
import subprocess
import tempfile

from test_main_config_commit import function

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = (ROOT / 'main/main.c').read_text()
    gate = function(source, 'static void sensorarrayWifiStartupReservationComplete(')
    scan = function(source, 'static void sensorarrayScanTask(')
    app = function(source, 'void app_main(')
    assert scan.index('sensorarrayInitSystem(ctx)') < scan.index('sensorarrayWifiStartupReservationComplete(2u)')
    assert app.index('sensorarrayTransportConfigureWifiBoot(') < app.index('sensorarrayNetStatusInit();')
    assert app.index('sensorarrayAsyncLogStartUsbSink();') < app.index('sensorarrayWifiStartupReservationComplete(1u)')
    failure = function(app, '    if (scanErr != ESP_OK)')
    assert failure.index('sensorarrayWifiStartupReservationComplete(3u)') < failure.index('while (true)')
    unit = r'''
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t s_wifiStartupReservations;
static unsigned released;
static void sensorarrayTransportReleaseWifiStartupGate(void) {
    __atomic_fetch_add(&released, 1u, __ATOMIC_RELAXED);
}
''' + gate + r'''
static void *complete(void *arg) {
    sensorarrayWifiStartupReservationComplete((uint32_t)(uintptr_t)arg);
    return NULL;
}
int main(void) {
    for (unsigned first=1; first<=2; first++) {
        s_wifiStartupReservations=0; released=0;
        sensorarrayWifiStartupReservationComplete(first);
        assert(released==0);
        sensorarrayWifiStartupReservationComplete(first);
        assert(released==0);
        sensorarrayWifiStartupReservationComplete(3u-first);
        sensorarrayWifiStartupReservationComplete(3u);
        assert(released==1);
    }
    s_wifiStartupReservations=0; released=0;
    sensorarrayWifiStartupReservationComplete(3u);
    assert(released==1);
    for (unsigned i=0; i<1000; i++) {
        s_wifiStartupReservations=0; released=0;
        pthread_t a,b;
        assert(pthread_create(&a,NULL,complete,(void *)(uintptr_t)1u)==0);
        assert(pthread_create(&b,NULL,complete,(void *)(uintptr_t)2u)==0);
        assert(pthread_join(a,NULL)==0 && pthread_join(b,NULL)==0);
        assert(released==1 && s_wifiStartupReservations==3u);
    }
    puts("WIFI_STARTUP_GATE_PASS,orders=2,concurrent=1000,failed_scan=1,hardware=NOT_RUN");
}
'''
    with tempfile.TemporaryDirectory(prefix='sensorarray-wifi-startup-') as temp:
        c = Path(temp)/'gate.c'; exe = Path(temp)/'gate'
        c.write_text(unit)
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-pthread',str(c),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True,timeout=15)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
