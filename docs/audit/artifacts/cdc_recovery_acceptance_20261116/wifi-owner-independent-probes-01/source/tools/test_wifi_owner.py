#!/usr/bin/env python3
"""Compile real Wi-Fi owner/backend/observer; inject SDK failures and blocked returns."""
from pathlib import Path
import os, subprocess, tempfile, sys
ROOT = Path(__file__).resolve().parents[1]
HEADERS = ['esp_heap_caps.h','esp_timer.h','esp_event.h','esp_netif.h','esp_wifi.h','esp_wifi_default.h',
           'sensorarrayWorkBudget.h','freertos/FreeRTOS.h','freertos/task.h','sdkconfig.h','lwip/inet.h','lwip/sockets.h']
CASES = ['off','success','oom','timer_create','timer_start','task_create','blocked_send','late_send',
         'off_inflight','partial','queue_epoch','priority','expiry','terminal','request_wrap','init_deadline',
         'failed_cleanup','profile_error','reply_off','callback_event']
with tempfile.TemporaryDirectory(prefix='wifi-owner-test-') as td:
    td=Path(td)
    for name in HEADERS:
        p=td/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "common.h"\n')
    cmd=[os.environ.get('CC','gcc'),'-std=c11','-D_GNU_SOURCE','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-pthread','-O2',
         '-I'+str(td),'-I'+str(ROOT/'tests/host/wifi_owner_stubs'),'-I'+str(ROOT/'tests/host/include'),
         '-I'+str(ROOT/'components/sensorarrayWifi/include'),'-I'+str(ROOT/'components/sensorarrayWifi'),
         str(ROOT/'tests/host/test_wifi_owner.c'),'-o',str(td/'test')]
    subprocess.run(cmd,check=True,cwd=ROOT)
    for case in CASES:
        subprocess.run([str(td/'test'),case],check=True,cwd=ROOT,timeout=8)
    # Every init phase, including both sockets/options and RX-task allocation.
    for phase in range(1,18):
        subprocess.run([str(td/'test'),'phase_failure',str(phase)],check=True,cwd=ROOT,timeout=8)
    subprocess.run([str(td/'test'),'phase_failure','26'],check=True,cwd=ROOT,timeout=8)
print('wifi_owner: real owner/backend/watchdog; 38 scenarios PASS; no hardware evidence')
