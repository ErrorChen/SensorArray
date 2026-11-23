#!/usr/bin/env python3
"""Compile real Wi-Fi owner/backend/observer; inject SDK failures and blocked returns."""
from pathlib import Path
import os, subprocess, tempfile, sys, hashlib
ROOT = Path(__file__).resolve().parents[1]
HEADERS = ['esp_heap_caps.h','esp_timer.h','esp_event.h','esp_netif.h','esp_wifi.h','esp_wifi_default.h',
           'sensorarrayWorkBudget.h','freertos/FreeRTOS.h','freertos/task.h','sdkconfig.h','lwip/inet.h','lwip/sockets.h']
CASES = ['off','success','oom','timer_create','timer_start','task_create','blocked_send','late_send',
         'off_inflight','partial','queue_epoch','priority','expiry','terminal','request_wrap','init_deadline',
         'failed_cleanup','profile_error','reply_off','callback_event','observer_interleave','cleanup_blocked','invalid_enum','idle_pending','writing_epoch_race','execution_snapshot']
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
    for phase in range(1,18):
        subprocess.run([str(td/'test'),'blocked_phase',str(phase)],check=True,cwd=ROOT,timeout=8)
    # These are explicitly reconstructed mutations of the current source, not
    # archived deployed firmware. They must actually reproduce the assertions
    # which the clock/cleanup fixes protect.
    owner=(ROOT/'components/sensorarrayWifi/sensorarrayWifiOwner.c').read_text()
    mutations={
        'observer_interleave': owner.replace('void sensorarrayWifiObserveDeadline(void)\n{\n    portENTER_CRITICAL(&s_wifiMux);\n    uint64_t now = wifiNow();',
            'void sensorarrayWifiObserveDeadline(void)\n{\n    uint64_t now = wifiNow();\n    portENTER_CRITICAL(&s_wifiMux);',1),
        'cleanup_blocked': owner.replace('        if (s_owner.jobCleanup && s_owner.cleanupError == ESP_OK) { s_owner.cleanupError = ESP_ERR_TIMEOUT; }\n','',1),
    }
    for case, mutant in mutations.items():
        assert mutant != owner
        mutated=td/(case+'.c');mutated.write_text(mutant)
        harness=(ROOT/'tests/host/test_wifi_owner.c').read_text()
        for name in ['sensorarrayWifiOwner.c','sensorarrayWifi.c','sensorarrayWifiWatchdog.c']:
            target=mutated if name=='sensorarrayWifiOwner.c' else ROOT/'components/sensorarrayWifi'/name
            harness=harness.replace('../../components/sensorarrayWifi/'+name,str(target))
        unit=td/(case+'-test.c');unit.write_text(harness)
        variant_cmd=cmd.copy();variant_cmd[variant_cmd.index(str(ROOT/'tests/host/test_wifi_owner.c'))]=str(unit)
        subprocess.run(variant_cmd,check=True,cwd=ROOT)
        result=subprocess.run([str(td/'test'),case],text=True,capture_output=True,cwd=ROOT,timeout=8)
        assert result.returncode != 0 and 'Assertion' in result.stderr, result
        print('WIFI_MUTATION_REPRODUCED,kind=synthetic_reconstruction,case='+case+',exit='+str(result.returncode)+',source_sha256='+hashlib.sha256(mutant.encode()).hexdigest())
        print(result.stderr.strip())

print('wifi_owner: real owner/backend/watchdog; 61 scenarios PASS; no hardware evidence')
