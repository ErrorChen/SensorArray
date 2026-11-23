#!/usr/bin/env python3
"""Actual shared Transport call paths plus the real persistent Wi-Fi owner."""
from pathlib import Path
import os, subprocess, tempfile, hashlib, re
from test_main_config_commit import function
ROOT=Path(__file__).resolve().parents[1]
HEADERS=['esp_heap_caps.h','esp_timer.h','esp_event.h','esp_netif.h','esp_wifi.h','esp_wifi_default.h',
         'sensorarrayWorkBudget.h','freertos/FreeRTOS.h','freertos/task.h','sdkconfig.h','lwip/inet.h','lwip/sockets.h']
NAMES=['static TickType_t sensorarrayTransportWaitTicks(',
       'static void sensorarrayTransportBuildWifiConfig(', 'static esp_err_t sensorarrayTransportStartWifiAp(',
       'static void sensorarrayTransportStatResult(', 'static void sensorarrayTransportWifiCompleted(',
       'static void sensorarrayTransportDrainGuaranteedText(', 'esp_err_t sensorarrayTransportRequestWifiMode(',
       'esp_err_t sensorarrayTransportApplyWifiMode(', 'esp_err_t sensorarrayTransportPublishControlReply(']
transport=(ROOT/'core/transport/sensorarrayTransport.c').read_text()
# No application Wi-Fi/lwIP SDK invocation remains in shared Transport or command.
for name in ['sensorarrayTransport.c','sensorarrayTransportCommand.c','sensorarrayTransportWifiControl.c']:
    source=(ROOT/'core/transport'/name).read_text()
    assert not re.search(r'\b(?:sendto|recvfrom|esp_wifi_\w+|sensorarrayWifiInit|sensorarrayWifiSendControlReply|sensorarrayWifiSend)\s*\(',source),name
with tempfile.TemporaryDirectory(prefix='wifi-shared-') as name:
    td=Path(name)
    for header in HEADERS:
        p=td/header;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "common.h"\n')
    (td/'wifi_shared_functions.h').write_text('\n'.join(function(transport,n) for n in NAMES))
    cmd=[os.environ.get('CC','gcc'),'-std=c11','-D_GNU_SOURCE','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-pthread','-O1','-fsanitize=undefined','-g']
    for inc in [td,ROOT/'tests/host/wifi_owner_stubs',ROOT/'tests/host/include',ROOT/'core/transport',ROOT/'components/sensorarrayWifi/include',ROOT/'components/sensorarrayWifi']:
        cmd+=['-I',str(inc)]
    cmd += [str(ROOT/'tests/host/test_wifi_transport_isolation.c'),str(ROOT/'core/transport/sensorarrayTransportWifiControl.c'),'-o',str(td/'test')]
    subprocess.run(cmd,check=True,cwd=ROOT)
    subprocess.run([str(td/'test')],check=True,cwd=ROOT,timeout=10)
print('WIFI_SHARED_SOURCE_SHA256,'+hashlib.sha256(transport.encode()).hexdigest())
