#!/usr/bin/env python3
"""Compile actual per-layer snapshot query formatter against actual POD types."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def pod(path, name):
    match = re.search(r'typedef\s+struct\s*\{[^{}]*\}\s*' + name + r'\s*;',
                      (ROOT / path).read_text())
    assert match, name
    return match.group(0)


def main():
    # Capability discovery must remain a complete legacy 512-byte CTRL record
    # on every supported USB backend, including the longer Serial/JTAG list.
    main_source = (ROOT / 'main/main.c').read_text()
    proto_formats = re.findall(r'"(PROTO,[^"\n]+)"', main_source)
    assert len(proto_formats) == 2 and proto_formats[0] == proto_formats[1]
    usb_source = (ROOT / 'main/control/sensorarrayUsbControl.c').read_text()
    capability_body = usb_source.split('const char *sensorarrayUsbControlCapabilities(void)', 1)[1].split('}', 1)[0]
    capabilities = re.findall(r'"([^"\n]+)"', capability_body)
    assert len(capabilities) == 3
    for capability in capabilities:
        reply = (proto_formats[0] % (2, 512, 2304, capability)).replace('\\n', '\n')
        assert len(reply.encode()) + 1 <= 512, (capability, len(reply))
        caps = reply.split(',caps=', 1)[1].split(',', 1)[0].split('/')
        assert all(name in caps for name in ('ble_flow_v1', 'ble_snapshot_v1', 'heap_v1', 'caps2_v1'))
        assert len(caps) == len(set(caps)) and reply.endswith(',adcCrc=crc8\n')
        print('PROTO backend=%s bytes=%d capacity=512' % (capability, len(reply)))
    with tempfile.TemporaryDirectory(prefix='sensorarray-snapshot-query-') as tmp:
        directory = Path(tmp)
        (directory / 'esp_err.h').write_text('''#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_FOUND 0x105
''')
        (directory / 'sensorarrayAsyncLog.h').write_text('#pragma once\n#include <stdint.h>\n' +
            pod('main/output/sensorarrayAsyncLog.h', 'sensorarrayAsyncLogProtectionStats_t') +
            '\nvoid sensorarrayAsyncLogGetProtectionStats(sensorarrayAsyncLogProtectionStats_t *);\n')
        (directory / 'sensorarrayTransport.h').write_text('#pragma once\n#include <stdint.h>\n' +
            pod('core/transport/sensorarrayTransport.h', 'sensorarrayTransportProtectionStats_t') + '\n' +
            pod('core/transport/sensorarrayTransport.h', 'sensorarrayTransportStats_t') +
            '\nvoid sensorarrayTransportGetProtectionStats(sensorarrayTransportProtectionStats_t *);\n'
            'void sensorarrayTransportGetStats(sensorarrayTransportStats_t *);\n')
        harness = directory / 'harness.c'
        harness.write_text(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarraySnapshotDiagnostics.h"
#include "sensorarrayAsyncLog.h"
#include "sensorarrayTransport.h"
static sensorarrayAsyncLogProtectionStats_t source;
static sensorarrayTransportProtectionStats_t protection;
static sensorarrayTransportStats_t transport;
static unsigned sourceCalls, protectionCalls, transportCalls;
void sensorarrayAsyncLogGetProtectionStats(sensorarrayAsyncLogProtectionStats_t *out) {
    sourceCalls++; *out = source;
}
void sensorarrayTransportGetProtectionStats(sensorarrayTransportProtectionStats_t *out) {
    protectionCalls++; *out = protection;
}
void sensorarrayTransportGetStats(sensorarrayTransportStats_t *out) {
    transportCalls++; *out = transport;
}
int main(void) {
    char reply[520];
    assert(!sensorarraySnapshotDiagnosticsHandles(NULL));
    assert(!sensorarraySnapshotDiagnosticsHandles("SNAPSRC=1"));
    assert(sensorarraySnapshotDiagnosticsHandles("SNAPSRC?"));
    assert(sensorarraySnapshotDiagnosticsHandles("SNAPNET?"));
    assert(sensorarraySnapshotDiagnosticsFormat(NULL, reply, sizeof reply) == ESP_ERR_INVALID_ARG);
    assert(sensorarraySnapshotDiagnosticsFormat("SNAPSRC?", NULL, 512) == ESP_ERR_INVALID_ARG);
    assert(sensorarraySnapshotDiagnosticsFormat("SNAPSRC?", reply, 0) == ESP_ERR_INVALID_ARG);
    assert(sensorarraySnapshotDiagnosticsFormat("BAD?", reply, 512) == ESP_ERR_NOT_FOUND);
    assert(sourceCalls == 0 && protectionCalls == 0 && transportCalls == 0);
    assert(sensorarraySnapshotDiagnosticsFormat("SNAPSRC?", reply, 512) == ESP_OK);
    assert(strstr(reply, "queued=0,") && strstr(reply, "stage=source_queue,") && strstr(reply, "pcReceipt=unknown\n"));
    assert(sourceCalls == 1 && protectionCalls == 0 && transportCalls == 0);
    memset(&source, 0xff, sizeof source);
    memset(&protection, 0xff, sizeof protection);
    memset(&transport, 0xff, sizeof transport);
    sensorarrayAsyncLogProtectionStats_t originalSource = source;
    sensorarrayTransportProtectionStats_t originalProtection = protection;
    sensorarrayTransportStats_t originalTransport = transport;
    const char *commands[] = {"SNAPSRC?", "SNAPNET?"};
    for (unsigned i = 0; i < 2; ++i) {
        memset(reply, 0x5a, sizeof reply);
        assert(sensorarraySnapshotDiagnosticsFormat(commands[i], reply, 512) == ESP_OK);
        size_t required = strlen(reply) + 1;
        assert(required <= 512 && reply[512] == 0x5a);
        assert(strstr(reply, "queued=4294967295,") && strstr(reply, "pcReceipt=unknown\n"));
        if (i) {
            assert(strstr(reply, "readout=sequential,") && strstr(reply, "policySkip=4294967295,"));
            assert(strstr(reply, "firstMiss=4294967295,derived=4294967295,"));
            assert(strstr(reply, "bleExpired=4294967295,wifiExpired=4294967295,"));
            assert(strstr(reply, "stage=ble_admission,"));
        }
        printf("%s max_bytes=%zu\n", commands[i], required - 1);
        for (size_t capacity = 1; capacity < required; ++capacity) {
            memset(reply, 0x5a, sizeof reply);
            assert(sensorarraySnapshotDiagnosticsFormat(commands[i], reply, capacity) == ESP_ERR_INVALID_SIZE);
            assert(reply[0] == '\0' && reply[capacity] == 0x5a);
        }
        memset(reply, 0x5a, sizeof reply);
        assert(sensorarraySnapshotDiagnosticsFormat(commands[i], reply, required) == ESP_OK);
        assert(reply[required] == 0x5a);
    }
    assert(memcmp(&source, &originalSource, sizeof source) == 0);
    assert(memcmp(&protection, &originalProtection, sizeof protection) == 0);
    assert(memcmp(&transport, &originalTransport, sizeof transport) == 0);
    puts("PASS snapshot query bounds, per-layer stages, readonly counters, max u32 and truncation fence");
    return 0;
}
''')
        binary = directory / 'test'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-I', str(directory), '-I', str(ROOT / 'main/control'),
                        str(ROOT / 'main/control/sensorarraySnapshotDiagnostics.c'), str(harness),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
