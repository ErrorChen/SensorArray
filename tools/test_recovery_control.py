#!/usr/bin/env python3
"""Execute real recovery admission/cache and main recovery branches with synthetic hardware."""
from pathlib import Path
import os
import hashlib
import re
from test_ble_pipeline import struct
import subprocess
import tempfile
from test_main_config_commit import function

ROOT = Path(__file__).resolve().parents[1]
s = (ROOT / 'main/main.c').read_text()
names = ['static bool sensorarrayRecoveryPost(', 'static bool sensorarrayRecoveryTake(',
         'static bool sensorarrayRestartPost(', 'static bool sensorarrayRestartTake(',
         'static void sensorarrayCompleteRecoveryRequest(', 'static esp_err_t sensorarrayStopAfterRecoveryFailure(',
         'static esp_err_t sensorarrayRunRecoveryLevel(', 'static void sensorarrayPerformManualRestart(',
         'static bool sensorarrayRunRecoveryBoundary(',
         'static esp_err_t sensorarrayFormatSerialReply(', 'static esp_err_t sensorarrayFormatSchedulerReply(', 'static void sensorarrayControlReplyPublished(',
         'static void sensorarrayControlReplyFailed(']
functions = {name: function(s, name) for name in names}
publish = function(s[s.index('/* 本轮新增: only the acquisition owner publishes'):],
                   'static sensorarrayConfigIdentity_t sensorarrayPublishConfigAuthority(')
frame = function(s, 'static esp_err_t sensorarrayRunOneFrame(')
frame = frame[:frame.index('    uint32_t frameSequence =')] + '    return ESP_OK;\n}'
frame = frame.replace('sensorarrayRunOneFrame(', 'sensorarrayTestActualFrameAdmission(')
harness = (ROOT / 'tests/host/test_recovery_control.c').read_text()
harness = harness.replace('/* PRODUCTION_PUBLISH */', publish)
dispatch = []
for actual, test in [('sensorarrayEarlyRuntimeQueryCommand', 'sensorarrayTestEarlyQuery'),
                     ('sensorarrayRuntimeQueryCommand', 'sensorarrayTestRuntimeQuery')]:
    prefix = function(s, 'static esp_err_t ' + actual + '(')
    prefix = prefix[:prefix.index('    if (strcmp(command, "SPI?")')]
    prefix = prefix.replace(actual + '(', test + '(')
    if actual == 'sensorarrayRuntimeQueryCommand':
        prefix += '    (void)frameSequence;\n'
    dispatch.append(prefix + '    return ESP_ERR_NOT_SUPPORTED;\n}')
runtime_restart = function(function(s, 'static esp_err_t sensorarrayRuntimeQueryCommand('),
                           '    if (strcmp(command, "RESTART") == 0)')
early_restart = function(s, '    case SENSORARRAY_EARLY_KIND_RESTART:')
dispatch.append('static esp_err_t sensorarrayTestRestartCommand(const char *command, char *response, size_t responseSize) {\n' +
                runtime_restart + '\nreturn ESP_ERR_NOT_SUPPORTED;\n}')
dispatch.append('static esp_err_t sensorarrayTestEarlyRestartCommand(char *response, size_t responseSize) {\n' +
                early_restart.replace('case SENSORARRAY_EARLY_KIND_RESTART:', '') + '\n}')
harness = harness.replace('/* PRODUCTION_FUNCTIONS */', '\n\n'.join(functions.values()) + '\n\n' + frame + '\n\n' + '\n\n'.join(dispatch))
# Real production implementations, not handwritten replicas of state decisions.
includes = '\n'.join('#include "' + str(ROOT / path) + '"' for path in [
    'core/measure/sensorarrayMeasurementMode.c', 'core/measure/sensorarrayRowModeProfile.c',
    'core/measure/sensorarrayConfigIdentity.c', 'core/transport/sensorarrayTransportGuaranteedText.c',
    'core/transport/sensorarrayTransportPool.c',
    'core/transport/sensorarrayTransportFrameMeta.c',
    'main/control/sensorarrayRecoveryControl.c', 'main/control/sensorarrayRestartControl.c',
    'components/sensorarrayWorkBudget/sensorarrayCpuOwner.c'])
harness = harness.replace('/* PRODUCTION_MODULES */', includes)
transport_source = (ROOT / 'core/transport/sensorarrayTransport.c').read_text()
transport_functions = '\n\n'.join(function(transport_source, name) for name in [
    'static void sensorarrayTransportSyncPoolStatsLocked(',
    'static bool sensorarrayTransportAllocateDescriptor(',
    'static bool sensorarrayTransportDescriptorCurrent(',
    'static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(',
    'static bool sensorarrayTransportReleaseDescriptor(',
    'static void sensorarrayTransportProtectionCount(',
    'static bool sensorarrayTransportSupersedePendingProtected(',
    'static esp_err_t sensorarrayTransportQueueFrameInternal(',
    'static esp_err_t sensorarrayTransportQueue(',
    'esp_err_t sensorarrayTransportPublishLifecycleObserved(',
    'static void sensorarrayTransportDrainGuaranteedText('])
harness = harness.replace('/* PRODUCTION_TRANSPORT */', transport_functions)
harness = harness.replace('/* PRODUCTION_TRANSPORT_CONSTANTS */', re.search(r'^#define SENSORARRAY_TRANSPORT_PROTECTED_DEADLINE_US .*$', transport_source, re.M)[0])

with tempfile.TemporaryDirectory(prefix='sensorarray-recovery-control-') as temporary:
    tmp = Path(temporary); fake = tmp / 'fake'; (fake / 'freertos').mkdir(parents=True)
    # Link the real default-OFF diagnostic implementation. This isolated
    # harness must not inherit the target's optional diagnostic-image config.
    (fake / 'sdkconfig.h').write_text('#pragma once\n#define CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS 0\n')
    (fake / 'freertos/FreeRTOS.h').write_text('''#pragma once
#include <pthread.h>
#include <stdint.h>
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) pthread_mutex_lock(p)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(p)
''')
    (fake / 'sensorarrayTransport.h').write_text('''#pragma once
#include <stddef.h>
#include "esp_err.h"
#include <stdbool.h>
#define SENSORARRAY_TRANSPORT_TEXT_MAX 2304u
esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *data, size_t length, bool *outQueued);
''' + '\n'.join(struct((ROOT/'core/transport/sensorarrayTransport.h').read_text(), name) for name in ('sensorarrayTransportStats_t','sensorarrayTransportProtectionStats_t')))
    unit = tmp / 'unit.c'; unit.write_text(harness)
    exe = tmp / 'test'
    cmd = [os.environ.get('CC', '/usr/bin/gcc'), '-std=c11', '-D_XOPEN_SOURCE=700', '-Wall', '-Wextra', '-Werror', '-pthread']
    for inc in [fake, ROOT/'tests/host/include', ROOT/'core/measure/include', ROOT/'core/config',
                ROOT/'core/transport', ROOT/'main/control', ROOT/'components/sensorarrayWorkBudget/include', ROOT/'components/sensorarrayWifi/include']:
        cmd += ['-I', str(inc)]
    subprocess.run(cmd + [str(unit), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

print('RECOVERY_CONTROL_SOURCE_SHA,transport=' + hashlib.sha256(transport_source.encode()).hexdigest() + ',fixture=' + hashlib.sha256((ROOT/'tests/host/test_recovery_control.c').read_bytes()).hexdigest())
