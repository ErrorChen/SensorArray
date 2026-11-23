#!/usr/bin/env python3
"""Execute a bounded, causal firmware pipeline under synthetic scheduling.

Real encoder, AsyncLog slot helpers, Transport pool/queue/drain and BLE owner
bodies are used. The Core0 scheduler, Core1 arrival clock and SDK costs are
explicit models, not HIL, CPU measurements or radio throughput acceptance.
The older always-ready scheduler regression is neither changed nor replaced.
This baseline uses generic DATA admission. test_protected_frame_pipeline executes
new typed source/transport protection independently with a BLE admission stub.
"""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import tempfile
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler

ROOT = Path(__file__).resolve().parents[1]


def struct(source, name):
    end = source.index('} ' + name + ';') + len('} ' + name + ';')
    start = source.rfind('typedef struct {', 0, end)
    return source[start:end]


def main():
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError('host execution is required; compile-only is not PASS')
    paths = {
        'ble': 'components/sensorarrayBle/sensorarrayBle.c',
        'transport': 'core/transport/sensorarrayTransport.c',
        'async': 'main/output/sensorarrayAsyncLog.c',
    }
    sources = {key: (ROOT / path).read_text() for key, path in paths.items()}
    selected = {}
    with tempfile.TemporaryDirectory(prefix='sensorarray-ble-pipeline-') as name:
        temp = Path(name)
        (temp / 'freertos').mkdir()
        for name in ('FreeRTOS.h', 'task.h', 'queue.h', 'semphr.h'):
            (temp / 'freertos' / name).write_text('#pragma once\n#include "common.h"\n')
        (temp / 'sdkconfig.h').write_text('#pragma once\n')
        (temp / 'sensorarrayConfig.h').write_text('#pragma once\n')
        (temp / 'esp_freertos_hooks.h').write_text(
            '#pragma once\n#include "common.h"\n'
            'typedef bool (*esp_freertos_idle_cb_t)(void);\n'
            'esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu);\n')

        def emit(name, key, signatures):
            selected[name] = signatures
            (temp / name).write_text('\n\n'.join(function(sources[key], s) for s in signatures))

        ble = sources['ble']
        (temp / 'ble_types.h').write_text('#include "sensorarrayBleOutputSchedule.h"\n' + ble[ble.index('#ifndef CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN'):
                                                   ble.index('/*\n * Statistics are observed')] + '\n' +
            re.search(r'^static esp_err_t s_bleInitResult = [^\n]+', ble, re.M)[0])
        emit('ble_functions.h', 'ble', [
            'static esp_err_t sensorarrayBleInitPayloadStorage(',
            'static esp_err_t sensorarrayBleFinishInit(',
            'static void sensorarrayBleStatsIncrement(', 'static void sensorarrayBleStatsNoteDrop(',
            'static void sensorarrayBleStatsNoteMessageSent(', 'static uint32_t sensorarrayBleNextGeneration(',
            'static uint32_t sensorarrayBleCountUsedSlotsLocked(', 'static void sensorarrayBleUpdateSlotUseLocked(',
            'static bool sensorarrayBleDataActiveLocked(',
            'static bool sensorarrayBleFrameSameBoot(', 'static bool sensorarrayBleFrameSameConfig(',
            'static bool sensorarrayBleFrameSequenceBefore(', 'static void sensorarrayBleFrameFlowSyncLocked(',
            'static void sensorarrayBleFrameFlowObserveLocked(', 'static bool sensorarrayBleFrameProtectedRepresentedLocked(',
            'static void sensorarrayBleFramePeriodicLocked(',
            'static void sensorarrayBleFrameFullCompletedLocked(',
            'static esp_err_t sensorarrayBleAllocateTxDescriptor(', 'static uint8_t *sensorarrayBleDescriptorData(',
            'static bool sensorarrayBleDescriptorCurrent(', 'static void sensorarrayBleReleaseTxDescriptor(',
            'static uint16_t sensorarrayBleValueHandle(', 'static char sensorarrayBleEnvelopeChannel(',
            'static uint32_t sensorarrayBleCrc32(', 'static void sensorarrayBleNoteDrop(',
            'static bool sensorarrayBleValidateAscii(', 'static bool sensorarrayBleSnapshotSendContext(',
            'static bool sensorarrayBleSendContextMatches(', 'static bool sensorarrayBleSendContextCurrent(',
            'static size_t sensorarrayBleDecimalChars(',
            'static size_t sensorarrayBleMaxPayloadSize(', 'static bool sensorarrayBlePlanFragments(',
            'static size_t sensorarrayBleTwoFragmentFirstChunk(',
            'static void sensorarrayBlePrepareConfirmation(', 'static void sensorarrayBleClearConfirmation(',
            'static esp_err_t sensorarrayBleAwaitConfirmation(',
            'static void sensorarrayBleObserveSendDeadlineLocked(',
            'void sensorarrayBleObserveSendDeadline(',
            'static esp_err_t sensorarrayBleSendGattValue(',
            'static esp_err_t sensorarrayBleSendFragment(', 'static esp_err_t sensorarrayBleSendPayload(',
            'static esp_err_t sensorarrayBleQueueTxDescriptor(', 'static esp_err_t sensorarrayBleNotifyInternal(',
            'esp_err_t sensorarrayBleNotify(', 'esp_err_t sensorarrayBleNotifyForGeneration(',
            'esp_err_t sensorarrayBleNotifyFrameForGeneration(',
            'esp_err_t sensorarrayBleNotifyLifecycleForGeneration(', 'bool sensorarrayBleGetSendGeneration(',
            'bool sensorarrayBleIsCongested(',
        ])
        emit('ble_send.h', 'ble', ['static esp_err_t sensorarrayBleSendDescriptor('])
        emit('ble_owner.h', 'ble', ['static bool sensorarrayBleReceiveProtected(', 'static bool sensorarrayBleReceiveNext(', 'static void sensorarrayBleTxTask('])
        (temp / 'ble_init.h').write_text(ble[ble.index('    s_ble.txQueue = xQueueCreateStatic('):
                                                ble.index('    BaseType_t taskOk = xTaskCreatePinnedToCore(')])
        tr = sources['transport']
        constants = '\n'.join(re.findall(r'^#define SENSORARRAY_TRANSPORT_(?:DATA_QUEUE_COUNT|LOG_QUEUE_COUNT|LIFECYCLE_QUEUE_COUNT|DRAIN_BATCH_COUNT|PROTECTED_QUEUE_COUNT|PROTECTED_DEADLINE_US|PROTECTED_RETRY_US) .*$', tr, re.M))
        (temp / 'transport_constants.h').write_text(constants)
        emit('transport_helpers.h', 'transport', [
            'static void sensorarrayTransportSyncPoolStatsLocked(', 'static bool sensorarrayTransportAllocateDescriptor(',
            'static bool sensorarrayTransportDescriptorCurrent(',
            'static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(',
            'static bool sensorarrayTransportReleaseDescriptor(', 'static void sensorarrayTransportFillFrameMeta(',
            'static void sensorarrayTransportStatResult(',
            'static bool sensorarrayTransportExpireOwnedProtected(', 'static void sensorarrayTransportNotifyTask(',
            'static void sensorarrayTransportProtectionCount(',
            'static void sensorarrayTransportExpireProtectedRetry(',
            'static void sensorarrayTransportExpireProtectedPending(',
            'static bool sensorarrayTransportHoldProtectedRetry(',
            'static TickType_t sensorarrayTransportWaitTicks(',
            'static bool sensorarrayTransportSupersedePendingProtected(',
            'static esp_err_t sensorarrayTransportQueueFrameInternal(',
            'static esp_err_t sensorarrayTransportQueue(',
        ])
        emit('transport_process.h', 'transport', ['static void sensorarrayTransportProcessDescriptor('])
        emit('transport_owner.h', 'transport', ['static bool sensorarrayTransportReceiveNext(', 'static void sensorarrayTransportTask('])
        async_source = sources['async']
        (temp / 'async_types.h').write_text(struct(async_source, 'sensorarrayAsyncLogFrameSlot_t') + '\n' +
                                          struct(async_source, 'sensorarrayAsyncLogSharedStats_t') + '\n' +
                                          struct((ROOT / 'main/output/sensorarrayAsyncLog.h').read_text(), 'sensorarrayAsyncLogProtectionStats_t'))
        emit('async_functions.h', 'async', [
            'static void sensorarrayAsyncLogNotifyTask(', 'static uint8_t sensorarrayAsyncLogFindFreeSlotLocked(',
            'static uint8_t sensorarrayAsyncLogFindFreeProtectedSlotLocked(',
            'static void sensorarrayAsyncLogRemovePendingLocked(',
            'static bool sensorarrayAsyncLogDropOldestQueuedFrameLocked(',
            'static bool sensorarrayAsyncLogSupersedeProtectedLocked(',
            'static uint32_t sensorarrayAsyncLogProtectionFlagsLocked(',
            'static bool sensorarrayAsyncLogPopFrame(',
            'esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(',
        ])
        sdk = (ROOT / 'sdkconfig').read_text()
        values = {name: int(re.search(r'^' + name + r'=(\d+)$', sdk, re.M)[1]) for name in (
            'CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS', 'CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY',
            'CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE', 'CONFIG_SENSORARRAY_SCAN_TASK_CORE',
            'CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN', 'CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY')}
        header = ''.join(f'#define {key} {value}u\n' for key, value in values.items())
        header += '#define SENSORARRAY_ASYNC_LOG_NORMAL_FRAME_SLOT_COUNT CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS\n'
        header += '#define SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT (SENSORARRAY_ASYNC_LOG_NORMAL_FRAME_SLOT_COUNT + 2u)\n'
        header += '#define SENSORARRAY_ASYNC_PROTECTED_PERIOD_US INT64_C(1000000)\n'
        header += '#define CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG 0\n'
        header += '#define CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES 1\n'
        priority = re.search(r'xTaskCreatePinnedToCore\(sensorarrayTransportTask,\s*"[^"\n]+",\s*\w+,\s*[^,]+,\s*(\d+)u?', tr)[1]
        header += f'#define PIPELINE_TRANSPORT_PRIORITY {priority}u\n'
        (temp / 'pipeline_config.h').write_text(header)
        includes = [temp, ROOT / 'tests/host/ble_pipeline_stubs', ROOT / 'tests/host/work_budget_stubs',
                    ROOT / 'tests/host/frame_stubs', ROOT / 'tests/host/include', ROOT / 'main',
                    ROOT / 'main/output', ROOT / 'core/measure/include', ROOT / 'core/config',
                    ROOT / 'core/transport', ROOT / 'core/boardSupport/include',
                    ROOT / 'components/ads126xAdc/include', ROOT / 'components/fdc2214Cap/include',
                    ROOT / 'components/tmuxSwitch/include', ROOT / 'components/sensorarrayBle/include',
                    ROOT / 'components/sensorarrayBle',
                    ROOT / 'components/sensorarrayWifi/include', ROOT / 'components/sensorarrayWorkBudget/include',
                    ROOT / 'components/sensorarrayWorkBudget']
        compiled = ['tests/host/test_ble_pipeline.c', 'components/sensorarrayBle/sensorarrayBleOutputSchedule.c', 'core/transport/sensorarrayTransportPool.c',
                    'core/transport/sensorarrayTransportFrameMeta.c', 'main/output/sensorarrayTextProtocol.c',
                    'core/measure/sensorarrayFrameBuilder.c', 'core/measure/sensorarrayMeasurementMode.c',
                    'core/measure/sensorarrayRowModeProfile.c']
        executable = temp / 'pipeline'
        command = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-g',
                   '-fsanitize=undefined', '-fno-omit-frame-pointer', '-ffunction-sections', '-fdata-sections']
        for path in includes:
            command += ['-I', str(path)]
        command += [str(ROOT / p) for p in compiled] + ['-Wl,--gc-sections', '-lm', '-o', str(executable)]
        subprocess.run(command, check=True)
        cases = ('burst', 'short', 'bt_preempt', 'long_wait', 'reject', 'unread', 'control_flow',
                 'sdk_busy', 'bt_saturated', 'never_returns', 'guarded_never_returns')
        results = []
        for case in cases:
            for priority in (7, 8):
                result = subprocess.run([str(executable), case, str(priority)], timeout=20,
                                        check=False, text=True, capture_output=True)
                print(result.stdout, end=''); print(result.stderr, end='')
                result.check_returncode()
                record = json.loads(next(line[16:] for line in result.stdout.splitlines()
                                         if line.startswith('PIPELINE_RESULT,')))
                results.append(record)
        assert len(results) == 22
        assert sum(r['bounded'] is False for r in results) == 4
        assert all(r['status'] == 'NOT_PROVABLY_BOUNDED' for r in results if r['scenario'] == 'never_returns')
        assert all(r['status'] == 'QUARANTINED_INFLIGHT_NOT_CANCELLED' and r['guardQuarantined'] == 1
                   for r in results if r['scenario'] == 'guarded_never_returns')
        # Backpressure is an observed performance failure, even when ownership
        # and bounded local-control invariants pass. Never promote this model
        # verdict into candidate throughput/HIL acceptance.
        for row in results:
            if row['scenario'] in ('sdk_busy', 'bt_saturated'):
                assert row['bleYields'] > 0 and row['bleSdkComplete'] < row['source']
        provenance = set(paths.values()) | set(compiled) | {
            'tools/test_ble_pipeline.py', 'tests/host/ble_pipeline_stubs/common.h',
            'components/sensorarrayWorkBudget/sensorarrayWorkBudget.c',
            'components/sensorarrayWorkBudget/include/sensorarrayWorkBudget.h',
            'tests/host/test_serial_io_priority.c', 'tools/test_serial_io.py', 'sdkconfig'}
        print('BLE_PIPELINE_REGRESSION,' + json.dumps({
            'scenarios':len(results), 'finite_cases':18, 'unbounded_counterexamples':2,
            'quarantined_nonreturning_owner_cases':2,
            'hardware':'NOT_RUN', 'actual_extracted_functions':sum(len(x) for x in selected.values()),
            'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(provenance)},
            'old_always_ready_regression':'UNCHANGED; its TX8 FAIL is not replaced by this test',
            'typed_protection':'separate test_protected_frame_pipeline; this scheduler bridge remains generic DATA',
            'performance_acceptance':'NOT_ESTABLISHED; finite saturated cases expose counted performance degradation',
            'models':'Core1 arrivals independent; actual encoder/slot/descriptor/owner bodies; output hub cost bridge and RTOS/SDK are synthetic',
        }, sort_keys=True))


if __name__ == '__main__':
    main()
