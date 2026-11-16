#!/usr/bin/env python3
"""Execute a bounded, causal firmware pipeline under synthetic scheduling.

Real encoder, AsyncLog slot helpers, Transport pool/queue/drain and BLE owner
bodies are used. The Core0 scheduler, Core1 arrival clock and SDK costs are
explicit models, not HIL, CPU measurements or radio throughput acceptance.
The older always-ready scheduler regression is neither changed nor replaced.
"""
from pathlib import Path
import sys
import difflib
import hashlib
import json
import os
import re
import subprocess
import tempfile
sys.path.insert(0, str(Path(os.environ.get('SENSORARRAY_MODEL_ROOT', '/home/csc/esp/workspace/SensorArray')) / 'tools'))
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler

ROOT = Path(os.environ.get('SENSORARRAY_MODEL_ROOT', '/home/csc/esp/workspace/SensorArray'))
MODEL = Path(__file__).with_name('test_ble_bounded_batch_model.c')


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

        # Only the temporary candidate body changes. All baseline ownership,
        # queue, SDK and CRC functions are extracted from current production.
        original_ble=sources['ble']
        payload=function(original_ble,'static esp_err_t sensorarrayBleSendPayload(')
        candidate=payload
        edits=[
            ('    bool confirm = context.confirm;\n    for (uint32_t index',
             '    bool confirm = context.confirm;\n    bool batchBoosted = sensorarrayBleBatchBegin(channel, &context);\n    for (uint32_t index'),
            ('        if (err != ESP_OK) {\n            sensorarrayBleNoteDrop(channel);\n            return err;',
             '        if (err != ESP_OK) {\n            sensorarrayBleBatchEnd(&batchBoosted);\n            sensorarrayBleNoteDrop(channel);\n            return err;'),
            ('            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);',
             '            sensorarrayBleBatchEnd(&batchBoosted);\n            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);'),
            ('            checkpointTick = xTaskGetTickCount();',
             '            checkpointTick = xTaskGetTickCount();\n            batchBoosted = sensorarrayBleBatchBegin(channel, &context);'),
            ('    sensorarrayBleStatsNoteMessageSent(channel, false);',
             '    sensorarrayBleBatchEnd(&batchBoosted);\n    sensorarrayBleStatsNoteMessageSent(channel, false);'),
        ]
        for before,after in edits:
            assert candidate.count(before)==1,before
            candidate=candidate.replace(before,after)
        sources['ble']=original_ble.replace(payload,candidate,1)
        (temp/'candidate_ble.c').write_text(sources['ble'])
        patch=''.join(difflib.unified_diff(payload.splitlines(True),candidate.splitlines(True),fromfile='deployed/SendPayload',tofile='temporary-candidate/SendPayload'))
        (temp/'candidate.patch').write_text(patch)
        ble = sources['ble']
        (temp / 'ble_types.h').write_text(ble[ble.index('#ifndef CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN'):
                                                   ble.index('/*\n * Statistics are observed')])
        emit('ble_functions.h', 'ble', [
            'static void sensorarrayBleStatsIncrement(', 'static void sensorarrayBleStatsNoteDrop(',
            'static void sensorarrayBleStatsNoteMessageSent(', 'static uint32_t sensorarrayBleNextGeneration(',
            'static uint32_t sensorarrayBleCountUsedSlotsLocked(', 'static void sensorarrayBleUpdateSlotUseLocked(',
            'static esp_err_t sensorarrayBleAllocateTxDescriptor(', 'static uint8_t *sensorarrayBleDescriptorData(',
            'static bool sensorarrayBleDescriptorCurrent(', 'static void sensorarrayBleReleaseTxDescriptor(',
            'static uint16_t sensorarrayBleValueHandle(', 'static char sensorarrayBleEnvelopeChannel(',
            'static uint32_t sensorarrayBleCrc32(', 'static void sensorarrayBleNoteDrop(',
            'static bool sensorarrayBleValidateAscii(', 'static bool sensorarrayBleSnapshotSendContext(',
            'static bool sensorarrayBleSendContextMatches(', 'static bool sensorarrayBleSendContextCurrent(',
            'static size_t sensorarrayBleMaxPayloadSize(', 'static bool sensorarrayBlePlanFragments(',
            'static void sensorarrayBlePrepareConfirmation(', 'static void sensorarrayBleClearConfirmation(',
            'static esp_err_t sensorarrayBleAwaitConfirmation(', 'static esp_err_t sensorarrayBleSendGattValue(',
            'static esp_err_t sensorarrayBleSendFragment(', 'static esp_err_t sensorarrayBleSendPayload(',
            'static esp_err_t sensorarrayBleQueueTxDescriptor(', 'static esp_err_t sensorarrayBleNotifyInternal(',
            'esp_err_t sensorarrayBleNotify(', 'esp_err_t sensorarrayBleNotifyForGeneration(',
            'esp_err_t sensorarrayBleNotifyLifecycleForGeneration(', 'bool sensorarrayBleGetSendGeneration(',
            'bool sensorarrayBleIsCongested(',
        ])
        emit('ble_send.h', 'ble', ['static esp_err_t sensorarrayBleSendDescriptor('])
        emit('ble_owner.h', 'ble', ['static bool sensorarrayBleReceiveNext(', 'static void sensorarrayBleTxTask('])
        (temp / 'ble_init.h').write_text(ble[ble.index('    s_ble.txQueue = xQueueCreateStatic('):
                                                ble.index('    BaseType_t taskOk = xTaskCreatePinnedToCore(')])
        tr = sources['transport']
        constants = '\n'.join(re.findall(r'^#define SENSORARRAY_TRANSPORT_(?:DATA_QUEUE_COUNT|LOG_QUEUE_COUNT|LIFECYCLE_QUEUE_COUNT|DRAIN_BATCH_COUNT) .*$', tr, re.M))
        (temp / 'transport_constants.h').write_text(constants)
        emit('transport_helpers.h', 'transport', [
            'static void sensorarrayTransportSyncPoolStatsLocked(', 'static bool sensorarrayTransportAllocateDescriptor(',
            'static bool sensorarrayTransportDescriptorCurrent(',
            'static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(',
            'static bool sensorarrayTransportReleaseDescriptor(', 'static void sensorarrayTransportFillFrameMeta(',
            'static void sensorarrayTransportStatResult(', 'static void sensorarrayTransportNotifyTask(',
            'static esp_err_t sensorarrayTransportQueue(',
        ])
        emit('transport_process.h', 'transport', ['static void sensorarrayTransportProcessDescriptor('])
        emit('transport_owner.h', 'transport', ['static bool sensorarrayTransportReceiveNext(', 'static void sensorarrayTransportTask('])
        async_source = sources['async']
        (temp / 'async_types.h').write_text(struct(async_source, 'sensorarrayAsyncLogFrameSlot_t') + '\n' +
                                          struct(async_source, 'sensorarrayAsyncLogSharedStats_t'))
        emit('async_functions.h', 'async', [
            'static void sensorarrayAsyncLogNotifyTask(', 'static uint8_t sensorarrayAsyncLogFindFreeSlotLocked(',
            'static bool sensorarrayAsyncLogDropOldestQueuedFrameLocked(', 'static bool sensorarrayAsyncLogPopFrame(',
            'esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(',
        ])
        sdk = (ROOT / 'sdkconfig').read_text()
        values = {name: int(re.search(r'^' + name + r'=(\d+)$', sdk, re.M)[1]) for name in (
            'CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS', 'CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY',
            'CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE', 'CONFIG_SENSORARRAY_SCAN_TASK_CORE',
            'CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN', 'CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY')}
        header = ''.join(f'#define {key} {value}u\n' for key, value in values.items())
        header += '#define SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS\n'
        header += '#define CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES 1\n'
        priority = re.search(r'xTaskCreatePinnedToCore\(sensorarrayTransportTask,\s*"[^"\n]+",\s*\w+,\s*[^,]+,\s*(\d+)u?', tr)[1]
        header += f'#define PIPELINE_TRANSPORT_PRIORITY {priority}u\n'
        serial=(ROOT/'core/transport/sensorarraySerialIo.c').read_text()
        usb_priority=re.search(r'#define SERIAL_IO_OTG_TX_TASK_PRIORITY (\d+)u',serial)[1]
        header += f'#define PIPELINE_USB_PRIORITY {usb_priority}u\n'
        (temp / 'pipeline_config.h').write_text(header)
        includes = [temp, ROOT / 'tests/host/ble_pipeline_stubs', ROOT / 'tests/host/work_budget_stubs',
                    ROOT / 'tests/host/frame_stubs', ROOT / 'tests/host/include', ROOT / 'main',
                    ROOT / 'main/output', ROOT / 'core/measure/include', ROOT / 'core/config',
                    ROOT / 'core/transport', ROOT / 'core/boardSupport/include',
                    ROOT / 'components/ads126xAdc/include', ROOT / 'components/fdc2214Cap/include',
                    ROOT / 'components/tmuxSwitch/include', ROOT / 'components/sensorarrayBle/include',
                    ROOT / 'components/sensorarrayWifi/include', ROOT / 'components/sensorarrayWorkBudget/include',
                    ROOT / 'components/sensorarrayWorkBudget']
        compiled = [str(MODEL), 'core/transport/sensorarrayTransportPool.c',
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
        cases=('burst','short','bt_preempt','long_wait','reject','unread','control_flow',
               'sdk_busy','bt_saturated','never_returns','epoch_change','reject_middle',
               'never_ready','usb_nohost','large_frame')
        results=[];failures=[]
        for case in cases:
            for policy in ('baseline','permanent','batch'):
                result=subprocess.run([str(executable),case,policy],timeout=20,check=False,text=True,capture_output=True)
                print(result.stdout,end='');print(result.stderr,end='')
                if result.returncode:
                    failures.append({'case':case,'policy':policy,'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr})
                    continue
                record=json.loads(next(line[16:] for line in result.stdout.splitlines() if line.startswith('PIPELINE_RESULT,')))
                record.update(json.loads(next(line[13:] for line in result.stdout.splitlines() if line.startswith('BATCH_DETAIL,'))))
                results.append(record)
        provenance=set(paths.values()) | set(compiled) | {'tools/test_ble_pipeline.py',
            'tests/host/ble_pipeline_stubs/common.h','components/sensorarrayWorkBudget/sensorarrayWorkBudget.c',
            'tests/host/test_serial_io_priority.c','tools/test_serial_io.py','sdkconfig'}
        report={'cases':len(results),'failures':failures,'results':results,
            'candidate_diff':patch,'candidate_sha256':hashlib.sha256(sources['ble'].encode()).hexdigest(),
            'temporary_candidate_only':True,'production_modified':False,'hardware':'NOT_RUN',
            'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(provenance)},
            'model':'Core1 real encoder/slots; Core0 actual transport/BLE owner/WorkBudget; synthetic USB bounded service and RTOS/SDK costs',
            'old_permanent8_failure':'UNCHANGED; old serial assertions/log retained',
            'target_admission':'NOT_ESTABLISHED'}
        output=os.environ.get('SENSORARRAY_MODEL_REPORT')
        if output:
            with open(output,'x') as f:json.dump(report,f,indent=2);f.write('\n')
        print('BATCH_REGRESSION,'+json.dumps({k:v for k,v in report.items() if k not in ('results','candidate_diff','source_sha256')},sort_keys=True))
        if failures:raise RuntimeError(f'{len(failures)} modeled invariant failures')



if __name__ == '__main__':
    main()
