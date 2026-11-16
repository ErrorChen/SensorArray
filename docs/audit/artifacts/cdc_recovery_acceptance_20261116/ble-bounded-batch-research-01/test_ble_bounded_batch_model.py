#!/usr/bin/env python3
"""Archived synthetic scheduling research, not a target acceptance test.

Creates a finite-batch candidate only in a temporary directory. Executes real
encoder/Async slot/Transport/BLE/WorkBudget functions plus explicit RTOS, SDK
and USB-service models. Preserves all 49 original serial model assertions and
its permanent-TX8 failure. Expected negative subprocesses are saved, not passed
as throughput. No production writes, target build, hardware or SDK changes.
Run from this archive with optional SENSORARRAY_MODEL_REPORT=/new/path.json.
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
def repository_root():
    configured=os.environ.get('SENSORARRAY_MODEL_ROOT')
    if configured:return Path(configured).resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent/'components/sensorarrayBle/sensorarrayBle.c').is_file():return parent
    raise RuntimeError('Set SENSORARRAY_MODEL_ROOT to the SensorArray checkout')
ROOT=repository_root()
sys.path.insert(0,str(ROOT/'tools'))
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler

MODEL = Path(__file__).with_name('test_ble_bounded_batch_model.c')


def struct(source, name):
    end = source.index('} ' + name + ';') + len('} ' + name + ';')
    start = source.rfind('typedef struct {', 0, end)
    return source[start:end]


def run_serial_adversary(temp):
    root=ROOT
    def actual_function(source,signature):return function(source,signature)
    priority_dir = temp / "priority"
    priority_dir.mkdir()
    serial_source = (root/"core/transport/sensorarraySerialIo.c").read_text()
    start = serial_source.index("#define SERIAL_IO_QUEUE_COUNT")
    end = serial_source.index("#define SERIAL_IO_VFS_PATH", start)
    constants = serial_source[start:end]
    constants += f"\n#define SERIAL_TEST_HAS_EPOCH {int('s_slotEpoch[' in serial_source)}\n"
    priority_token = re.search(r'xTaskCreatePinnedToCore\(sensorarraySerialTxTask,\s*"serialTx",\s*4096,\s*NULL,\s*(\w+)', serial_source).group(1)
    if priority_token == "txPriority":
        selection = re.search(r"unsigned txPriority = otg \? (\w+) : (\w+);", serial_source)
        assert selection, "task creation must use the once-selected backend priority"
        otg_priority = int(re.search(rf"#define {selection[1]}\s+(\d+)u?", serial_source).group(1))
        serial_priority = int(re.search(rf"#define {selection[2]}\s+(\d+)u?", serial_source).group(1))
    elif priority_token.isdigit():
        serial_priority = int(priority_token)
        otg_priority = serial_priority
    else:
        serial_priority = int(re.search(rf"#define {priority_token}\s+(\d+)u?", serial_source).group(1))
        otg_priority = serial_priority
    sdk = (root / "sdkconfig").read_text()
    def configured(name):
        return int(re.search(rf"^{name}=(\d+)$", sdk, re.M).group(1))
    transport_header = (root / "core/transport/sensorarrayTransport.h").read_text()
    transport_source = (root / "core/transport/sensorarrayTransport.c").read_text()
    def created_priority(task):
        return int(re.search(rf'xTaskCreatePinnedToCore\({task},\s*"[^"]+",\s*\w+,\s*[^,]+,\s*(\d+)u?', transport_source).group(1))
    for name in ("SENSORARRAY_TRANSPORT_TEXT_MAX", "SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX"):
        constants += re.search(rf"^#define {name}\s+\d+u$", transport_header, re.M).group() + "\n"
    for name, value in {
        "ACTUAL_SERIAL_PRIORITY": serial_priority,
        "ACTUAL_OTG_PRIORITY": otg_priority,
        "ACTUAL_OUTPUT_CORE": configured("CONFIG_SENSORARRAY_OUTPUT_TASK_CORE"),
        "ACTUAL_LOG_PRIORITY": configured("CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY"),
        "ACTUAL_BLE_PRIORITY": configured("CONFIG_SENSORARRAY_BLE_TX_TASK_PRIORITY"),
        "ACTUAL_CONTROL_PRIORITY": configured("CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY"),
        "ACTUAL_SERIAL_CONTROL_PRIORITY": created_priority("sensorarrayTransportSerialControlTask"),
        "ACTUAL_TRANSPORT_PRIORITY": created_priority("sensorarrayTransportTask"),
    }.items():
        constants += f"#define {name} {value}u\n"
    (priority_dir / "serial_priority_config.h").write_text(constants)
    # The current entry point dispatches OTG/USJ. This regression retains the
    # actual USJ task body; the OTG component has a separate owner test.
    legacy_task = "sensorarraySerialLegacyTxTask" if "static void sensorarraySerialLegacyTxTask(" in serial_source else "sensorarraySerialTxTask"
    (priority_dir / "serial_priority_source.h").write_text(
        f"#define SERIAL_TEST_LEGACY_TX {legacy_task}\n" +
        "\n".join(actual_function(serial_source, signature) for signature in (
        "static char *sensorarraySerialBytes(unsigned index)",
        "static uint8_t sensorarraySerialClaimSlot(unsigned lane)",
        "static void sensorarraySerialReleaseSlot(uint8_t index)",
        "static esp_err_t sensorarraySerialPublishSlot(uint8_t index, size_t length, unsigned lane)",
        "static esp_err_t sensorarraySerialQueueRecord(const char *data, size_t length, unsigned lane)",
        "esp_err_t sensorarraySerialIoQueueData(const char *data, size_t length)",
        "static void sensorarraySerialTxCheckpoint(void)",
        f"static void {legacy_task}(void *arg)")))
    priority_exe = priority_dir / "priority"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(priority_dir), "-I", str(root / "tests/host/work_budget_stubs"),
                    "-I", str(root / "tests/host/include"),
                    "-I", str(root / "components/sensorarrayWorkBudget/include"),
                    "-I", str(root / "components/sensorarrayWorkBudget"),
                    "-I", str(root / "core/transport"), str(Path(__file__).with_name("test_ble_bounded_batch_serial.c")),
                    "-o", str(priority_exe)], check=True)
    result=[]
    for scheduler in ('historical','idf'):
        for policy in ('baseline','permanent','batch'):
            run=subprocess.run([str(priority_exe),policy,scheduler],check=False,timeout=10,text=True,capture_output=True)
            print('SERIAL_BATCH_POLICY,'+scheduler+','+policy);print(run.stdout,end='');print(run.stderr,end='')
            result.append({'scheduler':scheduler,'policy':policy,'exit_code':run.returncode,'stdout':run.stdout,'stderr':run.stderr})
    return result


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
    protected=['components/sensorarrayBle/sensorarrayBle.c','components/sensorarrayWorkBudget/sensorarrayWorkBudget.c',
               'tests/host/test_ble_pipeline.c','tests/host/test_serial_io_priority.c','tools/test_ble_pipeline.py','tools/test_serial_io.py']
    protected_before={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in protected}
    # Every assertion from the historical fixture remains in the research
    # clone, including its negative low-priority and handoff expectations.
    def assertions(source):
        from collections import Counter
        result=[]
        for match in re.finditer(r'\bassert\s*\(',source):
            start=match.end()-1;depth=0
            for end in range(start,len(source)):
                if source[end]=='(':depth+=1
                elif source[end]==')':
                    depth-=1
                    if not depth:result.append(re.sub(r'\s+','',source[start+1:end]));break
        return Counter(result)
    old_assertions=assertions((ROOT/'tests/host/test_serial_io_priority.c').read_text())
    retained_assertions=assertions(Path(__file__).with_name('test_ble_bounded_batch_serial.c').read_text())
    assert not (old_assertions-retained_assertions), 'Historical assertions must not be removed or relaxed'

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
        print("MODEL_COMPILE,"+json.dumps(command))
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
                record['boostActiveState']=json.loads(next(line[13:] for line in result.stdout.splitlines() if line.startswith('BATCH_ACTIVE,')))
                results.append(record)
        serial_results=run_serial_adversary(temp)
        expected_negative={
            ('historical','permanent'):'sourceFrames == 200u && controlCommands == 40u',
            ('idf','baseline'):'s_stats.dataRejected > 0u || s_stats.normalExpired > 0u',
            ('idf','permanent'):'!handoffHeld && handoffObservedHeld == expectedHeld',
        }
        for row in serial_results:
            expected=expected_negative.get((row['scheduler'],row['policy']))
            row['classification']='PRESERVED_NEGATIVE_ASSERTION' if expected else 'MODEL_INVARIANTS_PASS'
            if expected:assert row['exit_code']!=0 and expected in row['stderr'],row
            else:assert row['exit_code']==0,row
        assert len(results)==45 and sum(not r['bounded'] for r in results)==6
        assert all(r['status']=='NOT_PROVABLY_BOUNDED' for r in results if not r['bounded'])
        assert all(r['boosts']==r['restores'] and not r['boostActiveState']['active'] for r in results if r['bounded'])
        assert max(r['maxBoostFragments'] for r in results)==4
        protected_after={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in protected}
        assert protected_before==protected_after,'Concurrent protected-source change; rerun against a stable snapshot'

        provenance=set(paths.values()) | set(compiled) | {'tools/test_ble_pipeline.py',
            'tests/host/ble_pipeline_stubs/common.h','components/sensorarrayWorkBudget/sensorarrayWorkBudget.c',
            'tests/host/test_serial_io_priority.c','tools/test_serial_io.py','sdkconfig'}
        report={'cases':len(results),'failures':failures,'results':results,'serial_adversary':serial_results,
            'candidate_diff':patch,'candidate_sha256':hashlib.sha256(sources['ble'].encode()).hexdigest(),
            'temporary_candidate_only':True,'production_modified':False,'hardware':'NOT_RUN',
            'source_sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(provenance)},
            'model':'Core1 real encoder/slots; Core0 actual transport/BLE owner/WorkBudget; synthetic USB bounded service and RTOS/SDK costs',
            'old_permanent8_failure':'UNCHANGED; old serial assertions/log retained',
            'target_admission':'HOLD_NOT_ADMITTED',
            'admission_reasons':['No full-message gain in modeled SDKbusy/BTsaturated pairs',
                'Individual SDK call remains unbounded and can retain priority8',
                'Historical stress-model calibration assumptions differ from actual IDF scheduling'],
            'retained_historical_assertions':sum(old_assertions.values()),
            'protected_source_unchanged':protected_before==protected_after,
            'guard_checks':['only owner','FAST DATA only','no priority/control boost','no confirm boost','non-base7 declines'],
            'new_candidate_tests':'SYNTHETIC_NATIVE_C_EXECUTED; not HIL'}
        output=os.environ.get('SENSORARRAY_MODEL_REPORT')
        if output:
            with open(output,'x') as f:json.dump(report,f,indent=2);f.write('\n')
        print('BATCH_REGRESSION,'+json.dumps({k:v for k,v in report.items() if k not in ('results','candidate_diff','source_sha256','serial_adversary')},sort_keys=True))
        if failures:raise RuntimeError(f'{len(failures)} modeled invariant failures')



if __name__ == '__main__':
    main()
