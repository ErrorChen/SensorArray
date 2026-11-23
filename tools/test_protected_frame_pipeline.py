#!/usr/bin/env python3
"""Execute actual source/transport ownership and encoder paths, synthetic BLE boundary.

No hardware, radio delivery, SDK cancellation, or throughput acceptance is modeled.
The baseline experiment uses the old actual AsyncLog bodies (pinned commit).
"""
from pathlib import Path
import hashlib
import re
import subprocess
import tempfile
import zlib
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler
from test_ble_pipeline import struct
from text_protocol import TextProtocolParser

ROOT = Path(__file__).resolve().parents[1]
BASELINE = '8a1794ce3be95ff0007698c4e4e667dcf25fbe19'  # source protection absent; never modifies the checkout
ASYNC_NAMES = [
    'static void sensorarrayAsyncLogNotifyTask(',
    'static uint8_t sensorarrayAsyncLogFindFreeSlotLocked(',
    'static uint8_t sensorarrayAsyncLogFindFreeProtectedSlotLocked(',
    'static void sensorarrayAsyncLogRemovePendingLocked(',
    'static bool sensorarrayAsyncLogDropOldestQueuedFrameLocked(',
    'static bool sensorarrayAsyncLogSupersedeProtectedLocked(',
    'static uint32_t sensorarrayAsyncLogProtectionFlagsLocked(',
    'static bool sensorarrayAsyncLogPopFrame(',
    'esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(',
]
TRANSPORT_NAMES = [
    'static void sensorarrayTransportSyncPoolStatsLocked(',
    'static bool sensorarrayTransportAllocateDescriptor(',
    'static bool sensorarrayTransportDescriptorCurrent(',
    'static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(',
    'static bool sensorarrayTransportReleaseDescriptor(',
    'static void sensorarrayTransportFillFrameMeta(',
    'static void sensorarrayTransportProtectionCount(',
    'static bool sensorarrayTransportHoldProtectedRetry(',
    'static TickType_t sensorarrayTransportWaitTicks(',
    'static void sensorarrayTransportStatResult(',
    'static void sensorarrayTransportWifiCompleted(',
            'static bool sensorarrayTransportExpireOwnedProtected(',
    'static void sensorarrayTransportProcessDescriptor(',
    'static void sensorarrayTransportExpireProtectedRetry(',
    'static void sensorarrayTransportExpireProtectedPending(',
    'static bool sensorarrayTransportReceiveNext(',
    'static bool sensorarrayTransportSupersedePendingProtected(',
    'static esp_err_t sensorarrayTransportQueueFrameInternal(',
    'static esp_err_t sensorarrayTransportQueue(',
    'esp_err_t sensorarrayTransportPublishFrame(',
    'esp_err_t sensorarrayTransportPublishFrameMetadata(',
]

def main():
    compiler, native = findCompiler()
    assert native, 'native execution required'
    async_source = (ROOT/'main/output/sensorarrayAsyncLog.c').read_text()
    transport = (ROOT/'core/transport/sensorarrayTransport.c').read_text()
    sdk = (ROOT/'sdkconfig').read_text()
    normal = int(re.search(r'^CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS=(\d+)$', sdk, re.M)[1])
    baseline = subprocess.check_output(['git', 'show', f'{BASELINE}:main/output/sensorarrayAsyncLog.c'], cwd=ROOT, text=True)
    with tempfile.TemporaryDirectory(prefix='sensorarray-protected-pipeline-') as directory:
        temp = Path(directory)
        (temp/'freertos').mkdir()
        for name in ('FreeRTOS.h','task.h','queue.h','semphr.h'):
            (temp/'freertos'/name).write_text('#pragma once\n#include "common.h"\n')
        (temp/'sdkconfig.h').write_text('#pragma once\n')
        (temp/'sensorarrayConfig.h').write_text('#pragma once\n')
        (temp/'protected_config.h').write_text(
            f'#define SENSORARRAY_ASYNC_LOG_NORMAL_FRAME_SLOT_COUNT {normal}u\n'
            '#define SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT (SENSORARRAY_ASYNC_LOG_NORMAL_FRAME_SLOT_COUNT+2u)\n'
            '#define CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES 1\n'
            '#define CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG 0\n' +
            re.search(r'^#define SENSORARRAY_ASYNC_PROTECTED_PERIOD_US .*$', async_source, re.M)[0] + '\n' +
            '\n'.join(re.findall(r'^#define SENSORARRAY_TRANSPORT_(?:DATA_QUEUE_COUNT|LOG_QUEUE_COUNT|LIFECYCLE_QUEUE_COUNT|PROTECTED_QUEUE_COUNT|PROTECTED_DEADLINE_US|PROTECTED_RETRY_US) .*$', transport, re.M)))
        (temp/'protected_types.h').write_text('\n'.join([
            struct(async_source, 'sensorarrayAsyncLogFrameSlot_t'),
            struct(async_source, 'sensorarrayAsyncLogSharedStats_t'),
            struct((ROOT/'main/output/sensorarrayAsyncLog.h').read_text(), 'sensorarrayAsyncLogProtectionStats_t')]))
        (temp/'protected_async.h').write_text('\n'.join(function(async_source, name) for name in ASYNC_NAMES))
        (temp/'protected_transport.h').write_text('\n'.join(function(transport, name) for name in TRANSPORT_NAMES))
        (temp/'protected_net.h').write_text(function((ROOT/'main/net/sensorarrayNetStatus.c').read_text(), 'esp_err_t sensorarrayNetFramePublish('))
        old_names = [ASYNC_NAMES[0], ASYNC_NAMES[1], ASYNC_NAMES[4], ASYNC_NAMES[8]]
        old = '\n'.join(function(baseline, name) for name in old_names)
        # Namespace the old actual functions, keeping shared test storage/encoder.
        for name in ('NotifyTask','FindFreeSlotLocked','DropOldestQueuedFrameLocked','PublishFrameSnapshot'):
            old = old.replace('sensorarrayAsyncLog'+name, 'baseline'+name)
        old = old.replace('SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT', 'SENSORARRAY_ASYNC_LOG_NORMAL_FRAME_SLOT_COUNT')
        (temp/'protected_baseline.h').write_text(old)
        includes = [temp, ROOT/'tests/host/ble_pipeline_stubs', ROOT/'tests/host/frame_stubs',
                    ROOT/'tests/host/include', ROOT/'main', ROOT/'main/output', ROOT/'core/measure/include',
                    ROOT/'core/config', ROOT/'core/transport', ROOT/'core/boardSupport/include',
                    ROOT/'components/ads126xAdc/include', ROOT/'components/fdc2214Cap/include',
                    ROOT/'components/tmuxSwitch/include', ROOT/'components/sensorarrayBle/include',
                    ROOT/'components/sensorarrayWifi/include']
        compiled = ['tests/host/test_protected_frame_pipeline.c','core/transport/sensorarrayTransportPool.c',
                    'core/transport/sensorarrayTransportFrameMeta.c','main/output/sensorarrayTextProtocol.c',
                    'core/measure/sensorarrayFrameBuilder.c','core/measure/sensorarrayMeasurementMode.c',
                    'core/measure/sensorarrayRowModeProfile.c']
        command = [compiler,'-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-g',
                   '-fsanitize=undefined','-fno-omit-frame-pointer',
                   '-Wl,--wrap=sensorarrayTransportParseFrameMeta', '-Wl,--wrap=strtoull']
        for path in includes: command += ['-I',str(path)]
        executable = temp/'protected'
        subprocess.run(command + [str(ROOT/p) for p in compiled] + ['-lm','-o',str(executable)],check=True)
        result = subprocess.run([str(executable)], text=True, capture_output=True, check=False, timeout=20)
        typed_frames = []
        for line in result.stdout.splitlines():
            if not line.startswith('PROTECTED_FRAME_HEX,'): print(line); continue
            _, label, encoded = line.split(',',2)
            data = bytes.fromhex(encoded)
            lines = data.splitlines(keepends=True)
            crc = re.search(rb'\bcrc=([0-9A-Fa-f]{8})',lines[-1])
            assert crc and int(crc[1],16) == zlib.crc32(b''.join(lines[:-1]))
            parser = TextProtocolParser()
            parsed = [frame for wire in data.decode('ascii').splitlines()
                      if (frame := parser.feed_line(wire)) is not None]
            assert len(parsed) == 1 and parsed[0].crc_ok and not parser.counters.malformed
            if label.startswith('typed_'):
                typed_frames.append((label, hashlib.sha256(data).hexdigest()))
                continue
            print(f'PROTECTED_ENCODER,{label},bytes={len(data)},crc=PASS,sha256={hashlib.sha256(data).hexdigest()}')
        assert len(typed_frames) == 96
        print('TYPED_ENCODER_WIRE,frames=96,tags=CVRM,rows=1..8,crc=zlib+real_parser,original_bytes_unchanged=1,synthetic_domain_mismatch=diagnostic_only,manifest_sha256='+
              hashlib.sha256(repr(typed_frames).encode()).hexdigest())
        print(result.stderr,end='')
        result.check_returncode()
    print('PROTECTED_FRAME_PIPELINE,passed=1,hardware=NOT_RUN,scope=actual_source+transport+encoder,ble=synthetic_admission_boundary,baseline='+BASELINE)

if __name__ == '__main__': main()
