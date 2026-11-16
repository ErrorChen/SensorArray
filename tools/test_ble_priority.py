#!/usr/bin/env python3
"""Execute production BLE ownership, scheduling and GAP observation bodies.

The FreeRTOS/Bluedroid boundary is synthetic; this is not an RF/HIL test.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "components/sensorarrayBle/sensorarrayBle.c").read_text()
    harness = (root / "tests/host/test_ble_priority.c").read_text()
    definitions = source[source.index("#ifndef CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN"):
                         source.index("/*\n * Statistics are observed")]
    harness = harness.replace("/* PRODUCTION_TYPES */", definitions)
    signatures = [
        "static void sensorarrayBleStatsIncrement(",
        "static void sensorarrayBleStatsNoteDrop(",
        "static void sensorarrayBleStatsNoteMessageSent(",
        "static uint32_t sensorarrayBleNextGeneration(",
        "static uint32_t sensorarrayBleCountUsedSlotsLocked(",
        "static void sensorarrayBleUpdateSlotUseLocked(",
        "static esp_err_t sensorarrayBleAllocateTxDescriptor(",
        "static uint8_t *sensorarrayBleDescriptorData(",
        "static bool sensorarrayBleDescriptorCurrent(",
        "static void sensorarrayBleReleaseTxDescriptor(",
        "static uint16_t sensorarrayBleValueHandle(",
        "static char sensorarrayBleEnvelopeChannel(",
        "static uint32_t sensorarrayBleCrc32(",
        "static void sensorarrayBleNoteDrop(",
        "static bool sensorarrayBleValidateAscii(",
        "static bool sensorarrayBleSnapshotSendContext(",
        "static bool sensorarrayBleSendContextMatches(",
        "static bool sensorarrayBleSendContextCurrent(",
        "static size_t sensorarrayBleDecimalChars(",
        "static size_t sensorarrayBleMaxPayloadSize(",
        "static bool sensorarrayBlePlanFragments(",
        "static void sensorarrayBlePrepareConfirmation(",
        "static void sensorarrayBleClearConfirmation(",
        "static esp_err_t sensorarrayBleAwaitConfirmation(",
        "static esp_err_t sensorarrayBleSendGattValue(",
        "static esp_err_t sensorarrayBleSendFragment(",
        "static esp_err_t sensorarrayBleSendPayload(",
        "static esp_err_t sensorarrayBleSendDescriptor(",
        "static bool sensorarrayBleReceiveNext(",
        "static void sensorarrayBleTxTask(",
        "static esp_err_t sensorarrayBleQueueTxDescriptor(",
        "static void sensorarrayBleDrainTxQueues(",
        "static void sensorarrayBleResetLinkObservationLocked(",
        "static void sensorarrayBleRequestLinkParameters(",
        "static bool sensorarrayBleObserveLinkEvent(",
        "static esp_err_t sensorarrayBleNotifyInternal(",
        "esp_err_t sensorarrayBleNotify(",
        "esp_err_t sensorarrayBleNotifyForGeneration(",
        "esp_err_t sensorarrayBleNotifyLifecycleForGeneration(",
        "void sensorarrayBleGetLinkSnapshot(",
    ]
    bodies = "\n\n".join(function(source, signature) for signature in signatures)
    harness = harness.replace("/* PRODUCTION_FUNCTIONS */", bodies)
    # Execute the unchanged CONF callback branch, including its real lock,
    # matching, stale-event and task-wakeup code. Avoid a test-only copy of
    # that predicate or a refactor of the firmware merely for extraction.
    conf_case = source[source.index("    case ESP_GATTS_CONF_EVT: {"):
                       source.index("    case ESP_GATTS_CONGEST_EVT:")]
    assert conf_case.count("case ESP_GATTS_CONF_EVT:") == 1
    harness = harness.replace("/* PRODUCTION_CONF_CASE */", conf_case)
    connection_cases = source[source.index("    case ESP_GATTS_CONNECT_EVT: {"):
                              source.index("    case ESP_GATTS_WRITE_EVT: {")]
    assert connection_cases.count("case ESP_GATTS_") == 3
    harness = harness.replace("/* PRODUCTION_CONNECTION_CASES */", connection_cases)
    init = source[source.index("    s_ble.txQueue = xQueueCreateStatic("):
                  source.index("    BaseType_t taskOk = xTaskCreatePinnedToCore(")]
    harness = harness.replace("/* PRODUCTION_QUEUE_INIT */", init)
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("host execution is required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-ble-priority-") as name:
        temp = Path(name)
        transport = (root / "core/transport/sensorarrayTransport.c").read_text()
        preferred_mtu = re.findall(r"\.preferredMtu\s*=\s*(\d+)u", transport)
        assert len(preferred_mtu) == 1
        reference = "#define TEST_APP_PREFERRED_MTU " + preferred_mtu[0] + "u\n"
        for name, fixture in (("referenceCapMtu", "cap_1.frame"), ("referenceVoltMtu", "volt_1.frame")):
            data = (root / "tests/fixtures/config_identity" / fixture).read_bytes()
            reference += "static const uint8_t " + name + "[] = {" + ",".join(map(str, data)) + "};\n"
        (temp / "mtu_reference.h").write_text(reference)
        (temp / "esp_err.h").write_text("""#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_CRC 0x109
""")
        generated = temp / "test.c"
        generated.write_text(harness)
        executable = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
                   "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                   "-I", str(temp), "-I", str(root / "components/sensorarrayBle/include"),
                   str(generated), "-o", str(executable)]
        subprocess.run(command, check=True)
        # This harness owns only static storage. LeakSanitizer cannot inspect
        # threads under the sandbox ptrace; retain ASan/UBSan bounds checking.
        environment = dict(os.environ, ASAN_OPTIONS="detect_leaks=0")
        wire = temp / "wire.jsonl"
        subprocess.run([str(executable), str(wire)], check=True, timeout=15, env=environment)
        # Replay exact notifications emitted by real C SendPayload/SendFragment
        # through the receiver, including raw values that no longer need G.
        from receive_ble_text import BleTextReceiver
        cases = {}
        for line in wire.read_text().splitlines():
            record = json.loads(line)
            cases.setdefault(record["case"], []).append(record)
        assert len(cases) == 16  # 12 allowed size/channel cases + 4 golden cases.
        for label, records in cases.items():
            receiver = BleTextReceiver(argparse.Namespace(tail=False, show_fragments=False), None)
            receiver._write = lambda line: None
            decoded = []
            process = receiver.process_message
            def observe(channel, payload):
                decoded.append((channel, payload))
                process(channel, payload)
            receiver.process_message = observe
            expected = [(r["channel"], bytes.fromhex(r["hex"])) for r in records if r["type"] == "expected"]
            assert len(expected) == 1
            for record in records:
                if record["type"] == "notification":
                    receiver.callback(record["channel"])(None, bytearray.fromhex(record["hex"]))
            assert decoded == expected, label
            assert receiver.parser.counters.crc_errors == 0
            if label.startswith("golden-volt"):
                assert receiver.parser.counters.voltage_frames == 1
            if label.startswith("golden-cap"):
                assert receiver.parser.counters.cap_frames == 1
        print("BLE_MTU_RECEIVER_ROUNDTRIP,actual_C_notifications=1,cases=16,hardware=NOT_RUN")
        # Exercise the actual installed SDK decision path as well as our GAP
        # observer. A successful request can be a15ms no-op in this SDK.
        idf = Path(os.environ['IDF_PATH']) if os.environ.get('IDF_PATH') else None
        description = root / 'build/project_description.json'
        if idf is None and description.exists():
            idf = Path(json.loads(description.read_text())['idf_path'])
        if idf is None:
            raise RuntimeError('actual ESP-IDF tree required for connection-request regression')
        sdk_path = idf / 'components/bt/host/bluedroid/stack/l2cap/l2c_ble.c'
        sdk = sdk_path.read_text()
        header = (idf / 'components/bt/host/bluedroid/stack/l2cap/include/l2c_int.h').read_text()
        btm = (idf / 'components/bt/host/bluedroid/stack/include/stack/btm_ble_api.h').read_text()
        constants = []
        for macro in ('L2C_BLE_CONN_UPDATE_DISABLE', 'L2C_BLE_NEW_CONN_PARAM', 'L2C_BLE_UPDATE_PENDING',
                      'L2C_BLE_NOT_DEFAULT_PARAM', 'L2C_BLE_UPDATE_PARAM_FULL'):
            constants.append(re.search(r'^#define\s+' + macro + r'\s+[^\n]+', header, re.M).group())
        for macro in ('BTM_BLE_CONN_INT_MIN', 'BTM_BLE_CONN_INT_MAX_DEF',
                      'BTM_BLE_CONN_SLAVE_LATENCY_DEF', 'BTM_BLE_CONN_TIMEOUT_DEF'):
            constants.append(re.search(r'^#define\s+' + macro + r'\s+[^\n]+', btm, re.M).group())
        for suffix, field in (('MIN', 'min_int'), ('MAX', 'max_int')):
            value = re.search(r'conn\.' + field + r'\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*;', source).group(1)
            constants.append(f'#define CURRENT_REQUEST_{suffix} {value}u')
        sdk_harness = (root / 'tests/host/test_ble_sdk_connection_request.c').read_text()
        sdk_harness = sdk_harness.replace('/* PRODUCTION_CONSTANTS */', '\n'.join(constants))
        # The first occurrence of start_conn_update is a prototype; the
        # extractor searches the implementation signature including its body.
        start = sdk.index('static BOOLEAN l2cble_start_conn_update (tL2C_LCB *p_lcb)\n{')
        sdk_harness = sdk_harness.replace('/* PRODUCTION_FUNCTIONS */',
            function(sdk[start:], 'static BOOLEAN l2cble_start_conn_update (') + '\n' +
            function(sdk, 'BOOLEAN L2CA_UpdateBleConnParams ('))
        generated = temp / 'sdk_connection.c'; generated.write_text(sdk_harness)
        executable = temp / 'sdk_connection'
        subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-g',
                        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                        str(generated), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10, env=environment)
        print('BLE_SDK_CONNECTION_SOURCE,path=' + str(sdk_path) + ',sha256=' +
              hashlib.sha256(sdk_path.read_bytes()).hexdigest())
    print(f"BLE_PRIORITY_SOURCE,production_functions={len(signatures)},hardware=NOT_RUN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
