#!/usr/bin/env python3
"""Execute production BLE ownership, scheduling and GAP observation bodies.

The FreeRTOS/Bluedroid boundary is synthetic; this is not an RF/HIL test.
"""
from pathlib import Path
import os
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
    init = source[source.index("    s_ble.txQueue = xQueueCreateStatic("):
                  source.index("    BaseType_t taskOk = xTaskCreatePinnedToCore(")]
    harness = harness.replace("/* PRODUCTION_QUEUE_INIT */", init)
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("host execution is required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-ble-priority-") as name:
        temp = Path(name)
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
        subprocess.run([str(executable)], check=True, timeout=15, env=environment)
    print(f"BLE_PRIORITY_SOURCE,production_functions={len(signatures)},hardware=NOT_RUN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
