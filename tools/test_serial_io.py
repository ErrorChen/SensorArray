#!/usr/bin/env python3
"""Execute actual SerialIo queue/TX/VFS code with synthetic driver results."""
from pathlib import Path
import argparse
import json
import os
import re
import subprocess
import tempfile
import zlib
root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--baseline", help="Also reproduce VFS/64-byte corruption using this unmodified git revision")
parser.add_argument("--fragment-baseline", help="Also reproduce printf fragment/priority concatenation using this unmodified git revision")
parser.add_argument("--tx-baseline", help="Also reproduce 20ms chunk expiry using this unmodified git revision")
parser.add_argument("--admission-baseline", help="Reproduce diagnostic FIFO saturation rejecting DATA at this unmodified revision")
parser.add_argument("--source", type=Path, help="Run all current production regressions against an explicitly saved SerialIo source")
args = parser.parse_args()
fixture = root / "tests/fixtures/serial_io"
captured_config = (fixture / "config_complete.rx.txt").read_bytes()
captured_corruption = (fixture / "config_interleaving.rx.txt").read_bytes()
assert captured_corruption.index(b"PR,") == 64
captured_pr = captured_corruption[64:captured_corruption.index(b"\n", 64) + 1]
assert captured_config.startswith(b"CONFIG,") and len(captured_config) > 64
captured_fragment = (fixture / "stdio_fragment_interleaving.rx.txt").read_bytes()
reply_offset = captured_fragment.index(b"USBSTREAM,")
assert reply_offset == 128
captured_ads_prefix = captured_fragment[:reply_offset]
captured_usb_reply = captured_fragment[reply_offset:]
encoded_frame = (root / "tests/fixtures/config_identity/cap_8.frame").read_bytes()
trailer_offset = encoded_frame.index(b"K,")
expected_crc = int(re.search(rb",crc=([0-9A-F]{8})", encoded_frame[trailer_offset:]).group(1), 16)
assert zlib.crc32(encoded_frame[:trailer_offset]) & 0xFFFFFFFF == expected_crc
# Use the project's actual installed IDF validator when available. The fallback
# is the same public 15-byte constraint so ordinary host CI needs no SDK install.
idf = Path(os.environ["IDF_PATH"]) if os.environ.get("IDF_PATH") else None
description = root / "build/project_description.json"
if idf is None and description.exists():
    idf = Path(json.loads(description.read_text())["idf_path"])
validator = '''static bool is_path_prefix_valid(const char *path, size_t length) {
    return (length >= 2) && (length <= ESP_VFS_PATH_MAX)
        && (path[0] == '/') && (path[length - 1] != '/');
}'''
path_max = 15
validator_source = "public-idf-contract"
reent_init = '''void esp_reent_init(struct _reent* r)
{
    memset(r, 0, sizeof(*r));
    _REENT_STDIN(r) = _REENT_STDIN(_GLOBAL_REENT);
    _REENT_STDOUT(r) = _REENT_STDOUT(_GLOBAL_REENT);
    _REENT_STDERR(r) = _REENT_STDERR(_GLOBAL_REENT);
    _REENT_CLEANUP(r) = _REENT_CLEANUP(_GLOBAL_REENT);
    _REENT_SDIDINIT(r) = _REENT_SDIDINIT(_GLOBAL_REENT);
}'''
if idf and (idf / "components/vfs/vfs.c").exists():
    source = (idf / "components/vfs/vfs.c").read_text()
    begin = source.index("static bool is_path_prefix_valid(")
    end = source.index("\n}", begin) + 2
    validator = source[begin:end]
    header = (idf / "components/vfs/include/esp_vfs.h").read_text()
    path_max = int(re.search(r"#define ESP_VFS_PATH_MAX\s+(\d+)", header).group(1))
    validator_source = str(idf / "components/vfs/vfs.c")
    reent_source = (idf / "components/newlib/src/reent_init.c").read_text()
    begin = reent_source.index("void esp_reent_init(")
    reent_init = reent_source[begin:reent_source.index("\n}", begin) + 2]
print(f"SERIAL_IO_VALIDATOR,source={validator_source},pathMax={path_max}", flush=True)
def actual_function(source, signature):
    begin = source.index(signature + "\n{")
    return source[begin:source.index("\n}", begin) + 2]

with tempfile.TemporaryDirectory(prefix="sensorarray-serial-io-") as temp:
    temp = Path(temp)
    # Generate the exact current seven-record fault burst from the production
    # formatter. Values are synthetic; this is not a reconstructed field log.
    fault_exe = temp / "fault_fixture"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "tests/host/include"), "-I", str(root / "core/measure/ads"),
                    str(root / "tests/host/emit_ads_fault_fixture.c"),
                    str(root / "core/measure/ads/sensorarrayAdsFault.c"), "-o", str(fault_exe)], check=True)
    fault_output = subprocess.check_output([str(fault_exe)], text=True)
    fault_body = next(part for part in fault_output.split("# fixture=")[1:] if part.startswith("POWER_COMPARE;"))
    fault_lines = fault_body.splitlines(keepends=True)[1:]
    assert len(fault_lines) == 7
    assert [line.split(",", 1)[0] for line in fault_lines[:4]] == ["ADSFAULT", "ADSFAULTRAIL", "ADSFAULTCTX", "ADSFAULTREG"]
    (temp / "serial_fault_burst.h").write_text(
        "/* Synthetic values, emitted by the actual current C fault formatter. */\n"
        + "static const char faultBurst[] = " + json.dumps("".join(fault_lines)) + ";\n"
        + "#define FAULT_FIRST_FOUR_BYTES " + str(sum(len(line.encode("ascii")) for line in fault_lines[:4])) + "u\n")
    print("ADS_FAULT_BURST_SOURCE,records=7,producer=actual_C_formatter,fixture=synthetic", flush=True)
    if idf and (idf / "components/esp_ringbuf/ringbuf.c").exists():
        ring = (idf / "components/esp_ringbuf/ringbuf.c").read_text()
        usb = (idf / "components/esp_driver_usb_serial_jtag/src/usb_serial_jtag.c").read_text()
        (temp / "idf_ring_bytebuf.h").write_text("\n".join(actual_function(ring, signature) for signature in (
            "static BaseType_t prvCheckItemFitsByteBuffer(Ringbuffer_t *pxRingbuffer, size_t xItemSize)",
            "static void prvCopyItemByteBuf(Ringbuffer_t *pxRingbuffer, const uint8_t *pucItem, size_t xItemSize)",
            "static size_t prvGetCurMaxSizeByteBuf(Ringbuffer_t *pxRingbuffer)")))
        (temp / "idf_ring_send.h").write_text(actual_function(ring,
            "BaseType_t xRingbufferSend(RingbufHandle_t xRingbuffer,\n                           const void *pvItem,\n                           size_t xItemSize,\n                           TickType_t xTicksToWait)"))
        (temp / "idf_usb_write.h").write_text(actual_function(usb,
            "int usb_serial_jtag_write_bytes(const void* src, size_t size, TickType_t ticks_to_wait)"))
        driver_exe = temp / "idf_driver_record"
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare",
                        "-I", str(temp), str(root / "tests/host/test_usb_driver_record.c"),
                        "-o", str(driver_exe)], check=True)
        subprocess.run([str(driver_exe)], check=True)
    else:
        print("IDF_USB_DRIVER_RECORD,NOT_RUN,reason=local_IDF_source_unavailable", flush=True)
    (temp / "freertos").mkdir()
    (temp / "driver").mkdir()
    common = '''#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include "esp_err.h"
typedef uint32_t TickType_t;
typedef uint8_t StackType_t;
typedef int BaseType_t;
typedef struct { unsigned length, count, read, write; uint8_t *data; pthread_mutex_t lock; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef void *TaskHandle_t;
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) pthread_mutex_lock(p)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(p)
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define portTICK_PERIOD_MS 10u
#define pdMS_TO_TICKS(n) ((n) / portTICK_PERIOD_MS)
#define portTICK_TYPE_IS_ATOMIC 1
#define configTICK_RATE_HZ 100u
TickType_t xTaskGetTickCount(void);
int xPortGetCoreID(void);
esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*callback)(void), unsigned cpu);
QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *bytes, StaticQueue_t *queue);
int xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks);
int xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks);
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, unsigned stack, void *arg, unsigned prio, TaskHandle_t *task, int core);
int xTaskNotifyGive(TaskHandle_t task);
uint32_t ulTaskNotifyTake(int clear, TickType_t ticks);
void vTaskDelay(TickType_t ticks);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task);
int64_t esp_timer_get_time(void);
typedef struct { uint32_t tx_buffer_size, rx_buffer_size; } usb_serial_jtag_driver_config_t;
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *config);
bool usb_serial_jtag_is_driver_installed(void);
bool usb_serial_jtag_is_connected(void);
int usb_serial_jtag_write_bytes(const void *data, size_t length, TickType_t ticks);
int usb_serial_jtag_read_bytes(void *data, uint32_t length, TickType_t ticks);
esp_err_t usb_serial_jtag_wait_tx_done(TickType_t ticks);
#define ESP_VFS_FLAG_DEFAULT 0
struct esp_vfs {
    int flags;
    int (*open)(const char *, int, int);
    ssize_t (*write)(int, const void *, size_t);
    int (*close)(int);
    int (*fstat)(int, struct stat *);
};
typedef struct esp_vfs esp_vfs_t;
esp_err_t esp_vfs_register(const char *path, const esp_vfs_t *vfs, void *context);
'''
    (temp / "common.h").write_text(common)
    with (temp / "common.h").open("a") as file:
        file.write(f"\n#define ESP_VFS_PATH_MAX {path_max}\n")
    (temp / "idf_vfs_prefix.h").write_text(validator + "\n")
    (temp / "idf_reent_init.h").write_text('''
struct _reent { FILE *_stdin, *_stdout, *_stderr; void (*cleanup)(void); int sdidinit; };
static struct _reent testGlobalReent;
#define _GLOBAL_REENT (&testGlobalReent)
#define _REENT_STDIN(r) ((r)->_stdin)
#define _REENT_STDOUT(r) ((r)->_stdout)
#define _REENT_STDERR(r) ((r)->_stderr)
#define _REENT_CLEANUP(r) ((r)->cleanup)
#define _REENT_SDIDINIT(r) ((r)->sdidinit)
''' + reent_init + "\n")
    (temp / "serial_capture.h").write_text(
        "/* Verbatim capture slices; scheduling injection is synthetic. */\n"
        + "static const char capturedConfig[] = " + json.dumps(captured_config.decode("ascii")) + ";\n"
        + "static const char capturedPr[] = " + json.dumps(captured_pr.decode("ascii")) + ";\n"
        + "static const char capturedAdsPrefix[] = " + json.dumps(captured_ads_prefix.decode("ascii")) + ";\n"
        + "static const char capturedUsbReply[] = " + json.dumps(captured_usb_reply.decode("ascii")) + ";\n"
        + "static const char encodedFrame[] = " + json.dumps(encoded_frame.decode("ascii")) + ";\n")
    frame_output = (root / "main/output/sensorarrayFrameOutput.c").read_text()
    (temp / "frame_output_function.h").write_text(frame_output[frame_output.index("esp_err_t sensorarrayFrameOutputPrint("):])
    usb_sink = (root / "main/output/sensorarrayUsbSink.c").read_text()
    usb_header = (root / "main/output/sensorarrayUsbSink.h").read_text()
    packet_header = (root / "main/output/sensorarrayTextProtocol.h").read_text()
    packet_max = re.search(r"^#define SENSORARRAY_TEXT_PACKET_MAX\s+\d+u$", packet_header, re.M).group()
    begin = usb_header.index("typedef struct {\n    uint32_t sentPackets;")
    end = usb_header.index("} sensorarrayUsbSinkStats_t;", begin) + len("} sensorarrayUsbSinkStats_t;")
    (temp / "usb_sink_functions.h").write_text(packet_max + "\n" + usb_header[begin:end] + "\n" +
        "static portMUX_TYPE s_usbStatsMux = portMUX_INITIALIZER_UNLOCKED;\n"
        "static sensorarrayUsbSinkStats_t s_usbStats;\nstatic bool s_usbInitialized = true;\n" +
        "\n".join(actual_function(usb_sink, signature) for signature in (
            "static esp_err_t sensorarrayUsbSinkPublishClass(const sensorarrayTextPacket_t *packet, bool diagnostic)",
            "esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet)",
            "esp_err_t sensorarrayUsbSinkPublishDiagnostic(const sensorarrayTextPacket_t *packet)")))
    for file in ["freertos/FreeRTOS.h", "freertos/task.h", "freertos/queue.h", "driver/usb_serial_jtag.h", "esp_timer.h", "esp_vfs.h", "esp_freertos_hooks.h"]:
        (temp / file).write_text('#include "common.h"\n')
    (temp / "esp_heap_caps.h").write_text("#include <stddef.h>\n#include <stdint.h>\n#define MALLOC_CAP_8BIT 1u\n#define MALLOC_CAP_SPIRAM 2u\n#define MALLOC_CAP_INTERNAL 4u\nvoid *heap_caps_calloc(size_t count, size_t size, uint32_t caps);\n")
    (temp / "sdkconfig.h").write_text("#define CONFIG_SENSORARRAY_OUTPUT_TASK_CORE 0\n")
    exe = temp / "test"
    # Reuse the WorkBudget address/attribute stubs while retaining this test's
    # synthetic scheduler headers first. The memory stub owns its test-only
    # storage classification flag; no production placement is claimed on host.
    compile_args = [os.environ.get("CC", "cc"), "-std=c11", "-D_GNU_SOURCE", "-pthread", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-I", str(temp), "-I", str(root / "tests/host/work_budget_stubs"), "-I", str(root / "tests/host/include"), "-I", str(root / "components/sensorarrayWifi/include"), "-I", str(root / "components/sensorarrayWorkBudget/include"), "-I", str(root / "core/transport"), str(root / "tests/host/test_serial_io.c"), str(root / "components/sensorarrayWorkBudget/sensorarrayWorkBudget.c")]
    if args.baseline:
        baseline = temp / "baselineSerialIo.c"
        baseline.write_bytes(subprocess.check_output(["git", "show", f"{args.baseline}:core/transport/sensorarraySerialIo.c"], cwd=root))
        baseline_exe = temp / "baseline"
        subprocess.run(compile_args + ["-DSERIAL_IO_BASELINE", f'-DSERIAL_IO_SOURCE="{baseline}"', "-o", str(baseline_exe)], check=True)
        subprocess.run([str(baseline_exe)], check=True)
    if args.fragment_baseline:
        baseline = temp / "fragmentBaselineSerialIo.c"
        baseline.write_bytes(subprocess.check_output(["git", "show", f"{args.fragment_baseline}:core/transport/sensorarraySerialIo.c"], cwd=root))
        baseline_exe = temp / "fragment_baseline"
        subprocess.run(compile_args + ["-DSERIAL_IO_FRAGMENT_BASELINE", f'-DSERIAL_IO_SOURCE="{baseline}"', "-o", str(baseline_exe)], check=True)
        subprocess.run([str(baseline_exe)], check=True)
    if args.tx_baseline:
        baseline = temp / "txBaselineSerialIo.c"
        baseline.write_bytes(subprocess.check_output(["git", "show", f"{args.tx_baseline}:core/transport/sensorarraySerialIo.c"], cwd=root))
        baseline_exe = temp / "tx_baseline"
        subprocess.run(compile_args + ["-DSERIAL_IO_TX_BASELINE", f'-DSERIAL_IO_SOURCE="{baseline}"', "-o", str(baseline_exe)], check=True)
        subprocess.run([str(baseline_exe)], check=True)
    if args.admission_baseline:
        baseline = temp / "admissionBaselineSerialIo.c"
        baseline.write_bytes(subprocess.check_output(["git", "show", f"{args.admission_baseline}:core/transport/sensorarraySerialIo.c"], cwd=root))
        baseline_exe = temp / "admission_baseline"
        subprocess.run(compile_args + ["-DSERIAL_IO_ADMISSION_BASELINE", f'-DSERIAL_IO_SOURCE="{baseline}"', "-o", str(baseline_exe)], check=True)
        subprocess.run([str(baseline_exe)], check=True)
    source_override = [f'-DSERIAL_IO_SOURCE="{args.source.resolve()}"'] if args.source else []
    subprocess.run(compile_args + source_override + ["-o", str(exe)], check=True)
    for allocation in ["psram", "internal", "failure_retry", "registration_retry", "stdout_retry", "stderr_retry", "route_probe_retry", "queue_retry", "task_retry", "foreign_driver"]:
        subprocess.run([str(exe), allocation], check=True)
