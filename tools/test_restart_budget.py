#!/usr/bin/env python3
"""Compile actual RTC restart-budget functions and READY/frame call sites.

Clock, RTC storage, heap and event sinks are host shims. No device reset occurs.
The current production name is sensorarrayBootPrepareRestart (not the audit
prompt's sensorarrayBootBreadcrumbPrepareRestart).
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "main/main.c").read_text()

def function(name: str) -> str:
    match = re.search(r"^static\s+[^\n]+\b" + re.escape(name) + r"\(", source, re.M)
    if not match:
        raise RuntimeError(f"production function missing: {name}")
    opening = source.index("{", match.start())
    # These functions contain no literal brace characters in strings. Refuse to
    # silently synthesize a replacement if production source changes shape.
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]

begin = source.index("#define SENSORARRAY_BOOT_BREADCRUMB_MAGIC")
end = source.index("#define SENSORARRAY_RECOVERY_LEVEL_NONE", begin)
rtc_types = source[begin:end]
ready_begin = source.index("    ctx->systemReady = initErr == ESP_OK && bootErr == ESP_OK;")
ready_end = source.index("    sensorarrayFdcMapVerifyDebug();", ready_begin)
ready = source[ready_begin:ready_end]
frame_match = re.search(r"    sensorarrayBootBreadcrumbMarkStableProgress\(\s*err == ESP_OK[^;]+;", source)
if not frame_match:
    raise RuntimeError("actual healthy frame caller missing")

prelude = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define RTC_NOINIT_ATTR
#define portMUX_INITIALIZER_UNLOCKED 0
typedef int portMUX_TYPE;
static unsigned lockDepth;
#define portENTER_CRITICAL(p) do { (void)(p); assert(lockDepth == 0); lockDepth++; } while (0)
#define portEXIT_CRITICAL(p) do { (void)(p); assert(lockDepth == 1); lockDepth--; } while (0)
typedef enum {ESP_RST_POWERON, ESP_RST_SW} esp_reset_reason_t;
typedef struct {
    struct { uint32_t sequence; bool freshFrame; } frame;
    struct { unsigned quality; } fdcBootSummary;
    bool systemReady;
} sensorarrayAppContext_t;
static uint64_t mockMonoUs, mockRtcUs;
static esp_reset_reason_t mockResetReason;
static unsigned readyEvents;
static int64_t esp_timer_get_time(void) { return (int64_t)mockMonoUs; }
static uint64_t esp_rtc_get_time_us(void) { return mockRtcUs; }
static esp_reset_reason_t esp_reset_reason(void) { return mockResetReason; }
static uint32_t esp_get_free_heap_size(void) { return 123456; }
static const char *sensorarrayResetReasonName(esp_reset_reason_t r) { return r == ESP_RST_POWERON ? "synthetic_poweron" : "synthetic_software"; }
static const char *sensorarrayAppFdcBootQualityName(unsigned q) { return "synthetic_ok"; }
static esp_err_t sensorarrayAsyncLogPublishProtocolEvent(const char *text, size_t length)
{
    assert(length > 6 && strncmp(text, "READY,", 6) == 0); readyEvents++; return ESP_OK;
}
'''
functions = "\n\n".join(function(name) for name in [
    "sensorarrayBootBreadcrumbSetStage", "sensorarrayBootBreadcrumbStart",
    "sensorarrayBootBreadcrumbCopy", "sensorarrayBootBreadcrumbMarkStableProgress",
    "sensorarrayBootPrepareRestart",
])
# RECOVER admission/cache/identity regression now executes its real module and
# the actual main boundary in tools/test_recovery_control.py.
unit_text = prelude + rtc_types + functions
unit_text += "\nstatic void actualReadyBlock(sensorarrayAppContext_t *ctx, esp_err_t initErr, esp_err_t bootErr)\n{\n" + ready + "}\n"
unit_text += "\nstatic void actualFrameProgress(sensorarrayAppContext_t *ctx, esp_err_t err, uint64_t validMask)\n{\n" + frame_match.group(0) + "\n}\n"
unit_text += "\n#undef printf\n" + (root / "tests/host/test_restart_budget.c").read_text()
with tempfile.TemporaryDirectory(prefix="sensorarray-restart-budget-") as temp:
    temp = Path(temp)
    unit = temp / "restart_budget.c"
    unit.write_text(unit_text)
    exe = temp / "test"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", str(unit), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
