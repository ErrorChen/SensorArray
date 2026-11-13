#!/usr/bin/env python3
"""Exercise actual main frontend-hold policy/callers with synthetic hardware.

Presence bus transactions/policy have separate production-module tests. This
test covers ownership admission, command terminals, restart gating and queries.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "main/main.c").read_text()

def function(name):
    match = re.search(r"^static\s+[^\n]+\b" + re.escape(name) + r"\(", source, re.M)
    if not match:
        raise RuntimeError(f"missing production function {name}")
    opening = source.index("{", match.start())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]

names = ["sensorarrayFrontendsHeld", "sensorarrayFrontendRowsRequest",
         "sensorarrayRejectHeldCommand", "sensorarrayFrontendOwnersQuiescent",
         "sensorarrayEnterFrontendHold", "sensorarrayRunFrontendPresenceBoundary",
         "sensorarrayFormatFrontendReply"]
functions = "\n\n".join(function(name) for name in names)
runtime = function("sensorarrayRuntimeQueryCommand")
prefix = runtime[:runtime.index('    if (strcmp(command, "ROUTE?") == 0)')]
prefix += "    (void)frameSequence; return ESP_OK;\n}\n"
scan = function("sensorarrayScanTask")
begin = scan.index("    if (initErr != ESP_OK) {")
end = scan.index("        for (;;) {", begin)
startup = "static void actualStartupFailure(sensorarrayAppContext_t *ctx, esp_err_t initErr)\n{\n" + scan[begin:end] + "    }\n}\n"
loop = function("sensorarrayRunMainLoop")
assert loop.index("sensorarrayRunFrontendPresenceBoundary(ctx)") < loop.index("sensorarrayRunRecoveryBoundary(ctx)")
assert loop.index("if (sensorarrayMeasureFdcWorkersQuarantined()") < loop.index("if (frontendHeld)")
assert loop.index("if (frontendHeld)") < loop.index("sensorarrayRunAdsRestoreRecoveryAttempt(ctx)")
assert "sensorarrayBringupAdsPresenceQuarantined()" in loop[:loop.index("if (frontendHeld)")]
assert "sensorarrayBringupAdsPresenceQuarantined()" in scan[scan.index("if (frontendWait) {"):]
assert "sensorarrayRejectHeldCommand(ctx, &command);" in function("sensorarrayApplyPendingCommands")
assert source.count("frontend_presence_v1") == 2
assert source.count('if (strcmp(command, "FRONTENDS?") == 0)') == 2
assert "sensorarrayScanConfigSetRequestCallback(sensorarrayFrontendRowsRequest)" in function("sensorarrayInitRuntime")
header = (root / "core/board/sensorarrayFrontendPresence.h").read_text()
types = header[header.index("typedef enum {"):header.index("/* One acquisition coordinator")]
test = (root / "tests/host/test_frontend_wait_integration.c").read_text()
test = test.replace("/* PRESENCE_TYPES */", types)
test = test.replace("/* ACTUAL_FUNCTIONS */", functions + "\n" + prefix + "\n" + startup)
id_begin = source.index('printf("FRONTEND_IDS,')
id_end = source.index(");", id_begin) + 2
id_format = source[id_begin:id_end].replace("printf(", "int written = snprintf(line, sizeof(line),", 1)
test = test.replace("/* ACTUAL_IDS_FORMAT */", id_format)
with tempfile.TemporaryDirectory(prefix="sensorarray-frontend-wait-") as directory:
    c_file = Path(directory) / "test.c"
    executable = Path(directory) / "test"
    c_file.write_text(test)
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                    "-Werror", "-O1", "-g", str(c_file), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True, timeout=20)
print("frontend main ownership/hold/terminal/restart/query integration: PASS")
