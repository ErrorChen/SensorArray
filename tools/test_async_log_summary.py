#!/usr/bin/env python3
"""Compare current production summary/state with the immutable pre-cleanup source.

Native C execution, synthetic frames/sinks; this is not timing or HIL evidence.
The baseline is an explicit repository commit, never the moving HEAD.
"""
from pathlib import Path
import gzip
import hashlib
import re
import subprocess
import tempfile
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders

BASELINE = "b58336eeaa136d0aca23e4c62ac11eb292d5a904"
BASELINE_SHA = "b644f12edb17fe79f67d2551ab6c04a98cf9f6e04d7c1c8344504f13799f9b8d"
SOURCE = "main/output/sensorarrayAsyncLog.c"


def structure(source, name):
    end = source.index("} " + name + ";") + len("} " + name + ";")
    start = source.rfind("typedef struct {", 0, end)
    assert start >= 0
    return source[start:end]


def production(source, prefix):
    signatures = [
        "static size_t sensorarrayAsyncLogTextAppend(",
        "static sensorarrayAsyncLogSharedStats_t sensorarrayAsyncLogReadStats(",
        "static void sensorarrayAsyncLogSummarySetBaseline(",
        "static void sensorarrayAsyncLogSummaryBegin(",
        "static void sensorarrayAsyncLogUpdateSummary(",
        "static const char *sensorarrayBatteryReasonName(",
        "static void sensorarrayAsyncLogPrintCompactSummary(",
    ]
    precision = "sensorarrayPrecisionWindow_t" in source
    text = ""
    if precision:
        text += structure(source, "sensorarrayPrecisionWindow_t") + "\n"
        text += "static sensorarrayPrecisionWindow_t s_precisionWindow;\n"
        text += function(source, "static void sensorarrayAsyncLogPrecisionReset(") + "\n"
        update = function(source, "static void sensorarrayAsyncLogPrecisionUpdate(")
        # Count execution of the actual old loop; do not replace its semantics.
        update = update.replace("++cell) {", "++cell) { oldCellVisits++;", 1)
        text += update + "\n"
    text += "\n".join(function(source, sig) for sig in signatures)
    wrapper = function(source, "static void sensorarrayAsyncLogMaybePrintSummary(")
    boundary = wrapper.index("    return;") + len("    return;")
    # Exact reachable wrapper prefix. Assert the old PREC20 consumer is beyond
    # this unconditional return, not a runtime condition or disabled branch.
    assert wrapper[:boundary].count("sensorarrayAsyncLogPrintCompactSummary(summary);") == 1
    assert not re.search(r"\bif\s*\(", wrapper[:boundary])
    if precision:
        assert wrapper.index('printf("PREC20,') > boundary
    text += "\n" + wrapper[:boundary] + "\n}\n"
    for name in sorted(set(re.findall(r"\b(?:sensorarrayAsyncLog\w+|sensorarrayBatteryReasonName|s_precisionWindow|s_sharedStats|s_asyncLogWorkspace|s_summaryTruncated|s_initialFreeHeap|s_asyncLogMux)\b", text)), key=len, reverse=True):
        if name.endswith("_t"):
            continue
        text = re.sub(r"\b" + name + r"\b", prefix + name, text)
    return text


def main():
    root = Path(__file__).resolve().parents[1]
    frozen = root / "tests/fixtures/async_log/b58336e-sensorarrayAsyncLog.c.gz"
    before_bytes = gzip.decompress(frozen.read_bytes())
    assert hashlib.sha256(before_bytes).hexdigest() == BASELINE_SHA
    before = before_bytes.decode()
    after = (root / SOURCE).read_text()
    assert "sensorarrayAsyncLogPrecisionUpdate" not in after
    assert "sensorarrayPrecisionWindow_t" not in after and "PREC20," not in after
    names = ("sensorarrayAdsPerformanceSummary_t", "sensorarrayAsyncLogSummary_t",
             "sensorarrayAsyncLogWorkspace_t", "sensorarrayAsyncLogSharedStats_t")
    for name in names:
        assert structure(before, name) == structure(after, name), name
    harness = (root / "tests/host/test_async_log_summary.c").read_text()
    sink = (root / "main/output/sensorarrayUsbSink.c").read_text()
    profile_functions = function(sink, "esp_err_t sensorarrayUsbSinkInit(") + "\n" + function(
        sink, "sensorarrayUsbStreamProfile_t sensorarrayUsbSinkGetStreamProfile(").replace(
            "sensorarrayUsbSinkGetStreamProfile", "actualUsbProfile") + "\n" + function(
                sink, "bool sensorarrayUsbSinkShouldEmitData(")
    harness = harness.replace("/* ACTUAL_USB_PROFILE */", profile_functions)
    harness = harness.replace("/* ACTUAL_TYPES */", "\n".join(structure(after, n) for n in names))
    # Compare declared state fields; C structure padding is not observable
    # state and may differ between otherwise identical aggregate assignments.
    comparisons = []
    external_structs = {
        "sensorarrayUsbSinkStats_t": (root / "main/output/sensorarrayUsbSink.h").read_text(),
        "sensorarrayNetSinkStats_t": (root / "main/net/sensorarrayNetStatus.h").read_text(),
    }
    def compare_fields(text, typename, prefix=""):
        for line in structure(text, typename).splitlines():
            match = re.fullmatch(r"\s*(\w+) (\w+);", line)
            if not match:
                continue
            kind, member = match.groups()
            name = prefix + member
            if kind in external_structs:
                compare_fields(external_structs[kind], kind, name + ".")
            elif kind.endswith("_t") and kind.startswith("sensorarray"):
                comparisons.append(f'assert(memcmp(&a->{name},&b->{name},sizeof(a->{name}))==0);')
            else:
                comparisons.append(f'assert(a->{name}==b->{name});')
    compare_fields(after, "sensorarrayAsyncLogSummary_t")
    harness = harness.replace("/* STATE_COMPARE_FIELDS */", "\n".join(comparisons))
    harness = harness.replace("/* ACTUAL_OLD */", production(before, "old_"))
    harness = harness.replace("/* ACTUAL_NEW */", production(after, "new_"))
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native execution required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-async-summary-") as directory:
        temp = Path(directory)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        generated = temp / "test.c"
        generated.write_text(harness)
        binary = temp / "test"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0"]
        for path in (fake, root / "tests/host/frame_stubs", root / "tests/host/include",
                     root / "main", root / "main/output", root / "main/net",
                     root / "core/measure/include", root / "core/measure/ads",
                     root / "core/config", root / "core/board", root / "core/transport",
                     root / "core/boardSupport/include", root / "components/ads126xAdc/include",
                     root / "components/fdc2214Cap/include", root / "components/tmuxSwitch/include",
                     root / "components/sensorarrayBle/include", root / "components/sensorarrayWifi/include"):
            command += ["-I", str(path)]
        command += [str(generated), "-lm", "-o", str(binary)]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
    print("ASYNC_SUMMARY_PROVENANCE,baseline=" + BASELINE + ",oldSha=" +
          hashlib.sha256(before.encode()).hexdigest() + ",newSha=" +
          hashlib.sha256(after.encode()).hexdigest() + ",target_timing=NOT_RUN")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
