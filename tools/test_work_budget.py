#!/usr/bin/env python3
"""Compile/run real CPU0 budget and drain functions; synthetic scheduler, no HIL."""
from pathlib import Path
import re
import subprocess
import tempfile
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler


def main():
    root = Path(__file__).resolve().parents[1]
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native executable required; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-work-budget-") as name:
        temporary = Path(name)
        includes = [root / "tests/host/work_budget_stubs", root / "tests/host/include",
                    root / "components/sensorarrayWorkBudget/include", root / "components/sensorarrayWorkBudget"]
        source = root / "tests/host/test_work_budget.c"
        executable = temporary / "budget"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-g"]
        for include in includes:
            command += ["-I", str(include)]
        subprocess.run(command + [str(source), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)
        harness_file = root / "tests/host/test_work_budget_drains.c"
        harness = harness_file.read_text()
        # Bind the extracted bodies to their actual production batch constants;
        # a source change must not leave a stale hand-maintained test limit.
        transport_source = (root / "core/transport/sensorarrayTransport.c").read_text()
        for name in ("SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT", "SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT"):
            definition = re.search(rf"^#define {name}\s+\d+u$", transport_source, re.M)
            if not definition:
                raise AssertionError(f"missing production batch constant: {name}")
            harness = re.sub(rf"^#define {name}\s+\d+u$", definition.group(), harness, flags=re.M)
        for path, signatures in (
            ("components/sensorarrayBle/sensorarrayBle.c", [
                "static bool sensorarrayBleReceiveNext(",
                "static void sensorarrayBleTxTask(",
                "static void sensorarrayBleControlTask("]),
            ("components/sensorarrayWifi/sensorarrayWifi.c", [
                "static void sensorarrayWifiControlTask("]),
            ("core/transport/sensorarrayTransport.c", [
                "static bool sensorarrayTransportReceiveNext(",
                "static void sensorarrayTransportTask(",
                "static void sensorarrayTransportSerialControlTask("]),
            ("main/output/sensorarrayAsyncLog.c", [
                "static bool sensorarrayAsyncLogPopFrame(",
                "static void sensorarrayAsyncLogDrainEvents(",
                "static void sensorarrayAsyncLogDrainProtocolEvents(",
                "static void sensorarrayAsyncLogTask("]),
        ):
            source = (root / path).read_text()
            for signature in signatures:
                name = signature.split("(")[0].split()[-1]
                harness = harness.replace(f"/* {name} */", function(source, signature))
        generated = temporary / "drains.c"
        generated.write_text(harness)
        executable = temporary / "drains"
        subprocess.run(command + [str(generated), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)
        # Existing fixture is synthetic, emitted by the real firmware encoder.
        # Keep its original K CRC; BLE envelopes use the real BLE CRC function.
        from text_protocol import TextProtocolParser
        frame_bytes = (root / "tests/fixtures/config_identity/volt_1.frame").read_bytes()
        parser = TextProtocolParser()
        parsed = None
        for line in frame_bytes.decode("ascii").splitlines():
            parsed = parser.feed_line(line) or parsed
        assert parsed is not None and parsed.crc_ok
        (temporary / "work_budget_frame.h").write_text(
            "static const uint8_t referenceFrame[] = {" +
            ",".join(str(b) for b in frame_bytes) + "};\n")
        harness = (root / "tests/host/test_work_budget_ble_fragments.c").read_text()
        source = (root / "components/sensorarrayBle/sensorarrayBle.c").read_text()
        batch_name = "SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT"
        definition = re.search(rf"^#define {batch_name}\s+\d+u$", source, re.M)
        assert definition, "missing production BLE fragment batch limit"
        harness = re.sub(rf"^#define {batch_name}\s+\d+u$", definition.group(), harness, flags=re.M)
        for signature in (
            "static uint32_t sensorarrayBleCrc32(",
            "static size_t sensorarrayBleMaxPayloadSize(",
            "static bool sensorarrayBlePlanFragments(",
            "static esp_err_t sensorarrayBleSendFragment(",
            "static esp_err_t sensorarrayBleSendPayload(",
        ):
            name = signature.split("(")[0].split()[-1]
            harness = harness.replace(f"/* {name} */", function(source, signature))
        generated = temporary / "fragments.c"
        generated.write_text(harness)
        executable = temporary / "fragments"
        subprocess.run(command + [str(generated), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
