#!/usr/bin/env python3
"""Execute firmware identity/encoder regressions; generated fixtures are synthetic."""
from pathlib import Path
import os
import subprocess
import tempfile
import zlib
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders


def main():
    root = Path(__file__).resolve().parents[1]
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("identity regression requires a host compiler; compile-only is not PASS")
    with tempfile.TemporaryDirectory(prefix="sensorarray-identity-") as temporary:
        temp = Path(temporary)
        fake = temp / "fake"
        fake.mkdir()
        writeFakeHeaders(fake)
        (fake / "sdkconfig.h").write_text("#pragma once\n")
        includes = [fake, root / "tests/host/frame_stubs", root / "tests/host/include",
                    root / "main", root / "main/output", root / "core/measure/include",
                    root / "core/config", root / "core/boardSupport/include",
                    root / "components/ads126xAdc/include", root / "components/fdc2214Cap/include",
                    root / "components/tmuxSwitch/include"]
        sources = [root / p for p in ["tests/host/test_config_identity.c",
                   "core/measure/sensorarrayConfigIdentity.c", "core/measure/sensorarrayMeasurementMode.c",
                   "core/measure/sensorarrayRowModeProfile.c", "core/measure/sensorarrayFrameBuilder.c",
                   "main/output/sensorarrayTextProtocol.c"]]
        executable = temp / "test_identity"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror"]
        for include in includes:
            command.extend(["-I", str(include)])
        command += [str(p) for p in sources] + ["-lm", "-o", str(executable)]
        subprocess.run(command, check=True)
        fixture_dir = root / "tests/fixtures/config_identity"
        fixture_dir.mkdir(parents=True, exist_ok=True)
        result = subprocess.run([str(executable), str(fixture_dir)], check=False, text=True, capture_output=True)
        print(result.stdout, end="")
        print(result.stderr, end="")
        result.check_returncode()
        count = 0
        for path in sorted(fixture_dir.glob("*.frame")):
            data = path.read_bytes()
            body, trailer = data.rsplit(b"K,", 1)
            fields = dict(field.split(b"=", 1) for field in trailer.strip().split(b",") if b"=" in field)
            assert int(fields[b"crc"], 16) == zlib.crc32(body), path
            header = dict(field.split(b"=", 1) for field in body.splitlines()[0].split(b",") if b"=" in field)
            for key in (b"session", b"bootId", b"rev"):
                assert header[key] == fields[key], (path, key)
            count += 1
        print(f"ENCODER_GOLDEN_CRC,passed={count},provenance=synthetic_real_encoder")
        def fields(path):
            first = path.read_text().splitlines()[0]
            return dict(token.split("=", 1) for token in first.split(",")[1:] if "=" in token)
        def identity_matches(frame, authority):
            return all(frame[key] == authority[key] for key in
                       ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid"))
        before = fields(fixture_dir / "cap_before.frame")
        volt = fields(fixture_dir / "volt_transition.frame")
        after = fields(fixture_dir / "cap_after.frame")
        reapplied = fields(fixture_dir / "cap_reapplied.frame")
        state_before = fields(fixture_dir / "cap_before.config")
        state_after = fields(fixture_dir / "cap_after.config")
        state_reapplied = fields(fixture_dir / "cap_reapplied.config")
        # Reproduce E5: C.gen=2 is ROWS, V.gen=3 is MODE; C.mgen=4.
        assert after["gen"] == after["rgen"] == "2"
        assert volt["gen"] == volt["mgen"] == "3"
        assert after["mgen"] == "4"
        # Missing terminal / frame before terminal: old authority cannot stamp
        # the frame. A side-effect-free query resolves the actual tuple.
        assert not identity_matches(after, state_before)
        assert identity_matches(after, state_after)
        # Late old terminal/frame and the same CAP reapplied cannot roll back.
        assert not identity_matches(before, state_reapplied)
        assert not identity_matches(after, state_reapplied)
        assert identity_matches(reapplied, state_reapplied)
        assert state_after["mtermId"] == after["mrid"] and state_after["mterm"] == "applied"
        print("CONFIG_RECOVERY_REPLAYS,passed=5,provenance=synthetic")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
