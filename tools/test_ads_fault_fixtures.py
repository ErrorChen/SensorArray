#!/usr/bin/env python3
"""Generate/verify synthetic fault replays through the production C formatter."""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--update", action="store_true", help="regenerate tracked synthetic fixtures")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise RuntimeError("host C compiler required")
    with tempfile.TemporaryDirectory(prefix="sensorarray-ads-fixtures-") as tmp:
        binary = Path(tmp) / "emit_ads_fault_fixture"
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(root / "tests/host/include"),
            "-I", str(root / "core/measure/ads"),
            str(root / "tests/host/emit_ads_fault_fixture.c"),
            str(root / "core/measure/ads/sensorarrayAdsFault.c"),
            "-o", str(binary),
        ], check=True)
        output = subprocess.check_output([str(binary)], text=True)
    count = 0
    for body in output.split("# fixture=")[1:]:
        name = body.split(";", 1)[0]
        text = "# fixture=" + body
        path = root / "tests/fixtures/ads_fault" / (name.lower() + ".synthetic.log")
        records = text.splitlines(keepends=True)
        assert len(records) == 8, (name, len(records))
        assert all(len(line.encode("ascii")) < 384 for line in records[1:])
        fields = dict(part.split("=", 1) for part in records[1].strip().split(",")[1:])
        assert fields["stage"] == name and fields["mode"] == "VOLT"
        assert fields["route"] == "VOLT" and fields["owner"] == "MATRIX"
        assert fields["err"] == ("0x107" if name == "REGISTER_HEALTH_READ" else "0x108")
        assert "attempted=0,acquired=0" in text
        assert "railSpanUv=5200000,avddUv=3400000,avssUv=-1800000" in text
        if name in ("POWER_COMPARE", "RESTORE_VERIFY"):
            assert "originalError=0x108,cleanupError=0x103" in text
        if args.update:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        else:
            assert path.read_text() == text, f"fixture drift: {path}"
        count += 1
    assert count == 10
    print(f"ADS_FAULT_FIXTURES,passed=1,count={count},fixture=synthetic,producer=actual_C_formatter")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
