#!/usr/bin/env python3
"""Validate the documented AIN8/AINCOM battery calculation."""

from __future__ import annotations


def calculate(ain8_diff_uv: int, zero_uv: int, aincom_uv: int | None,
              numerator: int, denominator: int) -> tuple[int, str]:
    if denominator <= 0 or numerator <= 0:
        return -1, "divider_invalid"
    if aincom_uv is None:
        return -1, "no_aincom_gnd_reference"
    ain8_gnd_uv = ain8_diff_uv - zero_uv + aincom_uv
    battery_mv = ain8_gnd_uv * numerator // denominator // 1000
    if not 2500 <= battery_mv <= 5000:
        return -1, "out_of_range"
    return battery_mv, "ok"


def aincom_from_rail(rail_uv: int, avdd_uv: int, avss_magnitude_uv: int) -> int:
    nominal_rail_uv = avdd_uv + avss_magnitude_uv
    if nominal_rail_uv <= 0:
        raise ValueError("invalid nominal rail")
    return rail_uv * (avdd_uv - avss_magnitude_uv) // (2 * nominal_rail_uv)


def main() -> int:
    assert calculate(1_999_592, -47, None, 2, 1) == (
        -1, "no_aincom_gnd_reference")
    value, reason = calculate(1_999_592, -47, 0, 2, 1)
    assert value == 3999 and reason == "ok"
    assert calculate(2_059_000, 0, 0, 2, 1) == (4118, "ok")
    assert calculate(2_059_000, 0, 0, 2, 0) == (-1, "divider_invalid")
    rail_uv = 3_300_000 + 1_800_000
    aincom_uv = aincom_from_rail(rail_uv, 3_300_000, 1_800_000)
    assert aincom_uv == 750_000
    assert aincom_from_rail(5_171_000, 3_300_000, 1_800_000) == 760_441
    value, reason = calculate(1_300_000, 0, aincom_uv, 2, 1)
    assert value == 4100 and reason == "ok"
    assert abs(5_171_000 - rail_uv) <= 250_000
    assert abs(5_804_000 - rail_uv) > 250_000
    print("BATTERY_FORMULA_TEST,pass=9,fail=0,rail=5100000,aincom=750000")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
