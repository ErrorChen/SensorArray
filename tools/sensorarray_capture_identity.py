"""New, pure identity checks for authorized final firmware captures.

This module performs no I/O. An ELF/boot identity is evidence of the intended
firmware/configuration domain, never evidence of physical analog isolation.
Legacy callers may omit all three expected fields. Final HIL entry points
require all three, sourced from a separately recorded BUILD/CONFIG read.
"""
from __future__ import annotations

from dataclasses import dataclass
import re


IDENTITY_FIELDS = ("session", "bootId", "rev", "mgen", "mrid", "rgen", "rrid", "pgen", "prid")
CONFIG_FIELDS = IDENTITY_FIELDS + (
    "boundary", "state", "mode", "rows", "profile", "err",
    "mtermId", "mterm", "merr", "ptermId", "pterm", "perr", "rtermId", "rterm", "rerr",
)
U32_FIELDS = set(IDENTITY_FIELDS[1:]) | {"boundary", "rows", "mtermId", "ptermId", "rtermId"}


class CaptureIdentityError(ValueError):
    """An unknown/mismatched identity cannot authorize a mutating command."""


def u32(value, name):
    if isinstance(value, bool) or not re.fullmatch(r"[0-9]+", str(value)):
        raise CaptureIdentityError("invalid_expected_or_config_u32:" + name)
    number = int(value)
    if number >= 1 << 32:
        raise CaptureIdentityError("invalid_expected_or_config_u32:" + name)
    return number


@dataclass(frozen=True)
class ExpectedCaptureIdentity:
    elf_sha256: str
    session: str
    boot_id: int

    def __post_init__(self):
        if not isinstance(self.elf_sha256, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", self.elf_sha256):
            raise CaptureIdentityError("full_expected_ELF_sha256_required")
        if not isinstance(self.session, str) or not re.fullmatch(r"[0-9a-fA-F]{16}", self.session):
            raise CaptureIdentityError("full_expected_session_required")
        object.__setattr__(self, "elf_sha256", self.elf_sha256.lower())
        object.__setattr__(self, "session", self.session.upper())
        object.__setattr__(self, "boot_id", u32(self.boot_id, "bootId"))

    def output(self):
        return {"elfSha256": self.elf_sha256, "session": self.session, "bootId": str(self.boot_id)}


def expected_from_options(elf_sha256=None, session=None, boot_id=None):
    """All absent is legacy/unbound; partial expected identity is an error."""
    values = (elf_sha256, session, boot_id)
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise CaptureIdentityError("expected_ELF_session_boot_must_be_supplied_together")
    return ExpectedCaptureIdentity(*values)


def readonly_command(command):
    # Indexed CPUOWNER queries do not end in '?'. Nonquery commands, including
    # USBBOOT/reset without '=', must not bypass the mutation guard.
    if not isinstance(command, str):
        return False
    if re.fullmatch(r"[A-Z][A-Z0-9_]*\?(?:[0-7])?", command) or re.fullmatch(r"WIFIQ\?[DLC]", command):
        return True
    request = re.fullmatch(r"WIFIREQ\?([0-9]{1,10})", command)
    return request is not None and int(request[1]) < 1 << 32


def validate_config(expected, config, *, require_running):
    if not isinstance(config, dict) or any(not isinstance(config.get(key), str) or not config[key]
                                           for key in CONFIG_FIELDS):
        raise CaptureIdentityError("complete_CONFIG_authority_required")
    for key in U32_FIELDS:
        u32(config[key], key)
    if (config["session"].upper() != expected.session or
            u32(config["bootId"], "bootId") != expected.boot_id):
        raise CaptureIdentityError("expected_boot_or_session_mismatch")
    if not 1 <= int(config["rows"]) <= 8 or not re.fullmatch(r"[CVR]{8}", config["profile"]):
        raise CaptureIdentityError("invalid_CONFIG_rows_or_profile")
    for key in ("err", "merr", "perr", "rerr"):
        if not re.fullmatch(r"0x[0-9a-fA-F]{1,8}", config[key]):
            raise CaptureIdentityError("invalid_CONFIG_error:" + key)
    if require_running and (config["state"] not in {"CAPACITANCE", "VOLTAGE", "RESISTANCE"}
                            or config["mode"] not in {"CAP", "VOLT", "RES", "MIXED"}
                            or int(config["err"], 16)):
        raise CaptureIdentityError("initial_CONFIG_not_running")
    return {key: config[key] for key in CONFIG_FIELDS}


def verify_identity_evidence(expected, build, opening, closing, *, require_running=True):
    """Verify a CONFIG -> BUILD -> CONFIG bracket from actual read replies."""
    if not isinstance(expected, ExpectedCaptureIdentity):
        raise CaptureIdentityError("expected_identity_required")
    if not isinstance(build, dict) or any(not isinstance(build.get(key), str) or not build[key]
                                        for key in ("idf", "target", "project", "proto", "version", "elfSha256")):
        raise CaptureIdentityError("complete_BUILD_authority_required")
    if (build["elfSha256"].lower() != expected.elf_sha256 or build["target"] != "esp32s3"
            or build["project"] != "SensorArray" or build["proto"] != "2"):
        raise CaptureIdentityError("expected_running_image_mismatch")
    first = validate_config(expected, opening, require_running=require_running)
    last = validate_config(expected, closing, require_running=require_running)
    if first != last:
        raise CaptureIdentityError("CONFIG_changed_during_image_authority_bracket")
    return {"expected": expected.output(), "BUILD": dict(build), "CONFIG_opening": first,
            "CONFIG_closing": last, "verified": True,
            "evidence_kind": "actual_CONFIG_BUILD_CONFIG_readonly_bracket",
            "physical_safety": "NOT_ESTABLISHED_BY_IDENTITY"}


class CaptureImageAuthority:
    """Bounded caller-owned authority state; no hidden I/O or retries."""
    def __init__(self, expected):
        self.expected = expected
        self.verified = False
        self.evidence = None

    def revoke(self):
        self.verified = False

    def verify(self, build, opening, closing, *, require_running=True):
        self.revoke()
        evidence = verify_identity_evidence(self.expected, build, opening, closing,
                                            require_running=require_running)
        self.evidence = evidence
        self.verified = True
        return evidence

    def require_command(self, command):
        if not readonly_command(command) and not self.verified:
            raise CaptureIdentityError("mutation_blocked_image_authority_unverified:" + str(command))
