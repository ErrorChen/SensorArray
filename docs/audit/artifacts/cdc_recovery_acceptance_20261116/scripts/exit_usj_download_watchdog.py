#!/usr/bin/env python3
"""One official ESP32-S3 watchdog exit after independently verified app flash.

No flash/eFuse/calibration write is requested. Normal esptool connect is kept:
its _post_connect can disable watchdogs before ROM MAC readback. A returned
watchdog call or a USB disconnect is NOT verified application boot. The caller
must bound this process to 30 seconds and separately inspect enumeration/BUILD.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re


def require(condition, message):
    if not condition:
        raise ValueError(message)


def normalize_mac(value):
    require(re.fullmatch(r"(?:[0-9a-fA-F]{12}|(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})", value) is not None,
            "MAC must contain exactly six bytes")
    return value.replace(":", "").upper()


def read_usj_identity(port_name, expected):
    tty = Path("/sys/class/tty") / Path(port_name).name / "device"
    base = tty.resolve(strict=True)
    usb = next((path for path in (base, *base.parents) if (path / "idVendor").is_file()), None)
    require(usb is not None, "no USB sysfs identity for requested tty")
    identity = {key: (usb / key).read_text().strip() for key in ("idVendor", "idProduct", "serial", "devnum")}
    require(identity["idVendor"].lower() == "303a" and identity["idProduct"].lower() == "1001",
            "requested tty is not Espressif native USB Serial/JTAG 303a:1001")
    require(normalize_mac(identity["serial"]) == expected, "USB sysfs serial/MAC mismatch")
    return identity


def emit(event):
    print(json.dumps(event), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--mac", required=True)
    args = parser.parse_args(argv)
    result = {"schema": "usj_watchdog_exit_v1", "watchdog_requested": False,
              "watchdog_call_returned": False, "app_boot_verified": False,
              "flash_writes": 0, "efuse_writes": 0, "automatic_retries": 0}
    port = None
    try:
        expected = normalize_mac(args.mac)
        require(args.port.startswith("/dev/") and Path(args.port).name.startswith("ttyACM"),
                "USJ helper requires an explicit /dev/ttyACM path")
        result["sysfs"] = read_usj_identity(args.port, expected)
        emit({"sysfs": result["sysfs"], "watchdog_requested": False})
        # Lazy imports leave argument/sysfs validation free of serial side
        # effects and let safety tests inject only fake SDK/serial modules.
        import serial
        from esptool.targets.esp32s3 import ESP32S3ROM
        port = serial.Serial(port=None, baudrate=115200, timeout=1, write_timeout=2, exclusive=True)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        esp = ESP32S3ROM(port, baud=115200)
        # Keep SDK chip/security validation and normal _post_connect behavior.
        esp.connect(mode="no_reset", attempts=1)
        require(esp.secure_download_mode is False, "secure download mode is not supported by this helper")
        require(esp.get_chip_id() == 9 and esp.CHIP_NAME == "ESP32-S3", "ROM chip identity is not ESP32-S3")
        actual = bytes(esp.read_mac()).hex().upper()
        require(actual == expected, "ROM MAC mismatch: " + actual)
        require(esp.uses_usb_jtag_serial() is True, "ROM transport is not USB Serial/JTAG")
        require(port.dtr is False and port.rts is False, "DTR/RTS must remain clear before watchdog exit")
        result.update(mac=actual, secure_download_mode=False, chip_id=9)
        before = esp.read_reg(esp.RTC_CNTL_OPTION1_REG)
        strap = esp.read_reg(esp.GPIO_STRAP_REG)
        esp.write_reg(esp.RTC_CNTL_OPTION1_REG, 0, esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
        after = esp.read_reg(esp.RTC_CNTL_OPTION1_REG)
        require(after & esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK == 0, "force-download bit remained set after masked clear")
        result.update(force_before=before, force_after=after, strap=strap, DTR=port.dtr, RTS=port.rts)
        emit({"mac": actual, "force_before": before, "force_after": after, "strap": strap,
              "DTR": port.dtr, "RTS": port.rts, "watchdog_requested": False})
        # Mark the attempt BEFORE invoking SDK: disconnect during its final
        # register ACK is an uncertain outcome, never an invitation to retry.
        result["watchdog_requested"] = True
        emit({"watchdog_requested": True, "watchdog_call_returned": False, "app_boot_verified": False})
        esp.watchdog_reset()
        result["watchdog_call_returned"] = True
    except Exception as error:
        result["error"] = {"type": type(error).__name__, "message": str(error)}
    finally:
        if port is not None:
            try:
                port.close()
            except Exception as error:
                result["cleanup_error"] = {"type": type(error).__name__, "message": str(error)}
    result["status"] = "WATCHDOG_CALL_RETURNED_BOOT_UNVERIFIED" if "error" not in result and "cleanup_error" not in result else "ERROR_BOOT_UNVERIFIED"
    emit(result)
    return 0 if "error" not in result and "cleanup_error" not in result else 1


if __name__ == "__main__":
    raise SystemExit(main())
