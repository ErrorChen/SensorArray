#!/usr/bin/env python3
"""Explicit S3 USJ download exit; no flash/efuse/calibration writes.

Use only after app hash verification. ESP32-S3 TRM 33.4 requires DTR clear
before the RTS reset sequence. A successful ioctl is NOT boot confirmation.
The caller must independently verify new app enumeration and BUILD identity.
"""
import argparse
import json
from pathlib import Path
import time

import serial
from esptool.targets.esp32s3 import ESP32S3ROM


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--mac', required=True)
    args = parser.parse_args()
    expected = args.mac.replace(':', '').upper()
    assert len(expected) == 12 and all(c in '0123456789ABCDEF' for c in expected)
    tty = Path('/sys/class/tty') / Path(args.port).name / 'device'
    base = tty.resolve(strict=True)
    usb = next(p for p in base.parents if (p / 'idVendor').is_file())
    identity = {k: (usb / k).read_text().strip() for k in
                ('idVendor', 'idProduct', 'serial', 'devnum')}
    assert identity['idVendor'] == '303a' and identity['idProduct'] == '1001'
    assert identity['serial'].replace(':', '').upper() == expected
    print(json.dumps({'sysfs': identity, 'flash_writes': 0}), flush=True)
    # Do not accept pySerial's default DTR=true on Linux. Opening with both
    # lines clear cannot be replaced by hard_reset()'s RTS-only transition.
    port = serial.Serial(port=None, baudrate=115200, timeout=1,
                         write_timeout=2, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = args.port
    try:
        port.open()
        esp = ESP32S3ROM(port, baud=115200)
        esp.connect(mode='no_reset', attempts=1)
        actual = bytes(esp.read_mac()).hex().upper()
        assert actual == expected, actual
        before = esp.read_reg(esp.RTC_CNTL_OPTION1_REG)
        strap = esp.read_reg(esp.GPIO_STRAP_REG)
        esp.write_reg(esp.RTC_CNTL_OPTION1_REG, 0,
                      esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
        after = esp.read_reg(esp.RTC_CNTL_OPTION1_REG)
        assert not after & esp.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK
        print(json.dumps({'mac': actual, 'force_before': before,
                          'force_after': after, 'strap': strap,
                          'DTR': port.dtr, 'RTS': port.rts}), flush=True)
        # TRM USJ normal boot: clear DTR, clear RTS, set RTS, clear RTS.
        # Deliberately no blind reconnect/reset retry if the USB port vanishes.
        port.dtr = False
        port.rts = False
        time.sleep(0.1)
        port.rts = True
        time.sleep(0.1)
        port.rts = False
        print(json.dumps({'reset_sequence_issued': True,
                          'app_boot_verified': False}), flush=True)
    finally:
        port.close()


if __name__ == '__main__':
    main()
