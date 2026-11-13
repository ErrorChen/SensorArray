#!/usr/bin/env bash
# Temporary (until host reboot) libusb permission for this exact prototype.
# No firmware/device reset, NVS access, driver detach or broad USB permission.
set -euo pipefail
serial=80B54ECDD9AC
rule=/run/udev/rules.d/70-sensorarray-otg-task.rules
line='SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="303a", ATTR{idProduct}=="4041", ATTR{serial}=="80B54ECDD9AC", GROUP="dialout", MODE="0660"'
if [[ ${1:-} == --dry-run ]]; then
    printf 'Create temporary rule %s:\n%s\nReload rules and apply only to matching device %s.\n' "$rule" "$line" "$serial"
    exit 0
fi
if [[ ${EUID} -ne 0 ]]; then
    printf 'Run with sudo; this changes only the matching USB device permission.\n' >&2
    exit 1
fi
getent group dialout >/dev/null
if [[ -e "$rule" ]] && [[ $(cat "$rule") != "$line" ]]; then
    printf 'Refusing to overwrite an existing different rule: %s\n' "$rule" >&2
    exit 1
fi
install -d -m 0755 /run/udev/rules.d
printf '%s\n' "$line" > "$rule"
chmod 0644 "$rule"
udevadm control --reload-rules
# An add/change udev event updates permissions; it is not a USB reset.
for device in /sys/bus/usb/devices/*; do
    [[ -r "$device/idVendor" && -r "$device/idProduct" && -r "$device/serial" ]] || continue
    [[ $(cat "$device/idVendor") == 303a && $(cat "$device/idProduct") == 4041 && $(cat "$device/serial") == "$serial" ]] || continue
    udevadm trigger --action=change "$device"
done
udevadm settle --timeout=5
printf 'Temporary SensorArray OTG permission installed; removed automatically on host reboot.\n'
