#!/usr/bin/env python3
"""SensorArray formal USB-OTG DATA/CTRL transport (new in this repair).

Imports PyUSB only when opening an explicitly selected device. Never opens CDC,
resets a device, detaches a kernel driver, or falls back to the old JTAG interface.
USB packet/read boundaries are not ASCII record boundaries; callers keep one
partial-line buffer and immutable raw recording per IN endpoint.
"""
from __future__ import annotations

import importlib.metadata
import math
import re
import time

USB_VID = 0x303A
USB_PID = 0x4041  # Experimental composite CDC1 + VENDOR2; not the old CDC PID.
USB_CAPABILITY = "usb_vendor_bulk_v1"
ROLES = {
    "data": {"interface": 0, "subclass": 0x53, "protocol": 1, "in": 0x81, "out": 0x01},
    "ctrl": {"interface": 1, "subclass": 0x53, "protocol": 2, "in": 0x82, "out": 0x02},
}


class UsbBulkError(OSError):
    """A local transport failure; it does not imply a firmware reset."""


class UsbBulkPort:
    transport = "usb_bulk"

    def __init__(self, serial: str, *, vid=USB_VID, pid=USB_PID,
                 usb_core=None, usb_util=None, clock=time.monotonic):
        if not isinstance(serial, str) or not re.fullmatch(r"[0-9A-Fa-f]{12}", serial):
            raise ValueError("usb_serial_requires_12_hex_MAC_without_colons")
        if not (0 < vid <= 0xFFFF and 0 < pid <= 0xFFFF):
            raise ValueError("usb_vid_pid_out_of_range")
        if vid == 0x303A and pid in {0x1001, 0x4001}:
            raise ValueError("refuse_legacy_JTAG_or_CDC_only_PID")
        self._clock, self._device, self._claimed = clock, None, []
        self._next_role = 0
        self._serial, self._vid, self._pid = serial.upper(), vid, pid
        self._stats = {"read_calls": 0, "read_timeouts": 0, "zero_length_reads": 0,
                       "write_calls": 0, "write_timeouts": 0, "short_writes": 0,
                       "data_rx_bytes": 0, "ctrl_rx_bytes": 0, "ctrl_out_bytes": 0}
        backend = None
        if usb_core is None or usb_util is None:
            try:
                import usb.core as usb_core
                import usb.util as usb_util
                import usb.backend.libusb1
            except ImportError as error:
                raise UsbBulkError("pyusb_missing_install_in_explicit_HIL_environment") from error
            # The timeout-with-progress behaviour is tested against PyUSB
            # 1.3.1's libusb1 backend. Do not silently select another backend.
            backend = usb.backend.libusb1.get_backend()
            if backend is None:
                raise UsbBulkError("libusb1_backend_unavailable")
        self._core, self._util = usb_core, usb_util
        selected = []
        try:
            filters = {"find_all": True, "idVendor": vid, "idProduct": pid}
            if backend is not None:
                filters["backend"] = backend
            for candidate in usb_core.find(**filters) or ():
                try:
                    # PyUSB local per-transfer limit for descriptor/configuration
                    # reads. This does not issue SET_CONFIGURATION or USB reset.
                    candidate.default_timeout = 250
                    actual_serial = candidate.serial_number
                    if isinstance(actual_serial, str) and actual_serial.upper() == self._serial:
                        selected.append(candidate)
                    else:
                        usb_util.dispose_resources(candidate)
                except BaseException:
                    usb_util.dispose_resources(candidate)
                    raise
            if len(selected) != 1:
                raise UsbBulkError("usb_device_not_found" if not selected else "usb_device_ambiguous_serial")
            self._device = selected[0]
            configuration = self._device.get_active_configuration()
            if configuration.bConfigurationValue != 1:
                raise UsbBulkError("usb_active_configuration_mismatch")
            self._role_metadata = {}
            for name, expected in ROLES.items():
                try:
                    interface = configuration[(expected["interface"], 0)]
                except (KeyError, IndexError) as error:
                    raise UsbBulkError("usb_role_interface_missing:" + name) from error
                if (interface.bInterfaceNumber != expected["interface"] or
                        interface.bAlternateSetting != 0 or interface.bInterfaceClass != 0xFF or
                        interface.bInterfaceSubClass != expected["subclass"] or
                        interface.bInterfaceProtocol != expected["protocol"]):
                    raise UsbBulkError("usb_role_descriptor_mismatch:" + name)
                endpoints = list(interface)
                if len(endpoints) != 2 or {e.bEndpointAddress for e in endpoints} != {expected["in"], expected["out"]}:
                    raise UsbBulkError("usb_role_endpoints_mismatch:" + name)
                if any((e.bmAttributes & 3) != 2 or e.wMaxPacketSize != 64 for e in endpoints):
                    raise UsbBulkError("usb_role_requires_FS_bulk_64:" + name)
                self._role_metadata[name] = {**expected, "alternate": 0, "class": 0xFF,
                                             "max_packet_bytes": 64, "transfer_type": "bulk"}
            # Validate both roles before claiming either. CDC interfaces 2/3
            # are neither inspected for I/O nor detached/claimed.
            for name in ("ctrl", "data"):
                interface_number = ROLES[name]["interface"]
                try:
                    if self._device.is_kernel_driver_active(interface_number):
                        raise UsbBulkError("usb_role_kernel_driver_active:" + name)
                except NotImplementedError:
                    pass  # Windows backend: claim still reports real binding errors.
                usb_util.claim_interface(self._device, interface_number)
                self._claimed.append(interface_number)
        except BaseException as original_error:
            owned = self._device
            cleanup_errors = []
            try:
                self.close()
            except Exception as error:
                cleanup_errors.append(str(error))
            for candidate in selected:
                if candidate is not owned:
                    try:
                        usb_util.dispose_resources(candidate)
                    except Exception as error:
                        cleanup_errors.append(str(error))
            if cleanup_errors and hasattr(original_error, "add_note"):
                original_error.add_note("USB cleanup errors: " + ";".join(cleanup_errors))
            raise

    def metadata(self):
        try:
            version = importlib.metadata.version("pyusb")
        except importlib.metadata.PackageNotFoundError:
            version = "not_installed_or_injected_backend"
        device = self._device
        backend = getattr(getattr(device, "_ctx", None), "backend", None)
        backend_name = (type(backend).__module__ + "." + type(backend).__name__) if backend is not None else "injected_or_unknown"
        return {"transport": self.transport, "capability": USB_CAPABILITY,
                "vid": self._vid, "pid": self._pid, "usb_serial": self._serial,
                "experimental_vid_pid": True,
                "bus": getattr(device, "bus", None), "address": getattr(device, "address", None),
                "configuration": 1, "roles": getattr(self, "_role_metadata", {}),
                "pyusb_version": version, "backend": backend_name,
                "descriptor_transfer_timeout_ms": 250,
                "read_poll_timeout_ms": 2, "write_poll_timeout_ms": 10,
                "ownership": "claim_DATA_and_CTRL_interfaces_only",
                "cdc_used": False, "device_reset_requested": False,
                "kernel_driver_detached": False, "configuration_changed": False,
                "tx_acceptance": "host_USB_transfer_not_firmware_command_applied",
                "stats": dict(self._stats)}

    def _timeout_ms(self, deadline, maximum=10):
        remaining = deadline - self._clock()
        if remaining <= 0:
            return None
        # libusb timeout=0 means infinite, never a nonblocking operation.
        return max(1, min(maximum, math.ceil(remaining * 1000)))

    @staticmethod
    def _is_timeout(error):
        return getattr(error, "backend_error_code", None) == -7 or getattr(error, "errno", None) in {60, 110}

    def _check_open(self):
        if self._device is None:
            raise UsbBulkError("usb_transport_closed")

    def read_some(self, max_bytes, deadline):
        """Return (role, bytes), or None after a bounded empty poll.

        Alternate service between CTRL and DATA even under continuous DATA.
        Positive partial transfers returned by PyUSB are retained, including
        the libusb1 backend's timeout-with-progress path. Empty packets/timeouts
        are not EOF; actual NO_DEVICE/PIPE/other errors remain explicit.
        """
        self._check_open()
        if max_bytes < 64:
            raise ValueError("usb_read_buffer_must_hold_one_packet")
        size = min(max_bytes, 16384) // 64 * 64
        role = ("ctrl", "data")[self._next_role]
        self._next_role ^= 1
        timeout = self._timeout_ms(deadline, maximum=2)
        if timeout is None:
            return None
        self._stats["read_calls"] += 1
        try:
            chunk = bytes(self._device.read(ROLES[role]["in"], size, timeout=timeout))
        except self._core.USBError as error:
            if self._is_timeout(error):
                self._stats["read_timeouts"] += 1
                return None
            raise UsbBulkError(f"usb_{role}_read_failed:{error}") from error
        if not chunk:
            self._stats["zero_length_reads"] += 1
            return None
        if len(chunk) > size:
            raise UsbBulkError("usb_backend_read_exceeds_buffer")
        self._stats[role + "_rx_bytes"] += len(chunk)
        return role, chunk

    def write_some(self, data, deadline):
        """Commands only use CTRL OUT. Caller retries a known short suffix."""
        self._check_open()
        timeout = self._timeout_ms(deadline)
        if timeout is None:
            return 0
        self._stats["write_calls"] += 1
        try:
            written = self._device.write(ROLES["ctrl"]["out"], bytes(data), timeout=timeout)
        except self._core.USBError as error:
            if self._is_timeout(error):
                self._stats["write_timeouts"] += 1
                return 0
            raise UsbBulkError(f"usb_ctrl_write_failed:{error}") from error
        if not isinstance(written, int) or not 0 <= written <= len(data):
            raise UsbBulkError("usb_backend_invalid_write_count")
        self._stats["short_writes"] += int(written < len(data))
        self._stats["ctrl_out_bytes"] += written
        return written

    def close(self):
        device, self._device = self._device, None
        if device is None:
            return
        first_error = None
        for interface in reversed(self._claimed):
            try:
                self._util.release_interface(device, interface)
            except Exception as error:
                first_error = first_error or error
        self._claimed.clear()
        try:
            self._util.dispose_resources(device)
        except Exception as error:
            first_error = first_error or error
        if first_error is not None:
            raise UsbBulkError(f"usb_release_failed:{first_error}") from first_error
