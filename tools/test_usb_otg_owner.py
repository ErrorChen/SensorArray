#!/usr/bin/env python3
"""Compile actual OTG owner/descriptors against deterministic host stubs.

Uses the pinned TinyUSB headers and descriptor macros, not copied USB grammar.
All event schedules/register responses are synthetic. This is not DMA or HIL
validation. It neither reads a serial port nor writes the IDF build directory.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = (
    "normal_commands", "reset_event17", "reset_during_read", "unmount_old_command",
    "partial_command_reset", "queued_receive_reset", "epoch_wrap", "rx_bounds",
    "endpoint_independence", "cdc_line_state", "stop_ack", "stop_timeout", "reset_not_latched",
    "restart_fallback", "recovery_boot", "efuse_gates", "dma_required", "descriptors",
    "panic_cpu0", "panic_cpu1", "panic_no_dma", "panic_quiesced",
    "panic_init_failed", "panic_phy_failed",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, help="Explicit saved owner source for negative replay")
    parser.add_argument("--scenario", choices=SCENARIOS, help="Run one scenario")
    parser.add_argument("--backend", choices=("vendor", "cdc", "both"), default="both")
    args = parser.parse_args()
    owner = args.source or ROOT / "components/sensorarrayUsbOtg/sensorarrayUsbOtg.c"
    descriptors = ROOT / "components/sensorarrayUsbOtg/sensorarrayUsbDescriptors.c"
    config = ROOT / "components/sensorarrayUsbOtg/include/tusb_config.h"
    header = ROOT / "components/sensorarrayUsbOtg/include/sensorarrayUsbOtg.h"
    with tempfile.TemporaryDirectory(prefix="sensorarray-usb-otg-host-") as directory:
        temp = Path(directory)
        tinyusb = ROOT / "managed_components/espressif__tinyusb"
        if not (tinyusb / "src/tusb.h").exists():
            archive = Path.home() / ".espressif/tools/components/espressif/tinyusb/0.21.0~1/espressif__tinyusb-v0.21.0_1.zip"
            if not archive.exists():
                raise SystemExit("Pinned TinyUSB 0.21.0~1 headers unavailable; resolve project dependencies first")
            tinyusb = temp / "tinyusb"
            with zipfile.ZipFile(archive) as package:
                # Registry archive is trusted toolchain material; reject paths
                # escaping this disposable extraction directory regardless.
                for name in package.namelist():
                    if Path(name).is_absolute() or ".." in Path(name).parts:
                        raise SystemExit("Unsafe TinyUSB package path")
                package.extractall(tinyusb)
        manifest = (tinyusb / "idf_component.yml").read_text()
        assert re.search(r"version:\s*['\"]?0\.21\.0~1", manifest)
        # Snapshot changing shared sources once; never compile a half-written
        # production file or emit generated files into the repository build.
        sources = {"owner": owner, "descriptors": descriptors, "config": config, "header": header}
        hashes = {}
        for key, path in sources.items():
            data = path.read_bytes()
            hashes[key] = hashlib.sha256(data).hexdigest()
            destination = {"owner": "owner.c", "descriptors": "descriptors.c",
                           "config": "tusb_config.h", "header": "sensorarrayUsbOtg.h"}[key]
            if key == "config":
                # Only the host OS/MCU/placement are substituted. All real
                # class counts, sizes, DMA flags and descriptors remain exact.
                text = data.decode().replace("#define CFG_TUSB_OS OPT_OS_FREERTOS", "#define CFG_TUSB_OS OPT_OS_NONE")
                text += "\n#define CFG_TUSB_MCU OPT_MCU_NONE\n#define TUP_DCD_ENDPOINT_MAX 7\n"
                (temp / destination).write_text(text)
            else:
                (temp / destination).write_bytes(data)
        wrappers = (
            "esp_err.h", "esp_attr.h", "sdkconfig.h", "esp_efuse.h", "esp_efuse_table.h",
            "esp_rom_sys.h", "esp_system.h", "esp_timer.h", "esp_mac.h", "esp_macros.h",
            "esp_cpu.h", "esp_private/esp_clk_utils.h", "esp_private/rtc_clk.h",
            "hal/clk_tree_ll.h", "xtensa/xtensa_api.h",
            "freertos/FreeRTOS.h", "freertos/task.h", "freertos/queue.h",
            "esp_private/periph_ctrl.h", "esp_private/usb_phy.h",
            "hal/usb_serial_jtag_hal.h", "hal/usb_serial_jtag_ll.h",
            "soc/rtc_cntl_reg.h", "soc/system_reg.h", "soc/dport_access.h", "soc/usb_reg.h",
            "soc/usb_dwc_struct.h", "rom/usb/chip_usb_dw_wrapper.h",
        )
        for name in wrappers:
            path = temp / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "common.h"\n')
        executable = temp / "owner_test"
        command = [os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                   "-Wno-unused-parameter", "-I", str(temp), "-I", str(ROOT / "tests/host/usb_otg_stubs"),
                   "-I", str(tinyusb / "src"), str(ROOT / "tests/host/test_usb_otg_owner.c"),
                   "-o", str(executable)]
        backends = ("vendor", "cdc") if args.backend == "both" else (args.backend,)
        for backend in backends:
            build = command + (["-DCONFIG_SENSORARRAY_USB_OTG_CDC=1"] if backend == "cdc" else [])
            subprocess.run(build, check=True)
            for scenario in ([args.scenario] if args.scenario else SCENARIOS):
                subprocess.run([str(executable), scenario], check=True, timeout=5)
        print(json.dumps({"suite": "USB_OTG_OWNER", "result": "PASS",
                          "scenarios": (1 if args.scenario else len(SCENARIOS)) * len(backends), "backends": backends,
                          "source_sha256": hashes, "tinyusb": "0.21.0~1",
                          "hardware": "NOT_RUN", "model": "synthetic SDK/RTOS events; actual production C and TinyUSB descriptors"}))


if __name__ == "__main__":
    main()
