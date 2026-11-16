#!/usr/bin/env python3
"""Regression tests for false config identifiers and real undefined symbols."""
from __future__ import annotations

import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import audit_config_surface as audit


class ConfigSurfaceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="sensorarray-config-surface-")
        self.addCleanup(self.temporary.cleanup)
        base = Path(self.temporary.name)
        self.root = base / "project"
        self.sdk = base / "sdk"
        (self.root / "main").mkdir(parents=True)
        (self.root / "docs/audit").mkdir(parents=True)
        (self.root / "build").mkdir()
        (self.sdk / "components/esp_psram/esp32s3").mkdir(parents=True)
        (self.sdk / "Kconfig").write_text("menuconfig SDK_MENU\n    bool\n")
        (self.sdk / "components/esp_psram/esp32s3/Kconfig.spiram").write_text(
            "config SPIRAM\n    bool\n"
        )
        self.kconfig = self.root / "main/Kconfig.projbuild"
        self.kconfig.write_text("config SENSORARRAY_KNOWN\n    bool\n")
        (self.root / "CMakeLists.txt").write_text("if(CONFIG_SENSORARRAY_KNOWN)\nendif()\n")
        (self.root / "build/project_description.json").write_text(
            json.dumps({"idf_path": str(self.sdk)})
        )
        (self.root / "docs/audit/run.txt").write_text(
            "CONFIG_IDENTITY_TESTS,passed=1\nCONFIG_SENSORARRAY_DOC_LOG\n"
        )
        self.source = self.root / "main/main.c"
        self.source.write_text(
            r'''// CONFIG_SENSORARRAY_COMMENT
/* CONFIG_SENSORARRAY_BLOCK
 * #ifndef CONFIG_SENSORARRAY_FAKE_FALLBACK
 */
const char *message = "CONFIG_TERM_DROP,escaped=\"CONFIG_SENSORARRAY_QUOTED\"";
const char *multiline = "CONFIG_SENSORARRAY_LITERAL\
 CONFIG_SENSORARRAY_CONTINUED_LITERAL";
int character = 'CONFIG_SENSORARRAY_CHARACTER';
// Continued comment \
CONFIG_SENSORARRAY_CONTINUED_COMMENT
#if CONFIG_SENSORARRAY_KNOWN && CONFIG_SPIRAM && CONFIG_SDK_MENU
#endif
'''
        )
        self.addCleanup(patch.stopall)
        patch.object(audit, "ROOT", self.root).start()
        patch.object(audit, "SCAN_FILES", [self.kconfig]).start()
        patch.dict(os.environ, {"IDF_PATH": str(base / "missing-sdk")}).start()

    def run_audit(self) -> tuple[int, str]:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = audit.main()
        return code, output.getvalue()

    def test_literals_comments_and_document_logs_are_not_macros(self) -> None:
        self.assertEqual(set(audit.collect_used()), {"SENSORARRAY_KNOWN", "SPIRAM", "SDK_MENU"})
        self.assertEqual(audit.collect_fallbacks(), {})
        self.assertNotIn(self.root / "docs/audit/run.txt", audit.iter_source_files())
        self.assertIn(self.root / "CMakeLists.txt", audit.iter_source_files())
        self.assertEqual(audit.find_sdk_root(), self.sdk)
        code, output = self.run_audit()
        self.assertEqual(code, 0, output)
        self.assertIn("usedButNotDefined: 0", output)

    def test_real_undefined_project_macro_still_fails_with_correct_line(self) -> None:
        text = self.source.read_text()
        line = len(text.splitlines()) + 1
        self.source.write_text(text + "#if CONFIG_SENSORARRAY_REAL_MISSING\n#endif\n")
        self.assertEqual(audit.collect_used()["SENSORARRAY_REAL_MISSING"], [f"main/main.c:{line}"])
        code, output = self.run_audit()
        self.assertEqual(code, 1, output)
        self.assertIn("usedButNotDefined: 1\n  SENSORARRAY_REAL_MISSING", output)

    def test_sdk_definitions_do_not_exempt_undefined_spiram_names(self) -> None:
        self.source.write_text("#if CONFIG_SPIRAM_UNDECLARED\n#endif\n")
        code, output = self.run_audit()
        self.assertEqual(code, 1, output)
        self.assertIn("SPIRAM_UNDECLARED", output)

    def test_no_sdk_uses_only_exact_spiram_compatibility_name(self) -> None:
        (self.root / "build/project_description.json").unlink()
        self.source.write_text("#if CONFIG_SPIRAM\n#endif\n")
        self.assertIsNone(audit.find_sdk_root())
        code, output = self.run_audit()
        self.assertEqual(code, 0, output)
        self.source.write_text("#if CONFIG_SPIRAM_UNDECLARED || CONFIG_SENSORARRAY_REAL_MISSING\n#endif\n")
        code, output = self.run_audit()
        self.assertEqual(code, 1, output)
        self.assertIn("usedButNotDefined: 2", output)

    def test_managed_examples_and_local_macros_are_not_project_uses(self) -> None:
        dependency = self.root / "managed_components/vendor__usb"
        example = dependency / "examples/other_board/main.c"
        example.parent.mkdir(parents=True)
        example.write_text(
            "enum { CONFIG_LOCAL_COMPLETE, CONFIG_LOCAL_TOTAL_LEN = 9 };\n"
            "#ifndef CONFIG_EXAMPLE_ONLY\n#define CONFIG_EXAMPLE_ONLY 1\n#endif\n"
            "#if CONFIG_SENSORARRAY_ENABLE_BLE\n#endif\n"
        )
        (dependency / "CMakeLists.txt").write_text("if(CONFIG_OTHER_BOARD_ONLY)\nendif()\n")
        self.assertNotIn(example, audit.iter_source_files())
        self.assertEqual(set(audit.collect_used()), {"SENSORARRAY_KNOWN", "SPIRAM", "SDK_MENU"})
        self.assertEqual(audit.collect_fallbacks(), {})
        code, output = self.run_audit()
        self.assertEqual(code, 0, output)
        self.assertIn("usedButNotDefined: 0", output)
        self.assertIn("legacyConfigStillUsedInCode: 0", output)

    def test_managed_local_macro_does_not_define_project_kconfig(self) -> None:
        dependency = self.root / "managed_components/vendor__usb/defs.h"
        dependency.parent.mkdir(parents=True)
        dependency.write_text("#define CONFIG_LOCAL_TOTAL_LEN 9\n")
        self.source.write_text("#if CONFIG_LOCAL_TOTAL_LEN\n#endif\n")
        code, output = self.run_audit()
        self.assertEqual(code, 1, output)
        self.assertIn("usedButNotDefined: 1\n  LOCAL_TOTAL_LEN", output)
        self.assertEqual(audit.collect_used()["LOCAL_TOTAL_LEN"], ["main/main.c:1"])

    def test_owned_component_undefined_config_still_fails(self) -> None:
        component = self.root / "components/owned/driver.c"
        component.parent.mkdir(parents=True)
        component.write_text("#ifndef CONFIG_SENSORARRAY_COMPONENT_MISSING\n#endif\n")
        code, output = self.run_audit()
        self.assertEqual(code, 1, output)
        self.assertIn("usedButNotDefined: 1\n  SENSORARRAY_COMPONENT_MISSING", output)
        self.assertEqual(audit.collect_fallbacks()["SENSORARRAY_COMPONENT_MISSING"],
                         ["components/owned/driver.c:1"])

    def test_only_top_level_managed_components_is_excluded(self) -> None:
        for relative in ("components/managed_components/owned.c",
                         "managed_components_local/owned.c"):
            with self.subTest(path=relative):
                source = self.root / relative
                source.parent.mkdir(parents=True, exist_ok=True)
                source.write_text("#if CONFIG_SENSORARRAY_OWNED_MISSING\n#endif\n")
                try:
                    self.assertIn(source, audit.iter_source_files())
                    code, output = self.run_audit()
                    self.assertEqual(code, 1, output)
                    self.assertIn("usedButNotDefined: 1\n  SENSORARRAY_OWNED_MISSING", output)
                finally:
                    source.unlink()

    def test_managed_kconfig_definitions_remain_available_to_project(self) -> None:
        dependency = self.root / "managed_components/vendor__usb/Kconfig"
        dependency.parent.mkdir(parents=True)
        dependency.write_text("config VENDOR_USB_KNOWN\n    bool\n")
        config_files = audit.iter_kconfig_files()
        self.assertIn(dependency, config_files)
        with patch.object(audit, "SCAN_FILES", config_files):
            self.source.write_text("#if CONFIG_VENDOR_USB_KNOWN\n#endif\n")
            self.assertEqual(audit.collect_defined()["VENDOR_USB_KNOWN"],
                             ["managed_components/vendor__usb/Kconfig:1"])
            code, output = self.run_audit()
            self.assertEqual(code, 0, output)
            self.assertIn("usedButNotDefined: 0", output)
            self.source.write_text("#if CONFIG_VENDOR_USB_UNKNOWN\n#endif\n")
            code, output = self.run_audit()
            self.assertEqual(code, 1, output)
            self.assertIn("usedButNotDefined: 1\n  VENDOR_USB_UNKNOWN", output)


if __name__ == "__main__":
    unittest.main(verbosity=2)
