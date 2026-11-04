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


if __name__ == "__main__":
    unittest.main(verbosity=2)
