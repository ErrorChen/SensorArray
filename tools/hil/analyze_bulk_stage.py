#!/usr/bin/env python3
"""Offline formal bulk/CTRL/CDC + optional BLE analysis; no device access.

Use --debug-summary for a finalized independent CDC observer (repeat for
separate sessions); absent or rejected evidence remains unknown. The existing
analyzer CLI writes immutable reports unless explicitly asked to replace one
of its own reports. This wrapper adds no automatic acceptance claim.
"""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from analyze_firmware_benchmark import main

if __name__ == '__main__':
    raise SystemExit(main())
