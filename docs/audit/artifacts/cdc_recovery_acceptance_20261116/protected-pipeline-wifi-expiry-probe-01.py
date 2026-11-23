#!/usr/bin/env python3
"""Saved synthetic Wi-Fi-only expiry regression probe; no hardware."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tools'))
import test_protected_frame_pipeline as harness
original_run=harness.subprocess.run
original_test=str(ROOT/'tests/host/test_protected_frame_pipeline.c')
replacement=str(Path(__file__).with_suffix('.c'))
def patched_run(command,*args,**kwargs):
    if isinstance(command,list): command=[replacement if item==original_test else item for item in command]
    return original_run(command,*args,**kwargs)
harness.subprocess.run=patched_run
harness.main()
