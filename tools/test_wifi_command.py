#!/usr/bin/env python3
"""Run the real bounded Wi-Fi command/terminal formatter, including max fields."""
from pathlib import Path
import os, subprocess, tempfile
ROOT=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='wifi-command-') as td:
    exe=Path(td)/'test'
    command=[os.environ.get('CC','gcc'),'-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-fsanitize=undefined','-g']
    for inc in ['tests/host/include','core/transport','components/sensorarrayWifi/include']:
        command+=['-I',str(ROOT/inc)]
    command += [str(ROOT/'tests/host/test_wifi_command.c'),str(ROOT/'core/transport/sensorarrayTransportWifiControl.c'),'-o',str(exe)]
    subprocess.run(command,check=True,cwd=ROOT)
    subprocess.run([str(exe)],check=True,cwd=ROOT,timeout=10)
