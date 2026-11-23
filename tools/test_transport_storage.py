#!/usr/bin/env python3
"""Run actual Transport init/storage bodies with bounded fake RTOS/heap boundaries."""
from pathlib import Path
import hashlib
import os
import subprocess
import tempfile
from test_main_config_commit import function
ROOT=Path(__file__).resolve().parents[1]
NAMES=['static esp_err_t sensorarrayTransportEnsurePayloadStorage(',
       'static esp_err_t sensorarrayTransportInitOwned(', 'esp_err_t sensorarrayTransportInit(',
       'void sensorarrayTransportGetMemoryDiagnostics(']

def main():
    source=(ROOT/'core/transport/sensorarrayTransport.c').read_text()
    # The real branches must remain in the compiled harness; these assertions
    # prevent a future erased-NVS recovery from escaping the negative cases.
    assert 'nvs_flash_erase(' not in source
    state=source[source.index('#ifndef CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT'):source.index('static void sensorarrayTransportLogSerialCommandError(')]
    with tempfile.TemporaryDirectory(prefix='transport-storage-') as name:
        temp=Path(name)
        (temp/'storage_state.h').write_text(state)
        (temp/'storage_functions.h').write_text('\n'.join(function(source,n) for n in NAMES))
        command=[os.environ.get('CC','gcc'),'-std=c11','-O1','-g','-Wall','-Wextra','-Werror',
                 '-Wno-unused-variable','-fsanitize=undefined','-fno-omit-frame-pointer','-pthread']
        for inc in [temp,ROOT/'tests/host/include',ROOT/'core/transport',ROOT/'components/sensorarrayWifi/include',ROOT/'components/sensorarrayBle/include']:
            command+=['-I',str(inc)]
        command+=[str(ROOT/'tests/host/test_transport_storage.c'),str(ROOT/'core/transport/sensorarrayTransportPool.c'),'-o',str(temp/'test')]
        subprocess.run(command,check=True)
        subprocess.run([str(temp/'test')],check=True,timeout=10)
    print('TRANSPORT_STORAGE_SOURCE_SHA256,'+hashlib.sha256(source.encode()).hexdigest())
    return 0
if __name__=='__main__':raise SystemExit(main())
