#!/usr/bin/env python3
"""Execute actual PGA/bypass stability paths; synthetic ADC/profile boundaries."""
from pathlib import Path
import argparse
import hashlib
import re
import subprocess
import tempfile
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders
from test_main_config_commit import function


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',help='Reproduce the PGA stability bug from this immutable revision')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    path='core/measure/ads/sensorarrayAdsMatrix.c'
    source=(subprocess.check_output(['git','show',f'{args.baseline}:{path}'],cwd=root).decode()
            if args.baseline else (root/path).read_text())
    harness=(root/'tests/host/test_ads_cell_stability.c').read_text()
    harness=harness.replace('/* TYPES */',source[source.index('typedef struct {'):source.index('static const sensorarrayAdsAutoRangeConfig_t')])
    constants=[]
    for name in ('sensorarrayAdsAutoRangeConfig_t','sensorarrayAdsResistanceConfig_t'):
        constants.append(re.search(r'static const '+name+r'[\s\S]*?\n};',source).group())
    harness=harness.replace('/* CONSTANTS */','\n'.join(constants))
    for marker,signature in {
        'GENERATION':'static uint32_t sensorarrayAdsMatrixConfigurationGeneration(',
        'FRESH':'static esp_err_t sensorarrayAdsMatrixReadFreshAfter(',
        'SPREAD':'static uint32_t sensorarrayAdsMatrixRawSpread(',
        'ADAPTIVE':'static esp_err_t sensorarrayAdsMatrixReadAdaptive(',
        'IO_ERROR':'static sensorarrayCellError_t sensorarrayAdsMatrixIoError(',
        'INVALIDATE':'void sensorarrayAdsMatrixEngineInvalidateCaches(',
        'MEASURE_CELL_IMPL':'static esp_err_t sensorarrayAdsMatrixMeasureCellImpl(',
    }.items():
        harness=harness.replace(f'/* {marker} */',function(source,signature))
    driver=(root/'components/ads126xAdc/ads126xAdc.c').read_text()
    harness=harness.replace('/* RAW_UV */',function(driver,'int32_t ads126xAdcRawToMicrovolts('))
    config=dict(re.findall(r'^(CONFIG_SENSORARRAY_ADS_\w+)=(-?\d+|0x[0-9a-fA-F]+)$',(root/'sdkconfig').read_text(),re.M))
    definitions=['#define SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_MARGIN_FACTOR 3u']
    for key in sorted(set(re.findall(r'CONFIG_SENSORARRAY_ADS_\w+',harness))):
        definitions.append(f'#define {key} {config[key]}')
    harness=harness.replace('/* CONFIGURATION */','\n'.join(definitions))
    compiler,native=findCompiler()
    if not native:raise RuntimeError('native host execution required')
    print(f'ADS_CELL_STABILITY_SOURCE,revision={args.baseline or "worktree"},sha256={hashlib.sha256(source.encode()).hexdigest()}',flush=True)
    with tempfile.TemporaryDirectory(prefix='sensorarray-cell-stability-') as temporary:
        temp=Path(temporary);fake=temp/'fake';fake.mkdir();writeFakeHeaders(fake)
        (fake/'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
        generated=temp/'test.c';generated.write_text(harness);exe=temp/'test'
        command=[compiler,'-std=c11','-Wall','-Wextra','-Werror','-g']
        for inc in (fake,root/'tests/host/frame_stubs',root/'tests/host/include',root/'main',
                    root/'core/measure/include',root/'core/measure/ads',root/'core/config',root/'core/board',
                    root/'core/boardSupport/include',root/'components/ads126xAdc/include',
                    root/'components/fdc2214Cap/include',root/'components/tmuxSwitch/include'):
            command+=['-I',str(inc)]
        if args.baseline:command+=['-DEXPECT_PGA_STABILITY_BUG']
        command+=[str(generated),str(root/'core/measure/ads/sensorarrayAdsCache.c'),
                  str(root/'core/measure/ads/sensorarrayAdsAutoRange.c'),
                  str(root/'core/measure/ads/sensorarrayAdsMath.c'),'-o',str(exe)]
        subprocess.run(command,check=True);subprocess.run([str(exe)],check=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
