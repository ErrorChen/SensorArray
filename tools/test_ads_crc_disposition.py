#!/usr/bin/env python3
"""Execute production ADC read/classification/frame code with synthetic wire outcomes."""
from pathlib import Path
import subprocess
import tempfile
from test_rowmodes_ack_guard import findCompiler, writeFakeHeaders
from test_main_config_commit import function


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'core/measure/ads/sensorarrayAdsMatrix.c').read_text()
    harness = (root / 'tests/host/test_ads_crc_disposition.c').read_text()
    private_types = source[source.index('typedef struct {'):source.index('static const sensorarrayAdsAutoRangeConfig_t')]
    harness = harness.replace('/* TYPES */', private_types)
    names = {
        'CONFIG': 'static uint32_t sensorarrayAdsMatrixConfigurationGeneration(',
        'FRESH': 'static esp_err_t sensorarrayAdsMatrixReadFreshAfter(',
        'ADAPTIVE': 'static esp_err_t sensorarrayAdsMatrixReadAdaptive(',
        'SPREAD': 'static uint32_t sensorarrayAdsMatrixRawSpread(',
        'CELL': 'static esp_err_t sensorarrayAdsMatrixMeasureCell(',
        'INVALID': 'static void sensorarrayAdsMatrixSetCellError(',
        'VALID': 'static void sensorarrayAdsMatrixSetCellValid(',
        'FINISH': 'static void sensorarrayAdsMatrixFinishFrame(',
        'DISPOSITION': 'static bool sensorarrayAdsMatrixCompleteCrcDisposition(',
        'FRAME': 'esp_err_t sensorarrayAdsMatrixEngineReadFrame(',
        'MODE': 'esp_err_t sensorarrayAdsMatrixEngineSetMode(',
    }
    for marker, signature in names.items():
        harness = harness.replace(f'/* {marker} */', function(source, signature))
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError('native compiler required; compile-only is not PASS')
    with tempfile.TemporaryDirectory(prefix='sensorarray-crc-disposition-') as temporary:
        temp = Path(temporary)
        fake = temp / 'fake'
        fake.mkdir()
        writeFakeHeaders(fake)
        (fake / 'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
        generated = temp / 'test.c'
        generated.write_text(harness)
        exe = temp / 'test'
        command = [compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-g']
        for path in (fake, root/'tests/host/frame_stubs', root/'tests/host/include',
                     root/'main', root/'core/measure/include', root/'core/measure/ads',
                     root/'core/config', root/'core/board', root/'core/boardSupport/include',
                     root/'components/ads126xAdc/include', root/'components/fdc2214Cap/include',
                     root/'components/tmuxSwitch/include'):
            command += ['-I', str(path)]
        command += [str(generated), str(root/'core/measure/sensorarrayFrameBuilder.c'),
                    str(root/'core/measure/ads/sensorarrayAdsCache.c'),
                    str(root/'core/measure/ads/sensorarrayAdsAutoRange.c'),
                    str(root/'core/measure/ads/sensorarrayAdsMath.c'),
                    str(root/'core/measure/sensorarrayMeasurementMode.c'), '-o', str(exe)]
        subprocess.run(command, check=True)
        subprocess.run([str(exe)], check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
