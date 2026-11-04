#!/usr/bin/env python3
"""Compile the production route controller/board map with fault-injected IO."""
import os
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]

def main():
    with tempfile.TemporaryDirectory(prefix='sensorarray-route-') as td:
        t = Path(td)
        (t/'driver').mkdir()
        (t/'esp_timer.h').write_text('#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
        (t/'esp_rom_sys.h').write_text('#include <stdint.h>\nvoid esp_rom_delay_us(uint32_t us);\n')
        (t/'sensorarrayConfig.h').write_text('''#pragma once
#define CONFIG_SENSORARRAY_FDC_SD_GPIO 8
#define CONFIG_SENSORARRAY_ADS_MATRIX_MODE_SETTLE_US 5000
#define CONFIG_SENSORARRAY_ADS_MATRIX_ROW_SETTLE_US 20
#define CONFIG_SENSORARRAY_ADS_DATA_RATE 15
#define CONFIG_SENSORARRAY_ADS_MATRIX_INTERNAL_REF_UV 2500000
#define CONFIG_SENSORARRAY_ADS_RES_BBM_ROW_SWITCH_ENABLE 1
#define CONFIG_TMUX1108_SWITCH_DELAY_US 2
#define SENSORARRAY_ADS_MUX_AINCOM 10
#define SENSORARRAY_NA "na"
#define SENSORARRAY_S1 1
#define SENSORARRAY_S2 2
#define SENSORARRAY_S3 3
#define SENSORARRAY_S4 4
#define SENSORARRAY_S5 5
#define SENSORARRAY_S6 6
#define SENSORARRAY_S7 7
#define SENSORARRAY_S8 8
#define SENSORARRAY_D1 1
#define SENSORARRAY_D2 2
#define SENSORARRAY_D3 3
#define SENSORARRAY_D4 4
#define SENSORARRAY_D5 5
#define SENSORARRAY_D6 6
#define SENSORARRAY_D7 7
#define SENSORARRAY_D8 8

''')
        (t/'sensorarrayAcqEvent.h').write_text('int sensorarrayAcqEventPrintf(const char *, ...);\n')
        (t/'sensorarrayMeasure.h').write_text('''#pragma once
#include "sensorarrayTypes.h"
bool sensorarrayMeasureFdcWorkersQuiescent(void);
esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t*,sensorarraySelaRoute_t,uint32_t,const char*,const char*);
esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t*,const char*);
''')
        (t/'driver/gpio.h').write_text('''#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef int gpio_num_t;
#define GPIO_NUM_NC -1
#define GPIO_MODE_INPUT_OUTPUT 3
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
typedef struct {uint64_t pin_bit_mask; int mode,pull_up_en,pull_down_en,intr_type;} gpio_config_t;
esp_err_t gpio_config(const gpio_config_t*);
esp_err_t gpio_set_level(gpio_num_t,uint32_t);
int gpio_get_level(gpio_num_t);
''')
        includes=[t,ROOT/'tests/host/frame_stubs',ROOT/'tests/host/include',ROOT/'main',ROOT/'core/measure/include',ROOT/'core/config',ROOT/'core/board',ROOT/'core/measure/ads',ROOT/'core/boardSupport/include',ROOT/'components/ads126xAdc/include',ROOT/'components/fdc2214Cap/include',ROOT/'components/tmuxSwitch/include']
        cmd=[os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-ffunction-sections','-fdata-sections']
        for i in includes:cmd+=['-I',str(i)]
        exe=t/'route'
        cmd += [str(ROOT/'tests/host/test_route_safety.c'),str(ROOT/'core/measure/sensorarrayRouteController.c'),str(ROOT/'core/board/sensorarrayBoardMap.c'),str(ROOT/'core/measure/sensorarrayMeasurementMode.c'),'-Wl,--gc-sections','-o',str(exe)]
        subprocess.run(cmd,check=True)
        subprocess.run([str(exe)],check=True)
    return 0
if __name__=='__main__':raise SystemExit(main())
