# main / Application Layer

`main/main.c` 保持默认主程序，`app_main()` 仍在这里完成默认业务编排。默认模式是 `PIEZO_READ / 压电读取`，旧 `sensorarrayAppRun()` 只在 `DEBUG / bring-up` app mode 下作为 debug dispatcher 入口。

## App Mode 选择

`app_main()` 按 Kconfig 编译期选择：

- `SENSORARRAY_APP_MODE_PIEZO_READ`: 默认，运行 `sensorarrayRunPiezoRead()`，`SW=GND`。
- `SENSORARRAY_APP_MODE_RESISTANCE_READ`: 运行 `sensorarrayRunResistanceRead()`，`SW=REF`。
- `SENSORARRAY_APP_MODE_DEBUG`: 运行旧 `sensorarrayAppRun()`，进入 bring-up/debug dispatcher；FDC/debug 默认使用 GND，除非 debug 子模式明确解析为 REF。

默认 `PIEZO_READ` 和 `RESISTANCE_READ` 不初始化、不轮询 FDC2214。

## SW 极性与模式关系

SensorArray board SW polarity:

```text
SW = LOW  -> REF
SW = HIGH -> GND
```

- `PIEZO_READ / 压电读取`: requires GND source, therefore SW = HIGH.
- `RESISTANCE_READ / 电阻读取`: requires REF source, therefore SW = LOW.
- `DEBUG / FDC2214`: uses GND unless the selected debug submode explicitly resolves another source.
- `ADS126x INTREF=ON` is allowed in `PIEZO_READ`; it is an ADC internal-reference state, not evidence that the external `REF/MID` node is forced into the SW path.
- Fatal `ref_sw_conflict` is only for real external REF/MID drive versus GND risk, SW command/readback mismatch, or ADS POWER/REFMUX readback that makes conversions untrustworthy.

## 初始化顺序

ADS126x 读取模式共用同一条高速 8x8 voltage scan 主体：

1. `boardSupportInit()`
2. `tmuxSwitchInit()`
3. `sensorarrayBringupInitAds()`
4. `route_apply_source_policy()` 按 source 配置 REF/SW：`PIEZO_READ -> GND` 保留 ADS126x internal reference、关闭 VBIAS，`RESISTANCE_READ -> REF` 打开 internal REF/VBIAS。
5. `RESISTANCE_READ` 执行 ADS REF/VBIAS readback 和 analog closed-loop check，打印 `DBGSYSREF`、`DBGADSREF`、`DBGADSREF_ANALOG`；`PIEZO_READ` 打印 internal reference allowed policy。
6. apply selected SW source，`PIEZO_READ -> GND`，`RESISTANCE_READ -> REF`
7. 进入 `S1-S8 x D1-D8` 8x8 voltage scan loop，每帧输出 `MATV`

启动日志会显示：

```text
APPMODE,compiled=PIEZO_READ,entry=main_fast_scan,swSource=GND,expectedSwLevel=1
APPPOLICY,appMode=PIEZO_READ,isDebugAppMode=0,mode=PIEZO_VOLTAGE,requiredSwSource=GND,requiredSwLevel=1,reason=piezo_requires_ground_source
DBGADSREF,mode=PIEZO,intref=1,vbias=0,refmux=0x00,adcReference=INTERNAL,...,result=ok
DBGREFPOLICY,mode=PIEZO_VOLTAGE,sw=1,source=GND,intref=1,vbias=0,result=allowed,reason=piezo_gnd_source_allows_internal_ref
ROUTEPOLICY,mode=PIEZO_VOLTAGE,swSource=GND,expectedSwLevel=1,cmdSwLevel=1,obsSwLevel=1,sela=ADS126X,fdcInitSkipped=1
VOLTSCAN_REFPOLICY,mode=PIEZO_VOLTAGE,swSource=GND,expectedSwLevel=1,intrefAllowed=1,vbiasRequired=0,externalRefDriveAllowed=0
VOLTSCAN_INIT,mode=PIEZO_VOLTAGE,appMode=PIEZO_READ,cnName=压电读取,swSource=GND,expectedSwLevel=1,...
```

或电阻读取：

```text
APPMODE,compiled=RESISTANCE_READ,entry=main_fast_scan,swSource=REF,expectedSwLevel=0
APPPOLICY,appMode=RESISTANCE_READ,isDebugAppMode=0,mode=RESISTIVE,requiredSwSource=REF,requiredSwLevel=0,reason=resistive_requires_reference_source
DBGREFPOLICY,source=REF,sw=0,intref=1,vbias=1,expected_ref_mv=700
DBGADSREF_ANALOG,stage=ain9_aincom_short,ain9_aincom_uv=...,expectedNearZero=1,...,result=ok
ROUTEPOLICY,mode=RESISTIVE,swSource=REF,expectedSwLevel=0,cmdSwLevel=0,obsSwLevel=0,sela=ADS126X,fdcInitSkipped=1
VOLTSCAN_REFPOLICY,mode=RESISTANCE_VOLTAGE,swSource=REF,expectedSwLevel=0,intrefAllowed=1,vbiasRequired=1,externalRefDriveAllowed=1
VOLTSCAN_INIT,mode=RESISTIVE,appMode=RESISTANCE_READ,cnName=电阻读取,swSource=REF,expectedSwLevel=0,...
```

## CSV

```text
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,...,S8D8
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<64 int32 microvolts>
```

点顺序始终为 `S1D1..S1D8,S2D1..S2D8,...,S8D8`。`timestamp_us` 是帧开始时刻，`duration_us` 是整帧耗时。可选输出：`MATV_RAW`、`MATV_GAIN`；错误帧固定输出 `MATV_ERR`。

## 切换模式

```bash
idf.py menuconfig
```

- 默认压电读取：`SensorArray application mode -> PIEZO_READ / 压电读取`
- 电阻读取：`SensorArray application mode -> RESISTANCE_READ / 电阻读取`
- FDC debug：`SensorArray application mode -> Debug / bring-up modes`，再选择 `Debug execution mode -> S5D5 FDC secondary`、`FDC I2C discovery` 或其他 debug 项。

## 文件边界

- `main.c`: app mode 分流、PIEZO/RESISTANCE 主循环、CSV 输出、auto-gain table。
- `sensorarrayApp.c/.h`: app mode 到 SW source 的单一语义来源；`PIEZO_READ -> GND`，`RESISTANCE_READ -> REF`，DEBUG 由子模式解析。
- `sensorarrayVoltageScan.c/.h`: 8x8 fast route 和 frame scan helper；生产路径必须调用 `WithSource` 版本，兼容 wrapper 只从 app mode source API 取默认 source。
- `sensorarrayBoardMap.c/.h`: 板级 mapping 单一真相源，SELA GPIO level 只能通过 `sensorarrayBoardMapSelaRouteToGpioLevel()` 转换。
- `sensorarrayBringup.c/.h`: ADS/FDC bring-up helper；REF/电阻模式要求 INTREF/VBIAS/internal REFMUX，PIEZO voltage scan 要求 GND source、INTREF allowed、VBIAS off。
- `sensorarrayApp.c` 与 `sensorarrayDebug*.c`: 旧 bring-up/debug dispatcher，仅 DEBUG mode 使用。

## REF/SW 冲突诊断

`SENSOR_ARRAY_DIAG_REF_SW_CONFLICT=1` 会进入不扫描矩阵的最小诊断循环：REF phase 每 500 ms 输出 `DBGDIAG_REF_SW,phase=REF,sw=0,intref=1,vbias=1`，持续 5 秒；GND phase 允许输出 `DBGDIAG_REF_SW,phase=GND,sw=1,intref=1,vbias=0`。示波器上只有真实外部 REF/MID driver 与 GND 短接风险或 SW readback mismatch 才应触发 fatal `ref_sw_conflict`。
