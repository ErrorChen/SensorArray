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

## 初始化顺序

ADS126x 读取模式共用同一条高速 8x8 voltage scan 主体：

1. `boardSupportInit()`
2. `tmuxSwitchInit()`
3. `sensorarrayBringupInitAds()`
4. `sensorarrayBringupPrepareAdsRefPath()` 打开 internal REF、VBIAS/AINCOM level shift、internal REFMUX
5. ADS REF/VBIAS readback 和 analog closed-loop check，打印 `DBGSYSREF`、`DBGADSREF`、`DBGADSREF_ANALOG`
6. apply selected SW source，`PIEZO_READ -> GND`，`RESISTANCE_READ -> REF`
7. 进入 `S1-S8 x D1-D8` 8x8 voltage scan loop，每帧输出 `MATV`

启动日志会显示：

```text
APPMODE,compiled=PIEZO_READ,entry=main_fast_scan,swSource=GND,expectedSwLevel=1
APPPOLICY,appMode=PIEZO_READ,isDebugAppMode=0,mode=PIEZO_VOLTAGE,requiredSwSource=GND,requiredSwLevel=1,reason=piezo_requires_ground_source
DBGADSREF_ANALOG,stage=ain9_aincom_short,ain9_aincom_uv=...,expectedNearZero=1,...,result=ok
ROUTEPOLICY,mode=PIEZO_VOLTAGE,swSource=GND,expectedSwLevel=1,cmdSwLevel=1,obsSwLevel=1,sela=ADS126X,fdcInitSkipped=1
VOLTSCAN_INIT,mode=PIEZO_VOLTAGE,appMode=PIEZO_READ,cnName=压电读取,swSource=GND,expectedSwLevel=1,...
```

或电阻读取：

```text
APPMODE,compiled=RESISTANCE_READ,entry=main_fast_scan,swSource=REF,expectedSwLevel=0
APPPOLICY,appMode=RESISTANCE_READ,isDebugAppMode=0,mode=RESISTIVE,requiredSwSource=REF,requiredSwLevel=0,reason=resistive_requires_reference_source
ROUTEPOLICY,mode=RESISTIVE,swSource=REF,expectedSwLevel=0,cmdSwLevel=0,obsSwLevel=0,sela=ADS126X,fdcInitSkipped=1
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
- `sensorarrayBringup.c/.h`: ADS/FDC bring-up helper；默认 ADS 读取只使用 ADS REF/VBIAS 相关路径。
- `sensorarrayApp.c` 与 `sensorarrayDebug*.c`: 旧 bring-up/debug dispatcher，仅 DEBUG mode 使用。
