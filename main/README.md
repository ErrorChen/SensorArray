# main / Application Layer

`main/main.c` 保持默认主程序，`app_main()` 仍在这里完成默认业务编排。默认模式是 `PIEZO_READ / 压电读取`，旧 `sensorarrayAppRun()` 只在 `DEBUG / bring-up` app mode 下作为 debug dispatcher 入口。

## App Mode 选择

`app_main()` 按 Kconfig 编译期选择：

- `SENSORARRAY_APP_MODE_PIEZO_READ`: 默认，运行 `sensorarrayRunPiezoRead()`，`SW=HIGH`，软件 source 为 `GND` / zero path，ADS126x REFOUT 关闭，ADC reference 使用 AVDD/AVSS。
- `SENSORARRAY_APP_MODE_RESISTANCE_READ`: 运行 `sensorarrayRunResistanceRead()`，软件 source 为 `REF`，保留 REF/MID 路径。
- `SENSORARRAY_APP_MODE_DEBUG`: 运行旧 `sensorarrayAppRun()`，进入 bring-up/debug dispatcher。

默认 `PIEZO_READ` 和 `RESISTANCE_READ` 不初始化、不轮询 FDC2214。

## 初始化顺序

ADS126x 读取模式共用同一条高速 8x8 voltage scan 主体：

1. `boardSupportInit()`
2. `tmuxSwitchInit()`
3. `sensorarrayBringupInitAds()`
4. `PIEZO_READ` 调用本地 no-ref policy，关闭 `POWER.INTREF` / REFOUT，关闭 `POWER.VBIAS`，打印 `DBGADSREFPOLICY`
5. `RESISTANCE_READ` 调用 `sensorarrayBringupPrepareAdsRefPath()`，打开 internal REF、VBIAS/AINCOM level shift、internal REFMUX，并打印 `DBGSYSREF` / `DBGADSREF`
6. apply selected SW source，`PIEZO_READ -> GND` 且 `expectedSwLevel=1`，`RESISTANCE_READ -> REF`
7. 打印 `DBGROUTEPOLICY` 和一次 `DBGTMUXPOLICY`
8. 进入 `S1-S8 x D1-D8` 8x8 voltage scan loop，每帧输出 `MATV`

启动日志会显示：

```text
APPMODE,active=PIEZO_READ,cnName=压电读取,skipAdsInit=0,skipFdcInit=1,sw=GND,expectedSwLevel=1,intrefExpected=0,vbiasExpected=0,expectedRefmux=0x24,vrefUv=5100000,adsRefPolicy=piezo_no_refout_no_vbias
DBGADSREFPOLICY,mode=PIEZO_READ,stage=piezo_no_ref_settled,...,powerIntref=0,powerVbias=0,expectedRefmux=0x24,vrefUv=5100000,refout=off,result=ok
DBGROUTEPOLICY,mode=PIEZO_READ,sw=GND,expectedSwLevel=1,adsIntRef=off,adsVbias=off,adsRefmux=0x24,vrefUv=5100000,refPrepPath=0,sela=ADS126X,fdcInitSkipped=1
DBGTMUXPOLICY,mode=PIEZO_READ,stage=voltage_scan_init,swSource=GND,cmdSwLevel=1,obsSwLevel=1,expectedSwLevel=1,...,result=ok
```

或电阻读取：

```text
APPMODE,active=RESISTANCE_READ,cnName=电阻读取,skipAdsInit=0,skipFdcInit=1,sw=REF,...,intrefExpected=1,vbiasExpected=1
DBGADSREF,stage=prepare_ref_bias_settled,...,intrefOk=1,vbiasOk=1,refmuxOk=1,result=ok
DBGROUTEPOLICY,mode=RESISTANCE_READ,sw=REF,...,adsIntRef=on,adsVbias=on,refPrepPath=1
```

## CSV

```text
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,...,S8D8
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<64 int32 microvolts>
```

点顺序始终为 `S1D1..S1D8,S2D1..S2D8,...,S8D8`。`timestamp_us` 是帧开始时刻，`duration_us` 是整帧耗时。可选输出：`MATV_RAW`、`MATV_GAIN`、`MATV_ERR`。

## 切换模式

```bash
idf.py menuconfig
```

- 默认压电读取：`SensorArray application mode -> PIEZO_READ / 压电读取`
- 电阻读取：`SensorArray application mode -> RESISTANCE_READ / 电阻读取`
- FDC debug：`SensorArray application mode -> Debug / bring-up modes`，再选择 `Debug execution mode -> S5D5 FDC secondary`、`FDC I2C discovery` 或其他 debug 项。

## 文件边界

- `main.c`: app mode 分流、PIEZO/RESISTANCE 主循环、CSV 输出、auto-gain table。
- `sensorarrayVoltageScan.c/.h`: 8x8 fast route 和 frame scan helper；新代码使用 `sensorarrayVoltageScan*WithSource()` 显式传入 `TMUX1108_SOURCE_REF/GND`。
- `sensorarrayBoardMap.c/.h`: 板级 mapping 单一真相源，SELA GPIO level 只能通过 `sensorarrayBoardMapSelaRouteToGpioLevel()` 转换。
- `sensorarrayBringup.c/.h`: ADS/FDC bring-up helper；REF/MID 准备路径只用于 `RESISTANCE_READ` 和 debug/bring-up 场景。
- `sensorarrayApp.c` 与 `sensorarrayDebug*.c`: 旧 bring-up/debug dispatcher，仅 DEBUG mode 使用。
