# main / Application Layer

`main/` 现在承担默认主业务编排。`main/main.c` 不再是薄入口；默认路径直接运行 ADS126x S1-S8 x D1-D8 电压矩阵扫描。旧 `sensorarrayAppRun()` 仍保留，但只在 `SENSORARRAY_APP_MODE_DEBUG=y` 时作为 debug dispatcher 入口。

## Default Flow

Voltage matrix scan 模式启动顺序：

1. `boardSupportInit()`
2. `tmuxSwitchInit()`，切到 ADS/resistive voltage path，当前 source 使用 `TMUX1108_SOURCE_REF`
3. `sensorarrayBringupInitAds()` / `sensorarrayBringupPrepareAdsRefPath()`
4. `ads126xAdcConfigureVoltageMode(defaultGain, dataRateDr, status=false, crc=false)`
5. `ads126xAdcStartAdc1()` continuous conversion
6. 默认执行一次 64 点 auto-gain bootstrap
7. 循环调用 `sensorarrayVoltageScanOneFrame()`，每帧输出一行 `MATV` CSV

默认模式不初始化 FDC2214，不执行 S5D5 polling/sweep/read loop。

## Files

- `main.c`: 默认 voltage scan 主循环、CSV 输出、auto-gain table bootstrap、Debug app mode 分流。
- `sensorarrayVoltageScan.c/.h`: 8x8 fast route 和 frame scan helper。
- `sensorarrayBoardMap.c/.h`: 板级 mapping 单一真相源。
- `sensorarrayBringup.c/.h`: ADS/FDC bring-up helper；默认 voltage path 只使用 ADS 相关函数。
- `sensorarrayMeasure.c/.h`: 旧 debug route/read helper，保留给 debug mode。
- `sensorarrayApp.c/.h` 与 `sensorarrayDebug*.c`: 旧 bring-up/debug dispatcher。

## CSV

```text
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,...,S8D8
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<64 int32 microvolts>
```

可选：`MATV_RAW`、`MATV_GAIN`、`MATV_ERR`。点顺序始终为 `S1D1..S1D8,S2D1..S2D8,...,S8D8`。`timestamp_us` 是帧开始时刻。

## Configuration

Top-level app mode：

- `SensorArray application mode -> Voltage matrix scan` 默认业务。
- `SensorArray application mode -> Debug / bring-up modes` 恢复旧 debug dispatcher。

Voltage tuning：

- `Voltage scan frame period in ms`: 0 表示最快。
- `Settle time after S row switch`
- `Settle time after SELA/SELB/SW path switch`
- `Settle time after ADS INPMUX switch`
- `Discard first conversion after mux switch`
- `Oversample count per point`
- `ADS126x ADC1 data rate DR code`
- `Voltage scan auto gain mode`

## Boundary Rules

- ADS component 只接收 `muxp/muxn`，不知道 `S1D1`。
- FDC component 只知道 FDC channel/register/profile，不知道 `S5D5`、SELA/SELB 或 I2C primary/secondary。
- `sensorarrayBoardMap.c` 负责 S/D 到 ADS mux 或 FDC channel 的板级映射。
- `sensorarrayVoltageScanApplyRouteFast()` 是 main 层 fast path，不调用旧 `sensorarrayMeasureApplyRoute()`，避免 debug 日志、ms delay 和每点 STOP/START。

## FDC Debug Recovery

在 menuconfig 中选择：

- `SensorArray application mode -> Debug / bring-up modes`
- `Debug execution mode -> S5D5 FDC secondary` 或 `FDC I2C discovery`

旧 debug 文件未删除，FDC component 中也保留并扩展了通用 helper。
