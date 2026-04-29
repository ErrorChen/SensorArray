# SensorArray Firmware

当前默认运行模式已切换为 ADS126x 电压矩阵高速扫描：固件启动后初始化 boardSupport、TMUX 和 ADS126x，不主动初始化或轮询 FDC2214，然后连续读取 S1-S8 x D1-D8 的 8x8 电压矩阵，并通过串口输出一帧 64 点 CSV。

## Project Structure

- `components/ads126xAdc`: 通用 ADS1262/ADS1263 SPI 驱动，包含电压模式、fast read、raw->uV 和 auto-gain API；不包含 D-line 或 S/D mapping。
- `components/fdc2214Cap`: 通用 FDC2214 I2C 驱动，包含已固化的单通道连续转换、relaxed/status 读样、频率/电容换算、drive-current sweep/lock helper；不包含 S5D5 或总线归属。
- `components/tmuxSwitch`: 通用 TMUX1108/TMUX1134 GPIO 控制。
- `main/main.c`: 当前默认主业务，编排 ADS126x 8x8 voltage matrix scan。
- `main/sensorarrayVoltageScan.c`: 8x8 voltage fast route 和 frame scan helper。
- `main/sensorarrayBoardMap.c`: 板级 mapping 单一真相源，例如 `D1..D8 -> ADS AIN0..AIN7, negative=AinCOM`。
- `main/sensorarrayApp.c` 与 `main/sensorarrayDebug*.c`: 旧 bring-up/debug dispatcher，仅在 Debug app mode 下运行。

## Build / Flash / Monitor

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py flash monitor
```

常用 menuconfig：

- `SensorArray application mode -> Voltage matrix scan`
- `Voltage scan frame period -> 0` 表示尽可能快输出。
- `ADS126x ADC1 data rate DR code -> 15` 表示 38400 SPS。
- `Voltage scan auto gain mode -> Bootstrap once at startup`
- `Output raw/gain -> optional`

## CSV Output

Header 固定 row-major 顺序：

```text
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,S1D2,...,S1D8,S2D1,...,S8D8
```

默认数据行一帧一行，64 个值都是 `int32_t microvolts`：

```text
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<S1D1_uV>,...,<S8D8_uV>
```

可选行：

```text
MATV_RAW,<seq>,<timestamp_us>,<raw_S1D1>,...,<raw_S8D8>
MATV_GAIN,<seq>,<timestamp_us>,<gain_S1D1>,...,<gain_S8D8>
MATV_ERR,<seq>,<timestamp_us>,<err_S1D1>,...,<err_S8D8>
```

`timestamp_us` 使用 `esp_timer_get_time()`，记录帧扫描开始时刻；`duration_us` 是帧结束减去该开始时刻。上位机转 volts 时除以 `1e6`。

## Auto Gain

ADS126x differential full-scale range是 `+-VREF/Gain`。默认 VREF=2.5 V，因此 gain 1/2/4/8/16/32 分别约为 `+-2.5 V`、`+-1.25 V`、`+-0.625 V`、`+-0.3125 V`、`+-0.15625 V`、`+-0.078125 V`。

默认 `BOOTSTRAP_ONCE`：启动时对 64 点各自选择一次最高但保留 headroom 的 gain，并保存到 per-point gain table，后续高速扫描复用。选择条件是：

```text
abs(input_uv) * gain < vref_uv * headroom_percent / 100
```

若后续 raw 接近满量程，frame 会标记 `clipped`，并可通过启用 `MATV_GAIN`/`MATV_ERR` 观察。`PER_FRAME` 和 `PER_POINT` 会重新评估 gain，但会显著降低扫描频率。

## Performance Tuning

默认高频路径避免每点 STOP/START、寄存器 dump 和逐点 printf。主要调参项：

- `SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE`: 默认 15，即 38400 SPS。
- `SENSORARRAY_VOLTAGE_SCAN_ROW_SETTLE_US`
- `SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US`
- `SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US`
- `SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST`: 默认 y；通道切换后更可靠，但每点至少多消耗一次 conversion。
- `SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE`: 默认 1。
- `SENSORARRAY_VOLTAGE_SCAN_FRAME_PERIOD_MS`: 默认 0，尽可能快。

38400 SPS 单次 conversion 理论周期约 26 us，64 点单样本理论下限约 1.67 ms；`discardFirst=y` 后至少翻倍。实际速度还受 TMUX settle、SPI transaction、DRDY wait、CSV printf 和 auto-gain 模式影响。

## Switch Back To FDC2214 Debug

默认 app 不再进入 S5D5/FDC discovery/FDC polling/sweep/read loop。需要恢复旧调试时：

- `SensorArray application mode -> Debug / bring-up modes`
- `Debug execution mode -> S5D5 FDC secondary` 或 `FDC I2C discovery` / `FDC self-test`

这样会重新走 `sensorarrayAppRun()` 和旧 debug dispatcher。FDC debug 源码仍保留。

## Hardware Notes

电压扫描 fast route 当前使用 `TMUX1108_SOURCE_REF` 作为 voltage source，并按已查验语义设置 `SELA=ADS1263 branch`、`SELB=0/resistive-ADS branch`。`SW_SOURCE_REF/GND` 的真实模拟语义仍建议后续用示波器确认；代码中也保留了 TODO，确认前不应静默反转。
