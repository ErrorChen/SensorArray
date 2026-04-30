# SensorArray Firmware

当前默认分支是 `main`。本轮修改要求的基线是 `8c7c35b` (`ChangeToVoltage`) 或它之后的小回滚 `1f18cb5` (`Rollback`)；当前主线继续保留 ADS126x 电压读取能力，不回到旧 FDC2214 调试默认路径。

## 当前默认模式

默认运行模式是 `PIEZO_READ / 压电读取`：

- `PIEZO_READ / 压电读取`: ADS126x voltage scan，读取 `S1-S8 x D1-D8`，`SW=GND`。
- `RESISTANCE_READ / 电阻读取`: ADS126x voltage scan，读取 `S1-S8 x D1-D8`，`SW=REF`。
- `DEBUG / bring-up`: 旧 debug dispatcher 和 FDC2214 bring-up 入口。

`main/main.c` 仍是默认主程序，`app_main()` 仍在 `main/main.c`，不是只调用 `sensorarrayAppRun()` 的薄入口。默认 `PIEZO_READ` 和 `RESISTANCE_READ` 都不会初始化或轮询 FDC2214；旧 FDC2214/S5D5 调试代码仍保留，只能通过 Debug / bring-up mode 进入。

## 目录入口

- [main/README.md](main/README.md): app mode 选择、默认主循环、CSV 输出。
- [components/ads126xAdc/README.md](components/ads126xAdc/README.md): 通用 ADS1262/ADS1263 SPI 驱动、REF/VBIAS、fast voltage read、raw->uV、auto gain。
- [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md): TMUX1108/TMUX1134 GPIO 控制、`GND`/`REF` source 语义。
- [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md): 通用 FDC2214 I2C 驱动和旧 debug 入口边界。
- [core/boardSupport/README.md](core/boardSupport/README.md): 板级 I2C 总线和回调适配。

## Menuconfig

```bash
idf.py menuconfig
```

应用模式：

- `SensorArray application mode -> PIEZO_READ / 压电读取: ADS126x voltage scan with SW=GND`
- `SensorArray application mode -> RESISTANCE_READ / 电阻读取: ADS126x voltage scan with SW=REF`
- `SensorArray application mode -> Debug / bring-up modes`

## SW 极性与模式关系

SensorArray board SW polarity:

```text
SW = LOW  -> REF
SW = HIGH -> GND
```

Mode requirements:

- `PIEZO_READ / 压电读取`: requires GND source, therefore SW = HIGH.
- `RESISTANCE_READ / 电阻读取`: requires REF source, therefore SW = LOW.
- `DEBUG / FDC2214`: uses GND unless the selected debug submode explicitly resolves another source.

`ADS126x voltage scan` 菜单保留高速扫描参数：ADS126x data rate、row/path/mux settle、discard first、oversample、frame period、auto gain，以及 raw/gain 可选输出；错误帧固定输出 `MATV_ERR`。旧 `VOLTAGE_MATRIX_SCAN` symbol 保留为兼容别名，当前等价于 `PIEZO_READ`，source 为 `SW=GND`。

## Build / Flash / Monitor

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 运行日志和 CSV

默认 `PIEZO_READ` 启动时应看到：

```text
APPMODE,compiled=PIEZO_READ,entry=main_fast_scan,swSource=GND,expectedSwLevel=1
APPPOLICY,appMode=PIEZO_READ,isDebugAppMode=0,mode=PIEZO_VOLTAGE,requiredSwSource=GND,requiredSwLevel=1,reason=piezo_requires_ground_source
DBGSYSREF,vdd_mv=3300,vss_mv=-1800,span_mv=5100,target_mid_gnd_mv=750,target_mid_avss_mv=2550
DBGADSREF,stage=prepare_ref_bias_settled,...,intrefOk=1,vbiasOk=1,refmuxOk=1,result=ok
DBGADSREF_ANALOG,stage=ain9_aincom_short,ain9_aincom_uv=...,expectedNearZero=1,...,result=ok
ROUTEPOLICY,mode=PIEZO_VOLTAGE,swSource=GND,expectedSwLevel=1,cmdSwLevel=1,obsSwLevel=1,sela=ADS126X,fdcInitSkipped=1
VOLTSCAN_INIT,mode=PIEZO_VOLTAGE,appMode=PIEZO_READ,cnName=压电读取,swSource=GND,expectedSwLevel=1,...
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,...,S8D8
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<64 int32 microvolts>
```

切换到 `RESISTANCE_READ` 后，`APPMODE`、`APPPOLICY` 和 `ROUTEPOLICY` 会显示 `swSource=REF,expectedSwLevel=0`。CSV 仍使用 `MATV_HEADER` / `MATV`，点顺序为 `S1D1..S1D8,S2D1..S2D8,...,S8D8`。错误帧会输出 `MATV_ERR`。

## REF / MID 说明

当前硬件目标：

```text
VDD = +3.3 V
VSS = -1.8 V
supply_span_mv = 5100
target_mid_gnd_mv = (3300 + (-1800)) / 2 = 750
target_mid_avss_mv = 750 - (-1800) = 2550
```

不要把 `(VDD + |VSS|) / 2 = 2.55 V` 当成相对 GND 的 midpoint；`2.55 V` 是相对 `VSS/AVSS` 的距离。

`DBGADSREF ... result=ok` 只说明固件已经配置并读回 ADS126x internal REF、VBIAS/AINCOM level shift 和 REFMUX。启动还会输出 `DBGADSREF_ANALOG`：先用 ADS 读取板上已知短接的 `AIN9-AINCOM`，确认差分接近 0；如果没有 REF/MID 的可测 mux 节点，还会输出 `register_ok_but_external_ref_not_verified`。如果 readback OK 但示波器仍约 `0.4 V`，应检查硬件 REF/MID 网络、负压、焊接、buffer、SW 极性、TMUX 实际通路和模拟负载。
