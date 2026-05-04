# SensorArray Firmware

当前默认分支是 `main`。本轮修改要求的基线是 `8c7c35b` (`ChangeToVoltage`) 或它之后的小回滚 `1f18cb5` (`Rollback`)；当前主线继续保留 ADS126x 电压读取能力，不回到旧 FDC2214 调试默认路径。

## 当前默认模式

默认运行模式是 `PIEZO_READ / 压电读取`：

- `PIEZO_READ / 压电读取`: ADS126x voltage scan，读取 `S1-S8 x D1-D8`，ADS126x internal reference / REFOUT 关闭，ADC reference 切到 AVDD/AVSS，TMUX1108 `SW=HIGH`，软件 source 名称为 `GND` / zero path。
- `RESISTANCE_READ / 电阻读取`: ADS126x voltage scan，读取 `S1-S8 x D1-D8`，保留 REF/MID 路径，ADS126x internal reference / VBIAS 开启，软件 source 名称为 `REF`。
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

- `SensorArray application mode -> PIEZO_READ / 压电读取: ADS126x voltage scan with SW=HIGH GND/zero, REFOUT off`
- `SensorArray application mode -> RESISTANCE_READ / 电阻读取: ADS126x voltage scan with SW=REF/MID`
- `SensorArray application mode -> Debug / bring-up modes`

`ADS126x voltage scan` 菜单保留高速扫描参数：ADS126x data rate、row/path/mux settle、discard first、oversample、frame period、auto gain，以及 raw/gain/error 可选输出。旧 `VOLTAGE_MATRIX_SCAN` symbol 保留为兼容别名，默认含义已迁移为 `PIEZO_READ`。

## Build / Flash / Monitor

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 运行日志和 CSV

默认 `PIEZO_READ` 启动时应看到：

```text
APPMODE,active=PIEZO_READ,cnName=压电读取,skipAdsInit=0,skipFdcInit=1,sw=GND,expectedSwLevel=1,intrefExpected=0,vbiasExpected=0,expectedRefmux=0x24,vrefUv=5100000,adsRefPolicy=piezo_no_refout_no_vbias
DBGADSREFPOLICY,mode=PIEZO_READ,stage=piezo_no_ref_settled,...,powerIntref=0,powerVbias=0,expectedIntref=0,expectedVbias=0,expectedRefmux=0x24,vrefUv=5100000,refout=off,result=ok
DBGROUTEPOLICY,mode=PIEZO_READ,sw=GND,expectedSwLevel=1,adsIntRef=off,adsVbias=off,adsRefmux=0x24,vrefUv=5100000,refPrepPath=0,sela=ADS126X,fdcInitSkipped=1
DBGTMUXPOLICY,mode=PIEZO_READ,stage=voltage_scan_init,swSource=GND,cmdSwLevel=1,obsSwLevel=1,expectedSwLevel=1,...,result=ok
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,...,S8D8
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<64 int32 microvolts>
```

切换到 `RESISTANCE_READ` 后，`APPMODE` 和 `DBGROUTEPOLICY` 会显示 `sw=REF`，并继续通过 `DBGADSREF ... prepare_ref_bias_settled` 验证 internal REF、VBIAS 和 internal REFMUX。CSV 仍使用 `MATV_HEADER` / `MATV`，点顺序为 `S1D1..S1D8,S2D1..S2D8,...,S8D8`。可选输出包括 `MATV_RAW`、`MATV_GAIN`、`MATV_ERR`。

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

`DBGADSREF ... result=ok` 只适用于需要 REF/MID 的电阻读取路径，说明固件已经配置并读回 ADS126x internal REF、VBIAS/AINCOM level shift 和 REFMUX。它不等价于板上模拟 `REF/MID` 节点一定已经到达 `0.75 V`。如果 readback OK 但示波器仍约 `0.4 V`，应检查硬件 REF/MID 网络、负压、焊接、buffer、SW 极性、TMUX 实际通路和模拟负载。
