# Hardware validation / 硬件验证

## 中文

使用 ESP-IDF 5.5.1 和仓库 `.venv`。不要修改生成的根 `sdkconfig`；建议使用隔离
build 目录和其自身配置文件。

```powershell
idf.py -B build_modes -D SDKCONFIG=build_modes\sdkconfig.validation reconfigure
idf.py -B build_modes -D SDKCONFIG=build_modes\sdkconfig.validation build
idf.py -B build_modes -D SDKCONFIG=build_modes\sdkconfig.validation -p <PORT> flash
.\.venv\Scripts\python.exe tools\validate_measurement_modes.py `
  --port <PORT> --baud 115200 --duration 240 `
  --rows 1,2,4,8 --modes CAP,VOLT,RES --cycles 10 `
  --rail-avdd-uv <measured-avdd-uv> `
  --rail-avss-uv <measured-negative-avss-uv> `
  --output-directory validation_artifacts\measurement-modes
```

当前验证器先执行 `ADS?`/`ADSCHK=100`，要求 ADS1262、ADC1 available、ADC2
unavailable、100 fresh、0 stale/timeout/reset/status/SPI error、restore ok。随后对
ROWS 1/2/4/8 和 CAP/VOLT/RES 收集完整 CRC/fresh frame，运行至少 10 个 mode cycle；
8×8 VOLT/RES 另收集默认 70 帧，依据 `ADS50/ADST50` 检查 profile hit >=95%、
attempts/cell <=1.25、raw conversions 接近 64、frameUs 分别 <50/60 ms，并保存
各阶段耗时。默认还在每个模式运行 120 秒、`BATPERIOD=1000`，比较 scheduler
run count、restore failure、mode 不变和每帧 freshness。可显式设置：

```powershell
--performance-frames 70 --battery-dwell-seconds 120 `
--res-settle-values 2000,1000,500,200,100,50 --res-settle-frames 6
```

RES settle sweep 会记录 S1D1/S8D8 第一帧与后续均值、标准差、相邻行差、open/
short 分类和 FPS。只有同时传入当次万用表值且满足 tolerance 时才选择最短档；
缺少 DMM 时工具恢复原 settle 并标为未验证。电池准确度也必须同步记录
VBAT-GND、AIN8-GND、AINCOM-GND 和实测 divider ratio；没有这三项只能验证调度、
freshness 和 restore，不能宣称电压精度通过。

`--rail-avdd-uv`/`--rail-avss-uv` 必须来自当前板的测量或明确的本次验证输入；验证器
通过共享命令 parser 发送 `RAILCFG`，等待 Core 1 帧边界 `RAPP` 后才请求 VOLT。
这两个值不是生产固件常量。若没有可靠 rail 输入，VOLT 应拒绝进入而不是使用
3.3 V/-1.8 V 标称值。

验证器检查 startup `MSELF/PSELF/ADSBOOT`、C/V/R packet CRC、rows/cell count、
fresh/error、`MACK/MAPP` 帧边界、`MODE?` 物理状态、`CELL?` telemetry 和至少十轮
`CAP -> VOLT -> RES -> CAP`。原始串口和 JSON summary 保存在被 Git 忽略的
artifact 目录。`ADSBOOT` 发生在有界 FDC boot sweep 之前；工具会在独立的
`--startup-timeout`（默认 60 秒）内等待第一个 CRC 正确、双 FDC freshness 完整的
CAP 帧，再发 MODE 请求，不能把“已 accepted、仍在启动校准”误报成切换失败。

当前调试板的 S1D1/S8D8 约 10 kΩ、S4D4/S5D5 为电容，仅是可选验收输入，不是
固件假设。获得万用表实测值后才传入：

```powershell
--known-resistor S1D1=<measured-ohms> --known-resistor S8D8=<measured-ohms> `
--known-capacitor S4D4 --known-capacitor S5D5
```

没有实测值时不得用标称 10000 冒充验收基准。VOLT/RES 还必须检查 raw/nodeUv/
AVDD/AVSS/Vref/Rref/PGA/error，确认容性 cell 在有限时间内成为 unstable/open/
invalid 而不是稳定约 10 kΩ，并记录 FPS、frame/transition duration、heap、stack、
queue/drop、reset/WDT。BLE/Wi-Fi 只有在主机和链路可用时另外验证；Serial 通过不
等于它们已验证。

### 2026-08-05 COM12 验证快照

隔离 build/flash 后，`validation_artifacts/20260805_full_modes_retry` 的结构化验证
完成 rows 1/2/4/8、CAP/VOLT/RES 和 10 轮 `CAP -> VOLT -> RES -> CAP`，总计
45 次 mode transition，`passed=1`、CRC/freshness failure 为 0、reset/WDT 为 0。
CAP 的 S4D4/S5D5 分别为 202.747930 pF 和 72.521560 pF；RES 的 S1D1/S8D8
分别为 10042.859 Ω 和 10039.919 Ω，S4D4/S5D5 在有限时间内报告 open/invalid。
测试 rail 输入 3391500/-1796500 uV 取自用户给出的当次实测范围中点，随后 RES
内部 monitor 刷新为 Vss=-1822906 uV、Vref=677094 uV。由于没有同步记录万用表
对 S1D1/S8D8 的实际阻值，按器件实测值/公差的最终准确度验收仍是**未验证**；
BLE 和 Wi-Fi 实际链路也是**未验证**。这些结果仅描述当前调试板，不构成坐标或
标称 rail 的生产假设。

## Australian English

Build and flash with ESP-IDF 5.5.1, then run the structured validator rather
than judging monitor output by eye. Known components are optional command-line
expectations only. Use measured resistor values and component tolerance; never
promote nominal values into production logic. Preserve raw logs and JSON under
the ignored validation-artifact directory. Supply measured AVDD and negative
AVSS explicitly; the validator applies them through `RAILCFG` at a frame
boundary. It waits for a fresh dual-FDC CAP frame after the bounded boot sweep,
not merely for the earlier ADS identity line. The 2026-08-05 COM12 run passed all three modes and ten cycles, but
resistor accuracy against simultaneous DMM values and real BLE/Wi-Fi links
remain unverified. A missing port, board, or link must be reported as unverified.
