# SensorArray ESP32-S3 固件

SensorArray 是面向 8×8 高精度检测矩阵的 ESP-IDF 固件，目标芯片为 ESP32-S3，当前支持三种互斥测量模式：

- `CAP`：使用双 FDC2214 采集电容；
- `VOLT`：使用 ADS1262/ADS1263 ADC1 采集节点电压；
- `RES`：使用 ADS1262/ADS1263 ADC1、参考电阻与矩阵 `REFOUT` 激励计算电阻。

固件保持双 FDC worker、Core 1 采集和 Core 0 输出的异步架构。Serial、BLE 与 Wi-Fi 控制面共用同一命令处理入口；数据与日志采用有界、非阻塞传输，慢速网络不能反压采集任务。

仓库地址：<https://github.com/ErrorChen/SensorArray>

## 当前能力

| 能力 | 状态 | 说明 |
| --- | --- | --- |
| 8×8 CAP | 支持 | 双 FDC2214，输出 `C/D/K` 文本帧 |
| 8×8 VOLT | 支持 | ADS ADC1，输出 `V/D/P/K` 文本帧 |
| 8×8 RES | 支持 | ADS ADC1、自动 PGA/旁路与显式无效原因，输出 `R/D/P/K` 文本帧 |
| Serial 控制、数据、日志 | 支持 | USB Serial/JTAG 文本通道 |
| BLE 控制、数据、日志 | 支持 | Service `0x00FF`；`FF10/FF11/FF20/FF30` |
| Wi-Fi SoftAP + UDP | 支持 | DATA `3333`、LOG `3334`、CTRL `3335` |
| `ROWMODES` mixed-row | 支持 | 每个 S 行独立选择 `C`/`V`/`R`，输出 `M/MR/K` |
| Wi-Fi STA/APSTA 与配网保存 | 未实现 | 命令明确返回 `ERR,cmd=WIFI,reason=sta_nyi` |

## 快速开始

### 1. 环境

- ESP-IDF `v5.5.1`；
- ESP32-S3 toolchain；
- 支持 USB Serial/JTAG 的数据线；
- 可选：Python 虚拟环境，用于协议测试与 HIL 验证。

在已激活 ESP-IDF 5.5.1 的 PowerShell 中执行：

```powershell
git clone https://github.com/ErrorChen/SensorArray.git
Set-Location SensorArray
idf.py set-target esp32s3
idf.py fullclean
idf.py build
```

### 2. 烧录与监视

```powershell
idf.py -p COMx flash
idf.py -p COMx monitor
```

将 `COMx` 替换为实际端口。不要在存在多个候选设备时盲目选择端口。

### 3. Serial 快速检查

向串口发送 ASCII 命令，每条命令以换行结束：

```text
MODE?
STATE?
TX?
ST?
BTX?
WIFI?
MODE=CAP
MODE=RES
ROWMODES?
ROWMODES=CVVRRVVC
RAILCFG=<AVDD_UV>,<negative_AVSS_UV>   # 可选 debug override
MODE=VOLT
MODE=CAP
```

`MODE=<name>` 会先返回 `MACK ... state=accepted`，再由 Core 1 在完整帧边界应用并输出 `MAPP ... state=applied`。主机必须等待匹配的 request ID，不能把 accepted 当作已经切换。

进入 `VOLT` 时固件会在矩阵隔离的 `SAFE_RAIL_MONITOR` 窗口通过 ADS126x internal analog supply monitor 自动获取 rail span，再恢复目标 route；普通用户不需要先发 `RAILCFG`。`RAILCFG` 仅作为可选的、易失的外部 DMM debug override，且不能在 VOLT route 内应用。电池断开时 AIN8 的 PWM/浮动/异常电压只应表现为 `valid=0,reason=...`，不会清除 last-good 或污染 CAP/VOLT/RES frame。

### 4. BLE 快速检查

使用 nRF Connect 或支持 GATT 的客户端：

1. 扫描 `CscArray_*`；
2. 连接 Service `0x00FF`，客户端支持时请求 MTU 247；
3. 为 `FF11`、`FF20`、`FF30` 同时开启 Notify 与 Indicate（CCCD `0x0003`）；
4. 向 `FF10` 写入 `STATE?\n`；
5. 在 `FF11` 收控制回复，在 `FF20` 收测量数据，在 `FF30` 收诊断日志；
6. 向 `FF10` 写入 `BTX=SAFE\n`，确认 `FF11` 通过 Indicate 返回 ACK，再用 `BTX?` 核对状态。

`BTX=FAST` 使用 Notify，`BTX=SAFE` 使用带确认的 Indicate。大于当前 ATT payload 的消息使用 `G,...` envelope 分片，主机必须完成重组、总长度检查和 CRC32 校验。

不要在 FF11 只有 Notify bit 时直接切到 SAFE：firmware 会先应用新模式，再按 SAFE 的 Indicate gating 发送本条命令的回复，因此命令可能已经生效但 ACK 无法送达。切换前先开启 FF11 Indicate；需要在 SAFE 下接收 DATA/LOG 时，也要先为 FF20/FF30 开启 Indicate。完整 nRF Connect 验收顺序见 [验证](docs/validation.md)。

详见 [BLE 协议](docs/ble-protocol.md)。

## 三种测量模式

| 模式 | 前端 | 矩阵激励 | ADS reference | 典型输出 |
| --- | --- | --- | --- | --- |
| `CAP` | FDC2214 | GND，无 `REFOUT` 激励 | `INTREF=0`，`REFMUX=AVDD/AVSS` | pF |
| `VOLT` | ADS ADC1 | GND，无 `REFOUT` 激励 | `INTREF=0`，`REFMUX=AVDD/AVSS` | µV |
| `RES` | ADS ADC1 | `REFOUT` | `INTREF=1`，内部 reference | mΩ wire value |

矩阵 `REFOUT`、ADS `INTREF`、`REFMUX`、`VBIAS` 和 SW 物理电平是彼此独立的状态，不能互相推导。完整路由与切换规则见 [测量模式](docs/measurement-modes.md) 和 [硬件概览](docs/hardware-overview.md)。

## 文档索引

- [快速入门](docs/getting-started.md)
- [软件架构](docs/architecture.md)
- [硬件概览](docs/hardware-overview.md)
- [测量模式](docs/measurement-modes.md)
- [完整命令参考](docs/command-reference.md)
- [通用 transport 协议](docs/transport-protocol.md)
- [BLE GATT 与分片协议](docs/ble-protocol.md)
- [测量帧与诊断日志格式](docs/output-format.md)
- [构建、自动测试与 HIL 验证](docs/validation.md)
- [故障排查](docs/troubleshooting.md)
- [上位机兼容清单](docs/software-integration.md)

## 仓库结构

```text
components/                 芯片与链路组件：ADS126x、FDC2214、BLE、Wi-Fi、TMUX
core/board/                 板级引脚、路由与模式 profile 真源
core/boardSupport/          I2C、SPI、GPIO 与中断资源
core/config/                动态行数配置
core/measure/               模式状态机、FDC/ADS 测量与数学逻辑
core/transport/             DATA/LOG/CTRL 策略、队列与 sink 统计
main/control/               跨 transport 命令 mailbox
main/output/                文本协议、异步日志与 USB sink
main/                       应用生命周期与采集任务
tools/                      Host parser、自动测试与 HIL 工具
tests/host/                 硬件无关 C 测试
docs/                       用户和集成文档
```

## 协议兼容性

当前兼容目标：

- BLE UUID 不变：`00FF`、`FF10`、`FF11`、`FF20`、`FF30`；
- 命令入口仍为 Serial、BLE `FF10` 和 Wi-Fi UDP `3335`；
- CAP `C/D/K`、VOLT/RES `V|R/D/P/K` 格式保持稳定；
- BLE `G,<channel>,...` 分片 envelope 保持稳定；
- 所有运行时设置均为易失状态，重启后恢复编译配置默认值。

## 已知限制

- Wi-Fi 只实现 SoftAP/UDP；STA、APSTA、扫描、凭据保存与自动连接均为 NYI。
- `RAILCFG`、模式、输出选择、BLE TX 模式和校准请求不会写入 NVS。
- 测量精度必须结合当前硬件、校准值和 DMM/标准件验收；固件中的调试板标称值不是生产校准。
- 编译通过不等于 Serial/BLE HIL 通过。发布报告必须分别给出链路、CRC、重启检测、长跑和 stack high-water 证据。

## 协作与贡献

提交修改时请同时更新源码、自动测试和对应文档。任何新增或删除的用户命令都必须通过：

```powershell
python tools/check_command_docs.py
```

涉及硬件路由、测量精度、BLE CCCD 或长跑稳定性的结论必须附真实设备证据；无法执行的测试应明确标为未验证，不能用静态分析代替。

## 许可证

许可证见 [LICENSE](LICENSE)。
