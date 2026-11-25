# SensorArray · ESP32-S3 矩阵测量固件 / Matrix measurement firmware

SensorArray 扫描最多 8 行 × 8 列的传感阵列。每个活动行可独立选择电容、电压或电阻模式。两个 FDC2214 并行读电容，ADS126x 的 ADC1 读电压、电阻与电池。采集与发送独立运行，正式数据可经原生 USB-OTG 或 BLE 接收。 / SensorArray scans up to 64 cells. Each active row can independently measure capacitance, voltage or resistance. Two parallel FDC2214s cover the eight capacitance channels; ADS ADC1 measures voltage, resistance and battery. Acquisition runs independently of USB-OTG, BLE and optional SoftAP/UDP output.

[构建与首次连接 / Get started](docs/getting-started.md) · [命令速查 / Commands](docs/command-reference.md) · [测试方法 / Test methods](docs/validation.md) · [全部迭代和实测结果 / History and measurements](docs/history/README.md)

## 从上电到一帧 / From boot to a frame

1. **识别前端 / Qualify the frontends.** ESP32-S3 启动时并行验证 I²C 和 SPI 速率，保存实际时钟、成功样本和 CRC 拒收计数。三颗前端失联时暂停测量，低速探测返回后重新初始化、选速。 / Boot qualification selects usable bus rates from actual reads. Loss of all three frontends suspends acquisition until presence and requalification succeed.
2. **按行测量 / Acquire by row.** `ROWS=1..8` 选择活动行数，`ROWMODES=CVRCVRCV` 给八个物理行分配模式。CAP 输出总 pF；VOLT 输出整数 µV；RES 输出整数 mΩ。ADS 的 shadow、采样缓存与自动 PGA 使用已验证的新转换。 / The scan owner snapshots configuration at frame start, routes each row, and preserves freshness and invalid reasons. A reported OPEN or failed CRC remains visible in the data.
3. **校准与辅助事务 / Calibrate and monitor.** 此版电路 U5 始终选通，透明内部 rail 测量不可用。VOLT 使用有效的对地双轨实测值。参考建立、输入建立、新鲜转换和电池恢复分别验证。 / The board lacks an all-open reference route. Use measured rails for VOLT; distinguish restoration of registers from analog settling. The measurement guides describe offset, baseline and battery handling.
4. **解码与恢复 / Decode and recover.** CAP/VOLT/RES 或混合帧携带采集时的配置身份与校验。订阅 BLE 或打开正式 CDC 后先查询 PROTO?、BUILD?、CONFIG?。ACK、配置应用、主机首个完整帧分别观测。 / Correlate replies by session and domain. Validate the complete scientific CRC and masks at the receiver, including after gaps, delayed terminals or reconnection.

## 构建与硬件 / Build and board

从已激活的 ESP-IDF **5.5.5** 终端在仓库根目录构建。当前默认目标 ESP32-S3，16 MB Flash、自定义分区、PSRAM；启用配置以实际 sdkconfig 为准。 / Build from an ESP-IDF 5.5.5 shell. The defaults target ESP32-S3 with 16 MB flash, a custom partition table and PSRAM. Check the effective configuration and running BUILD reply for a specific board.

```sh
idf.py -D IDF_TARGET=esp32s3 build
```

[电路图 / Schematic](datasheets/circuit.pdf) · [连通与电气限值 / Connectivity and electrical limits](docs/hardware/reliability_revision_notes.md) · [ESP-IDF 配置 / Defaults](sdkconfig.defaults) · [许可证 / License](LICENSE)

## README 总索引 / All module manuals

每个固件模块手册列出职责、函数签名、参数和调用入口。应用开发从 main 或 core/measure 进入；改驱动时先看对应芯片组件；写上位机从契约与编码器进入。 / Each firmware manual documents its functions and call sites. Start with the application or measurement owner for firmware changes, the chip component for bus-level changes, and the host contract and encoder for a receiver.

### 应用与采集 / Application and acquisition

| 手册 / Manual | 负责的工作 / Responsibility |
|---|---|
| [main](main/README.md) | 上电、任务生命周期、帧边界仲裁和恢复 / Startup, task lifecycle, frame boundaries and recovery. |
| [main/control](main/control/README.md) | 命令解析、邮箱和 USB 控制 / Command mailbox and USB control. |
| [main/net](main/net/README.md) | 应用网络适配 / Application networking adapters. |
| [main/output](main/output/README.md) | 科学帧编码、异步日志及各输出所有者 / Scientific encoding, async logging and output ownership. |
| [core/board](core/board/README.md) | 行列、引脚和路由真值 / Pin, channel and route definitions. |
| [core/boardSupport](core/boardSupport/README.md) | 双 I²C 总线、GPIO 与中断资源 / Board I²C buses, GPIO and interrupt resources. |
| [core/config](core/config/README.md) | 行数配置、快照及请求应用 / Active and pending row configuration. |
| [core/matrixEngine](core/matrixEngine/README.md) | 通用矩阵扫描接口 / Matrix scanning interface. |
| [core/powerCtrl](core/powerCtrl/README.md) | 预留电源使能、充电及电源正常引脚配置 / Reserved power-enable, charging and power-good pin configuration. |
| [core/measure](core/measure/README.md) | 模式、路由、计划、校准与恢复 / Modes, routing, scan plans, calibration and recovery. |
| [core/measure/fdc](core/measure/fdc/README.md) | 双 FDC 逐行 epoch、标定和救援 / Dual-FDC epochs, calibration and rescue. |
| [core/measure/ads](core/measure/ads/README.md) | ADC1、自动 PGA、rail、电池与新鲜度 / ADC acquisition, PGA, rails, battery and freshness. |
| [core/measure/mixed](core/measure/mixed/README.md) | 混合 profile 的行调度和汇总 / Row-subset scheduling and mixed-frame assembly. |
| [core/transport](core/transport/README.md) | 各通路准入、背压、命令与终态 / Sink admission, backpressure, commands and terminals. |

### 器件、网络与协议库 / Devices, transports and libraries

| 手册 / Manual | 负责的工作 / Responsibility |
|---|---|
| [ads126xAdc](components/ads126xAdc/README.md) | ADS SPI、DMA、寄存器、DRDY 与 CRC / ADC driver, conversion and integrity checks. |
| [fdc2214Cap](components/fdc2214Cap/README.md) | FDC 配置、状态、四通道读回 / FDC register profiles and four-channel readout. |
| [tmuxSwitch](components/tmuxSwitch/README.md) | TMUX 行选择和前端开关 / Multiplexer addressing and frontend switches. |
| [sensorarrayBle](components/sensorarrayBle/README.md) | GATT、订阅、优先队列、分片与保护帧 / GATT, subscriptions, scheduling, fragmentation and full snapshots. |
| [sensorarrayUsbOtg](components/sensorarrayUsbOtg/README.md) | OTG 接口、整帧传送及重枚举 / USB controller, whole-record transmission and recovery. |
| [sensorarrayWifi](components/sensorarrayWifi/README.md) | SoftAP、UDP 和独立 SDK owner / SoftAP, UDP and isolated SDK execution. |
| [sensorarrayWorkBudget](components/sensorarrayWorkBudget/README.md) | FreeRTOS 工作预算与 CPU 让出 / Task work budgets and CPU scheduling. |
| [protocolUsb](transport/protocolUsb/README.md) | 独立 USB 协议库 / Stand-alone binary USB protocol library. |
| [protocolWire](transport/protocolWire/README.md) | 二进制 wire 编解码 / Binary wire codec. |
| [transportWire](transport/transportWire/README.md) | 独立传输配置项 / Transport configuration surface. |
| [example](example/README.md) | Espressif 独立 BLE 吞吐示例及回调 / Stand-alone BLE throughput example and callbacks. |

### 主机工具与固定输入 / Host tools and fixtures

| 手册 / Manual | 内容 / Contents |
|---|---|
| [tools](tools/README.md) | 工作站命令、parser、诊断与测试入口 / Workstation utilities, parsers and diagnostic tools. |
| [tools/hil](tools/hil/README.md) | 正式 USB/BLE 验收、身份校验及离线分析 / Reusable acceptance collectors, identity checks and offline analysis. |
| [tests](tests/README.md) | Host C 回归、stub 和固定输入 / Host regression suites, substitutes and input provenance. |
| [ads_fault](tests/fixtures/ads_fault/README.md) | formatter 生成的故障回放 / Production-formatted synthetic faults. |
| [ble_crc_planner](tests/fixtures/ble_crc_planner/README.md) | CRC 与分片基准实现 / CRC and fragment-planner reference implementations. |
| [config_identity](tests/fixtures/config_identity/README.md) | 真 encoder 生成的配置与科学帧金样 / Real encoder configuration and frame goldens. |
| [reference_capture](tests/fixtures/reference_capture/README.md) | 带来源和偏移的实录字节片段 / Immutable device-recorded frame slices. |
| [transport_frame_meta](tests/fixtures/transport_frame_meta/README.md) | 发送队列元数据解析基准 / Differential metadata parser baseline. |
| [history](docs/history/README.md) | 历代迭代、失败、实测表及可恢复的原始证据 / Revision ledger, failures, measured results and archived evidence. |

## 专题文档 / Guides by task

| 需要完成的事 / Task | 入口 / Guide |
|---|---|
| 了解固件与电路 / Understand the design | [架构 / Architecture](docs/architecture.md) · [硬件 / Board](docs/hardware-overview.md) · [模式 / Measurement](docs/measurement-modes.md) |
| 实现接收器 / Implement a receiver | [接入步骤 / Integration](docs/software-integration.md) · [权威契约 / Host contract](docs/integration/firmware_host_contract.md) · [帧格式 / Wire format](docs/output-format.md) |
| 选择传输、查询状态 / Select a transport or query state | [通道 / Transports](docs/transport-protocol.md) · [BLE](docs/ble-protocol.md) · [命令参数 / Command arguments](docs/command-reference.md) |
| 复现实验或排错 / Reproduce a result | [历代测试方法 / Test catalogue](docs/validation.md) · [故障排查 / Troubleshooting](docs/troubleshooting.md) · [可靠性审计入口 / Audit index](docs/audit/firmware_reliability_fix_report.md) |
