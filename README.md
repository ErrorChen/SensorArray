# SensorArray Firmware / SensorArray 固件

面向 ESP32-S3 的 ESP-IDF 固件仓库，按可复用组件组织 TMUX 开关控制、ADS126x 模数转换、FDC2214 电容采样、矩阵访问和协议封装代码。当前默认 `app_main()` 仍以硬件 bring-up 和路由验证为主，尚未把矩阵扫描、协议打包和 BLE/有线传输串成完整产品路径。

ESP-IDF firmware repository for ESP32-S3. It is organized as reusable components for TMUX switching, ADS126x ADC reads, FDC2214 capacitance sensing, matrix access, and protocol framing. The current default `app_main()` is still a hardware bring-up and routing-validation path; it does not yet connect matrix scanning, framing, and BLE/wired transport into one finished data pipeline.

## Quick Navigation / 快速导航

| Path | Notes |
| --- | --- |
| [main/main.c](main/main.c) | 当前默认入口；硬件 bring-up 循环 / Current default entry point; hardware bring-up loop |
| [main/Kconfig.projbuild](main/Kconfig.projbuild) | 顶层项目配置 / Top-level project configuration |
| [components/ads126xAdc/README.md](components/ads126xAdc/README.md) | ADS1262/ADS1263 SPI 驱动 / ADS1262/ADS1263 SPI driver |
| [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md) | FDC2214 I2C 驱动 / FDC2214 I2C driver |
| [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md) | TMUX1108/TMUX1134 GPIO 控制 / TMUX1108/TMUX1134 GPIO control |
| [core/matrixEngine/README.md](core/matrixEngine/README.md) | 矩阵区域读写助手 / Matrix region I/O helper |
| [core/boardSupport/boardSupport.c](core/boardSupport/boardSupport.c) | I2C 总线初始化与适配 / I2C bus init and adapter callbacks |
| [transport/protocolWire/README.md](transport/protocolWire/README.md) | 固定 400 字节线协议帧 / Fixed 400-byte wire frame |
| [transport/protocolUsb/README.md](transport/protocolUsb/README.md) | USB 包装帧与解析器 / USB wrapper frame and parser |
| [example/example_ble_server_throughput.c](example/example_ble_server_throughput.c) | BLE 吞吐参考示例，不是默认应用 / BLE throughput reference example, not the default app |
| [datasheets/](datasheets/) | 仓库内参考资料 / In-repo reference materials |

## 中文说明

### 1. 项目概述

`SensorArray` 是一个 ESP-IDF 工程，默认目标为 `esp32s3`。仓库同时使用标准 `components/` 目录和根 `CMakeLists.txt` 里的 `EXTRA_COMPONENT_DIRS`（`core/`、`transport/`）来组织模块。

从代码现状看，这个仓库更像“驱动 + 集成骨架”：

- 已有可复用的底层驱动：ADS126x、FDC2214、TMUX。
- 已有可复用的中层模块：`matrixEngine`、`protocolWire`、`protocolUsb`。
- 当前默认应用仍然是 `main/main.c` 中的固定点位 bring-up 循环，而不是完整矩阵扫描器或传输固件。

### 2. 当前状态

| 状态 | 路径 | 说明 |
| --- | --- | --- |
| 已实现 | `components/ads126xAdc` | ADS1262/ADS1263 SPI 驱动；支持 ADC1、可选 ADC2、CRC/校验和、原始码转微伏 |
| 已实现 | `components/fdc2214Cap` | FDC2214 I2C 驱动；支持 reset、ID 读取、通道配置、单通道/自动扫描、28-bit 样本读取 |
| 已实现 | `components/tmuxSwitch` | TMUX1108/TMUX1134 GPIO 控制；包含安全行切换逻辑 |
| 已实现 | `core/boardSupport` | 初始化主 I2C 和可选第二 I2C，并提供 FDC 驱动使用的 I2C 回调 |
| 已实现 | `transport/protocolWire` | 固定 400 字节帧打包 |
| 已实现 | `transport/protocolUsb` | `[sync|len|crc|payload]` USB 包装帧与字节流解析器 |
| 部分集成 | `main/main.c` | 当前默认 `app_main()`；初始化板级总线和器件后，循环读取少量固定测点并打印日志 |
| 部分集成 | `core/matrixEngine` | 同步矩形区域读/写 API 已实现，但默认应用未接入；写路径目前只有有限的“驱动/路由”语义 |
| 占位 | `core/powerCtrl` | 仅有 Kconfig 和占位 `.c/.h` |
| 占位 | `transport/transportWire` | 仅有占位 `.c/.h` |
| 占位 | `transport/bleTransport` | 仅有占位 `.c/.h` |
| 参考代码 | `example/example_ble_server_throughput.c` | Espressif BLE 吞吐示例，保留在仓库中作为参考，不在默认构建路径中 |

### 3. 硬件/固件架构概览

```text
ESP32-S3
  |- GPIO -> TMUX1108: 行选择 + REF/GND 源切换
  |- GPIO -> TMUX1134: 列组/路径选择
  |- SPI  -> ADS1262 / ADS1263
  |- I2C0 -> FDC2214 #0
  `- I2C1 -> FDC2214 #1 (可选)

当前默认应用:
  路由选择 -> ADS/FDC 采样 -> 数值换算 -> printf("INIT,..."/"DBG,...")

仓库中已存在但默认应用未串起来的模块化路径:
  matrixEngine -> protocolWire -> protocolUsb -> transportWire / bleTransport
```

当前代码体现出的板级假设主要在 `main/main.c`：

- TMUX1108 用于 `S1..S8` 行选择，`SW` 用于在 `REF` 和 `GND` 之间切换输入源。
- TMUX1134 用于列组/路径选择。
- ADS 路径里，`D1..D8` 默认映射到 `AIN7..AIN0`。
- 电容路径里，默认把 `D1..D4` 交给主 FDC2214，把 `D5..D8` 交给第二颗 FDC2214。
- `main/main.c` 还硬编码了当前原理图下奇偶 `D` 线与 TMUX1134 `SEL` 使能关系，这属于当前板卡路由假设，不是通用抽象。

### 4. 仓库结构

| 路径 | 作用 |
| --- | --- |
| `components/` | ESP-IDF 标准组件目录；放底层硬件驱动 |
| `core/` | 通过 `EXTRA_COMPONENT_DIRS` 加入；放板级支持、矩阵访问、功耗抽象 |
| `transport/` | 通过 `EXTRA_COMPONENT_DIRS` 加入；放协议和传输层 |
| `main/` | 当前实际构建的应用入口 |
| `example/` | 参考代码；默认不参与当前应用构建 |
| `datasheets/` | 芯片资料、模块资料和电路图文件 |

如果你第一次阅读这个仓库，通常按下面顺序更高效：

1. [main/main.c](main/main.c)
2. [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md)
3. [components/ads126xAdc/README.md](components/ads126xAdc/README.md)
4. [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md)
5. [core/matrixEngine/README.md](core/matrixEngine/README.md)
6. [transport/protocolWire/README.md](transport/protocolWire/README.md)
7. [transport/protocolUsb/README.md](transport/protocolUsb/README.md)

### 5. 构建与烧录

前提：

- 已安装并导出 ESP-IDF 环境。
- 仓库中存在 `sdkconfig.defaults` 和 `sdkconfig.defaults.esp32s3`，但仓库没有显式锁定某个 ESP-IDF 版本。

典型流程：

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p <PORT> flash monitor
```

说明：

- `sdkconfig.defaults.esp32s3` 把默认目标设为 `esp32s3`。
- `main/` 才是当前默认应用入口；`example/` 下的 BLE 示例不会自动替代它。
- `main/CMakeLists.txt` 把 `matrixEngine`、`protocolWire`、`protocolUsb`、`transportWire`、`bleTransport` 都列为依赖，但当前 `main/main.c` 实际只直接使用了板级支持、TMUX、ADS 和 FDC 驱动。

### 6. 配置 / menuconfig

主要配置组及其当前实际意义如下：

| 菜单 | 当前作用 |
| --- | --- |
| `SensorArray Project` | ADS126x 型号选择、SPI DMA、SPI 最大传输长度会被当前代码使用；任务核绑定、任务栈、优先级、矩阵默认值更多是为 `matrixEngine`/后续任务预留 |
| `Board Support (Pins/Buses)` | `boardSupport` 当前使用主 I2C 和可选第二 I2C；SPI 主机和 ADS GPIO 由 `main/main.c` 中的 ADS 初始化直接使用 |
| `TMUX Switch` | 当前 TMUX 驱动直接使用这些 GPIO、极性和安全切换配置 |
| `ADS126x ADC` | 当前 ADS 驱动直接使用这些配置 |
| `FDC2214Cap` | 当前 FDC 驱动和 bring-up 路径会读取使能、地址和通道配置 |
| `Matrix Engine` | `ROWS`、`COLS`、`OVERSAMPLE` 会被 `matrixEngine.c` 使用；frame period、ringbuffer、task core/stack/prio 目前没有被实际任务代码消费 |
| `Wire Protocol` | `CONFIG_PROTOCOL_ENABLE_CRC16` 当前有效；`CONFIG_PROTOCOL_FRAME_VERSION` 目前没有出现在实际帧内容里 |
| `Wired Transport` / `BLE Transport` | 目前只有 Kconfig；占位模块没有真正消费这些配置项 |

需要特别注意的几个现状：

- `sdkconfig.defaults` 里仍保留了 BLE 吞吐示例相关选项，这不代表仓库已经集成完整 BLE 传输路径。
- `FDC2214CAP_CHANNELS` 的 Kconfig 范围当前是 `1..2`，但 `main/main.c` 的 bring-up 路径会把请求值提升到至少 `4`，因此默认应用当前实际上按 4 通道策略配置每个已探测到的 FDC 设备。

### 7. 数据流

#### 当前默认应用中真正跑通的数据流

1. `app_main()` 调用 `boardSupportInit()` 初始化主 I2C 和可选第二 I2C。
2. 初始化 `tmuxSwitch`，并把默认路由设到行 `0`、`SEL A/B = false`、TMUX1108 源为 `GND`。
3. 初始化 ADS126x SPI 设备，配置内部参考，准备参考路径。
4. 初始化主 FDC2214；如果第二 I2C 存在，则再初始化第二颗 FDC2214。
5. 在无限循环中读取少量固定测点：
   - `S1D1` 电阻，经 ADS 路径读取并换算为微欧姆级电阻值（输出单位为 mOhm）
   - `S2D4` 电容，经 FDC 读取 `Raw28`
   - `S2D4` 电压，经 ADS 读取微伏
   - `S2D8` 电容，经 FDC 读取 `Raw28`
   - `S2D8` 电压，经 ADS 读取微伏
6. 结果通过 `printf` 输出为 `INIT,...` 和 `DBG,...` 日志行。

#### 仓库中已有但默认应用尚未接通的模块化路径

从模块设计看，仓库已经具备下面这条“可拼起来”的链路：

```text
matrixEngineRegionIo()
  -> protocolWirePackFrame()
  -> protocolUsbBuildFrame()
  -> transportWire / bleTransport
```

但当前默认 `app_main()` 并没有调用这条链路，所以它目前仍是“模块可用、集成未完成”的状态。

### 8. 协议

#### `protocolWire`

- 固定小端帧，长度始终为 `400` 字节：
  - `seq` = 2 字节
  - `t0` = 4 字节
  - `validMask` = 8 字节
  - `offset[64]` = 128 字节
  - `data[64]` = 256 字节
  - `crc16` = 2 字节
- 即使关闭 `CONFIG_PROTOCOL_ENABLE_CRC16`，帧尾部这 2 字节仍然存在，只是被写成 `0`。
- 每个 `data[i]` 的高 4 bit 是 tag，低 28 bit 是 payload。
- 当前公开的 tag 只有：
  - `PROTOCOL_WIRE_DATA_TAG_NONE`
  - `PROTOCOL_WIRE_DATA_TAG_FDC2214`

#### `protocolUsb`

- 头部固定 6 字节：`sync(0xA55A) + len + crc16`，后跟 payload。
- 默认最大 payload 为 `512` 字节，足够承载当前 `protocolWire` 的 400 字节帧。
- 解析器在长度错误或 CRC 错误后会重新扫描 sync word 做重同步。

### 9. 组件概览

- [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md)
  - TMUX1108/TMUX1134 的 GPIO 驱动。
  - 支持安全行切换。
  - 当前实现假设 TMUX1108 `EN` 在板上常开，不由驱动控制。
- [components/ads126xAdc/README.md](components/ads126xAdc/README.md)
  - ADS1262/ADS1263 SPI 驱动。
  - 支持强制型号或按 ID 自动识别。
  - `CONFIG_ADS126X_HAS_ADC2` 打开且器件为 ADS1263 时，ADC2 API 可用。
- [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md)
  - 基于回调的 I2C 驱动。
  - 约束 `MUX_CONFIG` 固定位，检查 `FREF_DIVIDER != 0`。
  - 返回 `Raw28` 以及 `ErrWatchdog` / `ErrAmplitude` 标志。
- [core/matrixEngine/README.md](core/matrixEngine/README.md)
  - 提供矩形区域读/写 API，输出按 row-major 排列。
  - 当前 Kconfig 默认值是 8 行、8 列；默认列组大小为 4，并内置 `D1..D8 -> AIN7..AIN0` 与双 FDC 布局假设；这些映射可通过配置和回调覆盖。
  - 自身不负责初始化 ADS/FDC，也不负责创建扫描任务。

### 10. 入口点与示例

- 当前默认入口：[`main/main.c`](main/main.c)
  - 这是一个 bring-up / 路由验证程序。
  - 它不会调用 `matrixEngine`、`protocolWire`、`protocolUsb`、`transportWire` 或 `bleTransport`。
- 参考示例：[`example/example_ble_server_throughput.c`](example/example_ble_server_throughput.c)
  - 这是一个独立 BLE 吞吐示例，文件里自带 `app_main()`。
  - 默认构建不会把它编进当前应用，否则会与 `main/main.c` 的 `app_main()` 冲突。

### 11. 仓库内资料

`datasheets/` 目录当前包含：

- [datasheets/ads1263.pdf](datasheets/ads1263.pdf)
- [datasheets/fdc2212.pdf](datasheets/fdc2212.pdf)
- [datasheets/ESP32-S3-WROOM-1.pdf](datasheets/ESP32-S3-WROOM-1.pdf)
- [datasheets/esp32-s3_technical_reference_manual_en.pdf](datasheets/esp32-s3_technical_reference_manual_en.pdf)
- [datasheets/esp32-s3_technical_reference_manual_cn.pdf](datasheets/esp32-s3_technical_reference_manual_cn.pdf)
- [datasheets/esp32-s3_datasheet_cn.pdf](datasheets/esp32-s3_datasheet_cn.pdf)
- [datasheets/circuit.pdf](datasheets/circuit.pdf)
- [datasheets/circuit_page3-3.png](datasheets/circuit_page3-3.png)
- [datasheets/circuit_page4-4.png](datasheets/circuit_page4-4.png)

当前仓库里没有单独的 TMUX 数据手册 PDF；FDC 相关资料文件名也是 `fdc2212.pdf`，不是单独的 `fdc2214.pdf`。

### 12. 已知限制 / 当前范围

- 当前默认应用不是完整矩阵扫描固件，只是固定测点 bring-up 循环。
- `matrixEngine` 虽然可用，但没有被默认应用接入，也没有实际扫描任务、通信任务或 ringbuffer 流水线。
- `matrixEngine` 的 `WRITE` 模式当前没有“每点写入 payload”接口，只是在遍历区域时调用 drive/row/col-group 相关回调。
- `transportWire` 和 `bleTransport` 仍然是占位模块；仅有 Kconfig 不代表传输链路已经完成。
- `core/powerCtrl` 仍是占位。
- `core/boardSupport` 目前只覆盖 I2C 总线初始化与 FDC 回调适配，名称比当前实现范围更宽。
- 若更换板卡或改动原理图，`main/main.c` 中的当前路由假设大概率需要同步修改。

### 13. 许可证

仓库根目录包含 [LICENSE](LICENSE)，当前为 GNU GPL v2 文本。

另外，个别引入的参考示例文件可能保留自己的 SPDX 头注释；分发或复用这些文件时，应同时检查对应源文件头部声明。

## English

### 1. Project Summary

`SensorArray` is an ESP-IDF project with `esp32s3` as the default target. The repository uses the standard `components/` directory plus `EXTRA_COMPONENT_DIRS` in the root `CMakeLists.txt` (`core/` and `transport/`) to organize code.

In its current state, this repository is best understood as a "drivers plus integration skeleton" project:

- Reusable low-level drivers already exist for ADS126x, FDC2214, and TMUX parts.
- Reusable mid-layer modules already exist for `matrixEngine`, `protocolWire`, and `protocolUsb`.
- The active application is still the fixed-point bring-up loop in `main/main.c`, not a complete matrix scanner or transport firmware.

### 2. Current Status

| Status | Path | Notes |
| --- | --- | --- |
| Implemented | `components/ads126xAdc` | ADS1262/ADS1263 SPI driver with ADC1, optional ADC2, CRC/checksum handling, and raw-to-microvolt conversion |
| Implemented | `components/fdc2214Cap` | FDC2214 I2C driver with reset, ID readout, channel config, single-channel/autoscan modes, and 28-bit sample reads |
| Implemented | `components/tmuxSwitch` | TMUX1108/TMUX1134 GPIO control, including safe row switching |
| Implemented | `core/boardSupport` | Initializes the primary I2C bus and optional second I2C bus, and provides adapter callbacks for the FDC driver |
| Implemented | `transport/protocolWire` | Fixed 400-byte frame packer |
| Implemented | `transport/protocolUsb` | USB `[sync|len|crc|payload]` wrapper and byte-stream parser |
| Partially integrated | `main/main.c` | Current default `app_main()`; initializes board buses and devices, then loops over a small set of fixed measurement points |
| Partially integrated | `core/matrixEngine` | Synchronous rectangular region I/O API is implemented, but the default application does not use it; the write path has limited semantics |
| Placeholder | `core/powerCtrl` | Only Kconfig plus placeholder `.c/.h` files |
| Placeholder | `transport/transportWire` | Placeholder `.c/.h` only |
| Placeholder | `transport/bleTransport` | Placeholder `.c/.h` only |
| Reference code | `example/example_ble_server_throughput.c` | Espressif BLE throughput example kept for reference; not part of the default app build |

### 3. Hardware / Firmware Architecture

```text
ESP32-S3
  |- GPIO -> TMUX1108: row select + REF/GND source select
  |- GPIO -> TMUX1134: column-group / path select
  |- SPI  -> ADS1262 / ADS1263
  |- I2C0 -> FDC2214 #0
  `- I2C1 -> FDC2214 #1 (optional)

Current default app:
  route selection -> ADS/FDC sampling -> value conversion -> printf("INIT,..."/"DBG,...")

Modules that exist in the repo but are not wired into the default app:
  matrixEngine -> protocolWire -> protocolUsb -> transportWire / bleTransport
```

The board-level assumptions that are currently encoded in `main/main.c` include:

- TMUX1108 is used for `S1..S8` row selection, and `SW` switches the input source between `REF` and `GND`.
- TMUX1134 is used for column-group/path selection.
- On the ADS path, `D1..D8` are mapped to `AIN7..AIN0`.
- On the capacitive path, `D1..D4` are assigned to the primary FDC2214 and `D5..D8` to the secondary FDC2214.
- `main/main.c` also hardcodes the current schematic's odd/even `D`-line relationship to TMUX1134 `SEL` enable state. That is a current-board routing assumption, not a generic abstraction.

### 4. Repository Layout

| Path | Role |
| --- | --- |
| `components/` | Standard ESP-IDF component directory for low-level hardware drivers |
| `core/` | Added via `EXTRA_COMPONENT_DIRS`; board support, matrix access, power abstraction |
| `transport/` | Added via `EXTRA_COMPONENT_DIRS`; protocol helpers and transport layer |
| `main/` | The currently built application entry point |
| `example/` | Reference code; not built into the default app |
| `datasheets/` | Datasheets, module references, and circuit files |

For a first read-through, this order is usually the most productive:

1. [main/main.c](main/main.c)
2. [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md)
3. [components/ads126xAdc/README.md](components/ads126xAdc/README.md)
4. [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md)
5. [core/matrixEngine/README.md](core/matrixEngine/README.md)
6. [transport/protocolWire/README.md](transport/protocolWire/README.md)
7. [transport/protocolUsb/README.md](transport/protocolUsb/README.md)

### 5. Build and Flash

Prerequisites:

- ESP-IDF is installed and its environment is exported.
- The repository includes `sdkconfig.defaults` and `sdkconfig.defaults.esp32s3`, but it does not pin one explicit ESP-IDF version in-repo.

Typical workflow:

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p <PORT> flash monitor
```

Notes:

- `sdkconfig.defaults.esp32s3` sets `esp32s3` as the default target.
- `main/` is the current default application entry point; the BLE example in `example/` does not replace it automatically.
- `main/CMakeLists.txt` lists `matrixEngine`, `protocolWire`, `protocolUsb`, `transportWire`, and `bleTransport` as dependencies, but the current `main/main.c` directly uses only board support plus the TMUX, ADS, and FDC drivers.

### 6. Configuration / menuconfig

The main Kconfig groups and their current practical effect are:

| Menu | Current effect |
| --- | --- |
| `SensorArray Project` | ADS126x variant selection, SPI DMA, and SPI max transfer size are used by current code; task pinning, stack sizes, priorities, and matrix defaults are mostly reserved for `matrixEngine` or future tasks |
| `Board Support (Pins/Buses)` | `boardSupport` currently uses the primary and optional secondary I2C bus settings; SPI host and ADS GPIO settings are consumed directly by ADS init in `main/main.c` |
| `TMUX Switch` | Directly used by the current TMUX driver |
| `ADS126x ADC` | Directly used by the current ADS driver |
| `FDC2214Cap` | Used by the current FDC driver and bring-up path |
| `Matrix Engine` | `ROWS`, `COLS`, and `OVERSAMPLE` are used in `matrixEngine.c`; frame period, ringbuffer, and task core/stack/prio settings are not consumed by real task code yet |
| `Wire Protocol` | `CONFIG_PROTOCOL_ENABLE_CRC16` is used now; `CONFIG_PROTOCOL_FRAME_VERSION` is not serialized into the current frame implementation |
| `Wired Transport` / `BLE Transport` | Kconfig exists, but the placeholder transport modules do not actually consume those settings yet |

Two important caveats:

- `sdkconfig.defaults` still carries BLE throughput-example-related options. That does not mean a complete BLE transport path is integrated into the default app.
- `FDC2214CAP_CHANNELS` currently has a Kconfig range of `1..2`, but the bring-up logic in `main/main.c` promotes the requested value to at least `4`, so the default app effectively configures each detected FDC device for a 4-channel policy.

### 7. Data Flow

#### The data path that actually runs in the default app today

1. `app_main()` calls `boardSupportInit()` to initialize the primary I2C bus and optional second I2C bus.
2. It initializes `tmuxSwitch` and sets the default route to row `0`, `SEL A/B = false`, and TMUX1108 source = `GND`.
3. It initializes the ADS126x SPI device, enables the internal reference, and prepares the reference path.
4. It initializes the primary FDC2214 and, if the second I2C bus exists, a secondary FDC2214.
5. In an infinite loop it reads a small set of fixed points:
   - `S1D1` resistance via the ADS path, converted to mOhm
   - `S2D4` capacitance via FDC `Raw28`
   - `S2D4` voltage via ADS in microvolts
   - `S2D8` capacitance via FDC `Raw28`
   - `S2D8` voltage via ADS in microvolts
6. Results are printed as `INIT,...` and `DBG,...` log lines.

#### The modular path that exists in the repo but is not connected yet

The repository already contains the pieces for this chain:

```text
matrixEngineRegionIo()
  -> protocolWirePackFrame()
  -> protocolUsbBuildFrame()
  -> transportWire / bleTransport
```

The default `app_main()` does not call that chain yet, so this remains "modules implemented, integration incomplete."

### 8. Protocols

#### `protocolWire`

- Fixed little-endian frame with a total size of `400` bytes:
  - `seq` = 2 bytes
  - `t0` = 4 bytes
  - `validMask` = 8 bytes
  - `offset[64]` = 128 bytes
  - `data[64]` = 256 bytes
  - `crc16` = 2 bytes
- The trailing 2-byte CRC field always exists. If `CONFIG_PROTOCOL_ENABLE_CRC16` is disabled, it is written as `0`.
- The upper 4 bits of each `data[i]` word are a tag; the lower 28 bits are payload.
- The current public tags are:
  - `PROTOCOL_WIRE_DATA_TAG_NONE`
  - `PROTOCOL_WIRE_DATA_TAG_FDC2214`

#### `protocolUsb`

- Fixed 6-byte header: `sync(0xA55A) + len + crc16`, followed by payload.
- Default maximum payload size is `512` bytes, which is enough for the current 400-byte `protocolWire` frame.
- On length or CRC errors, the parser rescans for the sync word to recover framing.

### 9. Components Overview

- [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md)
  - GPIO driver for TMUX1108 and TMUX1134
  - Supports safe row switching
  - Current implementation assumes TMUX1108 `EN` is tied on-board and not actively controlled
- [components/ads126xAdc/README.md](components/ads126xAdc/README.md)
  - SPI driver for ADS1262 and ADS1263
  - Supports either forced device type or ID-based detection
  - ADC2 APIs are available only when `CONFIG_ADS126X_HAS_ADC2` is enabled and the device is ADS1263
- [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md)
  - Callback-based I2C driver
  - Enforces the fixed `MUX_CONFIG` bits and checks that `FREF_DIVIDER != 0`
  - Returns `Raw28` plus `ErrWatchdog` / `ErrAmplitude` flags
- [core/matrixEngine/README.md](core/matrixEngine/README.md)
  - Rectangular region I/O helper with row-major output ordering
  - Current Kconfig defaults are 8 rows and 8 columns; the default column-group size is 4, and the built-in mappings assume `D1..D8 -> AIN7..AIN0` plus a dual-FDC `C1..C4 / C5..C8` layout; those mappings can be overridden by config and callbacks
  - Does not initialize ADS/FDC devices or create scan tasks by itself

### 10. Entry Points and Examples

- Default entry point: [main/main.c](main/main.c)
  - This is a bring-up / routing-validation program.
  - It does not currently call `matrixEngine`, `protocolWire`, `protocolUsb`, `transportWire`, or `bleTransport`.
- Reference example: [example/example_ble_server_throughput.c](example/example_ble_server_throughput.c)
  - This is a standalone BLE throughput example with its own `app_main()`.
  - It is not part of the default build; otherwise it would conflict with `main/main.c`.

### 11. Datasheets and In-Repo References

The `datasheets/` directory currently contains:

- [datasheets/ads1263.pdf](datasheets/ads1263.pdf)
- [datasheets/fdc2212.pdf](datasheets/fdc2212.pdf)
- [datasheets/ESP32-S3-WROOM-1.pdf](datasheets/ESP32-S3-WROOM-1.pdf)
- [datasheets/esp32-s3_technical_reference_manual_en.pdf](datasheets/esp32-s3_technical_reference_manual_en.pdf)
- [datasheets/esp32-s3_technical_reference_manual_cn.pdf](datasheets/esp32-s3_technical_reference_manual_cn.pdf)
- [datasheets/esp32-s3_datasheet_cn.pdf](datasheets/esp32-s3_datasheet_cn.pdf)
- [datasheets/circuit.pdf](datasheets/circuit.pdf)
- [datasheets/circuit_page3-3.png](datasheets/circuit_page3-3.png)
- [datasheets/circuit_page4-4.png](datasheets/circuit_page4-4.png)

There is currently no separate TMUX datasheet PDF in the repository, and the FDC family reference file is named `fdc2212.pdf` rather than `fdc2214.pdf`.

### 12. Known Limitations / Current Scope

- The default app is not a complete matrix-scanning firmware; it is a fixed-point bring-up loop.
- `matrixEngine` exists, but it is not integrated into the default app and there is no real scan-task, comm-task, or ringbuffer pipeline yet.
- `matrixEngine`'s `WRITE` mode does not currently expose a per-point payload API; it only iterates region routing/drive callbacks.
- `transportWire` and `bleTransport` are still placeholders. Having Kconfig entries does not mean the transport path is finished.
- `core/powerCtrl` is still a placeholder.
- `core/boardSupport` currently covers I2C bus init and FDC callback adapters only; its name is broader than its present implementation.
- If the board or schematic changes, the routing assumptions hardcoded in `main/main.c` will likely need changes as well.

### 13. License

The repository root contains [LICENSE](LICENSE), currently GNU GPL v2 text.

Some imported reference/example files may also retain their own SPDX file headers. If you redistribute or reuse those files directly, check the header in the corresponding source file as well.
