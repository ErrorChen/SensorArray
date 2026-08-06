# SensorArray architecture / SensorArray 架构

## 中文说明

当前源码采用两个异步域和分层硬件所有权：

```text
Core 0: shared command parser -> CommandMailbox
        TextFrameBus/EventRing -> Serial/BLE/Wi-Fi sinks and backpressure

Core 1: complete-frame boundary -> mode/route owner -> FDC or ADS acquisition
        -> frame assembly -> one C/V/R fixed-slot formatting operation
```

| 层 | 路径 | 责任 |
|---|---|---|
| 应用编排 | `main/main.c` | task 生命周期、mailbox 消费、帧边界 mode apply、engine dispatch、异步发布；不写 route GPIO 或芯片寄存器。 |
| 控制/输出 | `main/control`, `main/output`, `main/net` | 共享命令、fixed TextFrameBus、EventRing 和 Core 0 sinks；callback 不切硬件。 |
| 板级真源 | `core/board` | S/D、D->AIN、SELA/SELB、SW physical/logical、matrix REF、INTREF/VBIAS/REFMUX mode profiles。 |
| 测量策略 | `core/measure` | 唯一 mode 状态机、安全 transition、route ownership、FDC/ADS frame semantics。 |
| ADS 算法 | `core/measure/ads` | 动态行 ADC1 scan、rail/math/calibration、autorange、fresh/status/error。 |
| FDC 生产路径 | `core/measure/fdc` | 原有双 worker row epoch、cache、freshness、sweep/rescue；不被 ADS 模式重写。 |
| 资源 | `core/boardSupport` | SPI/I2C/GPIO lifecycle、ISR service 和 guarded recovery。 |
| 芯片驱动 | `components/*` | ADS126x SPI、FDC2214 I2C、TMUX GPIO primitives；不知道 mode、cell 或 frame。 |
| 通用同步引擎 | `core/matrixEngine` | 非生产 reusable executor；不接入双核异步 mode runtime。 |

`sensorarrayMeasurementModeContext_t` 是唯一权威模式状态。Core 0 对 MODE 请求
立即发布 accepted 并写 mailbox；只有 Core 1 能在完整帧后进入 TRANSITION、先撤销
矩阵激励、停止 conversion、切 TMUX/SELA/SELB、配置并 readback frontend、settle/
discard、失效旧 payload/PGA cache、增加 generation 并发布 applied。transition
期间不构造普通帧，所以一帧不会混入两个 mode。错误统一回到无激励 SAFE/
DEGRADED。

CAP 的双 FDC worker、row epoch、freshness、cache/rescue 和 C/D/K 字节保持原路径。
VOLT/RES 使用 ADS1262 与 ADS1263 都具备的 ADC1；ADC2 能力由运行时 ID 决定。
Core 1 只格式化一次，Core 0 sink 复用同一 slot；队列满时丢旧保新，不反压采集。

ADS 访问由 `NONE/MATRIX/BATTERY/ADSCHK/RAIL/ZERO` owner 串行化。register shadow
属于 ADS context；per-mode/per-cell profile 和 value/noise cache 属于 matrix engine；
rail fingerprint 连接 rail 状态与 cache generation；time-based battery scheduler 只做
due/admission 决策，仍调用现有 `sensorarrayAdsGap` transaction。外部 transaction
保存完整 register/running snapshot，恢复并 readback 后更新 shadow；恢复失败则
invalidate 所有硬件假设并阻止普通 valid frame。

VOLT/RES 的物理 acquisition 默认 unlimited，输出 cap 位于 Core 0。Core 1 将每帧
64-cell 数据格式化一次；cache/timing telemetry 使用独立固定 text slot，避免扩张
V/R 1536-byte wire 上界。CAP gap battery 只有在预算大于估计 duration+guard 时
admit；VOLT/RES battery 与 ADSCHK 只在完整帧之后运行。

## Australian English Documentation

The firmware has two asynchronous domains. Core 0 owns the shared command
parser, non-blocking mailbox, TextFrameBus/EventRing, and Serial/BLE/Wi-Fi
sinks. Core 1 owns frame-boundary configuration, analogue routes, FDC/ADS
conversion, frame assembly, and one fixed-slot formatting operation.

Board meaning belongs in `core/board`; measurement policy and the single mode
state belong in `core/measure`; chip drivers remain unaware of matrix modes and
wire frames. CAP preserves the established dual-FDC worker epoch. VOLT/RES use
the independent ADS ADC1 engine, fixed-point math, fresh-generation checks and
autorange. A failed transition removes excitation and enters SAFE/DEGRADED;
ordinary frames are suppressed during transition and never mix modes.

See [measurement modes](measurement-modes.md),
[measurement protocol](measurement-protocol.md), and
[software integration](software-integration.md).

## Auxiliary ADS ownership and deadline semantics

The battery scheduler owns no SPI driver and creates no task. It only decides
due/gap/boundary policy from `esp_timer_get_time()`; the existing Core 1
`sensorarrayAdsGap` transaction acquires the shared ADS owner. Deadlines advance
from the previous absolute due time, not job completion. CAP first attempts
admission into an FDC conversion gap. If the complete measured duration plus
guard cannot fit, the same complete frame boundary runs the job; VOLT/RES only
offer complete matrix boundaries. Register shadow generation is saved and the
restored POWER/MODE2/INPMUX/REFMUX state is read back before ownership returns.
Restore failure invalidates matrix assumptions and enters SAFE/DEGRADED rather
than publishing a normal valid frame.
