# Measurement core / 测量核心

## 中文

`core/measure` 是矩阵测量策略层。它拥有唯一的测量模式状态、帧边界切换、
模拟路由安全顺序、FDC/ADS 引擎调度以及测量结果语义；芯片寄存器操作仍由
`components` 完成，GPIO 电平到板级路径的映射仍由 `core/board` 完成。

三个生产模式为：

- `CAP`：保留既有双 FDC2214 row-epoch、freshness、rescue、C/D/K 和异步输出路径。
- `VOLT`：ADS126x ADC1 按 S 行、D1..D8 顺序测量，输出整数微伏。
- `RES`：ADS126x ADC1 测量分压节点并输出整数毫欧。

VOLT/RES 永远按 `for row=1..activeRows`、`for dLine=1..8` 完整扫描；不存在四点
模式、半列分组或把旧 cell 标成 fresh 的生产路径。ADS ownership 显式区分
`MATRIX/BATTERY/ADSCHK/RAIL/ZERO/NONE`，因此 AIN8 电池事务和主动检查只能在
完整矩阵帧后执行，不能插入 64-cell 帧中间。CAP 的两个 FDC worker 不变；进入
VOLT/RES 时只在模式边界 sleep 并 readback 两颗 FDC，回 CAP 时恢复。

Core 1 的 capture limiter 默认关闭（VOLT/RES Kconfig target `0`）；Core 0 的
`OUTCAP` 只能限制 sink 输出。矩阵结果额外报告 `ADS50` cache 计数与 `ADST50`
分阶段耗时，`SF50` 分开报告 physical capture、emitted frame 和各 transport FPS。

`sensorarrayMeasurementModeContext_t` 是唯一权威模式状态。Serial、BLE 和
Wi-Fi 均进入同一个 parser 和 `CommandMailbox`；Core 0 只接受请求，Core 1
只在完整帧之后执行 `SAFE -> TRANSITION -> target`。切换会先停转换、撤销矩阵
激励、切换 TMUX1134/TMUX1108 和 SELA/SELB、配置 ADS/FDC、核对 GPIO/ADS
readback、等待建立并丢弃新转换，最后增加 generation 并发布 `MAPP`。任一步骤
失败都会停止转换、撤销激励并进入 `DEGRADED`/`SAFE`，不会发布伪有效数据。

职责拆分：

- `sensorarrayMeasurementMode.*`：纯逻辑状态机、accepted/applied/generation snapshot。
- `sensorarrayRouteController.*`：有所有权的安全路由切换和只读状态 snapshot。
- `sensorarrayRoutePolicy.h`：生产 controller 与 host fault injection 共用的纯 GPIO command/readback 判定。
- `ads/sensorarrayAdsMatrix.*`：动态 1..8 行 ADS 扫描与 cell telemetry。
- `ads/sensorarrayAdsAutoRange.*`：可脱离硬件测试的 PGA 决策。
- `ads/sensorarrayAdsCache.*`：寄存器 shadow、mode/cell profile、value/noise 和 rail fingerprint cache。
- `ads/sensorarrayBatteryScheduler.*`：基于微秒时钟的纯逻辑 due/defer/boundary admission。
- `ads/sensorarrayAdsMath.*`：rail split、电压和分压电阻 fixed-point 算法及分类。
- `fdc/*`：原有电容生产路径；本次模式扩展不改写 ready/read/rescue 算法。
- `mixed/*`：Mixed Row 生产路径；它通过统一 frame-boundary 状态机逐行调度
  CAP/VOLT/RES，并把每个子段的 acquired/fresh/valid/error 语义合并到同一帧。

模式、reference、rail、校准、路由、ADS reset 或连续 overrange 会使旧 ADS gain
cache 和旧 measurement payload 失效。热路径使用固定 context/slot 和有边界的
`int64_t` 运算，不对每个 sink 重复格式化，也不在 Core 1 发送 Serial/BLE/UDP。

## Australian English

`core/measure` is the matrix measurement policy layer. It owns the single
measurement-mode state, frame-boundary transitions, safe analogue-route order,
FDC/ADS engine dispatch, and result semantics. Chip register operations remain
in `components`; GPIO-level board meaning remains in `core/board`.

The production modes are `CAP` (the existing dual-FDC row-epoch path), `VOLT`
(ADS126x ADC1, row-major integer microvolts), and `RES` (ADS126x ADC1 divider
measurement, integer milliohms). All transports share one parser and mailbox.
Core 0 accepts a request; Core 1 applies it only after a complete frame using a
safe transition. A failed readback or setup step removes excitation, stops
conversion, enters `DEGRADED`/`SAFE`, and suppresses apparently valid data.

See [measurement modes](../../docs/measurement-modes.md),
[wire protocol](../../docs/measurement-protocol.md), and
[ADS implementation](ads/README.md).

## Battery boundary policy verified on hardware

`sensorarrayBatteryScheduler` advances absolute microsecond deadlines from the
previous deadline, so transaction duration does not accumulate into a 1 Hz
drift. CAP first evaluates a real FDC gap. If the complete job plus guard does
not fit, `gapDeferred` authorises the immediately following complete-frame
boundary; `maximumDeferMs` remains the fallback when no gap was evaluable.
VOLT/RES permit battery, ADSCHK, rail, and zero ownership only after the full
`activeRows * 8` matrix frame. The 2026-08-06 strict COM12 run observed no
mixed frames, CRC failures, freshness failures, or restore failures.
