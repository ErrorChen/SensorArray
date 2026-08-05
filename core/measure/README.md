# Measurement core / 测量核心

## 中文

`core/measure` 是矩阵测量策略层。它拥有唯一的测量模式状态、帧边界切换、
模拟路由安全顺序、FDC/ADS 引擎调度以及测量结果语义；芯片寄存器操作仍由
`components` 完成，GPIO 电平到板级路径的映射仍由 `core/board` 完成。

三个生产模式为：

- `CAP`：保留既有双 FDC2214 row-epoch、freshness、rescue、C/D/K 和异步输出路径。
- `VOLT`：ADS126x ADC1 按 S 行、D1..D8 顺序测量，输出整数微伏。
- `RES`：ADS126x ADC1 测量分压节点并输出整数毫欧。

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
- `ads/sensorarrayAdsAutoRange.*`：可脱离硬件测试的 PGA 决策和 per-mode/per-cell cache。
- `ads/sensorarrayAdsMath.*`：rail split、电压和分压电阻 fixed-point 算法及分类。
- `fdc/*`：原有电容生产路径；本次模式扩展不改写 ready/read/rescue 算法。
- `mixed/*`：仍不是生产模式；不要用它绕过统一状态机。

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
