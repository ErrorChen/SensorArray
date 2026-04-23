# boardSupport / 板级总线支持

## 1) Scope / 模块范围

**中文**

`boardSupport` 负责板级 I2C 总线初始化（主总线 + 可选第二总线），并提供与 FDC 驱动签名匹配的 I2C 回调适配函数。

**English**

`boardSupport` initializes board-level I2C buses (primary + optional secondary) and provides I2C callback adapters matching FDC driver signatures.

## 2) Public API / 对外接口

- `boardSupportInit`
- `boardSupportDeinit`
- `boardSupportIsI2c1Enabled`
- `boardSupportGetI2cCtx`
- `boardSupportGetI2c1Ctx`
- `boardSupportGetI2cBusInfo`
- `boardSupportI2cWriteRead`
- `boardSupportI2cWrite`

## 3) Boundary / 边界

**中文**

- 只做总线与回调适配。
- 不做板级 D-line 路由，不做 ADS/FDC 应用策略。

**English**

- Bus and callback adapter only.
- No D-line routing ownership and no ADS/FDC application policy.

## 4) Current Status / 当前状态

- Used by `main/sensorarrayApp.c` bring-up path.
- Exposes configured bus metadata for diagnostics such as secondary FDC single-point debug.
- Recovery now uses staged escalation:
  - `controller_only` for soft faults (no GPIO pulse / no reinstall)
  - `line_recovery` only when line-state indicates stuck-low
  - `driver_reinstall` only when required, with strict delete/init result handling
- If precheck is idle-high (`SCL=1,SDA=1`), startup soft-reinit does not force manual line recovery.
- Full driver reinstall increments per-port `busGeneration`; upper layers can invalidate stale device handles safely.
