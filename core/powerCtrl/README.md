# powerCtrl / 电源控制层（占位）

## 1) Scope / 模块范围

**中文**

`powerCtrl` 目前是预留模块，尚未定义稳定公共 API。

**English**

`powerCtrl` is currently a placeholder module with no stable public API yet.

## 2) Intended Direction / 预期方向

- 电源域开关与上电时序抽象。
- 低功耗策略入口（睡眠/唤醒前后钩子）。

- Power-domain switching and power-up sequencing abstraction.
- Low-power policy hooks (sleep/wake transitions).

## 3) Boundary / 边界

**中文**

- 后续实现应保持通用，不与某个单板 route map 强耦合。

**English**

- Future implementation should remain reusable and not tightly couple to a single board route map.

## 4) Current Status / 当前状态

- Placeholder only (`.c/.h` stubs).
