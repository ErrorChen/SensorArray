# powerCtrl / 电源控制占位层

## 中文说明

`core/powerCtrl` 当前是占位模块。`powerCtrl.h` 还没有稳定 public API，`powerCtrl.c` 只保留实现占位并明确不定义 `app_main()`。

Kconfig 中存在可选 GPIO：

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_POWER_MAIN_EN_GPIO` | `-1` | Main power enable GPIO, `-1` means always on or not controlled. |
| `CONFIG_POWER_ANALOG_EN_GPIO` | `-1` | Analogue rails enable GPIO. |
| `CONFIG_POWER_CHG_STAT_GPIO` | `-1` | Charger status GPIO. |
| `CONFIG_POWER_PG_GPIO` | `-1` | Power-good GPIO. |

当前 `main` 生命周期没有通过该模块控制 LM27762、TPS631000、BQ24074 或其它电源 IC。

## Australian English Documentation

`core/powerCtrl` is currently a placeholder. `powerCtrl.h` has no stable public API yet, and `powerCtrl.c` only keeps an implementation placeholder and explicitly does not define `app_main()`.

The Kconfig GPIOs are reserved for later board power control. The current `main` lifecycle does not drive LM27762, TPS631000, BQ24074, or other power ICs through this module.
