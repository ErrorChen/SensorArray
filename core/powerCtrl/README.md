# 电源引脚配置占位 / Reserved power-control pins

组件为后续板卡的电源使能、充电状态与 power-good 预留配置。当前头文件与实现没有可调用函数。应用中的前端休眠、FDC SD 与参考控制由测量路由和板级组件实现。

The component reserves configuration for future power enables, charger status and power-good pins. Its header and implementation expose no callable functions. Measurement routing and board code own frontend sleep, FDC SD and reference control in the application.

GPIO 选项默认 -1，表示该信号没有可由此组件驱动或采样的引脚。

All four GPIO settings default to -1, which leaves the corresponding signal without a pin to drive or sample through this component.

## 配置 / Configuration

| Kconfig | 引脚用途 / Signal |
| --- | --- |
| `POWER_MAIN_EN_GPIO` | 主电源使能 / Main supply enable |
| `POWER_ANALOG_EN_GPIO` | 模拟供电使能 / Analogue supply enable |
| `POWER_CHG_STAT_GPIO` | 充电状态 / Charger status |
| `POWER_PG_GPIO` | 电源正常输入 / Power-good input |

工程通过此组件的 [CMakeLists.txt](CMakeLists.txt) 引入 [Kconfig](Kconfig)。 [头文件 / Header](include/powerCtrl.h) · [实现 / Implementation](powerCtrl.c)

The component build registers the settings listed above. There is no callable entry in the header or implementation.
