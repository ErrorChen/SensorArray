# TMUX 行与输入开关 / TMUX row and input switches

TMUX1108 用 A0–A2 从八行中选择一行。TMUX1134 的 SELA 管 D1–D4，SELB 管 D5–D8。在项目板上，SEL 高电平选 FDC，低电平选 ADS。逻辑 REF 对应 SW 低电平，释放 Q1；SW 高电平令 Q1 把 U5 公共端钳到 GND。U5 EN 由 R25 上拉。REFOUT 电压以 AVSS 为参考。安全切换、激励和建立时间由测量路由控制器协调。 / TMUX1108 address lines A0–A2 select one of eight rows. TMUX1134 SELA routes D1–D4, SELB routes D5–D8. On the project board a high SEL selects FDC and a low SEL selects ADS. Logical REF drives SW low and releases Q1. Driving SW high clamps the U5 common node to GND. R25 pulls U5 EN high. REFOUT is referenced to AVSS. The measurement route controller coordinates switching, excitation and settling. [接口 / Header](include/tmuxSwitch.h) · [引脚与极性 / Configuration](Kconfig) · [板级真值表 / Board profiles](../../core/board/sensorarrayBoardMap.c) · [调用者 / Route controller](../../core/measure/sensorarrayRouteController.c)

上电先调用 `tmuxSwitchInit()`。由路由 owner 使用零基行号 0..7 对应 S1..S8。读命令缓存与 MCU 引脚观察值可用下例，`obs* = -1` 表示未接引脚。判断模拟导通需要物理测量。对 `esp_err_t` 检查 `ESP_OK`，未初始化、非法行或缺少必需参数分别见各调用。 / Initialise with `tmuxSwitchInit()`. The route owner uses zero-based rows 0..7 for S1..S8. Read commanded state and MCU-side pin observations as below. A pin observation of −1 denotes an unconfigured pin. Analogue conduction requires physical measurement. Check ESP_OK and handle invalid state or arguments at the call site.

```c
tmuxSwitchControlState_t pins;
esp_err_t err = tmuxSwitchGetControlState(&pins);
if (err == ESP_OK && pins.tmux1134EnControllable) {
    /* pins.obsEnLevel is the MCU pin observation. */
}
```

## [tmuxSwitch.c](tmuxSwitch.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t tmuxSwitchInit(void)` | 配置 GPIO 并设置初始源、行、SEL 和 EN。 / Configure GPIO and establish the initial source, row, SEL and EN settings. | [matrixEngineInit](../../core/matrixEngine/matrixEngine.c#L230) |
| `esp_err_t tmux1108SelectRow(uint8_t row)` | 选择 0..7 行，若启用安全源切换则临时切源并恢复。 / Select row 0..7, temporarily changing and restoring source if configured. | [sensorarrayRouteControllerSelectRow](../../core/measure/sensorarrayRouteController.c#L759) |
| `esp_err_t tmux1108SelectRowBreakBeforeMake(uint8_t row)` | 已选择 REF 的 RES 会话中只改 A0–A2 并执行切换延迟。 / With REF already selected in a RES session, update only A0–A2 and apply the switch delay. | [sensorarrayRouteControllerSelectRow](../../core/measure/sensorarrayRouteController.c#L759) |
| `esp_err_t tmux1108GetRow(uint8_t *rowOut)` | 读取最后命令的零基行号到 rowOut。 / Copy the last commanded zero-based row into rowOut. | 公开入口 / Callable API |
| `esp_err_t tmux1108SetSource(tmux1108Source_t source)` | 切换到 GND 或 REF 并记录命令状态。 / Command the GND or REF source and remember the selection. | [sensorarrayRouteApplyControlProfile](../../core/measure/sensorarrayRouteController.c#L325) |
| `esp_err_t tmux1108GetSource(tmux1108Source_t *sourceOut)` | 读出最后命令的源选择。 / Copy the last commanded source selection to sourceOut. | [sensorarrayRouteControllerSelectRow](../../core/measure/sensorarrayRouteController.c#L759) |
| `esp_err_t tmux1134SelectSelALevel(bool level)` | 写 SELA 原始逻辑电平，必要时使能可控 EN。 / Drive the raw SELA level, enabling a controllable EN if needed. | [tmux1134SetSelAEnabled](tmuxSwitch.c#L445) |
| `esp_err_t tmux1134SelectSelBLevel(bool level)` | 写 SELB 原始逻辑电平，必要时使能可控 EN。 / Drive the raw SELB level, enabling a controllable EN if needed. | [tmux1134SetSelBEnabled](tmuxSwitch.c#L450) |
| `esp_err_t tmux1134SetEnLogicalState(bool on)` | 按 EN 脚是否可控处理逻辑开关请求。 / Apply the requested logical enable according to EN pin availability. | [sensorarrayMeasurePrepareFdcMatrixPath](../../core/measure/sensorarrayMeasure.c#L517) |
| `esp_err_t tmux1134GetEnLogicalState(bool *onOut)` | 返回已跟踪的逻辑 EN 状态。 / Copy tracked logical enable state into onOut. | 公开入口 / Callable API |
| `esp_err_t tmux1134SetSelAEnabled(bool enabled)` | 按 SELA 配置极性实现逻辑 enabled。 / Drive SELA according to its configured enable polarity. | [tmuxSwitchSetSelAEnabled](tmuxSwitch.c#L562) |
| `esp_err_t tmux1134SetSelBEnabled(bool enabled)` | 按 SELB 配置极性实现逻辑 enabled。 / Drive SELB according to its configured enable polarity. | [tmuxSwitchSetSelBEnabled](tmuxSwitch.c#L567) |
| `esp_err_t tmux1134SetAllOff(void)` | 有可控 EN 则关 EN，否则驱动 SEL 到配置的关闭电平。 / Disable via EN when available, otherwise command the configured inactive SEL levels. | 公开入口 / Callable API |
| `esp_err_t tmux1134SetAllOn(void)` | 驱动已配置选择脚为有效电平并启用可控 EN。 / Set configured selects active and enable a controllable EN. | 公开入口 / Callable API |
| `esp_err_t tmuxSwitchGetControlState(tmuxSwitchControlState_t *outState)` | 在临界区内复制命令值、MCU 引脚值及 EN 有效标志。 / Snapshot commanded levels, MCU pin readings and EN validity under the critical section. | [sensorarrayBringupPrepareAdsRefPath](../../core/board/sensorarrayBringup.c#L920) |
| `esp_err_t tmuxSwitchSelectRow(uint8_t row)` | 兼容入口，接受零基 row 并转交 TMUX1108 换行。 / Compatibility entry forwarding a zero-based row to TMUX1108 selection. | [matrixEngineSelectRowDefault](../../core/matrixEngine/matrixEngine.c#L41) |
| `esp_err_t tmuxSwitchSetSelAEnabled(bool enabled)` | 兼容入口，按已配置极性驱动 SELA。 / Compatibility entry forwarding logical enable to SELA. | [matrixEngineSelectColGroupDefault](../../core/matrixEngine/matrixEngine.c#L50) |
| `esp_err_t tmuxSwitchSetSelBEnabled(bool enabled)` | 兼容入口，按已配置极性驱动 SELB。 / Compatibility entry forwarding logical enable to SELB. | [matrixEngineSelectColGroupDefault](../../core/matrixEngine/matrixEngine.c#L50) |
| `esp_err_t tmuxSwitchSet1108Source(tmux1108Source_t source)` | 兼容入口，转交 TMUX1108 的 GND/REF 选择。 / Compatibility entry forwarding the TMUX1108 logical source. | [matrixEngineDriveDefault](../../core/matrixEngine/matrixEngine.c#L77) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static tmux1108Source_t tmux1108SourceFromConfig(int value)` | 将配置值 0 或非零映射为 GND 或 REF。 / Map a zero/nonzero configuration setting to logical GND/REF. | [tmux1108SelectRow](tmuxSwitch.c#L255) |
| `static void tmuxDelayUs(void)` | 执行 Kconfig 配置的切换微秒延迟。 / Wait the configured switching delay in microseconds. | [tmux1108SelectRow](tmuxSwitch.c#L255) |
| `static int tmux1108SourceToLevel(tmux1108Source_t source)` | 按板上 SW_REF_LEVEL 极性把逻辑源换算成 GPIO 电平。 / Map a logical source to its GPIO level using SW_REF_LEVEL. | [tmux1108ApplySourceNoLock](tmuxSwitch.c#L58) |
| `static void tmux1108ApplySourceNoLock(tmux1108Source_t source)` | 调用者持锁时将 source 写到 SW 引脚。 / Drive SW for source under the caller's lock. | [tmux1108SelectRow](tmuxSwitch.c#L255) |
| `static void tmux1108ApplyRowNoLock(uint8_t row)` | 将零基行号的三位写到 A0–A2。 / Drive A0–A2 from the low three bits of the zero-based row. | [tmux1108SelectRow](tmuxSwitch.c#L255) |
| `static bool tmux1134EnabledToSelLevel(bool enabled, int enabled_level)` | 根据 enabled_level 把逻辑开关转换为原始选择电平。 / Convert logical enable to a raw select level using the configured polarity. | [tmux1134SetSelAEnabled](tmuxSwitch.c#L445) |
| `static void tmuxSetCachedSelLevelNoLock(int gpio, int level)` | 用 gpio 标识更新对应 SEL 的命令缓存。 / Update the commanded SEL level associated with gpio. | [tmux1134ApplySelNoLock](tmuxSwitch.c#L95) |
| `static void tmux1134ApplySelNoLock(int gpio, bool level)` | 对已连接的选择脚写电平并更新命令缓存。 / Drive a configured select pin and update the commanded level. | [tmux1134SelectSelALevel](tmuxSwitch.c#L376) |
| `static int tmuxReadGpioLevelNoLock(int gpio)` | 读取 MCU 端引脚，未配置时返回 −1。 / Return the MCU pin level, with −1 for an unconfigured GPIO. | [tmuxSwitchGetControlState](tmuxSwitch.c#L496) |
| `static void tmux1134SetEnStateNoLock(bool on)` | 按有效极性操作可控 EN 并记录状态。 / Apply the configured EN polarity where controllable and update its state. | [tmux1134SelectSelALevel](tmuxSwitch.c#L376) |
| `static esp_err_t tmuxValidateConfig(void)` | 核对必需及已启用可选 GPIO 的范围。 / Validate the required and enabled optional GPIO numbers. | [tmuxSwitchInit](tmuxSwitch.c#L151) |
