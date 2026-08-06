# Measurement modes / 测量模式

## 中文

上电首先进入无矩阵激励的安全路由，初始化和 self-test 成功后默认进入 `CAP`。
`MODE=CAP`、`MODE=VOLT`、`MODE=RES` 共用 Serial/BLE/Wi-Fi parser；不存在按
transport 分裂的模式状态。

| 模式 | SELA/SELB 前端 | TMUX1108 SW 物理电平 | SW 逻辑源 | 矩阵激励 | ADS INTREF | ADS REFMUX | VBIAS |
|---|---|---:|---|---|---|---|---|
| SAFE | FDC 分支、无扫描 | high | GND | off | off after entry | 不用于转换 | keep |
| CAP | FDC2214 | high | GND | off | off | AVDD/AVSS | on |
| VOLT | ADS126x | high | GND | off | off | AVDD/AVSS | on |
| RES | ADS126x | low | REF | REFOUT on | on | internal | on |

这里的“矩阵 REF”是经 TMUX1108 接入矩阵的 REFOUT 激励；`INTREF` 是 ADS126x
内部 reference 电路；`REFMUX` 决定转换使用的 reference；`VBIAS` 把 AINCOM
偏置到模拟电源中点；`SW high/low` 只是 MCU 物理电平。五者是独立状态，代码
分别建模并在 `MODE?` snapshot 中报告，调用者不得从一个 bool 或标签字符串
推导另一个状态。

当前板级逻辑经电路图与实现核对：SELA/SELB `1` 选择 TMUX1134 A/FDC 分支，
`0` 选择 B/ADS 分支；SELA 服务 D1..D4，SELB 服务 D5..D8。TMUX1108 common D
连 REFOUT，外部 Q1 的 SW high 把该节点拉到 GND，SW low 释放 REFOUT。所有原始
GPIO 映射集中在 `core/board/sensorarrayBoardMap.c`。

状态机为 `UNINITIALISED -> SAFE -> TRANSITION -> CAP|VOLT|RES`，并有
`DEGRADED`/`FAULT`。命令先立即发布 `MACK ... state=accepted`，Core 1 在当前帧
完整结束后开始 transition；transition 期间不产生普通数据帧，成功后发布带
requestId、generation、旧/新模式和应用帧序号的 `MAPP`。切换会清除旧模式
payload/freshness 和 ADS PGA cache，但不破坏 FDC 生产校准 cache。

任何 GPIO/ADS readback、rail/reference、settle 或 conversion 错误都会撤销激励、
停止 conversion 并退回安全状态。CAP、VOLT、RES 的模拟建立和转换时间不同，
所以其 FPS 不保证相同；以运行时 `dur`、队列、drop、heap 和 stack telemetry 为准。

进入 RES 时先按完整安全流程释放 Q1 clamp、开启 INTREF/REFOUT 并 readback。
原理图显示 TMUX1108 EN 只有上拉、不可由 MCU 控制；session 内换 row 的默认快路径
因此只在当前 source 已确认是 REF 时改变 A0..A2，依赖 TMUX1108 器件级
break-before-make，再等待运行时 `RESSETTLE` 并核对 GPIO、POWER、REFMUX。配置关闭
快路径时仍使用逐行 STOP1/INTREF-off/Q1-clamp 的保守序列。绝不能在 Q1 clamp
REFOUT 到 GND 时开启 INTREF。

VOLT/RES 始终扫描每个活动 row 的 D1..D8。两个 target FPS 默认 `0=unlimited`；
`FPSCAP=OFF` 关闭 Core 1 limiter，`OUTCAP` 仅在 Core 0 丢弃/节流输出。进入
VOLT/RES 时双 FDC 只在模式边界 sleep 并各 readback 一次，`MODE?`/`STATE?`
报告 active/sleep 和 verified；返回 CAP 后才恢复 worker。

## Australian English

Boot starts with a passive safe route and enters `CAP` only after successful
initialisation and self-tests. One parser and mailbox serve all transports.
Requests are accepted immediately, but Core 1 applies them at a complete-frame
boundary. No ordinary frame is produced during `TRANSITION`, and a frame never
contains values from two modes.

Matrix excitation REF, ADS INTREF, ADS REFMUX, VBIAS, and the SW physical level
are separate states. The table above is the authoritative board profile. A
failed readback or conversion removes excitation, stops conversion, invalidates
old data, and enters a safe/degraded state. During a resistance row change,
ADC1 and INTREF are stopped before REFOUT is clamped; excitation is restored
only after the row has changed. On this board TMUX1108 EN is pull-up only, so
the normal RES-session row change keeps INTREF active and uses the device's
break-before-make address transition plus a configurable, readback-checked
settle. The clamp/restart sequence remains the fallback. VOLT/RES capture
targets default to unlimited; output caps never pace Core 1 acquisition.

## Frame-boundary auxiliary transactions

CAP keeps both FDC workers active. VOLT and RES put both FDC devices to sleep
once at the mode boundary and require CONFIG readback before acquisition; the
mode snapshot exposes both sleep/active states and verification bits. AIN8,
ADSCHK, rail, and zero jobs never run inside the VOLT/RES row/column loops.
CAP offers a conversion gap first, then uses the immediately following full
frame boundary if the measured job plus guard cannot fit. This boundary policy
does not change the requested measurement mode or its generation.
