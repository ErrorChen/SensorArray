# 测量模式

## 权威 route profile

| 模式 | SELA/SELB 前端 | SW 物理电平 | SW 逻辑源 | matrix excitation | ADS `INTREF` | ADS `REFMUX` | `VBIAS` |
| --- | --- | ---: | --- | --- | --- | --- | --- |
| SAFE | FDC 分支、停止扫描 | high | GND | off | entry 后 off/keep | 不用于普通 conversion | keep |
| CAP | FDC2214 | high | GND | off | off | AVDD/AVSS | on |
| VOLT | ADS ADC1 | high | GND | off | off | AVDD/AVSS | on |
| RES | ADS ADC1 | low | REF | REFOUT on | on | internal | on |

`REFOUT` 矩阵激励、`INTREF`、`REFMUX`、`VBIAS` 和 SW physical level 是五个独立状态。`MODE?`/`STATE?` 分别报告这些字段，主机不能从单个 bool 或 mode 名称反推未上报的硬件状态。

## 状态机

```text
UNINITIALISED -> SAFE -> TRANSITION -> CAP | VOLT | RES
                         |                 |
                         +---- fault ------+-> SAFE / DEGRADED
```

上电先进入无矩阵激励的 SAFE route。初始化、自检和 frontend 验证成功后，默认进入 CAP。

## 模式切换事务

发送：

```text
MODE=RES
```

控制任务立即返回：

```text
MACK,id=<requestId>,old=<mode>,new=RES,state=accepted
```

Core 1 在当前完整帧结束后处理请求：

1. 停止当前 conversion；
2. 撤销矩阵激励；
3. 切换 SELA/SELB 与 SW；
4. 配置 POWER、REFMUX、VBIAS、PGA 等；
5. readback 并等待 settle/discard；
6. 清除旧 mode payload、freshness 与相关 cache；
7. 更新 generation/request ID；
8. 输出 `MAPP`。

```text
MAPP,id=<requestId>,gen=<generation>,old=<mode>,new=RES,seq=<frameSeq>,state=applied,transitionUs=<us>
```

TRANSITION 期间不产生普通数据帧，所以单帧不会混入两个 mode。任何 GPIO、register、reference、settle 或 conversion 错误都会撤销激励并进入安全/降级状态。

## CAP

- 保留双 FDC worker 与 row-epoch 生产路径；
- FDC primary/secondary 并行完成 D1..D8；
- CAP 使用 `C/D/K` 协议；
- 返回 CAP 时恢复并验证两个 FDC；
- 不允许 ADS 改写 FDC calibration cache。

## VOLT

- 使用 ADS ADC1 扫描每个活动 row 的 D1..D8；
- 使用 AVDD/AVSS reference；
- 输出有符号整数 µV；
- 合法负电压不是无效值；
- 无效 cell 使用 `Xhh` 并在 masks 中给出原因。

### VOLT 的自动 rail monitor

普通 `MODE=VOLT` 不要求主机预先发送 `RAILCFG`。在完整帧边界，route controller 会先进入矩阵隔离的 `SAFE_RAIL_MONITOR`：SW/SELA/ADS reference 组合被设置为只读 internal analog supply monitor，完成 settle、采样和 readback 后再恢复 VOLT route。monitor 得到的 rail span 会写入当前 frame 的 `avdd/avss/rail/age`。

`RAILCFG=<AVDD_UV>,<negative_AVSS_UV>` 仍保留为易失外部 DMM debug override，用于需要绝对校准证据的实验；它不是普通用户前置条件。VOLT 内仍拒绝改变该 override：

firmware 会拒绝在 VOLT 内更新 rail：

```text
ERR,cmd=RAILCFG,reason=apply_before_volt
```

需要更新时先切回 CAP/RES，重新同步 DMM 值并应用。缺少有效 external rail 时，`MODE=VOLT` 即使先返回 queued 的 `MACK`，在帧边界也会以 `MERR` 失败并进入 SAFE；不能把 `MACK` 当作已经进入 VOLT。

ADS supply-monitor rail 是内部运行 health/reference 数据，不是同步外部 DMM 精度证据；若需要绝对精度报告，仍应单独保留外部 DMM 记录。

## Mixed-row profile

`ROWMODES=CVVRRVVC` 为 8 个物理 S 行建立原子 profile：每行只能为 `C`/`V`/`R`。应用帧按 CAP、VOLT、RES 分组，每组只对属于该组的物理行执行 route 与 FDC/ADS row subset read，输出 `M` header、多个 `MR` row records 和 `K` CRC trailer。`ROWMODES?`/`MODE?` 会同时报告 active/pending profile、generation、request ID 与 `layout=MIXED`。

## RES

- 使用 ADS ADC1、内部 reference、REFOUT 和配置的 `Rref`；
- row 切换使用 TMUX1108 break-before-make，并等待 `RESSETTLE`；
- 输出整数 mΩ，物理 Ω 值为 wire value × `10^-3`；
- PGA/autorange、open/short/range/negative 等状态按 cell 输出；
- S1D1/S8D8 约 10 kΩ 只是当前调试板 sanity，不属于算法常量。

## 帧边界辅助事务

AIN8 battery、ADSCHK、rail 和 zero job 不在 VOLT/RES row/column loop 中运行：

- CAP 先尝试 FDC conversion gap，预算不足则在完整帧边界运行；
- VOLT/RES 中允许执行的辅助事务只在完整矩阵帧后运行；VOLT 的物理 clamp 不允许用内部 supply-monitor 刷新 external rail；
- transaction 保存并恢复 ADS 状态，restore readback 失败会使普通 frame 失效并进入 SAFE/DEGRADED。

## Host 规则

- 把 `MACK` 视为 queued，不要立即切 UI；
- 只在匹配 `MAPP.id` 后提交 mode；
- 丢弃旧 generation/request ID 的 late frame；
- 每次切换至少等待一帧 CRC 正确、cell 完整且 freshness 符合的目标 mode 数据；
- 不要把 `X0D` 自动解释成 reboot 根因。
