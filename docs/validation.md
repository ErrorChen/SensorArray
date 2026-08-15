# 构建、自动测试与 HIL 验证

## 验证原则

以下项目必须分开报告：

```text
BUILD
HOST / PROTOCOL TESTS
SERIAL HIL
BLE HIL
CAP / RES / VOLT
LONG-RUN
STACK / HEAP / SLOT
```

build、boot log 或 BLE connect 不能替代端到端验收。缺少串口、硬件或 BLE adapter 时，写明 `NOT RUN` 与原因，不能写 PASS。

## 状态模板与证据边界

任何结论必须落在下列状态之一，不能用“大概通过”“与上次相同”代替：

| Status | 含义 |
| --- | --- |
| `PASS` | 本次 freshly run 的 wire log / machine-readable summary 证明门槛全部满足 |
| `FAIL` | 本次 run 触发失败门槛（CRC、reboot/panic、超时、缺帧等） |
| `NOT RUN` | 未执行，写明原因（如无 BLE adapter） |
| `BLOCKED` | 缺少前置条件或外部授权，无法在当前环境执行，写明原因 |
| `NOT VERIFIED` | 没有同步外部标准/DMM 或未覆盖，不能宣称精度/范围已证明 |

规则：

- 当前硬件证明只来自本次 freshly run；历史 artifact 只保留在“历史实测结果”章节，不能转成当前 PASS。
- Serial stage 必须在 `/dev/ttyACM0`（或 `--port` 指定端口）先完成；本轮最终验收门槛是当前已烧录 image 的连续 1200 秒（20 分钟）Serial HIL。只有该 stage 返回 PASS 后才启动 BLE，BLE 最终门槛为 600 秒（10 分钟）。120 秒仅用于 smoke/快速回归，不能替代本轮最终门槛。
- 串口不可用或 serial stage 无法完成时，BLE 直接标 `BLOCKED`/`NOT RUN`，不启动 BLE stage。
- 自动 HIL 未执行受控 reset 时，`RECOVER`/`RECOVER=0|1|2`/`RESTART` 的 reboot/recovery 验收保持 `BLOCKED`；只有显式受控 reset 与重启后恢复检查才能标 PASS/FAIL。
- 没有同步 DMM/标准件时，VOLT/RES accuracy 一律 `NOT VERIFIED`，不能因 pipeline sanity 数字写成 PASS。

## Clean build

在 ESP-IDF 5.5.5 环境中：

```powershell
idf.py --version
idf.py fullclean
idf.py build
```

保存并报告：

- build success/failure；
- warning 数量和内容；
- firmware binary size；
- DRAM、IRAM、BSS；
- 最终 `sdkconfig` 与 `build/config/sdkconfig.h` 中的 task stack 配置。

不允许新增 format truncation、array bounds、unused static、type conversion 或 stack usage warning。

## Host 与协议测试

使用仓库虚拟环境：

```powershell
$python = ".\.venv\Scripts\python.exe"
& $python -c "import ast, pathlib; [ast.parse(path.read_text(encoding='utf-8')) for path in pathlib.Path('tools').glob('*.py')]; print('PY_SYNTAX_CHECK,passed=1')"
& $python tools\test_text_protocol.py
& $python tools\test_measurement_logic.py
& $python tools\check_command_docs.py
& $python tools\check_no_legacy_config_usage.py
& $python tools\audit_config_surface.py
```

当前测试职责：

| Test | 覆盖 |
| --- | --- |
| `test_text_protocol.py` | CAP/V/R parse、frame CRC、`Xhh`、PGA、battery/ADS telemetry、1536-byte worst-size fixture |
| `test_measurement_logic.py` | 编译并运行硬件无关 C tests：mode、rail/math、RES、PGA/cache、battery scheduler、route readback、congestion policy |
| `check_command_docs.py` | 从四层 parser 提取 command literals，与 `command-reference.md` 比较 |
| `check_no_legacy_config_usage.py` | 禁止业务源码重新引用已迁移配置名 |
| `audit_config_surface.py` | Kconfig/default/usage surface audit |

firmware boot 还会运行：

- `MSELF`：measurement logic self-test；
- `PSELF`：text builder、CRC 和最坏帧上限 self-test。

## HIL 依赖

HIL 工具为：

```text
tools/sensorarray_hil.py
```

安装 host-only 依赖：

```powershell
& $python -m pip install -r tools\requirements-hil.txt
```

Serial port 可由 `--port COMx` 或 `SENSORARRAY_SERIAL_PORT` 指定。不指定时工具只能列出候选，不应在多个设备间危险地自动选择。

VOLT HIL 默认验证 firmware 在矩阵隔离的 `SAFE_RAIL_MONITOR` 窗口自动读取 ADS126x internal analog supply monitor。外部 DMM `RAILCFG` 仍可作为可选 debug override，但不是普通 VOLT 前置条件。电池已断开时，AIN8 浮动/PWM/异常值应记录为 battery diagnostic invalid，不作为 acquisition failure。

下面的命令是可重复验收入口，不代表文档提交者已经在当前设备上运行或取得 PASS。只有保留下来的 wire log、machine-readable summary 和测试报告才能作为执行证据。

## Transport 验收顺序（Serial -> BLE）

可执行契约入口是 `tools/validate_transports.py`。Ubuntu 下默认串口为 `/dev/ttyACM0`，可用 `--port` 覆盖；同一 flashed image 必须先完成 serial stage，serial 返回非零或超时时立即中止，不启动 BLE。BLE stage 至少 60 秒，并把同一个 `/dev/ttyACM0` 作为 reset/panic sidecar 传给 `receive_ble_text.py`（`--serial-port`）。

串口不可用时（端口不存在、打开失败或 serial stage 超时/FAIL），serial stage 记录 `NOT RUN`/`FAIL`，BLE stage 记录 `BLOCKED`/`NOT RUN` 并立即中止；任何情况下都不允许跳过 serial stage 直接运行 BLE。

时长规则：

- `--serial-duration`：serial stage 秒数，默认 120；
- `--ble-duration`：BLE stage 秒数，默认 60；小于 60 被拒绝；
- `--duration`：legacy 兼容项，未给 stage-specific 参数时同时作为 serial/BLE/Wi-Fi duration；推荐改用独立参数。

Linux/bash 可重复入口：

```bash
set -o pipefail
mkdir -p validation_artifacts
# 同一已刷写 image、串口 /dev/ttyACM0、BLE adapter 已就绪
python3 tools/test_text_protocol.py 2>&1 | tee validation_artifacts/transport_host_text.log
python3 tools/test_validate_transports.py 2>&1 | tee validation_artifacts/transport_validate_host.log
python3 tools/test_sensorarray_hil.py 2>&1 | tee validation_artifacts/transport_hil_host.log
python3 tools/check_command_docs.py 2>&1 | tee validation_artifacts/transport_command_docs.log
python3 tools/validate_transports.py \
  --port /dev/ttyACM0 \
  --ble \
  --serial-duration 120 \
  --ble-duration 60 2>&1 | tee validation_artifacts/transport_serial_ble.log
```

Serial 成功门槛：serial stage 返回 0 后还必须看到 `SERIAL_DONE` summary，且 `lines` 与 frame count（`cap+volt+res`）> 0、`crc/bad/ascii/gap/missing` 均为 0；输出中出现 `ESP-ROM`、`rst:`、`RST,reason=`、`Guru Meditation`、`Stack canary`、`panic'ed`、`LoadProhibited`、`Watchdog` 任一标志即拒绝进入 BLE（`VAL,stage=serial,state=fail,reason=...`）。BLE 成功门槛：必须看到 `SER_SIDE,port=/dev/ttyACM0,...status=open` 与 `SER_SIDE_DONE,resets=0,lastError=none`，`BRX` 的 `ok > 0`（至少收到一个完整可解析消息/帧）、`crc=0`、`miss=0`，且存在 `BF` 行时 `cf/ms/dr` 均为 0；`BLE_TEST_SKIPPED` 或 `BLE_TEST_FAILED` 一律不算 PASS。

`validate_transports.py` 会原样回显子进程输出，但不会自动保存 wire log；需要保留输出时使用上面的 `set -o pipefail` + `tee` 管道，并以管道退出码为准。完整 full-profile artifact（summary JSON、wire log）仍用 `sensorarray_hil.py --output-directory` 生成。

BUILD、HOST/PROTOCOL TESTS、SERIAL HIL、BLE HIL 的证据都必须来自本次验证：不能拿历史 artifact 或旧 commit 的 PASS 替代当前 firmware 的 serial/BLE 验收。`validate_transports.py` 是可执行顺序契约；完整 full profile 仍需按下面的 `sensorarray_hil.py` 入口运行并单独报告。本文档不声明本次 HIL 已通过；未运行或无 BLE adapter 必须写 `NOT RUN`。

## Serial HIL

完整验收入口：

```powershell
& $python tools\sensorarray_hil.py serial `
  --profile full `
  --port COMx `
  --output-directory validation_artifacts\serial_full
```

必须覆盖：

```text
TX?
ST?
BTX?
WIFI?
MODE?
STATE?
MODE=CAP
MODE=RES
MODE=VOLT
MODE=CAP
```

每次 mode switch：

1. 等 `MACK`；
2. 等匹配 request ID 的 `MAPP`；
3. 等目标 mode 的完整、CRC 正确 frame；
4. 检查 generation/request ID；
5. 至少收 10 帧；
6. 禁止旧 mode/generation frame 被标为 fresh。

快速回归持久测试至少 120 秒（2 分钟），并保持 ADS autorange、battery periodic、compact summary 和 8×8 输出开启；本轮最终 acceptance 仍必须使用 1200 秒 Serial 与 600 秒 BLE。所有阶段同时记录完整 frame 数。S1D1/S8D8 可用 `5 kΩ < value < 20 kΩ` 做宽松 pipeline sanity；其他 open cell 不要求固定数值。

Serial observer 必须把以下任一新出现记录为 FAIL：

```text
ESP-ROM
rst:
RST,reason=
Guru Meditation
Stack canary
panic'ed
LoadProhibited
Watchdog
```

## BLE HIL

完整验收入口：

```powershell
[int]$railAvddUv = Read-Host "输入同步外部 DMM 实测 AVDD->GND（uV，正数）"
[int]$railAvssUv = Read-Host "输入同步外部 DMM 实测 AVSS->GND（uV，负数）"
if ($railAvddUv -le 0 -or $railAvssUv -ge 0) { throw "rail signs must be AVDD>0 and AVSS<0" }
& $python tools\sensorarray_hil.py ble `
  --profile full `
  --serial-port COMx `
  --rail-avdd-uv $railAvddUv `
  --rail-avss-uv $railAvssUv `
  --output-directory validation_artifacts\ble_full
```

BLE 测试期间 Serial observer 必须同时运行；BLE disconnect 本身不能区分客户端问题与设备 reboot。

### Subscription matrix

| Case | FF11 | FF20 | FF30 | 必须观察 |
| --- | ---: | ---: | ---: | --- |
| connected only | 0 | 0 | 0 | 1–2 分钟稳定；无 BLE DATA/LOG queue pressure |
| CTRL only | 1 | 0 | 0 | FF10 query -> FF11 reply；无 DATA/LOG |
| DATA only | 0 | 1 | 0 | 至少 1000 DATA message 或 2 分钟；无 LOG forwarding |
| LOG only | 0 | 0 | 1 | SF50/TR50/ADS50 等；无 measurement DATA |
| CTRL + DATA | 1 | 1 | 0 | ACK 与 DATA 并行，CTRL 不饥饿 |
| CTRL + LOG | 1 | 0 | 1 | ACK 与 LOG 并行 |
| DATA + LOG | 0 | 1 | 1 | 高流量组合，CRC/slot/drop 稳定 |
| full | 1 | 1 | 1 | CTRL/DATA/LOG 同时工作 |

每个分片 message 必须检查：

- `(channel,messageId)`；
- index/count 完整；
- chunk length；
- assembled total length；
- envelope CRC32；
- DATA 内部 `K` CRC32。

### FAST / SAFE

两种模式均测试：

```text
BTX=FAST
BTX?
BTX=SAFE
BTX?
BTX=FAST
```

FAST 使用 Notify bit；有 Indicate bit 时 SAFE 使用 Indicate confirmation，不能因 timeout 长期占用 slot。只有 Notify 的 Ubuntu/BlueZ 客户端使用有界 SAFE Notify fallback。

在支持选择 CCCD 的客户端中，发送 `BTX=SAFE` 前应把 FF11 的 Indicate bit 打开；需要继续接收 DATA/LOG 时，FF20/FF30 也应先打开 Indicate。推荐三个 TX characteristic 都使用 `0x0003`。Ubuntu/BlueZ 的 Bleak 无法在 Notify/Indicate 并存时选择 Indicate，固件会为这种客户端启用 SAFE Notify fallback，ACK、DATA、LOG 仍应保持可验收。

### Stress

- full subscription 下循环 `CAP -> RES -> VOLT -> CAP` 20–50 次，每 mode 至少 5 帧；
- FF20/FF30 subscribe/unsubscribe 尽可能达到 100 cycles；
- connect/subscribe/query/data/disconnect 至少 30 cycles；
- BLE + Serial observer + RES 至少 120 秒（2 分钟）；
- RES 长跑每 30–60 秒发一次 `STATE?`，证明 CTRL 仍存活。

## nRF Connect 最小手工验收

1. Scan `CscArray_*`；
2. Connect Service `0x00FF`；
3. 请求 MTU 247（客户端支持时）；
4. 为 `FF11`、`FF20`、`FF30` 同时开启 Notify 与 Indicate（CCCD `0x0003`，或在 UI 中分别点开两种订阅）；
5. 写 `STATE?\n` 到 `FF10`，确认 `FF11` 收回复；
6. 确认 `FF20` 持续 DATA、`FF30` 持续 LOG；
7. 写 `BTX=SAFE\n`，确认 `FF11` 通过 Indicate（Linux/BlueZ fallback 时为 Notify）返回 `ACK,cmd=BTX,v=SAFE`；
8. 写 `BTX?\n`，确认仍为 SAFE；再写 `BTX=FAST\n` 并确认 Notify 回复；
9. disable `FF30`，确认 `FF20` 继续 DATA；re-enable `FF30`；
10. 切 RES 并运行数分钟；
11. 写 `MODE=VOLT\n`，等待自动 monitor 后的 `MACK`、`MAPP` 与 FF20 的完整 VOLT DATA；需要外部 DMM override 时才在 CAP/RES 下发送 `RAILCFG`；
12. 写 `ROWMODES=CVVRRVVC\n`，等待 `RMACK`、`RMAPP`，并验证 `M/MR/K` 混合帧按 S 行重组；
13. disconnect；
14. reconnect，重新设置 CCCD 后重复 query/data。

整个过程 Serial 不得出现 reboot/panic。

nRF Connect 能验证人工 CCCD、命令与链路行为，但 raw fragment 计数不能替代自动 HIL 的重组、总长度和双层 CRC 校验。

## Stack、heap 与 slot

最坏测试至少包括：

```text
CAP  + Serial + BLE DATA/LOG
RES  + Serial + BLE DATA/LOG
VOLT + Serial + BLE DATA/LOG
```

报告真实 high-water，字段单位必须是 bytes：

| Task | 验收目标 |
| --- | ---: |
| `sensorarrayLogTask` | minimum remaining >= 2048 bytes，优先目标 3072–4096 |
| transport | 记录 minimum remaining |
| USB sink | 记录 minimum remaining |
| BLE TX | 记录 minimum remaining |
| BLE CTRL | 记录 minimum remaining |
| Serial CTRL | 记录 minimum remaining |

同时报告：

```text
initial free heap
minimum free heap
after-warmup free heap
final free heap
transport slot high-water
BLE slot high-water
DATA/LOG queue drops
BLE fragment errors / CRC mismatch
stale descriptor / release mismatch
```

允许 BLE/FreeRTOS lazy allocation 带来一次性下降，不允许 free heap 随 frame count 持续下降。

## 性能回归

分别记录 CAP/RES/VOLT physical FPS 与 emitted FPS。与同硬件、同 rows、同日志配置的 baseline 比较；下降超过 5% 必须调查，不能通过关闭 8×8、battery、ADS50/AB50、autorange 或 FF30 规避。

## 历史实测结果（不作为当前 HEAD 证据）

> 本节所有条目均为此前 commit/日期留下的历史记录，provenance 指向当时的 flash image 与 validation artifact。当前 HEAD 的 BUILD/HOST/SERIAL/BLE 验收必须重新运行并保存本次 artifact；历史目录、旧 commit 的 PASS 与旧 wire log 不能作为当前 firmware 验收证据。

本节只记录历史 artifact 已证明的范围。Targeted PASS 不替代 complete full profile，Bleak 自动化也不替代 nRF Connect 人工验收或外部 DMM 精度证据。

### 2026-08-13 `/dev/ttyACM0` 重试（历史）

- 固件刷写：PASS。ESP32-S3，应用镜像 SHA 校验通过，端口 `/dev/ttyACM0`。
- Serial smoke：PASS。CAP/RES/VOLT/CAP 模式切换、MAPP、CRC 和启动检查通过。
- 2 分钟持续采集：PASS。`serial_persistence_120s` 实测 2355 个 RES 完整帧，120.047 s，19.617 fps，CRC=0，sequence gap=0，运行期间 reset=0；S1D1=10043.711 Ω，S8D8=10041.236 Ω。
- Serial full profile：NOT PASS（telemetry prerequisite）。板子启动时报告 `APP_LOG_INIT_FAIL,err=0x101`，async logger 因内部堆不足未创建，因此 full profile 缺少 `STK50`，不能宣称完整稳定性验收 PASS。该次采集本身未观察到 post-ready reset 或 CRC 错误。
- 电池已断开：AIN8 浮动、长周期 PWM 或异常电压按 battery diagnostic invalid 处理，不作为矩阵采集失败。

| 阶段 | Status | 实测范围 | Artifact |
| --- | --- | --- | --- |
| Serial full profile | **PASS** | CAP/RES/VOLT/CAP、RES 2000-frame long-run、Serial wire/reset/stack/heap | [`final_serial_full`](../validation_artifacts/final_serial_full/serial_hil_summary.json) |
| Final-image Serial smoke | **PASS** | 最终 flash 上 6 query、CAP/RES/VOLT/CAP 各 10 帧、RES sanity、CRC/reset | [`final_serial_smoke_final_pass`](../validation_artifacts/final_serial_smoke_final_pass/serial_hil_summary.json) |
| BLE targeted long-run + reconnect | **PASS** | RES 2000 complete DATA、30 reconnect、Serial sidecar、FAST bounded-loss telemetry | [`final_ble_longrun_reconnect_wwr`](../validation_artifacts/final_ble_longrun_reconnect_wwr/ble_hil_summary.json) |
| nRF Connect 手工验收 | **NOT RUN** | 未执行人工 CCCD/客户端流程 | 无 |
| BLE complete full profile | **PASS** | 完整 subscription matrix、FAST/SAFE、20-cycle mode、100-cycle subscribe、RES 2000-frame、30 reconnect | [`final_ble_full_pass5`](../validation_artifacts/final_ble_full_pass5/ble_hil_summary.json) |

### Serial `final_serial_full`: PASS

summary 的 `passed=true`，运行端口为 COM12，profile 为 `full`。RES long-run 实测：

| 指标 | 实测值 |
| --- | ---: |
| 完整 RES frames | 2000 |
| Duration | 1193.687 s |
| Complete-frame rate | 1.675 fps |
| S1D1 sanity | 10050.623 Ω |
| S8D8 sanity | 10046.781 Ω |
| Post-ready wire CRC / malformed / non-ASCII | 0 / 0 / 0 |
| Unexpected reset/panic | 0 |

Heap 在 warmup 后没有继续下降：

| Initial free | Minimum free | Warmup free | Final free | Warmup -> final |
| ---: | ---: | ---: | ---: | ---: |
| 49500 B | 25596 B | 25612 B | 25612 B | 0 B |

该 Serial run 的 stack high-water telemetry 单位均为 bytes：

| Task | Configured stack | Minimum remaining |
| --- | ---: | ---: |
| `sensorarrayLogTask` | 12288 | 9344 |
| transport | 6144 | 4020 |
| USB sink | 6144 | 4228 |
| BLE TX | 4096 | 3292 |
| BLE CTRL | 6144 | 5236 |
| Serial CTRL | 6144 | 2516 |

此 run 使用 `ST=SER`，因此 transport/BLE slot high-water 均为 0；它们不是 BLE pool 压力证据。

### BLE targeted `final_ble_longrun_reconnect_wwr`: PASS

summary 的 `passed=true`，profile 字段为 `full`，但本 artifact 只执行 `longRun + reconnect` targeted phases；不能将它标成完整 BLE full-profile PASS。实测：

| 指标 | 实测值 |
| --- | ---: |
| 完整 RES DATA frames | 2000 |
| Duration | 108.219 s |
| Complete-frame rate | 18.481 fps |
| S1D1 sanity | 10050.655 Ω |
| S8D8 sanity | 10046.588 Ω |
| Reconnect cycles | 30 |
| BLE wire CRC / malformed / non-ASCII | 0 / 0 / 0 |
| Serial sidecar CRC / malformed / non-ASCII | 0 / 0 / 0 |
| Unexpected reset/panic | 0 |

FAST 保持 non-blocking，因此该 targeted window 允许有界 DATA loss，但不把不完整 message 当作完整 frame：host 观察到 4 个 stale incomplete fragment assemblies，同时仍完成并校验 2000 个 RES DATA messages。summary 还记录 `missing_frames=30`、`sequence_gaps=9`、window 结束时 `pendingBoundaryFragments=1`，firmware DATA drop delta 为 8；这些不是 CRC corruption。firmware `fe=0`、`dropC=0`、`ctrlExhaust=0`，CTRL 没有被 DATA/LOG 饿死。

Reconnect 阶段结束时的 heap：

| Initial free | Minimum free | Warmup free | Final free | Warmup -> final |
| ---: | ---: | ---: | ---: | ---: |
| 46332 B | 7340 B | 19084 B | 19084 B | 0 B |

最终 reconnect telemetry 的 stack 与 slot high-water：

| Task | Configured stack | Minimum remaining |
| --- | ---: | ---: |
| `sensorarrayLogTask` | 12288 | 9292 |
| transport | 6144 | 4004 |
| USB sink | 6144 | 4048 |
| BLE TX | 4096 | 1948 |
| BLE CTRL | 6144 | 2884 |
| Serial CTRL | 6144 | 3108 |

```text
transport slot high-water = 3
BLE slot high-water       = 7
```

### 精度证据边界

上述两次 HIL 向 `RAILCFG` 提供的 split 为 `3341176,-1822460` µV；该 split 来自 onboard ADS supply monitor，用途仅为验证 mode/command、VOLT protocol 与 transport stability。它不是同步外部 DMM 测量，不能作为 VOLT absolute accuracy 或 rail calibration accuracy 证据。

S1D1/S8D8 数字只证明当前约 10 kΩ 调试件落在宽松 pipeline sanity 范围；本轮没有同步 DMM/标准电阻对照，因此 resistance absolute accuracy 同样 **NOT VERIFIED**。nRF Connect 手工验收也 **NOT RUN**。

### BLE complete `final_ble_full_pass5`：PASS

该 artifact 来自最后一次 `fullclean`、build、flash 和 `verify_flash` 后的同一镜像，summary 为 `passed=true`，完整执行所有六个 phase：

| Phase | 实测结果 |
| --- | --- |
| Subscription matrix | NONE 60.015 s；FF11 only；FF20 only 1000 DATA；FF30 only 4 LOG；FF11+20；FF11+30；FF20+30；FF11+20+30 全部 PASS |
| TX modes | FAST/Notify → SAFE/Indicate（Linux/BlueZ 可为 Notify fallback）→ FAST/Notify，完成 DATA/LOG/CTRL，completed-fragment CRC error=0 |
| Mode stress | 20 cycles、80 transitions、542 complete frames（CAP 265 / RES 142 / VOLT 135），stale=0、CRC/malformed/non-ASCII/regression=0 |
| Subscribe stress | 100 cycles、200 subscription windows、200 unsubscribe quiet checks、grace notification=0 |
| RES long-run | 2000 required frames，107.391 s，18.624 complete fps；S1D1=10050.302 Ω、S8D8=10046.572 Ω |
| Reconnect | 30/30 cycles，1041 complete DATA frames；completed CRC error=0、stale=0、CTRL drop=0 |

RES long-run 的完整 envelope 统计：CTRL/DATA/LOG message 为 `5/2001/80`，notification 为 `10/8005/360`；三通道的 CRC、missing、gap、dropped、duplicate、out-of-order、tiny-fragment error 均为 0。DATA 内层 frame CRC、Serial sidecar CRC/malformed/non-ASCII 均为 0；unexpected reset/panic 为 0。window 结束时有 1 个不足 5 秒的新鲜 boundary partial，未被当作完整 message，`staleFragments=0`。

完整 run 的最坏 stack telemetry（单位 bytes）：

| Task | Configured stack | Minimum remaining |
| --- | ---: | ---: |
| `sensorarrayLogTask` | 12288 | 9292 |
| transport | 6144 | 3996 |
| USB sink | 6144 | 4160 |
| BLE TX | 4096 | 1960 |
| BLE CTRL | 6144 | 2864 |
| Serial CTRL | 6144 | 3116 |

Heap 与 pool：

```text
initial free heap             = 46180 B
minimum free heap             = 8400 B
after BLE warmup free heap    = 18804 B
final free heap               = 18804 B
warmup -> final delta         = 0 B
transport slot high-water     = 3 / 4
BLE slot high-water           = 6 / 8
transport queue drop DATA/LOG = 0 / 0
transport alloc/stale/release = 0 / 0 / 0
BLE stale/release/CRC/fragment= 0 / 0 / 0 / 0
BLE lifecycle cancellation    = 13
BLE channel drop DATA/LOG/CTRL= 25 / 1 / 0
```

FAST/non-blocking 设计允许在 unsubscribe/reconnect 或 BLE slot 满时有界丢弃，不能反向阻塞 acquisition。`lifecycle cancellation=13` 是 CCCD/连接 generation 在发送期间变化后主动取消旧 descriptor；它与 malformed fragment 分开统计。BLE slot alloc fail=12、DATA/LOG drop=25/1 均未造成 transport queue drop、CTRL drop、CRC corruption 或 frame regression。

## 发布结果格式

Serial 与 BLE 分开给结论，并使用统一状态模板：

```text
SERIAL PASS / FAIL / NOT RUN / BLOCKED
BLE PASS / FAIL / NOT RUN / BLOCKED
REBOOT / RECOVERY HIL PASS / FAIL / NOT RUN / BLOCKED
ACCURACY VERIFIED / NOT VERIFIED
```

每个 PASS 都要附 frame/message 数、duration、CRC、reset count、mode cycles、stack minimum、heap 与 drop counters。没有 BLE adapter 时应写：

```text
BLE HIL NOT RUN: reason=host BLE adapter unavailable
```

历史章节的 PASS 不进入当前状态模板；当前结论只能来自本次 freshly run 的证据。未做受控 reset 时 reboot/recovery 写 `BLOCKED: reason=no controlled reset procedure`；没有同步 DMM/标准件时 accuracy 写 `NOT VERIFIED: reason=no synchronized DMM/standard`。这不影响继续完成 build、host tests、Serial HIL 与 BLE HIL 工具测试。
