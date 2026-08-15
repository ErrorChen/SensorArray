# 输出格式

## 通用约定

- 所有 wire message 是 ASCII；
- 每行以 LF 结束，host 应兼容 CRLF；
- cell 顺序为 row-major；
- `rows * 8 == cells == n`；
- DATA 与 LOG 完整 message 最大 1536 bytes；CTRL reply 最大 512 bytes；BLE CTRL RX command 上限 128 bytes；
- 超限不截断：DATA/LOG 超限发布 `TXDROP,...reason=payload_too_large`，CTRL reply 超限发布 `CTRLDROP,...reason=reply_too_large`，builder 直接拒绝而不是截断；
- transport 轮转使用有界 DATA batch：每轮先发送有限数量的 DATA，再让 LIFECYCLE 与 LOG 通过，且 LIFECYCLE 优先于普通 LOG；不是 DATA-always-first；
- frame 内部使用 IEEE CRC-32；
- BLE 分片外层还有一层完整 message CRC，见 [BLE 协议](ble-protocol.md)。

## CAP frame

```text
C,seq=<n>,ts=<us>,rows=<1..8>,cells=<rows*8>,gen=<g>,rid=<r>,rf=<hex>,pf=<hex>,sf=<hex>,expected=<hex>,acquired=<hex>,bad=<stale>/<mixed>/<invalid>,fmt=pf6,n=<cells>
D0,<最多16个值>
D1,<最多16个值>
...
K,seq=<n>,gen=<g>,rid=<r>,crc=<CRC32>
```

CAP `D` 值是 `pF * 1,000,000` 的有符号整数。无效 sentinel 为 `-1000000`，host 必须同时检查 freshness/validity context，不能把合法 `0 pF` 自动当作无效。

字段：

| Field | 含义 |
| --- | --- |
| `seq` | measurement frame sequence |
| `ts` | timestamp，µs |
| `rows` / `cells` / `n` | 当前活动行、cell 数 |
| `gen` | row configuration generation |
| `rid` | 已应用 row request ID |
| `rf` | row fresh mask |
| `pf` / `sf` | primary/secondary FDC fresh mask |
| `expected` / `acquired` | acquisition contract mask：expected 是本帧计划采集的 cell，acquired 是本帧实际完成读取的 cell；只覆盖 active cells，acquired 必须是 expected 的子集 |
| `bad` | stale frame / mixed epoch / invalid cell count |
| `fmt=pf6` | fixed-point pF，六位小数 |

## VOLT / RES frame

```text
V,seq=...,ts=...,rows=...,cells=...,gen=...,rid=...,mode=VOLT,unit=V,scale=-6,valid=...,fresh=...,error=...,expected=...,acquired=...,ref=...,rail=...,age=...,avdd=...,avss=...,vexc=...,rref=...,dur=...,tr=...,gc=...,ov=...,aa=...,fb=...,ir=...,to=...,st=...,spi=...,fmt=uv-x,n=...,bad=...
D0,<integer-µV-or-Xhh>,...
P0,<每个cell两个十六进制字符>
K,seq=...,gen=...,rid=...,crc=...
```

RES 的 header tag 为 `R`，关键差异：

```text
mode=RES,unit=ohm,scale=-3,fmt=mohm-x
```

VOLT 有效值是整数 µV；物理 V = value × `10^-6`。RES 有效值是整数 mΩ；物理 Ω = value × `10^-3`。

### Mask 与状态字段

| Field | 含义 |
| --- | --- |
| `valid` | 本帧可用 cell bitmask |
| `fresh` | 本帧获得新 conversion 的 cell bitmask |
| `error` | 有错误原因的 cell bitmask |
| `expected` / `acquired` | acquisition contract mask：expected 是本帧计划采集的 cell，acquired 是本帧实际完成读取的 cell；只覆盖 active cells，acquired 必须是 expected 的子集 |
| `ref` | ADS reference source |
| `rail` / `age` | rail 是否有效及 age |
| `avdd` / `avss` | rail split，µV |
| `vexc` | matrix reference/excitation 电压，µV |
| `rref` | reference resistor，Ω |
| `dur` / `tr` | frame / transition duration，µs |
| `gc` | gain change count |
| `ov` | overrange count |
| `aa` / `fb` | autorange attempts / fallback |
| `ir` | recovered I/O retry count |
| `to` / `st` / `spi` | DRDY timeout / stale / SPI error count |
| `bad` | invalid cell count |

### Acquisition 语义

- `expected` 与 `acquired` 独立于 `valid` / `fresh` / `error`：OPEN/SHORT 等 cell 只要完成 conversion/read，就是 acquired 且可以是 fresh，同时保持 `valid=0`、`error=1`；
- 未完成 acquisition 的 cell 必须 `acquired=0` 且不得声称 `fresh`；
- 两个 mask 都只允许覆盖 active cells（`rows*8`）；`acquired` 中任何不在 `expected` 内的 bit 都是协议错误。

### `Xhh`

无效 VOLT/RES cell 不输出 0，而输出：

```text
Xhh
```

`hh` 是 `sensorarrayCellError_t` 的两位十六进制错误码，例如 `X0D` 为 open、`X08` 为 saturated。host 必须保留未知错误码并显示 invalid，不得把它转换为数值 0。

## Mixed-row frame

当 `ROWMODES` 不是全行同一模式时，DATA 使用独立的混合帧。`profile` 始终是恰好 8 个字符，每字符对应一个物理 S 行；行数少于 8 时，非活动行输出尾随 `N`（例如 `rows=3` 时 `profile=CVRNNNNN`）。只有非 `N` 行会输出 `MR`，`N` 行永远没有 `MR`。

```text
M,seq=...,ts=...,rows=8,cells=64,rgen=...,rrid=...,pgen=...,prid=...,profile=CVVRRVVC,expected=<hex>,acquired=<hex>,fmt=mix1
MR,s=1,m=CAP,unit=pf,scale=-6,expected=<hex>,acquired=<hex>,valid=...,fresh=...,error=...,fmt=pf6,D=...
MR,s=2,m=VOLT,unit=V,scale=-6,expected=<hex>,acquired=<hex>,valid=...,fresh=...,error=...,fmt=uv-x,D=...
...
K,seq=...,rgen=...,rrid=...,pgen=...,prid=...,crc=...
```

字段：

| Field | 含义 |
| --- | --- |
| `rows` / `cells` | 当前活动物理行数（1..8）与期望 cell 数（`rows*8`） |
| `rgen` / `rrid` | row configuration（`ROWS`）generation / request ID |
| `pgen` / `prid` | ROWMODES profile generation / request ID |
| `profile` | 8 字符，每字符一行的期望 mode；非活动行尾随 `N` |
| `expected` / `acquired` | M header 为 64-bit 全局 mask；每个 `MR` 为该物理行的 8-bit mask；语义与 C/V/R 一致 |
| `MR.s` | 物理行号（1..rows），仅非 `N` 行存在 |

语义：

- expected：host 必须收到与 profile 非 `N` 行数量相等的 `MR` 行，每行恰好 8 个值；期望 cell 数由 `rows*8` 给出；
- acquired：`MR` 行携带实际获得的 cell；缺失 `MR` 行或 `D` 不足 8 个值均视为 malformed；
- M header 的 `expected`/`acquired` 必须与各 `MR` 行 mask 按 row-major 拼装结果一致，否则视为 malformed；
- `valid` / `fresh` / `error` 是每行 8-bit mask：`valid` 表示本行可用 cell，`fresh` 表示本帧获得新 conversion 的 cell，`error` 表示带显式错误原因的 cell；未获得新 conversion 的 cell 不得声称 fresh；
- `valid=0` 的 cell 输出 `Xhh`：VOLT/RES 行输出具体 `errorReason`（如 `X08` saturated、`X0D` open），CAP 行无效 cell 输出 `X14`（unsupported）；
- CRC 覆盖 `M` header 与全部 `MR` 行（每行含末尾 LF），不包括 `K`；host 还应确认 `K.seq/rgen/rrid/pgen/prid` 与 header 一致。

每个 `MR` 只承载一个物理行的 8 个 cell；CAP/VOLT/RES 可在同一 `M` 中并存，`D` 使用正常数值或 `Xhh` invalid token。主机应按 `s` 放回矩阵，不要按 mode 或 `MR` 到达顺序重排。

### `P` chunk

每个 cell 使用两个十六进制字符表示 literal PGA gain：

```text
01 02 04 08 10 20
```

`00` 表示经过 MODE2 readback 验证的 PGA bypass，不是未知 gain。

## Frame CRC

CRC 算法是 IEEE CRC-32：polynomial reflected `0xEDB88320`，golden `CRC32("123456789")=0xCBF43926`。

CRC 覆盖：

- 从 `C`、`V` 或 `R` header 的第一个 byte 开始；
- 包括每一行末尾 LF；
- CAP 覆盖所有 `D` 行；
- V/R 覆盖所有 `D` 与 `P` 行；
- M 混合帧覆盖 `M` header 与全部 `MR` 行；
- 不包括 `K` 行。

host 还应确认 `K.seq/gen/rid`（混合帧为 `K.seq/rgen/rrid/pgen/prid`）与 header 一致。

## 控制事件

| Tag | 含义 |
| --- | --- |
| `RCMD` / `RAPP` | rows accepted / applied |
| `MACK` / `MAPP` | mode accepted / applied |
| `RACK` / `RAPP` | external rail accepted / applied |
| `RMACK` | ROWMODES 的 source-local CTRL reply（`RMACK,id=...,old=...,new=...,state=accepted`），只回复给请求来源 |
| `RMAPP` / `RMERR` | ROWMODES 的 LIFECYCLE 事件，在 Serial、BLE `FF30` 与 Wi-Fi `LOG` 通道广播；每个 `RMACK` 必须且只能对应一个 terminal |
| `ACK` / `ERR` | source-local command reply |
| `BAPP` | battery request complete |
| `E,type=cmd` | legacy/runtime mailbox command applied event |

accepted 与 applied 不能合并处理。`RMACK` 是 CTRL reply，不广播；`RMAPP`/`RMERR` 是 LIFECYCLE 事件，不是 CTRL reply。host 必须以 request id 关联：每个 `RMACK` 恰好跟随一个 `RMAPP` 或 `RMERR`（exact-one terminal），缺少 terminal、重复 terminal 或无前置 `RMACK` 的 terminal 都是协议错误。

## 周期诊断日志

| Tag | 说明 | 关键字段 |
| --- | --- | --- |
| `SF50` | physical capture、emitted 与 per-sink rate | `seq,n,rows,cfps,efps,ofps,bad,drop,q` |
| `TR50` | FDC/row/ADS gap timing | `r,fu,rau,rmu,rt,wt,rp,rs,co,ag` |
| `OT50` | transport/output 状态 | `st,tx,out,q,ofps` 及 channel drop/slot counters |
| `BL50` | BLE link 与 fragment summary | `conn,sub,mtu,phy,mode,mq,ms,md,sentD,sentL,dropD,dropL,dropC,ctrlRetry,ctrlExhaust,fs,fe,cg,stale,conf,tiny` |
| `I2C50` | I2C health | 地址、success、NACK、timeout、recovery |
| `ADS50` | ADS matrix cache/autorange summary | `mode,n,frameUs,attemptsPerCell,rawConversions,profileHit,...` |
| `ADST50` | ADS timing breakdown | route/mux/register/DRDY/read/aggregation/autorange/battery/ADSCHK µs |
| `AB50` | battery scheduler 与结果 summary | `bt,valid,br,ageMs,run,validRun,invalidRun,restoreFail,retry,unstable,timeout,...` |
| `STK50` | task stack、heap 与 slot health | `unit=bytes`；`log/transport/usb/bleTx/bleCtrl/serialCtrl`；`heap/tSlot/tq/bSlot/...` |

`50`/`<n>` 是聚合周期，不代表协议 version。不同模式吞吐不同，必须分开看 `cfps` 与 sink `ofps`。

`STK50` 的 task 字段均为 `<configuredBytes>/<minimumRemainingBytes>`。其他紧凑字段为：

| Field | 含义 |
| --- | --- |
| `heap=<initial>/<current>/<minimum>` | 启动初始、当前与历史最低 free heap bytes |
| `tSlot=<used>/<highWater>` | 通用 transport 当前占用与历史峰值 slot 数 |
| `ta` / `ts` / `tr` | transport slot allocation fail / stale descriptor / release mismatch |
| `tq=<data>/<log>` | transport DATA / LOG queue drop |
| `bSlot=<used>/<highWater>` | BLE DATA/LOG slot 当前占用与历史峰值 |
| `ba` / `bs` / `br` | BLE slot allocation fail / stale generation drop / release mismatch |
| `bc` / `bf` | BLE slot CRC mismatch / fragment error |
| `trunc` | compact summary overflow/truncation counter；非零必须调查 |

compact summary 有意拆成两条独立合法 LOG message：第一条承载 `SF50/TR50/OT50/BL50/I2C50/STK50`，第二条承载 `ADS50/ADST50/AB50`。任一 message 都不得超过 1536 bytes；builder 超限时不发布半截 message，而是输出 `LOGTRUNC` 并增加 `trunc`。

## 异常与审计日志

| Tag | 含义 |
| --- | --- |
| `TXDROP` | transport pool/queue 有界 drop 或 payload 超限拒绝，包含 channel、len/max 与 frame metadata（`seq/rows/cells/gen/rid/pgen/prid`） |
| `CTRLDROP` | CTRL reply 超过 512-byte contract 时拒绝发布，绝不截断 |
| `BLECORRUPT` | BLE slot enqueue/dequeue length 或 CRC 不一致 |
| `CMDERR` / `CMDERR_SUM` | malformed/unsupported Serial command 与限流 summary |
| `LOGTRUNC` | compact summary 超过单条 1536-byte contract；该条不作为有效 LOG 发布 |
| `RST` | boot/reset breadcrumb；运行测试中出现新的 `RST` 视为 unexpected reboot |
| `BLERXERR` | BLE CTRL RX 长度或 queue full |
| `MFAULT` | mode/runtime fault，已进入 SAFE/DEGRADED |
| `APP_FATAL` | 初始化或运行 fatal |
| `Guru Meditation` / `Stack canary` / `panic'ed` | crash，HIL 必须立即失败 |

## Battery 输出

`ABAT` 是当前 battery snapshot，`AB50` 是周期 summary。至少同时使用：

```text
valid, fresh, ageMs, periodMs, reason, restore,
validRun, invalidRun, retry, unstable, timeout, spreadRaw
```

无效 battery 使用 `bt=-1,valid=0,reason=<name>`；不能把 -1 或 0 当作真实电压，也不能仅由电压推导 SOC。`lastGoodMv/lastGoodValid/lastGoodFresh/lastGoodAgeMs/lastGoodFrame` 是独立的 last-known-good 记录：电池断开时 AIN8 的浮动、长周期 PWM 或异常范围只更新 latest invalid reason，不清除 last-good，也不使矩阵 frame invalid。

每次实际检测到无效电池时，固件还会立即尝试发布一条普通异步诊断事件 `BATERR`，例如：

```text
BATERR,seq=...,err=0x...,reason=range_error,valid=0,lastGoodMv=...,lastGoodValid=1,sampleUs=...,restore=ok,action=report_continue
```

`BATERR` 是普通异步诊断事件：只报告电池诊断结果，不触发 SAFE，不进入高优先级 lifecycle 队列，也不让采集线程等待 transport；电池 transaction 的 ADS 状态仍必须完成 restore。电池切换后的 ADS126x 转换稳定 guard 默认是 50 us，矩阵 route/row 转换稳定 guard 分别是 500/200 us（原为 5000/2000 us），三者都仍要求 fresh discard/DRDY freshness；`DRDY_TIMEOUT_US` 仍是独立的 600 us 故障上限。
