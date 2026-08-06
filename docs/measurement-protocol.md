# Measurement text protocol / 测量文本协议

## 中文

### 命令与状态

```text
MODE?
STATE?
MODE=CAP
MODE=VOLT
MODE=RES
RAIL?
RAILCFG=<AVDD_UV>,<negative_AVSS_UV>
CELL?=S1D1
ADS?
ADSCHK
ADSCHK=<1..1000>
BAT?
BATD
BATNOW
BATPERIOD?
BATPERIOD=<100..600000>
BATPERIOD=ON,<milliseconds>
BATPERIOD=OFF
RESSETTLE?
RESSETTLE=<0..10000>
```

接受和应用是两个事件：

```text
MACK,id=<requestId>,old=<mode>,new=<mode>,state=accepted
MAPP,id=<requestId>,gen=<generation>,old=<mode>,new=<mode>,seq=<frameSeq>,state=applied,transitionUs=<us>
```

`MODE?`/`STATE?` 只读 seqlock snapshot，包含 mode/state/route/SW/matrixRef/
INTREF/VBIAS/REFMUX/PGA/rail age、transition、heap 和 stack，不碰扫描硬件。
`CELL?` 返回最后一个完整 ADS 帧的 cell snapshot，包含 raw、nodeUv、vref、vss、
rref、PGA、fixed-point value、valid/fresh/error；它不触发一次新测量。

`ADS?` 同样只读 cache/snapshot；当身份未确认时必须返回 `chip=unknown,valid=0`。
`ADSCHK` 默认 100 个样本，先返回带 requestId 的 accepted ACK，Core 1 在完整帧后
执行并输出 `ADSCHK` 寄存器/能力行和 `ADSCHKSTAT` fresh/period/error/restore 行。
它读取实时 SPI/DRDY，不复用旧 measurement cache。`BATNOW` 和 `BATD` 也只排队；
前者走普通轻量 transaction，后者输出完整诊断。`BATPERIOD` 以毫秒配置真实时间
scheduler，不是 frame count。

### 数据帧

CAP 保持原有 `C` header、`D0..D3` 和 `K` 字节格式。VOLT/RES 分别使用 `V`/`R`
header，然后使用最多四个 16-cell `D` chunk、相同数量的 `P` chunk 和 `K`：

```text
V,seq=...,rows=...,cells=...,gen=...,rid=...,mode=VOLT,unit=V,scale=-6,...,fmt=uv-x
D0,<integer-uV-or-Xhh>,...
P0,<two-hex-digits-per-literal-gain>
K,seq=...,gen=...,rid=...,crc=<CRC32>

R,...,mode=RES,unit=ohm,scale=-3,...,fmt=mohm-x
```

cell 顺序固定为 row-major：S1D1..S1D8、S2D1..。`rows=1/2/4/8` 对应
8/16/32/64 cells。VOLT 的整数值单位为 `uV`（物理 V = value × 10^-6）；RES
为 `mΩ`（物理 Ω = value × 10^-3）。合法负电压仍是有符号整数；无效值使用
`Xhh` 十六进制错误码，因此不会与 0 或负数混淆。header 的 `valid`、`fresh`、
`error` mask 以及 `rail`/`age` 必须一起检查。

`P` payload 每 cell 两个十六进制字符，表示 literal gain（01/02/04/08/10/20）；
`00` 表示 gain-1 已触发 PGA alarm 后使用了经过 MODE2 readback 验证的 PGA bypass，
不是未知增益。V/R header 的 `ir` 是本帧被恢复的有界 I/O retry 次数；持续失败仍
输出 `Xhh`，不会被 retry 掩盖。`targetFps` 是当前模式 pacing 目标，`fps` 是实测
采集速率，两者不可混用。
CRC 是 IEEE CRC-32，范围从 header 第一个字节到最后一个 `P` 行换行，排除 `K`
行。CAP CRC 范围保持既有规则。8×8 极端 V/R golden fixture 为 1516 bytes；C
static worst-case 上限为 1518 bytes，均受 1536-byte fixed slot compile-time assert
和 formatter 运行时边界检查保护。

每个 V/R frame 的 `fresh` mask 必须覆盖全部 `rows*8` cell；单样本 fast accept
仍代表本帧真正的新 conversion，不代表复用旧值。`ADS50` 汇总 `frameUs`、
attempts/cell、raw conversion、profile/bypass/gain/register cache、single/triple/
precision/fresh/invalidation；`ADST50` 分解 row route、mux/register、DRDY、read、
aggregation、autorange、battery 和 ADSCHK 时间。`SF50` 分开报告 physical capture、
emitted frame、Serial、BLE 与 Wi-Fi FPS。

`ABAT` 至少携带 `valid/fresh/ageMs/periodMs/due/run/skip/defer/boundary/restoreFail`、
raw、`a8d/ac/a8g`、ratio、railState、VBIAS、sampleUs、restore 和 reason；`AB50`
是周期压缩计数。无效 battery 必须是 `bt=-1,valid=0,reason=<name>`，不能是 0。
AIN8 是独立通道：`a8g=a8d+ac`，`VBAT=a8g*num/den`（再应用有界校准）。

错误码由 `sensorarrayCellError_t` 定义，包括 route/SPI/DRDY timeout/stale/
reference/PGA alarm/saturation/common-mode/rail/reference/divide/open/short/
negative/range/overflow/unstable/autorange/unsupported/readback。host parser 必须保留
未知错误码，而不是转为 0。

## Australian English

CAP C/D/K bytes remain compatible. VOLT and RES add explicit V/R headers with
mode, unit, fixed-point scale, validity, freshness, error, reference, rail, and
autorange telemetry. `RAILCFG` supplies an externally measured AVDD/AVSS
snapshot at a frame boundary. Values are row-major; invalid cells are `Xhh`. PGA chunks
are covered by the same CRC as the header and data chunks. PGA `00` explicitly
means verified bypass, and header `ir` counts bounded recovered I/O retries.
The acquisition task
formats one fixed-slot packet, then Core 0 fans it out unchanged to the selected
Serial/BLE/Wi-Fi sinks.
