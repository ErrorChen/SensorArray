# 科学帧与诊断输出 / Science frames and diagnostics

## 数据布局 / Frame layout

固件发送 LF 结尾的 ASCII。行优先，位 `8 × (S−1) + (D−1)` 对应物理端子 S1…S8、D1…D8。活动行 1–8，每行八个测点，普通帧 `cells=n=rows×8`，混合帧用 `cells`。完整 DATA、LOG 上限各 2304 B，CTRL 回复上限 512 B。 / Firmware emits LF-terminated ASCII. Preserve the original bytes for CRC. Unverified, incomplete or oversized records require explicit handling. A valid numerical zero is a measurement.

| 记录 / Record | 排列 / Structure | 数值与身份 / Value and identity |
| --- | --- | --- |
| `C` | `C` → `D0`… → `K` | `fmt=pf6`，整数 × 10⁻⁶ pF，固定无效哨兵 `-1000000`。`0` 可有效。`gen/rid` 属于 ROWS。 / Fixed-point total capacitance. Retain sentinel and freshness context. Legacy identity belongs to row count. |
| `V` | `V` → `D0`… → `P0`… → `K` | `mode=VOLT,unit=V,scale=-6,fmt=uv-x`，有效整数 µV 或 `Xhh`。 / Signed microvolts or a cell error. Legacy `gen/rid` belongs to MODE. |
| `R` | `R` → `D0`… → `P0`… → `K` | `mode=RES,unit=ohm,scale=-3,fmt=mohm-x`，有效整数 mΩ 或 `Xhh`。 / Signed milliohms or a cell error. Legacy `gen/rid` belongs to MODE. |
| `M` | `M` → 每个活动物理行一个 `MR` → `K` | `fmt=mix1`，`rgen/rrid` 是 ROWS，`pgen/prid` 是 ROWMODES。 / Each MR inherits the enclosing identity. Decode row by its own mode and scale. |
| `B20` | 一行 CAP 预览 / Single CAP preview | `previewOnly=1`，端点 `d0/dl` 无效为 `na`，不含 D/P/K 或完整原始测点。 / A summary with captured identity. Exclude from full-frame receipt and periodic snapshot counts. |

普通帧的 D 块以 `Dn` 开头，最多 16 个逗号分隔的十进制数值或错误码。V/R 在所有 D 块之后按相同顺序输出 P 块，每点恰好两个连续十六进制字符，无额外分隔。`01 02 04 08 10 20` 是倍率 1、2、4、8、16、32。`00` 用于旁路。未采到的点也可保留初始化 `00`，主机须先检查 acquired/valid，单独的 `00` 无法证明物理旁路。 / Match each gain byte to the corresponding cell after checking acquisition. Full CAP uses D chunks only. [编码器 / Encoder](../main/output/sensorarrayTextProtocol.c) · [编码器实际生成的帧 / Encoder-generated frames](../tests/fixtures/config_identity/README.md)

## Header 与采集掩码 / Header and acquisition masks

| 字段 / Field | 含义与用法 / Meaning and use |
| --- | --- |
| `seq,ts,rows,cells` | 同一 boot 的帧序号、微秒时间戳、活动行数和测点数。 / Compare sequence and monotonic time within one verified boot. |
| `session,bootId,rev` | 新身份可用时加入 header 和 K。`session` 为 16 位十六进制字符串，`bootId` 是兼容启动计数，`rev` 为配置 revision。 / Decode by key even when fields follow K.crc. Preserve the full 64-bit session. |
| `mgen/mrid,rgen/rrid,pgen/prid` | mode、rows、profile 的已应用 generation 与请求 ID。 / Capture-time identity in C/V/R/M headers. M already carries the row and profile pairs. |
| `expected,acquired` | 位图表示本帧计划点与实际完成读取的点，字宽 64 位，限制在活动点内。`acquired` 是 `expected` 子集。 / Planned and acquired cells. A completed conversion can report OPEN/SHORT and still be acquired. An acquisition never started has its bit clear. |
| `C.rf,pf,sf,bad` | 行 fresh、主 FDC fresh、次 FDC fresh 掩码。`bad=<stale>/<mixedEpoch>/<invalidCount>`，前两项为 0 或 1。 / Row freshness, primary and secondary freshness, frame flags and invalid cell count. |
| `V/R.valid,fresh,error,bad` | 可用、新转换、显式错误位图及无效点数。 / Respect each mask independently. An unacquired cell cannot claim freshness. |
| `ref,rail,age,avdd,avss,vexc,rref` | ADS 参考源、rail 有效位、帧龄、AVDD/AVSS 与激励 µV、参考电阻 Ω。来源与估算看 `RAIL?`。 / Read rail provenance before interpreting a split or excitation as measured. |
| `dur,tr,gc,ov,aa,fb,ir,to,st,spi` | 帧/切换 µs、PGA 改档、超量程、自动量程尝试及回退、已恢复 I/O 重试、DRDY 超时、旧值、SPI 错误计数。 / Per-frame durations and conversion diagnostics. |
| `B20` | `seq,rows,cfps=na,efps=na,rf,pf,sf,bad,d0,dl,previewOnly,gen,rid,session,bootId,rev,mgen,mrid,rgen,rrid,pgen,prid`。 / A compact view of a queued CAP record carries its original row/configuration identity. |

主机用 `CONFIG?` 的同 session、revision、配置域和应用边界恢复身份。完整帧可能先于终态到达。FRAME CRC 良好只证明完整性，每点仍须检查科学有效位。 / Keep the original receipt while resolving a missing or delayed terminal. An ACK accepts a request, an applied event establishes the new boundary, and complete DATA provides measurement evidence. [配置域和回绕 / Domains and wrap](integration/firmware_host_contract.md)

## 混合行 / Mixed rows

同质性按八个已配置物理行判断。`ROWMODES=CVRCVRCV` 配合 `ROWS=1` 输出 `M`、一个 CAP `MR` 以及 `K`。CONFIG 保存全部八项，线上 M 的 `profile=CNNNNNNN` 标出实际活动行。 / Homogeneity uses all eight configured entries. Inactive trailing rows appear as N on the wire. Place each MR at its one-based physical S index. Each active row has exactly one MR with eight cells.

```text
M,seq=<seq>,ts=<us>,rows=<rows>,cells=<cells>,rgen=<rgen>,rrid=<rrid>,pgen=<pgen>,prid=<prid>,profile=<eight characters>,expected=<mask64>,acquired=<mask64>,fmt=mix1,<identity fields>
MR,s=<row>,m=CAP,unit=pF,scale=-6,expected=<mask8>,acquired=<mask8>,valid=<mask8>,fresh=<mask8>,error=<mask8>,fmt=pf6,D=<eight values>
K,seq=<seq>,rgen=<rgen>,rrid=<rrid>,pgen=<pgen>,prid=<prid>,crc=<hex>,<session fields>
```

上方为语法示意。VOLT 行为 `unit=V,scale=-6,fmt=uv-x`，RES 行为 `unit=ohm,scale=-3,fmt=mohm-x`。无效 CAP 行点输出 `X14`。拼回的各 MR expected/acquired 应与 M 位图逐位相同。 / Angle brackets denote placeholders. Reject missing or duplicate rows, wrong mode, incorrect cell counts, inconsistent header/trailer identity or inconsistent masks. [真实混合行帧 / Mixed-row golden frame](../tests/fixtures/config_identity/mixed_1.frame)

## 校验与错误码 / CRC and cell errors

IEEE CRC-32 使用反射多项式 `0xEDB88320`，初值及末异或均为 `0xFFFFFFFF`。从 C/V/R/M 首字节起，计算到 K 前的最后一个 LF。覆盖原始 header 的扩展字段、所有 D/P 或 MR 和相应换行。K 本身不参与 CRC，须另查 K.seq 和对应身份。 / CRC32 of `123456789` is `0xCBF43926`. Verify original bytes before translating line endings or reconstructing text. G-fragment message CRC32 and ADS conversion CRC8 protect different byte streams. [BLE 分片 / BLE envelope](ble-protocol.md)

`Xhh` 使用两位十六进制。无效测点保留原始码，未知码也保留并显示为无效。 / Record unknown codes verbatim. A numerical value and an invalid token have distinct scientific meaning.

| 代码 / Code | 定义 / Meaning | 代码 / Code | 定义 / Meaning |
| --- | --- | --- | --- |
| `00` | 未指定测点错误 / No cell error | `0B` | 参考无效 / Invalid reference |
| `01` | 路由 / Route | `0C` | 除数近零 / Near-zero divisor |
| `02` | SPI（可含 CRC 拒绝）/ SPI, including CRC rejection | `0D` | 开路 / Open |
| `03` | DRDY 超时 / DRDY timeout | `0E` | 短路 / Short |
| `04` | 旧转换 / Stale conversion | `0F` | 负值 / Negative |
| `05` | 参考告警 / Reference alarm | `10` | 量程 / Range |
| `06` | PGA 绝对输入 / PGA absolute input | `11` | 数值溢出 / Overflow |
| `07` | PGA 差分输入 / PGA differential input | `12` | 不稳定 / Unstable |
| `08` | 饱和 / Saturated | `13` | 自动量程 / Autorange |
| `09` | 共模 / Common mode | `14` | 不支持 / Unsupported |
| `0A` | 供轨无效 / Invalid rails | `15` | 回读失败 / Readback |

## 控制及故障 / Control and fault records

| Tag | 含义 / Meaning |
| --- | --- |
| `RCMD/RAPP`, `MACK/MAPP/MERR`, `RMACK/RMAPP/RMERR` | ROWS、MODE、ROWMODES 受理及应用或拒绝。用请求域、ID、session、边界关联。 / Source-local ACK and lifecycle terminal can arrive separately and out of order. |
| `RACK/RAPP`, `ACK/ERR`, `BAPP`, `E,type=cmd` | RAILCFG、恢复或重启命令有各自身份；BAPP 汇报电池请求，E 是兼容 mailbox 事件。 / Disambiguate reused tags by command, fields and domain. |
| `MFAULT`, `ADSFAULT`, `ADSFAULTRAIL`, `ADSFAULTCTX`, `ADSFAULTREG`, `ADSFAULTTXN` | 查看收尾前的路由、首错和 cleanup、寄存器掩码、计数、实际尝试和获取。 / Read source, units and transaction identity. A route error before conversion can leave every acquired bit clear. |
| `TXDROP/CTRLDROP/BLECORRUPT` | 本地池/队列拒绝、回复超限、BLE 槽长度/CRC 不符。 / Keep the responsible layer, cause and counts. Oversized records are rejected whole. |
| `CMDERR/CMDERR_SUM/BLERXERR/LOGTRUNC` | 串口输入错误与限流汇总、BLE 输入拒绝、日志超限。 / Distinguish control RX and log encoding failures from science DATA. |
| `RST/APP_FATAL`、panic | 启动/复位信息、初始化或运行致命错误。 / Use BOOT, READY and retained panic queries to identify the session and evidence. |

## 电池与周期统计 / Battery and runtime summaries

`ABAT` 是最近的电池观测，`AB50` 为聚合。`valid,fresh,ageMs,periodMs,reason,restore,validRun,invalidRun,retry,unstable,timeout,spreadRaw` 说明本次有效性、新鲜度、采样跨度及恢复结果。`bt=-1,valid=0` 是无效状态。`lastGoodMv,lastGoodValid,lastGoodFresh,lastGoodAgeMs,lastGoodFrame,lastGoodKnown` 单独描述最后成功值，未知数字哨兵为 `4294967295`。 / Retain the actual last-good measurement time. A battery validity failure with verified restoration may report `BATERR,action=report_continue`. Unknown or failed ADS restoration requires isolation. Use the reported action and cleanup error. [电池事务与误差 / Battery implementation](../core/measure/ads/README.md)

| Tag | 代表字段 / Fields | 解释 / Interpretation |
| --- | --- | --- |
| `SF50/OT50` | `cfps,efps,ofps,drop,q,st,tx,out` | 采集、发出和本地 sink 的不同速率。主机按完整实收单独计数。 / Separate acquisition, publication and sink rates from host receipt. |
| `TR50/I2C50` | `r,fu,rau,rmu,rt,wt,rp,rs,co,ag`，I²C 地址和错误 / bus address and errors | FDC 行采集、ADS gap、I²C 时序、恢复和超时。 / Row and bus timing, recovery and timeouts. |
| `BL50` | `conn,sub,mtu,phy,mode,mq,ms,md,sentD,sentL,dropD,dropL,dropC,ctrlRetry,ctrlExhaust,fs,fe,cg,stale,conf,tiny` | BLE 链路、分片和本地计数；同 boot/stream 取差值。 / Inspect changes over a bounded window. |
| `ADS50/ADST50/AB50` | `frameUs,attemptsPerCell,rawConversions,profileHit,validRun,invalidRun,restoreFail` 等 / and timing groups | ADS 缓存、PGA、等待、采集和电池动作。 / Conversion attempts, time, cache and auxiliary work. |
| `STK50` | `log,transport,usb,bleTx,bleCtrl,serialCtrl,heap,tSlot,ta,ts,tr,tq,bSlot,ba,bs,br,bc,bf,trunc` | 栈为配置/最小剩余 B，heap 为初始/当前/最小可用 B。tSlot/bSlot 为使用/峰值，ta/ba 分配失败，ts/bs 过期，tr/br 释放不匹配，tq 队列拒绝，bc/bf 为 BLE CRC/分片错。 / Stack, heap, slot occupancy/high-water and diagnostic counters. |

聚合周期按配置解释。一般摘要和 ADS 摘要可拆成独立 LOG message，超过容量使用 `LOGTRUNC`。 / Observe each summary independently. Use a per-session counter delta with its layer and denominator. [命令和查询 / Queries](command-reference.md)
