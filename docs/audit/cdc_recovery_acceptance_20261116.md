# 2026-11-16 软件恢复与双 CDC 验收 / Software recovery and dual-CDC acceptance

本记录接续 [11 月 14 日阶段](usb_otg_ble_priority_20261114.md)。用户已重启板卡，并授权继续烧录、调试；没有要求安装软件或使用管理员权限。历史失败记录保留，下面的下载成功不代表完整性能/精度验收通过。

This continues the [November 14 stage](usb_otg_ble_priority_20261114.md). The user rebooted the board and authorized continued flashing/debugging. No software installation or administrator account is required. Historical failures remain intact; successful recovery is not full performance or accuracy acceptance.

证据目录 / Evidence: [cdc_recovery_acceptance_20261116](artifacts/cdc_recovery_acceptance_20261116/). All command records retain argv, checkout HEAD, tracked-diff hash, exit status and logs. Hardware tools run as ordinary UID 1000; escaping the agent sandbox is not `sudo`.

## 已执行的恢复步骤 / Executed recovery steps

| 操作 / Operation | 实际结果 / Observed result |
|---|---|
| 重启后枚举 / Enumeration after user reboot | `303a:1001`, serial `80:B5:4E:CD:D9:AC`, tty accessible through existing `dialout` membership |
| `flash-01` app-only write | Source `4a183bca6a5a0566a834a727a58d5e25a80ad31b`; 1,590,656 bytes at `0x10000`; SHA256 `283e70782280789ed9c79f29e2263179deb23e9be57f6e35e8f5fb8595fda223`; esptool verified flash hash; no bootloader/partition/NVS/calibration write |
| esptool `after=hard_reset` | 未启动应用 / App not observed; subsequent no-reset sync reported the same MAC and an already-running stub |
| 明确 DTR/RTS 退出序列 / Explicit DTR/RTS exit | `usj-exit-01` completed without ioctl error, but no application enumeration observed; not a successful recovery |
| 软件 WDT 复位 / Software watchdog reset | `usj-watchdog-exit-01`: same MAC verified, FORCE_DOWNLOAD bit cleared with mask 1 and read back as 0, one installed esptool S3 `watchdog_reset()`; new `303a:4042`, devnum 15, formal IF0 and debug IF2 appeared |
| 应用身份 / Application identity | CDC and BLE returned `version=4a183bc`; ELF hash field contained only `9a16d7673`, a 9-character prefix; full running-hash attestation failed and was not waived |
| BLE 状态查询 / BLE state queries | `boot-05-readonly`: PROTO2, dual-CDC capability, all three frontends present, SPI qualification completed, selected requested/observed clock 750000/754000 Hz, BLE 2M PHY and MTU247 |
| BLE → USBBOOT | Exactly one action in `ble-rom-01`; kernel monitor recorded old `303a:4042` device 15 removed at monotonic 309074.718360 and new `303a:1001` device 16 added at 309075.628112 |
| ROM 同步 / Downloader sync | `ble-rom-01-identify`: no-reset esptool synchronized, confirmed ESP32-S3 rev0.2, MAC `80:b5:4e:cd:d9:ac`, 16 MB flash and embedded 8 MB PSRAM; no flash write |

`ble-rom-01-udev` 的退出码 124 是预设的 40 秒只读监视窗口结束，不是烧录超时。原始 remove/add 事件是重新枚举证据；RACK/RAPP 本身仅是终态发布证据。串口 RTS 失败的电气/主机根因尚未确定：默认 DTR 状态只能作为候选解释，明确清 DTR 的实测也未恢复。

The udev monitor's exit 124 means its planned 40-second observation window ended, not a flash timeout. Actual remove/add events establish re-enumeration; RACK/RAPP alone do not. The electrical/host cause of the RTS failure is unresolved: default DTR state is only a hypothesis, since the explicit clear-DTR attempt also failed.

esptool 的普通 ROM 连接可能按 SDK 流程暂时关闭下载器 watchdog；这是下载环境行为，不是删除应用安全 watchdog。退出助手不写 Flash/eFuse，真正的应用是否启动必须由后续枚举和 BUILD 核验判断。不得只凭成功返回就自动反复重启。

Normal esptool ROM connection may disable downloader watchdogs as part of its SDK procedure. This does not remove application watchdog protection. The exit helper writes no flash/eFuse; subsequent enumeration and BUILD queries establish whether the application booted. A returned reset call does not authorize an unbounded retry loop.

## 本轮复现与修改 / Reproductions and changes

### 恢复标记 / Recovery latch

旧 `sensorarrayUsbOtgEnabled()` 选择一次 USJ 后即清 RTC marker；若恢复启动再次 early panic，下个启动会重选 OTG。已用基线真实 C 函数复现失败。修复后普通 RESTART 和再次 panic 保留恢复锁存，只有显式 USBBOOT 完成原有有界收尾、设置并读回 ROM 启动标志后才解除。本 boot 的后端选择不热切换。

Previously, `sensorarrayUsbOtgEnabled()` consumed the RTC marker when selecting USJ. Another early panic could return the following boot to OTG. The baseline production C fails the added regression. The repair retains the latch through ordinary RESTART and repeated panic; only explicit USBBOOT releases it after bounded cleanup and verified ROM-flag arming. The current boot's backend remains fixed.

66 个双后端 owner 场景、SerialIo 和控制回复回归已执行通过。真实早崩溃故障注入尚未执行；RTC 保留不保证跨断电，也不保证任意硬件故障可软件恢复。

All 66 owner scenarios across both backends, SerialIo and control-reply regressions passed. Physical early-crash injection remains untested. RTC retention across power removal and recovery from arbitrary hardware faults are not guaranteed.

### 完整 BUILD 哈希 / Full BUILD hash

`boot-04` 原始帧确认只有 9 位 hash。实际 IDF 5.5.5 的 `esp_app_get_elf_sha256()` 从 `CONFIG_APP_RETRIEVE_LEN_ELF_SHA=9` 的缓存复制，传入 65 字节缓冲仍会截断。新增 `sensorarrayFormatBuildReply()` 直接读取应用描述符中的 32 字节 ELF SHA，早期和正常查询统一输出 64 位十六进制；不修改全局日志长度配置。真实 C 格式化、缓冲边界及两处 dispatch 已回归通过。

The raw `boot-04` reply contains only nine hash characters. In the installed IDF 5.5.5, `esp_app_get_elf_sha256()` copies a cache limited by `CONFIG_APP_RETRIEVE_LEN_ELF_SHA=9`, even with a 65-byte destination. The new shared formatter reads all 32 descriptor bytes for both early and normal BUILD queries. Actual C formatter, buffer-boundary and dispatch tests passed; the global logging setting is unchanged.

### 首次查询的旧错误 / Stale error at the first query

`boot-03` 将 UNKNOWN 归给 BUILD，但独立 debug 原始记录中的入站内容是 `READY,bootId=1,b`，并不是 `BUILD?`。随后只读查询实际收到了 BUILD。PTY 对照复现了默认 ECHO、切 raw、前导 LF 终结旧半行的机制；它不能替代现场 USB OUT 抓包证明。

`boot-03` attributed UNKNOWN to BUILD, but the independent debug log identifies the received command as `READY,bootId=1,b`, not `BUILD?`. A subsequent read-only query received BUILD. A PTY control reproduces the default-ECHO/raw-transition/leading-delimiter mechanism; it is not a capture of the original USB OUT traffic.

## 新镜像恢复闭环 / Repaired-image recovery loop

源码 `3a55543cb21fa3412ede26ee79c1cce7d8371804` 的目标构建和 58 个 host 测试入口均实际通过，179 个生产输入 hash 一致。app 1,591,088 B，binary SHA256 `59d3f07d772730692ea4a68e9af6437779ddfd24849c78f8fb9a03fbb2e31fee`，完整 ELF SHA256 `d4ac82a199bd2bc8bb730b331225ffdbb94acd2c84c410cd675636da706e92e5` 已由设备 BUILD 核对。正式 CDC USBBOOT 一次动作后 1.002248 秒观察新下载接口，随后核 MAC、app-only 重刷和再次软件启动通过；[闭环证据](artifacts/cdc_recovery_acceptance_20261116/cdc-rom-01-verified-loop.json)。未再按 BOOT、未用 sudo、未修改 NVS/校准。

The target build and all 58 host test entries passed with matching hashes for 179 production inputs. The running BUILD matched the complete ELF hash above. One formal-CDC USBBOOT action produced a new downloader interface after 1.002248 seconds, followed by same-MAC sync, app-only reflash and verified software startup. No further BOOT-button operation, sudo, NVS or calibration write was used.

普通 RESTART 的第一次观察脚本在新 tty 权限尚未可用时收到 EACCES；原 ERROR 保留，后续独立只读查询确认同一 BUILD 且 session 从 `50D0CC36C32764D1` 变为 `2D7D5CE074215BB2`，没有重发动作。工具现对新 epoch 的 ENOENT/EACCES 在原60秒期限内重新核身份并等待，24个合成场景通过；修后该工具的普通重启实机重测仍待执行。

The first ordinary-RESTART observer encountered EACCES opening the new tty. Its ERROR remains unchanged; a separate read-only query confirmed the same BUILD and a changed session, without another action. The observer now retries only new-epoch ENOENT/EACCES within the original 60-second deadline, revalidating identity each time. All 24 synthetic cases pass; a new hardware run of that revised observer remains pending.

## 120 秒负载筛查失败 / 120-second load screen failed

`screen-01` 完整120秒，Capture64.563554FPS、USB62.532747FPS/239序号缺口、BLE43.370896FPS/2537序号缺口。ADC CRC 4378/102828=4.257595%，超过用户2.5%要求；本boot选择550kHz、驱动观测551kHz。ASCII CRC及身份错误均0，记录窗口未观察MFAULT/reset。完整指标、分母和边界见[原始分析](artifacts/cdc_recovery_acceptance_20261116/screen-01/screen-01-analysis-01.json)、[失败判定](artifacts/cdc_recovery_acceptance_20261116/screen-01/screen-01-verdict.json)及README双语表格。

The full 120-second screen failed both the ADC CRC requirement and transport throughput expectations: capture64.563554FPS, USB62.532747FPS/239missing observations, BLE43.370896FPS/2537missing observations, and 4.257595% ADC rejection versus the 2.5% limit. This boot selected550kHz, observed551kHz. No ASCII CRC or identity errors, MFAULT or reset were observed in the recorded window. This is not a precision pass.

## 待完成 / Pending

先定位修复本次 CRC/输出背压，再重做筛查；600秒稳定性和32配置×120秒USB+BLE仍须实际执行。未运行项目不得由旧版本结果补齐。夹具、临时DMM rail校准和非精度验收边界沿用README；不执行CAL=SAVE。

Diagnose and repair the observed CRC/backpressure failures before repeating the screen. The600-second stability and32 configurations×120-second USB+BLE runs remain pending; older-image results cannot fill them. Fixture and temporary DMM rail limits apply, without CAL=SAVE.
