# USB-OTG 数据与控制 / USB-OTG data and control

ESP32-S3 内部 PHY 由启动时选择的后端独占。默认双 CDC 采用 DWC2 DMA 和全速 USB。CDC0 的 IN 传科学记录与控制回复，OUT 接受命令。CDC1 独立发送可丢的调试文本。可选厂商 bulk 配置将 DATA、CTRL 分到两个厂商接口，另有一个调试 CDC。开发描述符使用 303A:4042（双 CDC）或 303A:4041（厂商 bulk）。 / The boot-selected backend owns the ESP32-S3 internal PHY. The default dual CDC configuration uses DWC2 DMA on full-speed USB. CDC0 IN carries scientific records and control replies while OUT accepts commands. CDC1 carries best-effort debug output. The optional vendor configuration provides separate DATA and CTRL bulk interfaces and a debug CDC port. Development descriptors use 303A:4042 (dual CDC) or 303A:4041 (vendor bulk). [API](include/sensorarrayUsbOtg.h) · [端点缓冲 / Endpoint buffers](include/tusb_config.h) · [构建选项 / Kconfig](Kconfig) · [I/O owner](../../core/transport/sensorarraySerialIo.c)

## 任务边界与计数 / Ownership and counters

单一 SerialIo 任务初始化、轮询和写 TinyUSB。其它生产者经传输队列入队。作用域内可采用如下写法，`length` 是完整编码记录的字节长度。当返回零，保留未发送记录并由 owner 遵守已有期限。CDC0 的 DATA 进入 FIFO 前检查整条记录能否放下；CTRL、调试或厂商写入须处理部分接受。 / A single SerialIo task initialises, polls and writes TinyUSB. Other producers enqueue via transport. In owner context use the pattern below, with length in bytes for the complete encoded record. On zero accepted bytes retain an unstarted record within its existing deadline. CDC0 DATA checks capacity for the complete record before copying. CTRL, debug or vendor writes can accept a prefix.

```c
sensorarrayUsbOtgOwnerPoll();
int accepted = sensorarrayUsbOtgOwnerWrite(SENSORARRAY_USB_DATA, record, length);
```

设备批量端点的最大包长为 64 字节。512 字节 DMA 传输可包含多个全速包。FIFO 接受字节、厂商 TX 回调字节、CDC 完成回调次数和主机程序完整收到的记录应分别读取。CDC 回调没有返回字节数；`callbackBytesKnown` 标明来源。USB 复位或拔出改变 epoch，旧连接中未读完的命令会失效。DTR 变化可观测，下载通过显式 USBBOOT 及 ROM 能力判断进行。 / Bulk endpoints use 64-byte maximum packets. A 512-byte DMA operation can contain several full-speed packets. FIFO acceptance, vendor callback bytes, CDC callback count and complete records received in the host application are separate measurements. CDC callbacks carry no byte count. Inspect callbackBytesKnown. Bus reset or unplug changes the connection epoch and invalidates partial commands. DTR changes are observed. Download entry uses an explicit USBBOOT request and a ROM capability check.

## [sensorarrayUsbDescriptors.c](sensorarrayUsbDescriptors.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `void sensorarrayUsbOtgInitSerialString(void)` | 用 eFuse 默认 MAC 构造 12 位十六进制 USB 序列号字符串。 / Format the default eFuse MAC as a 12-digit hexadecimal USB serial string. | [sensorarrayUsbOtgOwnerInit](sensorarrayUsbOtg.c#L210) |
| `const uint8_t *tud_descriptor_device_cb(void)` | 返回所选构建的静态设备描述符地址。 / Return the static device descriptor for the selected build. | TinyUSB |
| `const uint8_t *tud_descriptor_configuration_cb(uint8_t index)` | index 为零时返回唯一配置描述符，其余返回 NULL。 / Return the sole configuration descriptor for index zero, otherwise NULL. | TinyUSB |
| `const uint8_t *tud_descriptor_bos_cb(void)` | 厂商后端返回含 MS OS 2.0 的 BOS，双 CDC 构建返回 NULL。 / Return a BOS with the MS OS 2.0 capability for the vendor build, or NULL for dual CDC. | TinyUSB |
| `bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t *request)` | 厂商后端按 SETUP 请求匹配并返回 WinUSB 描述符，其他 SETUP 返回 false。 / Serve the matching WinUSB descriptor setup request on rhport and reject unmatched setup requests. | TinyUSB |
| `const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t language)` | 按 index 生成临时 UTF-16 描述符，索引 0 返回语言表，无效索引返回 NULL。 / Build a temporary UTF-16 descriptor by index. Zero returns language IDs and invalid indexes return NULL. | TinyUSB |

## [sensorarrayUsbOtg.c](sensorarrayUsbOtg.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `bool sensorarrayUsbOtgEnabled(void)` | 在启动时依据构建和 RTC 恢复标志选择并缓存本次后端。 / Latch backend selection for this boot from build configuration and the retained recovery flag. | [sensorarraySerialTxTask](../../core/transport/sensorarraySerialIo.c#L601) |
| `bool sensorarrayUsbOtgCdcEnabled(void)` | 查询本次启动是否选择了正式的双 CDC 后端。 / Return whether the dual-CDC OTG backend is active for this boot. | [sensorarraySerialOtgTxTask](../../core/transport/sensorarraySerialIo.c#L482) |
| `uint32_t sensorarrayUsbOtgEpoch(void)` | 返回当前 USB 总线代号。 / Read the current bus connection epoch. | [sensorarrayUsbConsumeCommands](sensorarrayUsbOtg.c#L154) |
| `void tud_event_hook_cb(uint8_t rhport, uint32_t event, bool inIsr)` | 总线 reset 或 unplug 时推进 epoch，并从当前 ISR 或任务上下文唤醒 owner。 / Advance the epoch on reset/unplug and wake the owner from the supplied ISR/task context. | TinyUSB |
| `void tud_mount_cb(void)` | 标记已挂载、清暂停状态并计数。 / Mark the device mounted and resumed, and count the mount. | TinyUSB |
| `void tud_umount_cb(void)` | 记录卸载，清挂载和 DTR 缓存。 / Record unmount and clear mounted and DTR state. | TinyUSB |
| `void tud_suspend_cb(bool remoteWake)` | 记录暂停状态；此回调只观察状态。 / Mark the bus suspended when TinyUSB reports suspend. | TinyUSB |
| `void tud_resume_cb(void)` | 清除暂停标志。 / Clear the suspension flag on resume. | TinyUSB |
| `void tud_vendor_tx_cb(uint8_t itf, uint32_t bytes)` | 按厂商接口累加回调给出的字节数。 / Accumulate callback-reported bytes per vendor interface. | TinyUSB |
| `void tud_cdc_tx_complete_cb(uint8_t itf)` | 按 CDC 接口累加不含长度的完成回调次数。 / Count completion callbacks per CDC interface, which carry no length. | TinyUSB |
| `void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)` | 按接口记录 DTR，普通串口控制线更改只更新观测。 / Record DTR by CDC interface. Ordinary line-control changes update observations. | TinyUSB |
| `esp_err_t sensorarrayUsbOtgOwnerInit(void)` | 登记本任务为 owner，创建命令队列，启用 PHY/TinyUSB 并验证 DMA。 / Register the current owner, create the command queue, initialise PHY/TinyUSB and verify DMA enable. | [sensorarraySerialOtgTxTask](../../core/transport/sensorarraySerialIo.c#L482) |
| `void sensorarrayUsbOtgOwnerPoll(void)` | 非阻塞处理 TinyUSB 事件、代切换、总线状态和命令。 / Service TinyUSB events, epoch changes, bus status and command input without waiting for events. | [sensorarraySerialOtgTxTask](../../core/transport/sensorarraySerialIo.c#L482) |
| `bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane)` | 检查 lane 的连接代、ready 状态与 FIFO 空间，调试 CDC 还检查连接。 / Test the requested lane's epoch, readiness and available FIFO space, including debug CDC connection state. | [sensorarraySerialOtgTxTask](../../core/transport/sensorarraySerialIo.c#L482) |
| `int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t size)` | 对 lane 写入最多 size 字节，返回 FIFO 实际接受字节数。 / Write up to size bytes on lane and return the byte count accepted into its FIFO. | [sensorarraySerialOtgTxTask](../../core/transport/sensorarraySerialIo.c#L482) |
| `int sensorarrayUsbOtgRead(void *buffer, size_t size, uint32_t *epoch)` | 唯一命令消费者从分段缓冲取最多 size 字节，可取 epoch，零表示暂无数据。 / The single command consumer copies up to size bytes and optionally the epoch. Zero means no data available. | [sensorarraySerialIoReadWithEpoch](../../core/transport/sensorarraySerialIo.c#L722) |
| `void sensorarrayUsbOtgGetStats(sensorarrayUsbOtgStats_t *out)` | 复制已观测的接口状态、计数、时间和错误；out 由调用者提供。 / Copy observed interface status, counters, timestamps and errors into out. | [sensorarrayUsbControlHandle](../../main/control/sensorarrayUsbControl.c#L22) |
| `bool sensorarrayUsbOtgRomDownloadAvailable(void)` | 检查 eFuse 的 USB Serial/JTAG 与下载限制。 / Check the eFuse restrictions used to determine USB Serial/JTAG ROM download availability. | [sensorarrayUsbControlHandle](../../main/control/sensorarrayUsbControl.c#L22) |
| `esp_err_t sensorarrayUsbOtgPrepareRestart(int64_t deadlineUs, bool romDownload)` | 以绝对微秒期限等 owner 确认，按 romDownload 选择设置并核验 ROM 请求。 / Wait for the owner fence by the absolute µs deadline and optionally arm and verify the ROM request. | [sensorarrayUsbOtgRestart](sensorarrayUsbOtg.c#L442) |
| `void IRAM_ATTR __wrap_esp_restart_noos(void)` | 当硬件仍可能拥有 DMA 时采用数字系统复位，否则交给 IDF 无 OS 重启。 / Route a reset through digital-system reset when hardware may own DMA, otherwise call IDF's no-OS reset. | ESP-IDF 链接包装 / linker wrapper |
| `void sensorarrayUsbOtgRestart(bool romDownload)` | 执行有界 USB 收尾后走正常 IDF 重启或紧急复位，不返回。 / Fence USB before normal IDF restart or emergency reset. Does not return. | [sensorarrayRunFrontendPresenceBoundary](../../main/main.c#L2145) |
| `void sensorarrayUsbOtgManualRestart(bool romDownload)` | 对显式接受的手动 RESTART/USBBOOT 执行最终 USB 收尾，走无 OS 复位路径。 / For explicitly accepted manual RESTART/USBBOOT only, fence USB then enter the no-OS reset path. | [sensorarrayPerformManualRestart](../../main/main.c#L4527) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void sensorarrayUsbConsumeCommands(void)` | 有界读取控制端数据，按换行组装并以 epoch 入队，超长或队满计数。 / Bound control FIFO reads, build newline-delimited commands tagged with epoch and count oversize/full-queue drops. | [sensorarrayUsbOtgOwnerPoll](sensorarrayUsbOtg.c#L270) |
| `static void sensorarrayUsbOtgOwnerQuiesce(void)` | 最终重启前在 owner 中关中断、按阶段断开并停止 OTG，发布静止确认。 / Fence the final restart in owner context by disabling IRQ, staging disconnect/controller stop and publishing quiescence. | [sensorarrayUsbOtgOwnerPoll](sensorarrayUsbOtg.c#L270) |
| `static void IRAM_ATTR sensorarrayUsbEmergencyCoreReset(void)` | 保留恢复标志并触发 S3 数字系统复位，使 OTG 与 CPU 一同复位。 / Latch recovery and trigger an S3 digital-system reset that includes OTG and CPUs. | [__wrap_esp_restart_noos](sensorarrayUsbOtg.c#L436) |
