# 板级 I²C 与 GPIO 资源 / Board I²C and GPIO resources

该 ESP-IDF 组件为主总线和可选第二条总线保存驱动安装状态、互斥量、时钟及诊断。FDC 驱动把这里的 read/write 函数当作回调；GPIO ISR 服务由共享入口安装。主、次总线各自串行，设备地址使用 7 位格式。

This ESP-IDF component owns the installation state, mutex, clock and diagnostics of the primary and optional secondary I²C buses. The FDC driver uses its read/write callbacks. GPIO clients share one ISR service. Each bus serialises its own transactions and takes seven-bit device addresses.

## 调用方式 / Calling the module

```c
BoardSupportI2cBusInfo_t info;
bool known = boardSupportGetI2cBusInfo(false, &info);
```

查询示例选主总线。与硬件交互须在任务上下文进行，先调用 boardSupportInit。只有超时且观察到总线被拉低时才考虑恢复，间隔和失败预算受 Kconfig 约束。NACK、总线离线与驱动安装失败应分别处理。频率设置和恢复与正常读写共享所有权。

The example selects the primary bus. Initialise with boardSupportInit and perform bus I/O in task context. Recovery requires a timeout and a line observed stuck low, subject to configured cooldown and failure budgets. NACK, an offline bus and driver installation failure have distinct meanings. Clock changes and recovery share ownership with transfers.

## 总线配置 / Bus configuration

仓库配置的主 I²C SDA/SCL 为 GPIO9/10，次 I²C 为 GPIO11/12，两组初始频率均为 350,000 Hz。启动选速可改变实际频率。传输回调执行时需可在 FreeRTOS 任务内等待总线锁。ISR 安装与 ISR 回调是不同入口。总线已离线、正在恢复或资源不可用时，使用调用返回的错误和总线快照判断原因。

The checked-in settings use GPIO9/10 for primary SDA/SCL and GPIO11/12 for secondary SDA/SCL. Both are initially configured for 350,000 Hz. Startup qualification can select a different running rate. A transfer callback must be able to wait for the bus lock in task context. ISR installation and ISR execution use separate entry points. Consult transfer status and the bus snapshot for offline, recovering or unavailable resources.

[Kconfig](Kconfig) · [工程选项 / Project settings](../../sdkconfig.defaults.esp32s3)

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [boardSupport.c](boardSupport.c) · [include/boardSupport.h](include/boardSupport.h) | 管理两条 I²C 总线的驱动、每总线锁、GPIO ISR 安装、时钟切换和受限的低电平恢复。 / Owns both I²C drivers and per-bus locks, GPIO ISR service installation, clock changes and bounded stuck-low recovery. `BoardSupportI2cBusState_t` `BoardSupportI2cCtx_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

### boardSupport.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void boardSupportSetLogCallback(BoardSupportLogCallback_t callback)` [22](boardSupport.c#L22) | 登记板级日志接收函数，供总线和 GPIO 诊断使用。 / Register the board logger used by I²C and GPIO diagnostics. | `sensorarrayScanTask` |
| `static int boardSupportLogPrintf(const char *format, ...)` [27](boardSupport.c#L27) | 按可用的板级日志输出回调发送格式化文本。 / Format board diagnostics and forward them to the configured logger. | 无直接调用记录 / No direct call found |
| `esp_err_t sensorarrayBoardEnsureGpioIsrService(void)` [132](boardSupport.c#L132) | 以共享所有权安装或确认 GPIO ISR 服务。 / Install or detect the shared GPIO interrupt service. `ESP_ERR_NO_MEM` `ESP_ERR_TIMEOUT` | `ads126xAdcEnableDrdyNotification` / `sensorarrayFrontendStartupRun` |
| `void sensorarrayBoardNoteGpioIsrHandlerAdd(BoardSupportGpioIsrHandler_t handler)` [176](boardSupport.c#L176) | 登记 ISR handler 的安装结果，区分已有服务与添加回调。 / Record GPIO handler-add outcome separately from service installation. | `ads126xAdcEnableDrdyNotification` / `sensorarrayMeasureEnsureFdcIntb` |
| `void sensorarrayBoardGetGpioIsrInfo(BoardSupportGpioIsrInfo_t *outInfo)` [197](boardSupport.c#L197) | 复制 ISR 服务及 handler 安装计数的现状。 / Copy GPIO ISR service state and handler installation counters. | `sensorarrayBoardLogGpioIsrSummary` |
| `void sensorarrayBoardLogGpioIsrSummary(void)` [207](boardSupport.c#L207) | 输出 板级GPIOISR摘要。 / Report board GPIO ISR summary. | `sensorarrayInitSystem` |
| `static bool boardSupportI2cPortValid(int port)` [222](boardSupport.c#L222) | 判断I²C端口有效。 / Test I²C port valid. `port == I2C_NUM_0 \|\| port == I2C_NUM_1` | `boardSupportI2c1ConfiguredByPins` / `boardSupportI2cValidateCtx` |
| `static bool boardSupportI2cIsGpioValid(gpio_num_t gpio)` [227](boardSupport.c#L227) | 判断I²C是否GPIO有效。 / Test I²C is GPIO valid. `gpio >= 0 && gpio < GPIO_NUM_MAX` | `boardSupportI2cPinsValid` |
| `static bool boardSupportI2cPinsValid(int sdaGpio, int sclGpio)` [232](boardSupport.c#L232) | 核对所选 SDA/SCL 是可用且不同的 GPIO。 / Check that the selected SDA/SCL pins are usable and distinct. | `boardSupportI2c1ConfiguredByPins` / `boardSupportI2cReadPins` |
| `static bool boardSupportI2c1EnableRequested(void)` [239](boardSupport.c#L239) | 返回构建是否请求启动第二条 I²C 总线。 / Return whether this build requests the secondary I²C bus. | `boardSupportI2c1ConfiguredByPins` / `boardSupportInit` |
| `static bool boardSupportI2c1ConfiguredByPins(void)` [244](boardSupport.c#L244) | 检查次总线的引脚和端口配置，决定是否可初始化。 / Check secondary-bus pin and port configuration before initialisation. | `boardSupportInit` / `boardSupportIsI2c1Enabled` |
| `static BoardSupportI2cCtx_t *boardSupportI2cWritableCtx(const BoardSupportI2cCtx_t *ctx)` [251](boardSupport.c#L251) | 将驱动回调提供的板级上下文定位到内部可更新的总线状态。 / Resolve the driver-supplied board context to its mutable internal bus state. | `boardSupportRecoverI2cBus` / `boardSupportSetI2cFrequency` |
| `static const char *boardSupportI2cBusStateName(BoardSupportI2cBusState_t state)` [271](boardSupport.c#L271) | 返回I²C总线状态的显示名称。 / Return the display name for I²C bus state. | `boardSupportI2cHandleTransactionError` / `boardSupportI2cFinishTransaction` |
| `static TickType_t boardSupportI2cMsToTicksAtLeastOne(uint32_t timeoutMs)` [290](boardSupport.c#L290) | 将毫秒超时换算为至少一个 FreeRTOS tick。 / Convert a millisecond timeout to at least one FreeRTOS tick. | `boardSupportInitI2c` / `boardSupportRecoverI2cBus` |
| `static esp_err_t boardSupportI2cEnsureMutex(BoardSupportI2cCtx_t *ctx)` [296](boardSupport.c#L296) | 首次使用时为该总线建立互斥量。 / Create the bus mutex on first use. `ESP_ERR_INVALID_ARG` | `boardSupportInitI2c` / `boardSupportRecoverI2cBus` |
| `static esp_err_t boardSupportI2cLock(BoardSupportI2cCtx_t *ctx, TickType_t timeoutTicks)` [314](boardSupport.c#L314) | 加锁 I²C。 / Lock I²C. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `boardSupportInitI2c` / `boardSupportRecoverI2cBus` |
| `static void boardSupportI2cUnlock(BoardSupportI2cCtx_t *ctx)` [337](boardSupport.c#L337) | 解锁 I²C。 / Unlock I²C. | `boardSupportInitI2c` / `boardSupportI2cFinishTransaction` |
| `static esp_err_t boardSupportI2cReadPins(const BoardSupportI2cCtx_t *ctx, int *outSdaLevel, int *outSclLevel)` [344](boardSupport.c#L344) | 采样 SDA/SCL 并写入调用者提供的电平字段。 / Sample SDA/SCL into the caller's output fields. `ESP_ERR_INVALID_ARG` | `boardSupportI2cClassifyBusState` |
| `static BoardSupportI2cBusState_t boardSupportI2cClassifyBusState(const BoardSupportI2cCtx_t *ctx, int *outSdaLevel, int *outSclLevel)` [369](boardSupport.c#L369) | 按读到的 SDA/SCL 电平分类空闲或卡低总线。 / Classify the observed SDA/SCL levels as idle or stuck. | `boardSupportI2cFinishTransaction` / `boardSupportRecoverI2cBus` |
| `static bool boardSupportI2cBusStateStuck(BoardSupportI2cBusState_t state)` [397](boardSupport.c#L397) | 根据 SDA/SCL 的组合状态判断总线是否被拉低。 / Test the sampled SDA/SCL state for a stuck bus. | `boardSupportI2cShouldRecover` / `boardSupportI2cFinishTransaction` |
| `static esp_err_t boardSupportI2cValidateCtx(const BoardSupportI2cCtx_t *ctx)` [404](boardSupport.c#L404) | 检查传入上下文、总线引脚及状态是否允许后续操作。 / Validate bus context, pins and state before the requested operation. `ESP_ERR_INVALID_ARG` | `boardSupportInitI2cLocked` |
| `static void boardSupportI2cLogMem(const char *stage, const BoardSupportI2cCtx_t *ctx, esp_err_t err)` [421](boardSupport.c#L421) | 输出 I²C内存。 / Report I²C mem. | `boardSupportInitI2cLocked` |
| `static esp_err_t boardSupportInitI2cLocked(BoardSupportI2cCtx_t *ctx)` [433](boardSupport.c#L433) | 持有对应总线锁安装或重配 IDF I²C 驱动。 / Install or reconfigure the IDF I²C driver with the bus lock held. | `boardSupportInitI2c` / `boardSupportRecoverI2cBus` |
| `static esp_err_t boardSupportInitI2c(BoardSupportI2cCtx_t *ctx)` [500](boardSupport.c#L500) | 初始化 I²C。 / Initialise I²C. 调用 / Uses `boardSupportI2cEnsureMutex`, `boardSupportI2cLock`. | `boardSupportInit` |
| `static void boardSupportI2cMarkOffline(BoardSupportI2cCtx_t *ctx, const char *reason)` [516](boardSupport.c#L516) | 登记失败原因并使该总线离线，阻止正常传输继续。 / Record the cause and mark the bus offline before further regular transfers. | `boardSupportRecoverI2cBus` |
| `static bool boardSupportI2cShouldRecover(BoardSupportI2cCtx_t *ctx, esp_err_t err, BoardSupportI2cBusState_t busState, const char **outAction)` [530](boardSupport.c#L530) | 根据传输错误与总线电平决定是否可启动受限恢复。 / Decide whether transfer failure and line state admit guarded recovery. | `boardSupportI2cHandleTransactionError` |
| `static const char *boardSupportI2cErrKind(esp_err_t err)` [587](boardSupport.c#L587) | 返回I²C错误码类型。 / Return I²C err kind. | `boardSupportI2cHandleTransactionError` |
| `static void boardSupportI2cHandleTransactionError(BoardSupportI2cCtx_t *ctx, uint8_t addr7, const char *op, esp_err_t err, BoardSupportI2cBusState_t busState)` [601](boardSupport.c#L601) | 按 NACK、超时等分类错误，采样总线并决定是否执行受限恢复。 / Classify a failed transaction, observe the bus and determine guarded recovery eligibility. | `boardSupportI2cFinishTransaction` |
| `static esp_err_t boardSupportI2cCheckReadyLocked(BoardSupportI2cCtx_t *ctx, uint8_t addr7, const char *op)` [626](boardSupport.c#L626) | 持有对应总线锁时核对驱动已安装、在线并可传输。 / Check driver installation, online state and admission while holding the bus lock. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `boardSupportI2cWriteRead` / `boardSupportI2cWrite` |
| `static esp_err_t boardSupportI2cFinishTransaction(BoardSupportI2cCtx_t *ctx, uint8_t addr7, const char *op, esp_err_t err, int64_t startUs)` [660](boardSupport.c#L660) | 记录耗时、结果与传输计数，失败时分派受限错误处理。 / Record duration, result and counters, dispatching bounded error handling on failure. | `boardSupportI2cWriteRead` / `boardSupportI2cWrite` |
| `static esp_err_t boardSupportI2cConfigureRecoveryPins(BoardSupportI2cCtx_t *ctx)` [698](boardSupport.c#L698) | 在总线恢复阶段配置 SDA/SCL 使其能按要求释放或拉低。 / Configure SDA/SCL for controlled drive/release during bus recovery. `ESP_ERR_INVALID_ARG` | `boardSupportRecoverI2cBus` |
| `static void boardSupportI2cPulseRecoveryClock(BoardSupportI2cCtx_t *ctx)` [714](boardSupport.c#L714) | 在恢复预算内产生 SCL 脉冲并观察 SDA 是否释放。 / Pulse SCL during recovery and observe SDA release. | `boardSupportRecoverI2cBus` |
| `esp_err_t boardSupportRecoverI2cBus(const BoardSupportI2cCtx_t *userCtx)` [738](boardSupport.c#L738) | 在总线锁、冷却与失败预算约束下释放卡低线并重新建立驱动状态。 / Recover a stuck bus and re-establish driver state subject to bus lock, cooldown and failure budget. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `boardSupportI2cHandleTransactionError` |
| `esp_err_t boardSupportSetI2cFrequency(const BoardSupportI2cCtx_t *userCtx, uint32_t frequencyHz)` [861](boardSupport.c#L861) | 在对应总线所有权下更新时钟并报告应用结果。 / Change the I²C clock under bus ownership and return application status. `ESP_ERR_INVALID_ARG` | `frontendReadFdc` / `sensorarrayProbeFdcIdsAtClock` |
| `esp_err_t boardSupportInit(void)` [911](boardSupport.c#L911) | 初始化主 I²C 资源，在配置允许时初始化第二条总线。 / Initialise primary I²C resources and the secondary bus when configured. | `sensorarrayInitBoardAndRouting` |
| `esp_err_t boardSupportDeinit(void)` [990](boardSupport.c#L990) | 删除已安装的板级 I²C 驱动并清除初始化状态。 / Remove installed board I²C drivers and clear their initialisation state. | 独立接口 / Standalone entry |
| `bool boardSupportIsI2c1Enabled(void)` [1019](boardSupport.c#L1019) | 查询次总线的编译选择及可用引脚/端口。 / Check secondary-bus enablement and pin/port eligibility. | `boardSupportInit` |
| `const BoardSupportI2cCtx_t *boardSupportGetI2cCtx(void)` [1026](boardSupport.c#L1026) | 取得主总线的板级上下文指针，供驱动回调传递。 / Return the primary board bus context for driver callbacks. | `sensorarrayLogFdcParallelCfg` / `sensorarrayFrontendStartupRun` |
| `const BoardSupportI2cCtx_t *boardSupportGetI2c1Ctx(void)` [1031](boardSupport.c#L1031) | 在可用时返回次总线上下文，否则返回空指针。 / Return the secondary bus context when available or NULL. | `sensorarrayFrontendStartupRecheckFdcClock` / `sensorarraySweepFdcI2cClockUnderLoad` |
| `bool boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *outInfo)` [1039](boardSupport.c#L1039) | 将指定主/次总线的配置、在线状态、时钟和错误计数复制给调用者。 / Copy the selected primary or secondary bus configuration, online state, clock and error counts. | `sensorarrayFrontendStartupRecheckFdcClock` / `sensorarraySweepFdcI2cClockUnderLoad` |
| `esp_err_t boardSupportI2cWriteRead(void *userCtx, uint8_t addr7, const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxLen)` [1069](boardSupport.c#L1069) | 持锁执行组合写/读事务并按结果更新总线统计。 / Execute a combined write/read under the bus lock and account for its result. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupProbeFdcAddress` / `sensorarrayBringupReadFdcIdsRaw` |
| `esp_err_t boardSupportI2cWrite(void *userCtx, uint8_t addr7, const uint8_t *tx, size_t txLen)` [1114](boardSupport.c#L1114) | 向指定的 7 位从地址发出有界写事务，保留总线诊断。 / Write to the seven-bit slave address with bounded transfer and bus diagnostics. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupInitFdcDevice` |
| `esp_err_t boardSupportI2cRead(void *userCtx, uint8_t addr7, uint8_t *rx, size_t rxLen)` [1154](boardSupport.c#L1154) | 从指定的 7 位从地址读取给定字节数并保存传输状态。 / Read the requested byte count from the seven-bit slave address and retain transfer status. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `static bool boardSupportI2cIsDataAll1Sequence(const uint8_t *regs, size_t regCount)` [1194](boardSupport.c#L1194) | 检查读出的字节序列是否全为 0xFF。 / Test whether every byte of the read sequence is 0xFF. | `boardSupportI2cReadReg16Sequence` |
| `static esp_err_t boardSupportI2cBuildReg16SequenceCommand(i2c_cmd_handle_t cmd, uint8_t addr7, const uint8_t *regs, size_t regCount, uint8_t *rx)` [1207](boardSupport.c#L1207) | 在同一命令序列中建立多个 16 位寄存器的选址与读回操作。 / Build a command sequence that addresses and reads multiple 16-bit registers. `ESP_ERR_INVALID_ARG` | `boardSupportI2cReadReg16Sequence` |
| `esp_err_t boardSupportI2cReadReg16Sequence(void *userCtx, uint8_t addr7, const uint8_t *regs, size_t regCount, uint16_t *outValues, uint32_t timeoutMs)` [1246](boardSupport.c#L1246) | 读取指定顺序的一组 16 位寄存器并保留总线错误。 / Read an ordered sequence of 16-bit registers and preserve transfer errors. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupInitFdcDevice` / `frontendReadFdc` |
| `static bool boardSupportI2cExpectedSecondaryHighAddressNack(const BoardSupportI2cCtx_t *ctx, uint8_t addr7)` [1357](boardSupport.c#L1357) | 识别可预期的次总线高地址 NACK 并交给相应诊断策略。 / Identify an expected high-address NACK on the secondary bus for diagnostic handling. | `boardSupportI2cProbeAddress` |
| `esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *userCtx, uint8_t addr7)` [1362](boardSupport.c#L1362) | 发送地址探测以区分从设备应答、NACK 与总线错误。 / Probe the address and return ACK, NACK or transfer failure. `ESP_ERR_INVALID_ARG` `ESP_ERR_NO_MEM` | `sensorarrayBringupProbeFdcCandidate` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `...`, `format` | printf 风格的格式串与匹配格式的可变参数。 / printf-style format string and its matching variadic arguments. |
| `addr7` | 7 位 I²C 设备地址，匹配对应总线实例。 / Seven-bit I²C address on the selected bus. |
| `busState` | 总线状态。 / Bus state. |
| `callback` | 使用头文件定义签名的回调。提供的上下文与回调在注册或同步调用期间有效。 / Callback with the signature declared in the header. Keep its context valid while registered or used. |
| `cmd` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `ctx` | 所声明类型的应用、总线或调度上下文，供调用链共享。 / Application, bus or scheduler context of the declared type, shared with the call chain. |
| `err` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `frequencyHz` | 总线或器件时钟，Hz。请求值与驱动实际速率分别记录。 / Bus or device clock in Hz. Requested and actual driver clock are tracked separately. |
| `gpio`, `port` | ESP-IDF GPIO 编号或 I²C 控制器端口号。 / ESP-IDF GPIO number or I²C controller port. |
| `handler` | 处理器。 / Handler. |
| `op`, `stage` | 对应操作、诊断或状态机的枚举/标签，采用签名中所列类型的常量。 / Operation, diagnostic or state-machine discriminator from its declared enum or label domain. |
| `outAction` | 接收结果。对应操作、诊断或状态机的枚举/标签，采用签名中所列类型的常量。 / Output. Operation, diagnostic or state-machine discriminator from its declared enum or label domain. |
| `outInfo` | 接收结果。声明的诊断/状态记录，或从器件读回的原始状态字。 / Output. Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `outSclLevel` | 接收SCL电平。 / Receives sCL level. |
| `outSdaLevel` | 接收SDA电平。 / Receives sDA level. |
| `outValues` | 接收结果。要归约或校验的一组测量，数组长度由相邻 count 参数或签名给出。 / Output. Measurements to reduce or validate. The adjacent count or declared array bound gives their extent. |
| `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `regCount` | 寄存器数量。 / Register count. |
| `regs` | 对应寄存器字节或一批要读取的寄存器地址。 / Register bytes or the list of register addresses to read. |
| `rx`, `tx` | 接收、发送或长期存储缓冲；与签名中的容量或传输长度配对。 / Receive, transmit or backing buffer paired with the supplied capacity/transfer length. |
| `rxLen`, `txLen` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `sclGpio` | SCLGPIO。 / SCL GPIO. |
| `sdaGpio` | SDAGPIO。 / SDA GPIO. |
| `secondary` | 主/次总线、器件、采样结果或选择位，其类型由完整签名指定。 / Primary/secondary bus, device, sample or selector as indicated by its type. |
| `startUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `timeoutMs`, `timeoutTicks` | 有限等待预算或硬超时标志，时间单位见参数后缀。 / Bounded waiting budget or hard-timeout flag. Time units follow the suffix. |
| `userCtx` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |

[项目总览 / Project guide](../../README.md)
