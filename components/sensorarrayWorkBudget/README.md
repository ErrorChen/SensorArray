# CPU0 工作预算和运行时间 / CPU0 work budget and task runtime

CPU0 上的 BLE、传输、日志、串口及 Wi-Fi 任务在有界工作单元结束后调用检查点，每个角色只有一个写任务。按本地经过时间、单元数或真实 IDLE0 长时间未运行来决定是否阻塞一个 RTOS tick。这个等待算入调用者原有期限。CPU1 的采集周期单独调度。 / BLE, transport, logging, serial and Wi-Fi tasks on CPU0 call a checkpoint at bounded work boundaries. One writer owns each role. Local elapsed time, unit count or elapsed time without a real IDLE0 callback can trigger a single-tick block. Charge the block to the existing caller deadline. The CPU1 acquisition schedule is independent. [预算 API / Budget API](include/sensorarrayWorkBudget.h) · [运行时间 API / Runtime API](include/sensorarrayCpuOwner.h)

在系统启动时调用 `sensorarrayWorkBudgetInit()`。例如 BLE TX 任务在释放硬件、队列和日志锁后可调用下式。返回 true 表示请求了一次实际 RTOS 阻塞。当前本地量子为由 20 ms 换算的 tick（至少 1），全局空闲后备阈值取其两倍，上限为 32 个工作单元。计数可回绕。 / Call sensorarrayWorkBudgetInit at startup. The BLE TX task can call the following after releasing hardware, queue and logging locks. A true result means a real RTOS block was requested. The local quantum is 20 ms converted to ticks with a minimum of one tick. The global idle backstop is twice that quantum. The count limit is 32 work units. Counters wrap.

```c
(void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
```

可选 CPUOWNER?0..7 查询同一 boot 的任务调度运行时间，每角色最多每秒自采样一次，保留采样时刻和陈旧状态。运行时间可包含 ISR 并跨越非 SDK 工作，SDK 调用墙钟时间还包含抢占与等待。 / Optional CPUOWNER?0..7 queries report a role's scheduled runtime in the same boot, self-sampled at most once per second with timestamps and staleness. Runtime may include ISR time and non-SDK work, while SDK wall time includes preemption and waiting.

## [sensorarrayCpuOwner.c](sensorarrayCpuOwner.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `void sensorarrayCpuOwnerCheckpoint(sensorarrayWorkBudgetWorker_t worker)` | 由实际 CPU0 owner 有界采样自身任务号和运行时间；诊断关闭时为空操作。 / Sample the calling CPU0 owner's task number and accumulated runtime at bounded intervals. Compiles to a no-op when disabled. | [sensorarrayWorkBudgetCheckpoint](sensorarrayWorkBudget.c#L138) |
| `bool sensorarrayCpuOwnerCopySnapshot(sensorarrayWorkBudgetWorker_t worker, sensorarrayCpuOwnerSnapshot_t *snapshot)` | 复制已发布的角色数据，标出未知、陈旧、回退或冲突，非法参数返回 false。 / Copy the published role observation including unknown, stale, regressed or conflicting state. Reject invalid arguments. | [sensorarrayCpuOwnerFormat](sensorarrayCpuOwner.c#L156) |
| `bool sensorarrayCpuOwnerHandles(const char *command)` | 检查 command 是否属于 CPUOWNER? 前缀，具体角色由 formatter 验证。 / Recognise the CPUOWNER? command prefix. The formatter validates the role. | [sensorarrayCpuOwnerFormat](sensorarrayCpuOwner.c#L156) |
| `esp_err_t sensorarrayCpuOwnerFormat(const char *command, uint64_t session, char *response, size_t size)` | 验证角色 0..7，按 session 格式化已公布的 CPUOWNER 结果到给定容量。 / Validate role 0..7 and format its published CPUOWNER record for the supplied boot session and buffer capacity. | [sensorarrayEarlyRuntimeQueryCommand](../../main/main.c#L2638) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static const char *sensorarrayCpuOwnerReason(const sensorarrayCpuOwnerSnapshot_t *s)` | 根据快照选择 disabled、writer_conflict 等诊断原因。 / Map snapshot state to a reason such as disabled or writer_conflict. | [sensorarrayCpuOwnerFormat](sensorarrayCpuOwner.c#L156) |
| `static void sensorarrayCpuOwnerNumber(char text[21], bool known, uint64_t value)` | 将已知 64 位计数格式化为十进制，否则输出 unknown。 / Format a known 64-bit count in decimal, otherwise write unknown. | [sensorarrayCpuOwnerFormat](sensorarrayCpuOwner.c#L156) |

## [sensorarrayWorkBudget.c](sensorarrayWorkBudget.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t sensorarrayWorkBudgetInit(void)` | 验证存储并一次注册 CPU0 idle hook，返回注册或状态错误。 / Validate storage and register the CPU0 idle hook once, propagating registration or state errors. | [sensorarrayBleInit](../sensorarrayBle/sensorarrayBle.c#L2482) |
| `bool sensorarrayWorkBudgetCheckpoint(sensorarrayWorkBudgetWorker_t worker)` | 在工作边界按角色更新计数，达到阈值最多延迟一个 tick，并返回是否实际请求阻塞。 / Account one work unit, block for at most one tick when due, and report whether a block was requested. | [sensorarrayBleSendPayload](../sensorarrayBle/sensorarrayBle.c#L1573) |
| `uint32_t sensorarrayWorkBudgetIdleGeneration(void)` | 返回真实空闲代号，尚无观测或存储不可用时返回零。 / Return the observed idle generation, or zero for unknown/unavailable storage. | [sensorarrayBleSendGattValue](../sensorarrayBle/sensorarrayBle.c#L1356) |
| `void sensorarrayWorkBudgetCopySnapshot(sensorarrayWorkBudgetSnapshot_t *snapshot)` | 复制独立计数、tick 参数及当前未闭合空闲间隔，用 UINT32_MAX 表示未知年龄。 / Copy individually sampled counters, tick thresholds and open idle gap, using UINT32_MAX for unknown age. | [sensorarrayLinkDiagnosticsFormat](../../main/control/sensorarrayLinkDiagnostics.c#L17) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static bool sensorarrayWorkBudgetInternal(const void *address, size_t length)` | 检查整段存储均位于内部 DRAM 且按 32 位对齐。 / Validate aligned internal DRAM across the requested address range. | [sensorarrayWorkBudgetStorageInternal](sensorarrayWorkBudget.c#L77) |
| `static bool sensorarrayWorkBudgetStorageInternal(void)` | 验证所有预算状态及计数器的存储位置。 / Check the storage placement for all shared budget state and counters. | [sensorarrayWorkBudgetInit](sensorarrayWorkBudget.c#L115) |
| `static uint32_t sensorarrayWorkBudgetLoad(const uint32_t *value)` | 执行 32 位 relaxed 原子读取。 / Load a 32-bit counter with relaxed atomic ordering. | [sensorarrayWorkBudgetIdleHook](sensorarrayWorkBudget.c#L96) |
| `static bool sensorarrayWorkBudgetIdleHook(void)` | 在真实 CPU0 idle 回调发布 tick、最大空闲间隔与代号，返回 true 允许等待中断。 / Publish observed tick, longest idle gap and idle generation from CPU0 idle. Return true to permit wait-for-interrupt. | [sensorarrayWorkBudgetInit](sensorarrayWorkBudget.c#L115) |
