# Wi‑Fi AP、UDP 和请求状态 / Wi-Fi AP, UDP and request state

应用通过持久的 CPU0 owner 执行 ESP-IDF Wi-Fi 与 lwIP 发送调用，独立 reader 通过控制 UDP 端口收命令。AP 使用配置提供的 SSID、认证与 DATA/LOG/CTRL 端口。HIGH 和 BLE_COMPAT 配置分别选择 HT40 和 HT20，并有不同驱动缓冲预算。 / A persistent CPU0 owner performs ESP-IDF Wi-Fi and lwIP send calls. A separate reader receives commands on the control UDP port. Configuration supplies AP SSID, authentication and DATA/LOG/CTRL ports. HIGH selects HT40 while BLE_COMPAT selects HT20, with corresponding driver buffer budgets. [公共 API / Public API](include/sensorarrayWifi.h) · [后端边界 / Backend interface](sensorarrayWifiOwner.h) · [命令调用者 / Command handling](../../core/transport/sensorarrayTransportWifiControl.c)

## 调用与返回 / Requests and results

在启动门打开后通过 `sensorarrayWifiRequestAp(&cfg, &id)` 提交，按 id 查询终态。相同已用配置可直接得到 applied；若在已有 AP 上改变 SSID、端口等非 profile 字段，owner 会报告不支持。OFF 立即撤销输出授权并推进 epoch，物理 SDK 调用和 reader 可能仍在执行。初始化、发送和收尾均有观测期限；SDK 返回前会保留已拥有的存储。 / Once startup permits it, submit an AP request with sensorarrayWifiRequestAp(&cfg, &id) and query its terminal result by ID. An already-applied identical configuration can complete immediately. Changing non-profile fields such as SSID or ports on a running backend yields an unsupported result. OFF revokes output permission and advances epochs immediately. The physical SDK call and reader may still exist. Initialisation, send and cleanup have observed deadlines. Owned storage stays live until the underlying operation returns.

下面为上游提交普通数据报的形状。传入完整负载及其实际长度，身份已知时还填入原帧的 sessionId、bootId、revision 和 sequence。UDP 发送结果通过 owner 回调记录，PC 收到需要由接收端观察。 / For an ordinary publication, pass the whole datagram and its byte length as below. When identity is known, also fill sessionId, bootId, revision and sequence from the original frame. The owner callback records the UDP send result. Observe actual receipt at the host.

```c
uint32_t epoch;
if (sensorarrayWifiGetSendEpoch(false, &epoch)) {
    sensorarrayWifiPublication_t p = {
        .channel = SENSORARRAY_WIFI_CH_DATA,
        .data = bytes, .length = size, .expectedEpoch = epoch,
    };
    esp_err_t admission = sensorarrayWifiSubmit(&p);
    /* ESP_OK means the component copied and admitted the datagram. */
    (void)admission;
}
```

普通槽、生命周期槽和控制槽提供有界排队。数据最大 2304 字节，控制最大 512 字节。发送期限 500 ms，排队期限 1 s，初始化 5 s，清理 1 s，观察周期 100 ms。调用者可指定更早的绝对微秒期限。 / Ordinary, lifecycle and control slots bound queue capacity. Data supports up to 2304 bytes and control up to 512 bytes. Send budget is 500 ms, queue lifetime 1 s, initialisation 5 s, cleanup 1 s and observer period 100 ms. A caller may supply an earlier absolute deadline in microseconds.

| 参数或输出 / Argument or output | 使用约定 / Interpretation |
|---|---|
| `expectedEpoch`, `deadlineUs` | 从 GetSendEpoch 取得与通道匹配的 linkEpoch。可选绝对单调期限零表示使用队列寿命。 / Capture a matching link epoch using GetSendEpoch. Zero deadline selects the queue's own lifetime, otherwise pass absolute monotonic µs. |
| `outRequestId`, `id`, `result` | 接受后以非零 id 查请求；查询 id 为零取最新。result 给出原始/收尾错误及终态尝试次数。 / Look up an accepted request by nonzero ID, or query zero for latest. The result exposes original/cleanup errors and terminal enqueue attempts. |
| `peer`, `unicast`, `returnedBytes`, `sdkErrno` | 地址采用网络字节序 IPv4 和主机端口号。发送回调记录 SDK 返回字节数和 errno，完整数据报要求正好等于请求长度。 / A peer holds IPv4 in network order and port in host order. The callback reports SDK bytes and errno. A full datagram requires the requested byte count. |

## [sensorarrayWifi.c](sensorarrayWifi.c)

### 组件内部接口 / Component interface

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t sensorarrayWifiBackendApplyProfile(sensorarrayWifiProfile_t profile)` | 按 profile 配置发射功率、省电、协议和带宽，逐项检查 SDK 结果。 / Apply TX power, power save, protocol and bandwidth for profile, checking each SDK stage. | [wifiRunRequest](sensorarrayWifiOwner.c#L549) |
| `esp_err_t sensorarrayWifiBackendInitialize(const sensorarrayWifiConfig_t *config)` | 按受观测步骤建立 netif、AP、UDP socket 和持久控制 reader。 / Bring up netif, AP, UDP sockets and persistent control reader in observed SDK stages. | [wifiRunRequest](sensorarrayWifiOwner.c#L549) |
| `esp_err_t sensorarrayWifiBackendCleanup(void)` | 按已实际取得的资源收尾，reader 存活或驱动初始化状态未知时返回 INVALID_STATE。 / Release acquired resources where possible. Return INVALID_STATE for a live reader or uncertain driver initialisation. | [wifiRunRequest](sensorarrayWifiOwner.c#L549) |
| `bool sensorarrayWifiBackendHasReader(void)` | 返回控制 reader 是否已创建。 / Return whether a persistent control reader has been created. | [wifiRunRequest](sensorarrayWifiOwner.c#L549) |
| `esp_err_t sensorarrayWifiBackendSend(const sensorarrayWifiPublication_t *p, int *returned, int *sdkErrno)` | 向按通道选定的广播端口或给定 peer 单播，回报 sendto 的长度和 errno。 / Send to a channel broadcast address or supplied unicast peer. Report sendto byte count and errno. | [wifiRunDatagram](sensorarrayWifiOwner.c#L505) |
| `esp_err_t sensorarrayWifiBackendStationQuery(bool *connected)` | 调用 SDK 获取 AP station 列表，将是否有连接写入 connected。 / Query the AP station list and return presence via connected. | [sensorarrayWifiOwnerProcessOnce](sensorarrayWifiOwner.c#L600) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void sensorarrayWifiEvent(void *arg, esp_event_base_t base, int32_t eventId, void *eventData)` | 从 AP station 连接/断开事件更新 owner 对站点的观测。 / Forward AP station connect/disconnect events to the owner's link observation. | [sensorarrayWifiBackendInitialize](sensorarrayWifi.c#L176) |
| `static void sensorarrayWifiConfigureInit(wifi_init_config_t *init, sensorarrayWifiProfile_t profile)` | 按 HIGH 或 BLE_COMPAT 设置驱动收发缓冲、BA 窗口和管理缓冲数量。 / Fill driver RX/TX buffer counts, BA window and management buffers for HIGH or BLE_COMPAT. | [sensorarrayWifiPrepareInitScratch](sensorarrayWifi.c#L97) |
| `static void sensorarrayWifiPrepareInitScratch(const sensorarrayWifiConfig_t *config)` | 以持久 BSS 结构保存 SDK 初始化参数，包括 netif、驱动和 AP 配置。 / Prepare the persistent BSS netif, driver and AP configuration objects for SDK initialisation. | [sensorarrayWifiBackendInitialize](sensorarrayWifi.c#L176) |
| `static esp_err_t sensorarrayWifiSocketOption(int fd, int option, const void *value, socklen_t length)` | 在 SDK 阶段跟踪内调用 setsockopt(fd, SOL_SOCKET, option, …)，记录 errno。 / Apply the supplied socket option within the tracked SDK phase and retain errno. | [sensorarrayWifiOpenSockets](sensorarrayWifi.c#L133) |
| `static esp_err_t sensorarrayWifiOpenSocket(int *fd)` | 创建 IPv4 UDP socket 并将描述符写入 fd。 / Create an IPv4 UDP socket and store its descriptor in fd. | [sensorarrayWifiOpenSockets](sensorarrayWifi.c#L133) |
| `static esp_err_t sensorarrayWifiOpenSockets(void)` | 创建发送和控制 socket，配置广播，绑定控制端口及接收时限。 / Create TX/control sockets, enable broadcast, bind the control port and set its receive timeout. | [sensorarrayWifiBackendInitialize](sensorarrayWifi.c#L176) |
| `static void sensorarrayWifiControlTask(void *arg)` | 独立 reader 从 UDP recvfrom 获取命令及来源地址；隔离时停止接收新工作。 / Receive commands and source addresses with recvfrom in the isolated reader and stop admitting new RX work on quarantine. | [sensorarrayWifiBackendInitialize](sensorarrayWifi.c#L176) |
| `static esp_err_t sensorarrayWifiCloseSocket(int *fd)` | 尝试关闭已拥有的 fd，成功后设为 −1。 / Close an owned socket and set fd to −1 only after success. | [sensorarrayWifiBackendCleanup](sensorarrayWifi.c#L247) |

## [sensorarrayWifiOwner.c](sensorarrayWifiOwner.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `void sensorarrayWifiObserveDeadline(void)` | 由定时观察器在锁内检查期限并发布故障观测。 / Perform a deadline observation under lock from the timer callback. | [sensorarrayWifiWatchdogTick](sensorarrayWifiWatchdog.c#L9) |
| `void sensorarrayWifiConfigureBoot(uint64_t sessionId, uint32_t bootId)` | 首次准备资源前绑定非零 sessionId 和 bootId。 / Bind a nonzero sessionId and its bootId before resource preparation. | [sensorarrayTransportConfigureWifiBoot](../../core/transport/sensorarrayTransport.c#L417) |
| `void sensorarrayWifiAllowOwnerStart(void)` | 启动顺序到达允许点后打开 AP 请求准入。 / Open AP request admission once the startup sequence allows it. | [sensorarrayTransportReleaseWifiStartupGate](../../core/transport/sensorarrayTransport.c#L422) |
| `esp_err_t sensorarrayWifiRequestAp(const sensorarrayWifiConfig_t *config, uint32_t *outRequestId)` | 验证并复制配置，返回准入结果和可查询的请求号。 / Validate and copy the AP configuration and return admission status plus a queryable request ID. | [sensorarrayTransportStartWifiAp](../../core/transport/sensorarrayTransport.c#L410) |
| `esp_err_t sensorarrayWifiRequestOff(uint32_t *outRequestId)` | 撤销输出、推进 epoch，给被替代请求和本次逻辑 OFF 分别提交终态。 / Revoke output, advance epochs and complete both any superseded request and this logical OFF request. | [sensorarrayTransportRequestWifiMode](../../core/transport/sensorarrayTransport.c#L1319) |
| `bool sensorarrayWifiGetRequestedAp(bool *requestedAp)` | 返回是否已有请求，并将最近 AP/OFF 意图写入 requestedAp。 / Report whether a request exists and copy its most recent AP/OFF intent. | [sensorarrayTransportGetWifiMode](../../core/transport/sensorarrayTransportStats.c#L142) |
| `bool sensorarrayWifiGetSendEpoch(bool control, uint32_t *outEpoch)` | 按数据或控制准入条件读取有效链路代号。 / Capture a sendable link epoch for control or ordinary output. | [sensorarrayTransportDrainGuaranteedText](../../core/transport/sensorarrayTransport.c#L735) |
| `esp_err_t sensorarrayWifiSubmit(const sensorarrayWifiPublication_t *p)` | 根据通道、容量与 epoch 验证，深拷贝完整负载到槽；ESP_OK 表示已准入。 / Validate channel, size and epoch and deep-copy the payload into a slot. ESP_OK means admission. | [sensorarrayTransportProcessDescriptor](../../core/transport/sensorarrayTransport.c#L610) |
| `void sensorarrayWifiSetControlRxCallback(sensorarrayWifiControlRxCallback_t callback, void *context)` | 登记命令接收回调及其上下文。 / Register the control receive callback and its context. | [sensorarrayTransportInitOwned](../../core/transport/sensorarrayTransport.c#L1127) |
| `void sensorarrayWifiSetOwnerCallbacks(sensorarrayWifiTxResultCallback_t result, void *context, sensorarrayWifiWakeCallback_t wake)` | 登记数据报完成回调、上下文和传输唤醒回调。 / Register transmit-result and transport-wake callbacks with their context. | [sensorarrayTransportInitOwned](../../core/transport/sensorarrayTransport.c#L1127) |
| `void sensorarrayWifiGetStats(sensorarrayWifiStats_t *out)` | 获取各通道准入、丢弃、完整或部分发送及驱动接受字节。 / Copy per-channel admission, drops, complete/partial sends and driver-accepted byte counters. | [sensorarrayTransportPrintNetInit](../../core/transport/sensorarrayTransport.c#L592) |
| `bool sensorarrayWifiIsReady(void)` | 查询当前输出授权、后端就绪与隔离共同决定的状态。 / Check output permission, backend readiness and quarantine through current statistics. | [sensorarrayTransportWifiSinkEnabled](../../core/transport/sensorarrayTransportStats.c#L184) |
| `void sensorarrayWifiCopyOwnerSnapshot(sensorarrayWifiOwnerSnapshot_t *out)` | 复制 owner、SDK 阶段和期限、槽拥有量、隔离及结果快照。 / Copy owner, SDK phase/deadline, slot ownership, quarantine and fault evidence. | [sensorarrayTransportReleaseWifiStartupGate](../../core/transport/sensorarrayTransport.c#L422) |
| `bool sensorarrayWifiCopyRequestResult(uint32_t id, sensorarrayWifiRequestResult_t *out)` | 按 id 复制活动请求或缓存终态，零选最新，返回是否查到。 / Copy the current/cached result by ID, with zero selecting latest. Return whether found. | [sensorarrayTransportWifiCommand](../../core/transport/sensorarrayTransportWifiControl.c#L72) |
| `bool sensorarrayWifiCopyPendingTerminal(sensorarrayWifiRequestResult_t *out)` | 选出到达重试时间、还未用尽发送次数的终态副本。 / Copy a pending terminal whose retry time has arrived and attempt allowance remains. | [sensorarrayTransportWifiDrainTerminal](../../core/transport/sensorarrayTransportWifiControl.c#L167) |
| `void sensorarrayWifiNoteTerminalAttempt(uint32_t id, bool queued)` | 按 id 记录实际本地排队结果，失败时推迟下一次有限重试。 / Record actual local queue admission by ID and schedule another bounded attempt on failure. | [sensorarrayTransportWifiDrainTerminal](../../core/transport/sensorarrayTransportWifiControl.c#L167) |
| `bool sensorarrayWifiOwnerNeedsPoll(void)` | 指示管理请求、活动 SDK 或待发终态是否需要继续观察。 / Test whether management, active SDK work or a pending terminal needs polling. | [sensorarrayTransportWaitTicks](../../core/transport/sensorarrayTransport.c#L370) |
| `const char *sensorarrayWifiRequestOutcomeName(sensorarrayWifiRequestOutcome_t outcome)` | 将请求结果枚举映射为协议中使用的名称。 / Map request outcome to its protocol label. | [wifiFormatRequest](../../core/transport/sensorarrayTransportWifiControl.c#L41) |
| `const char *sensorarrayWifiPhaseName(sensorarrayWifiPhase_t phase)` | 将后端执行阶段映射为诊断名称，未知值返回 unknown。 / Name the backend SDK phase, returning unknown for unrecognised values. | [sensorarrayTransportWifiCommand](../../core/transport/sensorarrayTransportWifiControl.c#L72) |

### 组件内部接口 / Component interface

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `bool sensorarrayWifiOwnerMayRunSdk(void)` | 检查任务身份、owner、期限及隔离状态，返回是否可执行 SDK 步骤。 / Test task identity, ownership, deadline and quarantine before an SDK operation. | 公开入口 / Callable API |
| `bool sensorarrayWifiOwnerBeginSdk(sensorarrayWifiPhase_t phase)` | 登记阶段、callId、起始和绝对期限，拒绝越权或重入。 / Record phase, call ID, start and deadline, rejecting invalid ownership or re-entry. | [sensorarrayWifiSocketOption](sensorarrayWifi.c#L113) |
| `bool sensorarrayWifiOwnerEndSdk(esp_err_t error, int sdkErrno, int returnedBytes)` | 记录 SDK 错误、errno、字节及迟到，返回能否继续后续步骤。 / Record the SDK result, errno, byte count and late returns, reporting whether another step may proceed. | [sensorarrayWifiSocketOption](sensorarrayWifi.c#L113) |
| `bool sensorarrayWifiOwnerProcessOnce(void)` | 由 owner 每次选择并执行一项数据报、配置或站点核对工作。 / Process one eligible datagram, configuration request or station reconciliation from the actual owner. | [sensorarrayWifiOwnerTask](sensorarrayWifiOwner.c#L267) |
| `void sensorarrayWifiOwnerNoteStationEvent(bool connected)` | 按连接事件更新 station 状态和 linkEpoch，断开时安排再次核对。 / Update station state/link epoch and schedule reconciliation after a disconnect. | [sensorarrayWifiEvent](sensorarrayWifi.c#L59) |
| `bool sensorarrayWifiOwnerRxAllowed(void)` | 返回当前是否允许控制 reader 递交新工作。 / Return whether the control reader may dispatch work in the current quarantine state. | [sensorarrayWifiControlTask](sensorarrayWifi.c#L154) |
| `void sensorarrayWifiOwnerReceive(const uint8_t *data, size_t length, const sensorarrayWifiPeer_t *peer)` | 把借用的命令字节和来源 peer 转交已注册的回调。 / Deliver borrowed command bytes and their peer address to the registered callback. | [sensorarrayWifiControlTask](sensorarrayWifi.c#L154) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static uint64_t wifiNow(void)` | 给出当前单调微秒时间。 / Read monotonic time in microseconds. | [wifiRunDatagram](sensorarrayWifiOwner.c#L505) |
| `static uint64_t wifiAdd(uint64_t a, uint64_t b)` | 对无符号 64 位时间和预算做饱和加法。 / Add a budget to a 64-bit time with saturation. | [wifiObserveLocked](sensorarrayWifiOwner.c#L179) |
| `static uint32_t wifiNext(uint32_t value)` | 推进 32 位代号或 ID，回绕时跳过零。 / Advance a 32-bit generation or ID, skipping zero on wrap. | [wifiNewRequestLocked](sensorarrayWifiOwner.c#L340) |
| `static bool wifiTerminal(sensorarrayWifiRequestOutcome_t outcome)` | 判断请求是否为 applied、failed、superseded 或 quarantined。 / Test for an applied, failed, superseded or quarantined result. | [wifiFinishRequestLocked](sensorarrayWifiOwner.c#L144) |
| `static void wifiWake(void)` | 锁内快照上层唤醒回调，锁外执行。 / Copy the transport wake callback under the lock and invoke it outside. | [wifiRunDatagram](sensorarrayWifiOwner.c#L505) |
| `static void wifiNotifyOwner(void)` | 通知持久 owner 有新工作需要处理。 / Notify the persistent owner of available work. | [sensorarrayWifiRequestAp](sensorarrayWifiOwner.c#L351) |
| `static sensorarrayWifiRequestResult_t *wifiFindResultLocked(uint32_t id)` | 在当前请求和终态缓存查找 id，零表示最新。 / Find id in the live request or result cache. Zero selects the latest ID. | [sensorarrayWifiGetRequestedAp](sensorarrayWifiOwner.c#L413) |
| `static void wifiFinishRequestLocked(uint32_t id, sensorarrayWifiRequestOutcome_t outcome, esp_err_t error, esp_err_t cleanup, uint64_t now)` | 一次提交终态，保存首次错误、收尾结果和终态重试信息。 / Commit a terminal result once, with original error, cleanup result and notification retry state. | [wifiQuarantineLocked](sensorarrayWifiOwner.c#L163) |
| `static void wifiQuarantineLocked(esp_err_t error, uint64_t now)` | 锁定隔离状态并冻结首次错误和观测时间，撤止新准入。 / Latch quarantine, the first fault and detection time, revoking new admission. | [wifiObserveLocked](sensorarrayWifiOwner.c#L179) |
| `static void wifiObserveLocked(uint64_t now)` | 检测正在执行的 SDK 或仍在等待的请求有无越过期限。 / Check active SDK work and pending management requests for deadline expiry. | [sensorarrayWifiObserveDeadline](sensorarrayWifiOwner.c#L198) |
| `static bool wifiSameConfig(const sensorarrayWifiConfig_t *a, const sensorarrayWifiConfig_t *b, bool includeProfile)` | 比较两套 AP 配置，可选是否连 profile 一起比较。 / Compare two AP configurations with optional profile comparison. | [wifiRunRequest](sensorarrayWifiOwner.c#L549) |
| `static void sensorarrayWifiOwnerTask(void *arg)` | 循环处理待发数据、管理请求、站点核对，并在空闲时等待通知。 / Process queued datagrams, management and station reconciliation, waiting for work when idle. | [wifiPrepareResources](sensorarrayWifiOwner.c#L283) |
| `static esp_err_t wifiPrepareResources(void)` | 只申请一次持久负载槽、owner 任务和观察器，并公布初始化结果。 / Prepare persistent payload slots, owner task and observer once, publishing resource readiness. | [sensorarrayWifiRequestAp](sensorarrayWifiOwner.c#L351) |
| `static uint32_t wifiNewRequestLocked(bool ap, uint64_t now)` | 分配非零请求号，记录 AP/OFF 意图、epoch 和接受时间。 / Allocate a nonzero request ID and record AP/OFF intent, epoch and admission time. | [sensorarrayWifiRequestAp](sensorarrayWifiOwner.c#L351) |
| `static int wifiTakeSlotLocked(void)` | 先选控制与生命周期槽，再给管理请求机会，之后按入队顺序选普通槽。 / Choose CTRL then lifecycle, allow pending management to run, then take the oldest ordinary slot. | [sensorarrayWifiOwnerProcessOnce](sensorarrayWifiOwner.c#L600) |
| `static void wifiRunDatagram(unsigned index, const sensorarrayWifiPublication_t *publication)` | 核对代号和期限后发送，记录完整、部分、过期或陈旧结果，归还槽并回调。 / Check epoch and deadline, attempt the datagram, record full/partial/expired/stale result, reclaim the slot and notify. | [sensorarrayWifiOwnerProcessOnce](sensorarrayWifiOwner.c#L600) |
| `static void wifiRunRequest(const wifiExecution_t *execution)` | 执行 AP 初始化或 profile 切换，独立保存主错误与必要收尾错误。 / Run AP setup or a profile change, preserving the original and cleanup results separately. | [sensorarrayWifiOwnerProcessOnce](sensorarrayWifiOwner.c#L600) |

## [sensorarrayWifiWatchdog.c](sensorarrayWifiWatchdog.c)

### 组件内部接口 / Component interface

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t sensorarrayWifiWatchdogInit(void)` | 仅尝试一次创建周期观察定时器，后续返回已保存的结果。 / Attempt to create/start the periodic observer once and return the saved result thereafter. | [wifiPrepareResources](sensorarrayWifiOwner.c#L283) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void sensorarrayWifiWatchdogTick(void *context)` | 定时调用期限观察；上下文参数由 ESP Timer 提供。 / Run a deadline observation from the periodic ESP Timer callback. | [sensorarrayWifiWatchdogInit](sensorarrayWifiWatchdog.c#L15) |
