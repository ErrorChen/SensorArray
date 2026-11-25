# 逐行混合采集 / Mixed-row acquisition

一个计划为每个活动行指定 C、V 或 R。执行器按模式分组调用现有引擎，将返回位掩码和科学数据映射回原行。所有组共用一份目标帧，另用调用者提供的独立 segmentWorkspace 保存子结果。保留已计划行，即使子段失败也能按行输出无效原因。

A plan assigns C, V or R to each active row. The executor groups rows by mode, calls the existing engines and maps scientific values and masks back to their original rows. It uses one output frame and separate caller-owned segmentWorkspace for child results. Planned rows survive a child failure so the encoder can report invalid cells in their original positions.

## 调用方式 / Calling the module

```c
esp_err_t err = sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
    &ctx->adsEngine, &ctx->scanPlan, &ctx->frame, ctx->mixedSegmentWorkspace);
```

此调用属于帧边界与采集协调器，两个 frame 指针必须指向不同对象。先处理 rail 前置条件，再采 C/V/R 各组，各组起止时间用于报告采样偏移。行单位跟随自身模式，fresh 与电气上 valid 分开报告。

Call on the frame-boundary acquisition coordinator with two distinct frame objects. The executor admits rail preflight before acquiring C/V/R groups. Group start/end times quantify capture skew. Each row retains its own unit, and freshness and electrical validity are reported separately.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayMixedRow.c](sensorarrayMixedRow.c) · [sensorarrayMixedRow.h](sensorarrayMixedRow.h) | 使用各模式的行掩码执行分组帧，分别合并时序与单元结果，保留每个物理行的模式和单位。 / Executes mode-group frames with row masks and merges timings and cell results while preserving each physical row's mode and units. |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

### sensorarrayMixedRow.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static uint64_t sensorarrayMixedExpectedMask(uint8_t activeRows)` [11](sensorarrayMixedRow.c#L11) | 计算前 activeRows 行的 64 位预期采集掩码。 / Compute the 64-bit expected-cell mask for the first activeRows rows. | `sensorarrayMixedCountCells` / `sensorarrayMixedRowEngineReadFrame` |
| `static void sensorarrayMixedMergeSegment(sensorarrayFrame_t *target, const sensorarrayFrame_t *segment, uint8_t rowMask, sensorarrayMeasurementMode_t mode)` [18](sensorarrayMixedRow.c#L18) | 仅合入指定 rowMask 的对应测点，未采得的单元保留无效原因。 / Merge only selected rowMask cells and retain invalid reasons for unacquired cells. | `sensorarrayMixedRowEngineReadFrame` |
| `static void sensorarrayMixedMergeSegmentTiming(sensorarrayFrame_t *target, const sensorarrayFrame_t *segment, sensorarrayMeasurementMode_t mode)` [167](sensorarrayMixedRow.c#L167) | 保留该模式子段的起止时间及有关阶段统计。 / Preserve group start/end times and phase statistics for the selected mode. | `sensorarrayMixedRowEngineReadFrame` |
| `static uint32_t sensorarrayMixedCountCells(uint64_t mask, uint8_t activeRows)` [192](sensorarrayMixedRow.c#L192) | 对活动行覆盖范围内的置位测点计数。 / Count set cell bits within the active rows. | `sensorarrayMixedRowEngineReadFrame` |
| `esp_err_t sensorarrayMixedRowPrepareRailAtBoundary(sensorarrayAdsMatrixEngine_t *adsEngine, const sensorarrayScanPlan_t *plan, uint32_t frameSequence, esp_err_t *outCleanupError)` [203](sensorarrayMixedRow.c#L203) | 在所有子模式开始前准备可用 rail 并保留独立恢复错误。 / Prepare rail evidence before any child mode starts and preserve the independent cleanup error. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayMixedRowEngineReadFrame` |
| `esp_err_t sensorarrayMixedRowEngineReadFrame(sensorarrayFdcMatrixEngine_t *fdcEngine, sensorarrayAdsMatrixEngine_t *adsEngine, const sensorarrayScanPlan_t *plan, sensorarrayFrame_t *frame, sensorarrayFrame_t *segmentWorkspace)` [301](sensorarrayMixedRow.c#L301) | 按 C/V/R 行掩码分组执行各引擎，把每组新采得的单元与时序合入原坐标。 / Run child engines for C/V/R row masks and merge freshly acquired cells and timings at their original coordinates. `ESP_ERR_INVALID_ARG` | `sensorarrayRunOneFrame` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `activeRows` | 生效或申请的活动行数，范围为 1..8。 / Applied or requested number of active rows, 1..8. |
| `adsEngine`, `fdcEngine` | 已初始化且与本次采集绑定的引擎实例。 / Initialised engine instance bound to this acquisition. |
| `frame` | 当前采集帧。const 指针供读取/编码，可写指针供对应采集步骤产生或更新数据。 / Capture frame. A const pointer is an input to readers/encoders. A writable pointer receives or updates acquisition results. |
| `frameSequence` | 对应采集帧的序号，必须与提供的帧、事件或缓存上下文属于同次启动。 / Capture sequence belonging to the same boot as the supplied frame, event or cache context. |
| `mask`, `rowMask` | 作用域内的行、通道或测点选择位。完整科学帧的每个测点各占一位。 / Bitset of selected rows, channels or cells. A full science frame uses one bit per cell. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `outCleanupError` | 接收结果。操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Output. Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `plan` | 活动行的采样操作计划或逐行配置，所有组使用同一帧的配置快照。 / Acquisition plan or row selections for the active frame. All groups use the same capture snapshot. |
| `segment` | 子引擎已产生的单模式帧，供按 rowMask 合并。 / Single-mode child frame to merge into the destination by rowMask. |
| `segmentWorkspace` | 与目标帧不同的可写工作帧，单次调用结束才可复用。 / Writable scratch frame distinct from the destination and reusable after the call. |
| `target` | 此操作的目标，可能为目标帧、路由/测点结构或 sink，以签名类型为准。 / Target frame, route/cell or sink, according to the declared type. |

[项目总览 / Project guide](../../../README.md)
