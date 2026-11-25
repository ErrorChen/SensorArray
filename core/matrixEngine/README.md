# 同步矩形区域 I/O / Synchronous rectangular I/O

通用执行器在单个互斥量下按行优先顺序处理矩形区域。配置提供设备句柄、选行/列组/驱动回调、可选映射、过采样和参考电阻。生产矩阵的并行 CAP 与自适应 ADS 路径位于 core/measure。区域起点使用从零开始的 row/col。

This reusable executor visits a rectangle in row-major order under one mutex. Configuration supplies device handles, row/column-group/drive callbacks, optional mappings, oversampling and reference resistance. Production parallel CAP and adaptive ADS acquisition live in core/measure. Region origins use zero-based row and column indices.

## 调用方式 / Calling the module

```c
matrixEngineRegion_t region = {.row = 0, .col = 0, .rows = 1, .cols = 1};
matrixEngineRequest_t req = {.io = MATRIX_ENGINE_IO_READ,
                             .measure = MATRIX_ENGINE_MEASURE_CAP_RAW};
int32_t raw;
esp_err_t err = matrixEngineRegionIo(&region, &req, &raw, 1);
```

示例假定已用有效设备和对应板卡回调调用 matrixEngineInit。READ 的 outCount 以 int32_t 元素计，至少 rows × cols。阻塞调用在任务上下文执行。WRITE 通过驱动源和选择回调操作区域。

Initialise with matrixEngineInit, a valid device and appropriate board callbacks before this example. For READ, outCount measures int32_t elements and must cover rows × cols. Call from task context. WRITE operates on the selected region through the drive and selection callbacks.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [matrixEngine.c](matrixEngine.c) · [include/matrixEngine.h](include/matrixEngine.h) | 通过用户映射或默认回调选行和列组，按区域同步执行电容 raw、微伏或毫欧读取。 / Selects rows and column groups with supplied mappings or defaults and synchronously reads capacitance raw codes, µV or mΩ over a rectangle. `matrixEngineIo_t` `matrixEngineMeasure_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

### matrixEngine.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static esp_err_t matrixEngineSelectRowDefault(uint16_t row, void *userCtx)` [41](matrixEngine.c#L41) | 调用默认 TMUX 选行实现定位矩形区域中的行。 / Use the default TMUX row selector for the region. `ESP_ERR_INVALID_ARG` | `matrixEngineInit` |
| `static esp_err_t matrixEngineSelectColGroupDefault(uint16_t group, void *userCtx)` [50](matrixEngine.c#L50) | 按列组编号驱动 TMUX1134 SELA/SELB。 / Drive TMUX1134 SELA/SELB for the selected column group. `ESP_ERR_INVALID_ARG` | `matrixEngineInit` |
| `static esp_err_t matrixEngineDriveDefault(tmux1108Source_t source, void *userCtx)` [77](matrixEngine.c#L77) | 将区域操作指定的驱动源交给 TMUX1108。 / Pass the requested region drive source to TMUX1108. | `matrixEngineInit` |
| `static uint16_t matrixEngineGroupSize(void)` [83](matrixEngine.c#L83) | 选取显式配置或默认的列组大小。 / Select the configured or default number of columns per group. | `matrixEngineRegionIo` |
| `static uint8_t matrixEngineOversample(void)` [92](matrixEngine.c#L92) | 选择每测点的配置或默认过采样次数。 / Select the configured or default per-cell oversample count. | `matrixEngineReadAdcUv` / `matrixEngineReadCapRaw` |
| `static bool matrixEngineDefaultAdcMuxForCol(uint16_t col, uint8_t *muxp, uint8_t *muxn)` [101](matrixEngine.c#L101) | 根据列号推导 ADS 的正负输入对并检查范围。 / Resolve the default ADS input pair for a column and check bounds. | `matrixEngineReadAdcUv` |
| `static bool matrixEngineRegionValid(const matrixEngineRegion_t *region)` [113](matrixEngine.c#L113) | 检查矩形非空且起点和边界均在配置矩阵范围内。 / Check that the rectangle is nonempty and fits the configured matrix. | `matrixEngineRegionIo` |
| `static esp_err_t matrixEngineReadAdcUv(uint16_t col, int32_t *outUv)` [130](matrixEngine.c#L130) | 对该列按映射选择 ADS 输入并返回过采样后的 µV。 / Select the mapped ADS channel for the column and return the oversampled result in µV. `ESP_ERR_INVALID_STATE` | `matrixEngineRegionIo` |
| `static esp_err_t matrixEngineReadCapRaw(uint16_t col, int32_t *outRaw)` [166](matrixEngine.c#L166) | 根据列号选 FDC 器件和通道，返回其原始码。 / Resolve FDC device and channel by column and return its raw code. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_RESPONSE` | `matrixEngineRegionIo` |
| `static esp_err_t matrixEngineUvToResistanceMohm(int32_t uv, int32_t *outMohm)` [212](matrixEngine.c#L212) | 用配置的激励和参考电阻把采得 µV 转换成 mΩ。 / Convert measured µV to mΩ using the configured excitation and reference resistor. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | `matrixEngineRegionIo` |
| `esp_err_t matrixEngineInit(const matrixEngineConfig_t *cfg)` [230](matrixEngine.c#L230) | 保存调用方配置、补齐默认选择回调并建立引擎互斥量。 / Store caller configuration, supply default selection callbacks and establish the engine mutex. `ESP_ERR_INVALID_ARG` `ESP_ERR_NO_MEM` | 独立接口 / Standalone entry |
| `void matrixEngineDeinit(void)` [293](matrixEngine.c#L293) | 释放区域引擎互斥量并清除内部配置。 / Release the region engine mutex and reset its saved configuration. | 独立接口 / Standalone entry |
| `esp_err_t matrixEngineRegionIo(const matrixEngineRegion_t *region, const matrixEngineRequest_t *req, int32_t *outValues, size_t outCount)` [301](matrixEngine.c#L301) | 确认区域与输出容量后按行优先执行，请求读取时每个单元写入一个 int32_t。 / Validate rectangle and capacity and execute in row-major order, writing one int32_t per cell for reads. `ESP_ERR_INVALID_ARG` `ESP_ERR_TIMEOUT` | 独立接口 / Standalone entry |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `cfg` | 该入口声明的配置或策略结构，含所选阈值、时序或回调。 / Configuration or policy of the declared type holding the selected thresholds, timing or callbacks. |
| `col`, `row` | 行或列的坐标。通用 matrixEngineRegion_t 的原点为零；以板级 S/D 位置操作的接口从 1 编号。 / Row or column coordinate. Generic matrixEngineRegion_t origins are zero-based. Board S/D coordinates are one-based. |
| `group` | 分组。 / Group. |
| `muxn`, `muxp` | ADS 正、负模拟输入选择码，请按板级 D 线映射。指针形态用于接收选择码。 / ADS positive/negative input selector from the board D-line map. Pointer forms receive the selector. |
| `outCount` | 接收数量。 / Receives count. |
| `outMohm` | 接收mΩ。 / Receives mΩ. |
| `outRaw` | ADC 原始整数码或 FDC 原始频率码；单位与定点物理值分开。 / Integer ADC code or FDC frequency code, separate from fixed-point physical units. |
| `outUv` | 接收µV。 / Receives μV. |
| `outValues` | 接收结果。要归约或校验的一组测量，数组长度由相邻 count 参数或签名给出。 / Output. Measurements to reduce or validate. The adjacent count or declared array bound gives their extent. |
| `region` | 区域。 / Region. |
| `req` | 请求。 / Req. |
| `source` | 路由或测量的来源标记，区分外部实测、内部推算或对应开关源的枚举。 / Route/measurement source discriminator, such as external, internal estimate or switch source for the declared enum. |
| `userCtx` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `uv` | µV。 / ΜV. |

[项目总览 / Project guide](../../README.md)
