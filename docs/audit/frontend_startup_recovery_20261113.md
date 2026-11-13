# 前端启动选速率与掉电恢复（2026-11-13）

本轮继续基于 `7e1d2c6dd9c85883ce9018fbc89a5d2402410607`，不改上位机仓库。用户新增要求是启动时 SPI 与 I²C 并行扫描、细化 SPI 档位、扫描容许 2.5% CRC 错误，以及三颗前端失联后低频探测、恢复后重新启动并扫描。先前电阻夹具/CRC 失败实录仍保留在 [11月13日复测](reference_fixture_recheck_20261113.md)，不能由本次新增启动测试覆盖。

## 代码职责

- `core/board/sensorarrayFrontendStartup.[ch]`（本轮新增）：承接从 `main/main.c` 移出的原 FDC ID/负载扫描、选频、初始化和 SPI 并行任务。当前双 FDC 的初始化仍由采集核依次执行，SPI 在另一个核并行；正常逐行双 FDC 并行采集保留。
- `core/board/sensorarrayAdsSpiProbe.[ch]`（本轮新增）：只做独占的 ADS 链路资格测试和完整配置恢复，不生成科学测量帧。启动和前端恢复后的新 boot 共用此实现。
- `core/board/sensorarrayFrontendPresence.[ch]`（本轮新增）：低频三芯片通信证据及等待/恢复状态机。查询只复制已发布状态，不在命令回调中访问总线。
- ADS 驱动的统一持久 SPI 描述符应用频率覆盖；完成未知仍 quarantine，不能改频、释放或复用 DMA 对象。`sensorarrayRouteControllerEnterFrontendWait`（本轮新增）只执行等待期 GPIO 动作并将路由标为 UNKNOWN。

## SPI 资格标准

候选从配置上限向下：2 MHz 至 1 MHz 每档 100 kHz，950 kHz 至 150 kHz 每档 50 kHz，最后 125 kHz。请求频率经过 ESP32-S3 时钟分频量化；日志分别保留 requestedHz 与 IDF 返回的 actualHz，后者不是示波器测量值。实际完成事务后核验频率，避免把驱动拒绝覆盖后继续使用的旧频率误认为新档位。

默认每个候选初筛 400 次、独立确认 1600 次，均平分为 PGA gain1 和 bypass。每个 profile/阶段各自允许最多 2.5% CRC 错误：200 次中的 5 次、800 次中的 20 次；超过后才按 CRC 条件拒绝该档。坏 CRC 只计入分母和错误数，不计有效样本、CRC 奇偶覆盖、原始值范围或测量缓存。该门限仅用于启动选速率，生产测量的 CRC 校验与无效点语义不变。寄存器不符、芯片复位、STATUS 异常、饱和或事务完成未知仍是独立失败条件。

所有档位共用默认 30 秒单调期限，含任务调度、初筛、确认和恢复，预留 500 ms 收尾。期限用尽就报告失败，不跳过未测试档位，也不把未验证的最低速度作为成功结果。这个 SPI 期限不包含原有 I²C 启动流程的总耗时。日志中的累计 CRC 包含被淘汰候选；判断选中档位须使用对应 profile/phase 的结果。

测试把 AINCOM 同时接 ADC 两个差分输入，保持 INTREF OFF、REFMUX=AVDD/AVSS，包含偏置建立等待和新 DRDY 判据。不对外部输入作高低阻负载试探，不把短接输入的链路稳定性当成 RES/VOLT 精度或长期无错保证。测试后恢复真实寄存器及 driver shadow；所有后续矩阵/route/gap 初始化必须等待资格结果和恢复结果完成。

## 失联与恢复边界

两颗 FDC 的 Manufacturer/Device ID 与 ADS 的稳定 ID/核心寄存器响应是通信依据；`ready=false`、CRC 错误或 FDC 被主动 SD shutdown 都不能单独证明三颗芯片失联。探测不读取会清中断的 FDC STATUS。事务忙或完成未知时不得另起探测覆盖原 owner。

恢复采用受现有预算约束的 ESP 重启，重新调用同一个启动模块，而不是在旧 worker、DRDY ISR、DMA 和配置缓存上原地重建。新 session/boot 的帧计数和配置身份必须重新查询；RAM 中的临时 RAILCFG 会丢失，不能静默沿用为新电源周期的校准。没有写 NVS 或执行 CAL=SAVE。

当前板 U5 没有已证实的 REF 隔离能力。ADS 重新上电默认 INTREF ON；等待期间释放 Q1 是为了避免主动钳住返回的参考，禁用可控 TMUX1134 也不等于隔离 U5。失联识别存在延迟，软件不能消除上电间隙的所有电气风险，路由只可报告 UNKNOWN/physicalSafe=unknown。实机测试仍限用户的被动 10 kΩ/33 pF 夹具；真实供电切换及边沿需分别记录，不把 host 模拟当作物理断电验证。

## 验证记录

构建、全部 host 回归、烧录、启动实录和运行期采集结果将记录于 [本轮证据目录](artifacts/boot_autotune_20261113/)。每次命令的退出码、HEAD、dirty diff hash、包括新文件的逐源码 hash 独立保存。编译通过不等于 HIL 通过，有限启动 CRC 比例也不等于长期错误概率上限。
