# RES 模型、接口与本次限制 / RES model, interfaces and limits

本次只对已闭合的 RES8 原始记录进行离线归一化交叉检查。`10000 Ω` 是用户给定标称值，不是新增测得的标准值；不写固件或 NVS，不推算 R38/R45、外接电阻、开关或接触电阻的实际值。

The analysis uses recorded output only. A nominal 10000 Ω is an explicit assumption, not a traceable measurement. No unmeasured component value is inferred and no device calibration is applied.

## 当前数学路径 / Current computation

[sensorarrayAdsMathResistanceDivider](/home/csc/esp/workspace/SensorArray/core/measure/ads/sensorarrayAdsMath.c:91) implements:

```text
R_reported = Rref_config × (Vexc − Vnode) / (Vnode − AVSS) − Opath
Opath = globalOffset + bankOffset[D1–D4 or D5–D8] + enabledCellOffset
```

计算使用 checked int64，输出整数 mΩ，除法向零截断；OPEN/SHORT/NEGATIVE/RANGE/OVERFLOW 与采样 invalid 分开处理。该 1 mΩ 输出步距不是 1 mΩ 测量精度。

[sensorarrayAdsMatrixEngineReadFrame](/home/csc/esp/workspace/SensorArray/core/measure/ads/sensorarrayAdsMatrix.c:1725) 设置 `Vexc = AVSS + matrixReferenceSpanUv`；RES 的实际调用在 [divider 调用处](/home/csc/esp/workspace/SensorArray/core/measure/ads/sensorarrayAdsMatrix.c:2059)。`Vnode` 由 ADC 新鲜读数换成差分电压并加估计 AINCOM；外部轨配置的 AINCOM 仍按两轨中点估计，见 [sensorarrayAdsGapCopyRailSplit](/home/csc/esp/workspace/SensorArray/core/measure/ads/sensorarrayAdsGap.c:1638)。不能把 R 帧的 `ref=INTREF` 当成“电源轨由内部实测得到”，也不能把 `vexc` 当成相对 AVSS 的 span。

当前 [sdkconfig](/home/csc/esp/workspace/SensorArray/sdkconfig:722) 的 RREF 为10000 Ω、matrix REFOUT span为2500000 µV、全局路径 offset为0；引擎初始化未指定的 bank/cell offsets 为0。这些配置名虽称 calibrated，也不构成对实装器件的计量证明。逐帧 `rref`、`vexc`、`avdd`、`avss` 与实际 CAL/ARL 查询均另保存在 results.json。

At fixed excitation and rails, the divider is nonlinear in the measured node voltage:

```text
∂R/∂Vnode = −Rref × (Vexc − AVSS) / (Vnode − AVSS)²
```

因此节点电压 offset、AINCOM 中点估计、rail span、参考误差或建立误差不普遍等价于一个阻值乘法系数。纯 RREF 比例误差在理想模型中可以表现为乘法项，但本次未隔离此单一误差，也未证明全部列共享同一偏差；路径 offset 还会破坏简单比例关系。相同标称值的两只电阻无法同时识别增益和 offset，且第二只只覆盖另一条路径在同一阻值附近的表现。

The single fitted factor can check agreement at this nominal value. It cannot identify a general gain/offset model, correct all resistance ranges, or justify multiplying VOLT readings.

## 两条物理路径 / Two physical paths

现有 V1.0 原理图图像已实际查看：[ADC/参考电阻页](/home/csc/esp/workspace/SensorArray/datasheets/circuit_page3-3.png)、[矩阵开关页](/home/csc/esp/workspace/SensorArray/datasheets/circuit_page4-4.png)。原理图名义连接为：

- **S1D1**：U5/TMUX1108 的 S1 行 → 外接夹具电阻 → D1 → U6/TMUX1134 → R1/ADS AIN0；该 ADC 侧节点由 **R38，标称10 kΩ** 接向负轨。
- **S8D8**：U5 的 S8 行 → 另一只外接夹具电阻 → D8 → U7/TMUX1134 → R8/ADS AIN7；其 ADC 侧节点由 **R45，标称10 kΩ** 接向负轨。

板上 R38/R45 是两只不同的分压参考元件，不能与两只外接被测10 kΩ混为一物。图纸不能证明当前实装值、容差、温度、改版或接触状态；没有为外接夹具电阻强行指定图纸上的器件编号。

Two distinct row contacts, switch channels/banks, reference resistors, leads and contacts contribute. Switch on-resistance, leakage, ADC input loading and settling can depend on node voltage, path and temperature. No typical datasheet switch resistance is inserted as a precision correction. Other fixture elements and unpopulated matrix cells do not constitute an independently controlled voltage stimulus.

## 真实校准接口 / Actual calibration interfaces

- [sensorarrayAdsMatrixCalibration_t](/home/csc/esp/workspace/SensorArray/core/measure/ads/sensorarrayAdsMatrix.h:15) 支持版本、共同 RREF、matrix reference span、global/bank/per-cell offsets；[SetCalibration](/home/csc/esp/workspace/SensorArray/core/measure/ads/sensorarrayAdsMatrix.c:1625) 只允许 Core1 owner 在 NONE/SAFE 状态应用并使缓存失效。当前结构没有每列独立 RREF 参数，也没有任意电压全局增益字段。
- 当前 [main 的 CAL=SAVE/LOAD](/home/csc/esp/workspace/SensorArray/main/main.c:1655) 与 [启动加载](/home/csc/esp/workspace/SensorArray/main/main.c:3740) 已接入 [sensorarrayCalibrationSave/Load](/home/csc/esp/workspace/SensorArray/core/config/sensorarrayCalibration.c:214)。持久记录包含 schema、boardId、hardwareRevision、payload 和 CRC32。board/hardware 标识是固定配置常量，不是自动测得的实装版本。
- `CAL=ZERO/RAIL/ALL` 请求辅助零点/rail 操作；它们不会用夹具两个标称10 kΩ自动求解这个比例系数。`RAILCFG` 外部轨值属于另一条 RAM 校准路径，本次未发送任何此类命令。
- `core/measure/ads/README.md` 关于“尚未实现持久化”的历史文字与当前源码不一致；本次以实际源码和实录 CAL 查询为准，未改该文档。

## 不确定度与验收 / Uncertainty and acceptance

1. 外接两只电阻只有标称值；独立实测值、容差等级、温漂、仪表溯源与本次温度未知。不能因 S1D1 被拟合到10000就宣布它实际等于10000，也不能把 S8D8 的残差全部归为仪器误差。
2. R38/R45 实际值及各路径 Ron、连接/接触电阻未单独量过；当前记录只能检查整体路径在约10 kΩ处的一致性。
3. 外部 rail 输入是用户在 ADS 供电脚/近端去耦处报告的 AVDD 3.33–3.37 V、AVSS −1.795 V ±0.005 V，固件采用代表值3.350 V/−1.795 V。其覆盖区间不是已知概率分布或本窗同步波形，ARL 的 `uncertaintyUv=unknown` 保留。测点已包含供电路径当时压降，不能再臆测扣除 R54。
4. REFOUT span、ADC gain/offset、估计 AINCOM、PGA与建立过程也参与数学转换；本窗没有足以分离这些项的多个已知阻值、同点短路/开路精密基准或独立原始节点电压对照。
5. 统计 min/max、P05/P95 和标准差是重复观察分布，不是绝对误差界。固定系数不消除未知共同系统误差。验收限缺失，最终状态保持 **FIXTURE_SINGLE_VALUE_NORMALIZATION_CROSS_CHECK_ONLY / ABSOLUTE_ACCURACY_NOT_ESTABLISHED**。

Both raw and normalized results remain available. S8D8 is held out of fitting, but it is a second channel at the same nominal resistance, not an independent resistance-range validation. Traceable references and explicit acceptance limits are needed before an absolute calibration acceptance decision.
