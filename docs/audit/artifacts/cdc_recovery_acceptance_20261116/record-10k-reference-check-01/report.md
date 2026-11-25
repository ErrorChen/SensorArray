# 10 kΩ 单值归一化交叉检查 / Single-value fixture normalization

结论：仅对当前夹具做单值归一化交叉检查，绝对精度及全局增益校准均未建立。 / This is a one-value fixture cross-check; absolute accuracy and a global gain calibration are not established.

1007 个原始 RES8 帧，seq 83401–84407，两点各 1007 个有效值。原始 raw SHA256 `fe3c5bc360284ecc5b0658afbd61068f15ea7790e99135d942644d3263e0ca4b`；ELF `892f586bdb131e77614c1ca72723c9247159f9d3a7bf5b49917004383f252677`。
预定 S1D1 单独拟合：k = 10000 / mean(S1D1) = **0.983770502386**，精确分数 `5035000000/5118063601`。S8D8未进入拟合；之后只乘此固定k。S1D1归一化均值10000是构造结果，不是验证通过。

| 测点 / Point | 角色 / Role | 有效 n | 均值 Ω / Mean | min–max Ω | P05–P95 Ω | 均值相对标称误差 / Mean error |
|---|---|---:|---:|---|---|---|
| S1D1 | 原始 / original | 1007 | 10164.972395 | 10159.550000–10171.083000 | 10162.038000–10167.986200 | +164.972395 Ω / 1.649724% |
| S1D1 | 归一化 / normalized | 1007 | 10000.000000 | 9994.665608–10006.011433 | 9997.113229–10002.964892 | +0.000000 Ω / 0.000000% |
| S8D8 | 原始 / original | 1007 | 10166.389779 | 10160.526000–10171.652000 | 10163.794000–10168.978200 | +166.389779 Ω / 1.663898% |
| S8D8 | 归一化 / normalized | 1007 | 10001.394380 | 9995.625768–10006.571198 | 9998.840730–10003.940793 | +1.394380 Ω / 0.013944% |

同帧配对比值 S8D8/S1D1：n=1007，均值 1.000139464229，范围 0.999401786419–1.000901288090，P05–P95 0.999769547050–1.000498055619。原始配对差均值 +1.417383 Ω。比值只是诊断，未用于逐帧重标定。

原值为整数mΩ；归一化值是单列派生量，原包/CRC/invalid未改。min/max与P05/P95仅描述观察分布，不是参考不确定度、精度界或置信区间。两个标称10000Ω不等于已知实际值；参考实测值、容差、温度与允许误差限仍未知，故不标绝对精度PASS。

RES物理公式及接口见 physical-model.md。电阻、开关/接触、ADC/参考/rail误差不能由单个标称值分开识别。不同列具有独立板上参考电阻；本系数不应用到其它阻值、其它点或VOLT，也不推算任何未量器件阻值。

The coefficient is fitted only to S1D1's arithmetic mean, then frozen before evaluating S8D8. At the assumed 10000 Ω nominal, S8D8's normalized mean is 10001.394380 Ω, a conditional deviation of +1.394380 Ω. This tests consistency between two paths at one nominal value; it does not identify a transferable gain, offset, actual reference component value, or voltage correction. No firmware, NVS, calibration command or raw scientific data was changed.

复算 / Reproduction: execute `python3 analyse.py` in a fresh copy of this directory containing only the script, analysis-plan.json and physical-model.md, or inspect the recorded invocation. Existing outputs are never overwritten. The manifest binds source selection, original byte ranges, parser, production-source and schematic hashes.

