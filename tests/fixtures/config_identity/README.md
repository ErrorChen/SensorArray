# 合成协议回放夹具

本目录不是现场实录，不能当作 E1–E7 的硬件复现证据。

`*.frame` 是 `tests/host/test_config_identity.c` 使用真实固件 `sensorarrayTextProtocolBuildFrame()` 输出；`*.config` 使用真实 `sensorarrayConfigIdentityFormat()` 输出。运行 `python3 tools/test_config_identity.py` 会重新生成并独立验证 CRC。原始现场日志没有被改写或补全。

- `cap_before → volt_transition → cap_after → cap_reapplied`：CAP→VOLT→CAP、同一 CAP 新请求的连续配置；C legacy gen 始终来自 ROWS，explicit mgen 递增。
- 丢失 MAPP：只投递 cap_after.frame，然后查询 cap_after.config。禁止将 cap_before.config 的身份盖到新帧。
- 数据早于 MAPP：按上一顺序投递，查询匹配后解除 pending；Applied 不证明 PC 已收到过有效值。
- 旧终态/旧 epoch 迟到：已经确认 cap_reapplied.config 后，再投递 cap_before 或 cap_after 帧/关联请求终态；不能倒退模式状态。
- `health_before_conversion`：合成全帧 acquired=0 的健康前置失败。它并不是“64 次 ADC 转换失败”的实录。
- `*_1`、`*_8`：CAP/VOLT/RES/mixed 的 1/8 行 encoder 黄金样本。
- `res_extreme`、`mixed_extreme`：合法 12 位负 fixed value 和最大整数 metadata，分别 1844、2085 bytes，实际 CRC 正常。不是实际可施加的电压/电阻测试条件。

脚本执行五类状态恢复对照断言，但不替代真实 Host UI 的集成/HIL 测试。
