# ADS 故障回放 / ADS fault replays

这些是生产 `sensorarrayAdsFaultFormat` 产生的合成诊断。输入为显式 boot、seq、事务、寄存器和 stage；返回带长度上限的诊断行。调用 `python3 tools/test_ads_fault_fixtures.py` 会编译并比较所有预期字节。只有刻意更新协议金样时才加 `--update` 覆写合成文件。 / The host test supplies synthetic contexts to the production formatter/emitter. It checks pre-cleanup mode, first error, independent cleanup error, attempted/acquired flags and the 384-byte record bound. Use its update flag only to replace the versioned synthetic expectations.

十一个场景覆盖健康读、两个读取不一致、cache、POWER/REFMUX/MODE、RESET、row GPIO、owner、restore、转换 CRC。`reg` 对稳定配置比较使用 cache index，对读数不一致使用物理地址，MATRIX_CRC 的 255 表示无适用寄存器，expected/actual 为 CRC 字节。 / Interpret reg, mask and expected/actual in their stage. These diagnostic records have no science-frame CRC. [格式化器 / Formatter](../../../core/measure/ads/README.md)
