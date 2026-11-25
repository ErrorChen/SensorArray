# 实录被动夹具片段 / Recorded passive-fixture frames

每个 .recorded.txt 都是生产 encoder 的实收字节。manifest 保存原始 hash、偏移、长度及归档 Git 对象；这些字节和 CRC 必须原样保留。运行 `python3 tools/test_transport_frame_metadata.py` 比较旧/新解析，运行 `python3 tools/test_reference_fixture.py` 验证夹具分析。 / These immutable slices are real receiver input. Consult the manifests for provenance and byte offsets. A slice verifies parser behavior for that record; measuring a complete test window requires the full capture.

| 名称 / Stem | 解释 / Interpretation |
|---|---|
| cap8_total | 总 LC 电容，未扣同点空载。 / Total capacitance including parasitics. |
| res8_first | 该帧的 S8D8 标为 OPEN；保留原始状态用于解码测试。 / Keep the recorded OPEN token at that point. |
| res8_crc_rejected | ADC CRC 拒收使 S1D1 为 X02，外层科学帧 CRC 有效。 / A valid transport frame can contain a rejected ADC sample. |
| crc_partial_volt1…partial / next | 连续两帧显示一个无效点及随后恢复；分别保留 FD、FF 掩码。 / Adjacent frames retain the partial/complete masks and original CRCs. |

[夹具方法 / Fixture method](../../../docs/validation.md) · [记录来源 / Historical provenance](../../../docs/history/README.md)
