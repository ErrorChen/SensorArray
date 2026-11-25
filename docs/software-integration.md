# 上位机接入 / Host integration

1. 枚举真实接口，记录 PROTO、BUILD；查询 CONFIG 保存 boot/session/revision 和各配置域。 / Discover the actual transport and query protocol, image and atomic configuration identity.
2. 原样保留接收字节和单调主机时间。重组 BLE 分片，检查长度、外层 CRC 和科学帧 CRC。 / Retain original bytes and monotonic receipt times. Validate both the message envelope and scientific frame.
3. 按帧自身身份解析 row-major 数据、有效位及单位；按域关联命令的 ACK、终态与首帧。终态缺失时查询 CONFIG。 / Decode row order, units and masks from the frame. Match acknowledgements and terminals within their request domains and recover state through CONFIG.
4. 把采集、发送和实收分层展示；展示 unknown、OPEN 和 SETTLING。 / Show independent acquisition and delivery rates, invalid reasons and unknown values.

[完整字段契约 / Field contract](integration/firmware_host_contract.md) · [帧实例与 parser / Samples and parser](../tests/fixtures/config_identity/README.md) · [Python 工具 / Python tools](../tools/README.md)
