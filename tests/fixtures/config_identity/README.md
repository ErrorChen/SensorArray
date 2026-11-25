# 帧与配置金样 / Frame and configuration goldens

`tests/host/test_config_identity.c` 用真正的 `sensorarrayTextProtocolBuildFrame` 生成 .frame，用 `sensorarrayConfigIdentityFormat` 生成 .config。运行 `python3 tools/test_config_identity.py` 会在临时目录生成并核对字节及 CRC。 / These are synthetic inputs passed through the production C encoders. Run the host wrapper from the repository root. Configuration text and measurement frames have distinct formats. 仅在有意修改协议金样时使用 `--update`。 Use `--update` to explicitly replace the synthetic expectations after validation.

| 输入 / Input | 应检验的行为 / Expected behavior |
|---|---|
| cap_before, volt_transition, cap_after, cap_reapplied | CAP→VOLT→CAP 及重新应用 CAP。C 的 legacy gen 来自行数域，mgen 跟踪模式。 / Recover a lost or late MAPP through CONFIG. Reject a delayed old terminal without relabeling pending frames. |
| health_before_conversion | 前置健康故障，acquired=0。 / No point conversion was completed. |
| *_1, *_8 | 四种布局的首行/全八行；比较 header、K 与 snapshot。 / Verify row and mode identities across all layouts. |
| res_extreme, mixed_extreme | 极值与最大宽度 metadata 的消息尺寸及实际 CRC。 / Check size bounds and real CRC at the encoding extremes. |

[主机契约 / Host contract](../../../docs/integration/firmware_host_contract.md)
