# 帧元数据解析基准 / Transport metadata baseline

baseline.c.txt 是由 baseline.json 中的提交和 SHA-256 标识的生产解析器。运行 `python3 tools/test_transport_frame_metadata.py`，host wrapper 用不同符号名同时编译基准和当前模块，比较完整 payload slot。输入包括重复字段、缺少身份、整数边界、真实金样及实录。 / The differential runner compiles both parsers and compares all fields, including bytes retained when hasFrameMeta is false. The oracle covers first nonempty duplicates, optional invalid fields and numeric parsing. hasFrameMeta alone makes no claim about the scientific CRC.

[金样 / Goldens](../config_identity/README.md) · [实录 / Recordings](../reference_capture/README.md)
