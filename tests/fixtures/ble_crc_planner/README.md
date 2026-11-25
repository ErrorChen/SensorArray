# CRC 与 BLE 分片对照 / CRC and fragment-planner baseline

`baseline-functions.c.txt` 含 provenance.json 标识的四个原始生产函数；`esp-idf-v5.5.5-crc32-le.c.txt` 含 IDF 的软件参考实现及许可证。运行 `python3 tools/test_ble_crc_planner.py`，无需板卡或其它参数。 / The host runner compiles the pinned reference implementations, validates source hashes, and compares old and current CRCs and fragment plans against real encoder bytes. The software routine models the ROM API on the host. Timing on the ESP32-S3 requires a separate measurement.

[生产 BLE 接口 / BLE API](../../../components/sensorarrayBle/README.md) · [科学帧金样 / Encoder goldens](../config_identity/README.md)
