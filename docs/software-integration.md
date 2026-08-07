# 上位机兼容清单

## 必须支持

1. 通过 Serial、BLE `FF10` 或 Wi-Fi CTRL 发送同一组命令；
2. BLE 通过 `FF11` 接 CTRL、`FF20` 接 DATA、`FF30` 接 LOG；
3. 把 `MACK/RCMD/RACK` 视为 accepted，把 `MAPP/RAPP` 视为 applied；
4. mode/rows 切换后按 `gen/rid` 丢弃旧数据；
5. 解析 CAP `C/D/K` 与 VOLT/RES `V|R/D/P/K`；
6. 按动态 `rows/cells/n` 处理最后不足 16 个值的 chunk；
7. 验证 frame CRC；
8. BLE 还要先验证 envelope length/CRC；
9. 保留 `Xhh` 未知错误码，不能转换为 0；
10. 把 PGA `00` 显示为 verified bypass；
11. 明确使用 `unit/scale`，不能从数值范围猜测 pF/V/Ω；
12. battery UI 同时显示 `valid/fresh/age/reason/restore`，不能只显示 mV 或推导 SOC。

## 无 breaking change 的部分

- BLE Service/characteristic UUID；
- BLE `G,...` fragmentation envelope；
- CAP `C/D/K` 基本 wire format；
- UDP DATA/LOG/CTRL ports；
- shared command handler；
- non-blocking sink policy。

## 参考实现

- `tools/text_protocol.py`：CAP/V/R parser、CRC 与 BLE fragment reassembler；
- `tools/test_text_protocol.py`：host protocol tests；
- `tools/sensorarray_hil.py`：Serial/BLE HIL。

完整规格见 [命令参考](command-reference.md)、[BLE 协议](ble-protocol.md) 和 [输出格式](output-format.md)。
