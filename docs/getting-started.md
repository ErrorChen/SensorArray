# 快速入门

## 环境要求

- ESP-IDF `v5.5.1`；
- ESP32-S3 toolchain；
- 支持 USB Serial/JTAG 的数据线；
- Python 3 与仓库虚拟环境，用于 host 测试和 HIL；
- BLE HIL 需要主机 BLE adapter；Serial HIL 需要设备串口未被其他程序占用。

建议从 ESP-IDF 5.5.1 自带终端启动 PowerShell，避免混用其他 IDF/toolchain。

## 获取源码

```powershell
git clone https://github.com/ErrorChen/SensorArray.git
Set-Location SensorArray
git status --short
idf.py --version
```

需要复现固定版本时，显式 checkout 目标 commit，并再次检查 `git status`。

## 编译

```powershell
idf.py set-target esp32s3
idf.py fullclean
idf.py build
```

`sdkconfig.defaults` 和 `sdkconfig.defaults.esp32s3` 是项目默认配置；`sdkconfig` 与 `build/config/sdkconfig.h` 才是本次 build 的最终有效配置。定位 task stack 或 feature 开关时必须检查后两者，不能只看 defaults。

## 烧录与监视

```powershell
idf.py -p COMx flash
idf.py -p COMx monitor
```

如果没有明确端口，先列出候选：

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Name,PNPDeviceID
```

不要在多个相同设备同时连接时自动挑选端口。

## 第一次串口检查

设备完成 boot sweep 并出现第一帧完整、CRC 正确的 CAP 数据后，再依次发送：

```text
MODE?
STATE?
TX?
ST?
BTX?
WIFI?
ROWS?
ROWMODES?
ROWMODES=CVVRRVVC
```

随后验证模式切换：

```text
MODE=CAP
MODE=RES
RAILCFG=<AVDD_UV>,<negative_AVSS_UV>   # optional external-DMM override
MODE=VOLT
MODE=CAP
```

普通 `MODE=VOLT` 会先在矩阵隔离的 `SAFE_RAIL_MONITOR` 窗口通过 ADS126x internal analog supply monitor 自动取得 rail span，不需要先发 `RAILCFG`。`RAILCFG` 只用于可选的外部 DMM debug override，且必须在非-VOLT route 下发送。

使用外部 DMM override 时顺序为：

1. 在 CAP 或 RES 下发送 `RAILCFG`；
2. 等待匹配 request ID 的 `RACK ... source=external,state=accepted`；
3. 等待 `RAPP ... source=external,state=applied`；
4. 再发送 `MODE=VOLT`；
5. 等待匹配的 `MACK`、`MAPP` 和首个完整 VOLT frame。

如果已经在 VOLT 内发送 `RAILCFG`，firmware 会拒绝并返回 `ERR,cmd=RAILCFG,reason=apply_before_volt`。电池断开时 AIN8 的浮动、长周期 PWM 或异常值会报告 `valid=0` 和原因，同时保留 `lastGood*`；不会把矩阵采集判为故障。

每次切换都必须看到：

1. `MACK`：请求已进入 mailbox；
2. `MAPP`：Core 1 已在完整帧边界应用；
3. 新一代数据帧中的 `mode/gen/rid` 与 `MAPP` 匹配。

完整命令见 [命令参考](command-reference.md)。

## 第一次 BLE 检查

1. 扫描 `CscArray_*`；
2. 连接 Service `0x00FF`，客户端支持时请求 MTU 247；
3. 为 `FF11`、`FF20`、`FF30` 启用 Notify 与 Indicate（CCCD `0x0003`）；
4. 向 `FF10` 写 `STATE?\n`；
5. 确认 `FF11` 返回 `MODE,...`，`FF20` 有 DATA，`FF30` 有 LOG；
6. 写 `BTX=SAFE\n`，确认 `FF11` 以 Indicate（Linux/BlueZ fallback 时为 Notify）返回 ACK，再写 `BTX?\n` 核对；
7. disable `FF30`，确认 `FF20` 继续 DATA，再 re-enable `FF30`；
8. 检查整个过程串口没有 panic/reset。

推荐为 FF11、FF20、FF30 开启 Indicate bit（CCCD `0x0003`），这样 SAFE 使用带确认的 Indicate。Ubuntu/BlueZ 的 Bleak 无法在同时支持 Notify/Indicate 的 characteristic 上选择 Indicate；此时 SAFE 会有界降级为 Notify，不会阻塞 ACK、DATA 或 LOG。`0x0003` 仍是连接中往返 FAST/SAFE 的最稳妥设置。

Windows/Bleak 通常把 `FF10` 显示为 Bluetooth Base UUID：

```text
0000ff10-0000-1000-8000-00805f9b34fb
```

详见 [BLE 协议](ble-protocol.md) 与 [验证](validation.md)。

## 常见下一步

- 了解矩阵与电池接线：[硬件概览](hardware-overview.md)
- 理解 CAP/VOLT/RES 路由：[测量模式](measurement-modes.md)
- 实现上位机 parser：[输出格式](output-format.md)
- 排查断连、CRC、重启或无效值：[故障排查](troubleshooting.md)
