# 构建与首次连接 / Build and connect

在 ESP-IDF 5.5.5 环境中，从仓库根目录构建 ESP32-S3 工程。主工程在 `main/`，默认配置来自两个 sdkconfig.defaults 文件，本机最终生效值在 sdkconfig 和构建目录的 config/sdkconfig.h。自定义分区表和默认 Flash 容量见根目录。 / Use the ESP-IDF 5.5.5 environment and the repository root. The two defaults files seed configuration. Inspect the effective sdkconfig, generated header and partition table when reproducing a build.

```sh
idf.py --version
git rev-parse HEAD
git status --short
idf.py -D IDF_TARGET=esp32s3 build
idf.py size
```

## 找到对应接口 / Identify the interface

正式镜像把同一 USB 线上的两个 CDC 角色分开。Linux 的 formal 接口使用 VID:PID 303a:4042、interface 0 与设备序列号定位，interface 2 为 debug。tty 编号、Windows COM 名称会随模式和枚举改变。ROM 下载和恢复态 Serial/JTAG 有独立身份。先记录已连接设备的实际角色。 / The production image exposes separate data/control and debug CDC roles. Match the USB serial number, VID/PID and interface number. Application, recovery and ROM download may enumerate differently. Inspect the OS device list before opening a port.

```sh
python3 -m serial.tools.list_ports -v
python3 tools/receive_serial_text.py --help
python3 tools/receive_ble_text.py --help
```

进入已授权的烧录流程时，使用实际 ROM 下载端口执行 `idf.py -p PORT flash`。在应用接口发送 `USBBOOT` 是会重启的操作，须等待 ROM 的实际枚举再执行下载。在日常连接中，从 formal CDC 发查询，或订阅 BLE FF11/FF20，在 FF10 写入带 LF 的 ASCII 命令。 / Flash only the identified download port. USBBOOT changes device state and requires fresh ROM enumeration. To use the running application, issue queries on formal CDC or subscribe to BLE control/data and write commands to FF10.

```text
PROTO?
BUILD?
CONFIG?
STATE?
RAIL?
```

从 CONFIG 保存会话、boot、revision 和三组 gen/rid。进入 VOLT 有两种合法供轨路径：推荐在非 VOLT 状态下通过 `RAILCFG=<avdd_uv>,<avss_uv>` 写入该板 ADS 附近实测的 AVDD/AVSS（单位为带符号 µV）并核对 `RAPP`（冷启动免跑 RES 且精度最高）；或先运行一次 `MODE=RES` 采集内部 `monitor` 轨（随后 VOLT / 被动 C+V 混合模式 / `RAIL?` / `BAT` 会一直沿用该有效轨，不再因超过 100 帧报错 `0x106`）。RAM 中的外部测量值需在复位后重新确认。对每次配置请求分别确认接受、终态及首个目标配置完整帧。 / Save the configuration identity. Prepare VOLT rails outside VOLT either by applying signed microvolt DMM measurements with `RAILCFG=<avdd_uv>,<avss_uv>` and confirming `RAPP` (recommended for cold boot and highest accuracy), or by running `MODE=RES` once so VOLT, passive C+V mixed profiles, `RAIL?`, and `BAT` retain that valid monitor rail without 100-frame expiry. Re-confirm volatile RAM rail values after a reset. After each setter correlate the local acknowledgement, applied result and first complete frame independently.

[命令参数 / Command reference](command-reference.md) · [主机契约 / Host contract](integration/firmware_host_contract.md) · [故障排查 / Troubleshooting](troubleshooting.md)
