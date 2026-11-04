# E5 配置身份修复证据

基线 HEAD：8045e9e9ec9599533c52c15dfcb6002f79fd15f1，初始 clean；修复未提交。

| 证据/函数 | 确认的缺陷 | 修改 |
| --- | --- | --- |
| TextProtocolBuildCapFrame 与 BuildMeasurementFrame | CAP legacy gen/rid 从 ROWS snapshot，V/R 从 MODE；跨域比较会误判CAP旧数据 | 原域保持，新字段显式 mgen/mrid/rgen/rrid/pgen/prid + session/bootId/rev |
| MeasurementModeFailTransition / RowModeProfileFailTransition | failed pending ID 被写入 appliedRequestId | applied ID 保留；terminal ID/outcome/error 独立缓存 |
| ScanConfigRequestRows | 新ROWS静默替换旧 pending 请求，ID与MODE独立 | 单配置arbiter、共用allocator、忙时接受前拒绝 |
| CommandMailboxPostMeasurementMode | MODE没有ROWMODES的ACK/终态reservation | MODE/ROWS/ROWMODES共享有界reservation，ACK本地提交后Core1可见 |
| RunOneFrame / FrameBuilderInitInvalid | engine初始化会擦除上层预填metadata | 采样前局部capture，engine返回后恢复；各encoder只读frame |
| MODE?/STATE? | 多个独立snapshot不能拼成原子权威配置 | 新ConfigIdentityCapture/Publish/Format，CONFIG?复制一份受锁保护对象 |
| Mode/Profile generation回绕 | mode原先未跳过0；main候选profile用了直接+1 | mode/profile/候选保持跳0，Host分域serial arithmetic |
| boot breadcrumb | 断电RTC失效可能bootId重新1 | legacy保留，新增随机64bit session，跨session不比较计数 |
| 1536-byte slot | 全域identity使合法极值无法装入，原mixed极值已溢出 | 实测R1844/mixed2085；统一2304，仍拒绝非法超长/不截断 |

本轮新增接口注释标识在 ConfigIdentity 模块、Mode/Profile RejectRequest、ScanConfig allocator/arbiter/callback/stage/rejected、CommandMailbox ConfigAck/Terminal/ROWS。

实际执行（退出码0，2026-11-04）：

- `python3 tools/test_config_identity.py`：C encoder selftest 34 checks；50 MODE transitions；15 real-encoder synthetic frames 与独立zlib CRC（另由现有TextProtocolParser全部解析通过）；5 lost/late/early/reapplied/cross-domain recovery replay assertions。
- `python3 tools/test_rowmodes_ack_guard.py`：真实Mailbox/ScanConfig/GuaranteedText，MODE/ROWS/ROWMODES排他仲裁、ACK前不可见、ACK失败取消、满队列fallback、重复terminal拒绝、ROWS终态缓存。
- `python3 tools/test_main_config_commit.py`：从当前main抽取生产MODE commit/terminal/publish与mixed失败分支，真实Mode/Profile/Config代码加硬件stub；7场景覆盖snapshot/preAccept失败无MAPP、Apply失败appliedId保持、成功同rev、mixed RMERR最终DEGRADED权威状态。
- `python3 tools/test_rowmodes_frame_metadata.py`、`python3 tools/test_frame_semantics.py`、`python3 tools/test_fdc_iso_mailbox.py`：PASS。
- `CC=/usr/bin/gcc python3 tools/test_measurement_logic.py`：PASS。无CC覆盖的原命令因脚本未发现clang失败（不是C测试失败），指定实际gcc后执行成功。
- `python3 tools/check_command_docs.py`：PASS，96 commands；`git diff --check`：PASS。

新增黄金夹具均是合成输入，真实encoder/formatter输出，非现场原始日志。README说明回放顺序。无硬件HIL、Windows、蓝牙负载、2分钟实采：NOT_RUN。

Host sizeof输出（非Xtensa stack实测）：frame=4824、identity=72、authority=120、packet=2312 bytes。文本slot每份较1536旧上限增加768 bytes；具体实例总占用与target `.bss`/heap/stack 由根构建size报告记录。没有据此声称目标CPU/FPS/heap最小/stack水位已通过。

锁顺序：Mailbox短临界区 → ScanConfig短临界区（部分分配路径），无反向嵌套；配置callback指针在ScanConfig锁内读出后在锁外调用；ConfigAuthority锁仅复制/比较小snapshot，不调用Mailbox、硬件、printf或网络。mode/profile seqlock的writer是Core1采样owner，初始化在其启动前；版本号不被当作多writer互斥。

协议交接见 `docs/integration/firmware_host_contract.md`。原MAPP、RAPP、RMAPP不会因Host错读legacy域而被“统一数值”；软件仓库必须实现新协商/一致查询与2304-byte重组缓冲。
