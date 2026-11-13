# Original suite harness / 原套件工具快照

These files preserve the exact source bytes used by `suite-01`, including its
incorrect assumption that MIXED with one active CAP row emits an ordinary C
frame. They are evidence snapshots, not entry points to run in this directory.
To reproduce that version, use their original `tools/` paths in an isolated
checkout. The current tools contain the correction.

这些文件逐字节保留 `suite-01` 使用的工具版本，包括混合单活动 CAP 行被误判为
普通 C 帧的已知错误。它们是证据快照，不应在本目录直接执行；复现旧版本时应在
独立检出中恢复到原 `tools/` 路径。当前工具已修正该假设。

See `../../post-suite-harness-update.json` for before/after hashes, application
time, and executed regressions. Original captures and summaries remain unchanged.
