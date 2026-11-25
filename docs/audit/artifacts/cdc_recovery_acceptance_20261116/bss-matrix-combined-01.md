已核实通过 / Verified passing cases **9 / 32**. 观察器预检失败后通过 / Passed after observer preflight failure **1**. 科学窗口失败的配置 / Cases with a failed scientific window **1**.

| 模式 / Mode | 行 / Rows | 状态 / Status | 采集 / Capture FPS | USB FPS | BLE FPS | USB 实收 / 缺口 (率) · received / missing (%) | BLE 实收 / 缺口 (率) · received / missing (%) | ADC CRC 拒绝 / 检查 · rejected / checked | 无效点 / Invalid points (USB / BLE) |
|---|---:|---|---:|---:|---:|---:|---:|---|---:|
| CAP | 1 | PASS_CASE_SCREEN | 68.073 | 68.076 | 68.074 | 8170 / 0 (0.0000%) | 8169 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 2 | PASS_AFTER_PREFLIGHT_ERROR (bss-matrix-01-cap2 PREFLIGHT_ERROR → bss-matrix-02-cap2 PASS_CASE_SCREEN) | 40.298 | 40.298 | 40.295 | 4836 / 0 (0.0000%) | 4835 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 3 | PASS_CASE_SCREEN | 28.137 | 28.135 | 28.136 | 3376 / 0 (0.0000%) | 3376 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 4 | PASS_CASE_SCREEN | 21.882 | 21.882 | 21.882 | 2626 / 0 (0.0000%) | 2626 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 5 | PASS_CASE_SCREEN | 17.893 | 17.894 | 17.893 | 2147 / 0 (0.0000%) | 2147 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 6 | PASS_CASE_SCREEN | 15.139 | 15.139 | 15.139 | 1816 / 0 (0.0000%) | 1816 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 7 | PASS_CASE_SCREEN | 13.128 | 13.130 | 13.129 | 1576 / 0 (0.0000%) | 1576 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| CAP | 8 | PASS_CASE_SCREEN | 11.577 | 11.578 | 11.578 | 1390 / 0 (0.0000%) | 1390 / 0 (0.0000%) | NOT_OBSERVED | 0 / 0 |
| VOLT | 1 | PASS_CASE_SCREEN | 61.087 | 61.085 | 60.758 | 7331 / 0 (0.0000%) | 7290 / 43 (0.5864%) | 0 / 77984 (0.0000%) | 283 / 279 |
| VOLT | 2 | FAIL | 42.086 | 42.032 | 42.090 | 5043 / 7 (0.1386%) | 5050 / 0 (0.0000%) | 0 / 107060 (0.0000%) | 0 / 0 |
| VOLT | 3 | NOT_STARTED | — | — | — | — | — | — | — / — |
| VOLT | 4 | NOT_STARTED | — | — | — | — | — | — | — / — |
| VOLT | 5 | NOT_STARTED | — | — | — | — | — | — | — / — |
| VOLT | 6 | NOT_STARTED | — | — | — | — | — | — | — / — |
| VOLT | 7 | NOT_STARTED | — | — | — | — | — | — | — / — |
| VOLT | 8 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 1 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 2 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 3 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 4 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 5 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 6 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 7 | NOT_STARTED | — | — | — | — | — | — | — / — |
| RES | 8 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 1 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 2 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 3 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 4 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 5 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 6 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 7 | NOT_STARTED | — | — | — | — | — | — | — / — |
| MIXED | 8 | NOT_STARTED | — | — | — | — | — | — | — / — |

缺口率基于同一接收端观察到的完整帧序列。ADC CRC 使用诊断窗口内的已检查读取次数。破折号表示没有可核实的数据。

Gap rates use complete-frame sequence observations at each receiver. ADC CRC uses checked reads within its diagnostic window. A dash indicates unavailable evidence.

多次尝试时，表内数值取所列目录顺序中的第一个正式窗口。箭头列出尝试顺序，JSON 保留每次结果。每个配置只计一次。

For repeated attempts, the table shows the first formal window in input-directory order. Arrows give the attempt order. The JSON retains each result. Each configuration counts once.
