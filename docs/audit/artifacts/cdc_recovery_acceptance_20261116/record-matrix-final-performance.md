# 已闭合矩阵性能 / Closed matrix performance

已闭合且可核对的性能窗口 / Verified performance windows **32**. PASS_CASE_SCREEN 32.

ELF `892f586bdb131e77614c1ca72723c9247159f9d3a7bf5b49917004383f252677`, session `0119031D7DEF4971`, boot `1`.

| 工况 / Attempt | 判定 / Gate | 主机窗 / Host s | Capture FPS | Emit FPS · SF50 median [min–max] | USB DATA 提交 / submits (records/s) | BLE DATA 本地发送 / sentD (messages/s) | BLE SDK 接受 / accepted (calls/s) | USB 实收 / host frames @ FPS | BLE 实收 / host frames @ FPS |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| record-matrix-01-cap1 | PASS_CASE_SCREEN | 120.000170 | 68.206 | 68.10 [67.32–69.42] | 8,195 (68.193) | 8,100 (68.208) | 8,196 (68.193) | 8,185 @ 68.207 | 8,185 @ 68.211 |
| record-matrix-01-cap2 | PASS_CASE_SCREEN | 120.000090 | 40.243 | 40.28 [39.83–40.52] | 4,833 (40.235) | 4,800 (40.241) | 4,834 (40.238) | 4,829 @ 40.243 | 4,829 @ 40.243 |
| record-matrix-01-cap3 | PASS_CASE_SCREEN | 120.000091 | 28.100 | 28.07 [27.93–28.29] | 3,375 (28.097) | 3,350 (28.102) | 6,750 (56.193) | 3,372 @ 28.101 | 3,373 @ 28.102 |
| record-matrix-01-cap4 | PASS_CASE_SCREEN | 120.000150 | 21.867 | 21.87 [21.73–21.97] | 2,626 (21.862) | 2,600 (21.869) | 5,252 (43.727) | 2,624 @ 21.868 | 2,624 @ 21.869 |
| record-matrix-01-cap5 | PASS_CASE_SCREEN | 120.000139 | 17.888 | 17.88 [17.83–17.95] | 2,149 (17.889) | 2,100 (17.889) | 4,296 (35.766) | 2,146 @ 17.888 | 2,146 @ 17.888 |
| record-matrix-01-cap6 | PASS_CASE_SCREEN | 120.000133 | 15.137 | 15.14 [15.09–15.17] | 1,819 (15.140) | 1,800 (15.138) | 3,638 (30.280) | 1,817 @ 15.138 | 1,817 @ 15.138 |
| record-matrix-01-cap7 | PASS_CASE_SCREEN | 120.000143 | 13.121 | 13.11 [13.08–13.16] | 1,576 (13.120) | 1,550 (13.121) | 3,152 (26.236) | 1,574 @ 13.122 | 1,574 @ 13.122 |
| record-matrix-01-cap8 | PASS_CASE_SCREEN | 122.000212 | 11.575 | 11.57 [11.54–11.59] | 1,413 (11.573) | 1,350 (11.577) | 2,826 (23.145) | 1,413 @ 11.577 | 1,413 @ 11.576 |
| record-matrix-01-volt1 | PASS_CASE_SCREEN | 120.000083 | 61.673 | 61.51 [59.14–66.01] | 7,431 (61.679) | 7,297 (61.242) | 14,747 (122.379) | 7,402 @ 61.681 | 7,347 @ 61.230 |
| record-matrix-01-volt2 | PASS_CASE_SCREEN | 120.000139 | 43.094 | 42.98 [41.26–44.09] | 5,183 (43.082) | 5,100 (43.100) | 10,368 (86.179) | 5,171 @ 43.098 | 5,171 @ 43.094 |
| record-matrix-01-volt3 | PASS_CASE_SCREEN | 120.000137 | 33.713 | 33.73 [32.57–34.97] | 4,053 (33.709) | 4,000 (33.706) | 8,106 (67.418) | 4,046 @ 33.715 | 4,045 @ 33.715 |
| record-matrix-01-volt4 | PASS_CASE_SCREEN | 120.000172 | 27.695 | 27.80 [26.88–28.98] | 3,327 (27.680) | 3,300 (27.698) | 6,654 (55.359) | 3,322 @ 27.695 | 3,322 @ 27.694 |
| record-matrix-01-volt5 | PASS_CASE_SCREEN | 120.000133 | 22.203 | 22.27 [21.21–22.86] | 2,667 (22.197) | 2,600 (22.199) | 8,001 (66.592) | 2,665 @ 22.206 | 2,664 @ 22.207 |
| record-matrix-01-volt6 | PASS_CASE_SCREEN | 120.000133 | 19.165 | 19.21 [18.47–20.04] | 2,301 (19.151) | 2,250 (19.183) | 6,903 (57.452) | 2,300 @ 19.178 | 2,300 @ 19.177 |
| record-matrix-01-volt7 | PASS_CASE_SCREEN | 120.000118 | 16.794 | 16.84 [16.40–17.17] | 2,018 (16.792) | 1,950 (16.775) | 6,054 (50.375) | 2,016 @ 16.797 | 2,016 @ 16.796 |
| record-matrix-01-volt8 | PASS_CASE_SCREEN | 120.000137 | 14.793 | 14.68 [14.18–15.40] | 1,777 (14.792) | 1,700 (14.776) | 5,331 (44.373) | 1,775 @ 14.793 | 1,775 @ 14.794 |
| record-matrix-01-res1 | PASS_CASE_SCREEN | 120.000137 | 46.054 | 46.02 [44.93–46.53] | 5,535 (46.030) | 5,450 (46.041) | 11,070 (92.071) | 5,526 @ 46.055 | 5,527 @ 46.056 |
| record-matrix-01-res2 | PASS_CASE_SCREEN | 120.000301 | 28.908 | 28.94 [28.44–29.66] | 3,473 (28.900) | 3,400 (28.909) | 6,946 (57.806) | 3,469 @ 28.909 | 3,469 @ 28.911 |
| record-matrix-01-res3 | PASS_CASE_SCREEN | 120.000138 | 20.832 | 20.80 [20.29–21.23] | 2,503 (20.834) | 2,450 (20.831) | 5,004 (41.651) | 2,500 @ 20.835 | 2,500 @ 20.834 |
| record-matrix-01-res4 | PASS_CASE_SCREEN | 120.000235 | 16.253 | 16.22 [15.88–16.47] | 1,952 (16.245) | 1,900 (16.250) | 3,904 (32.490) | 1,950 @ 16.252 | 1,950 @ 16.252 |
| record-matrix-01-res5 | PASS_CASE_SCREEN | 120.000290 | 13.265 | 13.31 [13.02–13.63] | 1,593 (13.258) | 1,550 (13.269) | 3,188 (26.529) | 1,592 @ 13.272 | 1,592 @ 13.273 |
| record-matrix-01-res6 | PASS_CASE_SCREEN | 120.000221 | 11.140 | 11.12 [11.07–11.19] | 1,338 (11.138) | 1,300 (11.137) | 2,676 (22.276) | 1,337 @ 11.147 | 1,337 @ 11.147 |
| record-matrix-01-res7 | PASS_CASE_SCREEN | 120.000261 | 9.586 | 9.55 [9.40–9.72] | 1,152 (9.587) | 1,100 (9.590) | 2,304 (19.175) | 1,150 @ 9.588 | 1,150 @ 9.588 |
| record-matrix-01-res8 | PASS_CASE_SCREEN | 122.000155 | 8.243 | 8.22 [8.16–8.41] | 1,007 (8.246) | 950 (8.256) | 3,021 (24.736) | 1,007 @ 8.252 | 1,007 @ 8.252 |
| record-matrix-01-mixed1 | PASS_CASE_SCREEN | 120.000100 | 43.497 | 43.51 [42.99–43.81] | 5,225 (43.489) | 5,200 (43.503) | 5,226 (43.494) | 5,220 @ 43.499 | 5,220 @ 43.500 |
| record-matrix-01-mixed2 | PASS_CASE_SCREEN | 120.000197 | 20.902 | 20.91 [20.75–20.99] | 2,511 (20.902) | 2,450 (20.905) | 5,022 (41.804) | 2,509 @ 20.905 | 2,509 @ 20.905 |
| record-matrix-01-mixed3 | PASS_CASE_SCREEN | 120.000232 | 7.610 | 7.61 [7.59–7.62] | 914 (7.609) | 850 (7.610) | 1,828 (15.219) | 914 @ 7.611 | 913 @ 7.611 |
| record-matrix-01-mixed4 | PASS_CASE_SCREEN | 120.000148 | 7.066 | 7.06 [7.05–7.07] | 849 (7.067) | 800 (7.069) | 1,698 (14.136) | 848 @ 7.068 | 848 @ 7.067 |
| record-matrix-01-mixed5 | PASS_CASE_SCREEN | 120.000266 | 6.221 | 6.22 [6.20–6.26] | 747 (6.219) | 700 (6.225) | 2,241 (18.658) | 747 @ 6.222 | 747 @ 6.222 |
| record-matrix-01-mixed6 | PASS_CASE_SCREEN | 120.000241 | 5.494 | 5.49 [5.48–5.49] | 660 (5.495) | 600 (5.496) | 1,980 (16.487) | 660 @ 5.496 | 660 @ 5.496 |
| record-matrix-01-mixed7 | PASS_CASE_SCREEN | 120.000413 | 5.208 | 5.21 [5.20–5.21] | 625 (5.204) | 550 (5.210) | 1,875 (15.613) | 625 @ 5.210 | 625 @ 5.210 |
| record-matrix-01-mixed8 | PASS_CASE_SCREEN | 120.000101 | 4.774 | 4.76 [4.75–4.81] | 573 (4.771) | 500 (4.774) | 2,292 (19.084) | 573 @ 4.775 | 573 @ 4.775 |

SF50 各区间报告固件 emit 速率。USBQ 按 DATA 记录、BL50 按 DATA 消息、BLETX 按 SDK 调用计数，各有独立时间跨度。USB 物理总线完成与 BLE 空口完成均为 UNKNOWN。 / SF50 measures firmware emit intervals. USBQ counts DATA records, BL50 counts local DATA messages, and BLETX counts SDK calls over separate intervals. Physical USB and BLE completions are UNKNOWN.

| 工况 / Attempt | CPU % · BLE_TX / TRANSPORT / ASYNC_LOG / SERIAL_TX / BLE_CTRL / SERIAL_CTRL / WIFI_CTRL | 总空闲最低观测 / 生命周期最低 · observed free / lifetime min (B) | 最大连续 internal / external (B) | 栈余量 / Stack (B) · log / transport / bleTx / bleCtrl / serialCtrl | 恢复 / CAP8 ms · setter / capture end |
|---|---|---:|---:|---|---:|
| record-matrix-01-cap1 | 6.50 / 7.18 / 12.74 / 8.89 / UNKNOWN / 0.78 / UNKNOWN | 8,062,584 / 8,055,076 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,612 | 56.167 / 257.284 |
| record-matrix-01-cap2 | 4.13 / 4.21 / 7.83 / 6.53 / UNKNOWN / 0.71 / UNKNOWN | 8,062,504 / 8,055,076 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,612 | 80.452 / 248.645 |
| record-matrix-01-cap3 | 5.74 / 2.94 / 5.71 / 6.52 / UNKNOWN / 0.71 / UNKNOWN | 8,062,440 / 8,055,076 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 125.146 / 272.592 |
| record-matrix-01-cap4 | 4.56 / 2.26 / 4.50 / 5.80 / UNKNOWN / 0.70 / UNKNOWN | 8,062,260 / 8,055,060 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 94.040 / 195.834 |
| record-matrix-01-cap5 | 3.67 / 1.95 / 3.68 / 5.34 / UNKNOWN / 0.70 / UNKNOWN | 8,062,592 / 8,055,060 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 121.274 / 271.490 |
| record-matrix-01-cap6 | 3.13 / 1.67 / 3.17 / 5.05 / UNKNOWN / 0.71 / UNKNOWN | 8,062,552 / 8,055,060 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 143.857 / 293.961 |
| record-matrix-01-cap7 | 2.79 / 1.42 / 2.78 / 4.78 / UNKNOWN / 0.71 / UNKNOWN | 8,062,520 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 166.203 / 313.086 |
| record-matrix-01-cap8 | 2.42 / 1.31 / 2.49 / 4.50 / UNKNOWN / 0.68 / UNKNOWN | 8,062,480 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 156.896 / 292.886 |
| record-matrix-01-volt1 | 17.08 / 7.12 / 11.85 / 10.72 / UNKNOWN / 0.84 / UNKNOWN | 8,062,464 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 118.437 / 660.560 |
| record-matrix-01-volt2 | 12.51 / 4.87 / 8.23 / 9.07 / UNKNOWN / 0.80 / UNKNOWN | 8,062,576 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 104.075 / 470.207 |
| record-matrix-01-volt3 | 10.02 / 3.81 / 6.51 / 7.87 / UNKNOWN / 0.77 / UNKNOWN | 8,062,056 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 93.686 / 313.558 |
| record-matrix-01-volt4 | 8.33 / 3.21 / 5.39 / 6.81 / UNKNOWN / 0.75 / UNKNOWN | 8,062,488 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 122.485 / 320.363 |
| record-matrix-01-volt5 | 8.90 / 2.63 / 4.37 / 6.06 / UNKNOWN / 0.77 / UNKNOWN | 8,062,608 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 122.520 / 282.408 |
| record-matrix-01-volt6 | 7.77 / 2.24 / 3.82 / 6.21 / UNKNOWN / 0.75 / UNKNOWN | 8,062,136 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 142.266 / 312.869 |
| record-matrix-01-volt7 | 6.93 / 2.00 / 3.42 / 5.73 / UNKNOWN / 0.72 / UNKNOWN | 8,062,040 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 141.804 / 339.966 |
| record-matrix-01-volt8 | 6.19 / 1.86 / 3.03 / 5.49 / UNKNOWN / 0.74 / UNKNOWN | 8,062,512 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 170.351 / 334.592 |
| record-matrix-01-res1 | 12.75 / 5.16 / 8.52 / 9.17 / UNKNOWN / 0.79 / UNKNOWN | 8,062,284 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 66.856 / 283.887 |
| record-matrix-01-res2 | 8.02 / 3.23 / 5.44 / 6.92 / UNKNOWN / 0.75 / UNKNOWN | 8,062,440 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 77.509 / 249.553 |
| record-matrix-01-res3 | 5.78 / 2.39 / 3.94 / 5.71 / UNKNOWN / 0.73 / UNKNOWN | 8,062,152 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 117.688 / 334.834 |
| record-matrix-01-res4 | 4.58 / 1.87 / 3.11 / 5.08 / UNKNOWN / 0.71 / UNKNOWN | 8,062,560 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 107.426 / 316.659 |
| record-matrix-01-res5 | 3.82 / 1.49 / 2.58 / 4.62 / UNKNOWN / 0.71 / UNKNOWN | 8,062,528 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 122.697 / 277.656 |
| record-matrix-01-res6 | 3.22 / 1.29 / 2.17 / 4.22 / UNKNOWN / 0.69 / UNKNOWN | 8,062,504 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 144.379 / 260.707 |
| record-matrix-01-res7 | 2.76 / 1.11 / 1.88 / 4.11 / UNKNOWN / 0.71 / UNKNOWN | 8,062,480 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 265.354 / 453.616 |
| record-matrix-01-res8 | 3.17 / 0.99 / 1.64 / 3.84 / UNKNOWN / 0.70 / UNKNOWN | 8,062,612 / 8,054,948 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 164.968 / 354.698 |
| record-matrix-01-mixed1 | 3.98 / 3.82 / 8.13 / 6.93 / UNKNOWN / 0.70 / UNKNOWN | 8,062,488 / 8,054,932 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,420 | 72.425 / 249.882 |
| record-matrix-01-mixed2 | 4.87 / 1.71 / 4.64 / 8.89 / UNKNOWN / 0.71 / UNKNOWN | 8,062,464 / 8,054,932 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,372 | 82.746 / 226.266 |
| record-matrix-01-mixed3 | 1.75 / 0.70 / 1.93 / 5.51 / UNKNOWN / 0.57 / UNKNOWN | 8,063,016 / 8,054,932 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,372 | 193.112 / 343.061 |
| record-matrix-01-mixed4 | 1.72 / 0.60 / 1.83 / 5.28 / UNKNOWN / 0.60 / UNKNOWN | 8,062,444 / 8,054,932 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,372 | 194.279 / 318.255 |
| record-matrix-01-mixed5 | 2.11 / 0.53 / 1.62 / 4.97 / UNKNOWN / 0.57 / UNKNOWN | 8,063,016 / 8,054,852 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,372 | 205.819 / 344.592 |
| record-matrix-01-mixed6 | 1.90 / 0.47 / 1.47 / 4.78 / UNKNOWN / 0.58 / UNKNOWN | 8,063,016 / 8,054,852 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,372 | 122.745 / 236.998 |
| record-matrix-01-mixed7 | 1.83 / 0.47 / 1.41 / 4.60 / UNKNOWN / 0.57 / UNKNOWN | 8,062,464 / 8,054,852 | UNKNOWN / UNKNOWN | 12,492 / 3,108 / 2,084 / 5,196 / 1,372 | 265.005 / 396.625 |
| record-matrix-01-mixed8 | 1.98 / 0.48 / 1.31 / 4.44 / UNKNOWN / 0.56 / UNKNOWN | 8,062,544 / 8,054,852 | UNKNOWN / UNKNOWN | 12,428 / 3,108 / 2,084 / 5,196 / 1,372 | 146.513 / 251.239 |

CPU 为各任务独立采样跨度的 runtime 比例。总 heap 含可用 PSRAM，连续块按实际 MEM? 采样列出最小观测值。恢复两列均截止最终同 boot CAP8 CONFIG 实际回读。 / CPU ratios use each owner’s own interval. Total heap includes available PSRAM. Largest blocks use the minimum of observed MEM? samples by capability. Both recovery intervals end at the final same-boot CAP8 CONFIG reply. JSON retains the time bounds and every attempt.
