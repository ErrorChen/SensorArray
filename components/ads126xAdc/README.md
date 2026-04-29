# ads126xAdc / ADS126x Driver

`ads126xAdc` 是通用 ADS1262/ADS1263 SPI driver。它只处理 ADS 芯片级行为：寄存器访问、ADC1/ADC2 控制、INPMUX/REFMUX、internal reference、VBIAS/AINCOM level shift、raw code 读取、raw->microvolts、fast voltage read 和 auto-gain。它不包含 `S1D1`、`D1..D8`、`SW`、`PIEZO_READ`、`RESISTANCE_READ` 或任何板级 route mapping。

## Data Rate And Gain Enums

ADC1 MODE2 register：address `0x05`，bit7=`BYPASS`，bits 6:4=`GAIN[2:0]`，bits 3:0=`DR[3:0]`。

Public enums：

- `ads126xAdc1DataRate_t`: `ADS126X_ADC1_DR_2P5_SPS` through `ADS126X_ADC1_DR_38400_SPS`，DR code `0x0..0xF`。
- `ads126xGain_t`: `ADS126X_GAIN_1`, `2`, `4`, `8`, `16`, `32`。

`ads126xAdcConfigureVoltageMode(handle, gain, dataRateDr, enableStatusByte, enableCrc)` 会保留当前 internal reference setting，并配置 status/CRC、PGA gain 和 ADC1 data rate。

## Internal Reference / VBIAS / REFMUX

相关 API：

```c
esp_err_t ads126xAdcConfigure(ads126xAdcHandle_t *handle,
                              bool enableInternalRef,
                              bool enableStatusByte,
                              ads126xCrcMode_t crcMode,
                              uint8_t pgaGain,
                              uint8_t dataRateDr);

esp_err_t ads126xAdcSetVbiasEnabled(ads126xAdcHandle_t *handle, bool enableVbias);
esp_err_t ads126xAdcSetRefMux(ads126xAdcHandle_t *handle, uint8_t refmuxValue);
esp_err_t ads126xAdcReadCoreRegisters(ads126xAdcHandle_t *handle,
                                      uint8_t *outPower,
                                      uint8_t *outInterface,
                                      uint8_t *outMode2,
                                      uint8_t *outInpmux,
                                      uint8_t *outRefmux);
```

`ADS126X_REFMUX_INTERNAL` 表示 ADS126x internal 2.5 V reference。应用层的 `sensorarrayBringupPrepareAdsRefPath()` 会启用 internal reference、启用 VBIAS/AINCOM level shift、设置 REFMUX，并用 `ads126xAdcReadCoreRegisters()` 读回 `POWER`、`INTERFACE`、`MODE2`、`INPMUX`、`REFMUX`。ADS driver 只提供通用寄存器能力，不判断板上 `REF/MID` 模拟节点是否真的达到目标电压。

## Fast Voltage Read API

```c
typedef struct {
    uint8_t muxp;
    uint8_t muxn;
    bool stopBeforeMuxChange;
    bool startAfterMuxChange;
    uint32_t settleAfterMuxUs;
    bool discardFirst;
    uint8_t oversampleCount;
    uint32_t drdyTimeoutUs;
} ads126xAdcVoltageReadConfig_t;

esp_err_t ads126xAdcReadVoltageMicrovoltsFast(
    ads126xAdcHandle_t *handle,
    const ads126xAdcVoltageReadConfig_t *cfg,
    ads126xAdcVoltageSample_t *outSample);
```

Fast read 行为：

- 只接受 ADS `muxp/muxn`，不理解板级 D-line。
- 使用 `esp_timer_get_time()` + `gpio_get_level()` 做 us 级 DRDY polling。
- 默认不 STOP/START；是否 STOP/START 由 cfg 控制。
- `discardFirst=true` 时第一笔 DRDY/RDATA 会被读出并丢弃。
- `oversampleCount` 使用 `int64_t` 累加 raw，再平均为 `int32_t`。
- `microvolts` 来自 `ads126xAdcRawToMicrovolts()`。
- `clippedOrNearFullScale` 使用 `abs(raw) > 0.90 * INT32_MAX` 判定。
- 函数内部不 printf、不做高频 ESP_LOGI。

## Raw To Microvolts

ADC1 raw code 是 signed 32-bit。换算公式：

```text
microvolts = raw * vref_microvolts / (gain * 2^31)
```

默认 `vrefMicrovolts=2500000`，输出单位始终是 `int32_t microvolts`。

## Auto Gain

```c
esp_err_t ads126xAdcSelectAutoGainForVoltage(
    ads126xAdcHandle_t *handle,
    const ads126xAdcVoltageReadConfig_t *readCfg,
    const ads126xAdcAutoGainConfig_t *gainCfg,
    ads126xAdcAutoGainResult_t *outResult);
```

ADS126x differential full-scale range：

```text
FS = +-VREF / Gain
```

使用 2.5 V reference 时：

- gain 1: `+-2.5 V`
- gain 2: `+-1.25 V`
- gain 4: `+-0.625 V`
- gain 8: `+-0.3125 V`
- gain 16: `+-0.15625 V`
- gain 32: `+-0.078125 V`

Auto-gain 先用 `initialGain` 读取，再选择满足下面条件的最高合法 gain：

```text
abs(sample_uv) * gain < vref_uv * headroomPercent / 100
```

如果 raw 近满量程或 clipped，会向低 gain 回退。`keepConfiguredOnSuccess=true` 会把 ADS 保持在 selected gain；否则恢复原 gain。频繁 per-point/per-frame auto-gain 会额外写寄存器并重复读取，会明显降低扫描速度；高速矩阵扫描默认 bootstrap once。

## Typical Voltage Flow

```c
ads126xAdcConfigureVoltageMode(&adc, ADS126X_GAIN_1, ADS126X_ADC1_DR_38400_SPS, false, false);
ads126xAdcStartAdc1(&adc);

ads126xAdcVoltageReadConfig_t readCfg = {
    .muxp = 0,
    .muxn = 0x0A,
    .settleAfterMuxUs = 20,
    .discardFirst = true,
    .oversampleCount = 1,
    .drdyTimeoutUs = 2000,
};
ads126xAdcVoltageSample_t sample = {0};
ads126xAdcReadVoltageMicrovoltsFast(&adc, &readCfg, &sample);
```

Board/app layer is responsible for translating `D1..D8 -> muxp/muxn`.
