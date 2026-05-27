# boardSupport / ??????

## 1) Scope / ????

**??**

`boardSupport` ???? I2C ????????? + ???????????? FDC ??????? I2C ???????

**English**

`boardSupport` initializes board-level I2C buses (primary + optional secondary) and provides I2C callback adapters matching FDC driver signatures.

## 2) Public API / ????

- `boardSupportInit`
- `boardSupportDeinit`
- `boardSupportIsI2c1Enabled`
- `boardSupportGetI2cCtx`
- `boardSupportGetI2c1Ctx`
- `boardSupportGetI2cBusInfo`
- `boardSupportI2cWriteRead`
- `boardSupportI2cWrite`

## 3) Boundary / ??

**??**

- ??????????
- ???? D-line ????? ADS/FDC ?????

**English**

- Bus and callback adapter only.
- No D-line routing ownership and no ADS/FDC application policy.

## 4) Current Status / ????

- Used by `main/sensorarrayApp.c` bring-up path.
- Exposes configured bus metadata for diagnostics such as secondary FDC diagnostics.
