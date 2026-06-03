// Split from core/measure/sensorarrayMeasure.c to keep measurement domains isolated.
static sensorarrayFdcSampleStatus_t sensorarrayMeasureMapFdcStatus(Fdc2214CapSampleStatus_t sampleStatus)
{
    switch (sampleStatus) {
    case FDC2214_SAMPLE_STATUS_SAMPLE_VALID:
        return SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID;
    case FDC2214_SAMPLE_STATUS_STILL_SLEEPING:
        return SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING;
    case FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING;
    case FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION;
    case FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID;
    case FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT:
        return SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT;
    case FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT;
    case FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN:
    default:
        return SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
}

const char *sensorarrayMeasureFdcSampleStatusName(sensorarrayFdcSampleStatus_t status)
{
    switch (status) {
    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR:
        return "i2c_read_error";
    case SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN:
        return "config_unknown";
    case SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING:
        return "still_sleeping";
    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return "i2c_read_ok_but_not_converting";
    case SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return "no_unread_conversion";
    case SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return "zero_raw_invalid";
    case SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT:
        return "watchdog_fault";
    case SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return "amplitude_fault";
    case SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID:
        return "sample_valid";
    default:
        return "config_unknown";
    }
}

typedef esp_err_t (*sensorarrayFdcReadSampleFn_t)(Fdc2214CapDevice_t *dev,
                                                   Fdc2214CapChannel_t ch,
                                                   Fdc2214CapSample_t *outSample);

static esp_err_t sensorarrayMeasureReadFdcSampleDiagWithReader(sensorarrayFdcReadSampleFn_t readFn,
                                                               Fdc2214CapDevice_t *dev,
                                                               Fdc2214CapChannel_t ch,
                                                               bool discardFirst,
                                                               bool idOk,
                                                               bool configOk,
                                                               bool relaxedMode,
                                                               sensorarrayFdcReadDiag_t *outDiag)
{
    if (!readFn || !dev || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    FDCLOW_TRACE("FDCLOW,stage=diag_enter,channel=%u,discardFirst=%u,relaxed=%u\n",
                    (unsigned)ch,
                    discardFirst ? 1u : 0u,
                    relaxedMode ? 1u : 0u);

    *outDiag = (sensorarrayFdcReadDiag_t){
        .err = ESP_FAIL,
        .i2cOk = false,
        .idOk = idOk,
        .configOk = configOk,
        .statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR,
        .sampleValid = false,
        .provisionalReadable = false,
        .qualityDegraded = false,
    };

    if (discardFirst) {
        Fdc2214CapSample_t throwaway = {0};
        FDCLOW_TRACE("FDCLOW,stage=discard_sample_begin,channel=%u\n", (unsigned)ch);
        esp_err_t discardErr = readFn(dev, ch, &throwaway);
        FDCLOW_TRACE("FDCLOW,stage=discard_sample_done,channel=%u,err=%ld\n",
                        (unsigned)ch,
                        (long)discardErr);
        if (discardErr != ESP_OK) {
            outDiag->err = discardErr;
            FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                            (long)discardErr,
                            (unsigned)outDiag->statusCode);
            return discardErr;
        }
    }

    FDCLOW_TRACE("FDCLOW,stage=read_sample_begin,channel=%u\n", (unsigned)ch);
    esp_err_t err = readFn(dev, ch, &outDiag->sample);
    FDCLOW_TRACE("FDCLOW,stage=read_sample_done,channel=%u,err=%ld,raw=%lu,status=0x%04X,"
                    "config=0x%04X,mux=0x%04X,statusCode=%u\n",
                    (unsigned)ch,
                    (long)err,
                    (unsigned long)outDiag->sample.Raw28,
                    outDiag->sample.StatusRaw,
                    outDiag->sample.ConfigRaw,
                    outDiag->sample.MuxRaw,
                    (unsigned)outDiag->sample.SampleStatus);
    outDiag->err = err;
    outDiag->i2cOk = (err == ESP_OK);
    if (err != ESP_OK) {
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                        (long)err,
                        (unsigned)outDiag->statusCode);
        return err;
    }

    FDCLOW_TRACE("FDCLOW,stage=read_status_config_begin\n");
    esp_err_t statusCfgErr = Fdc2214CapReadCoreRegs(dev, &outDiag->coreRegs);
    FDCLOW_TRACE("FDCLOW,stage=read_status_config_done,err=%ld,statusConfig=0x%04X\n",
                    (long)statusCfgErr,
                    outDiag->coreRegs.StatusConfig);
    if (statusCfgErr != ESP_OK) {
        outDiag->err = statusCfgErr;
        outDiag->i2cOk = false;
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                        (long)statusCfgErr,
                        (unsigned)outDiag->statusCode);
        return statusCfgErr;
    }

    (void)Fdc2214CapDecodeStatusRaw(outDiag->sample.StatusRaw, &outDiag->status);

    outDiag->converting = outDiag->sample.Converting;
    outDiag->unreadConversionPresent = Fdc2214CapStatusHasUnreadForChannel(&outDiag->status, ch);

    sensorarrayFdcSampleStatus_t mappedStatus = sensorarrayMeasureMapFdcStatus(outDiag->sample.SampleStatus);
    if (!idOk || !configOk) {
        mappedStatus = SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
    bool watchdogFault = Fdc2214CapStatusHasWatchdogFault(&outDiag->status);
    bool amplitudeFault = Fdc2214CapStatusHasAmplitudeFault(&outDiag->status) ||
                          outDiag->sample.ErrAmplitude;
    bool readable = outDiag->unreadConversionPresent || outDiag->status.DataReady || outDiag->sample.DataReady;
    outDiag->statusCode = mappedStatus;
    outDiag->qualityDegraded = (!readable) ||
                               outDiag->sample.ErrWatchdog ||
                               outDiag->sample.ErrAmplitude ||
                               watchdogFault ||
                               amplitudeFault;
    outDiag->provisionalReadable = idOk &&
                                   configOk &&
                                   outDiag->sample.Converting &&
                                   (outDiag->sample.Raw28 != 0u) &&
                                   !outDiag->sample.ErrWatchdog &&
                                   !watchdogFault &&
                                   readable;
    outDiag->sampleValid = relaxedMode ? outDiag->provisionalReadable
                                       : (outDiag->provisionalReadable &&
                                          (mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID ||
                                           mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT));
    if (!idOk || !configOk) {
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
    }
    FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u,sampleValid=%u,provisional=%u\n",
                    (long)outDiag->err,
                    (unsigned)outDiag->statusCode,
                    outDiag->sampleValid ? 1u : 0u,
                    outDiag->provisionalReadable ? 1u : 0u);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadFdcSampleDiag(Fdc2214CapDevice_t *dev,
                                              Fdc2214CapChannel_t ch,
                                              bool discardFirst,
                                              bool idOk,
                                              bool configOk,
                                              sensorarrayFdcReadDiag_t *outDiag)
{
    return sensorarrayMeasureReadFdcSampleDiagWithReader(Fdc2214CapReadSample,
                                                         dev,
                                                         ch,
                                                         discardFirst,
                                                         idOk,
                                                         configOk,
                                                         false,
                                                         outDiag);
}

esp_err_t sensorarrayMeasureReadFdcSampleDiagRelaxed(Fdc2214CapDevice_t *dev,
                                                     Fdc2214CapChannel_t ch,
                                                     bool discardFirst,
                                                     bool idOk,
                                                     bool configOk,
                                                     sensorarrayFdcReadDiag_t *outDiag)
{
    return sensorarrayMeasureReadFdcSampleDiagWithReader(Fdc2214CapReadSampleRelaxed,
                                                         dev,
                                                         ch,
                                                         discardFirst,
                                                         idOk,
                                                         configOk,
                                                         true,
                                                         outDiag);
}

esp_err_t sensorarrayMeasureReadFdcSample(Fdc2214CapDevice_t *dev,
                                          Fdc2214CapChannel_t ch,
                                          bool discardFirst,
                                          Fdc2214CapSample_t *outSample)
{
    if (!dev || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcReadDiag_t diag = {0};
    esp_err_t err = sensorarrayMeasureReadFdcSampleDiag(dev, ch, discardFirst, true, true, &diag);
    if (err != ESP_OK) {
        return err;
    }
    *outSample = diag.sample;
    return ESP_OK;
}

double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz)
{
    if (raw28 == 0u || refClockHz == 0u) {
        return 0.0;
    }
    return ((double)raw28 * (double)refClockHz) / SENSORARRAY_FDC_RAW_SCALE_2P28;
}

const char *sensorarrayMeasureFdcRefClockSourceName(sensorarrayFdcRefClockSource_t source)
{
    switch (source) {
    case SENSORARRAY_FDC_REF_CLOCK_SOURCE_INTERNAL:
        return "internal_oscillator";
    case SENSORARRAY_FDC_REF_CLOCK_SOURCE_EXTERNAL:
        return "external_clkin";
    default:
        return "unknown";
    }
}

sensorarrayFdcRefClockSource_t sensorarrayMeasureFdcEffectiveRefClockSource(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_REF_CLOCK_SOURCE_EXTERNAL;
#else
    return SENSORARRAY_FDC_REF_CLOCK_SOURCE_INTERNAL;
#endif
}

uint32_t sensorarrayMeasureFdcEffectiveFclkHz(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_EXTERNAL_CLOCK_HZ;
#else
#if SENSORARRAY_FDC_INTERNAL_CLOCK_CALIBRATED_HZ > 0u
    return SENSORARRAY_FDC_INTERNAL_CLOCK_CALIBRATED_HZ;
#else
    return SENSORARRAY_FDC_INTERNAL_CLOCK_NOMINAL_HZ;
#endif
#endif
}

bool sensorarrayMeasureFdcDecodeClockDividers(uint16_t clockDividers,
                                              uint8_t *outFinSelCode,
                                              uint8_t *outFinFactor,
                                              uint16_t *outFrefDivider,
                                              const char **outStatus)
{
    if (!outFinSelCode || !outFinFactor || !outFrefDivider || !outStatus) {
        if (outStatus) {
            *outStatus = "invalid_arg";
        }
        return false;
    }

    uint8_t finSelCode = (uint8_t)((clockDividers >> 12) & 0x03u);
    uint16_t frefDivider = (uint16_t)(clockDividers & 0x03FFu);

    *outFinSelCode = finSelCode;
    *outFinFactor = 0u;
    *outFrefDivider = frefDivider;
    *outStatus = "ok";

    if (frefDivider == 0u) {
        *outStatus = "zero_fref_divider";
        return false;
    }
    if (finSelCode == 0u) {
        *outStatus = "invalid_fin_sel_zero";
        return false;
    }
    if (finSelCode == 1u) {
        *outFinFactor = 1u;
        return true;
    }
    if (finSelCode == 2u) {
        *outFinFactor = 2u;
        return true;
    }

    *outStatus = "unsupported_fin_sel_3";
    return false;
}

double sensorarrayMeasureFdcFinFactorFromClockDiv(uint16_t clockDividers)
{
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *status = "unknown";
    if (!sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                  &finSelCode,
                                                  &finFactor,
                                                  &frefDivider,
                                                  &status)) {
        return 0.0;
    }
    (void)finSelCode;
    (void)frefDivider;
    (void)status;
    return (double)finFactor;
}

double sensorarrayMeasureFdcFrefDividerFromClockDiv(uint16_t clockDividers)
{
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *status = "unknown";
    if (!sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                  &finSelCode,
                                                  &finFactor,
                                                  &frefDivider,
                                                  &status)) {
        return 0.0;
    }
    (void)finSelCode;
    (void)finFactor;
    (void)status;
    return (double)frefDivider;
}

double sensorarrayMeasureFdcRaw28ToFreqHz(uint32_t raw28,
                                          uint32_t effectiveFclkHz,
                                          uint16_t clockDividers)
{
    if (raw28 == 0u || effectiveFclkHz == 0u) {
        return 0.0;
    }

    double finFactor = sensorarrayMeasureFdcFinFactorFromClockDiv(clockDividers);
    double frefDivider = sensorarrayMeasureFdcFrefDividerFromClockDiv(clockDividers);
    if (finFactor <= 0.0 || frefDivider <= 0.0) {
        return 0.0;
    }

    double effectiveFrefHz = (double)effectiveFclkHz / frefDivider;
    return ((double)raw28 * effectiveFrefHz * finFactor) / SENSORARRAY_FDC_RAW_SCALE_2P28;
}

bool sensorarrayMeasureFdcComputeFrequencyDiag(uint32_t raw28,
                                               uint16_t clockDividers,
                                               sensorarrayFdcFrequencyDiag_t *outDiag)
{
    if (!outDiag) {
        return false;
    }

    *outDiag = (sensorarrayFdcFrequencyDiag_t){
        .clockDividers = clockDividers,
        .refClockSource = sensorarrayMeasureFdcEffectiveRefClockSource(),
        .effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz(),
        .valid = false,
        .status = "unknown",
    };

    if (raw28 == 0u) {
        outDiag->status = "zero_raw";
        return false;
    }
    if (outDiag->effectiveFclkHz == 0u) {
        outDiag->status = "zero_effective_fclk";
        return false;
    }

    const char *decodeStatus = "unknown";
    bool decodeOk = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                             &outDiag->finSelCode,
                                                             &outDiag->finFactor,
                                                             &outDiag->frefDivider,
                                                             &decodeStatus);
    outDiag->status = decodeStatus;
    if (!decodeOk) {
        return false;
    }

    outDiag->effectiveFrefHz = (double)outDiag->effectiveFclkHz / (double)outDiag->frefDivider;
    outDiag->freqHzBase = sensorarrayMeasureFdcRawToFrequencyHz(raw28, outDiag->effectiveFclkHz);
    outDiag->freqHzCorrected =
        sensorarrayMeasureFdcRaw28ToFreqHz(raw28, outDiag->effectiveFclkHz, clockDividers);
    outDiag->valid = true;
    outDiag->status = "ok";
    return true;
}

double sensorarrayMeasureFdcRawToSensorFrequencyHz(uint32_t raw28, uint16_t clockDividers)
{
    sensorarrayFdcFrequencyDiag_t diag = {0};
    if (!sensorarrayMeasureFdcComputeFrequencyDiag(raw28, clockDividers, &diag) || !diag.valid) {
        return 0.0;
    }
    return diag.freqHzCorrected;
}

bool sensorarrayMeasureFdcTryCapacitancePf(double frequencyHz, uint32_t inductorUh, double *outCapPf)
{
    return sensorarrayMeasureFdcComputeCapacitancePf(frequencyHz, (double)inductorUh, outCapPf);
}

bool sensorarrayMeasureFdcComputeCapacitancePf(double frequencyHz, double inductorValueUh, double *outCapPf)
{
    if (!outCapPf || frequencyHz <= 0.0 || inductorValueUh <= 0.0) {
        return false;
    }

    const double inductorH = inductorValueUh * 1e-6;
    const double omega = 2.0 * SENSORARRAY_PI * frequencyHz;
    const double denom = omega * omega * inductorH;
    if (!isfinite(denom) || denom <= 0.0) {
