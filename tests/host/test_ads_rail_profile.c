/* Synthetic registers and failures; executes the actual production rail chain.
 * It is not a recording or a substitute for external DMM/HIL validation. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ads126xAdc.h"
#include "sensorarrayAdsCache.h"

#define SENSORARRAY_ADS_REG_OFCAL0 0x07u
#define SENSORARRAY_ADS_REG_FSCAL0 0x0Au
#define SENSORARRAY_ADS_MUX_ANALOG_MONITOR 0x0Cu
#define SENSORARRAY_ADS_INTERNAL_VREF_UV 2500000u
#define SENSORARRAY_ADS_RAIL_MONITOR_SETTLE_MS 1u
#define SENSORARRAY_ADS_RAIL_MONITOR_DISCARD_COUNT 4u
#define SENSORARRAY_ADS_RAIL_CRC_RETRY_COUNT 1u
#define CONFIG_SENSORARRAY_ADS_DATA_RATE 15u
#define CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US 600u

typedef struct {
    ads126xAdcHandle_t ads;
    uint8_t adsRefMux;
    bool adsRefMuxValid;
    bool adsAdc1Running;
} sensorarrayState_t;
/* INSERT_PRODUCTION_TYPES */
static sensorarrayAdsRegisterCache_t s_cache;
static sensorarrayAdsRegisterCache_t *s_registerCache = &s_cache;
static bool s_batteryRestoreFailureLatched;
static struct { bool fallbackToBoundary; } s_snapshot;
static const uint8_t s_adsNeutralOffsetCal[3] = {0u, 0u, 0u};
static const uint8_t s_adsUnityFullScaleCal[3] = {0u, 0u, 0x40u};
static ads126xAdc1RegisterSnapshot_t s_physical, s_restoredCache;
static bool s_unclamped, s_isolated, s_running, s_cacheInvalidated;
static unsigned s_snapshotReads, s_samples, s_restoreCalls, s_stopCalls, s_cachePopulated;
static esp_err_t s_powerError, s_configError, s_mode2Error, s_sampleError, s_restoreError;
static unsigned s_failSnapshotRead, s_failStopCall;
static uint8_t s_profileMismatchReg, s_status;
static uint32_t s_delayMs;
static int64_t s_nowUs;
static unsigned s_waitCalls, s_timeoutWaitCall;
static esp_err_t s_sampleErrors[8];
static uint32_t s_readGenerations[8];

bool sensorarrayBoardMapHasIsolatedRailMonitor(void) { return s_isolated; }
static bool sensorarrayAdsInternalReferenceIsUnclamped(void) { return s_unclamped; }
void vTaskDelay(TickType_t ticks) { s_delayMs += ticks; s_nowUs += ticks * 1000u; }
int64_t esp_timer_get_time(void) { return s_nowUs; }
esp_err_t ads126xAdcReadPowerRegister(ads126xAdcHandle_t *ads, uint8_t *value)
{ (void)ads; *value = s_physical.power; return s_powerError; }
bool ads126xAdcHasAdc2(const ads126xAdcHandle_t *ads) { (void)ads; return false; }
bool ads126xAdcIsAdc1Running(const ads126xAdcHandle_t *ads) { (void)ads; return s_running; }
bool ads126xAdcIsAdc2Running(const ads126xAdcHandle_t *ads) { (void)ads; return false; }
esp_err_t ads126xAdcStopAdc2(ads126xAdcHandle_t *ads) { (void)ads; assert(false); return ESP_FAIL; }
esp_err_t ads126xAdcStartAdc2(ads126xAdcHandle_t *ads) { (void)ads; assert(false); return ESP_FAIL; }
esp_err_t ads126xAdcStopAdc1(ads126xAdcHandle_t *ads)
{ (void)ads; ++s_stopCalls; if (s_stopCalls == s_failStopCall) return ESP_ERR_TIMEOUT;
  s_running = false; return ESP_OK; }
esp_err_t ads126xAdcSetInternalReference(ads126xAdcHandle_t *ads, bool enabled)
{ assert(enabled && s_unclamped); s_physical.power |= ADS126X_POWER_INTREF;
  ads->enableInternalRef = enabled; return ESP_OK; }
esp_err_t ads126xAdcSetVbiasEnabled(ads126xAdcHandle_t *ads, bool enabled)
{ (void)ads; assert(enabled); s_physical.power |= ADS126X_POWER_VBIAS; return ESP_OK; }
esp_err_t ads126xAdcSetRefMuxWithVref(ads126xAdcHandle_t *ads, uint8_t mux, uint32_t uv)
{ s_physical.refmux = mux; ads->vrefMicrovolts = uv; return ESP_OK; }
esp_err_t ads126xAdcWaitReferenceReady(ads126xAdcHandle_t *ads) { (void)ads; return ESP_OK; }
esp_err_t ads126xAdcConfigureAdc1Mode(ads126xAdcHandle_t *ads, uint8_t m0, uint8_t m1)
{ if (s_configError != ESP_OK) return s_configError;
  s_physical.mode0 = ads->mode0 = m0; s_physical.mode1 = ads->mode1 = m1; return ESP_OK; }
bool ads126xAdcBuildMode2(bool bypass, uint8_t gain, uint8_t rate, uint8_t *mode2)
{ assert(!bypass && gain == 1u && rate == 15u); *mode2 = rate; return true; }
esp_err_t ads126xAdcSetMode2Verified(ads126xAdcHandle_t *ads, uint8_t mode2)
{ if (s_mode2Error != ESP_OK) return s_mode2Error;
  s_physical.mode2 = mode2; ads->pgaBypassed = false; ads->pgaGain = 1u;
  ads->dataRateDr = mode2 & 0x0Fu; return ESP_OK; }
esp_err_t ads126xAdcWriteRegisters(ads126xAdcHandle_t *ads, uint8_t address, const uint8_t *data, size_t length)
{ (void)ads; assert(length == 3u); if (address == 7u) memcpy(s_physical.offsetCal, data, 3u);
  else { assert(address == 10u); memcpy(s_physical.fullScaleCal, data, 3u); } return ESP_OK; }
esp_err_t ads126xAdcSetInputMux(ads126xAdcHandle_t *ads, uint8_t positive, uint8_t negative)
{ (void)ads; s_physical.inpmux = (uint8_t)((positive << 4u) | negative); return ESP_OK; }
esp_err_t ads126xAdcReadAdc1RegisterSnapshot(ads126xAdcHandle_t *ads, ads126xAdc1RegisterSnapshot_t *out)
{ (void)ads; ++s_snapshotReads; if (s_snapshotReads == s_failSnapshotRead) return ESP_FAIL;
  *out = s_physical;
  if (s_snapshotReads == 2u) {
      switch (s_profileMismatchReg) {
          case 1: out->power |= ADS126X_POWER_RESET; break;
          case 2: out->interface ^= 1u; break;
          case 3: out->mode0 ^= 0x10u; break;
          case 4: out->mode1 ^= 0x20u; break;
          case 5: out->mode2 |= 0x80u; break;
          case 6: out->inpmux ^= 1u; break;
          case 7: out->offsetCal[0] ^= 1u; break;
          case 10: out->fullScaleCal[0] ^= 1u; break;
          case 15: out->refmux ^= 1u; break;
          default: break;
      }
  } return ESP_OK; }
void ads126xAdcClearDrdyNotifications(ads126xAdcHandle_t *ads) { (void)ads; }
esp_err_t ads126xAdcStartAdc1(ads126xAdcHandle_t *ads)
{ (void)ads; assert(s_snapshotReads == 2u); s_running = true; return ESP_OK; }
uint32_t ads126xAdcGetDrdyGeneration(const ads126xAdcHandle_t *ads) { return ads->drdyGeneration; }
bool ads126xAdcIsQuarantined(const ads126xAdcHandle_t *ads) { return ads->spiQuarantined; }
bool ads126xAdcStatusByteHasAdc1NewData(const ads126xAdcHandle_t *ads, uint8_t status)
{ (void)ads; return (status & ADS126X_STATUS_ADC1_NEW_DATA) != 0u; }
esp_err_t ads126xAdcWaitDrdyGenerationUs(ads126xAdcHandle_t *ads, uint32_t generation,
                                      uint32_t timeout, uint32_t *outGeneration)
{ assert(timeout == 600u && s_running && generation == ads->drdyGeneration);
  ++s_waitCalls;
  if (s_waitCalls == s_timeoutWaitCall || (uint64_t)s_nowUs + 100u > ads->transactionDeadlineUs) {
      uint64_t remaining = ads->transactionDeadlineUs > (uint64_t)s_nowUs ?
          ads->transactionDeadlineUs - (uint64_t)s_nowUs : 0u;
      s_nowUs += remaining < timeout ? remaining : timeout; return ESP_ERR_TIMEOUT;
  }
  s_nowUs += 100u; *outGeneration = ++ads->drdyGeneration; return ESP_OK; }
esp_err_t ads126xAdcReadAdc1RawDmaReady(ads126xAdcHandle_t *ads, int32_t *raw,
                                     uint8_t *status, uint32_t *readUs)
{
  assert(s_physical.mode0 == 0u && s_physical.mode1 == 0u && s_physical.mode2 == 0x0Fu);
  assert(!ads->pgaBypassed && ads->pgaGain == 1u && ads->vrefMicrovolts == 2500000u);
  assert(s_physical.inpmux == 0xCCu && s_physical.refmux == 0u);
  assert(!memcmp(s_physical.offsetCal, s_adsNeutralOffsetCal, 3u));
  assert(!memcmp(s_physical.fullScaleCal, s_adsUnityFullScaleCal, 3u));
  assert(s_samples < 8u); s_readGenerations[s_samples] = ads->drdyGeneration;
  esp_err_t err = s_sampleErrors[s_samples] != ESP_OK ? s_sampleErrors[s_samples] : s_sampleError;
  ++s_samples; s_nowUs += 100u; *readUs = 100u;
  if (err == ESP_ERR_INVALID_CRC) {
      ads->lastAdc1CrcFault = (ads126xAdcCrcFault_t){.bytes={0x60u,0x42u,0x23u,0x54u,0x68u,0x21u},
          .length=6u,.received=0x21u,.computed=0x20u,.configurationGeneration=ads->configurationGeneration,
          .drdyGeneration=ads->drdyGeneration,.timestampUs=(uint64_t)s_nowUs};
      ads->adc1CrcErrorCount++;
  }
  if (err == ESP_OK) { *raw = 1109871204; *status = s_status; }
  return err; }
/* This rail harness stubs the physical wire layer; the selected transport is
 * exercised separately by test_ads_scientific_read with the full driver. */
static esp_err_t ads126xAdcReadAdc1ReadyWithPolicy(ads126xAdcHandle_t *ads,
    ads126xAdcAdc1ReadPolicy_t policy, int32_t *raw, uint8_t *status,
    uint32_t *readUs, ads126xAdcTailEvidence_t *tail)
{
    assert(policy == ADS126X_ADC1_READ_SPI_BYTES && tail == NULL);
    return ads126xAdcReadAdc1RawDmaReady(ads, raw, status, readUs);
}
int32_t ads126xAdcRawToMicrovolts(const ads126xAdcHandle_t *ads, int32_t raw)
{ assert(ads->pgaGain == 1u && raw == 1109871204); return 1292050; }
static void sensorarrayAdsRecordFreshSample(const sensorarrayAdsFreshSample_t *sample, esp_err_t err)
{ (void)err; assert(sample); }
esp_err_t ads126xAdcRestoreAdc1RegisterSnapshot(ads126xAdcHandle_t *ads,
    const ads126xAdc1RegisterSnapshot_t *saved, uint32_t uv, bool running)
{ ++s_restoreCalls; if (s_restoreError != ESP_OK) return s_restoreError;
  s_physical = *saved; s_running = running; ads->vrefMicrovolts = uv;
  ads->mode0 = saved->mode0; ads->mode1 = saved->mode1; ads->pgaBypassed = (saved->mode2 & 0x80u) != 0u;
  ++ads->configurationGeneration; return ESP_OK; }
static void sensorarrayAdsPopulateRestoredRegisterCache(sensorarrayAdsRegisterCache_t *cache,
    const ads126xAdc1RegisterSnapshot_t *snapshot, uint32_t uv, bool running)
{ assert(cache == s_registerCache && uv == 5167748u && running);
  s_restoredCache = *snapshot; ++s_cachePopulated; }
void sensorarrayAdsRegisterCacheInvalidate(sensorarrayAdsRegisterCache_t *cache)
{ assert(cache == s_registerCache); s_cacheInvalidated = true; }

/* INSERT_PRODUCTION_FUNCTIONS */

static sensorarrayState_t reset(uint8_t mode0, uint8_t mode1, uint8_t mode2)
{
    s_physical = (ads126xAdc1RegisterSnapshot_t){.id=3u,.power=3u,.interface=6u,
        .mode0=mode0,.mode1=mode1,.mode2=mode2,.inpmux=0x7Au,.refmux=0x24u,
        .offsetCal={0x12u,0x34u,0x56u},.fullScaleCal={0x34u,0x12u,0x40u}};
    s_unclamped = s_running = true; s_isolated = s_cacheInvalidated = false;
    s_batteryRestoreFailureLatched = false; s_snapshot.fallbackToBoundary = false;
    s_snapshotReads = s_samples = s_restoreCalls = s_stopCalls = s_cachePopulated = s_delayMs = 0u;
    s_failSnapshotRead = s_failStopCall = 0u; s_profileMismatchReg = UINT8_MAX;
    s_status = ADS126X_STATUS_ADC1_NEW_DATA;
    s_nowUs = 1000000; s_waitCalls = s_timeoutWaitCall = 0u;
    memset(s_sampleErrors,0,sizeof(s_sampleErrors)); memset(s_readGenerations,0,sizeof(s_readGenerations));
    s_powerError = s_configError = s_mode2Error = s_sampleError = s_restoreError = ESP_OK;
    return (sensorarrayState_t){.ads={.pgaGain=(uint8_t)(1u << ((mode2 >> 4u) & 7u)),
        .pgaBypassed=(mode2 & 0x80u) != 0u,.dataRateDr=(uint8_t)(mode2 & 15u),
        .vrefMicrovolts=5167748u,.crcMode=ADS126X_CRC_CRC8,.enableStatusByte=true,
        .enableInternalRef=true,.mode0=mode0,.mode1=mode1,
        .transactionDeadlineUs=1250000u,.spiDmaCapable=true}};
}

int main(void)
{
    sensorarrayState_t state;
    sensorarrayAdsRailDiag_t diag;
    int32_t raw, uv;
    const uint8_t profiles[][3] = {{0u,0u,0x0Fu},{0u,0u,0x8Fu},{0x30u,0x80u,0x5Au}};
    for (unsigned i = 0u; i < 3u; ++i) {
        state = reset(profiles[i][0],profiles[i][1],profiles[i][2]);
        ads126xAdc1RegisterSnapshot_t before = s_physical;
        diag = (sensorarrayAdsRailDiag_t){0}; raw = uv = -1;
        assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_OK);
        assert(raw == 1109871204 && uv == 1292050 && s_samples == 5u);
        assert(diag.acceptedSamples == 5u && diag.crcErrors == 0u);
        assert(state.ads.transactionDeadlineUs == 1250000u);
        assert(diag.profileVerified && diag.configuredKnown && diag.restored);
        assert(diag.configured.mode2 == 0x0Fu && diag.originalError == ESP_OK && diag.cleanupError == ESP_OK);
        assert(!memcmp(&s_physical,&before,sizeof(before)) && !memcmp(&s_restoredCache,&before,sizeof(before)));
        assert(s_restoreCalls == 1u && s_cachePopulated == 1u && !s_cacheInvalidated && s_delayMs == 1u);
        assert(state.ads.pgaBypassed == ((before.mode2 & 0x80u) != 0u));
        assert(state.ads.pgaGain == (1u << ((before.mode2 >> 4u) & 7u)) && state.ads.mode0 == before.mode0);
        assert(state.ads.dataRateDr == (before.mode2 & 15u) && state.ads.configurationGeneration == 1u);
        assert(state.adsRefMuxValid && state.adsRefMux == before.refmux && state.adsAdc1Running);
    }
    const uint8_t mismatchRegisters[] = {1u,2u,3u,4u,5u,6u,7u,10u,15u};
    for (unsigned i = 0u; i < sizeof(mismatchRegisters); ++i) {
        state = reset(0u,0u,0x8Fu); s_profileMismatchReg = mismatchRegisters[i]; raw = uv = -1;
        assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_RESPONSE);
        assert(!diag.profileVerified && diag.configuredKnown && diag.compareRegister == mismatchRegisters[i]);
        assert(!strcmp(diag.failureStage,"PROFILE_COMPARE") && diag.restored && s_samples == 0u);
        assert(raw == -1 && uv == -1 && s_restoreCalls == 1u);
    }
    state = reset(0u,0u,0x8Fu); s_failSnapshotRead = 2u;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_FAIL);
    assert(!diag.configuredKnown && !diag.profileVerified && diag.restored && s_samples == 0u);
    state = reset(0u,0u,0x8Fu); s_configError = ESP_ERR_TIMEOUT;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_TIMEOUT);
    assert(diag.originalError == ESP_ERR_TIMEOUT && diag.restored && s_samples == 0u);
    state = reset(0u,0u,0x8Fu); s_mode2Error = ESP_ERR_INVALID_RESPONSE;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_RESPONSE);
    assert(diag.restored && s_samples == 0u);
    for (unsigned bit = 0u; bit < 5u; ++bit) {
        state = reset(0u,0u,0x8Fu); s_status |= (1u << bit); raw = uv = -1;
        assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_RESPONSE);
        assert(diag.restored && s_samples == 1u && !strcmp(diag.failureStage,"SAMPLE_STATUS"));
        assert(raw == -1 && uv == -1);
    }
    state = reset(0u,0u,0x8Fu); s_sampleError = ESP_ERR_INVALID_CRC; s_restoreError = ESP_ERR_TIMEOUT;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_CRC);
    assert(diag.originalError == ESP_ERR_INVALID_CRC && diag.cleanupError == ESP_ERR_TIMEOUT && !diag.restored);
    assert(s_cacheInvalidated && s_batteryRestoreFailureLatched && s_snapshot.fallbackToBoundary);
    state = reset(0u,0u,0x8Fu); s_restoreError = ESP_ERR_TIMEOUT; raw = uv = -1;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_TIMEOUT);
    assert(raw == -1 && uv == -1 && diag.originalError == ESP_OK && diag.cleanupError == ESP_ERR_TIMEOUT);
    state = reset(0u,0u,0x8Fu); s_failStopCall = 2u;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_TIMEOUT);
    assert(diag.cleanupError == ESP_ERR_TIMEOUT && diag.restored && s_batteryRestoreFailureLatched);
    state = reset(0u,0u,0x8Fu); s_physical.power &= ~ADS126X_POWER_INTREF;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_STATE);
    assert(s_snapshotReads == 0u && s_stopCalls == 0u && s_restoreCalls == 0u);
    state = reset(0u,0u,0x8Fu); s_unclamped = false;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_STATE);
    assert(s_snapshotReads == 0u && s_stopCalls == 0u);
    state = reset(0u,0u,0x8Fu); s_powerError = ESP_FAIL;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_FAIL);
    assert(s_snapshotReads == 0u && s_stopCalls == 0u);
    state = reset(0u,0u,0x8Fu); s_failSnapshotRead = 1u;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_FAIL);
    assert(s_samples == 0u && s_stopCalls == 0u && s_restoreCalls == 0u);

    /* A bad read cannot consume one of the four discards or the final sample. */
    state = reset(0u,0u,0x8Fu); s_sampleErrors[0] = ESP_ERR_INVALID_CRC;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_OK);
    assert(s_samples == 6u && diag.acceptedSamples == 5u && diag.samplesRead == 5u);
    assert(diag.crcErrors == 1u && diag.crcRetries == 1u && diag.originalError == ESP_OK);
    assert(diag.firstCrc.received == 0x21u && diag.firstCrc.computed == 0x20u);
    assert(s_readGenerations[1] != s_readGenerations[0] && state.ads.adc1CrcRetryCount == 1u);
    assert(state.ads.transactionDeadlineUs == 1250000u && raw == 1109871204);

    /* The retry budget is shared across all logical reads, not reset per discard. */
    state = reset(0u,0u,0x8Fu); s_sampleErrors[0] = s_sampleErrors[3] = ESP_ERR_INVALID_CRC;
    raw = uv = -1;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_CRC);
    assert(s_samples == 4u && diag.acceptedSamples == 2u && diag.crcRetries == 1u && diag.crcErrors == 2u);
    assert(diag.firstCrc.drdyGeneration == s_readGenerations[0] && raw == -1 && uv == -1 && diag.restored);

    state = reset(0u,0u,0x8Fu); s_sampleError = ESP_ERR_INVALID_CRC; raw = uv = -1;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_CRC);
    assert(s_samples == 2u && diag.acceptedSamples == 0u && diag.crcRetries == 1u && diag.crcErrors == 2u);
    assert(raw == -1 && uv == -1 && diag.cleanupError == ESP_OK && diag.restored);

    state = reset(0u,0u,0x8Fu); s_sampleErrors[0] = ESP_ERR_INVALID_CRC; s_timeoutWaitCall = 2u;
    raw = uv = -1;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_CRC);
    assert(s_samples == 1u && diag.sampleTerminalError == ESP_ERR_TIMEOUT && diag.originalError == ESP_ERR_INVALID_CRC);
    assert(diag.restored && diag.cleanupError == ESP_OK && raw == -1 && uv == -1);
    assert(state.ads.transactionDeadlineUs == 1250000u && s_nowUs < 1250000);

    state = reset(0u,0u,0x8Fu); s_sampleErrors[0] = ESP_ERR_INVALID_CRC;
    state.ads.transactionDeadlineUs = 1001250u; raw = uv = -1;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_ERR_INVALID_CRC);
    assert(diag.sampleTerminalError == ESP_ERR_TIMEOUT && s_samples == 1u && s_nowUs == 1001250);
    assert(state.ads.transactionDeadlineUs == 1001250u && raw == -1 && uv == -1);

    state = reset(0u,0u,0x8Fu); s_sampleErrors[4] = ESP_ERR_INVALID_CRC;
    assert(sensorarrayAdsReadRailMonitorUv(&state,&raw,&uv,&diag) == ESP_OK);
    assert(diag.acceptedSamples == 5u && s_samples == 6u && diag.crcRetries == 1u && diag.restored);
    puts("ADS_RAIL_PROFILE_TESTS,passed=1,cases=33,source=production_functions_and_fresh_driver,fixture=synthetic");
    return 0;
}
