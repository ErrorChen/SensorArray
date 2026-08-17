#include "sensorarrayRouteController.h"

#include <string.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "sensorarrayConfig.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayRoutePolicy.h"
#include "tmuxSwitch.h"

static bool s_fdcSdGpioReady;

static bool sensorarrayRouteFdcSdGpioValid(void)
{
    int gpio = CONFIG_SENSORARRAY_FDC_SD_GPIO;
    return gpio >= 0 && gpio <= 48;
}

esp_err_t sensorarrayRouteControllerPrepareFdcSdGpio(void)
{
    if (s_fdcSdGpioReady) {
        return ESP_OK;
    }
    if (!sensorarrayRouteFdcSdGpioValid()) {
        printf("FDC_SD,stage=init,gpio=%d,ready=0,verified=0,reason=disabled\n",
               CONFIG_SENSORARRAY_FDC_SD_GPIO);
        return ESP_ERR_NOT_SUPPORTED;
    }
    gpio_config_t sdConfig = {
        .pin_bit_mask = 1ULL << CONFIG_SENSORARRAY_FDC_SD_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t sdErr = gpio_config(&sdConfig);
    if (sdErr == ESP_OK) {
        sdErr = gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_FDC_SD_GPIO, 0);
    }
    bool ready = sdErr == ESP_OK;
    bool verified =
        ready && gpio_get_level((gpio_num_t)CONFIG_SENSORARRAY_FDC_SD_GPIO) == 0;
    s_fdcSdGpioReady = ready && verified;
    printf("FDC_SD,stage=init,gpio=%d,sd=low,ready=%u,verified=%u,err=0x%lx\n",
           CONFIG_SENSORARRAY_FDC_SD_GPIO,
           ready ? 1u : 0u,
           verified ? 1u : 0u,
           (unsigned long)sdErr);
    if (!ready) {
        return sdErr;
    }
    return verified ? ESP_OK : ESP_FAIL;
}

static void sensorarrayRouteWriteBegin(sensorarrayRouteController_t *controller)
{
    __atomic_add_fetch(&controller->snapshotVersion, 1u, __ATOMIC_RELEASE);
}

static void sensorarrayRouteWriteEnd(sensorarrayRouteController_t *controller)
{
    __atomic_add_fetch(&controller->snapshotVersion, 1u, __ATOMIC_RELEASE);
}

static void sensorarrayRouteRecordError(sensorarrayRouteController_t *controller,
                                        esp_err_t err)
{
    if (!controller) {
        return;
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.lastError = (uint32_t)err;
    sensorarrayRouteWriteEnd(controller);
}

static esp_err_t sensorarrayRouteSetFdcSleep(sensorarrayFdcDeviceState_t *fdc,
                                             bool sleep,
                                             bool *outVerified)
{
    if (outVerified) {
        *outVerified = false;
    }
    if (!fdc || !fdc->ready || !fdc->handle) {
        return ESP_OK;
    }
    esp_err_t err = sleep ?
        Fdc2214CapEnterSleep(fdc->handle, fdc->configReg) :
        Fdc2214CapExitSleep(fdc->handle, fdc->configReg);
    if (outVerified) {
        *outVerified = err == ESP_OK;
    }
    return err;
}

static esp_err_t sensorarrayRouteSleepFdcFrontends(
    sensorarrayRouteController_t *controller,
    const char **outStage)
{
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayState_t *state = controller->state;
    esp_err_t firstErr = ESP_OK;
    bool primaryVerified = false;
    bool secondaryVerified = false;
    if (controller->snapshot.fdcSdHigh) {
        /* SD high is a deeper state than register sleep: both devices are
         * already shutdown and must not be touched over their now-dead I2C
         * paths until a device restart restores normal operation. */
        primaryVerified = true;
        secondaryVerified = true;
    } else {
        /* EnterSleep() verifies CONFIG by readback, which fails on a device
         * already in register sleep.  Skip the redundant transaction only
         * when this snapshot already confirms sleeping and verified. */
        bool primaryAlreadySleeping =
            controller->snapshot.fdcPrimarySleeping &&
            controller->snapshot.fdcPrimaryVerified;
        if (primaryAlreadySleeping) {
            primaryVerified = true;
        } else {
            esp_err_t err = sensorarrayRouteSetFdcSleep(&state->fdcPrimary,
                                                        true,
                                                        &primaryVerified);
            if (err != ESP_OK && firstErr == ESP_OK) {
                firstErr = err;
                if (outStage && (*outStage)[0] == '\0') {
                    *outStage = "fdc_primary_sleep";
                }
            }
        }
        bool secondaryAlreadySleeping =
            controller->snapshot.fdcSecondarySleeping &&
            controller->snapshot.fdcSecondaryVerified;
        if (secondaryAlreadySleeping) {
            secondaryVerified = true;
        } else {
            esp_err_t err = sensorarrayRouteSetFdcSleep(&state->fdcSecondary,
                                                        true,
                                                        &secondaryVerified);
            if (err != ESP_OK && firstErr == ESP_OK) {
                firstErr = err;
                if (outStage && (*outStage)[0] == '\0') {
                    *outStage = "fdc_secondary_sleep";
                }
            }
        }
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.fdcPrimarySleeping = primaryVerified;
    controller->snapshot.fdcPrimaryVerified = primaryVerified;
    controller->snapshot.fdcSecondarySleeping = secondaryVerified;
    controller->snapshot.fdcSecondaryVerified = secondaryVerified;
    sensorarrayRouteWriteEnd(controller);
    return firstErr;
}

static esp_err_t sensorarrayRouteStopFrontends(sensorarrayRouteController_t *controller,
                                                const char **outStage)
{
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayState_t *state = controller->state;
    esp_err_t firstErr = ESP_OK;
    if (outStage) {
        *outStage = "";
    }
    if (state->adsReady) {
        esp_err_t err = ads126xAdcStopAdc1(&state->ads);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
            if (outStage && (*outStage)[0] == '\0') {
                *outStage = "ads_stop";
            }
        }
        state->adsAdc1Running = false;
        if (ads126xAdcHasAdc2(&state->ads)) {
            err = ads126xAdcStopAdc2(&state->ads);
            if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED && firstErr == ESP_OK) {
                firstErr = err;
                if (outStage && (*outStage)[0] == '\0') {
                    *outStage = "ads_stop";
                }
            }
        }
    }
    esp_err_t err = sensorarrayRouteSleepFdcFrontends(controller, outStage);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    return firstErr;
}

static esp_err_t sensorarrayRouteWakeFdcFrontends(
    sensorarrayRouteController_t *controller)
{
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (controller->snapshot.fdcSdHigh) {
        printf("FDCISO,sd=high,reason=wake_after_isolation,restartRequired=1\n");
        /* SD shutdown resets every FDC register, so wake would both fail on
         * the dead I2C path and be insufficient even if it succeeded.  CAP
         * is recoverable only through a device restart. */
        return ESP_ERR_INVALID_STATE;
    }
    bool primaryVerified = false;
    bool secondaryVerified = false;
    esp_err_t firstErr = sensorarrayRouteSetFdcSleep(
        &controller->state->fdcPrimary, false, &primaryVerified);
    esp_err_t err = sensorarrayRouteSetFdcSleep(
        &controller->state->fdcSecondary, false, &secondaryVerified);
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.fdcPrimarySleeping = false;
    controller->snapshot.fdcPrimaryVerified = primaryVerified;
    controller->snapshot.fdcSecondarySleeping = false;
    controller->snapshot.fdcSecondaryVerified = secondaryVerified;
    sensorarrayRouteWriteEnd(controller);
    return firstErr;
}

static esp_err_t sensorarrayRouteDisableMatrixReference(
    sensorarrayRouteController_t *controller)
{
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayState_t *state = controller->state;
    if (!state->adsReady) {
        return ESP_OK;
    }
    /* On this board ADS REFOUT and the matrix REF net are the same node. SW
     * high turns Q1 on and clamps that node to GND, so INTREF must be disabled
     * before asserting the clamp. */
    esp_err_t err = ads126xAdcSetInternalReference(&state->ads, false);
    if (err == ESP_OK) {
        state->adsRefReady = false;
        esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_MODE_SETTLE_US);
    }
    return err;
}

static bool sensorarrayRouteGpioMatches(const sensorarrayBoardRouteProfile_t *profile,
                                        const tmuxSwitchControlState_t *control)
{
    if (!profile || !control) {
        return false;
    }
    int expectedSela = -1;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(profile->selaRoute, &expectedSela)) {
        return false;
    }
    int expectedSw = profile->swPhysicalLevel == SENSORARRAY_SW_PHYSICAL_HIGH ? 1 : 0;
    int expectedSelB = profile->selBLevel ? 1 : 0;
    sensorarrayRouteExpectedControl_t expected = {
        .logicalSource = profile->swLogicalSource,
        .swLevel = expectedSw,
        .selaLevel = expectedSela,
        .selbLevel = expectedSelB,
    };
    sensorarrayRouteObservedControl_t observed = {
        .commandedSource = control->cmdSource,
        .commandedSwLevel = control->cmdSwLevel,
        .observedSwLevel = control->obsSwLevel,
        .commandedSelaLevel = control->cmdSelaLevel,
        .observedSelaLevel = control->obsSelaLevel,
        .commandedSelbLevel = control->cmdSelbLevel,
        .observedSelbLevel = control->obsSelbLevel,
    };
    return sensorarrayRouteControlReadbackMatches(&expected, &observed);
}

static bool sensorarrayRouteAdsMatches(const sensorarrayBoardRouteProfile_t *profile,
                                       uint8_t power,
                                       uint8_t mode2,
                                       uint8_t refmux,
                                       uint8_t expectedGain)
{
    if (!profile) {
        return false;
    }
    bool expectedIntRef = profile->intRef == SENSORARRAY_ADS_INTREF_ON;
    bool expectedVbias = profile->vbias == SENSORARRAY_ADS_VBIAS_ON;
    uint8_t gain = 0u;
    return ((power & ADS126X_POWER_INTREF) != 0u) == expectedIntRef &&
           ((power & ADS126X_POWER_VBIAS) != 0u) == expectedVbias &&
           refmux == profile->adsRefMux &&
           ads126xAdcMode2DecodePgaGain(mode2, &gain) && gain == expectedGain;
}

static esp_err_t sensorarrayRouteApplyControlProfile(
    sensorarrayRouteController_t *controller,
    const sensorarrayBoardRouteProfile_t *profile)
{
    sensorarrayState_t *state = controller->state;
    /* Always remove matrix excitation before changing either TMUX1134 bank.
     * Disable REFOUT first; setting SW high first would short an enabled
     * REFOUT to GND through Q1. */
    esp_err_t err = sensorarrayRouteDisableMatrixReference(controller);
    if (err == ESP_OK) {
        err = tmux1108SetSource(TMUX1108_SOURCE_GND);
    }
    if (err == ESP_OK) {
        err = sensorarrayMeasureSetSelaPath(state,
                                            profile->selaRoute,
                                            0u,
                                            "mode_route",
                                            sensorarrayMeasurementModeName(profile->mode));
    }
    if (err == ESP_OK) {
        err = tmux1134SelectSelBLevel(profile->selBLevel);
    }
    if (err == ESP_OK) {
        err = tmux1134SetEnLogicalState(true);
    }
    return err;
}

static esp_err_t sensorarrayRouteConfigureAds(sensorarrayRouteController_t *controller,
                                              const sensorarrayBoardRouteProfile_t *profile,
                                              const sensorarrayAdsRailSplit_t *rail)
{
    if (!controller || !controller->state || !profile) {
        return ESP_ERR_INVALID_STATE;
    }
    sensorarrayState_t *state = controller->state;
    if (!state->adsReady) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    bool intRefEnabled = profile->intRef == SENSORARRAY_ADS_INTREF_ON;
    bool vbiasEnabled = profile->vbias == SENSORARRAY_ADS_VBIAS_ON;
    esp_err_t err = ads126xAdcConfigure(&state->ads,
                                        intRefEnabled,
                                        true,
                                        ADS126X_CRC_OFF,
                                        1u,
                                        (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE);
    if (err == ESP_OK) {
        err = ads126xAdcApplyPowerPolicy(&state->ads,
                                         true,
                                         intRefEnabled,
                                         true,
                                         vbiasEnabled,
                                         NULL,
                                         NULL);
    }
    bool usesInternalReference =
        profile->adsReferenceSource == SENSORARRAY_ADS_REFERENCE_INTERNAL;
    bool passiveWithoutRail =
        profile->mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE &&
        (!rail || !rail->valid);
    if (!usesInternalReference && !passiveWithoutRail &&
        (!rail || !rail->valid)) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t referenceUv = usesInternalReference ?
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_INTERNAL_REF_UV :
        (passiveWithoutRail ? state->ads.vrefMicrovolts :
         (uint32_t)((int64_t)rail->avddUv - rail->avssUv));
    if (err == ESP_OK) {
        err = passiveWithoutRail ?
            ads126xAdcSetRefMux(&state->ads, profile->adsRefMux) :
            ads126xAdcSetRefMuxWithVref(&state->ads,
                                        profile->adsRefMux,
                                        referenceUv);
    }
    if (err == ESP_OK) {
        state->adsRefMux = profile->adsRefMux;
        state->adsRefMuxValid = true;
        state->adsRefReady = profile->matrixExcitationEnabled;
    }
    return err;
}

static esp_err_t sensorarrayRouteCaptureAndVerify(sensorarrayRouteController_t *controller,
                                                  const sensorarrayBoardRouteProfile_t *profile,
                                                  const sensorarrayAdsRailSplit_t *rail,
                                                  bool verifyAds)
{
    tmuxSwitchControlState_t control = {0};
    esp_err_t err = tmuxSwitchGetControlState(&control);
    bool gpioValid = err == ESP_OK && sensorarrayRouteGpioMatches(profile, &control);
    uint8_t power = 0u;
    uint8_t mode2 = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    bool adsValid = !verifyAds;
    /* Always capture the registers when ADS is present so MODE?/STATE? is an
     * accurate snapshot even in SAFE/CAP. Verification is mode-dependent:
     * CAP does not consume ADS conversions and may survive a stale rail
     * calibration, while VOLT/RES may not. */
    if (err == ESP_OK && controller->state->adsReady) {
        err = ads126xAdcReadCoreRegisters(&controller->state->ads,
                                          &power,
                                          NULL,
                                          &mode2,
                                          &inpmux,
                                          &refmux);
        if (verifyAds) {
            adsValid = err == ESP_OK &&
                       sensorarrayRouteAdsMatches(profile, power, mode2, refmux, 1u);
        }
    }

    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.mode = profile->mode;
    controller->snapshot.profile = *profile;
    controller->snapshot.row = (uint8_t)(control.cmdRow + 1u);
    controller->snapshot.power = power;
    controller->snapshot.mode2 = mode2;
    controller->snapshot.inpmux = inpmux;
    controller->snapshot.refmux = refmux;
    controller->snapshot.pgaGain = verifyAds ? controller->state->ads.pgaGain : 0u;
    controller->snapshot.gpioReadbackValid = gpioValid;
    controller->snapshot.adsReadbackValid = adsValid;
    controller->snapshot.safe = profile->mode == SENSORARRAY_MEASUREMENT_MODE_NONE;
    if (rail) {
        controller->snapshot.avddUv = rail->avddUv;
        controller->snapshot.avssUv = rail->avssUv;
        controller->snapshot.railAgeFrames = rail->ageFrames;
        controller->snapshot.railValid = rail->valid;
    }
    sensorarrayRouteWriteEnd(controller);

    if (err != ESP_OK) {
        return err;
    }
    return gpioValid && adsValid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t sensorarrayRouteControllerInit(sensorarrayRouteController_t *controller,
                                         sensorarrayState_t *state)
{
    if (!controller || !state) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(controller, 0, sizeof(*controller));
    controller->state = state;
    controller->rowSettleUs =
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_ROW_SETTLE_US;
    controller->snapshot.mode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    controller->snapshot.safe = true;
    (void)sensorarrayRouteControllerPrepareFdcSdGpio();
    controller->snapshot.fdcSdVerified =
        sensorarrayRouteFdcSdGpioValid() && s_fdcSdGpioReady &&
        gpio_get_level((gpio_num_t)CONFIG_SENSORARRAY_FDC_SD_GPIO) == 0;
    return ESP_OK;
}

esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *controller,
                                              const char *reason)
{
    (void)reason;
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayBoardRouteProfile_t safeProfile;
    (void)sensorarrayBoardMapGetRouteProfile(SENSORARRAY_MEASUREMENT_MODE_NONE,
                                             &safeProfile);
    esp_err_t firstErr = sensorarrayRouteStopFrontends(controller, NULL);
    esp_err_t err = sensorarrayRouteDisableMatrixReference(controller);
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    if (err == ESP_OK) {
        err = tmux1108SetSource(TMUX1108_SOURCE_GND);
    }
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    err = sensorarrayMeasureSetSelaPath(controller->state,
                                        safeProfile.selaRoute,
                                        0u,
                                        "safe",
                                        "mode_safe");
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    err = tmux1134SelectSelBLevel(safeProfile.selBLevel);
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_MODE_SETTLE_US);
    err = sensorarrayRouteCaptureAndVerify(controller,
                                           &safeProfile,
                                           NULL,
                                           false);
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.safe = true;
    controller->snapshot.lastError = (uint32_t)firstErr;
    sensorarrayRouteWriteEnd(controller);
    return firstErr;
}

esp_err_t sensorarrayRouteControllerApplyMode(sensorarrayRouteController_t *controller,
                                              sensorarrayMeasurementMode_t mode,
                                              const sensorarrayAdsRailSplit_t *rail,
                                              uint64_t *outTransitionDurationUs)
{
    if (!controller || !controller->state ||
        !sensorarrayMeasurementModeIsDataMode(mode)) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t startUs = esp_timer_get_time();
    sensorarrayBoardRouteProfile_t profile;
    if (!sensorarrayBoardMapGetRouteProfile(mode, &profile)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = sensorarrayRouteStopFrontends(controller, NULL);
    if (err == ESP_OK) {
        err = sensorarrayRouteApplyControlProfile(controller, &profile);
    }
    if (err == ESP_OK && profile.swLogicalSource == TMUX1108_SOURCE_REF) {
        /* Release the Q1 clamp while REFOUT is still disabled. Enabling
         * INTREF in sensorarrayRouteConfigureAds() is then the final action
         * that energises the resistance divider. */
        err = tmux1108SetSource(TMUX1108_SOURCE_REF);
    }
    if (err == ESP_OK && mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        err = sensorarrayMeasurePrepareFdcMatrixPath(controller->state,
                                                     "mode_capacitance");
        if (err == ESP_OK) {
            err = sensorarrayRouteWakeFdcFrontends(controller);
        }
        if (err == ESP_OK && controller->state->adsReady) {
            err = sensorarrayRouteConfigureAds(controller, &profile, rail);
        }
    } else if (err == ESP_OK) {
        sensorarrayRouteSnapshot_t stopped = {0};
        if (!sensorarrayRouteControllerCopySnapshot(controller, &stopped) ||
            !stopped.fdcPrimaryVerified || !stopped.fdcSecondaryVerified ||
            !stopped.fdcPrimarySleeping || !stopped.fdcSecondarySleeping) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (err == ESP_OK && mode != SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        err = sensorarrayRouteConfigureAds(controller, &profile, rail);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_MODE_SETTLE_US);
        err = sensorarrayRouteCaptureAndVerify(controller,
                                               &profile,
                                               rail,
                                               mode != SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ||
                                                   controller->state->adsReady);
    }

    int64_t endUs = esp_timer_get_time();
    uint64_t durationUs = endUs > startUs ? (uint64_t)(endUs - startUs) : 0u;
    if (outTransitionDurationUs) {
        *outTransitionDurationUs = durationUs;
    }
    if (err != ESP_OK) {
        (void)sensorarrayRouteControllerEnterSafe(controller, "transition_failed");
        sensorarrayRouteRecordError(controller, err);
        return err;
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.generation++;
    controller->snapshot.transitionDurationUs = durationUs;
    controller->snapshot.lastError = 0u;
    controller->snapshot.safe = false;
    sensorarrayRouteWriteEnd(controller);
    return ESP_OK;
}

esp_err_t sensorarrayRouteControllerEnterSafeRailMonitor(
    sensorarrayRouteController_t *controller,
    uint64_t *outTransitionDurationUs)
{
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayBoardRouteProfile_t profile;
    if (!sensorarrayBoardMapGetSafeRailMonitorProfile(&profile)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    int64_t startUs = esp_timer_get_time();
    esp_err_t err = sensorarrayRouteStopFrontends(controller, NULL);
    if (err == ESP_OK) {
        err = sensorarrayRouteApplyControlProfile(controller, &profile);
    }
    if (err == ESP_OK) {
        err = tmux1108SetSource(TMUX1108_SOURCE_REF);
    }
    if (err == ESP_OK) {
        err = sensorarrayRouteConfigureAds(controller, &profile, NULL);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_MODE_SETTLE_US);
        err = sensorarrayRouteCaptureAndVerify(controller, &profile, NULL, true);
    }
    uint64_t durationUs = (uint64_t)(esp_timer_get_time() - startUs);
    if (outTransitionDurationUs) {
        *outTransitionDurationUs = durationUs;
    }
    if (err != ESP_OK) {
        (void)sensorarrayRouteControllerEnterSafe(controller, "rail_monitor_prepare_failed");
        return err;
    }
    return ESP_OK;
}

esp_err_t sensorarrayRouteControllerSelectRow(sensorarrayRouteController_t *controller,
                                             uint8_t row)
{
    if (!controller || !controller->state || row < 1u || row > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayRouteSnapshot_t snapshot;
    if (!sensorarrayRouteControllerCopySnapshot(controller, &snapshot) || snapshot.safe) {
        return ESP_ERR_INVALID_STATE;
    }
    bool restoreExcitation = snapshot.profile.matrixExcitationEnabled;
    bool useBreakBeforeMake = restoreExcitation &&
        CONFIG_SENSORARRAY_ADS_RES_BBM_ROW_SWITCH_ENABLE;
    esp_err_t err = ESP_OK;
    const char *failureStage = "none";
    uint8_t observedPower = snapshot.power;
    uint8_t observedRefmux = snapshot.refmux;
    if (restoreExcitation && !useBreakBeforeMake) {
        /* REFOUT and the matrix REF net are the same physical node.  SW high
         * turns Q1 on and clamps that node to GND, so asserting SW while
         * INTREF is enabled would short the ADS reference output.  Stop the
         * converter and remove INTREF first, then clamp/select/release, and
         * only re-enable INTREF after the new row is connected. */
        failureStage = "stop_adc1";
        err = ads126xAdcStopAdc1(&controller->state->ads);
        controller->state->adsAdc1Running = false;
        if (err == ESP_OK) {
            failureStage = "disable_intref";
            err = sensorarrayRouteDisableMatrixReference(controller);
        }
        if (err == ESP_OK) {
            failureStage = "clamp_gnd";
            err = tmux1108SetSource(TMUX1108_SOURCE_GND);
        }
        if (err == ESP_OK) {
            esp_rom_delay_us(CONFIG_TMUX1108_SWITCH_DELAY_US);
        }
    }
    if (err == ESP_OK) {
        failureStage = useBreakBeforeMake ? "bbm_select" : "safe_select";
        err = useBreakBeforeMake ?
            tmux1108SelectRowBreakBeforeMake((uint8_t)(row - 1u)) :
            tmux1108SelectRow((uint8_t)(row - 1u));
    }
    if (err == ESP_OK) {
        esp_rom_delay_us(controller->rowSettleUs);
    }
    if (err == ESP_OK && restoreExcitation && !useBreakBeforeMake) {
        failureStage = "release_ref";
        err = tmux1108SetSource(TMUX1108_SOURCE_REF);
        if (err == ESP_OK) {
            esp_rom_delay_us(CONFIG_TMUX1108_SWITCH_DELAY_US);
            failureStage = "enable_intref";
            err = ads126xAdcSetInternalReference(&controller->state->ads, true);
        }
        if (err == ESP_OK) {
            controller->state->adsRefReady = true;
            esp_rom_delay_us(controller->rowSettleUs);
        }
    }
    tmuxSwitchControlState_t control = {0};
    if (err == ESP_OK) {
        failureStage = "gpio_readback";
        err = tmuxSwitchGetControlState(&control);
    }
    if (err == ESP_OK &&
        (control.cmdRow != row - 1u || control.obsA0Level != (int)((row - 1u) & 1u) ||
         control.obsA1Level != (int)(((row - 1u) >> 1u) & 1u) ||
         control.obsA2Level != (int)(((row - 1u) >> 2u) & 1u) ||
         !sensorarrayRouteGpioMatches(&snapshot.profile, &control))) {
        failureStage = "gpio_mismatch";
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK && restoreExcitation && !useBreakBeforeMake) {
        /* The fast RES session does not modify POWER or REFMUX while changing
         * A[2:0]. MODE entry and the periodic register-shadow health check
         * verify those registers. Reading them on every row both wastes hot-
         * path SPI time and, with CS tied low, exposed transient DOUT bits at
         * 38.4 kSPS. The conservative path still verifies after it toggles
         * INTREF for each row. */
        failureStage = "ads_readback";
        err = ads126xAdcReadCoreRegisters(&controller->state->ads,
                                          &observedPower,
                                          NULL,
                                          NULL,
                                          NULL,
                                          &observedRefmux);
        if (err == ESP_OK &&
            (((observedPower & ADS126X_POWER_INTREF) == 0u) ||
             observedRefmux != snapshot.profile.adsRefMux)) {
            uint8_t retryPower = 0u;
            uint8_t retryRefmux = 0u;
            esp_err_t retryErr = ads126xAdcReadCoreRegisters(
                &controller->state->ads,
                &retryPower,
                NULL,
                NULL,
                NULL,
                &retryRefmux);
            if (retryErr == ESP_OK &&
                (retryPower & ADS126X_POWER_INTREF) != 0u &&
                retryRefmux == snapshot.profile.adsRefMux) {
                printf("ROUTE_ROW_READBACK_RETRY,row=%u,firstPower=0x%02X,firstRefmux=0x%02X,retryPower=0x%02X,retryRefmux=0x%02X,result=recovered\n",
                       (unsigned)row,
                       observedPower,
                       observedRefmux,
                       retryPower,
                       retryRefmux);
                observedPower = retryPower;
                observedRefmux = retryRefmux;
            } else {
                observedPower = retryPower;
                observedRefmux = retryRefmux;
                failureStage = retryErr == ESP_OK ?
                    "ads_mismatch" : "ads_retry_read";
                err = retryErr == ESP_OK ? ESP_ERR_INVALID_RESPONSE : retryErr;
            }
        }
        if (err == ESP_OK) {
            sensorarrayRouteWriteBegin(controller);
            controller->snapshot.power = observedPower;
            controller->snapshot.refmux = observedRefmux;
            controller->snapshot.adsReadbackValid = true;
            sensorarrayRouteWriteEnd(controller);
        }
    }
    if (err != ESP_OK) {
        tmux1108Source_t commandedSource = TMUX1108_SOURCE_GND;
        (void)tmux1108GetSource(&commandedSource);
        printf("ROUTE_ROW_FAIL,row=%u,stage=%s,err=0x%lx,bbm=%u,excitation=%u,cmdSource=%s,cmdRow=%u,obsSw=%d,power=0x%02X,intref=%u,refmux=0x%02X,expectedRefmux=0x%02X\n",
               (unsigned)row,
               failureStage,
               (unsigned long)err,
               useBreakBeforeMake ? 1u : 0u,
               restoreExcitation ? 1u : 0u,
               commandedSource == TMUX1108_SOURCE_REF ? "REF" : "GND",
               (unsigned)control.cmdRow,
               control.obsSwLevel,
               observedPower,
               (observedPower & ADS126X_POWER_INTREF) != 0u ? 1u : 0u,
               observedRefmux,
               snapshot.profile.adsRefMux);
        (void)sensorarrayRouteControllerEnterSafe(controller, "row_readback");
        sensorarrayRouteRecordError(controller, err);
        return err;
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.row = row;
    sensorarrayRouteWriteEnd(controller);
    return ESP_OK;
}

bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *controller,
                                            sensorarrayRouteSnapshot_t *outSnapshot)
{
    if (!controller || !outSnapshot) {
        return false;
    }
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt) {
        uint32_t before = __atomic_load_n(&controller->snapshotVersion, __ATOMIC_ACQUIRE);
        if ((before & 1u) != 0u) {
            continue;
        }
        *outSnapshot = controller->snapshot;
        uint32_t after = __atomic_load_n(&controller->snapshotVersion, __ATOMIC_ACQUIRE);
        if (before == after && (after & 1u) == 0u) {
            return true;
        }
    }
    return false;
}

void sensorarrayRouteControllerUpdateRailSnapshot(sensorarrayRouteController_t *controller,
                                                  const sensorarrayAdsRailSplit_t *rail)
{
    if (!controller || !rail) {
        return;
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.avddUv = rail->avddUv;
    controller->snapshot.avssUv = rail->avssUv;
    controller->snapshot.railAgeFrames = rail->ageFrames;
    controller->snapshot.railValid = rail->valid;
    sensorarrayRouteWriteEnd(controller);
}

bool sensorarrayRouteControllerSetRowSettleUs(sensorarrayRouteController_t *controller,
                                              uint32_t settleUs)
{
    if (!controller || settleUs > 10000u) {
        return false;
    }
    controller->rowSettleUs = settleUs;
    return true;
}

uint32_t sensorarrayRouteControllerGetRowSettleUs(
    const sensorarrayRouteController_t *controller)
{
    return controller ? controller->rowSettleUs : 0u;
}

esp_err_t sensorarrayRouteControllerForceFdcShutdown(
    sensorarrayRouteController_t *controller,
    bool *outVerified)
{
    if (outVerified) {
        *outVerified = false;
    }
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_fdcSdGpioReady) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    sensorarrayRouteSnapshot_t before = {0};
    if (!sensorarrayRouteControllerCopySnapshot(controller, &before)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (before.fdcSdHigh) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *stage = "";
    esp_err_t err = sensorarrayRouteStopFrontends(controller, &stage);
    sensorarrayRouteSnapshot_t stopped = {0};
    bool stoppedOk =
        sensorarrayRouteControllerCopySnapshot(controller, &stopped);
    if (err != ESP_OK || !stoppedOk ||
        !stopped.fdcPrimaryVerified || !stopped.fdcSecondaryVerified ||
        !stopped.fdcPrimarySleeping || !stopped.fdcSecondarySleeping) {
        if (stage[0] == '\0') {
            stage = stoppedOk ? "sleep_verify" : "snapshot";
        }
        printf("FDCISO_ERR,stage=%s,err=0x%lx,sd=low,primarySleeping=%u,primaryVerified=%u,secondarySleeping=%u,secondaryVerified=%u\n",
               stage,
               (unsigned long)err,
               stopped.fdcPrimarySleeping ? 1u : 0u,
               stopped.fdcPrimaryVerified ? 1u : 0u,
               stopped.fdcSecondarySleeping ? 1u : 0u,
               stopped.fdcSecondaryVerified ? 1u : 0u);
        return err != ESP_OK ? err : ESP_ERR_INVALID_RESPONSE;
    }

    err = gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_FDC_SD_GPIO, 1);
    if (err != ESP_OK) {
        printf("FDCISO_ERR,stage=fdc_sd_gpio,err=0x%lx,sd=low,readback=0,reason=command_failed\n",
               (unsigned long)err);
        return err;
    }
    bool sdHigh =
        gpio_get_level((gpio_num_t)CONFIG_SENSORARRAY_FDC_SD_GPIO) == 1;
    if (!sdHigh) {
        /* Pin readback is MCU-side evidence only, but a mismatch must not
         * leave a half-commanded level: restore the safe low state. */
        (void)gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_FDC_SD_GPIO, 0);
        printf("FDCISO_ERR,stage=fdc_sd_gpio,err=0x%lx,sd=low,readback=0,reason=readback_mismatch\n",
               (unsigned long)ESP_ERR_INVALID_RESPONSE);
    }
    sensorarrayRouteWriteBegin(controller);
    controller->snapshot.fdcSdHigh = sdHigh;
    controller->snapshot.fdcSdVerified = sdHigh;
    sensorarrayRouteWriteEnd(controller);
    if (outVerified) {
        *outVerified = sdHigh;
    }
    return sdHigh ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}
