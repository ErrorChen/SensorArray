#include "sensorarrayRouteController.h"

#include <string.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sensorarrayConfig.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayRoutePolicy.h"
#include "tmuxSwitch.h"

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

static esp_err_t sensorarrayRouteStopFdc(sensorarrayFdcDeviceState_t *fdc)
{
    if (!fdc || !fdc->ready || !fdc->handle) {
        return ESP_OK;
    }
    return Fdc2214CapEnterSleepWriteOnly(fdc->handle, fdc->configReg);
}

static esp_err_t sensorarrayRouteStopFrontends(sensorarrayRouteController_t *controller)
{
    if (!controller || !controller->state) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayState_t *state = controller->state;
    esp_err_t firstErr = ESP_OK;
    if (state->adsReady) {
        esp_err_t err = ads126xAdcStopAdc1(&state->ads);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
        state->adsAdc1Running = false;
        if (ads126xAdcHasAdc2(&state->ads)) {
            err = ads126xAdcStopAdc2(&state->ads);
            if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED && firstErr == ESP_OK) {
                firstErr = err;
            }
        }
    }
    esp_err_t err = sensorarrayRouteStopFdc(&state->fdcPrimary);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    err = sensorarrayRouteStopFdc(&state->fdcSecondary);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
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
    controller->snapshot.mode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    controller->snapshot.safe = true;
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
    esp_err_t firstErr = sensorarrayRouteStopFrontends(controller);
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

    esp_err_t err = sensorarrayRouteStopFrontends(controller);
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
        if (err == ESP_OK && controller->state->adsReady) {
            err = sensorarrayRouteConfigureAds(controller, &profile, rail);
        }
    } else if (err == ESP_OK) {
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
    esp_err_t err = ESP_OK;
    if (restoreExcitation) {
        /* REFOUT and the matrix REF net are the same physical node.  SW high
         * turns Q1 on and clamps that node to GND, so asserting SW while
         * INTREF is enabled would short the ADS reference output.  Stop the
         * converter and remove INTREF first, then clamp/select/release, and
         * only re-enable INTREF after the new row is connected. */
        err = ads126xAdcStopAdc1(&controller->state->ads);
        controller->state->adsAdc1Running = false;
        if (err == ESP_OK) {
            err = sensorarrayRouteDisableMatrixReference(controller);
        }
        if (err == ESP_OK) {
            err = tmux1108SetSource(TMUX1108_SOURCE_GND);
        }
        if (err == ESP_OK) {
            esp_rom_delay_us(CONFIG_TMUX1108_SWITCH_DELAY_US);
        }
    }
    if (err == ESP_OK) {
        err = tmux1108SelectRow((uint8_t)(row - 1u));
    }
    if (err == ESP_OK) {
        esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_ROW_SETTLE_US);
    }
    if (err == ESP_OK && restoreExcitation) {
        err = tmux1108SetSource(TMUX1108_SOURCE_REF);
        if (err == ESP_OK) {
            esp_rom_delay_us(CONFIG_TMUX1108_SWITCH_DELAY_US);
            err = ads126xAdcSetInternalReference(&controller->state->ads, true);
        }
        if (err == ESP_OK) {
            controller->state->adsRefReady = true;
            esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_ROW_SETTLE_US);
        }
    }
    tmuxSwitchControlState_t control = {0};
    if (err == ESP_OK) {
        err = tmuxSwitchGetControlState(&control);
    }
    if (err == ESP_OK &&
        (control.cmdRow != row - 1u || control.obsA0Level != (int)((row - 1u) & 1u) ||
         control.obsA1Level != (int)(((row - 1u) >> 1u) & 1u) ||
         control.obsA2Level != (int)(((row - 1u) >> 2u) & 1u) ||
         !sensorarrayRouteGpioMatches(&snapshot.profile, &control))) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK && restoreExcitation) {
        uint8_t power = 0u;
        uint8_t refmux = 0u;
        err = ads126xAdcReadCoreRegisters(&controller->state->ads,
                                          &power,
                                          NULL,
                                          NULL,
                                          NULL,
                                          &refmux);
        if (err == ESP_OK &&
            (((power & ADS126X_POWER_INTREF) == 0u) ||
             refmux != snapshot.profile.adsRefMux)) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
        if (err == ESP_OK) {
            sensorarrayRouteWriteBegin(controller);
            controller->snapshot.power = power;
            controller->snapshot.refmux = refmux;
            controller->snapshot.adsReadbackValid = true;
            sensorarrayRouteWriteEnd(controller);
        }
    }
    if (err != ESP_OK) {
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
