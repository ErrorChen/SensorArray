// Split from core/measure/sensorarrayMeasure.c to keep measurement domains isolated.
esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t *state,
                                        sensorarraySelaRoute_t selaRoute,
                                        uint32_t settleDelayMs,
                                        const char *stage,
                                        const char *label)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    int selaWriteLevel = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayMeasureWriteSela(state, selaRoute, settleDelayMs, stage, label);
}

esp_err_t sensorarrayMeasureApplyRouteLevels(sensorarrayState_t *state,
                                             uint8_t sColumn,
                                             uint8_t dLine,
                                             sensorarrayRoutePathKind_t path,
                                             tmux1108Source_t swSource,
                                             sensorarraySelaRoute_t selaRoute,
                                             bool selBLevel,
                                             uint32_t delayAfterRowMs,
                                             uint32_t delayAfterSelAMs,
                                             uint32_t delayAfterSelBMs,
                                             uint32_t delayAfterSwMs,
                                             const char *label)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    if (swSource == TMUX1108_SOURCE_REF) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ref_guard",
                                              sColumn,
                                              dLine,
                                              path,
                                              TMUX1108_SOURCE_GND,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw_pre_ref_guard",
                                              "isolate_ref_before_route");
        if (err != ESP_OK) {
            return err;
        }
        err = sensorarrayMeasureApplyRefPolicy(state,
                                               "enter_ref_mode",
                                               "route_apply_levels",
                                               SENSORARRAY_MATRIX_D_SOURCE_REF,
                                               SENSORARRAY_ADS_INTREF_ON,
                                               SENSORARRAY_ADS_VBIAS_ON,
                                               "ref_d_before_connect");
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ground",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw_pre_ground",
                                              "ground_d_before_route");
        if (err != ESP_OK) {
            return err;
        }
        if (path == SENSORARRAY_ROUTE_PATH_CAPACITIVE) {
            sensorarrayAdsIntRefPolicy_t intrefPolicy =
                state->adsReady ? SENSORARRAY_ADS_INTREF_OFF : SENSORARRAY_ADS_INTREF_KEEP;
            sensorarrayAdsVbiasPolicy_t vbiasPolicy =
                state->adsReady ? SENSORARRAY_ADS_VBIAS_OFF : SENSORARRAY_ADS_VBIAS_KEEP;
            err = sensorarrayMeasureApplyRefPolicy(state,
                                                   "cap_fdc_ref_off",
                                                   "route_apply_levels",
                                                   SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                   intrefPolicy,
                                                   vbiasPolicy,
                                                   "fdc_cap_path_no_ads_ref");
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterRowMs);

    err = sensorarrayMeasureSetSelaPath(state, selaRoute, delayAfterSelAMs, "selA", label);
    sensorarrayLogRouteStep("selA",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            "set_sela_path");
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(selBLevel);
    sensorarrayLogRouteStep("selB", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSelBMs);

    if (swSource == TMUX1108_SOURCE_GND) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_final_assert",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "ok",
                                              "final_ground_assert");
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw",
                                              "connect_ref_after_route");
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSwMs);

    sensorarrayLogDbgExtraCaptureCtrl();
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyRoute(sensorarrayState_t *state,
                                       uint8_t sColumn,
                                       uint8_t dLine,
                                       sensorarrayPath_t path,
                                       tmux1108Source_t swSource,
                                       const char **outMapLabel)
{
    if (outMapLabel) {
        *outMapLabel = SENSORARRAY_NA;
    }
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    const sensorarrayRouteMap_t *routeMap = sensorarrayBoardMapFindRoute(sColumn, dLine, path);
    if (!routeMap) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    sensorarrayRoutePathKind_t routePath = sensorarrayBoardMapPathToRoutePath(path, swSource);
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(routeMap->selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_STATE;
    }

    printf("ROUTEDEBUG,stage=apply_begin,label=%s,sColumn=%u,dLine=%u,path=%s,sw=%s,selaRequest=%s,"
           "selBLevel=%u,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n",
           routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogRoutePathName(routePath),
           sensorarrayLogSwSourceName(swSource),
           sensorarrayBoardMapSelaRouteName(routeMap->selaRoute),
           routeMap->selBLevel ? 1u : 0u);

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                                           routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    if (swSource == TMUX1108_SOURCE_REF) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ref_guard",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              TMUX1108_SOURCE_GND,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw_pre_ref_guard",
                                              "isolate_ref_before_route");
        if (err != ESP_OK) {
            return err;
        }
        err = sensorarrayMeasureApplyRefPolicy(state,
                                               "enter_ref_mode",
                                               "route_apply",
                                               SENSORARRAY_MATRIX_D_SOURCE_REF,
                                               SENSORARRAY_ADS_INTREF_ON,
                                               SENSORARRAY_ADS_VBIAS_ON,
                                               "ref_d_before_connect");
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ground",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw_pre_ground",
                                              "ground_d_before_route");
        if (err != ESP_OK) {
            return err;
        }
        if (path == SENSORARRAY_PATH_CAPACITIVE) {
            sensorarrayAdsIntRefPolicy_t intrefPolicy =
                state->adsReady ? SENSORARRAY_ADS_INTREF_OFF : SENSORARRAY_ADS_INTREF_KEEP;
            sensorarrayAdsVbiasPolicy_t vbiasPolicy =
                state->adsReady ? SENSORARRAY_ADS_VBIAS_OFF : SENSORARRAY_ADS_VBIAS_KEEP;
            err = sensorarrayMeasureApplyRefPolicy(state,
                                                   "cap_fdc_ref_off",
                                                   "route_apply",
                                                   SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                   intrefPolicy,
                                                   vbiasPolicy,
                                                   "fdc_cap_path_no_ads_ref");
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_COLUMN_MS);

    err = sensorarrayMeasureSetSelaPath(state,
                                        routeMap->selaRoute,
                                        SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                        "selA",
                                        routeMap->mapLabel);
    sensorarrayLogRouteStep("selA",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_sela_path");
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(routeMap->selBLevel);
    sensorarrayLogRouteStep("selB",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_PATH_MS);

    if (swSource == TMUX1108_SOURCE_GND) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_final_assert",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "ok",
                                              "final_ground_assert");
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw",
                                              "connect_ref_after_route");
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_SW_MS);

    sensorarrayLogDbgExtraCaptureCtrl();

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    printf("ROUTEDEBUG,stage=apply_done,label=%s,sColumn=%u,dLine=%u,path=%s,selaRequest=%s,"
           "selBLevel=%u,err=0,status=ok,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n",
           routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogRoutePathName(routePath),
           sensorarrayBoardMapSelaRouteName(routeMap->selaRoute),
           routeMap->selBLevel ? 1u : 0u);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyFdcCellRoute(sensorarrayState_t *state,
                                              const sensorarrayFdcCellTarget_t *target,
                                              const char *reason)
{
    if (!state || !target ||
        !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine) ||
        target->fdcChannel > (uint8_t)FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }

    bool selBLevel = false;
    if (!sensorarrayBoardMapFdcSelBLevel(&selBLevel)) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *source = reason ? reason : SENSORARRAY_NA;
    printf("FDC_CELL_ROUTE,stage=begin,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           source);

    uint32_t settleMs = ((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US + 999u) / 1000u;
    esp_err_t err = sensorarrayMeasureApplyRouteLevels(state,
                                                       target->sColumn,
                                                       target->dLine,
                                                       SENSORARRAY_ROUTE_PATH_CAPACITIVE,
                                                       TMUX1108_SOURCE_GND,
                                                       SENSORARRAY_SELA_ROUTE_FDC2214,
                                                       selBLevel,
                                                       settleMs,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_SW_MS,
                                                       target->mapLabel ? target->mapLabel : "fdc_cell_route");
    if (err == ESP_OK) {
        uint8_t discardCount = (uint8_t)CONFIG_SENSORARRAY_FDC_MATRIX_DISCARD_SAMPLES;
        if (discardCount < SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT) {
            discardCount = SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT;
        }
        err = sensorarrayMeasureFdcDiscardStaleSamples(state, target, discardCount, "row_switch");
    }

    printf("FDC_CELL_ROUTE,stage=done,s=%u,d=%u,index=%u,device=%s,ch=%u,err=0x%lx,status=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           (unsigned long)err,
           (err == ESP_OK) ? "ok" : "failed");
    return err;
}

esp_err_t sensorarrayMeasureReadAdsRawWithRetry(sensorarrayState_t *state,
                                                int32_t *outRaw,
                                                uint8_t retryCount,
                                                bool *outTimedOut,
                                                uint8_t *outStatusByte)
{
    if (!state || !outRaw) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outTimedOut) {
        *outTimedOut = false;
    }
    if (outStatusByte) {
        *outStatusByte = 0u;
    }

    for (uint8_t attempt = 0; attempt <= retryCount; ++attempt) {
        esp_err_t err = ads126xAdcReadAdc1Raw(&state->ads, outRaw, outStatusByte);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err == ESP_ERR_TIMEOUT && outTimedOut) {
            *outTimedOut = true;
        }

        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)retryCount,
               (long)err);

        if (attempt == retryCount) {
            return err;
        }
        sensorarrayDelayMs(1u);
    }

    return ESP_FAIL;
}

esp_err_t sensorarrayMeasureReadAdsPairUv(sensorarrayState_t *state,
                                          const sensorarrayAdsReadPolicy_t *policy,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          bool discardFirst,
                                          int32_t *outRaw,
                                          int32_t *outUv,
                                          uint8_t *outStatusByte)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    const sensorarrayAdsReadPolicy_t *readPolicy = sensorarrayReadPolicyOrDefault(policy);

    muxp &= 0x0Fu;
    muxn &= 0x0Fu;

    sensorarrayLogDbgExtraSetMux(muxp, muxn);
    if (state->adsRefMuxValid) {
        sensorarrayLogDbgExtraSetRefMux(state->adsRefMux);
    }

    uint16_t totalDiscard = (uint16_t)readPolicy->baseDiscardCount + (discardFirst ? 1u : 0u);
    if (totalDiscard > 255u) {
        totalDiscard = 255u;
    }
    sensorarrayLogDbgExtraSetDiscardCount((uint8_t)totalDiscard);

    printf("DBGADSSEQ,step=set_input_mux,muxp=%u(%s),muxn=%u(%s),discardFirst=%u,discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    if (readPolicy->stopBeforeMuxChange && state->adsAdc1Running) {
        esp_err_t stopErr = ads126xAdcStopAdc1(&state->ads);
        if (stopErr == ESP_OK) {
            state->adsAdc1Running = false;
        } else {
            printf("DBGADSSEQ,step=stop1,err=%ld,status=stop_error\n", (long)stopErr);
            return stopErr;
        }
    }

    printf("DBGADSSEQ,step=read,muxp=%u(%s),muxn=%u(%s),discardFirst=%u,discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    const bool startEachRead = readPolicy->startEveryRead || !state->adsAdc1Running;
    bool readTimedOut = false;
    uint8_t statusByte = 0u;
    esp_err_t err = ESP_FAIL;
    for (uint8_t attempt = 0u; attempt <= readPolicy->readRetryCount; ++attempt) {
        err = ads126xAdcReadSingleDiffUv(&state->ads,
                                         muxp,
                                         muxn,
                                         startEachRead,
                                         readPolicy->settleAfterMuxMs,
                                         (uint8_t)totalDiscard,
                                         outRaw,
                                         outUv,
                                         &statusByte);
        if (err == ESP_OK) {
            state->adsAdc1Running = true;
            break;
        }
        readTimedOut = (err == ESP_ERR_TIMEOUT);
        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,muxp=%u,muxn=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)readPolicy->readRetryCount,
               (unsigned)muxp,
               (unsigned)muxn,
               (long)err);
        if (attempt < readPolicy->readRetryCount) {
            sensorarrayDelayMs(1u);
        }
    }

    if (err != ESP_OK) {
        printf("DBGADSREAD,status=error,muxp=%u(%s),muxn=%u(%s),refmux=0x%02X,discardCount=%u,drdyTimeout=%u,err=%ld\n",
               (unsigned)muxp,
               sensorarrayLogAdsMuxName(muxp),
               (unsigned)muxn,
               sensorarrayLogAdsMuxName(muxn),
               state->adsRefMuxValid ? state->adsRefMux : 0u,
               (unsigned)totalDiscard,
               readTimedOut ? 1u : 0u,
               (long)err);
        return err;
    }

    if (outStatusByte) {
        *outStatusByte = statusByte;
    }

    printf("DBGADSSEQ,step=read_done,muxp=%u(%s),muxn=%u(%s),raw=%ld,uv=%ld,statusByte=0x%02X,discardFirst=%u,"
           "discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           (long)*outRaw,
           (long)*outUv,
           statusByte,
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);
    sensorarrayLogDbgExtraCaptureCtrl();
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadAdsUv(sensorarrayState_t *state,
                                      const sensorarrayAdsReadPolicy_t *policy,
                                      uint8_t dLine,
                                      bool discardFirst,
                                      int32_t *outRaw,
                                      int32_t *outUv)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayBoardMapAdsMuxForDLine(dLine, &muxp, &muxn)) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayMeasureReadAdsPairUv(state, policy, muxp, muxn, discardFirst, outRaw, outUv, NULL);
}

sensorarrayResConvertResult_t sensorarrayMeasureTryResistanceMohm(int32_t uv, int32_t *outMohm)
{
    if (!outMohm) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }
    if (uv < 0) {
        return SENSORARRAY_RES_CONVERT_SIGNED_INPUT;
    }
    if (uv == 0 || (uint32_t)uv >= SENSORARRAY_RESIST_EXCITATION_UV) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    int64_t numerator = (int64_t)SENSORARRAY_RESIST_REF_OHMS * 1000 * (int64_t)uv;
    int64_t denominator = (int64_t)SENSORARRAY_RESIST_EXCITATION_UV - (int64_t)uv;
    if (denominator == 0) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    *outMohm = (int32_t)(numerator / denominator);
    return SENSORARRAY_RES_CONVERT_OK;
}

const char *sensorarrayMeasureDividerModelStatus(int32_t uv, int32_t *outMohm, bool *outHaveMohm)
{
    if (outHaveMohm) {
        *outHaveMohm = false;
    }

    sensorarrayResConvertResult_t resResult = sensorarrayMeasureTryResistanceMohm(uv, outMohm);
    if (resResult == SENSORARRAY_RES_CONVERT_OK) {
        if (outHaveMohm) {
