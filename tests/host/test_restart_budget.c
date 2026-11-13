
static void setExhaustedBudget(void)
{
    sensorarrayBootBreadcrumbMarkStableProgress(false);
    s_bootBreadcrumb.autoRestartCount = SENSORARRAY_BOOT_AUTO_RESTART_MAX;
    s_bootBreadcrumb.guardState = SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE;
    s_bootBreadcrumb.autoRestartWindowStartUs = mockRtcUs;
}

static void healthyAt(sensorarrayAppContext_t *ctx, uint64_t second)
{
    mockMonoUs = second * 1000000;
    ctx->frame.freshFrame = true;
    actualFrameProgress(ctx, ESP_OK, 1);
}

int main(void)
{
    sensorarrayAppContext_t ctx = {0};
    mockResetReason = ESP_RST_POWERON;
    mockRtcUs = 1000000;
    sensorarrayBootBreadcrumbStart();
    assert(s_bootBreadcrumb.autoRestartCount == 0);

    /* Real RTC breadcrumb and boot-start logic preserve budget across slow
     * software restarts. Each startup exceeds the old 30-second window. */
    for (unsigned i = 1; i <= SENSORARRAY_BOOT_AUTO_RESTART_MAX; ++i) {
        assert(sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO));
        assert(s_bootBreadcrumb.autoRestartCount == i);
        uint64_t firstAttemptUs = s_bootBreadcrumb.autoRestartWindowStartUs;
        uint32_t previousBootId = s_bootBreadcrumb.bootId;
        mockRtcUs += (30ULL * 1000000ULL) * 2;
        mockMonoUs = (30ULL * 1000000ULL) * 2;
        mockResetReason = ESP_RST_SW;
        sensorarrayBootBreadcrumbStart();
        assert(s_bootBreadcrumb.bootId == previousBootId + 1);
        assert(s_bootBreadcrumb.autoRestartCount == i);
        assert(s_bootBreadcrumb.autoRestartWindowStartUs == firstAttemptUs);
        actualReadyBlock(&ctx, ESP_OK, ESP_OK);
        assert(ctx.systemReady && s_bootBreadcrumb.autoRestartCount == i);
    }
    mockRtcUs += 3600ULL * 1000000;
    assert(!sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO));
    assert(s_bootBreadcrumb.autoRestartCount == 3);
    assert(s_bootBreadcrumb.guardState == SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE);
    actualReadyBlock(&ctx, ESP_OK, ESP_OK);
    assert(s_bootBreadcrumb.autoRestartCount == 3);
    assert(s_bootBreadcrumb.guardState == SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE);
    assert(readyEvents == 4);

    /* Idle time and READY cannot grant a fourth auto restart. */
    mockMonoUs += 3600ULL * 1000000;
    sensorarrayBootBreadcrumbMarkStableProgress(false);
    assert(!sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO));
    assert(s_bootBreadcrumb.autoRestartCount == 3);
    assert(sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_MANUAL));
    assert(s_bootBreadcrumb.autoRestartCount == 3); /* explicit action, not fabricated health */

    /* Exercise the exact production call site with plenty of healthy frames:
     * 119 seconds is insufficient even after exceeding 128 true frames. */
    healthyAt(&ctx, 1);
    for (uint64_t sample = 1; sample <= 238; ++sample) {
        mockMonoUs = 1000000 + sample * 500000;
        ctx.frame.freshFrame = true;
        actualFrameProgress(&ctx, ESP_OK, 1);
        assert(s_bootBreadcrumb.autoRestartCount == 3);
    }
    healthyAt(&ctx, 121);
    assert(s_bootBreadcrumb.autoRestartCount == 0);
    assert(s_bootBreadcrumb.guardState == SENSORARRAY_BOOT_GUARD_NORMAL);
    assert(s_bootBreadcrumb.autoRestartWindowStartUs == 0);
    assert(sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO));
    assert(s_bootBreadcrumb.autoRestartCount == 1);

    /* A valid frame after a 120-second complete stall cannot reset budget. */
    setExhaustedBudget();
    healthyAt(&ctx, 1000);
    healthyAt(&ctx, 1120);
    assert(s_bootBreadcrumb.autoRestartCount == 3);
    assert(s_bootBreadcrumb.guardState == SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE);

    /* Sparse observations restart qualification; neither 128 samples nor a
     * long wall time can conceal >30-second gaps. This is not a scan fault. */
    setExhaustedBudget();
    for (uint64_t sample = 0; sample < 256; ++sample) {
        healthyAt(&ctx, 2000 + sample * 31);
        assert(s_bootBreadcrumb.autoRestartCount == 3);
    }

    /* Even a continuous 126-second interval has only 127 frames. The next
     * actual frame is the 128th, and both conditions then permit reset. */
    setExhaustedBudget();
    for (uint64_t sample = 0; sample < 127; ++sample) {
        healthyAt(&ctx, 12000 + sample);
        assert(s_bootBreadcrumb.autoRestartCount == 3);
    }
    healthyAt(&ctx, 12127);
    assert(s_bootBreadcrumb.autoRestartCount == 0);

    /* Failed conversion, non-fresh data and no valid data each break the
     * consecutive health interval through the actual caller expression. */
    for (unsigned failure = 0; failure < 3; ++failure) {
        setExhaustedBudget();
        uint64_t start = 15000 + failure * 1000;
        for (uint64_t sample = 0; sample < 120; ++sample) {
            healthyAt(&ctx, start + sample);
        }
        mockMonoUs = (start + 120) * 1000000;
        ctx.frame.freshFrame = failure != 1;
        actualFrameProgress(&ctx, failure == 0 ? ESP_FAIL : ESP_OK,
                            failure == 2 ? 0 : 1);
        assert(s_bootBreadcrumb.autoRestartCount == 3);
        for (uint64_t sample = 0; sample < 127; ++sample) {
            healthyAt(&ctx, start + 121 + sample);
            assert(s_bootBreadcrumb.autoRestartCount == 3);
        }
        healthyAt(&ctx, start + 248);
        assert(s_bootBreadcrumb.autoRestartCount == 0);
    }

    /* READY breaks the current healthy interval and preserves budget. */
    setExhaustedBudget();
    for (uint64_t sample = 0; sample < 120; ++sample) {
        healthyAt(&ctx, 20000 + sample);
    }
    actualReadyBlock(&ctx, ESP_OK, ESP_OK);
    assert(s_bootBreadcrumb.autoRestartCount == 3);
    for (uint64_t sample = 0; sample < 127; ++sample) {
        healthyAt(&ctx, 20120 + sample);
        assert(s_bootBreadcrumb.autoRestartCount == 3);
    }
    healthyAt(&ctx, 20247);
    assert(s_bootBreadcrumb.autoRestartCount == 0);

    /* A new monotonic epoch cannot make subtraction pass immediately. */
    setExhaustedBudget();
    healthyAt(&ctx, 25000);
    for (uint64_t sample = 1; sample <= 127; ++sample) {
        healthyAt(&ctx, sample);
        assert(s_bootBreadcrumb.autoRestartCount == 3);
    }
    healthyAt(&ctx, 128);
    assert(s_bootBreadcrumb.autoRestartCount == 0);

    setExhaustedBudget();
    mockResetReason = ESP_RST_POWERON;
    sensorarrayBootBreadcrumbStart();
    assert(s_bootBreadcrumb.autoRestartCount == 0);
    assert(s_bootBreadcrumb.guardState == SENSORARRAY_BOOT_GUARD_NORMAL);
    puts("RESTART_BUDGET_PASS,productionFunctions=5,readyBlock=actual,frameCaller=actual,maxAuto=3,slowBootWindowReset=0,stableHealthyUs=120000000,minHealthyFrames=128,maxGapUs=30000000,stallAndSparseDoNotClear=1,failBreaksStreak=1,clockRollback=1,powerOnReset=1");
    return 0;
}
