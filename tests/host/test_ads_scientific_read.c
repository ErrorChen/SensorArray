/* Appended to the real driver + board harness. Hardware/IRQ timing is fake;
 * no test here claims a physical pad or analogue settling measurement. */
static ads126xAdcHandle_t scientificHandle(uint8_t value)
{
    ads126xAdcHandle_t h = handle();
    prepareFrame(&h, value);
    assert(ads126xAdcSetAdc1ReadPolicy(&h, ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED) == ESP_OK);
    return h;
}

static void releaseScientific(ads126xAdcHandle_t *h)
{
    pending = NULL; /* Synthetic driver completion only, never production cancel. */
    free(h->spiTxBuf); free(h->spiRxBuf);
}

static void testScientificEntries(void)
{
    for (unsigned kind = 0u; kind < 4u; ++kind) {
        ads126xAdcHandle_t h = scientificHandle((uint8_t)kind);
        notifyHandle = &h;
        uint8_t physical[32]; memcpy(physical, registers, sizeof(physical));
        int32_t raw = 0; uint8_t status = 0; uint32_t readUs = 0;
        if (kind & 1u) { ads126xAdcEndTransaction(&h); }
        uint64_t deadline = h.transactionDeadlineUs;
        esp_err_t err = kind < 2u ? ads126xAdcReadAdc1RawDmaReady(&h, &raw, &status, &readUs) :
            ads126xAdcReadAdc1Raw(&h, &raw, &status);
        assert(err == ESP_OK && raw == (int32_t)(0x12345600u | kind));
        assert(h.dmaTransaction.length == 55u && tailEdges == 2u);
        assert(h.transactionDeadlineUs == deadline && h.lastAdc1Tail.routeRestored);
        assert(memcmp(registers, physical, sizeof(physical)) == 0);
        releaseScientific(&h);
    }
    puts("PASS4 common ADC1 entries: ready/legacy x outer/noouter, no analogue register mutation");
}

static void testScientificCrcAndControls(void)
{
    for (unsigned kind = 0u; kind < 5u; ++kind) {
        ads126xAdcHandle_t h = scientificHandle((uint8_t)kind);
        notifyHandle = &h;
        int32_t raw = 123; uint8_t status = 45u;
        uint8_t budget = kind == 0u ? 1u : 0u;
        ads126xAdcFreshReadReport_t read = {0};
        if (kind == 0u) {
            memcpy(dataFrames[1], dataFrames[0], 6u); dataFrameLimit = 2u;
            notifyEdgesRemaining = 2u;
        }
        if (kind <= 1u) { dataFrames[0][5] ^= 1u; }
        esp_err_t err;
        if (kind == 2u) {
            err = ads126xAdcReadAdc1FreshSpiDiagnostic(&h, 0u, 600u, &budget, &raw, &status, &read);
        } else if (kind == 3u) {
            err = ads126xAdcReadAdc1RawSpiDiagnostic(&h, &raw, &status, NULL);
        } else {
            err = ads126xAdcReadAdc1FreshChecked(&h, 0u, 600u, &budget, &raw, &status, &read);
        }
        assert(h.adc1ReadPolicy == ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED);
        if (kind == 1u) {
            assert(err == ESP_ERR_INVALID_CRC && raw == 123 && status == 45u);
            assert(h.adc1CrcValidEvenCount + h.adc1CrcValidOddCount == 0u);
        } else { assert(err == ESP_OK); }
        if (kind <= 1u) {
            const ads126xAdcCrcFault_t *c = &read.firstCrcFault;
            assert(c->readSource == ADS126X_READ_SOURCE_SPI55_GPIO1 && !c->wireKnown);
            assert(c->gpioTailKnown && c->gpioTailBit == (dataFrames[0][5] & 1u));
            assert(c->spiPrefix[6] == (dataFrames[0][5] & 0xFEu));
            assert(c->bytes[5] == (uint8_t)(c->spiPrefix[6] | c->gpioTailBit));
            assert(c->tailStage == ADS126X_TAIL_STAGE_COMPLETE && read.firstError == ESP_ERR_INVALID_CRC);
            assert(read.crcErrors == 1u && h.adc1CrcErrorCount == 1u);
        }
        if (kind == 0u) {
            assert(read.crcRetries == 1u && read.terminalError == ESP_OK && budget == 0u);
            assert(dataFrameGeneration[1] > dataFrameGeneration[0]);
        }
        if (kind == 2u || kind == 3u) {
            assert(h.dmaTransaction.length == 56u && tailEdges == 0u && tailRouteChanges == 0u);
        } else { assert(h.dmaTransaction.length == 55u); }
        releaseScientific(&h);
    }
    puts("PASS5 selected fresh retry, bad-bit provenance, no bad output, explicit STD fresh/held controls");
}

static void testInterfaceInvalidation(void)
{
    for (unsigned kind = 0u; kind < 8u; ++kind) {
        ads126xAdcHandle_t h = scientificHandle(9u);
        notifyHandle = &h;
        uint8_t iface = kind == 1u ? 0x0Eu : 0x06u;
        if (kind <= 1u) {
            assert(ads126xAdcWriteRegisters(&h, 2u, &iface, 1u) == ESP_OK);
        } else if (kind == 2u) {
            uint8_t group[2] = {2u,6u};
            assert(ads126xAdcWriteRegisters(&h, 1u, group, 2u) == ESP_OK);
        } else if (kind == 3u) {
            assert(ads126xAdcSendCommand(&h, ADS126X_CMD_RESET) == ESP_OK);
        } else if (kind == 4u) {
            assert(ads126xAdcHardwareReset(&h) == ESP_OK);
        } else if (kind == 5u) {
            assert(ads126xAdcSetInputMuxFast(&h, 1u, 10u) == ESP_OK);
        } else if (kind == 6u) {
            queueError = ESP_FAIL;
            assert(ads126xAdcWriteRegisters(&h, 2u, &iface, 1u) == ESP_FAIL);
            queueError = ESP_OK;
        } else {
            registers[1] |= ADS126X_POWER_RESET;
            uint8_t power = 0;
            assert(ads126xAdcReadPowerRegister(&h, &power) == ESP_OK);
            assert((power & ADS126X_POWER_RESET) != 0u);
        }
        assert(h.interfaceKnown == (kind == 5u));
        unsigned before = queueCalls;
        int32_t raw = 123; uint8_t status = 45u;
        if (kind != 5u) {
            assert(ads126xAdcReadAdc1RawDmaReady(&h, &raw, &status, NULL) == ESP_ERR_INVALID_STATE);
            assert(queueCalls == before && raw == 123 && status == 45u && tailEdges == 0u);
            assert(h.lastAdc1Tail.stage == ADS126X_TAIL_STAGE_INTERFACE_PROOF);
        }
        if (kind == 7u) {
            assert(ads126xAdcClearResetFlag(&h) == ESP_OK && !h.interfaceKnown);
        }
        ads126xAdc1RegisterSnapshot_t snapshot;
        assert(ads126xAdcReadAdc1RegisterSnapshot(&h, &snapshot) == ESP_OK && h.interfaceKnown);
        esp_err_t readErr = ads126xAdcReadAdc1RawDmaReady(&h, &raw, &status, NULL);
        assert(readErr == (kind == 1u ? ESP_ERR_INVALID_STATE : ESP_OK));
        if (kind == 1u) {
            assert(ads126xAdcConfigure(&h, false, true, ADS126X_CRC_CRC8, 1u, 15u) == ESP_OK);
            assert(registers[2] == 6u && h.interfaceReg == 6u && h.interfaceKnown);
        }
        releaseScientific(&h);
    }
    puts("PASS8 INTERFACE single/block/failed write, timeout, command/hardware/POWER reset invalidate; MUX preserves; physical proof rebuilds");
}

static void testScientificHardFailures(void)
{
    for (unsigned kind = 0u; kind < 8u; ++kind) {
        ads126xAdcHandle_t h = scientificHandle(10u);
        notifyHandle = &h;
        if (kind == 0u) { tailOwnerAvailable = false; }
        if (kind == 1u) { completionBlocked = true; }
        if (kind == 2u) { tailRestoreFailure = true; }
        if (kind == 3u) {
            ads126xAdcEndTransaction(&h);
            lockDelayUs = 90000u; queueDelayUs = 8000u; completionDelayUs = 3000u;
        }
        if (kind == 4u) {
            h.transactionDeadlineUs = (uint64_t)nowUs + 50u;
            completionDelayUs = 100u;
        }
        if (kind >= 5u) {
            if (kind == 5u) { dataFrames[0][0] |= ADS126X_STATUS_RESET_OCCURRED; }
            dataFrames[0][5] ^= 1u;
            notifyEdgesRemaining = kind == 6u ? 0u : 2u;
            if (kind == 7u) { configChangeOnEdge = 2u; }
        }
        unsigned before = queueCalls;
        int32_t raw = 123; uint8_t status = 45u; esp_err_t err;
        if (kind >= 5u) {
            uint8_t retry = 1u; ads126xAdcFreshReadReport_t report;
            err = ads126xAdcReadAdc1FreshChecked(&h, 0u, 600u, &retry, &raw, &status, &report);
            assert(err == ESP_ERR_INVALID_CRC && report.terminalError ==
                (kind == 6u ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE));
            assert(queueCalls == before + 1u);
            if (kind == 5u) { assert(!h.interfaceKnown); }
        } else { err = ads126xAdcReadAdc1RawDmaReady(&h, &raw, &status, NULL); }
        assert(err != ESP_OK && raw == 123 && status == 45u);
        if (kind == 0u) { assert(queueCalls == before && tailEdges == 0u); }
        if (kind == 1u || kind == 2u || kind == 3u) {
            assert(ads126xAdcIsQuarantined(&h) && !h.interfaceKnown);
            before = queueCalls;
            assert(ads126xAdcReadAdc1Raw(&h, &raw, &status) != ESP_OK);
            assert(queueCalls == before);
        }
        if (kind == 4u) {
            assert(err == ESP_ERR_TIMEOUT && h.lastAdc1Tail.frameComplete && h.lastAdc1Tail.routeRestored);
            assert(!ads126xAdcIsQuarantined(&h));
        }
        releaseScientific(&h);
    }
    puts("PASS8 owner loss never falls back; DMA/route/deadline isolation; firstCRC then RESET/timeout/config change stays hard");
}

static bool injectedStdTailFault(const uint8_t *tx, uint8_t *rx, size_t bytes)
{
    if (tx[0] != ADS126X_CMD_RDATA1) { return false; }
    assert(bytes == 7u && pending != NULL);
    memcpy(rx + 1u, dataFrames[0], 6u);
    if (pending->length == 56u) { rx[6] ^= 1u; }
    return true;
}

static void testLastBitSyndrome(void)
{
    for (unsigned parity = 0u; parity < 2u; ++parity) {
        ads126xAdcHandle_t h = scientificHandle(0u); notifyHandle = &h;
        while ((dataFrames[0][5] & 1u) != parity) {
            dataFrames[0][4]++;
            dataFrames[0][5] = ads126xAdcCrc8(dataFrames[0] + 1u, 4u);
        }
        transactionHook = injectedStdTailFault;
        int32_t raw = 123; uint8_t status = 45u;
        assert(ads126xAdcReadAdc1RawSpiDiagnostic(&h, &raw, &status, NULL) == ESP_ERR_INVALID_CRC);
        assert(raw == 123 && status == 45u && h.lastAdc1CrcFault.wireKnown);
        assert(h.lastAdc1CrcFault.readSource == ADS126X_READ_SOURCE_SPI56);
        assert((h.lastAdc1CrcFault.received ^ h.lastAdc1CrcFault.computed) == 1u);
        assert(ads126xAdcReadAdc1RawDmaReady(&h, &raw, &status, NULL) == ESP_OK);
        assert(h.lastAdc1Tail.gpioBit == parity && h.lastAdc1Tail.assembledCrc == dataFrames[0][5]);
        assert(h.adc1CrcErrorCount == 1u && h.adc1CrcValidEvenCount + h.adc1CrcValidOddCount == 1u);
        releaseScientific(&h);
    }
    puts("PASS2 actual SPI56 rejection vs selected55+physical-bit acceptance under injected0-to1/1-to0 tail sampling faults");
}

static void testPolicyAndRestore(void)
{
    for (unsigned kind = 0u; kind < 8u; ++kind) {
        ads126xAdcHandle_t h = handle(); prepareFrame(&h, 7u); notifyHandle = &h;
        if (kind == 0u) { h.deviceIdentityKnown = false; }
        if (kind == 1u) { h.deviceType = ADS126X_DEVICE_ADS1263; }
        if (kind == 2u) { h.adc1Running = true; }
        if (kind == 3u) { h.interfaceKnown = false; }
        if (kind == 4u) { h.crcMode = ADS126X_CRC_CHECKSUM; }
        if (kind == 5u) { tailOwnerAvailable = false; }
        esp_err_t err = ads126xAdcSetAdc1ReadPolicy(&h, ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED);
        if (kind < 6u) { assert(err != ESP_OK && h.adc1ReadPolicy == ADS126X_ADC1_READ_SPI_BYTES); }
        else {
            assert(err == ESP_OK);
            ads126xAdc1RegisterSnapshot_t saved;
            assert(ads126xAdcReadAdc1RegisterSnapshot(&h, &saved) == ESP_OK);
            if (kind == 7u) { corruptReadRegister = 0x06u; }
            err = ads126xAdcRestoreAdc1RegisterSnapshot(&h, &saved, h.vrefMicrovolts, false);
            assert((err == ESP_OK) == (kind == 6u));
            assert(h.interfaceKnown == (kind == 6u));
            assert(h.adc1ReadPolicy == ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED);
        }
        releaseScientific(&h);
    }
    puts("PASS8 identity/type/running/interface/shadow/owner admission and physical restore success/mismatch");
}

static void testProductionBringup(void)
{
    for (unsigned kind = 0u; kind < 6u; ++kind) {
        ads126xAdcHandle_t reset = handle();
        releaseScientific(&reset);
        presenceBusy = presenceQuarantined = false;
        busInitError = busAddError = ESP_OK;
        registers[0] = kind == 1u ? 0xFFu : kind == 2u ? 0x23u : 0x03u;
        if (kind == 3u) { busInitError = ESP_ERR_INVALID_STATE; }
        if (kind == 4u) { presenceBusy = true; }
        if (kind == 5u) { corruptReadRegister = 2u; }
        sensorarrayState_t state = {0};
        esp_err_t err = sensorarrayBringupInitAds(&state);
#if CONFIG_SENSORARRAY_ADS_GPIO_TAIL_READ_ENABLE
        assert((err == ESP_OK) == (kind == 0u));
        if (err == ESP_OK) {
            assert(state.ads.adc1ReadPolicy == ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED);
            assert(state.ads.deviceIdentityKnown && state.ads.idRegRaw == 0x03u);
            assert(state.ads.interfaceKnown && state.ads.interfaceReg == 6u);
        }
#else
        assert((err == ESP_OK) == (kind != 5u));
        if (err == ESP_OK) { assert(state.ads.adc1ReadPolicy == ADS126X_ADC1_READ_SPI_BYTES); }
#endif
        if (err == ESP_OK) {
            assert(state.ads.enableStatusByte && state.ads.crcMode == ADS126X_CRC_CRC8);
            assert(!state.ads.enableInternalRef);
            assert(ads126xAdcDeinit(&state.ads) == ESP_OK);
        }
    }
#if CONFIG_SENSORARRAY_ADS_GPIO_TAIL_READ_ENABLE
    sensorarrayState_t state = {0}; unsigned before = queueCalls;
    assert(sensorarrayBringupAttachAdsNoReset(&state) == ESP_ERR_NOT_SUPPORTED);
    assert(queueCalls == before);
#endif
    printf("PASS6 actual Bringup enabled=%u: STATUS unified, physical identity proof, no implicit fallback\n",
        (unsigned)CONFIG_SENSORARRAY_ADS_GPIO_TAIL_READ_ENABLE);
}

static void testBatteryRestoreProof(void)
{
    for (unsigned kind = 0u; kind < 4u; ++kind) {
        ads126xAdcHandle_t h = scientificHandle(12u);
        notifyHandle = &h;
        ads126xAdc1RegisterSnapshot_t saved;
        assert(ads126xAdcReadAdc1RegisterSnapshot(&h, &saved) == ESP_OK);
        assert(ads126xAdcSetInputMux(&h, 8u, 10u) == ESP_OK);
        assert(ads126xAdcSetMode2Fast(&h, 0x8Du) == ESP_OK);
        if (kind == 1u) { corruptReadRegister = 2u; }
        if (kind == 2u) { queueError = ESP_FAIL; }
        if (kind == 3u) { h.interfaceKnown = false; }
        esp_err_t err = sensorarrayAdsRestoreBatteryCoreState(&h, &saved,
            h.vrefMicrovolts, false, saved.power, 0x8Du, 0x8Au, saved.refmux, NULL);
        assert((err == ESP_OK) == (kind == 0u || kind == 3u));
        assert(h.interfaceKnown == (err == ESP_OK));
        assert(h.adc1ReadPolicy == ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED);
        if (err == ESP_OK) {
            assert(h.interfaceReg == 6u && registers[6] == saved.inpmux && registers[5] == saved.mode2);
        } else {
            unsigned before = queueCalls; int32_t raw = 0;
            assert(ads126xAdcReadAdc1RawDmaReady(&h, &raw, NULL, NULL) != ESP_OK);
            assert(before == queueCalls);
        }
        releaseScientific(&h);
    }
    puts("PASS4 actual fast battery restore: physical INTERFACE verified/rebuilt; failures block next RDATA");
}

static void testActualRailProvenance(void)
{
    for (unsigned kind = 0u; kind < 6u; ++kind) {
        ads126xAdcCrcFault_t fault = {
            .readSource = kind < 2u ? ADS126X_READ_SOURCE_SPI56 : ADS126X_READ_SOURCE_SPI55_GPIO1,
            .wireKnown = kind == 0u, .gpioTailKnown = kind >= 2u && kind != 4u,
            .gpioTailBit = kind == 3u ? 1u : kind == 5u ? 2u : 0u,
            .tailStage = ADS126X_TAIL_STAGE_COMPLETE, .length = 6u,
            .bytes={0x60u,0x12u,0x34u,0x56u,0x78u,0x80u},
            .spiPrefix={0u,0x60u,0x12u,0x34u,0x56u,0x78u,0x80u},
            .received=0x80u,.computed=0x81u,.timestampUs=UINT64_MAX,
            .configurationGeneration=UINT32_MAX,.drdyGeneration=UINT32_MAX,
        };
        actualRailLog(UINT32_MAX, UINT32_MAX, &fault);
        assert(strlen(railOutput) < 384u);
        if (kind == 0u) { assert(strstr(railOutput,"bytes=601234567880") && strstr(railOutput,"wireKnown=1")); }
        else { assert(strstr(railOutput,"bytes=unknown") && strstr(railOutput,"wireKnown=0")); }
        if (kind == 2u || kind == 3u) {
            assert(strstr(railOutput,"assembledBytes=601234567880,spiPrefix=00601234567880,gpioKnown=1"));
            assert(strstr(railOutput, kind == 2u ? "gpioBit=0," : "gpioBit=1,"));
        }
        if (kind >= 4u) {
            assert(strstr(railOutput,"assembledBytes=unknown,spiPrefix=unknown,gpioKnown=0,gpioBit=unknown"));
        }
    }
    puts("PASS6 actual RAILCRC log block: split/STD/unknown/invalid-bit provenance, maximum integers <384B");
}

int main(void)
{
    testScientificEntries();
    testScientificCrcAndControls();
    testInterfaceInvalidation();
    testScientificHardFailures();
    testPolicyAndRestore();
    testProductionBringup();
    testBatteryRestoreProof();
    testActualRailProvenance();
    testLastBitSyndrome();
    puts("ADS_SCIENTIFIC_TRANSPORT_TESTS,passed=51,source=complete_driver_actual_bringup_battery_restore_rail_log,fixture=synthetic");
    return 0;
}
