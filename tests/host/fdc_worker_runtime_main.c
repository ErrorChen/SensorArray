#undef printf
static void *threadMain(void *arg)
{
    sensorarrayFdcWorkerContext_t *ctx = arg;
    currentTask = ctx->task;
    sensorarrayMeasureFdcWorkerTask(ctx);
    return NULL;
}
int main(void)
{
    for (scenario = 0; scenario <= 6; ++scenario) {
        memset(s_fdcWorkers, 0, sizeof(s_fdcWorkers));
        memset(runCount, 0, sizeof(runCount)); memset(consumed, 0, sizeof(consumed));
        eventBits = SENSORARRAY_FDC_WORKER_DONE_ALL_BITS; /* stale wakeup */
        TestTask tasks[2] = {0};
        for (unsigned i = 0; i < 2; ++i) {
            tasks[i].id = i;
            pthread_mutex_init(&tasks[i].mutex, NULL); pthread_cond_init(&tasks[i].cond, NULL);
            s_fdcWorkers[i].task = &tasks[i]; s_fdcWorkers[i].devId = i; s_fdcWorkers[i].initialized = true;
            appState.devices[i].ready = true; appState.devices[i].handle = &appState;
            pthread_create(&tasks[i].thread, NULL, threadMain, &s_fdcWorkers[i]);
        }
        unsigned repeats = scenario == 0 ? 50u : 1u;
        for (unsigned iteration = 1; iteration <= repeats; ++iteration) {
            sensorarrayFdcAutoscanSamples_t outputs[2] = {{0xdead}, {0xbeef}};
            sensorarrayFdcRuntimeChannelConfig_t configs[2][4] = {0};
            sensorarrayFdcDeviceTiming_t timing[2] = {0};
            sensorarrayFdcRowTiming_t row = {0};
            sensorarrayFdcFrameReadTracker_t tracker = {.frameSeq = iteration};
            bool timeout = false;
            int64_t start = esp_timer_get_time();
            esp_err_t err = sensorarrayMeasureReadFdcMatrixRowParallelEpoch(&appState, 1, iteration, iteration,
                &outputs[0], &outputs[1], configs, &timing[0], &timing[1], &row, &tracker, &timeout);
            assert(esp_timer_get_time() - start < 85000); /* fixed 40ms+20ms reclaim, scheduler margin */
            if (scenario == 0 || scenario == 4) {
                assert(err == ESP_OK && outputs[0].value == iteration && outputs[1].value == iteration);
                assert(sensorarrayMeasureFdcWorkersQuiescent());
                assert(!sensorarrayMeasureFdcWorkersQuarantined());
            } else if (scenario == 5) {
                assert(err != ESP_OK && runCount[0] == 0 && runCount[1] == 0);
                assert(sensorarrayMeasureFdcWorkersQuiescent());
            } else {
                assert(err != ESP_OK && sensorarrayMeasureFdcWorkersQuarantined());
                assert(!sensorarrayMeasureFdcWorkersQuiescent()); /* cannot perform mode transition */
                assert(outputs[0].value == 0xdead && outputs[1].value == 0xbeef);
                /* Returning does not return any worker buffer to a stack. */
                outputs[0].value = 0x1234; outputs[1].value = 0x5678;
                delayUs(100000);
                assert(sensorarrayMeasureFdcWorkersQuiescent());
                assert(sensorarrayMeasureFdcWorkersQuarantined()); /* late release cannot re-use slot */
                assert(outputs[0].value == 0x1234 && outputs[1].value == 0x5678);
                assert(sensorarrayMeasureReadFdcMatrixRowParallelEpoch(&appState, 1, 999, 999,
                    &outputs[0], &outputs[1], configs, &timing[0], &timing[1], &row, &tracker, &timeout) == ESP_ERR_INVALID_STATE);
            }
        }
        for (unsigned i = 0; i < 2; ++i) {
            pthread_mutex_lock(&tasks[i].mutex); tasks[i].stop = true; pthread_cond_signal(&tasks[i].cond); pthread_mutex_unlock(&tasks[i].mutex);
            pthread_join(tasks[i].thread, NULL); pthread_mutex_destroy(&tasks[i].mutex); pthread_cond_destroy(&tasks[i].cond);
        }
    }
    puts("FDC_WORKER_RUNTIME_PASS,actualFunctions=worker+coordinator,iterations=56,asan=1,ubsan=1,scenarios=normal50;job_unconsumed;ack_delay;run_delay;done_lost;cancel_before_run;blocked_bus;mode_guard;stale_done");
    return 0;
}
