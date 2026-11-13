#include <assert.h>
#include <stdio.h>
#include "/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-cadence-source.c"
static uint32_t tick,delays;
static esp_freertos_idle_cb_t idle;
TickType_t xTaskGetTickCount(void){return tick;}
int xPortGetCoreID(void){return 0;}
void vTaskDelay(TickType_t ticks){assert(ticks==1);tick+=ticks;delays++;}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb,UBaseType_t cpu){assert(cpu==0);idle=cb;return ESP_OK;}
int main(void){
 assert(sensorarrayWorkBudgetInit()==ESP_OK);
 for(unsigned i=0;i<100;i++){
  tick=i*3;assert(idle()); /* A real idle event occurred before this burst. */
  tick+=2; /* Other higher-priority work runs before this low-priority task. */
  (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX);
 }
 sensorarrayWorkBudgetSnapshot_t s;sensorarrayWorkBudgetCopySnapshot(&s);
 printf("LOW_PRIORITY_CADENCE_REPRO,realIdleEvents=%u,checkpoints=%u,yields=%u,delayCalls=%u,maxBurst=%u\n",s.idleGeneration,s.workers[3].checkpoints,s.workers[3].yields,delays,s.workers[3].maxBurstUnits);
 assert(s.idleGeneration==100 && s.workers[3].checkpoints==100 && s.workers[3].yields==100);
}
