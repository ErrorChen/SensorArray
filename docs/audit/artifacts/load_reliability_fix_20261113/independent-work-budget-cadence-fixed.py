#!/usr/bin/env python3
"""Independent cadence/grace/backstop checks on the current production component."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
harness = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "WORK_BUDGET_SOURCE"
static uint32_t tick,delays,registrations;
static bool failRegister;
static esp_freertos_idle_cb_t idle;
TickType_t xTaskGetTickCount(void){return tick;}
int xPortGetCoreID(void){return 0;}
void vTaskDelay(TickType_t ticks){assert(ticks==1);tick+=ticks;delays++;}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb,UBaseType_t cpu){
 assert(cpu==0);registrations++;if(failRegister)return ESP_ERR_NO_MEM;idle=cb;return ESP_OK;
}
int main(int argc,char **argv){
 assert(argc==2);unsigned scenario=(unsigned)strtoul(argv[1],NULL,10);
 failRegister=scenario==5;
 assert(sensorarrayWorkBudgetInit()==(failRegister?ESP_ERR_NO_MEM:ESP_OK));
 if(scenario==0){
  for(unsigned i=0;i<100;i++){
   tick=i*3;assert(idle());tick+=2;
   assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  }
  assert(delays==0);
 }else if(scenario==1||scenario==5){
  tick=3;
  for(unsigned i=0;i<20;i++)assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  assert(delays==20); /* Never invent idle credit before a real callback. */
 }else if(scenario==2){
  assert(idle());tick=2;
  assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  for(unsigned i=0;i<6;i++)assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  assert(delays==6); /* Its own delay must not refill credit. */
 }else if(scenario==3){
  assert(idle());tick=4;
  assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  assert(delays==1); /* Backstop overrides the newly observed generation. */
 }else if(scenario==4){
  __atomic_store_n(&s_idleGeneration,UINT32_MAX,__ATOMIC_RELAXED);
  tick=UINT32_MAX-2u;__atomic_store_n(&s_idleTick,tick,__ATOMIC_RELAXED);
  assert(idle());tick=UINT32_MAX;
  assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  tick=1;
  assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
  assert(delays==1);
 }else assert(false);
 sensorarrayWorkBudgetSnapshot_t s;sensorarrayWorkBudgetCopySnapshot(&s);
 assert(registrations==1);
 printf("INDEPENDENT_CADENCE_FIXED_PASS,scenario=%u,idleEvents=%u,checkpoints=%u,yields=%u,delayCalls=%u,synthetic_scheduler=1\n",scenario,s.idleGeneration,s.workers[3].checkpoints,s.workers[3].yields,delays);
}
'''.replace("WORK_BUDGET_SOURCE", str(root / "components/sensorarrayWorkBudget/sensorarrayWorkBudget.c"))
with tempfile.TemporaryDirectory(prefix="sa-cadence-fixed-") as name:
    tmp=Path(name);src=tmp/"review.c";exe=tmp/"review";src.write_text(harness)
    cmd=["/usr/bin/gcc","-std=c11","-Wall","-Wextra","-Werror"]
    for inc in [root/"tests/host/work_budget_stubs",root/"tests/host/include",root/"components/sensorarrayWorkBudget/include"]:
        cmd += ["-I",str(inc)]
    subprocess.run(cmd+[str(src),"-o",str(exe)],check=True)
    for case in range(6):
        subprocess.run([str(exe),str(case)],check=True,timeout=5)
