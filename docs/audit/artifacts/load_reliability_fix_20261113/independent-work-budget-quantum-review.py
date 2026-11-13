#!/usr/bin/env python3
"""Independent actual production-component checks; synthetic time, no HIL."""
from pathlib import Path
import hashlib, json, subprocess, tempfile
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
SRC=ROOT/'components/sensorarrayWorkBudget/sensorarrayWorkBudget.c'
harness=r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "sensorarrayWorkBudget.c"
static uint32_t tick,delays;static int core;static bool failed,model;
static esp_freertos_idle_cb_t hook;static uint64_t us,blocked[7];static unsigned worker;
TickType_t xTaskGetTickCount(void){return tick;}
int xPortGetCoreID(void){return core;}
void vTaskDelay(TickType_t t){assert(t==1);delays++;if(model)blocked[worker]=(us/10000+1)*10000;else tick++;}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb,UBaseType_t c){assert(c==0);if(failed)return ESP_ERR_NO_MEM;hook=cb;return ESP_OK;}
static bool cp(unsigned w){return sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)w);}
int main(int argc,char**argv){assert(argc==2);unsigned c=strtoul(argv[1],NULL,10);failed=c==6;
 tick=(c==1?UINT32_MAX-1u:100u);assert(sensorarrayWorkBudgetInit()==(failed?ESP_ERR_NO_MEM:ESP_OK));
 uint32_t origin=tick;
 if(c==0||c==1||c==6){
  tick=origin+2u;assert(!cp(3));assert(s_idleGeneration==0);tick=origin+4u;
  for(unsigned i=0;i<20;i++){assert(cp(3));assert(s_idleGeneration==0&&s_idleTick==0);}
  sensorarrayWorkBudgetSnapshot_t s;sensorarrayWorkBudgetCopySnapshot(&s);assert(!s.idleKnown&&s.idleAgeTicks==UINT32_MAX&&delays==20);
 }else if(c==2){
  assert(hook());assert(!cp(3));tick+=2;assert(cp(3));assert(s_idleTick==origin&&s_idleGeneration==1);
  assert(!cp(3));tick=origin+4u;for(unsigned i=0;i<20;i++){assert(cp(3));assert(s_idleTick==origin&&s_idleGeneration==1);}
 }else if(c==3){
  assert(hook());for(unsigned i=1;i<=128;i++)assert(cp(4)==(i%32==0));assert(delays==4&&s_idleGeneration==1&&s_workers[4].maxBurstUnits==32);
  assert(cp(4));assert(delays==5); /* Fresh local account cannot evade global backstop. */
 }else if(c==4){
  for(unsigned i=0;i<100;i++){tick+=1;assert(hook());tick+=2;for(unsigned j=0;j<4;j++)assert(!cp(3));}
  assert(delays==0); /* Work in other tasks is no longer a per-record penalty. */
 }else if(c==5){
  assert(hook());uint32_t deadline=origin+4u;assert(!cp(3));tick+=2;assert(cp(3));assert(tick==origin+3u&&deadline==origin+4u);
  tick=deadline;assert(cp(3));assert(tick-deadline==1u&&s_idleTick==origin); /* Caller deadline is never replaced by local quantum. */
 }else if(c==7){
  tick=UINT32_MAX-1u;assert(hook());assert(!cp(0));tick=0;assert(cp(0));assert(tick==1);assert(!cp(0));tick=2;assert(cp(0));assert(delays==2);
 }else if(c==8||c==9){
  assert(hook());model=true;us=(uint64_t)tick*10000+(c==9?9700:0);uint64_t end=us+2000000;
  const unsigned pri[7]={5,5,7,4,6,4,5},cost[7]={900,700,1300,200,400,300,500};unsigned n[7]={0},rot=0,idles=0;
  while(us<end){int best=-1;for(unsigned j=0;j<7;j++){unsigned w=(rot+j)%7;if(blocked[w]<=us&&(best<0||pri[w]>pri[best]))best=w;}
   if(best<0){tick=us/10000;assert(hook());idles++;uint64_t next=UINT64_MAX;for(unsigned j=0;j<7;j++)if(blocked[j]<next)next=blocked[j];assert(next>us);us=next;continue;}
   worker=best;rot=(worker+1)%7;us+=cost[worker];tick=us/10000;n[worker]++;(void)cp(worker);
  }
  sensorarrayWorkBudgetSnapshot_t s;sensorarrayWorkBudgetCopySnapshot(&s);for(unsigned j=0;j<7;j++)assert(n[j]>0);
  assert(idles>0&&s.maxIdleGapTicks<=5);printf("MODEL,phase=%u,idles=%u,maxGap=%u,work=%u/%u/%u/%u/%u/%u/%u\n",c,idles,s.maxIdleGapTicks,n[0],n[1],n[2],n[3],n[4],n[5],n[6]);
 }else assert(false);
 printf("INDEPENDENT_QUANTUM_PASS,case=%u,delayCalls=%u,idleGeneration=%u,synthetic=1\n",c,delays,s_idleGeneration);return 0;}
'''
commands=[];outputs=[]
with tempfile.TemporaryDirectory(prefix='sa-quantum-review-') as d:
 p=Path(d);(p/'review.c').write_text(harness);cmd=['/usr/bin/gcc','-std=c11','-Wall','-Wextra','-Werror']
 for x in ['tests/host/work_budget_stubs','tests/host/include','components/sensorarrayWorkBudget/include','components/sensorarrayWorkBudget']:cmd+=['-I',str(ROOT/x)]
 cmd += [str(p/'review.c'),'-o',str(p/'review')]
 r=subprocess.run(cmd,capture_output=True,text=True);commands.append({'argv':cmd,'exit':r.returncode});outputs.append(r.stdout+r.stderr);r.check_returncode()
 for c in range(10):
  cmd=[str(p/'review'),str(c)];r=subprocess.run(cmd,capture_output=True,text=True,timeout=5);commands.append({'argv':cmd,'exit':r.returncode});outputs.append(r.stdout+r.stderr);r.check_returncode()
(HERE/'independent-work-budget-quantum-review.log').write_text(''.join(outputs))
paths=['components/sensorarrayWorkBudget/sensorarrayWorkBudget.c','components/sensorarrayWorkBudget/include/sensorarrayWorkBudget.h','core/transport/sensorarraySerialIo.c','components/sensorarrayBle/sensorarrayBle.c','tests/host/test_work_budget.c']
result={'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'sourceHashes':{x:hashlib.sha256((ROOT/x).read_bytes()).hexdigest() for x in paths},'commands':commands,'scenarios':10,'result':'PASS','productionEdits':False,'hardware':'NOT_RUN','checks':['unknown idle and nonzero init epoch','unknown idle tick wrap','own yield never changes shared idle; backstop bypasses local restart','32-unit flood bound','delayed worker four-record burst without per-record penalty','absolute caller deadline unchanged by block','hook registration failure bounded','local/global tick wrap','seven continuously ready workers heterogeneous 200..1300us work, two tick phases'],'sourceReview':{'serialDeadline':'Absolute enqueue deadline is retained; loop rechecks expiration after checkpoint before selecting/writing another packet. No new deadline on wake.','idleWriter':'Only real idle hook writes idleGeneration/idleTick; no watchdog feed.','mutex':'Checkpoint releases no caller ownership and takes no hardware/queue/stdio lock; sole worker writer contract retained.'},'limits':['Synthetic RTOS stubs model one-tick block; not target scheduler timing proof.','Seven-worker model uses bounded work units and excludes independently busy SDK tasks/interrupts; lower-priority fairness is observed only under those assumptions.','A synchronous SDK call can overshoot quantum before the next checkpoint. Four ticks is a checkpoint decision threshold, not a hard upper bound on real idle latency.','Stat snapshots are individually sampled, not atomic CPU utilization.'],'blockingFindings':[]}
(HERE/'independent-work-budget-quantum-review.json').write_text(json.dumps(result,indent=2)+'\n')
print(''.join(outputs),end='')
