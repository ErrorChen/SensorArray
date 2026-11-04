#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayRouteController.h"
#include "sensorarrayMeasure.h"
#include "tmuxSwitch.h"

static bool workersReleased=true, rejectDisable, badPower, badGpio;
static bool rejectStop;
static uint8_t power,refmux;
static int sd,clamps,spiCalls;
static int64_t now;
static tmuxSwitchControlState_t control;
int64_t esp_timer_get_time(void){return ++now;}
void esp_rom_delay_us(uint32_t us){now+=us;}
int sensorarrayAcqEventPrintf(const char *fmt,...){(void)fmt;return 0;}
esp_err_t gpio_config(const gpio_config_t *c){return ESP_OK;}
esp_err_t gpio_set_level(gpio_num_t p,uint32_t v){sd=(int)v;return ESP_OK;}
int gpio_get_level(gpio_num_t p){return sd;}
bool sensorarrayMeasureFdcWorkersQuiescent(void){return workersReleased;}
bool ads126xAdcIsQuarantined(const ads126xAdcHandle_t *h){return h->spiQuarantined;}
bool ads126xAdcHasAdc2(const ads126xAdcHandle_t *h){return false;}
esp_err_t ads126xAdcStopAdc1(ads126xAdcHandle_t *h){spiCalls++;return rejectStop?ESP_ERR_TIMEOUT:ESP_OK;}
esp_err_t ads126xAdcStopAdc2(ads126xAdcHandle_t *h){return ESP_ERR_NOT_SUPPORTED;}
esp_err_t ads126xAdcSetInternalReference(ads126xAdcHandle_t *h,bool on){spiCalls++;if(rejectDisable&&!on)return ESP_ERR_TIMEOUT;power=on?(power|ADS126X_POWER_INTREF):(power&~ADS126X_POWER_INTREF);return ESP_OK;}
esp_err_t ads126xAdcReadCoreRegisters(ads126xAdcHandle_t*h,uint8_t*p,uint8_t*i,uint8_t*m,uint8_t*n,uint8_t*r){spiCalls++;if(p)*p=power|(badPower?ADS126X_POWER_INTREF:0);if(i)*i=0x06;if(m)*m=0x0f;if(n)*n=0x0a;if(r)*r=refmux;return ESP_OK;}
bool ads126xAdcMode2DecodePgaGain(uint8_t m,uint8_t*g){*g=1;return true;}
esp_err_t ads126xAdcConfigure(ads126xAdcHandle_t*h,bool ir,bool st,ads126xCrcMode_t crc,uint8_t gain,uint8_t dr){assert(crc==ADS126X_CRC_CRC8);h->pgaGain=gain;power=ir?ADS126X_POWER_INTREF:0;return ESP_OK;}
esp_err_t ads126xAdcApplyPowerPolicy(ads126xAdcHandle_t*h,bool ai,bool ir,bool av,bool vb,uint8_t*b,uint8_t*a){power=(ir?ADS126X_POWER_INTREF:0)|(vb?ADS126X_POWER_VBIAS:0);return ESP_OK;}
esp_err_t ads126xAdcSetRefMux(ads126xAdcHandle_t*h,uint8_t r){refmux=r;return ESP_OK;}
esp_err_t ads126xAdcSetRefMuxWithVref(ads126xAdcHandle_t*h,uint8_t r,uint32_t v){refmux=r;h->vrefMicrovolts=v;return ESP_OK;}
esp_err_t Fdc2214CapEnterSleep(Fdc2214CapDevice_t*h,uint16_t c){return ESP_OK;}
esp_err_t Fdc2214CapExitSleep(Fdc2214CapDevice_t*h,uint16_t c){return ESP_OK;}
esp_err_t tmux1108SetSource(tmux1108Source_t s){if(s==TMUX1108_SOURCE_GND){assert((power&ADS126X_POWER_INTREF)==0);clamps++;}control.cmdSource=s;control.cmdSwLevel=control.obsSwLevel=s==TMUX1108_SOURCE_GND;return ESP_OK;}
esp_err_t tmux1108GetSource(tmux1108Source_t *s){*s=control.cmdSource;return ESP_OK;}
esp_err_t tmux1134SelectSelBLevel(bool b){control.cmdSelbLevel=control.obsSelbLevel=b;return ESP_OK;}
esp_err_t tmux1134SetEnLogicalState(bool b){return ESP_OK;}
esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t*s,sensorarraySelaRoute_t r,uint32_t u,const char*a,const char*b){int level;assert(sensorarrayBoardMapSelaRouteToGpioLevel(r,&level));control.cmdSelaLevel=control.obsSelaLevel=level;return ESP_OK;}
esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t*s,const char*r){return ESP_OK;}
esp_err_t tmuxSwitchGetControlState(tmuxSwitchControlState_t*c){*c=control;if(badGpio)c->obsSwLevel=!c->cmdSwLevel;return ESP_OK;}
esp_err_t tmux1108SelectRow(uint8_t r){control.cmdRow=r;control.obsA0Level=r&1;control.obsA1Level=(r>>1)&1;control.obsA2Level=(r>>2)&1;return ESP_OK;}
esp_err_t tmux1108SelectRowBreakBeforeMake(uint8_t r){return tmux1108SelectRow(r);}
static sensorarrayState_t state;
static sensorarrayRouteController_t route;
static void reset(void){memset(&state,0,sizeof(state));memset(&control,0,sizeof(control));workersReleased=true;rejectDisable=badPower=badGpio=rejectStop=false;power=ADS126X_POWER_INTREF;refmux=0;clamps=spiCalls=0;state.adsReady=true;state.ads.pgaGain=1;state.adsAdc1Running=true;state.fdcPrimary.ready=state.fdcSecondary.ready=true;state.fdcPrimary.handle=(void*)1;state.fdcSecondary.handle=(void*)2;assert(sensorarrayRouteControllerInit(&route,&state)==ESP_OK);assert(!route.snapshot.safe);}
int main(void){
 reset();int before=spiCalls;uint64_t us=99;assert(!sensorarrayBoardMapHasIsolatedRailMonitor());assert(sensorarrayRouteControllerEnterSafeRailMonitor(&route,&us)==ESP_ERR_NOT_SUPPORTED);assert(us==0&&spiCalls==before&&clamps==0);
 reset();workersReleased=false;assert(sensorarrayRouteControllerEnterSafe(&route,"busy")==ESP_ERR_INVALID_STATE);assert(!route.snapshot.safe&&!route.snapshot.known&&spiCalls==0&&clamps==0);
 reset();rejectDisable=true;assert(sensorarrayRouteControllerEnterSafe(&route,"spi")==ESP_ERR_TIMEOUT);assert(!route.snapshot.safe&&!route.snapshot.verified&&clamps==0);
 reset();badPower=true;assert(sensorarrayRouteControllerEnterSafe(&route,"power")==ESP_ERR_INVALID_RESPONSE);assert(!route.snapshot.safe&&clamps==0);
 reset();badGpio=true;assert(sensorarrayRouteControllerEnterSafe(&route,"gpio")==ESP_ERR_INVALID_RESPONSE);assert(!route.snapshot.safe&&!route.snapshot.known);
 reset();rejectStop=true;assert(sensorarrayRouteControllerEnterSafe(&route,"stop")==ESP_ERR_TIMEOUT);assert(!route.snapshot.safe&&state.adsAdc1Running);
 reset();assert(sensorarrayRouteControllerEnterSafe(&route,"ok")==ESP_OK);assert(route.snapshot.safe&&route.snapshot.applied&&route.snapshot.verified&&!route.snapshot.analogIsolationVerified);
 reset();state.adsReady=false;assert(sensorarrayRouteControllerEnterSafe(&route,"unknown")!=ESP_OK);assert(!route.snapshot.safe&&clamps==0);
 reset();state.ads.spiQuarantined=true;assert(sensorarrayRouteControllerApplyMode(&route,SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,NULL,&us)==ESP_ERR_INVALID_STATE);assert(clamps==0&&spiCalls==0);
 reset();assert(sensorarrayRouteControllerApplyMode(&route,SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,NULL,&us)==ESP_OK);assert(route.snapshot.applied&&route.snapshot.known&&!route.snapshot.safe);int c=clamps;assert(sensorarrayRouteControllerSelectRow(&route,2)==ESP_OK);assert(clamps==c&&(power&ADS126X_POWER_INTREF));badGpio=true;assert(sensorarrayRouteControllerSelectRow(&route,3)==ESP_ERR_INVALID_RESPONSE);assert(!route.snapshot.safe&&strcmp(route.snapshot.failureStage,"gpio_mismatch")==0&&route.snapshot.cleanupError==ESP_ERR_INVALID_RESPONSE);
 puts("ROUTE_SAFETY_TESTS,passed=10,production_controller=1,physical_isolation=not_simulated");return 0;
}
