/* Actual production owner and descriptor regression, synthetic USB/RTOS only. */
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include "tusb.h"
#include "device/dcd.h"
#include "sensorarrayUsbOtg.h"

static int64_t nowUs;
static bool ownerProgress, ignoreReset, injectReadReset, injectReceiveReset;
static bool mounted = true, suspended, cdcConnected[2];
static bool dmaOn = true, efuses[4];
static bool tusbInitOk = true;
static esp_err_t phyError;
static unsigned currentCore, unstallMask, interruptsOff, clockSync, defaultClock, realRestarts;
static unsigned available[3] = {4096,4096,512}, written[3], reads[3];
static uint8_t rx[3][2048];
static size_t rxCount[3];
static unsigned events[64], eventRead, eventWrite, pollCount, delayCount;
static unsigned registers[HOST_REGISTER_COUNT], notifications, irqDisabled, switches;
static unsigned sequence[32], sequenceCount;
static jmp_buf restartJump;
host_usb_dwc_t USB_DWC;
enum { TRACE_IRQ=1, TRACE_DISCONNECT, TRACE_PERSIST, TRACE_RESET, TRACE_USJ, TRACE_CPU, TRACE_CORE, TRACE_REAL };
static void trace(unsigned value) { assert(sequenceCount < 32); sequence[sequenceCount++] = value; }
static void enqueueEvent(unsigned event)
{
    assert(eventWrite < 64); events[eventWrite++] = event;
    tud_event_hook_cb(0, event, true);
}
static void putRx(unsigned lane, const void *bytes, size_t size)
{
    assert(rxCount[lane] + size <= sizeof(rx[lane]));
    memcpy(rx[lane] + rxCount[lane], bytes, size); rxCount[lane] += size;
}

QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *storage, StaticQueue_t *queue)
{ *queue = (StaticQueue_t){.capacity=count,.itemSize=size,.storage=storage}; return queue; }
BaseType_t xQueueSend(QueueHandle_t q, const void *p, TickType_t ticks)
{
    assert(ticks == 0); if (q->count == q->capacity) return pdFALSE;
    memcpy(q->storage + q->write * q->itemSize,p,q->itemSize);
    q->write=(q->write+1)%q->capacity; q->count++; return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *p, TickType_t ticks)
{
    assert(ticks <= 1); if (!q->count) return pdFALSE;
    memcpy(p,q->storage+q->read*q->itemSize,q->itemSize);
    q->read=(q->read+1)%q->capacity; q->count--;
    if (injectReceiveReset) { injectReceiveReset=false; enqueueEvent(DCD_EVENT_BUS_RESET); }
    return pdTRUE;
}
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (void *)1; }
void xTaskNotifyGive(TaskHandle_t t) { assert(t); notifications++; }
void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *w) { xTaskNotifyGive(t); *w=pdFALSE; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1); delayCount++; nowUs+=10000;
    if (ownerProgress) sensorarrayUsbOtgOwnerPoll();
}
int64_t esp_timer_get_time(void) { return nowUs; }
esp_err_t usb_new_phy(const usb_phy_config_t *config, usb_phy_handle_t *handle)
{ assert(config->controller == USB_PHY_CTRL_OTG); if(phyError!=ESP_OK)return phyError;*handle=(void *)2; return ESP_OK; }
void usb_serial_jtag_hal_phy_set_external(void *h, bool external)
{ assert(!external); switches++; trace(TRACE_USJ); }
void usb_serial_jtag_ll_phy_disable_pull_override(void) {}
esp_err_t esp_efuse_mac_get_default(uint8_t *mac)
{ const uint8_t value[6]={2,3,4,5,6,7}; memcpy(mac,value,6); return ESP_OK; }
bool esp_efuse_read_field_bit(unsigned field) { assert(field<4); return efuses[field]; }
void esp_restart(void) { trace(TRACE_CPU); longjmp(restartJump,1); }
void esp_rom_software_reset_system(void) { trace(TRACE_CORE); longjmp(restartJump,2); }
void __real_esp_restart_noos(void) { realRestarts++;trace(TRACE_REAL);longjmp(restartJump,3); }
void xt_ints_off(uint32_t mask) { assert(mask==UINT32_MAX);interruptsOff++; }
uint32_t clk_ll_xtal_load_freq_mhz(void) { return 40u; }
void esp_clk_utils_mspi_speed_mode_sync_before_cpu_freq_switching(uint32_t source,uint32_t frequency)
{ assert(source==40u&&frequency==40u);clockSync++; }
void rtc_clk_cpu_set_to_default_config(void) { assert(clockSync);defaultClock++; }
int esp_cpu_get_core_id(void) { return (int)currentCore; }
void esp_cpu_unstall(int core)
{ assert(core>=0&&core<SOC_CPU_CORES_NUM&&core!=(int)currentCore);assert(interruptsOff);unstallMask|=1u<<(unsigned)core; }
void chip_usb_set_persist_flags(uint32_t flags)
{ assert(flags==0); assert(!(registers[SYSTEM_PERIP_RST_EN0_REG]&SYSTEM_USB_RST)); trace(TRACE_PERSIST); }
uint32_t hostRegisterRead(unsigned r) { assert(r<HOST_REGISTER_COUNT); return registers[r]; }
void hostRegisterSet(unsigned r,uint32_t bits)
{
    assert(r<HOST_REGISTER_COUNT);
    if (r==SYSTEM_PERIP_RST_EN0_REG && (bits&SYSTEM_USB_RST)) {
        trace(TRACE_RESET); if(ignoreReset)return;
        USB_DWC.gahbcfg_reg.dmaen=0; registers[USB_GAHBCFG_REG]=0;
    }
    registers[r]|=bits;
}
void hostRegisterClear(unsigned r,uint32_t bits) { assert(r<HOST_REGISTER_COUNT); registers[r]&=~bits; }

bool tusb_rhport_init(uint8_t port,const tusb_rhport_init_t *init)
{ assert(port==0 && init->role==TUSB_ROLE_DEVICE); USB_DWC.gahbcfg_reg.dmaen=dmaOn; registers[USB_GAHBCFG_REG]=dmaOn?USB_DMAEN:0; return tusbInitOk; }
bool tud_mounted(void) { return mounted; }
bool tud_suspended(void) { return suspended; }
bool tud_disconnect(void) { trace(TRACE_DISCONNECT); mounted=false; return true; }
void dcd_int_disable(uint8_t port) { assert(port==0); irqDisabled++; trace(TRACE_IRQ); }
void tud_task_ext(uint32_t timeout,bool inIsr)
{
    assert(timeout==0 && !inIsr); pollCount++;
    for(unsigned n=0;n<CFG_TUD_TASK_EVENTS_PER_RUN && eventRead<eventWrite;n++) {
        unsigned e=events[eventRead++];
        if(e==DCD_EVENT_BUS_RESET || e==DCD_EVENT_UNPLUGGED) {
            memset(rxCount,0,sizeof(rxCount)); mounted=false;
            if(e==DCD_EVENT_UNPLUGGED)tud_umount_cb();
        }
    }
}
bool tud_task_event_ready(void) { return eventRead<eventWrite; }
uint32_t tud_vendor_n_read(uint8_t lane,void *buffer,uint32_t count)
{
    assert(lane<2); reads[lane]++; if(count>rxCount[lane])count=rxCount[lane];
    memcpy(buffer,rx[lane],count); memmove(rx[lane],rx[lane]+count,rxCount[lane]-count); rxCount[lane]-=count;
    if(lane==1 && injectReadReset) { injectReadReset=false; enqueueEvent(DCD_EVENT_BUS_RESET); }
    return count;
}
uint32_t tud_vendor_n_write_available(uint8_t lane) { assert(lane<2);return available[lane]; }
uint32_t tud_vendor_n_write(uint8_t lane,const void *buffer,uint32_t count)
{ assert(lane<2); if(count>available[lane])count=available[lane]; available[lane]-=count;written[lane]+=count;return count; }
uint32_t tud_vendor_n_write_flush(uint8_t lane) { assert(lane<2);return 0; }
static unsigned cdcTxLane(uint8_t lane)
{ assert(lane<CFG_TUD_CDC);return CFG_TUD_CDC==2&&lane==0?0u:2u; }
static unsigned cdcRxLane(uint8_t lane)
{ assert(lane<CFG_TUD_CDC);return CFG_TUD_CDC==2&&lane==0?1u:2u; }
bool tud_cdc_n_ready(uint8_t lane) { assert(lane<CFG_TUD_CDC);return mounted&&!suspended; }
bool tud_cdc_n_connected(uint8_t lane) { assert(lane<CFG_TUD_CDC);return mounted&&!suspended&&cdcConnected[lane]; }
uint8_t tud_cdc_n_get_line_state(uint8_t lane) { assert(lane<CFG_TUD_CDC);return cdcConnected[lane]?1u:0u; }
uint32_t tud_cdc_n_write_available(uint8_t lane) { return available[cdcTxLane(lane)]; }
uint32_t tud_cdc_n_write(uint8_t lane,const void *buffer,uint32_t count)
{ unsigned wire=cdcTxLane(lane);if(count>available[wire])count=available[wire];available[wire]-=count;written[wire]+=count;return count; }
uint32_t tud_cdc_n_write_flush(uint8_t lane) { assert(lane<CFG_TUD_CDC);return 0; }
uint32_t tud_cdc_n_read(uint8_t lane,void *buffer,uint32_t count)
{
    unsigned wire=cdcRxLane(lane); reads[wire]++;if(count>rxCount[wire])count=rxCount[wire];
    memcpy(buffer,rx[wire],count);memmove(rx[wire],rx[wire]+count,rxCount[wire]-count);rxCount[wire]-=count;
    if(wire==1 && injectReadReset) { injectReadReset=false;enqueueEvent(DCD_EVENT_BUS_RESET); }
    return count;
}
static const void *controlData; static uint16_t controlSize;
bool tud_control_xfer(uint8_t port,const tusb_control_request_t *request,void *buffer,uint16_t size)
{ assert(port==0);controlData=buffer;controlSize=size;return true; }

#include "owner.c"
#include "descriptors.c"

static void initialize(void)
{ assert(sensorarrayUsbOtgEnabled());assert(sensorarrayUsbOtgOwnerInit()==ESP_OK); }
static int readCommand(char *bytes,size_t size,uint32_t *epoch)
{ return sensorarrayUsbOtgRead(bytes,size,epoch); }
static void assertEmpty(void)
{ char bytes[256];uint32_t epoch;assert(readCommand(bytes,sizeof(bytes),&epoch)==0); }
static void assertCommand(const char *expected)
{
    char bytes[256]={0};uint32_t epoch=UINT32_MAX;
    int n=readCommand(bytes,sizeof(bytes),&epoch);
    assert(n==(int)strlen(expected));assert(memcmp(bytes,expected,(size_t)n)==0);
    assert(epoch==sensorarrayUsbOtgEpoch());
}
static void normalCommands(void)
{
    initialize();putRx(1,"BUILD?\r\nMODE?\n",14);sensorarrayUsbOtgOwnerPoll();
    assertCommand("BUILD?\n");assertCommand("MODE?\n");assertEmpty();
    putRx(0,"USBBOOT\n",8);putRx(2,"USBBOOT\n",8);sensorarrayUsbOtgOwnerPoll();assertEmpty();
}
static void resetEvent17(void)
{
    initialize();putRx(1,"USBBOOT\n",8);
    for(unsigned i=0;i<16;i++)enqueueEvent(DCD_EVENT_SOF);
    enqueueEvent(DCD_EVENT_BUS_RESET);sensorarrayUsbOtgOwnerPoll();
    assert(eventRead==16);assert(reads[1]==0);assertEmpty();
    assert(!sensorarrayUsbOtgOwnerCanWrite(1));
    sensorarrayUsbOtgOwnerPoll();assertEmpty();
    mounted=true;tud_mount_cb();putRx(1,"BUILD?\n",7);sensorarrayUsbOtgOwnerPoll();assertCommand("BUILD?\n");
}
static void resetDuringRead(void)
{
    initialize();putRx(1,"USBBOOT\n",8);injectReadReset=true;sensorarrayUsbOtgOwnerPoll();assertEmpty();
    sensorarrayUsbOtgOwnerPoll();mounted=true;putRx(1,"BUILD?\n",7);sensorarrayUsbOtgOwnerPoll();assertCommand("BUILD?\n");
}
static void unmountOldCommand(void)
{
    initialize();putRx(1,"USBBOOT\n",8);sensorarrayUsbOtgOwnerPoll();
    tud_umount_cb();mounted=false;assertEmpty();sensorarrayUsbOtgOwnerPoll();
    mounted=true;tud_mount_cb();putRx(1,"BUILD?\n",7);sensorarrayUsbOtgOwnerPoll();assertCommand("BUILD?\n");
}
static void partialCommandReset(void)
{
    initialize();putRx(1,"MODE=VO",7);sensorarrayUsbOtgOwnerPoll();assertEmpty();
    enqueueEvent(DCD_EVENT_BUS_RESET);sensorarrayUsbOtgOwnerPoll();
    mounted=true;putRx(1,"BUILD?\n",7);sensorarrayUsbOtgOwnerPoll();assertCommand("BUILD?\n");
}
static void queuedReceiveReset(void)
{
    initialize();putRx(1,"USBBOOT\n",8);sensorarrayUsbOtgOwnerPoll();injectReceiveReset=true;assertEmpty();
}
static void epochWrap(void)
{
    initialize();atomic_store(&s_epoch,UINT32_MAX);atomic_store(&s_processedEpoch,UINT32_MAX);
    putRx(1,"USBBOOT\n",8);sensorarrayUsbOtgOwnerPoll();enqueueEvent(DCD_EVENT_BUS_RESET);sensorarrayUsbOtgOwnerPoll();assert(sensorarrayUsbOtgEpoch()==0);assertEmpty();
    mounted=true;putRx(1,"BUILD?\n",7);sensorarrayUsbOtgOwnerPoll();assertCommand("BUILD?\n");
}
static void rxBounds(void)
{
    initialize();char over[194];memset(over,'A',sizeof(over));over[193]='\n';putRx(1,over,sizeof(over));
    putRx(1,"BUILD?\n",7);sensorarrayUsbOtgOwnerPoll();assertCommand("BUILD?\n");assert(s_stats.rxOversize==1);
    char flood[1024];for(unsigned i=0;i<sizeof(flood);i++)flood[i]=(i%2)?'\n':'X';putRx(1,flood,sizeof(flood));
    unsigned prior=reads[1];sensorarrayUsbOtgOwnerPoll();assert(reads[1]-prior==8);assert(rxCount[1]==512);assert(s_rxQueue->count==4);assert(s_stats.rxDropped>0);
}
static void endpointIndependence(void)
{
    initialize();available[0]=0;
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
    assert(sensorarrayUsbOtgCdcEnabled());
    assert(!sensorarrayUsbOtgOwnerCanWrite(0)&&!sensorarrayUsbOtgOwnerCanWrite(1));
    cdcConnected[1]=true;tud_cdc_line_state_cb(1,true,false);
    assert(sensorarrayUsbOtgOwnerCanWrite(2));assert(sensorarrayUsbOtgOwnerWrite(2,"DEBUG",5)==5);
    available[0]=8;cdcConnected[1]=false;tud_cdc_line_state_cb(1,false,false);
    assert(!sensorarrayUsbOtgOwnerCanWrite(2));
    /* Formal DTR=0 still supports a host that reads without asserting DTR. */
    assert(sensorarrayUsbOtgOwnerWrite(0,"DATA",4)==4);
    assert(sensorarrayUsbOtgOwnerWrite(1,"CTRL",4)==4);
    assert(written[0]==8&&written[1]==0&&s_stats.accepted[0]==4&&s_stats.accepted[1]==4);
    assert(!sensorarrayUsbOtgOwnerCanWrite(0)&&!sensorarrayUsbOtgOwnerCanWrite(1));
#else
    assert(!sensorarrayUsbOtgCdcEnabled());
    assert(!sensorarrayUsbOtgOwnerCanWrite(0));assert(sensorarrayUsbOtgOwnerCanWrite(1));assert(!sensorarrayUsbOtgOwnerCanWrite(2));
    assert(sensorarrayUsbOtgOwnerWrite(0,"DATA",4)==0);assert(sensorarrayUsbOtgOwnerWrite(1,"CTRL",4)==4);assert(written[0]==0&&written[1]==4);
#endif
    assert(!sensorarrayUsbOtgOwnerCanWrite(3));
}
static void cdcLineState(void)
{
    initialize();uint32_t epoch=sensorarrayUsbOtgEpoch();
    for(unsigned itf=0;itf<CFG_TUD_CDC;itf++) {
        for(unsigned state=0;state<4;state++) {
            bool dtr=(state&1)!=0;cdcConnected[itf]=dtr;tud_cdc_line_state_cb(itf,dtr,(state&2)!=0);
            sensorarrayUsbOtgOwnerPoll();assert(s_stats.cdcDtr[itf]==dtr);
            assert(sensorarrayUsbOtgEpoch()==epoch&&!atomic_load(&s_stopRequested)&&sequenceCount==0);
        }
        tud_cdc_tx_complete_cb(itf);tud_cdc_tx_complete_cb(itf);assert(s_stats.cdcTxCallbacks[itf]==2);
    }
    assert(CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED==0);
    assert(CFG_TUD_CDC_RX_PERSISTENT==0&&CFG_TUD_CDC_TX_PERSISTENT==0);
    for(unsigned lane=0;lane<3;lane++) {
        assert(s_stats.callbackBytes[lane]==0);
        assert(s_stats.callbackBytesKnown[lane]==(CFG_TUD_VENDOR==2&&lane<2));
    }
    tud_umount_cb();for(unsigned itf=0;itf<CFG_TUD_CDC;itf++)assert(!s_stats.cdcDtr[itf]);
}
static void stopAck(void)
{
    initialize(); ownerProgress=true;
    usb_phy_handle_t phy=s_phy; TaskHandle_t owner=s_owner;
    esp_err_t result=sensorarrayUsbOtgPrepareRestart(25000,true);
    fprintf(stderr,"LATE_ACK_OBSERVED,result=%d,nowUs=%lld,deadlineUs=25000,romFlag=%u,quiesced=%u,dmaMayOwn=%u\n",
        result,(long long)nowUs,(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT)!=0,
        atomic_load(&s_quiesced),atomic_load(&s_hardwareMayOwnDma));
    assert(result==ESP_ERR_TIMEOUT);
    assert(nowUs==30000 && s_detachedAtUs==10000 && delayCount==3);
    /* Cleanup may actually finish late. Its proof must remain true, without
     * converting that late completion into transaction success / ROM arm. */
    assert(atomic_load(&s_quiesced) && s_stats.quiesced);
    assert(atomic_load(&s_hardwareMayOwnDma)==0u);
    assert(registers[SYSTEM_PERIP_RST_EN0_REG]&SYSTEM_USB_RST);
    assert(switches==1 && irqDisabled==1);
    assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
    assert(s_phy==phy && s_owner==owner);
    assert(sensorarrayUsbOtgPrepareRestart(nowUs,true)==ESP_ERR_TIMEOUT);
    assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
    assert(delayCount==3);
}
static void stopTimeout(void)
{
    initialize();usb_phy_handle_t saved=s_phy;assert(sensorarrayUsbOtgPrepareRestart(25000,true)==ESP_ERR_TIMEOUT);
    assert(nowUs==30000&&delayCount==3);assert(!atomic_load(&s_quiesced));assert(s_phy==saved&&s_owner);
    assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));assert(switches==0);
}
static void detachWindow(void)
{
    initialize();atomic_store(&s_stopRequested,true);
    sensorarrayUsbOtgOwnerPoll();
    assert(s_detachStarted&&s_detachedAtUs==0&&nowUs==0);
    assert(irqDisabled==1&&sequenceCount==2&&sequence[0]==TRACE_IRQ&&sequence[1]==TRACE_DISCONNECT);
    assert(!atomic_load(&s_quiesced)&&atomic_load(&s_hardwareMayOwnDma)==1u);
    assert(!(registers[SYSTEM_PERIP_RST_EN0_REG]&SYSTEM_USB_RST)&&switches==0);
    /* Repeated polling at zero cannot re-arm the timestamp or report ACK. */
    sensorarrayUsbOtgOwnerPoll();assert(s_detachedAtUs==0&&sequenceCount==2);
    nowUs=USB_DETACH_DWELL_US-1;sensorarrayUsbOtgOwnerPoll();
    assert(!atomic_load(&s_quiesced)&&atomic_load(&s_hardwareMayOwnDma)==1u&&switches==0&&sequenceCount==2);
    nowUs=USB_DETACH_DWELL_US;sensorarrayUsbOtgOwnerPoll();
    assert(atomic_load(&s_quiesced)&&atomic_load(&s_hardwareMayOwnDma)==0u&&switches==1);
    assert(registers[SYSTEM_PERIP_RST_EN0_REG]&SYSTEM_USB_RST);
    assert(sequenceCount==5&&sequence[2]==TRACE_PERSIST&&sequence[3]==TRACE_RESET&&sequence[4]==TRACE_USJ);
}
static void detachDeadline(void)
{
    initialize();ownerProgress=true;
    usb_phy_handle_t phy=s_phy;TaskHandle_t owner=s_owner;
    assert(sensorarrayUsbOtgPrepareRestart(10000,true)==ESP_ERR_TIMEOUT);
    assert(nowUs==10000&&s_detachStarted&&s_detachedAtUs==10000);
    assert(s_phy==phy&&s_owner==owner&&!atomic_load(&s_quiesced));
    assert(atomic_load(&s_hardwareMayOwnDma)==1u&&USB_DWC.gahbcfg_reg.dmaen==1u);
    assert(switches==0&&!(registers[SYSTEM_PERIP_RST_EN0_REG]&SYSTEM_USB_RST));
    assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
    assert(sequenceCount==2);
}
static void lateCleanupAck(void)
{
    initialize();ownerProgress=true;
    /* 100 Hz: first owner service at10ms, complete20ms detach at30ms.
     * ACK is real, but arrives after the caller's25ms absolute deadline. */
    assert(sensorarrayUsbOtgPrepareRestart(25000,true)==ESP_ERR_TIMEOUT);
    assert(nowUs==30000&&s_detachedAtUs==10000&&delayCount==3);
    assert(atomic_load(&s_quiesced)&&s_stats.quiesced&&atomic_load(&s_hardwareMayOwnDma)==0u);
    assert(registers[SYSTEM_PERIP_RST_EN0_REG]&SYSTEM_USB_RST);
    assert(switches==1&&!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
    /* Equality is expired as well, even when cleanup was already ACKed. */
    assert(sensorarrayUsbOtgPrepareRestart(nowUs,true)==ESP_ERR_TIMEOUT);
    assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT)&&delayCount==3);
}
static void resetNotLatched(void)
{
    initialize();ownerProgress=true;ignoreReset=true;
    assert(sensorarrayUsbOtgPrepareRestart(25000,true)==ESP_ERR_TIMEOUT);
    assert(!atomic_load(&s_quiesced));assert(switches==0);assert(irqDisabled==1);assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
}
static void restartFallback(void)
{
    initialize();int reset=setjmp(restartJump);if(reset==0)sensorarrayUsbOtgRestart(true);
    assert(reset==2);assert(sequence[sequenceCount-1]==TRACE_CORE);assert(!atomic_load(&s_quiesced));
    assert(!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
    assert(s_recoveryMagic==USB_RECOVERY_MAGIC&&s_recoveryInverse==~USB_RECOVERY_MAGIC);
}
static void recoveryBoot(void)
{
    s_recoveryMagic=USB_RECOVERY_MAGIC;s_recoveryInverse=~USB_RECOVERY_MAGIC;
    assert(!sensorarrayUsbOtgEnabled());assert(!sensorarrayUsbOtgCdcEnabled());assert(!s_recoveryMagic&&!s_recoveryInverse);
    s_backendSelected=false;assert(sensorarrayUsbOtgEnabled());
}
static void efuseGates(void)
{
    assert(sensorarrayUsbOtgRomDownloadAvailable());
    for(unsigned i=0;i<4;i++){efuses[i]=true;assert(!sensorarrayUsbOtgRomDownloadAvailable());assert(sensorarrayUsbOtgPrepareRestart(1,true)==ESP_ERR_NOT_SUPPORTED);efuses[i]=false;}
}
static void dmaRequired(void)
{
    dmaOn=false;assert(sensorarrayUsbOtgOwnerInit()==ESP_ERR_INVALID_STATE);assert(!s_stats.initialized);assert(s_phy!=NULL);
}
static void assertEmergencyReset(void)
{
    assert(sequence[sequenceCount-1]==TRACE_CORE&&realRestarts==0);
    /* Panic must not enter clock/cache helpers which may lock or wait on a
     * peripheral. S3 Core Reset resets the digital clock selection itself. */
    assert(interruptsOff==1&&clockSync==0&&defaultClock==0);
    assert(unstallMask==(1u<<(1u-currentCore)));
    assert(!(registers[RTC_CNTL_USB_CONF_REG]&RTC_CNTL_USB_RESET_DISABLE));
    assert(s_recoveryMagic==USB_RECOVERY_MAGIC&&s_recoveryInverse==~USB_RECOVERY_MAGIC);
}
static void panicCpu0(void)
{
    initialize();registers[RTC_CNTL_USB_CONF_REG]=RTC_CNTL_USB_RESET_DISABLE;
    int reset=setjmp(restartJump);if(reset==0)__wrap_esp_restart_noos();
    assert(reset==2);assertEmergencyReset();assert(delayCount==0&&pollCount==0);
}
static void panicCpu1(void) { currentCore=1;panicCpu0(); }
static void panicNoDma(void)
{
    int reset=setjmp(restartJump);if(reset==0)__wrap_esp_restart_noos();
    assert(reset==3&&realRestarts==1);assert(!interruptsOff&&!clockSync&&!unstallMask);
    assert(!s_recoveryMagic&&!s_recoveryInverse);
}
static void panicQuiesced(void)
{
    initialize();ownerProgress=true;assert(sensorarrayUsbOtgPrepareRestart(50000,true)==ESP_OK);
    assert(atomic_load(&s_hardwareMayOwnDma)==0u);
    int reset=setjmp(restartJump);if(reset==0)__wrap_esp_restart_noos();
    assert(reset==3&&realRestarts==1);assert(!interruptsOff);
    assert(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    assert(!s_recoveryMagic&&!s_recoveryInverse);
}
static void panicDuringDetach(void)
{
    initialize();atomic_store(&s_stopRequested,true);sensorarrayUsbOtgOwnerPoll();
    assert(s_detachStarted&&!atomic_load(&s_quiesced)&&atomic_load(&s_hardwareMayOwnDma)==1u);
    int reset=setjmp(restartJump);if(reset==0)__wrap_esp_restart_noos();
    assert(reset==2);assertEmergencyReset();
    assert(switches==0&&!(registers[RTC_CNTL_OPTION1_REG]&RTC_CNTL_FORCE_DOWNLOAD_BOOT));
    assert(realRestarts==0&&delayCount==0&&pollCount==0);
}
static void panicInitFailed(void)
{
    tusbInitOk=false;assert(sensorarrayUsbOtgOwnerInit()==ESP_FAIL);
    assert(atomic_load(&s_hardwareMayOwnDma)==1u&&!s_stats.initialized);
    int reset=setjmp(restartJump);if(reset==0)__wrap_esp_restart_noos();
    assert(reset==2);assertEmergencyReset();
}
static void panicPhyFailed(void)
{
    phyError=ESP_ERR_NO_MEM;assert(sensorarrayUsbOtgOwnerInit()==ESP_ERR_NO_MEM);
    assert(atomic_load(&s_hardwareMayOwnDma)==0u);panicNoDma();
}
static unsigned le16(const uint8_t *p) { return p[0]|((unsigned)p[1]<<8); }
static void descriptors(void)
{
    sensorarrayUsbOtgInitSerialString();assert(strcmp(s_serial,"020304050607")==0);
    assert(CFG_TUD_DWC2_DMA_ENABLE==1&&CFG_TUD_DWC2_SLAVE_ENABLE==0);assert(CFG_TUD_MEM_DCACHE_ENABLE==0);
    assert(CFG_TUD_VENDOR_TX_EPSIZE>=64);
    const tusb_desc_device_t *d=(const void*)tud_descriptor_device_cb();assert(d->idVendor==0x303a&&d->bcdDevice==0x0100&&d->bMaxPacketSize0==64);
    const uint8_t *c=tud_descriptor_configuration_cb(0);unsigned total=le16(c+2),eps=0,ifaces=0,mask=0;
    assert(total==USB_CONFIG_BYTES&&c[4]==4);assert(tud_descriptor_configuration_cb(1)==NULL);
    for(unsigned off=0;off<total;off+=c[off]){assert(c[off]>=2&&off+c[off]<=total);if(c[off+1]==TUSB_DESC_INTERFACE)ifaces++;if(c[off+1]==TUSB_DESC_ENDPOINT){unsigned ep=c[off+2];eps++;mask|=1u<<(ep&15);assert(le16(c+off+4)==((c[off+3]&3)==TUSB_XFER_INTERRUPT?8:64));}}
#ifdef CONFIG_SENSORARRAY_USB_OTG_CDC
    assert(CFG_TUD_VENDOR==0&&CFG_TUD_CDC==2&&d->idProduct==0x4042&&d->bcdUSB==0x0200);
    assert(CFG_TUD_CDC_TX_BUFSIZE==4096&&CFG_TUD_CDC_TX_EPSIZE==512&&CFG_TUD_CDC_RX_EPSIZE==64);
    assert(ifaces==4&&eps==6&&mask==0x1e&&tud_descriptor_bos_cb()==NULL);
    unsigned classes[4]={0},endpointMask=0,ins=1;
    for(unsigned off=0;off<total;off+=c[off]) {
        if(c[off+1]==TUSB_DESC_INTERFACE)classes[c[off+2]]=c[off+5];
        if(c[off+1]==TUSB_DESC_ENDPOINT){unsigned ep=c[off+2],bit=(ep&15)+((ep&0x80)?8:0);assert(!(endpointMask&(1u<<bit)));endpointMask|=1u<<bit;if(ep&0x80)ins++;}
    }
    assert(classes[0]==TUSB_CLASS_CDC&&classes[1]==TUSB_CLASS_CDC_DATA&&classes[2]==TUSB_CLASS_CDC&&classes[3]==TUSB_CLASS_CDC_DATA&&ins==5);
#else
    assert(CFG_TUD_VENDOR==2&&CFG_TUD_CDC==1&&d->idProduct==0x4041);
    assert(ifaces==4&&eps==7&&mask==0x1e);
    assert(sizeof(s_msOs)==338&&le16(s_msOs+8)==338&&le16(s_msOs+16)==328);
    assert(s_msOs[22]==0&&s_msOs[182]==1&&le16(s_msOs+24)==160&&le16(s_msOs+184)==160);
    assert(memcmp(s_msOs+30,"WINUSB\0\0",8)==0&&memcmp(s_msOs+190,"WINUSB\0\0",8)==0);
    tusb_control_request_t request={.bmRequestType=0xc0,.bRequest=0x20,.wIndex=7,.wValue=0,.wLength=338};
    assert(tud_vendor_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request));assert(controlData==s_msOs&&controlSize==338);
    request.wIndex=8;assert(!tud_vendor_control_xfer_cb(0,CONTROL_STAGE_SETUP,&request));
#endif
}
int main(int argc,char **argv)
{
    assert(argc==2);
    struct { const char *name;void(*test)(void); } cases[]={
        {"normal_commands",normalCommands},{"reset_event17",resetEvent17},{"reset_during_read",resetDuringRead},
        {"unmount_old_command",unmountOldCommand},{"partial_command_reset",partialCommandReset},{"queued_receive_reset",queuedReceiveReset},
        {"epoch_wrap",epochWrap},{"rx_bounds",rxBounds},{"endpoint_independence",endpointIndependence},{"cdc_line_state",cdcLineState},
        {"stop_ack",stopAck},{"stop_timeout",stopTimeout},{"detach_window",detachWindow},{"detach_deadline",detachDeadline},
        {"late_cleanup_ack",lateCleanupAck},
        {"reset_not_latched",resetNotLatched},{"restart_fallback",restartFallback},
        {"recovery_boot",recoveryBoot},{"efuse_gates",efuseGates},{"dma_required",dmaRequired},{"descriptors",descriptors},
        {"panic_cpu0",panicCpu0},{"panic_cpu1",panicCpu1},{"panic_no_dma",panicNoDma},
        {"panic_quiesced",panicQuiesced},{"panic_during_detach",panicDuringDetach},
        {"panic_init_failed",panicInitFailed},{"panic_phy_failed",panicPhyFailed},
    };
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);i++)if(strcmp(argv[1],cases[i].name)==0){cases[i].test();printf("USB_OTG_OWNER,%s,PASS\n",cases[i].name);return 0;}
    return 2;
}
