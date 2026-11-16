/* Real local ESP-IDF L2CA/update bodies, synthetic peer/controller boundary.
 * Generated request constants come from the current firmware. No radio/HIL. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef bool BOOLEAN;
typedef uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef uint8_t BD_ADDR[6];
#define TRUE 1
#define FALSE 0
#define BD_ADDR_LEN 6u
#define BT_TRANSPORT_LE 1u
#define HCI_SUCCESS 0u
#define HCI_ERR_ILLEGAL_COMMAND 0x0cu
#define HCI_ROLE_MASTER 0u
#define HCI_ROLE_SLAVE 1u
#define BLE_LLT_INCLUDED TRUE
#define BLE_SLAVE_UPD_CONN_PARAMS TRUE
#define BLE_CE_LEN_MIN 0u
#define BTU_TTYPE_L2CAP_UPDA_CONN_PARAMS 3u
#define BTM_BLE_LEGACY_GAP_CONNECTION_PARAMS_UPDATE_EVT 4u
#define L2CAP_TRACE_WARNING(...) ((void)0)
#define L2CAP_TRACE_ERROR(...) ((void)0)
#define HCI_LE_CONN_PARAM_REQ_SUPPORTED(features) (((features)[0] & 1u) != 0u)
/* PRODUCTION_CONSTANTS */
typedef struct {
    UINT16 transport, current_used_conn_interval, current_used_conn_latency, current_used_conn_timeout;
    UINT16 conn_update_mask, waiting_update_conn_min_interval, waiting_update_conn_max_interval;
    UINT16 waiting_update_conn_latency, waiting_update_conn_timeout;
    UINT16 updating_conn_min_interval, updating_conn_max_interval, handle;
    UINT8 link_role;
    bool updating_param_flag;
    unsigned upda_con_timer;
    BD_ADDR remote_bd_addr;
} tL2C_LCB;
typedef struct { UINT8 peer_le_features[8]; } tACL_CONN;
typedef struct { struct {
    UINT8 status;
    BD_ADDR remote_bd_addr;
    UINT16 min_conn_int, max_conn_int, conn_int, slave_latency, supervision_tout;
} conn_params_update; } tBTM_BLE_LEGACY_GAP_CB_PARAMS;
typedef struct { UINT8 as_array[8]; } feature_t;
typedef struct { const feature_t *(*get_features_ble)(void); } controller_t;
static tL2C_LCB link;
static tACL_CONN acl;
static feature_t features;
static tBTM_BLE_LEGACY_GAP_CB_PARAMS callback;
static bool missingPeer;
static unsigned callbacks, hciRequests, peerRequests, timers;
static UINT16 sentMin, sentMax, sentLatency, sentTimeout;
static const feature_t *getFeatures(void) { return &features; }
static const controller_t *controller_get_interface(void)
{ static const controller_t controller = {getFeatures}; return &controller; }
static tACL_CONN *btm_bda_to_acl(BD_ADDR peer, unsigned transport)
{ assert(transport == BT_TRANSPORT_LE && !memcmp(peer, link.remote_bd_addr, 6u)); return missingPeer ? NULL : &acl; }
static tL2C_LCB *l2cu_find_lcb_by_bd_addr(BD_ADDR peer, unsigned transport)
{ assert(transport == BT_TRANSPORT_LE && !memcmp(peer, link.remote_bd_addr, 6u)); return missingPeer ? NULL : &link; }
static void BTM_LegacyBleCallbackTrigger(unsigned event, tBTM_BLE_LEGACY_GAP_CB_PARAMS *value)
{ assert(event == BTM_BLE_LEGACY_GAP_CONNECTION_PARAMS_UPDATE_EVT); callbacks++; callback = *value; }
static void noteRequest(UINT16 min, UINT16 max, UINT16 latency, UINT16 timeout)
{ sentMin = min; sentMax = max; sentLatency = latency; sentTimeout = timeout; }
static void btsnd_hcic_ble_upd_ll_conn_params(UINT16 handle, UINT16 min, UINT16 max,
                                            UINT16 latency, UINT16 timeout, UINT16 ceMin, UINT16 ceMax)
{ assert(handle == link.handle && ceMin == 0u && ceMax == 0u); hciRequests++; noteRequest(min, max, latency, timeout); }
static void l2cu_send_peer_ble_par_req(tL2C_LCB *lcb, UINT16 min, UINT16 max, UINT16 latency, UINT16 timeout)
{ assert(lcb == &link); peerRequests++; noteRequest(min, max, latency, timeout); }
static UINT32 CalConnectParamTimeout(tL2C_LCB *lcb) { assert(lcb == &link); return 9u; }
static void btu_start_timer(unsigned *timer, unsigned kind, UINT32 period)
{ assert(timer == &link.upda_con_timer && kind == BTU_TTYPE_L2CAP_UPDA_CONN_PARAMS && period == 9u); timers++; }
/* PRODUCTION_FUNCTIONS */
static void reset(void)
{
    memset(&link, 0, sizeof(link)); memset(&acl, 0, sizeof(acl));
    memset(&features, 0, sizeof(features)); memset(&callback, 0, sizeof(callback));
    missingPeer = false; callbacks = hciRequests = peerRequests = timers = 0u;
    sentMin = sentMax = sentLatency = sentTimeout = 0u;
    link.transport = BT_TRANSPORT_LE; link.link_role = HCI_ROLE_SLAVE;
    link.current_used_conn_interval = 12u; link.current_used_conn_timeout = 400u;
    link.handle = 9u; memset(link.remote_bd_addr, 0x5a, 6u);
}
static bool requestCurrent(void)
{
    return L2CA_UpdateBleConnParams(link.remote_bd_addr, CURRENT_REQUEST_MIN,
                                   CURRENT_REQUEST_MAX, 0u, 400u);
}
int main(void)
{
    reset();
    /* Historical range used by the real previous firmware. At current12,
     * actual SDK short-circuits to success despite requested minimum6. */
    assert(L2CA_UpdateBleConnParams(link.remote_bd_addr, 6u, 12u, 0u, 400u));
    assert(callbacks == 1u && callback.conn_params_update.status == HCI_SUCCESS);
    assert(callback.conn_params_update.conn_int == 12u && !peerRequests && !hciRequests && !timers);
    puts("SDK_CONNECTION_REQUEST,case=old_6_12_current12,callbackInterval=12,actualUpdateRequests=0");

    reset(); assert(requestCurrent());
    assert(CURRENT_REQUEST_MIN == 6u && CURRENT_REQUEST_MAX == 6u);
    assert(!callbacks && peerRequests == 1u && !hciRequests && timers == 1u);
    assert(sentMin == 6u && sentMax == 6u && sentLatency == 0u && sentTimeout == 400u);
    assert(link.current_used_conn_interval == 12u); /* Request is not applied. */
    assert(link.conn_update_mask & L2C_BLE_UPDATE_PENDING);
    puts("SDK_CONNECTION_REQUEST,case=new_6_6_current12,path=L2CAP,actualIntervalStill=12");

    reset(); link.current_used_conn_interval = 6u; assert(requestCurrent());
    assert(callbacks == 1u && callback.conn_params_update.conn_int == 6u);
    assert(!peerRequests && !hciRequests && !timers); /* Already actually7.5ms. */

    reset(); features.as_array[0] = acl.peer_le_features[0] = 1u; assert(requestCurrent());
    assert(hciRequests == 1u && !peerRequests && !callbacks && timers == 1u);
    assert(sentMin == 6u && sentMax == 6u && link.current_used_conn_interval == 12u);

    reset(); link.link_role = HCI_ROLE_MASTER; assert(requestCurrent());
    assert(hciRequests == 1u && !peerRequests && !callbacks && timers == 1u);

    reset(); link.conn_update_mask = L2C_BLE_UPDATE_PARAM_FULL; assert(!requestCurrent());
    assert(callbacks == 1u && callback.conn_params_update.status == HCI_ERR_ILLEGAL_COMMAND);
    assert(!hciRequests && !peerRequests && !timers);

    reset(); link.conn_update_mask = L2C_BLE_UPDATE_PENDING; assert(requestCurrent());
    assert(link.conn_update_mask & L2C_BLE_UPDATE_PARAM_FULL);
    assert(!hciRequests && !peerRequests && !callbacks && !timers);

    reset(); missingPeer = true; assert(!requestCurrent());
    assert(!callbacks && !hciRequests && !peerRequests && !timers);
    puts("BLE_SDK_CONNECTION_REQUEST_PASS,cases=8,actualSDKFunctions=2,hardware=NOT_RUN");
    return 0;
}
