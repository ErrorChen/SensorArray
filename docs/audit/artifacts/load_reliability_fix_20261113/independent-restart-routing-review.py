#!/usr/bin/env python3
"""Independent Serial/network admission cross-product on the actual restart/queue path."""
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(root / "tools"))
test = (root / "tests/host/test_recovery_control.c").read_text()
case = r'''
static void auditRestartSinkCrossProduct(void) {
    unsigned cases=0;
    for(unsigned networkMask=0;networkMask<4;networkMask++) {
        for(unsigned serialFailure=0;serialFailure<2;serialFailure++) {
            for(unsigned networkFailure=0;networkFailure<2;networkFailure++) {
                sensorarrayAppContext_t c;reset(&c);
                netBleEnabled=(networkMask&1)!=0;netWifiEnabled=(networkMask&2)!=0;
                serialQueueError=serialFailure?ESP_ERR_TIMEOUT:ESP_OK;
                networkQueueAccept=!networkFailure;
                uint32_t id=0;assert(sensorarrayRestartPost(&id));
                restartAck(id,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c));
                assert(restarts==1&&serialQueueCalls==1);
                assert(lifecycleCalls==(networkMask!=0));
                bool expectedNetworkQueued=networkMask!=0&&!networkFailure;
                assert(s_restart.last.serialQueued==!serialFailure);
                assert(s_restart.last.networkQueued==expectedNetworkQueued);
                assert(s_restart.last.terminalQueued==(!serialFailure||expectedNetworkQueued));
                assert(s_restart.last.serialError==serialQueueError);
                assert(s_restart.last.networkError==((networkMask&&networkFailure)?ESP_ERR_TIMEOUT:ESP_OK));
                assert(strstr(serialText,"RAPP,cmd=RESTART,")&&strstr(serialText,"kind=manual,"));
                if(networkMask)assert(strcmp(serialText,lifecycleText)==0);
                assert(!sensorarrayRunRecoveryBoundary(&c)&&restarts==1);
                cases++;
            }
        }
    }
    printf("INDEPENDENT_RESTART_ROUTING_PASS,cases=%u,actual_restart_main_transport_queue=1,serial_always_attempted=1,no_network_not_queued=1,each_admission_error_preserved=1,matching_full_terminal=1,no_second_reset=1,hardware_synthetic=1\n",cases);
}
'''
assert "int main(void) {" in test
test = test.replace("int main(void) {", case + "\nint main(void) {\n    auditRestartSinkCrossProduct();")
script = (root / "tools/test_recovery_control.py").read_text()
script = script.replace("(ROOT / 'tests/host/test_recovery_control.c').read_text()", repr(test))
exec(compile(script, str(root / "tools/test_recovery_control.py"), "exec"),
     {"__file__": str(root / "tools/test_recovery_control.py"), "__name__": "__main__"})
