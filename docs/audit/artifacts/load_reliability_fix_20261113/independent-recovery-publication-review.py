#!/usr/bin/env python3
"""Independent synthetic publication-failure checks; production files are read-only."""
from pathlib import Path
import sys
import tempfile

root = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(root / "tools"))
test = (root / "tests/host/test_recovery_control.c").read_text()
case = r'''
static void auditPublicationFailure(void) {
    sensorarrayAppContext_t c; reset(&c);
    sensorarrayConfigAuthority_t a; assert(sensorarrayConfigIdentityCopy(&a));
    uint32_t id=0;
    assert(sensorarrayRecoveryControlPost(1,&a,true,&id)==ESP_OK);
    assert(sensorarrayRecoveryControlAck(id,ESP_OK));
    sensorarrayRecoverRequest_t r;
    assert(sensorarrayRecoveryControlTake(&a,true,&r)==SENSORARRAY_RECOVER_TAKE_READY);
    auditFailPublish=true;
    assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_REJECTED,
        "recovery_failed",ESP_ERR_INVALID_CRC,ESP_ERR_TIMEOUT,&a,true,
        false,false,false)==ESP_ERR_NO_MEM);
    auditFailPublish=false;
    expectQuery("last=rejected,reason=recovery_failed",
        "originalError=0x109,cleanupError=0x107");
    expectQuery("terminalQueued=0","busy=0,phase=none");
    assert(!sensorarrayTransportGuaranteedTextIsReserved());
    uint32_t nextId=0;
    assert(sensorarrayRecoveryControlPost(0,&a,true,&nextId)==ESP_OK);
    assert(nextId!=id);
    /* Late old callbacks must not release/cancel the new request's slot. */
    assert(!sensorarrayRecoveryControlAck(id,ESP_OK));
    assert(!sensorarrayRecoveryControlAck(id,ESP_ERR_TIMEOUT));
    assert(sensorarrayTransportGuaranteedTextIsReserved());
    assert(sensorarrayRecoveryControlTake(&a,true,&r)==SENSORARRAY_RECOVER_TAKE_NONE);
    assert(sensorarrayRecoveryControlAck(nextId,ESP_ERR_TIMEOUT));
    expectQuery("reason=ack_publish_failed","terminalQueued=0");
    assert(!sensorarrayTransportGuaranteedTextIsReserved());
    puts("INDEPENDENT_RECOVERY_PUBLICATION_PASS,forced_local_publish_failure=1,cache_preserved=1,first_cleanup=1,slot_released=1,late_old_ack_ignored=1,next_request_not_applied_without_ack=1,synthetic_boundary=1");
}
'''
test = test.replace("int main(void) {", case + "\nint main(void) {\n    auditPublicationFailure();")
with tempfile.TemporaryDirectory(prefix="sa-recovery-review-") as tmp:
    harness = Path(tmp) / "audit.c"
    harness.write_text(test)
    script = (root / "tools/test_recovery_control.py").read_text()
    script = script.replace("(ROOT / 'tests/host/test_recovery_control.c').read_text()",
                            repr(test))
    # Only the terminal publication boundary is fault-injected. The recovery
    # state machine, cache, main boundary code and guaranteed slot are actual C.
    production_include = '#include "' + str(root / "main/control/sensorarrayRecoveryControl.c") + '"'
    wrapper = '''static bool auditFailPublish;
static esp_err_t auditPublish(const char *s, size_t n) {
    return auditFailPublish ? ESP_ERR_NO_MEM : sensorarrayTransportGuaranteedTextPublish(s,n);
}
#define sensorarrayTransportGuaranteedTextPublish auditPublish
''' + production_include + '''
#undef sensorarrayTransportGuaranteedTextPublish
'''
    needle = "harness = harness.replace('/* PRODUCTION_MODULES */', includes)"
    assert needle in script
    script = script.replace(needle, "includes = includes.replace(" + repr(production_include) +
                            ", " + repr(wrapper) + ")\n" + needle)
    exec(compile(script, str(root / "tools/test_recovery_control.py"), "exec"),
         {"__file__": str(root / "tools/test_recovery_control.py"), "__name__": "__main__"})
