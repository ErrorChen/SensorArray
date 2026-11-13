#!/usr/bin/env python3
"""Replay one synthetic late-ACK schedule using saved real production sources.
No hardware/build-directory writes. A pinned TinyUSB 0.21.0~1 header tree is
required by the saved owner test runner. Expected before=FAIL, after=PASS.
"""
from pathlib import Path
import hashlib,json,resource,subprocess,tempfile
HERE=Path(__file__).resolve().parent
ROOT=next(p for p in HERE.parents if (p/'components/sensorarrayUsbOtg').is_dir())
F=HERE/'fixtures'
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
expected=json.loads((HERE/'fixture-hashes.json').read_text())
for name,digest in expected.items():
    assert hashlib.sha256((F/name).read_bytes()).hexdigest()==digest,name
text=(F/'runner-template.py').read_text()
text=text.replace('ROOT = Path(__file__).resolve().parents[1]',f'ROOT = Path({str(ROOT)!r})')
text=text.replace('ROOT / "components/sensorarrayUsbOtg/sensorarrayUsbDescriptors.c"',f'Path({str(F/"descriptors.c")!r})')
text=text.replace('ROOT / "components/sensorarrayUsbOtg/include/tusb_config.h"',f'Path({str(F/"tusb_config.h")!r})')
text=text.replace('ROOT / "components/sensorarrayUsbOtg/include/sensorarrayUsbOtg.h"',f'Path({str(F/"sensorarrayUsbOtg.h")!r})')
text=text.replace('str(ROOT / "tests/host/test_usb_otg_owner.c")','str(temp / "test_usb_otg_owner.c")')
text=text.replace('str(ROOT / "tests/host/usb_otg_stubs")',f'str(Path({str(F)!r}))')
text=text.replace('        wrappers = (', f'        (temp / "test_usb_otg_owner.c").write_bytes(Path({str(F/"test_usb_otg_owner.c")!r}).read_bytes())\n        wrappers = (')
results=[]
with tempfile.TemporaryDirectory(prefix='sensorarray-detach-replay-') as temporary:
    runner=Path(temporary)/'runner.py';runner.write_text(text)
    for version in ('before','after'):
        for backend in ('vendor','cdc'):
            command=['python3',str(runner),'--source',str(F/f'owner-{version}.c'),'--scenario','stop_ack','--backend',backend]
            p=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=10)
            expected_failure=version=='before'
            ok=(p.returncode!=0 and 'result=0,nowUs=30000,deadlineUs=25000,romFlag=1' in p.stderr and 'result==ESP_ERR_TIMEOUT' in p.stderr) if expected_failure else (p.returncode==0 and 'result=263,nowUs=30000,deadlineUs=25000,romFlag=0,quiesced=1,dmaMayOwn=0' in p.stderr)
            results.append({'version':version,'backend':backend,'command':command,'exit_code':p.returncode,'expected_failure':expected_failure,'expected_outcome_observed':ok,'stdout':p.stdout,'stderr':p.stderr})
print(json.dumps({'result':'PASS_REPLAY' if all(r['expected_outcome_observed'] for r in results) else 'FAIL_REPLAY','hardware':'NOT_RUN','schedule':'100 Hz synthetic; owner first runs at10ms, detach completes at30ms, caller deadline25ms','results':results},indent=2))
raise SystemExit(0 if all(r['expected_outcome_observed'] for r in results) else 1)
