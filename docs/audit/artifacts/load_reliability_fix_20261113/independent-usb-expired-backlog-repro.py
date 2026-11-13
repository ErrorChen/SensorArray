from pathlib import Path
import tempfile,sys
root=Path(__file__).resolve().parents[4]
source=Path(__file__).with_name("independent-usb-expired-backlog-source.c")
test=(root/"tests/host/test_serial_io.c").read_text()
case=r'''
    resetClock(); scenario=1; writeBudget=0; outputLength=0;
    for(unsigned i=0;i<4;i++) assert(sensorarraySerialIoQueue("EXPIRE_CTRL\n",12,true)==ESP_OK);
    drain();
    unsigned retained=0;for(unsigned i=5;i<9;i++)retained+=s_inUse[i]?1u:0u;
    fprintf(stderr,"AUDIT_NOHOST_BACKLOG,nowUs=%lld,queued=%u,retained=%u\n",(long long)nowUs,s_queues[1]->count,retained);
    assert(s_queues[1]->count==0 && retained==0);
'''
needle='    printf("SERIAL_IO_PASS,allocation='
assert needle in test
test=test.replace(needle,case+needle)
with tempfile.TemporaryDirectory(prefix="sa-usb-review-") as tmp:
    p=Path(tmp)/"audit_serial_io.c";p.write_text(test)
    script=(root/"tools/test_serial_io.py").read_text()
    script=script.replace('str(root / "tests/host/test_serial_io.c")',repr(str(p)))
    script=script.replace('"-std=c11", "-D_GNU_SOURCE"','"-std=c11", '+repr('-DSERIAL_IO_SOURCE="'+str(source)+'"')+', "-D_GNU_SOURCE"')
    sys.argv=["test_serial_io.py"]
    exec(compile(script,str(root/"tools/test_serial_io.py"),"exec"),{"__file__":str(root/"tools/test_serial_io.py"),"__name__":"__main__"})
