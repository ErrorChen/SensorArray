#!/usr/bin/env python3
"""Read-only audit of the exact build02 ELF. No hardware, build, or mutation of inputs."""
import datetime, hashlib, json, re, subprocess
from pathlib import Path
from elftools.elf.elffile import ELFFile
ROOT=Path('/home/csc/esp/workspace/SensorArray')
ART=ROOT/'docs/audit/artifacts/cdc_recovery_acceptance_20261116'
EXPECTED='575c5dc25217721cfb47f651556d0e28edd4b5fe666a022200b8737608ff2e40'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
elf=ROOT/'build/SensorArray.elf'; mp=ROOT/'build/SensorArray.map'
assert sha(elf)==EXPECTED, 'Refuse a different candidate ELF; keep original evidence immutable.'
base=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
old=subprocess.check_output(['git','show',base+':core/transport/sensorarrayTransportPool.h'],cwd=ROOT)
maplines=mp.read_text().splitlines()
symbols=['s_pool','s_lastProtectedPublishedUs','s_protectedIdentity','s_protectedIdentityKnown','s_asyncProtectionStats','s_pendingSlots','s_frameSlotIdentity','s_frameSlotProtection','s_frameSlotState','s_frameSlots','s_asyncLogWorkspace','s_ble']
maprecords=[]
for i,line in enumerate(maplines):
 if any(re.search(r'\.(?:bss|data)\.'+re.escape(s)+r'(?:\s|$)',line) for s in symbols):
  raw=line
  if not re.search(r'0x[0-9a-f]+\s+0x[0-9a-f]+',line): raw+='\n'+maplines[i+1]
  match=re.search(r'0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(.+)',raw)
  if match: maprecords.append({'raw':raw,'address':hex(int(match[1],16)),'bytes':int(match[2],16),'object':match[3]})
types={}
wanted={'sensorarrayTransportPayloadSlot_t','sensorarrayTransportPool_t','sensorarrayAsyncLogWorkspace_t','sensorarrayAsyncLogFrameSlot_t'}
with elf.open('rb') as f:
 dwarf=ELFFile(f).get_dwarf_info()
 for cu in dwarf.iter_CUs():
  for die in cu.iter_DIEs():
   a=die.attributes.get('DW_AT_name'); n=a.value.decode(errors='replace') if a and isinstance(a.value,bytes) else ''
   if die.tag!='DW_TAG_typedef' or n not in wanted or n in types: continue
   t=die.get_DIE_from_attribute('DW_AT_type'); members={}
   for child in t.iter_children():
    if child.tag=='DW_TAG_member': members[child.attributes['DW_AT_name'].value.decode()]=child.attributes['DW_AT_data_member_location'].value
   types[n]={'die_offset':hex(t.offset),'bytes':t.attributes['DW_AT_byte_size'].value,'members':members}
assert types['sensorarrayTransportPool_t']['bytes']==17216
assert types['sensorarrayTransportPayloadSlot_t']['bytes']==2456
paths=['core/transport/sensorarrayTransport.c','core/transport/sensorarrayTransportPool.c','core/transport/sensorarrayTransportPool.h','main/output/sensorarrayAsyncLog.c','main/output/sensorarrayAsyncLogStorage.h','sdkconfig']
state=ART/'ble-snapshot-state-01.summary.json'; sv=json.loads(state.read_text())
observed={a['command']:a['fields'] for a in sv['attempts'] if 'fields' in a and a['command'] in ['BUILD?','BOOT?','READY?','SPI?']}
oldlog=ART/'tail-01-test_transport_pool.txt'
assert 'slots=5,descriptorBytes=8,slotBytes=2376' in oldlog.read_text()
rec={
 'schema':'sensorarray_pipeline_internal_memory_audit_v1',
 'recorded_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'reviewer':'/root/protected_pipeline','scope':'Read-only current ELF map/DWARF, prior source/layout evidence, actual SDK source; no hardware query, firmware build, stack reduction or pool change.',
 'head':base,'dirty_diff_sha256':hashlib.sha256(subprocess.check_output(['git','diff','--binary'],cwd=ROOT)).hexdigest(),
 'elf':{'path':'build/SensorArray.elf','sha256':sha(elf)},'map':{'path':'build/SensorArray.map','sha256':sha(mp)},
 'source_read_sha256':{p:sha(ROOT/p) for p in paths},'map_extract':maprecords,'target_dwarf_types':types,
 'transport_comparison':{
  'baseline_source_commit':base,'baseline_header_sha256':hashlib.sha256(old).hexdigest(),
  'old_slot_count':5,'old_slot_bytes':2376,'old_slot_size_evidence':{'path':str(oldlog.relative_to(ROOT)),'sha256':sha(oldlog),'kind':'preserved executed host ABI test; not an old target ELF'},
  'old_pool_bytes_derived':11904,'derivation':'5 * 2376 + 20-byte stats, rounded to 8-byte structure alignment. Old target ELF/map not available here; this value is derived, not measured from a prior target ELF.',
  'new_slot_count':7,'new_slot_bytes_target_dwarf':2456,'new_pool_bytes_target_map_and_dwarf':17216,
  'pool_growth_bytes_against_derived_prior':5312,'growth_breakdown':'2 * 2456 extra slots + 5 * (2456 - 2376) changed existing slots = 5312.',
  'current_cpu_payload_bytes_in_internal_pool':16135,'payload_derivation':'7 * (2304 wire bytes + 1 private NUL) = 16135. Metadata and padding are separate.',
  'placement':'internal DRAM .bss, address 0x3fcc69e0',
  'selected_fix':'NO_CHANGE_TO_TRANSPORT_POOL. Root selected only BLE CPU-payload migration (18432 bytes) for the next candidate; this audit does not verify that implementation.'},
 'source_storage':{
  'normal_slots_configured':4,'protected_slots_added':2,'total_slots':6,
  'slot_bytes_target_dwarf':7160,'slot_storage_bytes':42960,'workspace_bytes_target_dwarf':13536,'total_cpu_storage_bytes':56496,
  'additional_slot_storage_bytes':14320,
  'allocation':'Existing startup-owned permanent heap_caps_calloc SPIRAM|8BIT preferred, explicit INTERNAL|8BIT fallback; payload/frame storage is not static BSS. Actual placement requires APP_LOG_INIT frameSlotsMemory/workspaceMemory.',
  'task_stack_bytes_configured':16384,'task_stack':'Explicit WithCaps SPIRAM|8BIT for formatter on this sdkconfig; TCB remains internal.',
  'actual_failed_boot_app_log_init_placement':'unknown: no APP_LOG_INIT record in supplied failed-boot observation',
  'metadata':'New per-slot identities/protection/state and counters remain internal; actual symbol sizes are retained in map_extract. Existing static event/protocol queues are not new payload allocations.'},
 'sdk':{
  'root':'/home/csc/esp/esp-idf-v5.5.5','normal_task_caps':'MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT',
  'normal_stack_bytes_needed':16384,'tcb_allocation':'separate internal pvPortMalloc allocation',
  'with_caps_limitation':'Only explicitly requested stack moves to chosen caps; TCB still internal.',
  'config':['CONFIG_SPIRAM=y','CONFIG_SPIRAM_USE_CAPS_ALLOC=y','CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y','CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y','CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY is not set'],
  'source_refs':['components/freertos/heap_idf.c:43','components/freertos/esp_additions/idf_additions.c:39','components/esp_common/include/esp_err.h:25'],
  'independent_sdk_review':'ble-snapshot-internal-memory-sdk-review-01.json'},
 'observed_failed_boot':{'artifact':str(state.relative_to(ROOT)),'sha256':sha(state),'fields':observed},
 'interpretation':{
  'status':'HIL_STARTUP_FAIL_NO_MEM','error':'0x101 is actual SDK ESP_ERR_NO_MEM; scan task creation did not succeed.',
  'total_heap_is_not_internal_largest':'BOOT heap=8295496 includes aggregate available heap; it is not evidence of a contiguous 16384-byte INTERNAL|8BIT stack plus TCB.',
  'unresolved':'At-failure internalFree/internalLargest were not captured in this observation; fragmentation versus total internal exhaustion/TCB failure is not distinguished.',
  'spi_crc':'NOT_RUN: SPI phase idle, attempted=0, samples=0. Initialized crc=0 is not a measured zero-error rate.',
  'control':'Observed USB control replies survive; CONFIG reply deadlines expired. This is not a whole-device death.',
  'next_candidate':'Keep scan stack, all protected slots, dual FDC and ADC integrity; migrate BLE CPU-only payload with fixed ownership and explicit allocation failure, then independently build/boot/HIL and capture per-capability heap before/after scan creation.'}}
out=ART/'ble-snapshot-02-pipeline-memory-audit-01.json'
assert not out.exists(), 'Do not overwrite audit evidence'
out.write_text(json.dumps(rec,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'artifact':str(out.relative_to(ROOT)),'artifact_sha256':sha(out),'elf_sha256':sha(elf),'pool_bytes':17216,'prior_derived_pool_bytes':11904,'delta_bytes':5312,'source_frame_storage_bytes':42960,'source_workspace_bytes':13536,'status':'READ_ONLY_AUDIT_COMPLETE_HIL_STARTUP_FAIL'}))
