#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayTransportFrameMeta.h"

static void parse(sensorarrayTransportPayloadSlot_t *slot, const char *data)
{
    strcpy(slot->data, data);
    slot->length = (uint16_t)strlen(data);
    sensorarrayTransportParseFrameMeta(slot);
}
int main(void)
{
    sensorarrayTransportPayloadSlot_t slot = {0};
    parse(&slot, "C,seq=11,rows=1,cells=8,mgen=4,mrid=3,gen=2,rid=1,session=0123456789ABCDEF,bootId=99,rev=6,rgen=2,rrid=1,pgen=5,prid=4\nK,gen=999\n");
    assert(slot.hasFrameMeta && slot.generation == 2 && slot.requestId == 1);
    assert(slot.modeGeneration == 4 && slot.modeRequestId == 3);
    assert(slot.rowsGeneration == 2 && slot.rowsRequestId == 1);
    assert(slot.profileGeneration == 5 && slot.profileRequestId == 4);
    assert(slot.sessionId == UINT64_C(0x0123456789abcdef) && slot.bootId == 99 && slot.configRevision == 6);
    /* Metadata stays attached to queued payload; no global-state restamping. */
    sensorarrayTransportPayloadSlot_t queued = slot;
    parse(&slot, "V,seq=12,rows=1,cells=8,gen=7,rid=6,mgen=7,mrid=6,rgen=2,rrid=1\n");
    assert(queued.modeGeneration == 4 && slot.modeGeneration == 7);
    parse(&slot, "M,seq=13,rows=1,cells=8,pgen=8,prid=7,rgen=2,rrid=1,mgen=9,mrid=8\n");
    assert(slot.generation == 2 && slot.requestId == 1 && slot.profileGeneration == 8);
    parse(&slot, "C,seq=14,rows=1,cells=8,mgen=77,mrid=66\nK,gen=22,rid=11\n");
    assert(slot.generation == 0 && slot.requestId == 0 && slot.modeGeneration == 77);
    char preview[384];
    parse(&slot, "C,seq=15,rows=1,cells=8,gen=2,rid=1,mgen=77,mrid=66,rgen=2,rrid=1,pgen=3,prid=4,session=0123456789abcdef,bootId=99,rev=5\nD,42,43\n");
    assert(sensorarrayTransportBuildPreview(&slot, preview, sizeof(preview)));
    assert(strstr(preview, "previewOnly=1,gen=2,rid=1,session=0123456789ABCDEF,bootId=99,rev=5,mgen=77,mrid=66,rgen=2,rrid=1,pgen=3,prid=4"));
    assert(strstr(preview, "d0=42,dl=43"));
    parse(&slot, "C,seq=16,rows=1,cells=8,mgen=1\nD,X01,X01\n");
    assert(sensorarrayTransportBuildPreview(&slot, preview, sizeof(preview)));
    assert(strstr(preview, "d0=na,dl=na"));
    parse(&slot, "C,seq=4294967296,rows=1,cells=8,gen=1,rid=1\n");
    assert(!slot.hasFrameMeta);
    puts("TRANSPORT_FRAME_METADATA_PASS,domains=rows;mode;profile;session,exact_fields=1,immutable=1");
}
