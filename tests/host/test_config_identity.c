#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorarrayConfigIdentity.h"
#include "sensorarrayFrameBuilder.h"
#include "sensorarrayTypes.h"
#include "sensorarrayTextProtocol.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL,line=%d,%s\n", __LINE__, #x); return 1; } } while (0)

static sensorarrayFrame_t frame;
static sensorarrayTextPacket_t packet;

static int testIdentity(void)
{
    sensorarrayMeasurementModeContext_t mode;
    sensorarrayRowModeProfile_t profile;
    sensorarrayMeasurementModeInit(&mode);
    sensorarrayRowModeProfileInit(&profile, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);
    sensorarrayFrameConfigSnapshot_t rows = {.rows=1, .cells=8, .rowMask=1, .generation=2, .requestId=1};
    sensorarrayConfigIdentity_t captured;
    for (uint32_t i=0; i<50; ++i) {
        sensorarrayMeasurementMode_t target = (i%3 == 1) ? SENSORARRAY_MEASUREMENT_MODE_VOLTAGE : SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
        CHECK(sensorarrayMeasurementModeAccept(&mode, target, i+10));
        CHECK(!sensorarrayMeasurementModeAccept(&mode, target, 999));
        CHECK(sensorarrayMeasurementModeBeginTransition(&mode));
        CHECK(sensorarrayMeasurementModeCompleteTransition(&mode, 8953+i, 10));
        sensorarrayMeasurementMode_t uniform[8];
        for (unsigned row=0;row<8;++row) { uniform[row]=target; }
        CHECK(sensorarrayRowModeProfileAccept(&profile,uniform,i+10));
        CHECK(sensorarrayRowModeProfileCompleteTransition(&profile,8953+i,10));
        sensorarrayConfigIdentityCapture(UINT64_C(0xAABBCCDDEE112233), 7, &mode.snapshot, &rows, &profile, 8953+i, &captured);
        CHECK(captured.modeRequestId == i+10 && captured.rowsGeneration == 2);
        sensorarrayConfigIdentityPublish(&captured, &mode.snapshot, &profile, &rows);
        sensorarrayConfigAuthority_t authority;
        CHECK(sensorarrayConfigIdentityCopy(&authority));
        CHECK(authority.identity.revision == i+1);
        CHECK(authority.identity.modeGeneration == mode.snapshot.generation);
        CHECK(authority.modeTerminalId == i+10);
        char text[512];
        CHECK(sensorarrayConfigIdentityFormat(text, sizeof(text)) == ESP_OK);
        CHECK(strstr(text, "rgen=2,rrid=1") && strstr(text, "mterm=applied"));
    }
    /* A lost/delayed MAPP can be recovered by the atomic query. The C rows
     * generation is deliberately lower than the MODE generation. */
    sensorarrayConfigIdentity_t queued = captured;
    rows.generation++; rows.requestId=99; rows.rows=8; rows.cells=64; rows.rowMask=255;
    sensorarrayConfigIdentityCapture(UINT64_C(0xAABBCCDDEE112233), 7, &mode.snapshot, &rows, &profile, 9003, &captured);
    CHECK(captured.modeGeneration == queued.modeGeneration && captured.revision == queued.revision+1);
    CHECK(queued.rowsGeneration == 2); /* Already queued frame is immutable. */
    sensorarrayConfigIdentityPublish(&captured, &mode.snapshot, &profile, &rows);
    uint32_t appliedId = mode.snapshot.appliedRequestId;
    CHECK(sensorarrayMeasurementModeAccept(&mode, SENSORARRAY_MEASUREMENT_MODE_RESISTANCE, 900));
    CHECK(sensorarrayMeasurementModeBeginTransition(&mode));
    sensorarrayMeasurementModeFailTransition(&mode, 0x108, 11);
    CHECK(mode.snapshot.appliedRequestId == appliedId && mode.snapshot.lastRequestId == 900);
    CHECK(mode.snapshot.lastOutcome == SENSORARRAY_CONFIG_OUTCOME_REJECTED);
    sensorarrayConfigIdentityCapture(UINT64_C(0xAABBCCDDEE112233), 7, &mode.snapshot, &rows, &profile, 9004, &captured);
    sensorarrayConfigIdentityPublish(&captured, &mode.snapshot, &profile, &rows);
    char text[512];
    CHECK(sensorarrayConfigIdentityFormat(text, sizeof(text)) == ESP_OK);
    CHECK(strstr(text,"mtermId=900,mterm=rejected,merr=0x108"));
    /* Profile failure also preserves the successful applied ID. */
    sensorarrayMeasurementMode_t modes[8];
    CHECK(sensorarrayRowModeProfileParse("CVRCVRCV",8,modes));
    CHECK(sensorarrayRowModeProfileAccept(&profile,modes,901));
    CHECK(sensorarrayRowModeProfileCompleteTransition(&profile,9004,10));
    CHECK(sensorarrayRowModeProfileAccept(&profile,modes,902));
    sensorarrayRowModeProfileFailTransition(&profile,0x108,10);
    CHECK(profile.appliedRequestId == 901 && profile.lastRequestId == 902);
    sensorarrayRowModeProfileRejectRequest(&profile,904,0x103);
    CHECK(profile.appliedRequestId==901 && profile.lastRequestId==904 && profile.lastRequestError==0x103);
    /* Recovery re-applies CAP with a new generation; wrap skips unknown zero. */
    mode.snapshot.generation=UINT32_MAX;
    CHECK(sensorarrayMeasurementModeAccept(&mode,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,903));
    CHECK(sensorarrayMeasurementModeBeginTransition(&mode));
    CHECK(sensorarrayMeasurementModeCompleteTransition(&mode,9005,10));
    CHECK(mode.snapshot.generation == 1);
    /* A cold boot can repeat legacy bootId; the new session resets revision. */
    sensorarrayMeasurementModeInit(&mode);
    sensorarrayConfigIdentityCapture(UINT64_C(0xAABBCCDDEE112234), 7, &mode.snapshot, &rows, &profile, 0, &captured);
    CHECK(captured.revision == 1 && captured.sessionId != queued.sessionId);
    return 0;
}

static void seedFrame(unsigned rows, sensorarrayMeasurementMode_t mode)
{
    sensorarrayFrameBuilderInitInvalid(&frame);
    frame.sequence=9198; frame.timestampUs=123456789; frame.activeRows=rows;
    frame.configSnapshot=(sensorarrayFrameConfigSnapshot_t){.rows=rows,.cells=rows*8,.rowMask=rows==8?255:1,.generation=2,.requestId=1};
    frame.measurement.mode=mode; frame.measurement.modeGeneration=4; frame.measurement.modeRequestId=3;
    frame.measurement.unit=mode==SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE?SENSORARRAY_MEASUREMENT_UNIT_PF:mode==SENSORARRAY_MEASUREMENT_MODE_VOLTAGE?SENSORARRAY_MEASUREMENT_UNIT_VOLT:SENSORARRAY_MEASUREMENT_UNIT_OHM;
    frame.measurement.decimalScale=mode==SENSORARRAY_MEASUREMENT_MODE_RESISTANCE?-3:-6;
    frame.expectedMask=rows==8?UINT64_MAX:255;
    frame.acquiredMask=frame.expectedMask; frame.capValidMask=frame.expectedMask;
    frame.measurement.validMask=frame.expectedMask; frame.measurement.freshMask=frame.expectedMask;
    frame.measurement.errorMask=0; frame.errorMask=0; frame.freshMask=frame.expectedMask;
    frame.validCount=rows*8; frame.freshFrame=true;
    frame.rowProfileGeneration=5; frame.rowProfileRequestId=3;
    frame.identity=(sensorarrayConfigIdentity_t){.sessionId=UINT64_C(0xAABBCCDDEE112233),.bootId=7,.revision=8,
      .modeGeneration=4,.modeRequestId=3,.rowsGeneration=2,.rowsRequestId=1,.profileGeneration=5,.profileRequestId=3};
    for(unsigned i=0;i<64;++i) { frame.capTotalPf[i]=12.25+i; frame.measurement.valuesFixed[i]=50000+i; frame.measurement.pgaGain[i]=1; }
    for(unsigned i=0;i<8;++i) {frame.rowMode[i]=mode;frame.rowScale[i]=frame.measurement.decimalScale;}
}

static int saveFrame(const char *directory,const char *name)
{
    CHECK(sensorarrayTextProtocolBuildFrame(&frame,&packet)==ESP_OK);
    char path[1024]; snprintf(path,sizeof(path),"%s/%s.frame",directory,name);
    FILE *file=fopen(path,"wb"); CHECK(file); CHECK(fwrite(packet.data,1,packet.length,file)==packet.length); CHECK(fclose(file)==0);
    printf("GOLDEN,name=%s,bytes=%u\n",name,(unsigned)packet.length);
    sensorarrayMeasurementModeSnapshot_t mode={.state=sensorarrayMeasurementStateForMode(frame.measurement.mode),
        .activeMode=frame.measurement.mode,.generation=frame.identity.modeGeneration,.appliedRequestId=frame.identity.modeRequestId,
        .lastRequestId=frame.identity.modeRequestId,.lastOutcome=SENSORARRAY_CONFIG_OUTCOME_APPLIED};
    sensorarrayRowModeProfile_t profile;
    sensorarrayRowModeProfileInit(&profile,frame.measurement.mode);
    memcpy(profile.modes,frame.rowMode,sizeof(profile.modes));
    profile.generation=frame.identity.profileGeneration;profile.appliedRequestId=frame.identity.profileRequestId;
    profile.lastRequestId=profile.appliedRequestId;profile.lastOutcome=SENSORARRAY_CONFIG_OUTCOME_APPLIED;
    sensorarrayConfigIdentity_t id=frame.identity;
    id.mode=mode.activeMode;id.state=mode.state;id.rows=frame.activeRows;id.appliedFrameBoundary=frame.sequence;
    CHECK(sensorarrayRowModeProfileFormat(profile.modes,id.profile));
    frame.configSnapshot.lastTerminal=true;frame.configSnapshot.lastApplied=true;
    frame.configSnapshot.lastRequestId=frame.configSnapshot.requestId;
    sensorarrayConfigIdentityPublish(&id,&mode,&profile,&frame.configSnapshot);
    char query[512];CHECK(sensorarrayConfigIdentityFormat(query,sizeof(query))==ESP_OK);
    snprintf(path,sizeof(path),"%s/%s.config",directory,name);
    file=fopen(path,"wb");CHECK(file);CHECK(fwrite(query,1,strlen(query),file)==strlen(query));CHECK(fclose(file)==0);
    return 0;
}

int main(int argc,char **argv)
{
    CHECK(argc==2); CHECK(testIdentity()==0);
    printf("IDENTITY_MEMORY,frame=%zu,identity=%zu,authority=%zu,packet=%zu\n",sizeof(frame),sizeof(frame.identity),sizeof(sensorarrayConfigAuthority_t),sizeof(packet));
    for(unsigned rows=1;rows<=8;rows+=7) {
        const char *names[]={"cap","volt","res"};
        for(unsigned mode=0;mode<3;++mode) {seedFrame(rows,(sensorarrayMeasurementMode_t)mode);char name[32];snprintf(name,sizeof(name),"%s_%u",names[mode],rows); CHECK(saveFrame(argv[1],name)==0);}
        seedFrame(rows,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);frame.mixedProfile=true;
        for(unsigned r=0;r<8;++r){frame.rowMode[r]=(sensorarrayMeasurementMode_t)(r%3);frame.rowScale[r]=r%3==2?-3:-6;}
        char name[32];snprintf(name,sizeof(name),"mixed_%u",rows);CHECK(saveFrame(argv[1],name)==0);
    }
    for (unsigned stage=0;stage<4;++stage) {
        sensorarrayMeasurementMode_t mode=stage==1?SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
        seedFrame(1,mode);
        frame.sequence=9197+stage;
        frame.identity.revision=6+stage;
        frame.identity.modeGeneration=frame.measurement.modeGeneration=2+stage;
        frame.identity.modeRequestId=frame.measurement.modeRequestId=1+stage;
        frame.identity.profileGeneration=frame.rowProfileGeneration=3+stage;
        frame.identity.profileRequestId=frame.rowProfileRequestId=1+stage;
        const char *names[]={"cap_before","volt_transition","cap_after","cap_reapplied"};
        CHECK(saveFrame(argv[1],names[stage])==0);
    }
    seedFrame(8,SENSORARRAY_MEASUREMENT_MODE_VOLTAGE);
    frame.acquiredMask=0;frame.measurement.validMask=0;frame.measurement.freshMask=0;frame.measurement.errorMask=UINT64_MAX;
    for(unsigned i=0;i<64;++i)frame.measurement.errorReason[i]=SENSORARRAY_CELL_ERROR_READBACK;
    CHECK(saveFrame(argv[1],"health_before_conversion")==0);
    /* Evaluate the real encoder at maximum legal integer widths, including
     * mixed row values. Packet length must be bounded; no fake CRC fixture. */
    seedFrame(8,SENSORARRAY_MEASUREMENT_MODE_RESISTANCE);
    frame.sequence=UINT32_MAX;frame.timestampUs=UINT64_MAX;
    frame.identity.sessionId=UINT64_MAX;frame.identity.bootId=UINT32_MAX;frame.identity.revision=UINT32_MAX;
    frame.identity.modeGeneration=frame.identity.modeRequestId=frame.identity.rowsGeneration=frame.identity.rowsRequestId=frame.identity.profileGeneration=frame.identity.profileRequestId=UINT32_MAX;
    frame.configSnapshot.generation=frame.configSnapshot.requestId=UINT32_MAX;
    frame.rowProfileGeneration=frame.rowProfileRequestId=UINT32_MAX;
    frame.measurement.referenceSource=SENSORARRAY_ADS_REFERENCE_AVDD_AVSS;
    frame.measurement.modeGeneration=frame.measurement.modeRequestId=UINT32_MAX;
    frame.measurement.frameDurationUs=frame.measurement.transitionDurationUs=UINT64_MAX;
    frame.measurement.gainChangeCount=frame.measurement.overrangeCount=frame.measurement.autorangeAttemptCount=frame.measurement.autorangeFallbackCount=frame.measurement.ioRetryCount=frame.measurement.drdyTimeoutCount=frame.measurement.staleCount=frame.measurement.spiErrorCount=UINT32_MAX;
    frame.measurement.avddUv=INT32_MAX;frame.measurement.avssUv=INT32_MIN;frame.measurement.matrixReferenceUv=INT32_MAX;frame.measurement.referenceResistorOhms=UINT32_MAX;frame.measurement.railAgeFrames=UINT32_MAX;
    for(unsigned i=0;i<64;++i)frame.measurement.valuesFixed[i]=-SENSORARRAY_TEXT_MEASUREMENT_VALUE_MAX;
    CHECK(saveFrame(argv[1],"res_extreme")==0);
    frame.mixedProfile=true;
    for(unsigned i=0;i<8;++i){frame.rowMode[i]=i%2?SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;frame.rowScale[i]=i%2?-6:-3;}
    CHECK(saveFrame(argv[1],"mixed_extreme")==0);
    uint32_t selfChecks=0;CHECK(sensorarrayTextProtocolSelfTest(&frame,&selfChecks));
    printf("ENCODER_SELFTEST,checks=%u\n",(unsigned)selfChecks);
    puts("CONFIG_IDENTITY_TESTS,passed=1,modeTransitions=50");
    return 0;
}
