#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "sensorarrayTransportFrameMeta.h"

/* Linked from the SHA-pinned pre-optimization source fixture, not Git. */
void sensorarrayTransportBaselineParseFrameMeta(sensorarrayTransportPayloadSlot_t *slot);
bool sensorarrayTransportBaselineBuildPreview(const sensorarrayTransportPayloadSlot_t *slot,
                                              char *out, size_t outSize);

static uint32_t randomState = UINT32_C(0x84138a5b);
static unsigned compared, accepted, rejected;
static volatile uint64_t benchmarkSink;

static uint32_t randomWord(void)
{
    randomState ^= randomState << 13;
    randomState ^= randomState >> 17;
    randomState ^= randomState << 5;
    return randomState;
}

static void compare(const char *data, size_t length, uint16_t declaredLength,
                    sensorarrayTransportPayloadSlot_t *out)
{
    assert(length <= SENSORARRAY_TRANSPORT_POOL_TEXT_MAX);
    char oldData[SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE] = {0};
    char newData[SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE] = {0};
    sensorarrayTransportPayloadSlot_t oldSlot, newSlot;
    memset(&oldSlot, 0xa5, sizeof(oldSlot));
    oldSlot.inUse = true;
    oldSlot.hasFrameMeta = true;
    oldSlot.data = oldData;
    memcpy(oldSlot.data, data, length);
    oldSlot.length = declaredLength;
    memcpy(&newSlot, &oldSlot, sizeof(newSlot));
    newSlot.data = newData;
    memcpy(newData, oldData, sizeof(newData));
    sensorarrayTransportBaselineParseFrameMeta(&oldSlot);
    sensorarrayTransportParseFrameMeta(&newSlot);
    /* Compare metadata and bytes independently; storage addresses differ. */
    oldSlot.data = newSlot.data;
    if (memcmp(&oldSlot, &newSlot, sizeof(oldSlot)) != 0 ||
        memcmp(oldData, newData, sizeof(oldData)) != 0) {
        fprintf(stderr, "FRAME_META_MISMATCH,case=%u,len=%zu,declared=%u,input=%s\n",
                compared, length, declaredLength, data);
        abort();
    }
    ++compared;
    if (newSlot.hasFrameMeta) { ++accepted; } else { ++rejected; }
    if (out) {
        char *outData = out->data; assert(outData);
        memcpy(out, &newSlot, sizeof(newSlot));
        out->data = outData; memcpy(outData, newData, sizeof(newData));
    }
}

static void compareString(const char *text, sensorarrayTransportPayloadSlot_t *out)
{
    size_t length = strlen(text);
    compare(text, length, (uint16_t)length, out);
}

static void fixedCases(void)
{
    static const char *const cases[] = {
        "", "C", "C,", "N,seq=1,rows=1,cells=8", "MR,seq=1,rows=1,cells=8",
        "C,seq=1,rows=1,cells=8", "C,seq=0,rows=1,cells=8,", "C,seq=4294967295,rows=8,cells=64",
        "C,seq=4294967296,rows=1,cells=8", "C,seq=-1,rows=1,cells=8", "C,seq=+1,rows=1,cells=8",
        "C,seq= 1,rows=1,cells=8", "C,seq=\t+1,rows=1,cells=8", "C,seq=1 ,rows=1,cells=8",
        "C,seq=1,rows=0,cells=0", "C,seq=1,rows=9,cells=72", "C,seq=1,rows=1,cells=16",
        "C,seq=,seq=1,rows=1,cells=8", "C,seq=x,seq=1,rows=1,cells=8", "C,seq=1,seq=x,rows=1,cells=8",
        "C,seq=1,rows=1,cells=8,gen=,gen=3", "C,seq=1,rows=1,cells=8,gen=x,gen=3",
        "C,seq=1,rows=1,cells=8,gen=0x10", "C,seq=1,rows=1,cells=8,session=0xFFFFFFFFFFFFFFFF",
        "C,seq=1,rows=1,cells=8,session=FFFFFFFFFFFFFFFFF", "C,seq=1,rows=1,cells=8,session=\t-1",
        "M,seq=1,rows=1,cells=8,gen=99,rid=88,rgen=7,rrid=8,pgen=9,prid=10",
        "C,seq=1,rows=1,cells=8,mgen=7,xgen=8\nK,gen=999,rid=999\n",
        "C,seq=1,rows=1,cells=8,gen=2\rgen=4\n", "C,seq=1,rows=1,cells=8,gen=\nK,gen=99\n",
        "C,seq=1,rows=1,cells=8,gen=4294967295,rid=4294967295,mgen=4294967295,mrid=4294967295,rgen=4294967295,rrid=4294967295,pgen=4294967295,prid=4294967295,bootId=4294967295,rev=4294967295,session=FFFFFFFFFFFFFFFF",
        "C,seq=1,rows=1,cells=8,gen=4294967296,rid=4294967296,mgen=4294967296,mrid=4294967296,rgen=4294967296,rrid=4294967296,pgen=4294967296,prid=4294967296,bootId=4294967296,rev=4294967296,session=10000000000000000",
        "C,seq=1,rows=,rows=1,cells=,cells=8", "C,seq=1,rows=,rows=x,rows=1,cells=8",
        "C,seq=1,rows=1,cells=8,session=,session=0xABCD", "C,seq=1,rows=1,cells=8,session=z,session=ABCD",
        "C,seq=1,rows=1,cells=8,session=FFFFFFFFFFFFFFFFF,session=1",
        "C,seq=1,rows=1,cells=8,session=\t+0x12,mgen=\v+1,mrid=\f1",
        "C,seq=1,rows=1,cells=8, gen=5,genx=6,xgen=7,gen=2",
        "C,,cells=8,,rows=1,,seq=1,,", "C,seq=1,rows=1,cells=8\r\nK,gen=999\n",
        "V,seq=1,rows=1,cells=8,gen=7,rid=6,rgen=2,rrid=1,mgen=7,mrid=6",
        "R,seq=1,rows=1,cells=8,gen=7,rid=6,rgen=2,rrid=1,mgen=7,mrid=6"
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(*cases); ++i) {
        compareString(cases[i], NULL);
    }
    const char embeddedNul[] = "C,seq=1,rows=1,cells=8,gen=2\0,gen=99\n";
    compare(embeddedNul, sizeof(embeddedNul) - 1u, sizeof(embeddedNul) - 1u, NULL);
    const char leadingNul[] = "\0,seq=1,rows=1,cells=8";
    compare(leadingNul, sizeof(leadingNul) - 1u, sizeof(leadingNul) - 1u, NULL);
    const char *complete = "C,seq=1,rows=1,cells=8,gen=9";
    /* Preserve the old API's two-byte length guard, even for an inconsistent
     * synthetic slot. Real TransportQueue always appends NUL at length. */
    for (uint16_t declared = 0; declared < 3; ++declared) {
        compare(complete, strlen(complete), declared, NULL);
    }
    char maximum[SENSORARRAY_TRANSPORT_POOL_TEXT_MAX + 1u];
    memset(maximum, 'x', sizeof(maximum) - 1u);
    maximum[sizeof(maximum) - 1u] = '\0';
    static const char prefix[] = "C,seq=1,rows=1,cells=8,unknown=";
    memcpy(maximum, prefix, sizeof(prefix) - 1u);
    compare(maximum, sizeof(maximum) - 1u, sizeof(maximum) - 1u, NULL);

    /* Expected semantics independently asserted, beyond old/new equivalence. */
    char slotData[SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE];
    sensorarrayTransportPayloadSlot_t slot = {.data=slotData};
    compareString("C,seq=,seq=1,rows=1,cells=8,gen=,gen=3", &slot);
    assert(slot.hasFrameMeta && slot.frameSeq == 1u && slot.generation == 3u);
    compareString("C,seq=x,seq=1,rows=1,cells=8", &slot);
    assert(!slot.hasFrameMeta && slot.frameSeq == UINT32_C(0xa5a5a5a5));
    compareString("C,seq=1,rows=1,cells=8,gen=x,gen=3", &slot);
    assert(slot.hasFrameMeta && slot.generation == 0u);
    compareString("C,seq=4294967295,rows=8,cells=64,gen=4294967295,session=FFFFFFFFFFFFFFFF", &slot);
    assert(slot.hasFrameMeta && slot.frameSeq == UINT32_MAX && slot.generation == UINT32_MAX && slot.sessionId == UINT64_MAX);
    compareString("C,seq=1,rows=1,cells=8,gen=4294967296,session=10000000000000000", &slot);
    assert(slot.hasFrameMeta && slot.generation == 0u && slot.sessionId == 0u);
    compareString("M,seq=1,rows=1,cells=8,gen=99,rid=88,rgen=7,rrid=8,pgen=9,prid=10,mgen=11,mrid=12", &slot);
    assert(slot.hasFrameMeta && slot.generation == 7u && slot.requestId == 8u && slot.profileGeneration == 9u && slot.modeGeneration == 11u);
    compareString("C,seq=1,rows=1,cells=8,mgen=77\nK,gen=99,session=FFFFFFFFFFFFFFFF\n", &slot);
    assert(slot.hasFrameMeta && slot.generation == 0u && slot.modeGeneration == 77u && slot.sessionId == 0u);
    compareString("C,seq=\t+1,rows=1,cells=8,session=0xABCD", &slot);
    assert(slot.hasFrameMeta && slot.frameSeq == 1u && slot.sessionId == UINT64_C(0xabcd));
    compare(embeddedNul, sizeof(embeddedNul) - 1u, sizeof(embeddedNul) - 1u, &slot);
    assert(slot.hasFrameMeta && slot.generation == 2u);

    for (unsigned tag = 0; tag < 4; ++tag) {
        for (unsigned rows = 1; rows <= 8; ++rows) {
            char text[256];
            int n = snprintf(text, sizeof(text), "%c,seq=0,rows=%u,cells=%u,gen=4,rid=3,rgen=2,rrid=1,mgen=4,mrid=3,pgen=5,prid=6,session=FFFFFFFFFFFFFFFF,bootId=4294967295,rev=4294967295\n", "CVRM"[tag], rows, rows * 8u);
            assert(n > 0 && (size_t)n < sizeof(text));
            compareString(text, &slot);
            assert(slot.hasFrameMeta && slot.rows == rows && slot.cells == rows * 8u);
            assert(slot.generation == (tag == 3u ? 2u : 4u));
            assert(slot.rowsGeneration == 2u && slot.modeGeneration == 4u && slot.profileGeneration == 5u);
        }
    }
}

static void seededCases(void)
{
    static const char *const keys[] = {"seq=", "rows=", "cells=", "gen=", "rid=", "session=", "bootId=", "rev=", "mgen=", "mrid=", "rgen=", "rrid=", "pgen=", "prid="};
    static const char *const values[] = {"", "0", "1", "8", "64", "4294967295", "4294967296", "18446744073709551615", "18446744073709551616", "-1", "+1", " 1", "\t1", "\t+1", "\v1", "\f1", "1 ", "0x10", "0XFFFFFFFFFFFFFFFF", "z", "01", "1x"};
    for (unsigned n = 0; n < 50000u; ++n) {
        char text[SENSORARRAY_TRANSPORT_POOL_TEXT_MAX + 1u];
        char tag = "CVRMX"[randomWord() % 5u];
        size_t at = (size_t)snprintf(text, sizeof(text), "%c,seq=%s,rows=1,cells=8", tag, n % 7u ? "1" : "");
        for (unsigned k = 0u, count = randomWord() % 25u; k < count; ++k) {
            const char *key = keys[randomWord() % (sizeof(keys) / sizeof(*keys))];
            const char *value = values[randomWord() % (sizeof(values) / sizeof(*values))];
            int written = snprintf(text + at, sizeof(text) - at, ",%s%s%s", randomWord() % 11u == 0u ? "x" : "", key, value);
            assert(written >= 0 && (size_t)written < sizeof(text) - at);
            at += (size_t)written;
        }
        const char *tail = n % 3u == 0u ? "\r\nK,gen=99,rid=98\n" : n % 3u == 1u ? "\nK,gen=99\n" : "";
        int written = snprintf(text + at, sizeof(text) - at, "%s", tail);
        assert(written >= 0 && (size_t)written < sizeof(text) - at);
        at += (size_t)written;
        compare(text, at, (uint16_t)at, NULL);
    }
}

static double nowNs(void)
{
    struct timespec time;
    assert(clock_gettime(CLOCK_MONOTONIC, &time) == 0);
    return time.tv_sec * 1e9 + time.tv_nsec;
}

static double benchmark(void (*parse)(sensorarrayTransportPayloadSlot_t *),
                        sensorarrayTransportPayloadSlot_t *slot)
{
    uint64_t sum = 0u;
    double start = nowNs();
    for (unsigned i = 0; i < 100000u; ++i) {
        parse(slot);
        sum += slot->frameSeq + slot->modeGeneration + slot->hasFrameMeta;
    }
    benchmarkSink += sum;
    return (nowNs() - start) / 100000.0;
}

int main(int argc, char **argv)
{
    sensorarrayTransportBaselineParseFrameMeta(NULL);
    sensorarrayTransportParseFrameMeta(NULL);
    fixedCases();
    unsigned fixedCount = compared;
    seededCases();
    assert(accepted != 0u && rejected != 0u);
    bool measure = argc > 1 && strcmp(argv[1], "--benchmark") == 0;
    int first = measure ? 2 : 1;
    unsigned fixtureCount = 0, truncatedCount = 0;
    for (int i = first; i < argc; ++i) {
        FILE *file = fopen(argv[i], "rb");
        assert(file);
        char data[SENSORARRAY_TRANSPORT_POOL_TEXT_MAX + 1u];
        size_t length = fread(data, 1u, sizeof(data), file);
        assert(!ferror(file) && feof(file) && length <= SENSORARRAY_TRANSPORT_POOL_TEXT_MAX);
        fclose(file);
        data[length] = '\0';
        char slotData[SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE];
    sensorarrayTransportPayloadSlot_t slot = {.data=slotData};
        compare(data, length, (uint16_t)length, &slot);
        assert(slot.hasFrameMeta);
        ++fixtureCount;
        for (size_t cut = 0u; cut < length; ++cut) {
            compare(data, cut, (uint16_t)cut, NULL);
            ++truncatedCount;
        }
        char oldPreview[384], newPreview[384];
        memset(oldPreview, 0xcc, sizeof(oldPreview));
        memset(newPreview, 0xcc, sizeof(newPreview));
        assert(sensorarrayTransportBaselineBuildPreview(&slot, oldPreview, sizeof(oldPreview)) ==
               sensorarrayTransportBuildPreview(&slot, newPreview, sizeof(newPreview)));
        assert(memcmp(oldPreview, newPreview, sizeof(oldPreview)) == 0);
        if (measure) {
            for (unsigned round = 0; round < 3u; ++round) {
                double oldNs, newNs;
                if (round & 1u) {
                    newNs = benchmark(sensorarrayTransportParseFrameMeta, &slot);
                    oldNs = benchmark(sensorarrayTransportBaselineParseFrameMeta, &slot);
                } else {
                    oldNs = benchmark(sensorarrayTransportBaselineParseFrameMeta, &slot);
                    newNs = benchmark(sensorarrayTransportParseFrameMeta, &slot);
                }
                printf("FRAME_META_HOST_BENCH,fixture=%s,round=%u,iterations=100000,oldNs=%.3f,newNs=%.3f,scope=host_only\n", argv[i], round, oldNs, newNs);
            }
        }
    }
    assert(fixtureCount > 0u);
    printf("TRANSPORT_FRAME_METADATA_DIFFERENTIAL_PASS,fixed=%u,seeded=50000,fixtures=%u,truncated=%u,total=%u,accepted=%u,rejected=%u,slotBytesCompared=%zu,preview=unchanged\n",
           fixedCount, fixtureCount, truncatedCount, compared, accepted, rejected, sizeof(sensorarrayTransportPayloadSlot_t));
    return 0;
}
