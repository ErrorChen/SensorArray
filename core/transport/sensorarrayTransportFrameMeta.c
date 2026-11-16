#include "sensorarrayTransportFrameMeta.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool field(const char *text, const char *key, int base, uint64_t maximum, uint64_t *value)
{
    size_t keyLength = strlen(key);
    const char *endLine = strpbrk(text, "\r\n");
    if (!endLine) { endLine = text + strlen(text); }
    for (const char *p = text; p < endLine;) {
        const char *end = memchr(p, ',', (size_t)(endLine - p));
        if (!end) { end = endLine; }
        if ((size_t)(end - p) > keyLength && memcmp(p, key, keyLength) == 0) {
            const char *start = p + keyLength;
            if (*start == '-' || *start == '+' || *start == ' ') { return false; }
            errno = 0;
            char *parsedEnd;
            unsigned long long parsed = strtoull(start, &parsedEnd, base);
            if (errno || parsedEnd != end || parsed > maximum) { return false; }
            *value = (uint64_t)parsed;
            return true;
        }
        p = end < endLine ? end + 1 : endLine;
    }
    return false;
}

enum {
    META_SEQ, META_ROWS, META_CELLS, META_GEN, META_RID, META_SESSION,
    META_BOOT, META_REV, META_MGEN, META_MRID, META_RGEN, META_RRID,
    META_PGEN, META_PRID, META_COUNT
};

/* 本轮新增: exact keys for the single-pass metadata extractor. The original
 * field helper remains unchanged for the separate CAP preview path. */
static const struct {
    const char *key;
    size_t length;
} s_metaFields[META_COUNT] = {
#define META_KEY(name) {name, sizeof(name) - 1u}
    META_KEY("seq="), META_KEY("rows="), META_KEY("cells="),
    META_KEY("gen="), META_KEY("rid="), META_KEY("session="),
    META_KEY("bootId="), META_KEY("rev="), META_KEY("mgen="),
    META_KEY("mrid="), META_KEY("rgen="), META_KEY("rrid="),
    META_KEY("pgen="), META_KEY("prid="),
#undef META_KEY
};

void sensorarrayTransportParseFrameMeta(sensorarrayTransportPayloadSlot_t *slot)
{
    if (!slot) { return; }
    slot->hasFrameMeta = false;
    if (slot->length < 2 || slot->data[1] != ',' ||
        !strchr("CVRM", slot->data[0])) { return; }
    /* TransportQueue owns the slot and appends its private NUL at length.
     * Extract only the first line; this does not validate a frame or its CRC. */
    const char *endLine = strpbrk(slot->data, "\r\n");
    if (!endLine) { endLine = slot->data + strlen(slot->data); }
    bool mixed = slot->data[0] == 'M';
    uint64_t values[META_COUNT] = {0};
    uint32_t seen = mixed ? (UINT32_C(1) << META_GEN) | (UINT32_C(1) << META_RID) : 0u;
    uint32_t valid = 0u;
    for (const char *p = slot->data; p < endLine;) {
        const char *end = memchr(p, ',', (size_t)(endLine - p));
        if (!end) { end = endLine; }
        for (unsigned i = 0u; i < META_COUNT; ++i) {
            uint32_t bit = UINT32_C(1) << i;
            if ((seen & bit) || (size_t)(end - p) <= s_metaFields[i].length ||
                *p != *s_metaFields[i].key ||
                memcmp(p, s_metaFields[i].key, s_metaFields[i].length) != 0) {
                continue;
            }
            /* Preserve first-NONEMPTY semantics: an empty value may be
             * followed by a usable duplicate; a malformed nonempty one may
             * not. Optional invalid/missing values retain their zero default. */
            seen |= bit;
            const char *start = p + s_metaFields[i].length;
            if (*start != '-' && *start != '+' && *start != ' ') {
                uint64_t maximum = i == META_SESSION ? UINT64_MAX :
                    i == META_ROWS ? 8u : i == META_CELLS ? 64u : UINT32_MAX;
                errno = 0;
                char *parsedEnd;
                unsigned long long parsed = strtoull(start, &parsedEnd,
                    i == META_SESSION ? 16 : 10);
                if (!errno && parsedEnd == end && parsed <= maximum) {
                    values[i] = (uint64_t)parsed;
                    valid |= bit;
                }
            }
            break;
        }
        p = end < endLine ? end + 1 : endLine;
    }
    const uint32_t required = (UINT32_C(1) << META_SEQ) |
        (UINT32_C(1) << META_ROWS) | (UINT32_C(1) << META_CELLS);
    if ((valid & required) != required || values[META_ROWS] < 1u ||
        values[META_CELLS] != values[META_ROWS] * 8u) { return; }
    /* Commit only after required fields pass; do not restamp from globals or
     * reinterpret the legacy domain. M aliases ROWS, C/V/R retain gen/rid. */
    slot->frameSeq = (uint32_t)values[META_SEQ];
    slot->rows = (uint8_t)values[META_ROWS];
    slot->cells = (uint8_t)values[META_CELLS];
    slot->generation = (uint32_t)values[mixed ? META_RGEN : META_GEN];
    slot->requestId = (uint32_t)values[mixed ? META_RRID : META_RID];
    slot->sessionId = values[META_SESSION];
    slot->bootId = (uint32_t)values[META_BOOT];
    slot->configRevision = (uint32_t)values[META_REV];
    slot->modeGeneration = (uint32_t)values[META_MGEN];
    slot->modeRequestId = (uint32_t)values[META_MRID];
    slot->rowsGeneration = (uint32_t)values[META_RGEN];
    slot->rowsRequestId = (uint32_t)values[META_RRID];
    slot->profileGeneration = (uint32_t)values[META_PGEN];
    slot->profileRequestId = (uint32_t)values[META_PRID];
    slot->hasFrameMeta = true;
}

bool sensorarrayTransportBuildPreview(const sensorarrayTransportPayloadSlot_t *slot,
                                               char *out,
                                               size_t outSize)
{
    if (!slot || !slot->hasFrameMeta || !out || outSize == 0u) {
        return false;
    }
    const char *data = slot->data;
    size_t length = slot->length;
    if (length == 0u) {
        return false;
    }
    const char *firstLineEnd = memchr(data, '\n', length);
    if (!firstLineEnd || data[0] != 'C' || data[1] != ',') {
        return false;
    }
    char cLine[512];
    size_t cLineLength = (size_t)(firstLineEnd - data);
    if (cLineLength >= sizeof(cLine)) {
        return false;
    }
    memcpy(cLine, data, cLineLength);
    cLine[cLineLength] = '\0';

    uint64_t rf = 0, pf = 0, sf = 0;
    (void)field(cLine, "rf=", 16, UINT32_MAX, &rf);
    (void)field(cLine, "pf=", 16, UINT32_MAX, &pf);
    (void)field(cLine, "sf=", 16, UINT32_MAX, &sf);

    const char *bad = strstr(cLine, "bad=");
    char badText[24] = "na";
    if (bad) {
        bad += 4u;
        size_t index = 0u;
        while (bad[index] != '\0' && bad[index] != ',' && index + 1u < sizeof(badText)) {
            badText[index] = bad[index];
            index++;
        }
        badText[index] = '\0';
    }

    int32_t first = 0;
    int32_t last = 0;
    bool firstSet = false;
    const char *cursor = firstLineEnd + 1;
    const char *end = data + length;
    while (cursor < end) {
        const char *lineEnd = memchr(cursor, '\n', (size_t)(end - cursor));
        if (!lineEnd) {
            break;
        }
        if (cursor[0] == 'D') {
            const char *value = strchr(cursor, ',');
            while (value && value < lineEnd) {
                value++;
                char *valueEnd = NULL;
                long parsed = strtol(value, &valueEnd, 10);
                if (valueEnd == value || valueEnd > lineEnd) {
                    break;
                }
                if (!firstSet) {
                    first = (int32_t)parsed;
                    firstSet = true;
                }
                last = (int32_t)parsed;
                value = valueEnd < lineEnd ? strchr(valueEnd, ',') : NULL;
            }
        }
        cursor = lineEnd + 1;
    }

    char firstText[16] = "na", lastText[16] = "na";
    if (firstSet) {
        (void)snprintf(firstText, sizeof(firstText), "%ld", (long)first);
        (void)snprintf(lastText, sizeof(lastText), "%ld", (long)last);
    }
    int written = snprintf(out, outSize,
        "B20,seq=%lu,rows=%u,cfps=na,efps=na,rf=%02lX,pf=%02lX,sf=%02lX,bad=%s,d0=%s,dl=%s,previewOnly=1,gen=%lu,rid=%lu,session=%016llX,bootId=%lu,rev=%lu,mgen=%lu,mrid=%lu,rgen=%lu,rrid=%lu,pgen=%lu,prid=%lu\n",
        (unsigned long)slot->frameSeq, (unsigned)slot->rows,
        (unsigned long)rf, (unsigned long)pf, (unsigned long)sf, badText, firstText, lastText,
        (unsigned long)slot->generation, (unsigned long)slot->requestId,
        (unsigned long long)slot->sessionId, (unsigned long)slot->bootId,
        (unsigned long)slot->configRevision, (unsigned long)slot->modeGeneration,
        (unsigned long)slot->modeRequestId, (unsigned long)slot->rowsGeneration,
        (unsigned long)slot->rowsRequestId, (unsigned long)slot->profileGeneration,
        (unsigned long)slot->profileRequestId);
    return written > 0 && (size_t)written < outSize;
}

