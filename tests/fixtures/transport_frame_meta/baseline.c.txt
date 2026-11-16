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

static uint32_t u32(const char *text, const char *key)
{
    uint64_t value = 0;
    (void)field(text, key, 10, UINT32_MAX, &value);
    return (uint32_t)value;
}

void sensorarrayTransportParseFrameMeta(sensorarrayTransportPayloadSlot_t *slot)
{
    if (!slot) { return; }
    slot->hasFrameMeta = false;
    if (slot->length < 2 || slot->data[1] != ',' ||
        !strchr("CVRM", slot->data[0])) { return; }
    uint64_t rows, cells, sequence;
    if (!field(slot->data, "seq=", 10, UINT32_MAX, &sequence) ||
        !field(slot->data, "rows=", 10, 8, &rows) || rows < 1 ||
        !field(slot->data, "cells=", 10, 64, &cells) || cells != rows * 8) { return; }
    slot->frameSeq = (uint32_t)sequence;
    slot->rows = (uint8_t)rows;
    slot->cells = (uint8_t)cells;
    slot->generation = u32(slot->data, slot->data[0] == 'M' ? "rgen=" : "gen=");
    slot->requestId = u32(slot->data, slot->data[0] == 'M' ? "rrid=" : "rid=");
    slot->sessionId = 0;
    (void)field(slot->data, "session=", 16, UINT64_MAX, &slot->sessionId);
    slot->bootId = u32(slot->data, "bootId=");
    slot->configRevision = u32(slot->data, "rev=");
    slot->modeGeneration = u32(slot->data, "mgen=");
    slot->modeRequestId = u32(slot->data, "mrid=");
    slot->rowsGeneration = u32(slot->data, "rgen=");
    slot->rowsRequestId = u32(slot->data, "rrid=");
    slot->profileGeneration = u32(slot->data, "pgen=");
    slot->profileRequestId = u32(slot->data, "prid=");
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

