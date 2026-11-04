#pragma once
#include <stddef.h>
#include <stdint.h>
#define MALLOC_CAP_SPIRAM 0x01u
#define MALLOC_CAP_INTERNAL 0x02u
#define MALLOC_CAP_8BIT 0x04u
void *heap_caps_calloc(size_t count, size_t size, uint32_t caps);
