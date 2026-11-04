#pragma once
#include <stddef.h>
#define MALLOC_CAP_DMA 1
#define MALLOC_CAP_8BIT 2
void *heap_caps_malloc(size_t size, int flags);
void heap_caps_free(void *ptr);
