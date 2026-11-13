#pragma once
#include <stdbool.h>
/* Test-only address classification injection; no fake physical-memory claim. */
static bool workBudgetTestStorageInternal = true;
static inline bool esp_ptr_in_dram(const void *address)
{
    (void)address;
    return workBudgetTestStorageInternal;
}
