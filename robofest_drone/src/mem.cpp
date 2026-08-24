#include "mem.h"

#if defined(ARDUINO) && defined(BOARD_HAS_PSRAM)
#include <esp_heap_caps.h>
#if __has_include(<esp_memory_utils.h>)
#include <esp_memory_utils.h>
#define ROBOFEST_PTR_IN_PSRAM(p) esp_ptr_external_psram(p)
#elif __has_include(<soc/soc_memory_layout.h>)
#include <soc/soc_memory_layout.h>
#define ROBOFEST_PTR_IN_PSRAM(p) esp_ptr_external_ram(p)
#endif
#ifndef ROBOFEST_PTR_IN_PSRAM
#define ROBOFEST_HAS_HEAP_CAPS 1
#endif
#define ROBOFEST_HAS_HEAP_CAPS 1
#endif

#include <cstdlib>

namespace RobofestDrone {

void* robofest_big_alloc(size_t n) {
    if (n == 0) return nullptr;
    void* p = nullptr;
#ifdef ROBOFEST_HAS_HEAP_CAPS
    // Prefer external PSRAM; fall back to the largest available internal pool.
    p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    if (p == nullptr) {
        p = heap_caps_malloc(n, MALLOC_CAP_DEFAULT);
    }
#else
    p = std::malloc(n);
#endif
    return p;
}

void robofest_big_free(void* p) {
    if (p == nullptr) return;
#ifdef ROBOFEST_HAS_HEAP_CAPS
    heap_caps_free(p);
#else
    std::free(p);
#endif
}

bool robofest_big_is_psram(const void* p) {
    if (p == nullptr) return false;
#if defined(ROBOFEST_PTR_IN_PSRAM)
    return ROBOFEST_PTR_IN_PSRAM(p);
#else
    (void)p;
    return false;
#endif
}

} // namespace RobofestDrone
