#pragma once

#include <stddef.h>

namespace RobofestDrone {

// ============================================================================
// BIG-BUFFER ALLOCATOR (REQ item 9 / DRAM relief)
// ----------------------------------------------------------------------------
// Large scratch buffers (vision masks, BFS queues, grids) must not live in
// precious internal DRAM on the ESP32-S3. On target hardware this allocator
// prefers PSRAM (8 MB) and falls back to internal heap; on host test builds
// it is plain malloc/free so unit tests exercise identical code paths.
// ============================================================================

// Allocates n bytes, preferring PSRAM when available. Returns nullptr on OOM.
void* robofest_big_alloc(size_t n);

// Frees a buffer obtained from robofest_big_alloc (nullptr is a safe no-op).
void robofest_big_free(void* p);

// True when the last robofest_big_alloc landed in external PSRAM.
bool robofest_big_is_psram(const void* p);

} // namespace RobofestDrone
