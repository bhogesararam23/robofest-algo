#pragma once

#include <stdint.h>

namespace RobofestDrone {

// ============================================================================
// STAGE PROFILER (REQ-DER-118, item 18)
// ----------------------------------------------------------------------------
// High-resolution execution-time instrumentation for the control and CV
// hot paths. Each stage tracks count / avg / max microseconds against a
// hard budget; budget violations raise an alert flag consumed by telemetry
// (and a throttled log) so regressions surface in bench runs immediately.
//
// Usage in the pipeline:
//   {
//       ProfScope guard(PROF_SEGMENT);
//       ... segment work ...
//   }   // timing committed on scope exit
//
// Zero allocation, lock-free single-threaded by design; the vision task
// owns all calls. Host builds use the same hal_micros() clock as target.
// ============================================================================

enum ProfStage : uint8_t {
    PROF_FRAME_ACQ = 0,   // hal_camera_get_frame + adapter convert
    PROF_ENHANCE,         // night blend / gamma / CLAHE / dehaze / shadow
    PROF_SEGMENT,         // HSV multi-label segmentation
    PROF_MORPH,           // separable morphology cleanup
    PROF_BLOBS,           // connected components + shape descriptors
    PROF_TRACKS,          // persistence fusion + candidate report
    PROF_CONTROL,         // state machine + planner + safety tick
    PROF_STAGE_COUNT
};

struct ProfStats {
    uint32_t samples = 0;
    float avg_us = 0.0f;
    uint32_t max_us = 0;
    uint32_t budget_us = 0;   // 0 = no budget enforced
    uint32_t overruns = 0;
};

class Profiler {
public:
    static Profiler& instance();

    void begin(ProfStage s);
    void end(ProfStage s);

    const ProfStats& stats(ProfStage s) const { return stats_[s]; }
    bool anyBudgetExceeded() const;

    // Emits one "[PROF] ..." summary line per stage through hal_log,
    // throttled to once per interval from the caller's timestamp.
    void dump(uint32_t now_ms, uint32_t interval_ms);

    void reset();

private:
    Profiler();

    ProfStats stats_[PROF_STAGE_COUNT];
    uint32_t start_us_[PROF_STAGE_COUNT];
    bool active_[PROF_STAGE_COUNT];
    uint32_t last_dump_ms_ = 0;
};

// RAII guard for exception/skip-safe stage timing.
class ProfScope {
public:
    explicit ProfScope(ProfStage s) : stage_(s) {
        Profiler::instance().begin(stage_);
    }
    ~ProfScope() {
        Profiler::instance().end(stage_);
    }
    ProfScope(const ProfScope&) = delete;
    ProfScope& operator=(const ProfScope&) = delete;

private:
    ProfStage stage_;
};

} // namespace RobofestDrone
