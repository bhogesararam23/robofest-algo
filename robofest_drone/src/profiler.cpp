#include "profiler.h"
#include "../hal/hal_system.h"
#include "../config/thresholds.h"
#include <cstdio>

namespace RobofestDrone {

namespace {
constexpr const char* kStageNames[PROF_STAGE_COUNT] = {
    "frame_acq", "enhance", "segment", "morph",
    "blobs", "tracks", "control",
};
}

Profiler& Profiler::instance() {
    static Profiler p;
    return p;
}

Profiler::Profiler() {
    reset();
}

void Profiler::reset() {
    for (uint8_t s = 0; s < PROF_STAGE_COUNT; ++s) {
        stats_[s] = ProfStats();
        start_us_[s] = 0;
        active_[s] = false;
    }
    stats_[PROF_FRAME_ACQ].budget_us = Config::PROF_BUDGET_FRAME_US;
    stats_[PROF_ENHANCE].budget_us = Config::PROF_BUDGET_ENHANCE_US;
    stats_[PROF_SEGMENT].budget_us = Config::PROF_BUDGET_SEGMENT_US;
    stats_[PROF_MORPH].budget_us = Config::PROF_BUDGET_MORPH_US;
    stats_[PROF_BLOBS].budget_us = Config::PROF_BUDGET_BLOBS_US;
    stats_[PROF_TRACKS].budget_us = Config::PROF_BUDGET_TRACKS_US;
    stats_[PROF_CONTROL].budget_us = Config::PROF_BUDGET_CONTROL_US;
    last_dump_ms_ = 0;
}

void Profiler::begin(ProfStage s) {
    if (s >= PROF_STAGE_COUNT) return;
    start_us_[s] = Hal::hal_micros();
    active_[s] = true;
}

void Profiler::end(ProfStage s) {
    if (s >= PROF_STAGE_COUNT || !active_[s]) return;
    active_[s] = false;

    const uint32_t now_us = Hal::hal_micros();
    uint32_t dt = now_us - start_us_[s]; // wraps are statistically harmless

    ProfStats& st = stats_[s];
    st.samples++;
    // Running average without floats accumulating error:
    st.avg_us += (static_cast<float>(dt) - st.avg_us) /
                 static_cast<float>(st.samples);
    if (dt > st.max_us) st.max_us = dt;

    if (st.budget_us != 0 && dt > st.budget_us) {
        st.overruns++;
    }
}

bool Profiler::anyBudgetExceeded() const {
    for (uint8_t s = 0; s < PROF_STAGE_COUNT; ++s) {
        if (stats_[s].budget_us != 0 && stats_[s].overruns > 0) return true;
    }
    return false;
}

void Profiler::dump(uint32_t now_ms, uint32_t interval_ms) {
    if (last_dump_ms_ != 0 &&
        (now_ms - last_dump_ms_) < interval_ms) {
        return;
    }
    last_dump_ms_ = now_ms;

    for (uint8_t s = 0; s < PROF_STAGE_COUNT; ++s) {
        const ProfStats& st = stats_[s];
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "[PROF] stage=%u name=%s n=%lu avg_us=%.1f max_us=%lu "
                      "budget_us=%lu over=%lu",
                      static_cast<unsigned>(s), kStageNames[s],
                      static_cast<unsigned long>(st.samples),
                      static_cast<double>(st.avg_us),
                      static_cast<unsigned long>(st.max_us),
                      static_cast<unsigned long>(st.budget_us),
                      static_cast<unsigned long>(st.overruns));
        Hal::hal_log(buf);
    }
}

} // namespace RobofestDrone
