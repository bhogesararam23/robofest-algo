#pragma once

#include <stdint.h>
#include "../types.h"
#include "../vision_pipeline.h"
#include "../../config/thresholds.h"

namespace RobofestDrone {

struct HsvTunerStats {
    uint32_t total_pixels = 0;
    uint32_t masked_pixels = 0;
    float masked_ratio = 0.0f;
    uint8_t avg_hue = 0;
    uint8_t avg_sat = 0;
    uint8_t avg_val = 0;
};

// Onboard HSV calibration tuner. Operates directly on the VisionPipeline's
// shared profile table (no duplicated profile storage), so tuned values are
// exactly what the pipeline consumes. Hue scale matches the pipeline (0-180,
// OpenCV-compatible).
class HsvTuner {
public:
    HsvTuner();

    void init();
    void reset();

    bool setActiveProfileByIndex(uint8_t index);
    bool setActiveProfileByType(Types::VisionMarkerType type);
    void cycleProfile();
    Types::VisionMarkerType getActiveProfileType() const;
    uint8_t getActiveProfileIndex() const { return active_index_; }

    // Select which HSV variant (primary "sunny" / alt "overcast") the delta
    // commands and frame statistics target.
    void setEditAltBand(bool alt) { edit_alt_ = alt; }
    bool editingAltBand() const { return edit_alt_; }

    VisionMarkerProfile* getActiveProfile();
    const VisionMarkerProfile* getActiveProfile() const;

    // Process frame buffer and update color segmentation statistics
    void processFrame(const uint8_t* rgb_frame, uint16_t width, uint16_t height);

    // Apply serial command adjustments:
    //   "H_MIN +5", "V_MAX -10", "EXT_MIN +5" (+0.05 extent gate),
    //   "CORN_MIN -1", "SAVE", "RESET", "PROFILE_NEXT", "PROFILE_N <i>",
    //   "PROFILE_GROUND", "PROFILE_BURIED", "BAND_ALT", "BAND_PRIMARY",
    //   "ENABLE_ALL"
    bool parseCommand(const char* cmd_str);

    // Persistence with HAL storage (one telemetry record per profile; the
    // laptop-tool JSON/export path remains the full-fidelity calibration store)
    bool saveToStorage();
    bool loadFromStorage();

    const HsvTunerStats& getStats() const { return stats_; }

    void printStatus() const;

private:
    void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint8_t& h, uint8_t& s, uint8_t& v) const;

private:
    uint8_t active_index_ = 0;
    bool edit_alt_ = false;
    HsvTunerStats stats_;
};

} // namespace RobofestDrone
