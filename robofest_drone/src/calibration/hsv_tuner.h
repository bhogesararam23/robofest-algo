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

class HsvTuner {
public:
    HsvTuner();

    void init();
    void reset();

    void setActiveProfile(Types::VisionMarkerType profile_type);
    Types::VisionMarkerType getActiveProfileType() const { return active_profile_type_; }

    VisionMarkerProfile& getActiveProfile() { return active_profile_; }
    const VisionMarkerProfile& getActiveProfile() const { return active_profile_; }

    // Process frame buffer and update color segmentation statistics
    void processFrame(const uint8_t* rgb_frame, uint16_t width, uint16_t height);

    // Apply serial command adjustments (e.g. "H_MIN +5", "V_MAX -10", "SAVE", "RESET")
    bool parseCommand(const char* cmd_str);

    // Persistence with HAL storage
    bool saveToStorage();
    bool loadFromStorage();

    const HsvTunerStats& getStats() const { return stats_; }

    void printStatus() const;

private:
    void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint8_t& h, uint8_t& s, uint8_t& v) const;

private:
    Types::VisionMarkerType active_profile_type_ = Types::VisionMarkerType::ON_GROUND_MINE;
    VisionMarkerProfile active_profile_;
    VisionMarkerProfile on_ground_profile_;
    VisionMarkerProfile buried_marker_profile_;

    HsvTunerStats stats_;
};

} // namespace RobofestDrone
