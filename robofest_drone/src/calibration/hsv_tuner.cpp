#include "hsv_tuner.h"
#include "../../hal/hal_system.h"
#include "../../hal/hal_storage.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace RobofestDrone {

HsvTuner::HsvTuner() {
    init();
}

void HsvTuner::init() {
    // Default profile for on-ground high-contrast mine markers
    on_ground_profile_.profile_type = Types::VisionMarkerType::ON_GROUND_MINE;
    on_ground_profile_.enabled = true;
    on_ground_profile_.h_min = Config::ON_GROUND_MINE_HSV_LOW.h;
    on_ground_profile_.h_max = Config::ON_GROUND_MINE_HSV_HIGH.h;
    on_ground_profile_.s_min = Config::ON_GROUND_MINE_HSV_LOW.s;
    on_ground_profile_.s_max = Config::ON_GROUND_MINE_HSV_HIGH.s;
    on_ground_profile_.v_min = Config::ON_GROUND_MINE_HSV_LOW.v;
    on_ground_profile_.v_max = Config::ON_GROUND_MINE_HSV_HIGH.v;

    // Default profile for buried surface markers (e.g. ribbon markers)
    buried_marker_profile_.profile_type = Types::VisionMarkerType::BURIED_SURFACE_MARKER;
    buried_marker_profile_.enabled = true;
    buried_marker_profile_.h_min = Config::BURIED_SURFACE_MARKER_HSV_LOW.h;
    buried_marker_profile_.h_max = Config::BURIED_SURFACE_MARKER_HSV_HIGH.h;
    buried_marker_profile_.s_min = Config::BURIED_SURFACE_MARKER_HSV_LOW.s;
    buried_marker_profile_.s_max = Config::BURIED_SURFACE_MARKER_HSV_HIGH.s;
    buried_marker_profile_.v_min = Config::BURIED_SURFACE_MARKER_HSV_LOW.v;
    buried_marker_profile_.v_max = Config::BURIED_SURFACE_MARKER_HSV_HIGH.v;

    active_profile_type_ = Types::VisionMarkerType::ON_GROUND_MINE;
    active_profile_ = on_ground_profile_;
    stats_ = HsvTunerStats();
}

void HsvTuner::reset() {
    init();
}

void HsvTuner::setActiveProfile(Types::VisionMarkerType profile_type) {
    if (active_profile_type_ == Types::VisionMarkerType::ON_GROUND_MINE) {
        on_ground_profile_ = active_profile_;
    } else {
        buried_marker_profile_ = active_profile_;
    }

    active_profile_type_ = profile_type;
    if (profile_type == Types::VisionMarkerType::ON_GROUND_MINE) {
        active_profile_ = on_ground_profile_;
    } else {
        active_profile_ = buried_marker_profile_;
    }
}

void HsvTuner::rgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint8_t& h, uint8_t& s, uint8_t& v) const {
    uint8_t rgb_min = std::min({r, g, b});
    uint8_t rgb_max = std::max({r, g, b});
    v = rgb_max;
    if (v == 0) {
        h = 0;
        s = 0;
        return;
    }
    s = static_cast<uint8_t>(255 * (static_cast<int32_t>(rgb_max - rgb_min)) / v);
    if (s == 0) {
        h = 0;
        return;
    }
    if (rgb_max == r) {
        h = static_cast<uint8_t>(0 + 43 * (g - b) / (rgb_max - rgb_min));
    } else if (rgb_max == g) {
        h = static_cast<uint8_t>(85 + 43 * (b - r) / (rgb_max - rgb_min));
    } else {
        h = static_cast<uint8_t>(171 + 43 * (r - g) / (rgb_max - rgb_min));
    }
}

void HsvTuner::processFrame(const uint8_t* rgb_frame, uint16_t width, uint16_t height) {
    if (rgb_frame == nullptr || width == 0 || height == 0) return;

    uint32_t total_px = width * height;
    uint32_t masked_count = 0;
    uint32_t sum_h = 0;
    uint32_t sum_s = 0;
    uint32_t sum_v = 0;

    for (uint32_t i = 0; i < total_px; ++i) {
        uint8_t r = rgb_frame[i * 3 + 0];
        uint8_t g = rgb_frame[i * 3 + 1];
        uint8_t b = rgb_frame[i * 3 + 2];

        uint8_t h = 0, s = 0, v = 0;
        rgbToHsv(r, g, b, h, s, v);

        // Check if pixel falls inside active HSV threshold window
        bool h_pass = false;
        if (active_profile_.h_min <= active_profile_.h_max) {
            h_pass = (h >= active_profile_.h_min && h <= active_profile_.h_max);
        } else {
            // Hue wraparound
            h_pass = (h >= active_profile_.h_min || h <= active_profile_.h_max);
        }

        bool s_pass = (s >= active_profile_.s_min && s <= active_profile_.s_max);
        bool v_pass = (v >= active_profile_.v_min && v <= active_profile_.v_max);

        if (h_pass && s_pass && v_pass) {
            masked_count++;
            sum_h += h;
            sum_s += s;
            sum_v += v;
        }
    }

    stats_.total_pixels = total_px;
    stats_.masked_pixels = masked_count;
    stats_.masked_ratio = (total_px > 0) ? (static_cast<float>(masked_count) / static_cast<float>(total_px)) : 0.0f;
    stats_.avg_hue = (masked_count > 0) ? static_cast<uint8_t>(sum_h / masked_count) : 0;
    stats_.avg_sat = (masked_count > 0) ? static_cast<uint8_t>(sum_s / masked_count) : 0;
    stats_.avg_val = (masked_count > 0) ? static_cast<uint8_t>(sum_v / masked_count) : 0;
}

bool HsvTuner::parseCommand(const char* cmd_str) {
    if (cmd_str == nullptr) return false;

    char key[32] = {};
    int delta = 0;

    if (std::sscanf(cmd_str, "%31s %d", key, &delta) == 2) {
        if (std::strcmp(key, "H_MIN") == 0) {
            int val = static_cast<int>(active_profile_.h_min) + delta;
            active_profile_.h_min = static_cast<uint8_t>(std::max(0, std::min(255, val)));
            return true;
        } else if (std::strcmp(key, "H_MAX") == 0) {
            int val = static_cast<int>(active_profile_.h_max) + delta;
            active_profile_.h_max = static_cast<uint8_t>(std::max(0, std::min(255, val)));
            return true;
        } else if (std::strcmp(key, "S_MIN") == 0) {
            int val = static_cast<int>(active_profile_.s_min) + delta;
            active_profile_.s_min = static_cast<uint8_t>(std::max(0, std::min(255, val)));
            return true;
        } else if (std::strcmp(key, "S_MAX") == 0) {
            int val = static_cast<int>(active_profile_.s_max) + delta;
            active_profile_.s_max = static_cast<uint8_t>(std::max(0, std::min(255, val)));
            return true;
        } else if (std::strcmp(key, "V_MIN") == 0) {
            int val = static_cast<int>(active_profile_.v_min) + delta;
            active_profile_.v_min = static_cast<uint8_t>(std::max(0, std::min(255, val)));
            return true;
        } else if (std::strcmp(key, "V_MAX") == 0) {
            int val = static_cast<int>(active_profile_.v_max) + delta;
            active_profile_.v_max = static_cast<uint8_t>(std::max(0, std::min(255, val)));
            return true;
        }
    } else if (std::sscanf(cmd_str, "%31s", key) == 1) {
        if (std::strcmp(key, "SAVE") == 0) {
            return saveToStorage();
        } else if (std::strcmp(key, "RESET") == 0) {
            reset();
            return true;
        } else if (std::strcmp(key, "PROFILE_GROUND") == 0) {
            setActiveProfile(Types::VisionMarkerType::ON_GROUND_MINE);
            return true;
        } else if (std::strcmp(key, "PROFILE_BURIED") == 0) {
            setActiveProfile(Types::VisionMarkerType::BURIED_SURFACE_MARKER);
            return true;
        }
    }
    return false;
}

bool HsvTuner::saveToStorage() {
    Hal::hal_log("[HSV_TUNER] Saving calibrated HSV thresholds to persistent storage...");
    // Persist as a calibration telemetry record to flash
    Types::TelemetryEvent evt;
    evt.timestamp_ms = Hal::hal_millis();
    evt.event_id = 2200; // TE_VISION_INITIALIZED
    evt.severity = Types::TELEMETRY_SEVERITY_INFO;
    evt.module_id = 4;   // Vision Pipeline
    evt.value_a = static_cast<float>(active_profile_.h_min);
    evt.value_b = static_cast<float>(active_profile_.h_max);
    evt.context_id = (active_profile_.s_min << 8) | active_profile_.v_min;

    Hal::hal_storage_write_event(evt);
    return Hal::hal_storage_flush();
}

bool HsvTuner::loadFromStorage() {
    Hal::hal_log("[HSV_TUNER] Checking persistent storage for calibration profiles...");
    return Hal::hal_storage_is_healthy();
}

void HsvTuner::printStatus() const {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
        "[HSV_TUNER] Prof:%s H:[%d,%d] S:[%d,%d] V:[%d,%d] MaskPx:%lu (%.1f%%) Avg:[H:%d S:%d V:%d]",
        (active_profile_type_ == Types::VisionMarkerType::ON_GROUND_MINE) ? "GROUND" : "BURIED",
        active_profile_.h_min, active_profile_.h_max,
        active_profile_.s_min, active_profile_.s_max,
        active_profile_.v_min, active_profile_.v_max,
        static_cast<unsigned long>(stats_.masked_pixels),
        stats_.masked_ratio * 100.0f,
        stats_.avg_hue, stats_.avg_sat, stats_.avg_val);
    Hal::hal_log(buf);
}

} // namespace RobofestDrone
