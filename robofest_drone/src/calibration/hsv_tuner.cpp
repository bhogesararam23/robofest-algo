#include "hsv_tuner.h"
#include "../../hal/hal_system.h"
#include "../../hal/hal_storage.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace RobofestDrone {

namespace {
    // Telemetry event block used by onboard calibration saves (vision range
    // 2200-2299; profile saves occupy 2250..2250+VISION_PROFILE_MAX-1).
    constexpr uint16_t TE_HSV_TUNER_PROFILE_SAVE_BASE = 2250;
}

HsvTuner::HsvTuner() {
    init();
}

void HsvTuner::init() {
    active_index_ = 0;
    edit_alt_ = false;
    stats_ = HsvTunerStats();
}

void HsvTuner::reset() {
    init();
}

static VisionPipeline& tuner_pipeline() {
    return vision_pipeline_get_instance();
}

bool HsvTuner::setActiveProfileByIndex(uint8_t index) {
    if (index >= tuner_pipeline().getProfileCount()) return false;
    active_index_ = index;
    return true;
}

bool HsvTuner::setActiveProfileByType(Types::VisionMarkerType type) {
    for (uint8_t i = 0; i < tuner_pipeline().getProfileCount(); ++i) {
        const VisionMarkerProfile* p = tuner_pipeline().getProfileByIndex(i);
        if (p != nullptr && p->profile_type == type) {
            active_index_ = i;
            return true;
        }
    }
    return false;
}

void HsvTuner::cycleProfile() {
    uint8_t count = tuner_pipeline().getProfileCount();
    if (count == 0) return;
    active_index_ = static_cast<uint8_t>((active_index_ + 1) % count);
}

Types::VisionMarkerType HsvTuner::getActiveProfileType() const {
    const VisionMarkerProfile* p = getActiveProfile();
    return (p != nullptr) ? p->profile_type : Types::VisionMarkerType::UNKNOWN;
}

VisionMarkerProfile* HsvTuner::getActiveProfile() {
    return tuner_pipeline().getProfileByIndex(active_index_);
}

const VisionMarkerProfile* HsvTuner::getActiveProfile() const {
    return tuner_pipeline().getProfileByIndex(active_index_);
}

// Hue scale matches the vision pipeline exactly: H in [0,180), S/V in [0,255].
void HsvTuner::rgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint8_t& h, uint8_t& s, uint8_t& v) const {
    uint8_t c_max = std::max(r, std::max(g, b));
    uint8_t c_min = std::min(r, std::min(g, b));
    uint8_t delta = c_max - c_min;

    v = c_max;
    s = (c_max == 0) ? 0 : static_cast<uint8_t>((255UL * delta) / c_max);
    h = 0;

    if (delta > 0) {
        int16_t h_calc = 0;
        if (c_max == r) {
            h_calc = 30 * (static_cast<int16_t>(g) - static_cast<int16_t>(b)) / delta;
        } else if (c_max == g) {
            h_calc = 60 + 30 * (static_cast<int16_t>(b) - static_cast<int16_t>(r)) / delta;
        } else {
            h_calc = 120 + 30 * (static_cast<int16_t>(r) - static_cast<int16_t>(g)) / delta;
        }
        if (h_calc < 0) h_calc += 180;
        h = static_cast<uint8_t>(h_calc);
    }
}

void HsvTuner::processFrame(const uint8_t* rgb_frame, uint16_t width, uint16_t height) {
    if (rgb_frame == nullptr || width == 0 || height == 0) return;

    const VisionMarkerProfile* p = getActiveProfile();
    if (p == nullptr) return;

    // Band selection mirrors the pipeline's adaptive-lighting rule so the
    // statistics describe exactly the band that will run in production.
    bool use_alt = edit_alt_ ||
        (tuner_pipeline().getLightingMode() == VisionLightingMode::OVERCAST_ALT && p->has_alt_band);

    uint8_t bh_min = use_alt ? p->alt_h_min : p->h_min;
    uint8_t bh_max = use_alt ? p->alt_h_max : p->h_max;
    uint8_t bs_min = use_alt ? p->alt_s_min : p->s_min;
    uint8_t bs_max = use_alt ? p->alt_s_max : p->s_max;
    uint8_t bv_min = use_alt ? p->alt_v_min : p->v_min;
    uint8_t bv_max = use_alt ? p->alt_v_max : p->v_max;

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

        if (vision_hsv_in_band(h, s, v, bh_min, bh_max, bs_min, bs_max, bv_min, bv_max)) {
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

    VisionMarkerProfile* p = getActiveProfile();
    if (p != nullptr) {
        uint8_t* hmin = edit_alt_ ? &p->alt_h_min : &p->h_min;
        uint8_t* hmax = edit_alt_ ? &p->alt_h_max : &p->h_max;
        uint8_t* smin = edit_alt_ ? &p->alt_s_min : &p->s_min;
        uint8_t* smax = edit_alt_ ? &p->alt_s_max : &p->s_max;
        uint8_t* vmin = edit_alt_ ? &p->alt_v_min : &p->v_min;
        uint8_t* vmax = edit_alt_ ? &p->alt_v_max : &p->v_max;

        if (std::sscanf(cmd_str, "%31s %d", key, &delta) == 2) {
            struct UintField { const char* name; uint8_t* ptr; };
            UintField fields[] = {
                {"H_MIN", hmin}, {"H_MAX", hmax},
                {"S_MIN", smin}, {"S_MAX", smax},
                {"V_MIN", vmin}, {"V_MAX", vmax},
            };
            for (auto& f : fields) {
                if (std::strcmp(key, f.name) == 0) {
                    int val = static_cast<int>(*f.ptr) + delta;
                    *f.ptr = static_cast<uint8_t>(std::max(0, std::min(255, val)));
                    return true;
                }
            }

            struct ShapeField { const char* name; float* ptr; float scale; };
            ShapeField shape_fields[] = {
                {"EXT_MIN", &p->extent_min, 0.01f},
                {"EXT_MAX", &p->extent_max, 0.01f},
                {"SOL_MIN", &p->solidity_min, 0.01f},
                {"ASP_MIN", &p->aspect_min, 0.01f},
                {"ASP_MAX", &p->aspect_max, 0.01f},
            };
            for (auto& f : shape_fields) {
                if (std::strcmp(key, f.name) == 0) {
                    *f.ptr += delta * f.scale;
                    if (*f.ptr < 0.0f) *f.ptr = 0.0f;
                    return true;
                }
            }

            if (std::strcmp(key, "CORN_MIN") == 0) {
                int val = static_cast<int>(p->corners_min) + delta;
                p->corners_min = static_cast<uint8_t>(std::max(0, std::min(255, val)));
                return true;
            }
            if (std::strcmp(key, "CORN_MAX") == 0) {
                int val = static_cast<int>(p->corners_max) + delta;
                p->corners_max = static_cast<uint8_t>(std::max(0, std::min(255, val)));
                return true;
            }

            if (std::strcmp(key, "AREA_MIN") == 0) {
                int val = static_cast<int>(p->min_area_px) + delta;
                p->min_area_px = std::max(0.0f, static_cast<float>(val));
                return true;
            }
            if (std::strcmp(key, "AREA_MAX") == 0) {
                int val = static_cast<int>(p->max_area_px) + delta;
                p->max_area_px = std::max(0.0f, static_cast<float>(val));
                return true;
            }
        }
    }

    if (std::sscanf(cmd_str, "%31s", key) == 1) {
        if (std::strcmp(key, "SAVE") == 0) {
            return saveToStorage();
        } else if (std::strcmp(key, "RESET") == 0) {
            tuner_pipeline().restoreProfileDefaults();
            reset();
            return true;
        } else if (std::strcmp(key, "PROFILE_GROUND") == 0) {
            return setActiveProfileByType(Types::VisionMarkerType::ON_GROUND_MINE);
        } else if (std::strcmp(key, "PROFILE_BURIED") == 0) {
            return setActiveProfileByType(Types::VisionMarkerType::BURIED_SURFACE_MARKER);
        } else if (std::strcmp(key, "PROFILE_NEXT") == 0) {
            cycleProfile();
            return true;
        } else if (std::strcmp(key, "BAND_ALT") == 0) {
            edit_alt_ = true;
            return true;
        } else if (std::strcmp(key, "BAND_PRIMARY") == 0) {
            edit_alt_ = false;
            return true;
        } else if (std::strcmp(key, "ENABLE_ALL") == 0) {
            tuner_pipeline().setAllProfilesEnabled(true);
            return true;
        } else if (std::strncmp(key, "PROFILE_N", 9) == 0) {
            // "PROFILE_N <index>"
            char dummy[32] = {};
            int idx = -1;
            if (std::sscanf(cmd_str, "%31s %d", dummy, &idx) == 2 && idx >= 0) {
                return setActiveProfileByIndex(static_cast<uint8_t>(idx));
            }
            return false;
        }
    }
    return false;
}

bool HsvTuner::saveToStorage() {
    Hal::hal_log("[HSV_TUNER] Saving calibrated profiles to persistent storage...");
    VisionPipeline& vp = tuner_pipeline();

    bool ok = true;
    for (uint8_t i = 0; i < vp.getProfileCount(); ++i) {
        const VisionMarkerProfile* prof = vp.getProfileByIndex(i);
        if (prof == nullptr) continue;

        Types::TelemetryEvent evt;
        evt.timestamp_ms = Hal::hal_millis();
        evt.event_id = static_cast<uint16_t>(TE_HSV_TUNER_PROFILE_SAVE_BASE + i);
        evt.severity = Types::TELEMETRY_SEVERITY_INFO;
        evt.module_id = Types::TELEMETRY_MODULE_VISION;
        // Pack primary band bounds as exactly-representable integers
        evt.value_a = static_cast<float>(
            prof->h_min | (prof->h_max << 8) | (prof->s_min << 16));
        evt.value_b = static_cast<float>(
            prof->v_min | (prof->v_max << 8));
        evt.context_id = static_cast<uint16_t>(
            i | (static_cast<uint8_t>(prof->profile_type) << 8));

        Hal::hal_storage_write_event(evt);
    }
    ok = Hal::hal_storage_flush();
    return ok;
}

bool HsvTuner::loadFromStorage() {
    Hal::hal_log("[HSV_TUNER] Checking persistent storage for calibration profiles...");
    return Hal::hal_storage_is_healthy();
}

void HsvTuner::printStatus() const {
    const VisionMarkerProfile* p = getActiveProfile();
    if (p == nullptr) {
        Hal::hal_log("[HSV_TUNER] No active profile.");
        return;
    }

    char buf[224];
    std::snprintf(buf, sizeof(buf),
        "[HSV_TUNER] Idx:%u Type:%u Band:%s H:[%d,%d] S:[%d,%d] V:[%d,%d] "
        "Area:[%.0f,%.0f] Circ:%.2f Ext:[%.2f,%.2f] Sol:%.2f Corn:[%u,%u] "
        "MaskPx:%lu (%.1f%%)",
        static_cast<unsigned>(active_index_),
        static_cast<unsigned>(static_cast<uint8_t>(p->profile_type)),
        edit_alt_ ? "ALT" : "PRIMARY",
        edit_alt_ ? p->alt_h_min : p->h_min,
        edit_alt_ ? p->alt_h_max : p->h_max,
        edit_alt_ ? p->alt_s_min : p->s_min,
        edit_alt_ ? p->alt_s_max : p->s_max,
        edit_alt_ ? p->alt_v_min : p->v_min,
        edit_alt_ ? p->alt_v_max : p->v_max,
        p->min_area_px, p->max_area_px, p->circularity_min,
        p->extent_min, p->extent_max, p->solidity_min,
        static_cast<unsigned>(p->corners_min),
        static_cast<unsigned>(p->corners_max),
        static_cast<unsigned long>(stats_.masked_pixels),
        stats_.masked_ratio * 100.0f);
    Hal::hal_log(buf);
}

} // namespace RobofestDrone

