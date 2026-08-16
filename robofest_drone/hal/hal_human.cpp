#include "hal_human.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_human_initialized = false;
    static Types::HumanDetectionSample s_mock_detection;
    static bool s_mock_available = false;
}

bool hal_human_init() {
    // Note: Replace with real ESP32-S3 camera / ESP-WHO person detection model inference.
    s_human_initialized = true;
    s_mock_available = false;
    hal_log("[HAL_HUMAN] Onboard human detection HAL stub initialized (safe default).");
    return true;
}

bool hal_human_is_healthy() {
    return s_human_initialized;
}

bool hal_human_read_detection(Types::HumanDetectionSample& out_sample) {
    if (!s_human_initialized) {
        out_sample = Types::HumanDetectionSample();
        return false;
    }

    if (s_mock_available) {
        out_sample = s_mock_detection;
        s_mock_available = false;
        return true;
    }

    // Default stub: No detection in passive loop
    out_sample = Types::HumanDetectionSample();
    return false;
}

void hal_human_set_mock_detection(const Types::HumanDetectionSample& sample) {
    s_mock_detection = sample;
    s_mock_available = true;
}

} // namespace Hal
} // namespace RobofestDrone
