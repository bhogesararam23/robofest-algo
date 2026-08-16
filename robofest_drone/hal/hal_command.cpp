#include "hal_command.h"
#include "hal_system.h"

namespace RobofestDrone {
namespace Hal {

namespace {
    static bool s_command_initialized = false;
    static bool s_gesture_enabled = true;
    static bool s_voice_enabled = false;
    static bool s_has_mock = false;
    static Types::CommandSample s_mock_sample;
}

bool hal_command_init() {
    s_command_initialized = true;
    hal_log("[HAL_COMMAND] Onboard gesture/voice command HAL stub initialized (safe default).");
    return true;
}

bool hal_command_is_healthy() {
    return s_command_initialized;
}

bool hal_command_read_gesture(Types::CommandSample& sample) {
    if (!s_command_initialized || !s_gesture_enabled) {
        sample.valid = false;
        sample.command = Types::CommandType::NONE;
        sample.confidence = 0.0f;
        sample.source = Types::CommandSource::COMMAND_SOURCE_NONE;
        sample.timestamp_ms = hal_millis();
        return false;
    }

    if (s_has_mock) {
        sample = s_mock_sample;
        return sample.valid;
    }

    // Default stub: No active gesture detected
    sample.valid = false;
    sample.command = Types::CommandType::NONE;
    sample.confidence = 0.0f;
    sample.source = Types::CommandSource::COMMAND_SOURCE_GESTURE;
    sample.timestamp_ms = hal_millis();
    return false;
}

void hal_command_set_mock_gesture(const Types::CommandSample& sample) {
    s_mock_sample = sample;
    s_has_mock = true;
}

void hal_command_clear_mock() {
    s_has_mock = false;
}

bool hal_command_read_voice(Types::CommandSample& sample) {
    if (!s_command_initialized || !s_voice_enabled) {
        sample.valid = false;
        sample.command = Types::CommandType::NONE;
        sample.confidence = 0.0f;
        sample.source = Types::CommandSource::COMMAND_SOURCE_NONE;
        sample.timestamp_ms = hal_millis();
        return false;
    }

    // Default stub: No active voice command detected
    sample.valid = false;
    sample.command = Types::CommandType::NONE;
    sample.confidence = 0.0f;
    sample.source = Types::CommandSource::COMMAND_SOURCE_VOICE;
    sample.timestamp_ms = hal_millis();
    return false;
}

void hal_command_enable_gesture(bool enabled) {
    s_gesture_enabled = enabled;
}

void hal_command_enable_voice(bool enabled) {
    s_voice_enabled = enabled;
}

} // namespace Hal
} // namespace RobofestDrone
