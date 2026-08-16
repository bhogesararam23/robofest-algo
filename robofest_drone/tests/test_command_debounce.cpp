#include "test_framework.h"
#include "../src/command_layer.h"
#include "../src/types.h"
#include "../hal/hal_command.h"
#include "../config/thresholds.h"

using namespace RobofestDrone;

TEST(CommandDebounceTest, SingleFrameRejectedByHysteresis) {
    CommandLayer cmd_layer;
    cmd_layer.init();
    cmd_layer.setSystemState(Types::DroneState::WAIT_FOR_START);

    // Feed a single frame of START
    Types::CommandSample sample;
    sample.valid = true;
    sample.command = Types::CommandType::START;
    sample.confidence = 0.95f;
    sample.source = Types::CommandSource::COMMAND_SOURCE_GESTURE;
    sample.timestamp_ms = 1000UL;

    Hal::hal_command_set_mock_gesture(sample);
    cmd_layer.update(1000UL);

    // Hysteresis requirement is >= 3 frames AND >= 300ms debounce -> 1st frame is NOT valid yet
    ASSERT_FALSE(cmd_layer.isCommandValid());
    Hal::hal_command_clear_mock();
}

TEST(CommandDebounceTest, ConsecutiveFramesAccepted) {
    CommandLayer cmd_layer;
    cmd_layer.init();
    cmd_layer.setSystemState(Types::DroneState::WAIT_FOR_START);

    Types::CommandSample sample;
    sample.valid = true;
    sample.command = Types::CommandType::START;
    sample.confidence = 0.95f;
    sample.source = Types::CommandSource::COMMAND_SOURCE_GESTURE;

    // Feed consecutive frames over 350ms (> COMMAND_DEBOUNCE_MS of 300ms)
    for (uint32_t t = 1000UL; t <= 1350UL; t += 20UL) {
        sample.timestamp_ms = t;
        Hal::hal_command_set_mock_gesture(sample);
        cmd_layer.update(t);
    }

    // After required consecutive frames and debounce window, command must be accepted
    ASSERT_TRUE(cmd_layer.isCommandValid());
    ASSERT_EQ(cmd_layer.getLatestCommand(), Types::CommandType::START);

    Hal::hal_command_clear_mock();
}

TEST(CommandDebounceTest, LowConfidenceRejected) {
    CommandLayer cmd_layer;
    cmd_layer.init();
    cmd_layer.setSystemState(Types::DroneState::SEARCHING);

    Types::CommandSample sample;
    sample.valid = true;
    sample.command = Types::CommandType::STOP_ABORT;
    sample.confidence = 0.20f; // Below STOP_CONFIDENCE_MIN (0.90)
    sample.source = Types::CommandSource::COMMAND_SOURCE_GESTURE;

    for (uint32_t t = 1000UL; t <= 1400UL; t += 20UL) {
        sample.timestamp_ms = t;
        Hal::hal_command_set_mock_gesture(sample);
        cmd_layer.update(t);
    }

    // Low confidence command should be rejected
    ASSERT_FALSE(cmd_layer.isCommandValid());
    Hal::hal_command_clear_mock();
}

TEST(CommandDebounceTest, HighConfidenceEmergencyAccepted) {
    CommandLayer cmd_layer;
    cmd_layer.init();
    cmd_layer.setSystemState(Types::DroneState::SEARCHING);

    Types::CommandSample sample;
    sample.valid = true;
    sample.command = Types::CommandType::STOP_ABORT;
    sample.confidence = 0.95f; // High confidence (>= 0.90)
    sample.source = Types::CommandSource::COMMAND_SOURCE_GESTURE;

    for (uint32_t t = 1000UL; t <= 1350UL; t += 20UL) {
        sample.timestamp_ms = t;
        Hal::hal_command_set_mock_gesture(sample);
        cmd_layer.update(t);
    }

    ASSERT_TRUE(cmd_layer.isCommandValid());
    ASSERT_EQ(cmd_layer.getLatestCommand(), Types::CommandType::STOP_ABORT);

    Hal::hal_command_clear_mock();
}
