#pragma once

#include <stdint.h>
#include "types.h"
#include "state_machine_types.h"

namespace RobofestDrone {

class StateMachine {
public:
    StateMachine();

    void init();
    void reset();
    void update(const StateMachineInputs& inputs, uint32_t now_ms);

    Types::DroneState getState() const { return m_outputs.current_state; }
    Types::DroneState getPreviousState() const { return m_outputs.previous_state; }
    Types::DroneState getResumeState() const { return m_outputs.resume_state; }

    uint32_t getStateEnteredTimeMs() const { return m_outputs.state_entered_ms; }
    uint32_t getTimeInStateMs() const { return m_outputs.time_in_state_ms; }

    uint32_t getMissionElapsedMs() const { return m_outputs.mission_elapsed_ms; }
    bool isMissionTimerRunning() const { return m_outputs.mission_timer_running; }
    bool isMissionTimeOver() const { return m_outputs.mission_time_over; }

    bool isStateActive(Types::DroneState state) const { return m_outputs.current_state == state; }
    bool isFlightActive() const;
    bool isMissionActive() const;
    bool isTerminalState() const;

    StateMachineOutputs getOutputs() const { return m_outputs; }
    StateMachineEvent getLastEvent() const { return m_outputs.last_event; }

private:
    void setState(Types::DroneState new_state, uint32_t now_ms, StateMachineEvent event);

    void handleInit(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleCalibrate(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleWaitForStart(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleTakeoff(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleFormation(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleSearching(const StateMachineInputs& inputs, uint32_t now_ms);
    void handlePlanning(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleGuiding(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleMissionComplete(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleLanding(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleDisarmed(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleHold(const StateMachineInputs& inputs, uint32_t now_ms);
    void handleEmergency(const StateMachineInputs& inputs, uint32_t now_ms);

    void updateMissionTimer(const StateMachineInputs& inputs, uint32_t now_ms);
    void applySafetyOverrides(const StateMachineInputs& inputs, uint32_t now_ms);
    void updateOutputFlags();

    bool canEnterTakeoff(const StateMachineInputs& inputs) const;
    bool canEnterSearching(const StateMachineInputs& inputs) const;
    bool canEnterPlanning(const StateMachineInputs& inputs) const;
    bool canEnterGuiding(const StateMachineInputs& inputs) const;
    bool canEnterMissionComplete(const StateMachineInputs& inputs) const;
    bool canEnterLanding(const StateMachineInputs& inputs) const;
    bool shouldEnterHold(const StateMachineInputs& inputs) const;
    bool shouldEnterEmergency(const StateMachineInputs& inputs) const;
    bool shouldReturnFromHold(const StateMachineInputs& inputs) const;

    uint16_t mapStateToTelemetryEvent(Types::DroneState state) const;

private:
    StateMachineOutputs m_outputs;
    uint32_t m_mission_start_time_ms = 0;
    bool m_mission_timer_active = false;
    bool m_hold_due_to_pause = false;
};

// Top-level module facade functions called by scheduler / main loop
void state_machine_init();
void state_machine_update(uint32_t now_ms);
Types::DroneState state_machine_get_state();
StateMachineOutputs state_machine_get_outputs();
StateMachine& state_machine_get_instance();

} // namespace RobofestDrone
