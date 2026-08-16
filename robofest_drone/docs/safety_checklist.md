# Pre-Flight Safety Checklist & Field Procedures

---

## 1. Safety Directives & Warnings

> [!CAUTION]
> **MANDATORY SAFETY RULE**: Always remove all propellers during bench testing, software flashing, and initial sensor bring-up. Never power motors with propellers attached while connected to a programming computer.

> [!WARNING]
> The optical flow dead-reckoning pipeline accumulates drift over time. Field testing must always maintain a designated safety pilot ready to trigger the physical hardware kill switch at any moment.

---

## 2. Stage 1: Propeller-Off Bench Testing

Before attaching propellers to any drone in the swarm, complete the following verification steps:

- [ ] **1.1 Firmware Flash & Boot**: ESP32-S3 boots and serial monitor confirms all 15 modules initialized with zero memory allocation errors.
- [ ] **1.2 Hardware Kill Switch Test**: Flip physical kill switch switch. Confirm `TE_KILL_SWITCH_ACTIVE (1028)` is logged instantly and `SafetyAction::EMERGENCY_CUT` is dispatched.
- [ ] **1.3 Serial Bridge Telemetry**: Tilt drone manually by $\pm 30^\circ$ in pitch and roll. Confirm attitude feedback in `FcBridge` reflects true physical orientation.
- [ ] **1.4 Optical Flow & ToF Reading**: Lift drone by $0.5\text{ m}$. Confirm Time-of-Flight rangefinder reports correct altitude within $\pm 0.03\text{ m}$.
- [ ] **1.5 Swarm Radio Broadcast**: Power up at least 2 companion swarm drones. Confirm heartbeat packets are received and active peer count equals total online drones.
- [ ] **1.6 Command Layer Recognition**: Trigger hand gesture or voice commands. Confirm debounced commands transition only through permitted state gates.

---

## 3. Stage 2: Pre-Arming Ground Checklist

Perform this checklist on the competition field before initiating takeoff:

- [ ] **2.1 Battery Voltage Verification**: Confirm 4S LiPo battery is fully charged ($\ge 16.6\text{ V}$, minimum $14.8\text{ V}$). Confirm telemetry voltage readout matches physical multimeter.
- [ ] **2.2 Start Zone Alignment**: Place Drone 1 (Scout Left), Drone 2 (Scout Right), and Drone 3 (Guide) on the designated Start Zone $(Y \in [0.0, 1.0]\text{ m})$ facing longitudinal $(+Y)$ toward the minefield.
- [ ] **2.3 Geofence Confirmation**: Verify `SOFTWARE_GEOFENCE_X_MIN = 0.5f`, `SOFTWARE_GEOFENCE_X_MAX = 14.5f`, `SOFTWARE_GEOFENCE_Y_MIN = 0.5f`, `SOFTWARE_GEOFENCE_Y_MAX = 59.5f`.
- [ ] **2.4 Clean Camera Lens**: Ensure downward camera optics and optical flow lens are clean of dust and debris.
- [ ] **2.5 Self-Check & Calibration State**: Confirm the status LED / marker indicates `WAIT_FOR_START` (amber pulse or ready pattern).

---

## 4. Stage 3: Tethered & Low-Altitude Flight Testing

- [ ] **3.1 Takeoff Command**: Issue `START` gesture/voice command. Verify smooth autonomous ascent to $2.0\text{ m}$ AGL (`MISSION_ALTITUDE_M`).
- [ ] **3.2 Station Keeping & Position Hold**: Verify drone holds horizontal position within $\pm 0.20\text{ m}$ without oscillations.
- [ ] **3.3 Virtual Wall Geofence Bounce**: Allow drone to approach simulated $0.5\text{ m}$ margin. Verify proactive smooth velocity repulsion toward field center.
- [ ] **3.4 Mine Detection & Clearance**: Place test mine marker on ground. Confirm drone detects blob, emits candidate, fuses into `MineMap`, and plans A* corridor maintaining $\ge 1.0\text{ m}$ radial exclusion.
- [ ] **3.5 Autonomous Landing Test**: Send `STOP_ABORT` or allow mission completion. Verify controlled descent at $0.20\text{ m/s}$ step, motor disarm upon touchdown, and telemetry flash sync before power down.

---

## 5. Emergency Response Protocols

| Scenario | Autonomous Reaction | Manual Safety Action Required |
| :--- | :--- | :--- |
| **Battery $< 13.6\text{ V}$** | Safety manager forces `SafetyAction::LAND` immediately. | Clear landing footprint below drone. |
| **Drift $> 2.0\text{ m}$** | Enters `HOLD`, requests swarm peer reposition or land. | Be prepared to command landing if near obstacle. |
| **Swarm Peer Loss** | Autonomous role failover reassigns orphaned lane. | Verify other drones continue search without collision. |
| **Imminent Collision** | Instant motor cutoff via `EMERGENCY_CUT`. | **FLIP HARDWARE KILL SWITCH IMMEDIATELY.** |
| **Mission Time $\ge 600\text{ s}$** | Hard timeout forces immediate autonomous landing. | Confirm all swarm members touch down safely. |
