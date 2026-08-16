# Robofest Gujarat 6.0 Minefield Swarm Drone — SITL Simulation & Scoring Toolset

---

## 1. Executive Summary & Purpose

This directory contains an **optional, external Software-In-The-Loop (SITL) simulation and scoring harness** designed to validate multi-agent swarm coordination, optical flow drift estimation, vision perception noise, 1.0 meter mine clearance A* path planning, and human guidance behavior before deploying to real physical aircraft.

### Critical Operational Tenets:
- **External Validation Toolset**: This simulation runs on a host development laptop. It is **not onboard flight software** and has zero footprint in flight binaries.
- **Strict Ground Truth Isolation**: Simulated drones **never receive GPS or true global coordinates**. Drones receive only noisy sensor observations (dead-reckoning optical flow velocities, ToF rangefinder distances, noisy vision bounding boxes, and lossy radio packets).
- **100% Local Execution**: Zero cloud dependencies, zero external network APIs, zero internet connection required.
- **Deterministic & Repeatable**: Given a fixed `--seed`, runs produce exact repeatable trajectories for algorithmic benchmarking.

---

## 2. File & Module Structure

```text
sim/
├── README.md               # User guide and command reference
├── config.py               # Central repository of all tunable simulation constants
├── map_generator.py        # Synthetic 15m x 60m field & minefield generator
├── sensor_sim.py           # Optical flow drift, ToF noise, and vision candidate generator
├── drone_model.py          # High-level drone autonomy, dead-reckoning, and mapping model
├── swarm_model.py          # Distributed P2P packet exchange, role failover, and claim/yield
├── human_model.py          # Person-at-risk pedestrian walking model and deviation generator
├── scenarios.py            # Preset mission environments (easy, nominal, dense, drift stress, etc.)
├── sitl_harness.py         # Main closed-loop simulation loop and event dispatcher
├── metrics.py              # Precision/recall, 1.0m path clearance, and safety metric evaluator
└── score_eval.py           # Robofest category scoring and report generator
```

---

## 3. How to Run Simulations

### 3.1 Standard Nominal Mission Run
```bash
python sim/sitl_harness.py --seed 1 --scenario nominal_map --drones 3 --mines 40 --duration 600 --output run_nominal.json --report run_nominal.md
```

### 3.2 High-Density Minefield Stress Test
```bash
python sim/sitl_harness.py --seed 42 --scenario dense_map --drones 4 --mines 60 --duration 600 --output run_dense.json --report run_dense.md
```

### 3.3 Optical Flow Drift Stress Test
```bash
python sim/sitl_harness.py --seed 100 --scenario drift_stress_map --drones 3 --duration 600 --output run_drift.json --report run_drift.md
```

### 3.4 Standalone Score Evaluation on Existing Run JSON
```bash
python sim/score_eval.py run_nominal.json --report run_nominal_score.md
```

---

## 4. Scenario Presets Reference

| Preset Name | Mine Count | Buried % | Noise Multipliers | Description |
| :--- | :---: | :---: | :--- | :--- |
| `easy_map` | 20 | 15% | Low noise ($0.8\times$ drift) | High-contrast markers, ideal lighting benchmark |
| `nominal_map` | 40 | 25% | Standard ($1.0\times$) | Standard competition baseline configuration |
| `dense_map` | 60 | 25% | Standard ($1.0\times$) | High density testing narrow corridor clearance |
| `obstacle_map` | 40 | 25% | Standard + 4 obstacles | Stanchions/trees embedded into costmap |
| `high_false_positive_map`| 40 | 25% | $2.5\times$ FP multiplier | High surface glare testing deduplication & decay |
| `low_visibility_map` | 40 | 50% | $0.75\times$ TP multiplier | Dim arena lighting with more buried markers |
| `drift_stress_map` | 40 | 25% | $2.5\times$ drift multiplier | High random walk dead-reckoning noise |
| `peer_failure_map` | 40 | 25% | Drone 2 lost @ 60s | Evaluates dynamic lane expansion & role failover |
| `human_deviation_map` | 40 | 25% | Deviating human model | Person leaves safe path, testing dynamic re-route |

---

## 5. Scoring Categories & Interpretation

| Category | Max Weight | Evaluation Criteria |
| :--- | :---: | :--- |
| **Takeoff & Activation** | 10.0 pts | Autonomous start command debouncing and climb to 2.0m |
| **Command Recognition** | 10.0 pts | Robust gesture/voice command gating without false triggers |
| **Swarm Coordination** | 10.0 pts | Rapid lane formation and role failover upon peer loss |
| **Mine Detection & Mapping**| 25.0 pts | F1-Score of discovered confirmed mines vs ground truth |
| **Safe Path (1.0m Clearance)**| 20.0 pts | Zero segment violations $< 1.0\text{ m}$ to any true mine |
| **Safe Human Crossing** | 20.0 pts | Person safely guided from Start Zone to Exit Zone |
| **Time Bonus** | 5.0 pts | Crossing completed under 300s / 450s |
| **Penalties** | Deductive | $-10\text{ pts}$ per collision/proximity, $-20\text{ pts}$ per crash/surface contact |

---

## 6. How Simulation Informs Firmware Tuning

- **If 1.0m Path Clearance Violations Occur**:
  Increase `PATH_GRID_RESOLUTION_M` or increase mine inflation radius in `config/thresholds.h`.
- **If False Positive Mine Count is High**:
  Increase `MINE_CONFIRM_PERSISTENCE_MIN` from 3 to 5 in `config/thresholds.h` to suppress transient glare.
- **If Buried Marker Recall is Low**:
  Tune HSV thresholds in `src/calibration/hsv_tuner.cpp` or widen `Config::BURIED_SURFACE_MARKER_HSV_LOW/HIGH`.
- **If Drift Uncertainty Exceeds Geofence Limits**:
  Increase `GEOFENCE_UNCERTAINTY_EXTRA_MARGIN_M` in `config/mission_config.h`.
