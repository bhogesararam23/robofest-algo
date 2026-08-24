# Integration Testing & HIL Protocol

**Covers:** REQ-DER-116 (plan item 16) — end-to-end mission scenarios,
Hardware-in-the-Loop setup, and field validation procedures.
**Related:** `docs/rtm.md`, `docs/safety_checklist.md`, `sim/benchmark_report.py`

---

## 1. Test Pyramid Overview

| Level | Environment | What runs | Gate |
|---|---|---|---|
| L1 Unit | Host g++ (`robofest_unit_tests.exe`) | Pure logic: CV math, fusion, consensus, TTC, profiles | 84+ tests green |
| L2 SITL | Python sim harness | Full firmware control + CV logic against mocked sensors | Scenario KPIs, benchmark budgets |
| L3 HIL | Bench + real FC/compute | Actual ESP32-S3 + Matek H743 against real-time simulator feeds | Boot log, self-test, loop timing |
| L4 Field | Tethered → free flight | Real sensors, real markers | Safety checklist sign-off |

A change may only advance one level when the previous level passes clean.

---

## 2. Level 1 — Host Unit Tests (automated)

```bash
cd robofest_drone
$sources = Get-ChildItem src\*.cpp | Where-Object Name -ne "main.cpp"
g++ -std=c++17 -Wall -Wextra -I./src -I./hal -I./config tests/*.cpp $sources src/calibration/*.cpp hal/*.cpp -o robofest_unit_tests.exe
.\robofest_unit_tests.exe   # exit code != 0 on any failure
```

Suites cover: vision bands/scoring/geometry/confidence, config profile integrity,
marker render table + guidance mapping, path clearance, mine dedup/fusion,
geofence, localization math, command debounce, perception (human/gesture),
swarm fusion + consensus + TTC + landing prediction + buried cores.

---

## 3. Level 2 — SITL Mission Scenarios (automated)

```bash
python sim/benchmark_report.py            # full 12-scenario matrix + report
python sim/benchmark_report.py --quick    # CI-friendly subset
```

### Scenario → requirement coverage

| Scenario | Validates |
|---|---|
| `nominal_map` | Baseline 40-mine mission, scoring reference (F1 ≈ 0.35 known baseline) |
| `dense_map` / `obstacle_map` | A* corridor with 1.0 m clearance under density/obstacles (REQ-OFF-009) |
| `high_false_positive_map` | Dedup + persistence filtering (REQ-OFF-015) |
| `low_visibility_map` / `night_low_light_map` | Buried markers; night-mode threshold relaxation (items 13/15) |
| `drift_stress_map` | Geofence margin scaling, localization watchdogs (REQ-DER-010) |
| `peer_failure_map` / `swarm_fusion_stress_map` | Role failover; VISION_OBS fusion + consensus voting under noise/loss (items 11/12) |
| `human_deviation_map` / `dynamic_obstacle_map` | Re-routing guidance; TTC margins with fast human (items 14, REQ-OFF-019) |

### Acceptance gates per run
1. No crash/hang; wall/sim ratio ≤ 12×.
2. Mapping F1 ≥ scenario baseline − 0.05 (baselines recorded in `sim/benchmark_report.md`).
3. Zero path-clearance violations (`path_clearance_violation_count == 0`).
4. Benchmark report exits 0 (no budget alerts).

---

## 4. Level 3 — Hardware-in-the-Loop Protocol

### 4.1 Bench wiring (propellers OFF)
1. XIAO ESP32-S3 Sense flashed via `pio run -e seeed_xiao_esp32s3 -t upload`.
2. Matek H743 connected over UART @ 921600 (`FcBridge`). FC powered from bench PSU set to 4S voltage (start 16.0 V).
3. Serial monitor at 115200 for telemetry (`monitor_speed`).

### 4.2 Real-time feed substitution
- **Camera**: point the OV5640 at a printed marker board OR replay frames from a laptop screen running `sim/cv_universal.py --image <synthetic>` in slideshow mode. Verify `TE_VISION_PROFILE_TABLE_LOADED`, frame counter advancing, and processing < 20 ms typical (66 ms hard slot).
- **Optical flow/ToF**: slide the drone over a textured mat by hand at ≤ 0.5 m/s; confirm flow velocity and ToF altitude track hand motion within ±0.05 m.
- **Swarm**: second drone (or ESP32 devkit replaying logged packets) broadcasts heartbeats + VISION_OBS; confirm `active_peer_count`, fusion events (`TE_SWARM_VISION_OBS_RECEIVED`), and that ambiguous-marker flags appear when votes split.

### 4.3 Pass criteria
- All 15 modules initialize; zero allocation failures in boot log (`docs/build_and_flash.md` §4 reference log).
- `bench_self_test` green end-to-end.
- Vision slot p99 ≤ 66 ms over ≥ 5 min soak; DRAM headroom ≥ 25 % (see boot banner).

---

## 5. Level 4 — Field Validation Procedure

1. Complete `docs/safety_checklist.md` Stages 1–2 **plus §4 Vision System checklist** before arming.
2. Run scenario `nominal_map` layout physically: place 8–12 markers in a 15 × 20 m subfield (scaled rehearsal of the official 15 × 60 m arena).
3. Data collection (blackbox + operator log):
   - LittleFS blackbox download post-flight.
   - Operator records: ambient light (lx via phone sensor), ground type, wind, per-marker ground-truth GPS-free survey (tape measure grid).
4. Post-flight: diff confirmed-mine list vs ground truth; compute F1/RMSE with `sim/score_eval.py` on exported map; attach to flight log.

**Field regression rule:** any metric worse than the same layout's previous session by > 15 % relative requires a calibration pass (`calibration_mode`) before the next flight.

---

## 6. Continuous Integration Hook

Minimal GitHub Actions / local CI sequence:

```bash
pio run -e seeed_xiao_esp32s3                 # firmware compiles
python scripts/gen_vision_profiles.py --check # generated header valid
python sim/cv_universal.py --selftest         # laptop CV stack green
python sim/benchmark_report.py --quick        # SITL subset + budgets
```

All four must exit 0.
