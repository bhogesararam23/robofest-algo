# Requirements Traceability Matrix (RTM) — Robofest Gujarat 6.0

> **Rule** (REQ item 23): every requirement carries an ID; every ID maps to at least one
> implementing module and at least one verifying test. This matrix is updated at the end of
> every development sprint/phase. `PLANNED(phase N)` entries must be resolved before sign-off.

## 1. Official Mission Requirements → Code → Tests

| Req ID | Requirement (short) | Implementing Module(s) | Verifying Test(s) | Status |
|---|---|---|---|---|
| REQ-OFF-001 | Swarm ≥ 3 drones | `swarm_comm`, `mission_config` | SITL multi-drone scenarios | DONE |
| REQ-OFF-002 | Ideal 3–4 drones | `mission_config.MAX_SWARM_DRONES` | SITL fleet sizing | DONE |
| REQ-OFF-003/004 | 600 s limit + timeout landing | `state_machine`, `safety_manager` | SITL timeout scenario | DONE |
| REQ-OFF-005..008 | Field geometry & zones | `config/mission_config.h`, `geofence` | `GeofenceTest.*` | DONE |
| REQ-OFF-009 | ≥ 1.0 m mine path clearance | `path_planner` | `PathClearanceTest.*` | DONE |
| REQ-OFF-010..013 | No GPS/ext/cloud/pilot | architecture (no such deps exist) | code review checklist | DONE |
| REQ-OFF-014 | Onboard mine detection | `vision_pipeline`, `hal_camera` | `Vision*Test.*`, bench self-test | DONE |
| REQ-OFF-015 | Mapping + dedup | `mine_map` | `MineDedupTest.*` | DONE |
| REQ-OFF-016 | Safe path generation | `path_planner` | `PathClearanceTest.ComputePathAvoidsMines*` | DONE |
| REQ-OFF-017 | Visual path guidance | `marker_controller`, `hal_marker` | bench pattern test; Phase 2 driver tests | PARTIAL |
| REQ-OFF-018 | Human command understanding | `hal_command`, `command_layer` | `CommandDebounceTest.*`; Phase 4 pipelines | PARTIAL |
| REQ-OFF-019 | Human tracking + re-route | `human_tracker`, `search_behavior` | Phase 5/6 scenarios | PARTIAL |
| REQ-OFF-020 | Autonomous landing | `state_machine`, `fc_bridge` | SITL end-of-mission | DONE |
| REQ-OFF-021 | Surface contact restriction | `safety_manager`, geofence | SITL altitude floors | DONE |
| REQ-OFF-022 | Collision/proximity avoidance | `safety_manager`, `swarm_comm` | Phase 5 TTC tests | PARTIAL |

## 2. Derived Engineering Requirements → Code → Tests

| Req ID | Requirement (short) | Implementing Module(s) | Verifying Test(s) | Status |
|---|---|---|---|---|
| REQ-DER-001/002 | ~40 mines, 75/25 split | `sim/map_generator.py` | scenario fixtures | DONE |
| REQ-DER-003 | Static obstacle costmap | `path_planner` | Phase 6 obstacle scenario | PARTIAL |
| REQ-DER-004/005 | FC separation of concerns | `fc_bridge` | bench self-test | DONE |
| REQ-DER-006 | 50 Hz deterministic loop | `scheduler` | Phase 6 benchmarks | PARTIAL |
| REQ-DER-007..010 | Kill switch / battery / watchdogs | `safety_manager` | bench checklist §2 items | DONE |
| REQ-DER-011 | Inter-drone separation | `swarm_comm`, `safety_manager` | SITL swarm proximity | PARTIAL |

## 3. Feature Enhancement Requirements (24-item plan, REQ-DER-100 series)

| Req ID | Feature (plan item) | Implementing Module(s) | Verifying Test(s) | Status |
|---|---|---|---|---|
| REQ-DER-101 | Real camera HAL, unified frame struct (1) | `hal/hal_camera.cpp` esp32-camera backend | host stub tests; bench self-test; Phase 6 | PLANNED(Phase 1) |
| REQ-DER-102 | Human detection HAL + model (2) | `hal/hal_human.cpp`, `lib/robofest_vision_ml` | `test_human_detector.cpp` (Phase 6) | PLANNED(Phase 4) |
| REQ-DER-103 | Gesture + voice recognition (3) | `hal/hal_command.cpp`, `command_layer` | `CommandDebounceTest.*`, new KWS tests | PLANNED(Phase 4) |
| REQ-DER-104 | LED/laser visual output driver (4) | `hal/hal_marker.cpp` RMT WS2812 | pattern unit tests (Phase 2) | PLANNED(Phase 2) |
| REQ-DER-105 | Resolution/format adaptation (5) | `src/frame_adapter.{h,cpp}` | `test_frame_adapter.cpp` | PLANNED(Phase 3) |
| REQ-DER-106 | Internal calibration + fallback profiles (6) | `scripts/gen_vision_profiles.py`, tuner tools | `config_profiles.*` tests | PARTIAL (Phase 3 completes persistence) |
| REQ-DER-107 | Concave shape detection (7) | `vision_pipeline` contour tracer | `test_shape_analysis.cpp` | PLANNED(Phase 3) |
| REQ-DER-108 | Advanced lighting preprocessing (8) | `vision_pipeline` CLAHE/AWB/shadow | `test_lighting_pipeline.cpp` | PLANNED(Phase 3) |
| REQ-DER-109 | Morphology performance (9) | `vision_pipeline` separable ops | timing budget test | PLANNED(Phase 3) |
| REQ-DER-110 | OCR / QR / barcode reading (10) | `lib/quirc`, `src/code_reader` | `test_code_reader.cpp` | PLANNED(Phase 4) |
| REQ-DER-111 | Cross-drone vision fusion (11) | `swarm_comm`, `mine_map` | `test_vision_fusion.cpp` | PLANNED(Phase 5) |
| REQ-DER-112 | Marker classification consensus (12) | `swarm_comm`, `mine_map` | `test_marker_consensus.cpp` | PLANNED(Phase 5) |
| REQ-DER-113 | Buried-mine visual detection (13) | `src/buried_detector.{h,cpp}` | `test_buried_detector.cpp` | PLANNED(Phase 5) |
| REQ-DER-114 | Dynamic obstacle TTC avoidance (14) | `safety_manager`, human detector | `test_obstacle_ttc.cpp` | PLANNED(Phase 5) |
| REQ-DER-115 | Night / low-light operation (15) | `hal_camera` exposure + pipeline night mode | night SITL scenario | PLANNED(Phase 3) |
| REQ-DER-116 | Integration/HIL testing (16) | `sim/scenarios.py`, docs | benchmark runs | PLANNED(Phase 6) |
| REQ-DER-117 | CV unit-test coverage (17) | `tests/*` | full suite green | IN PROGRESS |
| REQ-DER-118 | Performance benchmarks (18) | stage timers, `sim/benchmark_report.py` | report artifacts | PLANNED(Phase 6) |
| REQ-DER-119 | Generated vision profiles present in build (19) | `scripts/gen_vision_profiles.py`, `pio_extra_script.py` | `tests/test_config_profiles.cpp`, `--check` CI | **DONE** |
| REQ-DER-120 | Camera intrinsics calibration (20) | `sim/calibrate_camera.py`, undistort LUT | reprojection residual check | PLANNED(Phase 7) |
| REQ-DER-121 | Vision pre-flight checklist (21) | `docs/safety_checklist.md` §4-V | operator sign-off flow | **DONE** |
| REQ-DER-122 | Formal PRD (22) | `docs/PRD.md` | review sign-off | PLANNED(Phase 7) |
| REQ-DER-123 | Traceability maintenance (23) | `docs/rtm.md` (this file) | sprint review | **ONGOING** |
| REQ-DER-124 | Sim-vs-reality gap analysis (24) | `docs/sim_vs_reality.md` | field delta tables | PLANNED(Phase 7) |

## 4. Maintenance Protocol

1. No requirement is "done" until its row lists a passing test or a signed checklist.
2. New requirements enter as `REQ-DER-<next free number>` with owner + phase tags.
3. During each phase closeout: update statuses here in the same commit as the code.
4. Annual/event review: delete rows only via PRD change control (REQ-DER-122).
