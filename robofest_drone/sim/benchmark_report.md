# Performance Benchmark Report

_Generated 2026-08-25 01:27:15 - 3 scenarios_

| Scenario | OK | Wall s | F1 | Recall | RMSE m | Safe path | Alerts |
|---|---|---|---|---|---|---|---|
| nominal_map | yes | 1.05 | 0.349 | 0.475 | 0.17 | no | - |
| night_low_light_map | yes | 2.73 | 0.347 | 0.625 | 0.16 | no | - |
| swarm_fusion_stress_map | yes | 1.6 | 0.348 | 0.509 | 0.19 | no | - |

## Hard budgets

- Vision slot: <= 66.0 ms (enforced onboard via TE_VISION_PROCESSING_SLOW; see bench self-test stream)
- Main loop: <= 20.0 ms (scheduler task overruns surface as watchdog telemetry)
- Sim wall/sim ratio: <= 12.0x

**Total alerts: 0**
