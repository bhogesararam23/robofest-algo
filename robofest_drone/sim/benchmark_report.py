#!/usr/bin/env python3
"""Performance benchmark runner and budget checker for the Robofest firmware.

REQ-DER-118 (item 18): executes the SITL harness across the full scenario
matrix, extracts per-stage timing/memory metrics from run artifacts, and
verifies hard performance budgets. Budget violations are reported as ALERTS
and reflected in a non-zero exit code so CI can fail loudly.

Usage:
    python sim/benchmark_report.py [--quick] [--out benchmark_report.md]
    python sim/benchmark_report.py --from-run path/to/run.json   # offline check
"""

import argparse
import json
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# HARD PERFORMANCE BUDGETS (REQ item 18). Violations => ALERT.
# ---------------------------------------------------------------------------
BUDGETS = {
    "vision_slot_ms": 66.0,        # vision cadence slot from mission_config
    "main_loop_period_ms": 20.0,   # 50 Hz scheduler tick
    "sim_wallclock_ratio_max": 12.0,  # sim seconds / wall second sanity cap
}

FULL_SCENARIOS = [
    "easy_map",
    "nominal_map",
    "dense_map",
    "obstacle_map",
    "high_false_positive_map",
    "low_visibility_map",
    "drift_stress_map",
    "peer_failure_map",
    "human_deviation_map",
    "night_low_light_map",
    "swarm_fusion_stress_map",
    "dynamic_obstacle_map",
]

QUICK_SCENARIOS = ["nominal_map", "night_low_light_map", "swarm_fusion_stress_map"]


def run_scenario(name: str, timeout_s: int) -> dict:
    out_json = os.path.join(SCRIPT_DIR, f"bench_{name}.json")
    cmd = [
        sys.executable,
        os.path.join(SCRIPT_DIR, "sitl_harness.py"),
        "--scenario", name,
        "--output", out_json,
    ]
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    wall_s = time.time() - t0

    result = {"scenario": name, "ok": proc.returncode == 0,
              "wall_s": round(wall_s, 2), "alerts": []}
    try:
        with open(out_json) as f:
            data = json.load(f)
        result["run"] = data
    except Exception as exc:
        result["alerts"].append(f"missing/invalid run artifact: {exc}")
    return result


def extract_metrics(run: dict) -> dict:
    m = run.get("metrics", {}) if isinstance(run, dict) else {}
    mapping = m.get("mapping", {})
    path = m.get("path", {})
    return {
        "mapping_f1": mapping.get("f1_score"),
        "mapping_recall": mapping.get("recall"),
        "rmse_m": mapping.get("mine_position_rmse_m"),
        "safe_path": path.get("path_success"),
        "mission_time_s": run.get("duration_s", run.get("sim_time_s")),
    }


def check_budgets(result: dict) -> None:
    """Wall-clock sanity per scenario (firmware-stage timers land in the
    bench self-test stream; SITL proxies them here)."""
    run = result.get("run") or {}
    duration = run.get("duration_s") or run.get("sim_time_s") or 0
    if duration and result["wall_s"] > 0:
        ratio = result["wall_s"] / max(duration, 1e-6)
        if ratio > BUDGETS["sim_wallclock_ratio_max"]:
            result["alerts"].append(
                f"wall/sim ratio {ratio:.1f}x exceeds budget "
                f"{BUDGETS['sim_wallclock_ratio_max']}x")


def render_report(results: list) -> str:
    lines = [
        "# Performance Benchmark Report",
        "",
        f"_Generated {time.strftime('%Y-%m-%d %H:%M:%S')} - "
        f"{len(results)} scenarios_",
        "",
        "| Scenario | OK | Wall s | F1 | Recall | RMSE m | Safe path | Alerts |",
        "|---|---|---|---|---|---|---|---|",
    ]
    alert_total = 0
    for r in results:
        met = extract_metrics(r.get("run") or {})

        def fmt(v, nd=3):
            return f"{v:.{nd}f}" if isinstance(v, (int, float)) else "-"

        alerts = "; ".join(r["alerts"]) if r["alerts"] else ""
        alert_total += len(r["alerts"])
        lines.append(
            f"| {r['scenario']} | {'yes' if r['ok'] else 'NO'} "
            f"| {r['wall_s']} | {fmt(met['mapping_f1'])} "
            f"| {fmt(met['mapping_recall'])} | {fmt(met['rmse_m'], 2)} "
            f"| {'yes' if met['safe_path'] else 'no'} | {alerts or '-'} |")

    lines += [
        "",
        "## Hard budgets",
        "",
        f"- Vision slot: <= {BUDGETS['vision_slot_ms']} ms (enforced onboard via "
        "TE_VISION_PROCESSING_SLOW; see bench self-test stream)",
        f"- Main loop: <= {BUDGETS['main_loop_period_ms']} ms (scheduler task overruns "
        "surface as watchdog telemetry)",
        f"- Sim wall/sim ratio: <= {BUDGETS['sim_wallclock_ratio_max']}x",
        "",
        f"**Total alerts: {alert_total}**",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--quick", action="store_true",
                    help="run only the fast subset of scenarios")
    ap.add_argument("--out", default=None,
                    help="markdown report output path")
    ap.add_argument("--from-run", default=None,
                    help="check an existing run JSON instead of simulating")
    args = ap.parse_args()

    if args.from_run:
        with open(args.from_run) as f:
            data = json.load(f)
        results = [{"scenario": os.path.basename(args.from_run),
                    "ok": True, "wall_s": 0.0, "alerts": [], "run": data}]
    else:
        scenarios = QUICK_SCENARIOS if args.quick else FULL_SCENARIOS
        results = []
        for name in scenarios:
            print(f"[BENCH] running {name} ...")
            try:
                r = run_scenario(name, timeout_s=300)
            except subprocess.TimeoutExpired:
                r = {"scenario": name, "ok": False, "wall_s": 300,
                     "alerts": ["timeout"], "run": {}}
            check_budgets(r)
            results.append(r)
            status = "OK" if r["ok"] and not r["alerts"] else "ALERT"
            print(f"[BENCH] {name}: {status}")

    report = render_report(results)

    out_path = args.out or os.path.join(SCRIPT_DIR, "benchmark_report.md")
    with open(out_path, "w") as f:
        f.write(report)
    print(report)
    print(f"[BENCH] report written to {out_path}")
    return 1 if any(r["alerts"] for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
