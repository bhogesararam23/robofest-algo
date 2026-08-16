"""
Score Evaluation and Report Generator for Robofest Gujarat 6.0 SITL Harness.
Computes official-like category points, deductions, and engineering tuning recommendations.
"""

import sys
import json
from typing import Dict, Any, Tuple
try:
    from sim import config
except ImportError:
    import config


def evaluate_score(metrics: Dict[str, Any], duration_s: float) -> Tuple[Dict[str, float], float, float, list]:
    """
    Computes category breakdown, penalty deductions, and actionable recommendations.
    """
    scores: Dict[str, float] = {}
    recommendations = []

    # 1. Takeoff & Activation (Max 10 pts)
    act = metrics.get("activation", {})
    if act.get("start_command_accepted") and act.get("takeoff_success"):
        scores["takeoff_and_activation"] = config.TAKEOFF_AND_ACTIVATION_WEIGHT
    else:
        scores["takeoff_and_activation"] = 0.0
        recommendations.append("Verify start command debouncing or check battery self-test status.")

    # 2. Command Recognition (Max 10 pts)
    cmd = metrics.get("command_recognition", {})
    if cmd.get("commands_accepted", 0) > 0 and cmd.get("false_triggers", 0) == 0:
        scores["command_recognition"] = config.COMMAND_RECOGNITION_WEIGHT
    else:
        scores["command_recognition"] = 5.0

    # 3. Swarm Formation & Coordination (Max 10 pts)
    swm = metrics.get("swarm", {})
    if swm.get("formation_success", False):
        scores["swarm_formation"] = config.SWARM_FORMATION_WEIGHT
    else:
        scores["swarm_formation"] = 0.0
        recommendations.append("Increase peer heartbeat timeout or check radio latency.")

    # 4. Mine Detection & Mapping (Max 25 pts)
    mapping = metrics.get("mapping", {})
    f1 = mapping.get("f1_score", 0.0)
    scores["mine_detection_mapping"] = round(config.MINE_DETECTION_MAPPING_WEIGHT * f1, 2)
    if mapping.get("recall", 0.0) < 0.80:
        recommendations.append("Improve buried surface marker detection HSV thresholds or lower confidence threshold.")
    if mapping.get("precision", 0.0) < 0.80:
        recommendations.append("Increase persistence count threshold to suppress false positive glare.")

    # 5. Safe Path Creation (Max 20 pts)
    pth = metrics.get("path", {})
    if pth.get("path_success", False):
        scores["safe_path_creation"] = config.SAFE_PATH_CREATION_MARKING_WEIGHT
    else:
        scores["safe_path_creation"] = 5.0
        if pth.get("path_clearance_violation_count", 0) > 0:
            recommendations.append("Increase A* obstacle inflation radius to guarantee >= 1.0m mine clearance.")

    # 6. Safe Human Crossing (Max 20 pts)
    hmn = metrics.get("human_crossing", {})
    if hmn.get("crossing_success", False):
        scores["safe_human_crossing"] = config.SAFE_HUMAN_CROSSING_WEIGHT
    else:
        scores["safe_human_crossing"] = 0.0
        recommendations.append("Adjust human tracking search bias and ensure guidance marker remains visible.")

    # 7. Time Bonus (Max 5 pts)
    if duration_s < 300.0 and hmn.get("crossing_success", False):
        scores["time_bonus"] = config.TIME_BONUS_WEIGHT
    elif duration_s < 450.0 and hmn.get("crossing_success", False):
        scores["time_bonus"] = 2.5
    else:
        scores["time_bonus"] = 0.0

    # Penalties
    safety = metrics.get("safety", {})
    penalty_total = 0.0

    col_count = safety.get("collision_count", 0)
    if col_count > 0:
        penalty_total += (col_count * config.COLLISION_OR_UNSAFE_PROXIMITY_PENALTY)
        recommendations.append("Increase inter-drone separation distance in search behavior.")

    surf_count = safety.get("surface_contact_count", 0)
    if surf_count > 0:
        penalty_total += (surf_count * config.CRASH_OR_SURFACE_CONTACT_PENALTY)
        recommendations.append("Inspect ToF altitude hold PID gains to prevent unwanted ground contact.")

    path_viol = pth.get("path_clearance_violation_count", 0)
    if path_viol > 0:
        penalty_total += (path_viol * config.CLEARANCE_VIOLATION_PENALTY)

    gross_score = sum(scores.values())
    total_score = max(0.0, round(gross_score + penalty_total, 2))

    return scores, penalty_total, total_score, recommendations


def generate_markdown_report(run_config: Dict[str, Any],
                             metrics: Dict[str, Any],
                             scores: Dict[str, float],
                             penalty_total: float,
                             total_score: float,
                             recommendations: list) -> str:
    """
    Renders structured Markdown report suitable for review and verification.
    """
    lines = []
    lines.append("# Robofest Gujarat 6.0 SITL Simulation & Score Evaluation Report\n")
    lines.append("## 1. Mission Configuration & Execution Summary\n")
    lines.append(f"- **Scenario**: `{run_config.get('scenario', 'nominal_map')}`")
    lines.append(f"- **Random Seed**: `{run_config.get('seed', 1)}`")
    lines.append(f"- **Drones**: `{run_config.get('drones', 3)}`")
    lines.append(f"- **True Mines**: `{run_config.get('mines', 40)}`")
    lines.append(f"- **Mission Duration**: `{run_config.get('duration_s', 0.0):.1f} s` (Limit: 600.0 s)\n")

    lines.append("## 2. Category Scoring Breakdown\n")
    lines.append("| Score Category | Weight | Awarded Points | Status |")
    lines.append("| :--- | :---: | :---: | :---: |")
    lines.append(f"| Takeoff & Activation | 10.0 | {scores.get('takeoff_and_activation', 0.0):.1f} | {'PASS' if scores.get('takeoff_and_activation', 0.0) >= 10.0 else 'WARN'} |")
    lines.append(f"| Command Recognition | 10.0 | {scores.get('command_recognition', 0.0):.1f} | {'PASS' if scores.get('command_recognition', 0.0) >= 10.0 else 'WARN'} |")
    lines.append(f"| Swarm Coordination | 10.0 | {scores.get('swarm_formation', 0.0):.1f} | {'PASS' if scores.get('swarm_formation', 0.0) >= 10.0 else 'WARN'} |")
    lines.append(f"| Mine Detection & Mapping | 25.0 | {scores.get('mine_detection_mapping', 0.0):.1f} | {'PASS' if scores.get('mine_detection_mapping', 0.0) >= 18.0 else 'WARN'} |")
    lines.append(f"| Safe Path (1.0m Clearance) | 20.0 | {scores.get('safe_path_creation', 0.0):.1f} | {'PASS' if scores.get('safe_path_creation', 0.0) >= 20.0 else 'FAIL'} |")
    lines.append(f"| Safe Human Crossing | 20.0 | {scores.get('safe_human_crossing', 0.0):.1f} | {'PASS' if scores.get('safe_human_crossing', 0.0) >= 20.0 else 'FAIL'} |")
    lines.append(f"| Time Efficiency Bonus | 5.0 | {scores.get('time_bonus', 0.0):.1f} | INFO |")
    lines.append(f"| **Penalty Deductions** | — | **{penalty_total:.1f}** | {'OK' if penalty_total == 0 else 'PENALTY'} |")
    lines.append(f"| **ESTIMATED TOTAL SCORE** | **100.0** | **{total_score:.1f}** | {'EXCELLENT' if total_score >= 80 else 'ACCEPTABLE'} |\n")

    lines.append("## 3. Detailed Per-Subsystem Metrics\n")
    lines.append("### Mapping & Detection:")
    m = metrics.get("mapping", {})
    lines.append(f"- True Mines: {m.get('true_mine_count')} | Detected: {m.get('detected_true_mines')} | False Positives: {m.get('false_positive_mines')}")
    lines.append(f"- Precision: {m.get('precision'):.1%} | Recall: {m.get('recall'):.1%} | F1 Score: {m.get('f1_score'):.3f} | Pos RMSE: {m.get('mine_position_rmse_m')} m\n")

    lines.append("### Path & Safety Verification:")
    p = metrics.get("path", {})
    s = metrics.get("safety", {})
    lines.append(f"- Path Success: `{p.get('path_success')}` | Min Clearance to True Mines: `{p.get('minimum_clearance_to_true_mines_m')} m`")
    lines.append(f"- 1.0m Clearance Violations: `{p.get('path_clearance_violation_count')}`")
    lines.append(f"- Collisions: `{s.get('collision_count')}` | Unsafe Proximities: `{s.get('unsafe_proximity_count')}` | Surface Contacts: `{s.get('surface_contact_count')}`\n")

    lines.append("## 4. Firmware Tuning Recommendations\n")
    if recommendations:
        for r in recommendations:
            lines.append(f"- [RECOMMENDATION] {r}")
    else:
        lines.append("- [PASSED] All subsystems executed cleanly within safety and precision bounds.")

    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python score_eval.py <sim_run.json> [--report <output.md>]")
        sys.exit(1)

    json_path = sys.argv[1]
    report_path = None
    if "--report" in sys.argv:
        idx = sys.argv.index("--report")
        if idx + 1 < len(sys.argv):
            report_path = sys.argv[idx + 1]

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    metrics = data.get("metrics", {})
    run_cfg = data.get("config", {})
    dur = run_cfg.get("duration_s", 600.0)

    scores, penalty_total, total_score, recs = evaluate_score(metrics, dur)
    report_md = generate_markdown_report(run_cfg, metrics, scores, penalty_total, total_score, recs)

    if report_path:
        with open(report_path, "w", encoding="utf-8") as f:
            f.write(report_md)
        print(f"Report written to {report_path}")
    else:
        print(report_md)
