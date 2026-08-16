"""
Main SITL Simulation Runner for Robofest Gujarat 6.0 Minefield Swarm Drone.
Executes closed-loop multi-agent simulation, logs telemetry events, and evaluates mission score.
"""

import sys
import json
import math
import random
import argparse
from typing import Dict, List, Any

try:
    from sim import config
    from sim.map_generator import MapGenerator
    from sim.sensor_sim import SensorSimulator
    from sim.drone_model import SimulatedDrone
    from sim.swarm_model import SwarmModel
    from sim.human_model import HumanModel
    from sim.scenarios import get_scenario
    from sim.metrics import calculate_metrics
    from sim.score_eval import evaluate_score, generate_markdown_report
except ImportError:
    import config
    from map_generator import MapGenerator
    from sensor_sim import SensorSimulator
    from drone_model import SimulatedDrone
    from swarm_model import SwarmModel
    from human_model import HumanModel
    from scenarios import get_scenario
    from metrics import calculate_metrics
    from score_eval import evaluate_score, generate_markdown_report


class SITLHarness:
    def __init__(self, seed: int = 1, scenario_name: str = "nominal_map",
                 num_drones: int = 3, num_mines: int = 40, max_duration_s: float = 600.0):
        self.seed = seed
        self.scenario = get_scenario(scenario_name)
        self.num_drones = max(3, min(config.MAX_DRONE_COUNT, num_drones))
        self.num_mines = num_mines if num_mines != 40 else self.scenario.get("mine_count", 40)
        self.max_duration_s = max_duration_s

        random.seed(seed)

        # 1. Generate Synthetic Arena Map
        map_gen = MapGenerator(seed=seed)
        self.field_data = map_gen.generate_field(
            mine_count=self.num_mines,
            on_ground_ratio=self.scenario.get("on_ground_ratio", config.ON_GROUND_MINE_RATIO),
            obstacle_count=self.scenario.get("obstacle_count", 0)
        )
        self.true_mines = self.field_data["mines"]
        self.true_obstacles = self.field_data["obstacles"]

        # 2. Instantiate Swarm Drones with Safe Initial Separation
        self.drones: List[SimulatedDrone] = []
        roles = ["SCOUT_LEFT", "SCOUT_RIGHT", "GUIDE_MARKER", "RESERVE"]
        start_x_offsets = [4.0, 11.0, 7.5, 2.0]

        for i in range(self.num_drones):
            drone_id = i + 1
            role = roles[i] if i < len(roles) else "RESERVE"
            sx = start_x_offsets[i] if i < len(start_x_offsets) else 7.5
            sy = 0.5  # Start Zone
            self.drones.append(SimulatedDrone(drone_id, role, sx, sy))

        # 3. Instantiate Subsystems
        self.sensor_sim = SensorSimulator(seed=seed)
        self.swarm_model = SwarmModel(self.drones, seed=seed)
        self.human_model = HumanModel(
            human_id=1,
            behavior=self.scenario.get("human_behavior", "obedient_human"),
            seed=seed
        )

        # 4. Simulation State
        self.sim_time_s = 0.0
        self.events: List[Dict[str, Any]] = []
        self.fused_mines_map: List[Dict[str, Any]] = []
        self.final_safe_path: List[Dict[str, float]] = []
        self.last_proximity_warn_s = 0.0

    def log_event(self, event_type: str, drone_id: Any = "global", details: str = ""):
        self.events.append({
            "timestamp_s": round(self.sim_time_s, 3),
            "drone_id": drone_id,
            "event_type": event_type,
            "details": details
        })

    def run(self) -> Dict[str, Any]:
        """
        Executes the main simulation loop.
        """
        self.log_event("mission_start", "global", f"Scenario: {self.scenario['name']} Seed: {self.seed}")
        self.log_event("self_check_pass", "global", "All 15 modules ready on ground")
        self.log_event("start_command_accepted", "global", "START gesture validated")

        # Command all drones to Takeoff
        for d in self.drones:
            d.state = "TAKEOFF"
            self.log_event("takeoff_started", d.drone_id)

        dt = config.SIM_DT_S
        takeoff_logged = False
        formation_logged = False
        planned_path_once = False

        while self.sim_time_s < self.max_duration_s:
            self.sim_time_s += dt

            # Check peer failure injection scenario
            peer_fail_id = 2 if (self.scenario.get("peer_failure_enabled", False) and self.sim_time_s > 60.0) else None

            # 1. Update Swarm Communications
            self.swarm_model.update_swarm(self.sim_time_s, fail_peer_id=peer_fail_id)

            # 2. Update Drone Sensors & Autonomous Behaviors
            for drone in self.drones:
                if not self.swarm_model.peer_online_status.get(drone.drone_id, True):
                    continue

                # Generate noisy sensor observations
                flow_vx, flow_vy, tof_alt, qual, flow_valid = self.sensor_sim.generate_flow_and_tof(
                    drone.true_vx, drone.true_vy, drone.true_alt_m, dt,
                    drift_multiplier=self.scenario.get("drift_multiplier", 1.0)
                )

                detections = self.sensor_sim.generate_vision_detections(
                    drone.drone_id, drone.true_x, drone.true_y, drone.true_alt_m,
                    self.true_mines, self.sim_time_s,
                    tp_multiplier=self.scenario.get("true_positive_multiplier", 1.0),
                    fp_multiplier=self.scenario.get("false_positive_multiplier", 1.0)
                )

                # Feed observations to drone
                drone.receive_sensor_observation(flow_vx, flow_vy, tof_alt, qual, flow_valid, detections, dt)

                # Broadcast confirmed mines to swarm
                for m in drone.local_mines.values():
                    if m["status"] == "CONFIRMED":
                        self.swarm_model.broadcast_packet(drone.drone_id, "MINE_UPDATE", {
                            "x": m["x"], "y": m["y"], "confidence": m["confidence"],
                            "marker_type": m["marker_type"]
                        }, self.sim_time_s)

                # Update physics & navigation
                drone.update_physics_and_behavior(dt)

            # Check Takeoff & Formation completion events
            if not takeoff_logged and all(d.true_alt_m >= (config.MISSION_ALTITUDE_M - 0.1) for d in self.drones):
                takeoff_logged = True
                self.log_event("takeoff_complete", "global", f"Alt: {config.MISSION_ALTITUDE_M}m")

            if not formation_logged and all(d.state in ["FORMATION", "SEARCHING", "WAIT_FOR_PATH"] for d in self.drones):
                formation_logged = True
                self.log_event("formation_complete", "global", "Fleet in lane holding")

            # 3. Path Planning & Guidance Trigger (once sufficient mines discovered)
            total_confirmed = sum(d.confirmed_mine_count for d in self.drones)
            guide_drone = next((d for d in self.drones if d.role == "GUIDE_MARKER"), self.drones[0])

            if (total_confirmed >= 6 or self.sim_time_s > 40.0) and not planned_path_once:
                guide_drone.plan_safe_path()
                self.final_safe_path = guide_drone.active_path
                self.swarm_model.broadcast_packet(guide_drone.drone_id, "PATH_UPDATE", {
                    "path": self.final_safe_path,
                    "version": guide_drone.active_path_version
                }, self.sim_time_s)
                planned_path_once = True
                guide_drone.state = "GUIDING"
                self.log_event("path_planned", guide_drone.drone_id, f"Waypoints: {len(self.final_safe_path)}")

            # 4. Human Guidance & Walking
            guidance_cmd = "SAFE_PATH" if planned_path_once else "WAIT"
            self.human_model.update(self.final_safe_path, guidance_cmd, self.sim_time_s, dt)

            # 5. Check Inter-Drone Proximity & Collisions (with debounced logging)
            for i in range(len(self.drones)):
                for j in range(i + 1, len(self.drones)):
                    d1, d2 = self.drones[i], self.drones[j]
                    dist = math.hypot(d1.true_x - d2.true_x, d1.true_y - d2.true_y)
                    if dist < config.DRONE_COLLISION_RADIUS_M * 2:
                        self.log_event("collision_detected", "global", f"Drone {d1.drone_id} & Drone {d2.drone_id}")
                    elif dist < config.UNSAFE_PROXIMITY_M:
                        if (self.sim_time_s - self.last_proximity_warn_s) > 3.0:
                            self.last_proximity_warn_s = self.sim_time_s
                            self.log_event("unsafe_proximity_detected", "global", f"Dist: {dist:.2f}m")

            # 6. Mission Stop Conditions
            if self.human_model.reached_exit:
                self.log_event("human_exit_reached", "global", f"Time: {self.sim_time_s:.1f}s")
                self.log_event("mission_complete", "global", "Safe corridor traversed successfully")
                for d in self.drones:
                    d.state = "LANDING"
                break

        if self.sim_time_s >= self.max_duration_s:
            self.log_event("mission_timeout", "global", "10-minute time limit reached")
            for d in self.drones:
                d.state = "LANDING"

        # Fuse global map for metric calculation (filter single-frame transient noise)
        fused = {}
        for d in self.drones:
            for mid, m in d.local_mines.items():
                if m["persistence_count"] >= 2 or m["status"] == "CONFIRMED":
                    fused[f"{d.drone_id}_{mid}"] = m
        self.fused_mines_map = list(fused.values())

        # Compile Run Results
        run_data = {
            "config": {
                "scenario": self.scenario["name"],
                "seed": self.seed,
                "drones": self.num_drones,
                "mines": self.num_mines,
                "duration_s": round(self.sim_time_s, 2)
            },
            "events": self.events,
            "drones": [d.get_state() for d in self.drones],
            "human": self.human_model.get_state(),
            "fused_mines": self.fused_mines_map,
            "final_path": self.final_safe_path
        }

        # Calculate Metrics & Scores
        metrics = calculate_metrics(run_data, self.true_mines)
        run_data["metrics"] = metrics

        scores, penalty_total, total_score, recs = evaluate_score(metrics, self.sim_time_s)
        run_data["score"] = {
            "category_scores": scores,
            "penalty_total": penalty_total,
            "total_score": total_score,
            "recommendations": recs
        }

        return run_data


def main():
    parser = argparse.ArgumentParser(description="Robofest Gujarat 6.0 Swarm SITL Simulation Harness")
    parser.add_argument("--seed", type=int, default=1, help="Random seed for repeatable runs")
    parser.add_argument("--scenario", type=str, default="nominal_map", help="Scenario preset name")
    parser.add_argument("--drones", type=int, default=3, help="Swarm drone fleet count (3 to 4)")
    parser.add_argument("--mines", type=int, default=40, help="Expected minefield mine count")
    parser.add_argument("--duration", type=float, default=600.0, help="Maximum mission time in seconds")
    parser.add_argument("--output", type=str, default="sim_run.json", help="Output JSON results filepath")
    parser.add_argument("--report", type=str, default="sim_report.md", help="Output Markdown report filepath")
    parser.add_argument("--headless", action="store_true", default=True, help="Run without graphical UI")

    args = parser.parse_args()

    harness = SITLHarness(
        seed=args.seed,
        scenario_name=args.scenario,
        num_drones=args.drones,
        num_mines=args.mines,
        max_duration_s=args.duration
    )

    print(f"[SITL] Starting Simulation: Scenario={args.scenario}, Seed={args.seed}, Drones={args.drones}...")
    results = harness.run()
    print(f"[SITL] Completed in {results['config']['duration_s']:.1f}s. Estimated Score: {results['score']['total_score']:.1f}/100.0")

    # Write output JSON
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"[SITL] Run data saved to {args.output}")

    # Generate Markdown Report
    report_md = generate_markdown_report(
        results["config"],
        results["metrics"],
        results["score"]["category_scores"],
        results["score"]["penalty_total"],
        results["score"]["total_score"],
        results["score"]["recommendations"]
    )

    with open(args.report, "w", encoding="utf-8") as f:
        f.write(report_md)
    print(f"[SITL] Markdown report saved to {args.report}")


if __name__ == "__main__":
    main()
