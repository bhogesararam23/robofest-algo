"""
Swarm Coordination and Mesh Communication Model for Robofest Gujarat 6.0 SITL Harness.
Simulates distributed P2P radio packets, claim/yield arbitration, and role failover.
"""

import random
from typing import Dict, List, Any, Optional
try:
    from sim import config
    from sim.drone_model import SimulatedDrone
except ImportError:
    import config
    from drone_model import SimulatedDrone


class SwarmModel:
    def __init__(self, drones: List[SimulatedDrone], seed: Optional[int] = None):
        self.drones = {d.drone_id: d for d in drones}
        if seed is not None:
            random.seed(seed)

        self.last_heartbeat_time_s: Dict[int, float] = {d.drone_id: 0.0 for d in drones}
        self.peer_online_status: Dict[int, bool] = {d.drone_id: True for d in drones}

        # Swarm Metrics counters
        self.messages_sent = 0
        self.messages_received = 0
        self.messages_dropped = 0
        self.claim_conflicts = 0
        self.yield_events = 0
        self.duplicate_suppressions = 0
        self.peer_lost_events = 0
        self.role_failovers = 0

    def broadcast_packet(self, sender_id: int, packet_type: str, payload: Dict[str, Any], timestamp_s: float):
        """
        Broadcasts packet to all peers with simulated radio packet loss.
        """
        self.messages_sent += 1

        for peer_id, peer_drone in self.drones.items():
            if peer_id == sender_id:
                continue

            if not self.peer_online_status.get(peer_id, False):
                continue

            # Simulate radio packet drop
            if random.random() < config.RADIO_PACKET_LOSS_PROBABILITY:
                self.messages_dropped += 1
                continue

            self.messages_received += 1
            self._handle_incoming_packet(peer_drone, sender_id, packet_type, payload, timestamp_s)

    def _handle_incoming_packet(self, receiver: SimulatedDrone, sender_id: int,
                                packet_type: str, payload: Dict[str, Any], timestamp_s: float):
        if packet_type == "HEARTBEAT":
            self.last_heartbeat_time_s[sender_id] = timestamp_s

        elif packet_type == "MINE_UPDATE":
            # Cross-drone deduplication and fusion
            mine_x = payload["x"]
            mine_y = payload["y"]
            conf = payload["confidence"]
            mtype = payload["marker_type"]

            # Deduplication
            matched = False
            for local_m in receiver.local_mines.values():
                dist = ((local_m["x"] - mine_x)**2 + (local_m["y"] - mine_y)**2)**0.5
                if dist < 0.50:  # Cross drone dedup radius
                    matched = True
                    self.duplicate_suppressions += 1
                    local_m["persistence_count"] += 1
                    local_m["confidence"] = min(100.0, local_m["confidence"] * 0.8 + conf * 0.2)
                    if local_m["persistence_count"] >= 3 and local_m["confidence"] >= 70.0:
                        if local_m["status"] != "CONFIRMED":
                            local_m["status"] = "CONFIRMED"
                            receiver.confirmed_mine_count += 1
                    break

            if not matched:
                mid = receiver.next_mine_id
                receiver.next_mine_id += 1
                receiver.local_mines[mid] = {
                    "mine_id": mid,
                    "x": mine_x,
                    "y": mine_y,
                    "confidence": conf,
                    "persistence_count": 1,
                    "status": "CANDIDATE",
                    "marker_type": mtype
                }

        elif packet_type == "CLAIM":
            mine_hash = payload.get("mine_hash", 0)
            if mine_hash in receiver.claimed_mine_hashes:
                # Claim conflict! Drone with lower ID yields
                self.claim_conflicts += 1
                if receiver.drone_id > sender_id:
                    receiver.claimed_mine_hashes.remove(mine_hash)
                    self.yield_events += 1

        elif packet_type == "PATH_UPDATE":
            if payload.get("version", 0) > receiver.active_path_version:
                receiver.active_path = payload.get("path", [])
                receiver.active_path_version = payload.get("version", 0)

    def update_swarm(self, timestamp_s: float, fail_peer_id: Optional[int] = None):
        """
        Runs periodic peer liveness and dynamic role failover.
        """
        # Inject simulated peer failure if specified
        if fail_peer_id and fail_peer_id in self.peer_online_status:
            self.peer_online_status[fail_peer_id] = False

        # Periodic Heartbeat broadcast
        for drone in self.drones.values():
            if self.peer_online_status.get(drone.drone_id, False):
                self.broadcast_packet(drone.drone_id, "HEARTBEAT", {
                    "role": drone.role,
                    "state": drone.state,
                    "battery": drone.battery_percent
                }, timestamp_s)

        # Peer timeout check
        for peer_id, last_hb in self.last_heartbeat_time_s.items():
            if not self.peer_online_status[peer_id]:
                continue

            if (timestamp_s - last_hb) > config.PEER_LOST_TIMEOUT_S and timestamp_s > 3.0:
                self.peer_online_status[peer_id] = False
                self.peer_lost_events += 1

                # Role Failover: Assign orphaned lane to active peer
                self._trigger_role_failover(peer_id)

    def _trigger_role_failover(self, failed_peer_id: int):
        self.role_failovers += 1
        failed_drone = self.drones.get(failed_peer_id)
        if not failed_drone:
            return

        # Reassign other active drone to expand coverage
        for peer_id, d in self.drones.items():
            if peer_id != failed_peer_id and self.peer_online_status.get(peer_id, False):
                d.lane_x_min = config.FIELD_X_MIN + 0.5
                d.lane_x_max = config.FIELD_X_MAX - 0.5
                break
