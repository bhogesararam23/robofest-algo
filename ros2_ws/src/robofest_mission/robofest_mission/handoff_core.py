"""Pure handoff protocol state machine used by the ROS 2 swarm node."""
from dataclasses import dataclass, field
from typing import Iterable, Tuple

PENDING = 'PENDING'
ACCEPT = 'ACCEPT'
ACK = 'ACK'
RELEASE = 'RELEASE'
EXPIRED = 'EXPIRED'

@dataclass
class Handoff:
    handoff_id: str = ''
    status: str = ''
    target_id: int = 0
    created_at: float = 0.0
    expires_at: float = 0.0
    acknowledged: bool = False

@dataclass
class SwarmPeer:
    drone_id: int
    battery_percent: float
    is_available: bool
    current_task: str
    handoff_id: str = ''
    handoff_status: str = ''
    handoff_target_id: int = 0

class HandoffManager:
    def __init__(self, self_id: int, lease_seconds: float = 5.0, handoff_timeout: float = 8.0):
        self.self_id = self_id
        self.lease_seconds = lease_seconds
        self.handoff_timeout = handoff_timeout
        self.current = Handoff()
        self.counter = 0

    def choose_recipient(self, peers: Iterable[SwarmPeer]) -> Tuple[int, ...]:
        return tuple(sorted(p.drone_id for p in peers if p.drone_id != self.self_id and p.is_available and p.battery_percent > 50.0))

    def start(self, now: float, peers: Iterable[SwarmPeer], local_task: str, local_battery: float) -> Handoff:
        if local_task != 'GUIDE_HUMAN' or local_battery >= 20.0 or self.current.status == PENDING:
            return self.current
        recipients = self.choose_recipient(peers)
        if not recipients:
            return self.current
        self.counter += 1
        handoff_id = f'{self.self_id}-{self.counter}'
        self.current = Handoff(handoff_id, PENDING, recipients[0], now, now + self.handoff_timeout, False)
        return self.current

    def receive(self, now: float, peer: SwarmPeer, local_available: bool) -> Handoff:
        if peer.drone_id == self.self_id or not peer.handoff_id:
            return self.current
        if peer.handoff_status == PENDING and peer.handoff_target_id == self.self_id and local_available:
            self.current = Handoff(peer.handoff_id, ACK, self.self_id, now, now + self.lease_seconds, True)
        elif peer.handoff_status == RELEASE and peer.handoff_id == self.current.handoff_id:
            self.current.status = RELEASE
        return self.current

    def apply_ack(self, peer: SwarmPeer, now: float) -> bool:
        if self.current.status != PENDING or peer.handoff_id != self.current.handoff_id:
            return False
        if peer.drone_id == self.current.target_id and peer.handoff_status == ACK:
            self.current.status = RELEASE
            self.current.acknowledged = True
            self.current.expires_at = now + self.lease_seconds
            return True
        return False

    def tick(self, now: float) -> Handoff:
        if self.current.status == PENDING and now > self.current.expires_at:
            self.current.status = EXPIRED
        elif self.current.status == ACK and now > self.current.expires_at:
            self.current.status = EXPIRED
        return self.current
