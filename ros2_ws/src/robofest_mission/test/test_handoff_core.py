import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from robofest_mission.handoff_core import HandoffManager, SwarmPeer, PENDING, ACK, RELEASE, EXPIRED


def test_handoff_requires_safe_recipient_and_acknowledgement():
    manager = HandoffManager(self_id=1, handoff_timeout=8.0)
    peers = [SwarmPeer(2, 75.0, True, 'IDLE'), SwarmPeer(3, 90.0, False, 'IDLE')]
    pending = manager.start(10.0, peers, 'GUIDE_HUMAN', 15.0)
    assert pending.status == PENDING and pending.target_id == 2
    assert manager.start(11.0, peers, 'GUIDE_HUMAN', 15.0).handoff_id == pending.handoff_id
    assert not manager.apply_ack(SwarmPeer(3, 90.0, False, 'IDLE', pending.handoff_id, ACK), 11.0)
    assert manager.apply_ack(SwarmPeer(2, 75.0, True, 'ACCEPT_GUIDANCE', pending.handoff_id, ACK), 11.0)
    assert manager.current.status == RELEASE


def test_recipient_acknowledges_only_targeted_handoff():
    recipient = HandoffManager(self_id=2)
    peer = SwarmPeer(1, 10.0, False, 'GUIDE_HUMAN', '1-4', PENDING)
    peer.handoff_target_id = 2
    assert recipient.receive(2.0, peer, True).status == ACK
    assert recipient.current.handoff_id == '1-4'


def test_handoff_expires_without_ack():
    manager = HandoffManager(self_id=1, handoff_timeout=2.0)
    manager.start(0.0, [SwarmPeer(2, 80.0, True, 'IDLE')], 'GUIDE_HUMAN', 10.0)
    assert manager.tick(2.1).status == EXPIRED
