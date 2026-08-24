import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from robofest_simulation.analyze_bag import verify_paths_clearance

def test_clearance_validator():
    assert not verify_paths_clearance([[(0,0),(0,2)]],[(2,0)])
    assert verify_paths_clearance([[(0,0),(1,0)]],[(1.5,0)])
