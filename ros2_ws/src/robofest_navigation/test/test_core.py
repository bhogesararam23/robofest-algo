import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from robofest_navigation.map_core import Detection, MineMapStore
from robofest_navigation.planner_core import GridPlanner

def test_map_fuses_and_decays():
    store=MineMapStore(); store.add_global(Detection(x=2,y=3,confidence=.8),0); store.add_global(Detection(x=2.2,y=3.1,confidence=.7),1)
    assert len(store.mines)==1 and store.mines[0].confidence>.8
    store.decay(7); assert store.mines[0].confidence < 1.0

def test_map_relative_transform():
    x,y=MineMapStore.transform_relative(1,0,4,5,math.pi/2); assert abs(x-4)<1e-6 and abs(y-6)<1e-6

def test_planner_avoids_inflated_mine():
    planner=GridPlanner(); planner.rebuild([(30,7.5)]); path=planner.plan((1,7.5),(59,7.5)); assert path
    for x,y in path: assert math.hypot(x-30,y-7.5)>=.99

def test_planner_rejects_blocked_endpoints():
    planner=GridPlanner(); planner.rebuild([(1,1)]); assert planner.plan((1,1),(5,5)) is None
