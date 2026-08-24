import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from robofest_navigation.planner_core import GridPlanner


def test_overlapping_mines_form_union_and_removed_mines_clear_grid():
    planner = GridPlanner()
    planner.rebuild([(20.0, 7.5), (20.8, 7.5)])
    assert planner.costs[planner._index(*planner.cell((20.4, 7.5)))] == 253
    planner.rebuild([])
    assert planner.costs[planner._index(*planner.cell((20.4, 7.5)))] == 0


def test_no_route_is_returned_when_cluster_seals_the_arena():
    planner = GridPlanner()
    # A mine row spaced at 0.8 m has overlapping 1.0 m clearance discs across the arena width.
    planner.rebuild([(x, 7.5) for x in [1.0 + 0.8 * i for i in range(73)]])
    assert planner.plan((0.5, 7.5), (59.0, 7.5)) is None
