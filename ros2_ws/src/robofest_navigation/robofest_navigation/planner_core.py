"""A* safe-path planner for the 60 m x 15 m RoboFest arena."""
from heapq import heappop, heappush
from math import hypot
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

class GridPlanner:
    def __init__(self, width: float = 60.0, height: float = 15.0, resolution: float = 0.2,
                 clearance: float = 1.0):
        self.width, self.height, self.resolution, self.clearance = width, height, resolution, clearance
        self.cols, self.rows = round(width / resolution), round(height / resolution)
        self.costs = [0] * (self.cols * self.rows)

    def cell(self, point: Point) -> Tuple[int, int]:
        return int(point[0] / self.resolution), int(point[1] / self.resolution)

    def point(self, cell: Tuple[int, int]) -> Point:
        return ((cell[0] + 0.5) * self.resolution, (cell[1] + 0.5) * self.resolution)

    def _index(self, col: int, row: int) -> int:
        return row * self.cols + col

    def rebuild(self, mines: Iterable[Point]) -> None:
        self.costs = [0] * (self.cols * self.rows)
        radius = int(self.clearance / self.resolution + 0.999)
        for x, y in mines:
            c, r = self.cell((x, y))
            for rr in range(max(0, r - radius), min(self.rows, r + radius + 1)):
                for cc in range(max(0, c - radius), min(self.cols, c + radius + 1)):
                    if hypot(cc - c, rr - r) * self.resolution <= self.clearance:
                        self.costs[self._index(cc, rr)] = 253

    def plan(self, start: Point, goal: Point) -> Optional[List[Point]]:
        sc, sr = self.cell(start); gc, gr = self.cell(goal)
        if not (0 <= sc < self.cols and 0 <= sr < self.rows and 0 <= gc < self.cols and 0 <= gr < self.rows):
            return None
        start_i, goal_i = self._index(sc, sr), self._index(gc, gr)
        if self.costs[start_i] >= 253 or self.costs[goal_i] >= 253:
            return None
        open_set = [(hypot(gc - sc, gr - sr), 0.0, start_i)]
        came = {}
        g_score = {start_i: 0.0}
        directions = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
        while open_set:
            _, current_g, current = heappop(open_set)
            if current_g > g_score.get(current, float('inf')):
                continue
            if current == goal_i:
                cells = [current]
                while cells[-1] in came:
                    cells.append(came[cells[-1]])
                cells.reverse()
                return self._smooth([self.point(self._cell_from_index(i)) for i in cells], start, goal)
            cc, cr = self._cell_from_index(current)
            for dx, dy in directions:
                nc, nr = cc + dx, cr + dy
                if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                    continue
                ni = self._index(nc, nr)
                if self.costs[ni] >= 253:
                    continue
                step = 1.41421356237 if dx and dy else 1.0
                tentative = g_score[current] + step
                if tentative < g_score.get(ni, float('inf')):
                    came[ni] = current; g_score[ni] = tentative
                    h = hypot(gc - nc, gr - nr)
                    heappush(open_set, (tentative + h, tentative, ni))
        return None

    def _cell_from_index(self, index: int) -> Tuple[int, int]:
        return index % self.cols, index // self.cols

    def _clear_segment(self, a: Point, b: Point) -> bool:
        distance = hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(distance / (self.resolution * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            c, r = self.cell((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            if not (0 <= c < self.cols and 0 <= r < self.rows) or self.costs[self._index(c, r)] >= 253:
                return False
        return True

    def _smooth(self, path: Sequence[Point], start: Point, goal: Point) -> List[Point]:
        if not path:
            return []
        result = [start]
        anchor = 0
        while anchor < len(path) - 1:
            farthest = anchor + 1
            for candidate in range(anchor + 1, len(path)):
                if self._clear_segment(path[anchor], path[candidate]):
                    farthest = candidate
                else:
                    break
            result.append(path[farthest]); anchor = farthest
        if hypot(result[-1][0] - goal[0], result[-1][1] - goal[1]) > self.resolution:
            result.append(goal)
        else:
            result[-1] = goal
        return result
