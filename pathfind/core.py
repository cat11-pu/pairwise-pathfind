"""Grid pathfinding (A*) and field of view, standard library only."""

import heapq
import math

SQRT2 = math.sqrt(2.0)

_ORTHOGONAL_STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))
_DIAGONAL_STEPS = ((1, 1), (1, -1), (-1, 1), (-1, -1))


class Grid(object):
    """A rectangular grid of cells.

    Cells listed in walls block movement.  Cells listed in terrain carry a
    cost multiplier that is paid when a move enters them.
    """

    def __init__(self, width, height, walls=(), terrain=None):
        if int(width) <= 0 or int(height) <= 0:
            raise ValueError("width and height must be positive")
        self.width = int(width)
        self.height = int(height)
        self.walls = set(tuple(cell) for cell in walls)
        self.terrain = dict(
            (tuple(cell), float(cost)) for cell, cost in (terrain or {}).items()
        )

    def __repr__(self):
        return "Grid(%d, %d)" % (self.width, self.height)

    def in_bounds(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_walkable(self, cell):
        return self.in_bounds(cell) and tuple(cell) not in self.walls

    def is_blocked(self, cell):
        return not self.is_walkable(cell)

    def terrain_cost(self, cell):
        return self.terrain.get(tuple(cell), 1.0)

    def move_cost(self, from_cell, to_cell):
        """Cost of one step between two adjacent cells."""
        dx = abs(from_cell[0] - to_cell[0])
        dy = abs(from_cell[1] - to_cell[1])
        if max(dx, dy) != 1:
            raise ValueError(
                "cells are not adjacent: %r -> %r" % (tuple(from_cell), tuple(to_cell))
            )
        base = SQRT2 if dx == 1 and dy == 1 else 1.0
        return base * self.terrain_cost(to_cell)

    def iter_neighbors(self, cell, diagonal=True):
        """Yield the walkable neighbours of a cell, orthogonal ones first."""
        x, y = cell
        for dx, dy in _ORTHOGONAL_STEPS:
            neighbour = (x + dx, y + dy)
            if self.is_walkable(neighbour):
                yield neighbour
        if not diagonal:
            return
        for dx, dy in _DIAGONAL_STEPS:
            neighbour = (x + dx, y + dy)
            if not self.is_walkable(neighbour):
                continue
            side_x = (x + dx, y)
            side_y = (x, y + dy)
            if not (self.is_walkable(side_x) and self.is_walkable(side_y)):
                continue
            yield neighbour


def heuristic(a, b, diagonal=True):
    """Estimated distance between two cells."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if diagonal:
        return float(max(dx, dy)) + (SQRT2 - 1.0) * min(dx, dy)
    return float(dx + dy)


def pop_best(open_list):
    """Remove and return the next cell to expand."""
    return open_list.pop(0)[1]


def reconstruct_path(came_from, current):
    """Return the recorded chain of cells for current."""
    path = [current]
    node = current
    while node in came_from:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path


def find_path(grid, start, goal, diagonal=True):
    """Return the path from start to goal."""
    start = tuple(start)
    goal = tuple(goal)
    if not grid.is_walkable(start):
        raise ValueError("start cell is not walkable: %r" % (start,))
    if not grid.is_walkable(goal):
        raise ValueError("goal cell is not walkable: %r" % (goal,))
    if start == goal:
        return [start]

    open_heap = [(heuristic(start, goal, diagonal), 0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, current_g, current = heapq.heappop(open_heap)
        if current == goal:
            return reconstruct_path(came_from, current)
        if current in closed:
            continue
        closed.add(current)
        for neighbour in grid.iter_neighbors(current, diagonal):
            if neighbour in closed:
                continue
            tentative_g = current_g + grid.move_cost(current, neighbour)
            if tentative_g < g_score.get(neighbour, float("inf")):
                g_score[neighbour] = tentative_g
                came_from[neighbour] = current
                heapq.heappush(
                    open_heap,
                    (tentative_g + heuristic(neighbour, goal, diagonal),
                     tentative_g, neighbour),
                )
    return None


def path_cost(grid, path):
    """Total cost of walking a path; None for a missing path."""
    if path is None:
        return None
    total = 0.0
    for index in range(len(path) - 1):
        total += grid.move_cost(path[index], path[index + 1])
    return total


def line(start, end):
    """Cells of the straight line between two cells."""
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    step_x = 1 if x1 >= x0 else -1
    step_y = 1 if y1 >= y0 else -1
    err = dx - dy
    cells = []
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if (x, y) == (x1, y1):
            break
        err2 = 2 * err
        if err2 > -dy:
            err -= dy
            x += step_x
        if err2 < dx:
            err += dx
            y += step_y
    return cells


def compute_fov(grid, origin, radius):
    """Cells visible from origin within a Chebyshev radius."""
    origin = tuple(origin)
    if radius < 0:
        raise ValueError("radius must not be negative")
    if not grid.is_walkable(origin):
        raise ValueError("origin must be walkable: %r" % (origin,))
    ox, oy = origin
    visible = {origin}
    for x in range(ox - radius, ox + radius + 1):
        for y in range(oy - radius, oy + radius + 1):
            target = (x, y)
            if target == origin or not grid.in_bounds(target):
                continue
            for cell in line(origin, target):
                visible.add(cell)
                if grid.is_blocked(cell):
                    break
    return visible
