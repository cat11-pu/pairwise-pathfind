"""Tests for the pathfind kernel (unittest, standard library only)."""

import math
import unittest

from pathfind.core import Grid, compute_fov, find_path, path_cost

SQRT2 = math.sqrt(2.0)


class PathTests(unittest.TestCase):
    def test_returned_path_is_a_valid_chain_from_start_to_goal(self):
        grid = Grid(5, 5, walls={(2, 0), (2, 1), (2, 3), (2, 4)})
        path = find_path(grid, (0, 2), (4, 2))
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 5)
        self.assertEqual(path[0], (0, 2))
        self.assertEqual(path[-1], (4, 2))
        for cell in path:
            self.assertFalse(grid.is_blocked(cell))
        for first, second in zip(path, path[1:]):
            step = max(abs(first[0] - second[0]), abs(first[1] - second[1]))
            self.assertLessEqual(step, 1)

    def test_diagonal_step_price_is_sqrt_two(self):
        grid = Grid(4, 4)
        path = find_path(grid, (0, 0), (3, 3))
        self.assertEqual(path, [(0, 0), (1, 1), (2, 2), (3, 3)])
        self.assertAlmostEqual(path_cost(grid, path), 3.0 * SQRT2, places=9)

    def test_route_is_minimal_on_a_weighted_grid(self):
        grid = Grid(
            4,
            5,
            walls={(1, 3), (2, 2)},
            terrain={(3, 0): 2.0, (3, 2): 7.0, (3, 3): 7.0},
        )
        path = find_path(grid, (0, 0), (3, 4))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (3, 4))
        self.assertAlmostEqual(path_cost(grid, path), 7.0, places=9)

    def test_route_is_minimal_with_terrain_cells(self):
        grid = Grid(5, 3, walls={(3, 1)}, terrain={(2, 2): 6.0, (3, 0): 6.0})
        path = find_path(grid, (0, 1), (4, 2))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 1))
        self.assertEqual(path[-1], (4, 2))
        self.assertAlmostEqual(path_cost(grid, path), 8.0 + SQRT2, places=9)

    def test_route_is_minimal_when_diagonals_help(self):
        grid = Grid(5, 7, walls={(1, 1), (4, 5)})
        path = find_path(grid, (0, 0), (4, 6))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 6))
        self.assertAlmostEqual(path_cost(grid, path), 4.0 + 3.0 * SQRT2, places=9)

    def test_blocked_corner_is_not_cut(self):
        grid = Grid(2, 2, walls={(1, 0)})
        self.assertEqual(find_path(grid, (0, 0), (1, 1)), [(0, 0), (0, 1), (1, 1)])

    def test_unreachable_and_blocked_goals_are_reported(self):
        sealed = Grid(3, 3, walls={(1, 0), (0, 1), (1, 1)})
        self.assertIsNone(find_path(sealed, (2, 2), (0, 0)))
        blocked = Grid(3, 3, walls={(1, 1)})
        with self.assertRaises(ValueError):
            find_path(blocked, (0, 0), (1, 1))

    def test_path_cost_equals_the_sum_of_its_step_costs(self):
        grid = Grid(6, 4, walls={(2, 1), (2, 2)}, terrain={(4, 3): 3.0})
        path = find_path(grid, (0, 0), (5, 3))
        self.assertIsNotNone(path)
        for first, second in zip(path, path[1:]):
            step = max(abs(first[0] - second[0]), abs(first[1] - second[1]))
            self.assertLessEqual(step, 1)
            self.assertFalse(grid.is_blocked(first))
            self.assertFalse(grid.is_blocked(second))
        expected = 0.0
        for first, second in zip(path, path[1:]):
            expected += grid.move_cost(first, second)
        self.assertAlmostEqual(path_cost(grid, path), expected, places=9)
        self.assertEqual(path_cost(grid, []), 0.0)
        self.assertIsNone(path_cost(grid, None))


class FieldOfViewTests(unittest.TestCase):
    def test_fov_covers_the_radius_and_stops_behind_walls(self):
        open_grid = Grid(11, 11)
        visible = compute_fov(open_grid, (5, 5), 3)
        for x in range(2, 9):
            for y in range(2, 9):
                if max(abs(x - 5), abs(y - 5)) <= 3:
                    self.assertIn((x, y), visible)
        self.assertNotIn((9, 5), visible)
        self.assertNotIn((5, 9), visible)

        walled = Grid(9, 9, walls={(3, 4)})
        seen = compute_fov(walled, (4, 4), 4)
        self.assertIn((3, 4), seen)
        self.assertNotIn((2, 4), seen)
        self.assertNotIn((1, 4), seen)
        self.assertNotIn((0, 4), seen)


if __name__ == "__main__":
    unittest.main()
