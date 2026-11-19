"""
tests.py
========
Unit & Integration Tests
Author   : Sayan Majhi (BT24CSD028)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

Run with:
    python -m unittest tests.py -v
  or:
    python -m pytest tests.py -v
  or:
    python tests.py
"""

from __future__ import annotations
import unittest

from warehouse_grid import (
    WarehouseGrid, make_default_grid, DEFAULT_LAYOUT,
    COST_TEST_LAYOUT, BOTTLENECK_LAYOUT,
    OPEN, CROWDED, SHELF, RESTRICTED, START, GOAL, TROLLEY,
    CELL_COST, IMPASSABLE
)
from search_algorithms import bfs, dfs, greedy, astar, run_all, manhattan, ALGORITHMS
from trolley_replanning import TrolleySimulator
from metrics import benchmark


# ---------------------------------------------------------------------------
# Minimal test grids
# ---------------------------------------------------------------------------

SIMPLE_LAYOUT = [
    "S..",
    "...",
    "..G",
]

BLOCKED_LAYOUT = [
    "S##",
    "###",
    "##G",
]

CROWDED_LAYOUT = [
    "S.G",
    ".C.",
    "...",
]

# A grid where the direct path is costly (CCCC) and a detour is cheaper
DETOUR_LAYOUT = [
    "S.CCCCG",
    ".......",
]

# A narrow map with a restricted zone guarding one of two approaches
RESTRICTED_LAYOUT = [
    "S..R.",
    "###R.",
    "...R.",
    "###.#",
    "....G",
]


# ===========================================================================
# Tests
# ===========================================================================

class TestWarehouseGrid(unittest.TestCase):

    def setUp(self):
        self.grid = WarehouseGrid(SIMPLE_LAYOUT)

    # --- basic attributes ---

    def test_dimensions(self):
        self.assertEqual(self.grid.rows, 3)
        self.assertEqual(self.grid.cols, 3)

    def test_start_goal_detection(self):
        self.assertEqual(self.grid.start, (0, 0))
        self.assertEqual(self.grid.goal,  (2, 2))

    def test_in_bounds(self):
        self.assertTrue(self.grid.in_bounds(0, 0))
        self.assertFalse(self.grid.in_bounds(-1, 0))
        self.assertFalse(self.grid.in_bounds(3, 3))

    def test_is_passable(self):
        self.assertTrue(self.grid.is_passable(0, 0))   # S
        self.assertTrue(self.grid.is_passable(0, 1))   # .

    def test_impassable_shelf(self):
        shelf_grid = WarehouseGrid(BLOCKED_LAYOUT)
        # Only (0,0) and (2,2) are passable
        self.assertTrue(shelf_grid.is_passable(0, 0))
        self.assertFalse(shelf_grid.is_passable(0, 1))

    def test_entry_cost_open(self):
        self.assertEqual(self.grid.entry_cost(0, 1), 1)   # '.'

    def test_entry_cost_crowded(self):
        g = WarehouseGrid(CROWDED_LAYOUT)
        self.assertEqual(g.entry_cost(1, 1), 4)           # 'C'

    def test_path_cost_simple(self):
        path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)]
        cost = self.grid.path_cost(path)
        self.assertEqual(cost, 4)  # 4 cells entered (skip start)

    def test_path_cost_empty(self):
        self.assertEqual(self.grid.path_cost([]), 0.0)
        self.assertEqual(self.grid.path_cost([(0, 0)]), 0.0)

    def test_actions_centre(self):
        acts = self.grid.actions((1, 1))
        neighbours = {(nr, nc) for nr, nc, _ in acts}
        self.assertEqual(neighbours, {(0, 1), (2, 1), (1, 0), (1, 2)})

    def test_actions_corner(self):
        acts = self.grid.actions((0, 0))
        neighbours = {(nr, nc) for nr, nc, _ in acts}
        self.assertIn((0, 1), neighbours)
        self.assertIn((1, 0), neighbours)
        self.assertNotIn((-1, 0), neighbours)

    def test_goal_test(self):
        self.assertTrue(self.grid.goal_test((2, 2)))
        self.assertFalse(self.grid.goal_test((0, 0)))

    def test_mark_impassable(self):
        self.grid.mark_impassable(0, 1)
        self.assertFalse(self.grid.is_passable(0, 1))

    def test_clone_independence(self):
        clone = self.grid.clone()
        clone.mark_impassable(0, 1)
        self.assertTrue(self.grid.is_passable(0, 1))


class TestManhattan(unittest.TestCase):

    def test_zero(self):
        self.assertEqual(manhattan((3, 4), (3, 4)), 0)

    def test_simple(self):
        self.assertEqual(manhattan((0, 0), (3, 4)), 7)

    def test_symmetry(self):
        self.assertEqual(manhattan((2, 5), (7, 1)), manhattan((7, 1), (2, 5)))


class TestBFS(unittest.TestCase):

    def test_simple_reaches_goal(self):
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = bfs(g)
        self.assertTrue(r.goal_reached)
        self.assertEqual(r.path[0], g.start)
        self.assertEqual(r.path[-1], g.goal)

    def test_blocked_no_path(self):
        g = WarehouseGrid(BLOCKED_LAYOUT)
        r = bfs(g)
        self.assertFalse(r.goal_reached)

    def test_path_is_connected(self):
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = bfs(g)
        path = r.path
        for i in range(len(path) - 1):
            dr = abs(path[i+1][0] - path[i][0])
            dc = abs(path[i+1][1] - path[i][1])
            self.assertEqual(dr + dc, 1, msg=f"Disconnected step {path[i]}→{path[i+1]}")

    def test_bfs_optimal_moves(self):
        # BFS guarantees fewest moves
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = bfs(g)
        # Manhattan distance from (0,0) to (2,2) = 4 moves minimum
        self.assertEqual(r.path_length, 4)


class TestDFS(unittest.TestCase):

    def test_simple_reaches_goal(self):
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = dfs(g)
        self.assertTrue(r.goal_reached)

    def test_blocked_no_path(self):
        g = WarehouseGrid(BLOCKED_LAYOUT)
        r = dfs(g)
        self.assertFalse(r.goal_reached)

    def test_path_is_connected(self):
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = dfs(g)
        path = r.path
        for i in range(len(path) - 1):
            dr = abs(path[i+1][0] - path[i][0])
            dc = abs(path[i+1][1] - path[i][1])
            self.assertEqual(dr + dc, 1)


class TestGreedy(unittest.TestCase):

    def test_simple_reaches_goal(self):
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = greedy(g)
        self.assertTrue(r.goal_reached)

    def test_blocked_no_path(self):
        g = WarehouseGrid(BLOCKED_LAYOUT)
        r = greedy(g)
        self.assertFalse(r.goal_reached)


class TestAStar(unittest.TestCase):

    def test_simple_reaches_goal(self):
        g = WarehouseGrid(SIMPLE_LAYOUT)
        r = astar(g)
        self.assertTrue(r.goal_reached)
        self.assertEqual(r.path[0], g.start)
        self.assertEqual(r.path[-1], g.goal)

    def test_blocked_no_path(self):
        g = WarehouseGrid(BLOCKED_LAYOUT)
        r = astar(g)
        self.assertFalse(r.goal_reached)

    def test_optimal_cost(self):
        """A* must return minimum-cost path; BFS path cost >= A* path cost."""
        g = make_default_grid()
        astar_r = astar(g)
        bfs_r   = bfs(g)
        self.assertLessEqual(astar_r.path_cost, bfs_r.path_cost)

    def test_astar_beats_or_ties_greedy_cost(self):
        g = make_default_grid()
        astar_r  = astar(g)
        greedy_r = greedy(g)
        if greedy_r.goal_reached:
            self.assertLessEqual(astar_r.path_cost, greedy_r.path_cost)

    def test_detour_cheaper(self):
        """A* should find the cheaper detour around crowded cells."""
        g = WarehouseGrid(DETOUR_LAYOUT)
        r = astar(g)
        self.assertTrue(r.goal_reached)
        # Direct path through CCCC would cost 4*4=16 + 2 = 18 total; detour ≤ 6
        self.assertLess(r.path_cost, 18)

    def test_start_override(self):
        g = make_default_grid()
        # Start from a mid-point
        mid = (3, 3)
        r = astar(g, start_override=mid)
        # Path should start at mid, end at goal
        self.assertEqual(r.path[0], mid)
        self.assertEqual(r.path[-1], g.goal)


class TestTrolleySimulator(unittest.TestCase):

    def test_simulation_runs(self):
        g = make_default_grid()
        sim = TrolleySimulator(g)
        result = sim.run()
        # Must complete without exception
        self.assertIsNotNone(result)

    def test_replan_count_positive(self):
        g = make_default_grid()
        sim = TrolleySimulator(g)
        result = sim.run()
        # At least one trolley was placed → at least 1 replan
        if sim.trolley_positions:
            self.assertGreaterEqual(result.replan_count, 1)

    def test_goal_reachable_without_trolley(self):
        """Without any trolleys the robot should always reach the goal."""
        g = make_default_grid()
        sim = TrolleySimulator(g, trolley_positions=[])
        result = sim.run()
        self.assertTrue(result.goal_reached)

    def test_custom_trolley_position(self):
        g = make_default_grid()
        # Find a valid cell on the initial path
        init_result = astar(g)
        if len(init_result.path) > 2:
            tpos = [init_result.path[2]]
            sim  = TrolleySimulator(g, trolley_positions=tpos)
            res  = sim.run()
            self.assertEqual(res.replan_count, 1)

    def test_trace_starts_at_start(self):
        g = make_default_grid()
        sim = TrolleySimulator(g)
        result = sim.run()
        self.assertEqual(result.final_trace[0], g.start)


class TestRunAll(unittest.TestCase):

    def test_returns_four_results(self):
        g = make_default_grid()
        results = run_all(g)
        self.assertEqual(len(results), 4)
        names = [r.algorithm for r in results]
        self.assertIn('BFS', names)
        self.assertIn('DFS', names)
        self.assertIn('Greedy', names)
        self.assertIn('A*', names)

    def test_all_reach_goal_on_default_grid(self):
        g = make_default_grid()
        for r in run_all(g):
            self.assertTrue(r.goal_reached, msg=f"{r.algorithm} failed to find path")


class TestLayoutValidation(unittest.TestCase):

    def test_ragged_layout_rejected(self):
        with self.assertRaises(ValueError):
            WarehouseGrid(["S..", "..G.."])

    def test_empty_layout_rejected(self):
        with self.assertRaises(ValueError):
            WarehouseGrid([])

    def test_missing_goal_rejected(self):
        with self.assertRaises(ValueError):
            WarehouseGrid(["S..", "..."])


class TestDefaultEnvironment(unittest.TestCase):

    def test_default_grid_uses_every_cell_type(self):
        g = make_default_grid()
        self.assertGreaterEqual(g.count(OPEN), 1)
        self.assertGreaterEqual(g.count(CROWDED), 1)
        self.assertGreaterEqual(g.count(SHELF), 1)
        self.assertGreaterEqual(g.count(RESTRICTED), 1)

    def test_default_grid_is_solvable(self):
        self.assertTrue(astar(make_default_grid()).goal_reached)

    def test_restricted_zone_is_impassable(self):
        g = WarehouseGrid(RESTRICTED_LAYOUT)
        self.assertFalse(g.is_passable(0, 3))
        self.assertFalse(g.is_passable(1, 3))
        self.assertTrue(g.is_passable(0, 2))


class TestCostVsMoves(unittest.TestCase):

    def test_bfs_fewest_moves_but_higher_cost(self):
        g = WarehouseGrid(COST_TEST_LAYOUT)
        b = bfs(g)
        self.assertEqual(b.path_length, 10)
        self.assertEqual(b.path_cost, 22.0)

    def test_astar_takes_detour_to_save_cost(self):
        g = WarehouseGrid(COST_TEST_LAYOUT)
        a = astar(g)
        self.assertEqual(a.path_length, 12)
        self.assertEqual(a.path_cost, 12.0)
        self.assertLess(a.path_cost, bfs(g).path_cost)


class TestNoAlternativeRoute(unittest.TestCase):

    def test_replan_reports_unreachable(self):
        sim = TrolleySimulator(WarehouseGrid(BOTTLENECK_LAYOUT))
        result = sim.run()
        self.assertEqual(result.replan_count, 1)
        self.assertFalse(result.goal_reached)
        self.assertIsNone(result.replan_events[0].new_path)

    def test_robot_never_enters_the_blocked_cell(self):
        sim = TrolleySimulator(WarehouseGrid(BOTTLENECK_LAYOUT))
        result = sim.run()
        self.assertTrue(sim.trolley_positions)
        self.assertNotIn(sim.trolley_positions[0], result.final_trace)


class TestBenchmark(unittest.TestCase):

    def test_benchmark_covers_all_algorithms(self):
        results = benchmark(make_default_grid(), repeats=2)
        self.assertEqual([r.algorithm for r in results], list(ALGORITHMS))

    def test_benchmark_keeps_deterministic_metrics(self):
        g = make_default_grid()
        for res in benchmark(g, repeats=2):
            single = ALGORITHMS[res.algorithm](g)
            self.assertEqual(res.path_length, single.path_length)
            self.assertEqual(res.path_cost, single.path_cost)
            self.assertEqual(res.nodes_expanded, single.nodes_expanded)
            self.assertGreater(res.elapsed_sec, 0.0)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
