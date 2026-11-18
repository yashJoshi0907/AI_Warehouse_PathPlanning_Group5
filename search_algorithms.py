"""
search_algorithms.py
====================
Task 3 – BFS, DFS, Greedy Best-First Search, and A*
Author   : Yash Joshi (BT24CSD024) – BFS & DFS
           Yuvraj Singh (BT24CSD031) – Greedy Best-First Search & heuristic
           Harsh Patil  (BT24CSD032) – A* & weighted path costs
Project  : AI-Based Route Planning for a Warehouse Delivery Robot
"""

from __future__ import annotations
from collections import deque
import heapq
import time
from typing import List, Tuple, Optional, Dict

from warehouse_grid import WarehouseGrid


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

class SearchResult:
    """Holds the outcome of a single search run."""

    def __init__(self,
                 algorithm: str,
                 path: Optional[List[Tuple[int, int]]],
                 nodes_expanded: int,
                 path_cost: float,
                 elapsed_sec: float) -> None:
        self.algorithm      = algorithm
        self.path           = path          # None → no path found
        self.nodes_expanded = nodes_expanded
        self.path_cost      = path_cost
        self.path_length    = len(path) - 1 if path else 0  # number of moves
        self.elapsed_sec    = elapsed_sec
        self.goal_reached   = path is not None

    def __repr__(self) -> str:
        status = f"reached (len={self.path_length}, cost={self.path_cost:.1f})" \
                 if self.goal_reached else "FAILED"
        return (f"<SearchResult [{self.algorithm}] {status} "
                f"| nodes={self.nodes_expanded} | t={self.elapsed_sec:.4f}s>")


# ---------------------------------------------------------------------------
# Helper: reconstruct path from parent dict
# ---------------------------------------------------------------------------

def _reconstruct(came_from: Dict, goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Heuristic
# ---------------------------------------------------------------------------

def manhattan(state: Tuple[int, int], goal: Tuple[int, int]) -> float:
    """
    Manhattan distance heuristic.

    Admissibility proof (why it is a lower bound):
    ------------------------------------------------
    In a 4-connected grid the minimum number of moves to reach (gr, gc)
    from (r, c) is exactly |r − gr| + |c − gc|.  Every move enters one
    cell whose minimum entry cost is 1 (open aisle).  Therefore the true
    remaining cost g*(n) ≥ |Δr| + |Δc| = h(n), so h never over-estimates
    and the heuristic is admissible.  A* with an admissible heuristic is
    guaranteed to find the optimal (lowest-cost) path.
    """
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])


# ---------------------------------------------------------------------------
# 1. Breadth-First Search  (Yash Joshi – BT24CSD024)
# ---------------------------------------------------------------------------

def bfs(grid: WarehouseGrid) -> SearchResult:
    """
    BFS explores states level by level using a FIFO queue.
    It guarantees the path with the fewest moves (not lowest cost).
    """
    start = time.perf_counter()

    frontier: deque = deque()
    frontier.append(grid.start)
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {grid.start: None}
    nodes_expanded = 0

    while frontier:
        state = frontier.popleft()
        nodes_expanded += 1

        if grid.goal_test(state):
            path = _reconstruct(came_from, state)
            cost = grid.path_cost(path)
            return SearchResult('BFS', path, nodes_expanded, cost,
                                time.perf_counter() - start)

        for nr, nc, _ in grid.actions(state):
            neighbour = (nr, nc)
            if neighbour not in came_from:
                came_from[neighbour] = state
                frontier.append(neighbour)

    return SearchResult('BFS', None, nodes_expanded, float('inf'),
                        time.perf_counter() - start)


# ---------------------------------------------------------------------------
# 2. Depth-First Search  (Yash Joshi – BT24CSD024)
# ---------------------------------------------------------------------------

def dfs(grid: WarehouseGrid) -> SearchResult:
    """
    DFS explores states depth-first using a LIFO stack.
    May find a non-optimal (long or costly) path; not complete on infinite graphs
    but complete on finite acyclic grids when visited states are tracked.
    """
    start = time.perf_counter()

    frontier: List[Tuple[int, int]] = [grid.start]
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {grid.start: None}
    nodes_expanded = 0

    while frontier:
        state = frontier.pop()          # LIFO
        nodes_expanded += 1

        if grid.goal_test(state):
            path = _reconstruct(came_from, state)
            cost = grid.path_cost(path)
            return SearchResult('DFS', path, nodes_expanded, cost,
                                time.perf_counter() - start)

        for nr, nc, _ in grid.actions(state):
            neighbour = (nr, nc)
            if neighbour not in came_from:
                came_from[neighbour] = state
                frontier.append(neighbour)

    return SearchResult('DFS', None, nodes_expanded, float('inf'),
                        time.perf_counter() - start)


# ---------------------------------------------------------------------------
# 3. Greedy Best-First Search  (Yuvraj Singh – BT24CSD031)
# ---------------------------------------------------------------------------

def greedy(grid: WarehouseGrid) -> SearchResult:
    """
    Greedy Best-First Search orders the frontier by h(n) = Manhattan distance.
    Fast in practice but NOT optimal — it ignores accumulated cost g(n).

    Why Manhattan distance is a valid heuristic lower bound:
        See docstring of manhattan() above.
    """
    start_time = time.perf_counter()

    # Priority queue: (h, counter, state) — counter breaks ties deterministically
    counter = 0
    frontier: List = []
    heapq.heappush(frontier, (manhattan(grid.start, grid.goal), counter, grid.start))
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {grid.start: None}
    nodes_expanded = 0

    while frontier:
        _, _, state = heapq.heappop(frontier)
        nodes_expanded += 1

        if grid.goal_test(state):
            path = _reconstruct(came_from, state)
            cost = grid.path_cost(path)
            return SearchResult('Greedy', path, nodes_expanded, cost,
                                time.perf_counter() - start_time)

        for nr, nc, _ in grid.actions(state):
            neighbour = (nr, nc)
            if neighbour not in came_from:
                came_from[neighbour] = state
                h = manhattan(neighbour, grid.goal)
                counter += 1
                heapq.heappush(frontier, (h, counter, neighbour))

    return SearchResult('Greedy', None, nodes_expanded, float('inf'),
                        time.perf_counter() - start_time)


# ---------------------------------------------------------------------------
# 4. A* Search  (Harsh Patil – BT24CSD032)
# ---------------------------------------------------------------------------

def astar(grid: WarehouseGrid,
          start_override: Optional[Tuple[int, int]] = None) -> SearchResult:
    """
    A* orders the frontier by f(n) = g(n) + h(n).

    g(n) : accumulated real path cost (sum of cell entry costs so far).
    h(n) : admissible Manhattan-distance heuristic (see manhattan()).

    Optimality guarantee: because h is admissible (never over-estimates),
    A* expands nodes in non-decreasing order of f and returns the minimum-cost
    path when the goal is first dequeued.

    Weighted cells demo (Task 3 requirement):
        A cell with cost 4 ('C') may cause A* to prefer a longer (more moves)
        but cheaper route — illustrating that path length ≠ path cost.
    """
    start_time = time.perf_counter()
    src = start_override if start_override is not None else grid.start

    counter = 0
    # heap entry: (f, counter, state)
    frontier: List = []
    heapq.heappush(frontier, (manhattan(src, grid.goal), counter, src))

    g_cost: Dict[Tuple[int, int], float] = {src: 0.0}
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {src: None}
    nodes_expanded = 0

    while frontier:
        f, _, state = heapq.heappop(frontier)

        # Stale entry check
        if g_cost.get(state, float('inf')) < f - manhattan(state, grid.goal):
            continue

        nodes_expanded += 1

        if grid.goal_test(state):
            path = _reconstruct(came_from, state)
            cost = grid.path_cost(path)
            return SearchResult('A*', path, nodes_expanded, cost,
                                time.perf_counter() - start_time)

        for nr, nc, _ in grid.actions(state):
            neighbour = (nr, nc)
            new_g = g_cost[state] + grid.entry_cost(nr, nc)

            if new_g < g_cost.get(neighbour, float('inf')):
                g_cost[neighbour]    = new_g
                came_from[neighbour] = state
                h = manhattan(neighbour, grid.goal)
                counter += 1
                heapq.heappush(frontier, (new_g + h, counter, neighbour))

    return SearchResult('A*', None, nodes_expanded, float('inf'),
                        time.perf_counter() - start_time)


# ---------------------------------------------------------------------------
# Convenience: run all four algorithms on one grid
# ---------------------------------------------------------------------------

def run_all(grid: WarehouseGrid) -> List[SearchResult]:
    """Run BFS, DFS, Greedy, and A* on *grid*; return list of SearchResults."""
    return [
        bfs(grid),
        dfs(grid),
        greedy(grid),
        astar(grid),
    ]
