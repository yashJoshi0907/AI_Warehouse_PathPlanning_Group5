# Task 3 — Search Algorithms

**Assigned To:**
- Yash Joshi (BT24CSD024) — BFS & DFS
- Yuvraj Singh (BT24CSD031) — Greedy Best-First Search & heuristic explanation
- Harsh Patil (BT24CSD033) — A* implementation & weighted path costs

**Source File:** [`search_algorithms.py`](../search_algorithms.py)  
**Objective:** Implement four search algorithms — BFS, DFS, Greedy Best-First Search, and A* — on the same warehouse grid. Use Manhattan distance as the heuristic. Explain why it is a valid lower bound. Demonstrate that the shortest route is not always the cheapest.

---

## 3.1 Overview

Four classical search algorithms are implemented and compared on the same warehouse grid defined in Task 1. Each algorithm uses a different strategy for exploring the state space, leading to different paths, costs, and expansion counts.

All algorithms share a common structure:
1. Maintain a **frontier** (data structure holding states to explore next).
2. Maintain a **came_from** dictionary to reconstruct the path once the goal is found.
3. Expand states from the frontier, adding successors that have not been visited.
4. Return a `SearchResult` containing the path, cost, nodes expanded, and execution time.

---

## 3.2 Shared Infrastructure

### SearchResult Class

Every algorithm returns a `SearchResult` object:

```python
class SearchResult:
    algorithm: str           # Algorithm name
    path: Optional[List]     # Sequence of (r,c) from start to goal; None if no path
    nodes_expanded: int      # Number of states popped from the frontier
    path_cost: float         # Total entry cost of the path
    path_length: int         # Number of moves (edges traversed)
    elapsed_sec: float       # Wall-clock execution time
    goal_reached: bool       # True if a path was found
```

### Path Reconstruction

All algorithms use a shared `_reconstruct()` function that traces the `came_from` dictionary from goal back to start:

```python
def _reconstruct(came_from, goal):
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path
```

---

## 3.3 Algorithm 1: Breadth-First Search (BFS)

**Author:** Yash Joshi (BT24CSD024)

### Concept

BFS explores the state space **level by level** using a **FIFO queue** (First-In, First-Out). It dequeues the shallowest unexpanded node first, ensuring that all nodes at depth $d$ are expanded before any node at depth $d+1$.

### Properties

| Property | Value |
|----------|-------|
| **Data structure** | `collections.deque` (FIFO queue) |
| **Complete?** | ✅ Yes (finite state space) |
| **Optimal (moves)?** | ✅ Yes — guarantees fewest moves |
| **Optimal (cost)?** | ❌ No — ignores variable cell costs |
| **Time complexity** | $O(b^d)$ where $b$ = branching factor, $d$ = solution depth |
| **Space complexity** | $O(b^d)$ (all frontier + explored nodes stored) |

### Implementation

```python
def bfs(grid: WarehouseGrid) -> SearchResult:
    frontier = deque([grid.start])
    came_from = {grid.start: None}
    nodes_expanded = 0

    while frontier:
        state = frontier.popleft()       # FIFO: oldest first
        nodes_expanded += 1

        if grid.goal_test(state):
            path = _reconstruct(came_from, state)
            return SearchResult('BFS', path, nodes_expanded, grid.path_cost(path), ...)

        for nr, nc, _ in grid.actions(state):
            if (nr, nc) not in came_from:
                came_from[(nr, nc)] = state
                frontier.append((nr, nc))

    return SearchResult('BFS', None, nodes_expanded, inf, ...)
```

### Results on Default Grid

```
   0 1 2 3 4 5 6 7 8 9
 0 S . # . . . . C . .
 1 * . # . . # . . . .
 2 * . # . . # . C . .
 3 * . . . C . . . . .
 4 * # # # # # . . . .
 5 * * * * * * . . . .
 6 . . . . # * . . . .
 7 . C . . # * . C . .
 8 . . . . # * * * . .
 9 . . . . . . # * * G
```

| Metric | Value |
|--------|-------|
| Moves | **18** |
| Path Cost | **21.0** |
| Nodes Expanded | **86** |
| Time | 0.136 ms |

> [!NOTE]
> BFS found the **fewest-move** path (18 moves), but the cost is 21.0 — higher than A*'s optimal 20.0. This is because BFS's path passes through the crowded cell at $(5,2)$ (cost 4), inflating the total by 3 compared to the route that avoids it.

---

## 3.4 Algorithm 2: Depth-First Search (DFS)

**Author:** Yash Joshi (BT24CSD024)

### Concept

DFS explores the state space **depth-first** using a **LIFO stack** (Last-In, First-Out). It pushes successors onto a stack and always expands the most recently discovered node, diving deep before backtracking.

### Properties

| Property | Value |
|----------|-------|
| **Data structure** | Python `list` used as a stack (`pop()` = LIFO) |
| **Complete?** | ✅ Yes (with visited-state tracking on finite graphs) |
| **Optimal (moves)?** | ❌ No |
| **Optimal (cost)?** | ❌ No |
| **Time complexity** | $O(b^m)$ where $m$ = maximum depth |
| **Space complexity** | $O(bm)$ (stack depth) |

### Implementation

```python
def dfs(grid: WarehouseGrid) -> SearchResult:
    frontier = [grid.start]              # List used as LIFO stack
    came_from = {grid.start: None}
    nodes_expanded = 0

    while frontier:
        state = frontier.pop()           # LIFO: newest first
        nodes_expanded += 1

        if grid.goal_test(state):
            path = _reconstruct(came_from, state)
            return SearchResult('DFS', path, nodes_expanded, grid.path_cost(path), ...)

        for nr, nc, _ in grid.actions(state):
            if (nr, nc) not in came_from:
                came_from[(nr, nc)] = state
                frontier.append((nr, nc))

    return SearchResult('DFS', None, nodes_expanded, inf, ...)
```

### Results on Default Grid

```
   0 1 2 3 4 5 6 7 8 9
 0 S * # . . . . C . .
 1 . * # . . # . . . .
 2 * * # . . # . C . .
 3 * . . . C . . . . .
 4 * # # # # # . . . .
 5 * * * * * * * * * *
 6 . . . . # . . . . *
 7 . C . . # . . * * *
 8 . . . . # . . * . .
 9 . . . . . . # * * G
```

| Metric | Value |
|--------|-------|
| Moves | **24** |
| Path Cost | **30.0** |
| Nodes Expanded | **43** |
| Time | 0.063 ms |

> [!WARNING]
> DFS found a valid path but it is highly suboptimal: 24 moves (vs. BFS's 18) and cost 30.0 (vs. A*'s 20.0). The erratic, winding path is characteristic of DFS — it dives deep along one branch before backtracking, often missing shorter or cheaper alternatives.

---

## 3.5 Algorithm 3: Greedy Best-First Search

**Author:** Yuvraj Singh (BT24CSD031)

### Concept

Greedy Best-First Search uses a **priority queue** ordered solely by the heuristic value $h(n)$. It always expands the node that *appears* closest to the goal, aggressively moving toward the target. It does **not** consider the accumulated path cost $g(n)$.

### Properties

| Property | Value |
|----------|-------|
| **Data structure** | `heapq` min-heap, ordered by $(h(n), \text{counter}, \text{state})$ |
| **Priority function** | $h(n) = \text{Manhattan}(n, \text{goal})$ |
| **Complete?** | ✅ Yes (with visited-state tracking) |
| **Optimal?** | ❌ No — ignores accumulated cost |
| **Time complexity** | $O(b^m)$ worst case, often much better in practice |
| **Space complexity** | $O(b^m)$ |

### Implementation

```python
def greedy(grid: WarehouseGrid) -> SearchResult:
    counter = 0
    frontier = []
    heapq.heappush(frontier, (manhattan(grid.start, grid.goal), counter, grid.start))
    came_from = {grid.start: None}
    nodes_expanded = 0

    while frontier:
        _, _, state = heapq.heappop(frontier)
        nodes_expanded += 1

        if grid.goal_test(state):
            ...  # reconstruct and return

        for nr, nc, _ in grid.actions(state):
            if (nr, nc) not in came_from:
                came_from[(nr, nc)] = state
                h = manhattan((nr, nc), grid.goal)
                counter += 1
                heapq.heappush(frontier, (h, counter, (nr, nc)))
```

### Results on Default Grid

```
   0 1 2 3 4 5 6 7 8 9
 0 S . # . . . . C . .
 1 * . # . . # . . . .
 2 * . # . . # . C . .
 3 * . . . C . . . . .
 4 * # # # # # . . . .
 5 * . C . . . . . . .
 6 * . . . # . . . . .
 7 * C . . # . . C . .
 8 * . . . # * * * . .
 9 * * * * * * # * * G
```

| Metric | Value |
|--------|-------|
| Moves | **20** |
| Path Cost | **20.0** |
| Nodes Expanded | **21** |
| Time | 0.047 ms |

> [!TIP]
> Greedy expanded the fewest nodes (21) of all four algorithms — its aggressive heuristic-guided expansion cuts straight toward the goal. On this grid, Greedy happens to find the same optimal-cost path as A*, but this is **not guaranteed** in general. On different grid layouts, Greedy can return suboptimal solutions.

---

## 3.6 Algorithm 4: A* Search

**Author:** Harsh Patil (BT24CSD033)

### Concept

A* orders the frontier by the evaluation function:

$$f(n) = g(n) + h(n)$$

where:
- $g(n)$ = **accumulated real path cost** from start to node $n$ (sum of cell entry costs).
- $h(n)$ = **heuristic estimate** of the remaining cost from $n$ to the goal.

A* expands nodes in non-decreasing order of $f$ and returns the **minimum-cost path** when the goal is first dequeued — provided $h$ is **admissible** (never overestimates).

### Properties

| Property | Value |
|----------|-------|
| **Data structure** | `heapq` min-heap, ordered by $(f(n), \text{counter}, \text{state})$ |
| **Priority function** | $f(n) = g(n) + h(n)$ |
| **Complete?** | ✅ Yes |
| **Optimal?** | ✅ Yes (with admissible heuristic) |
| **Time complexity** | $O(b^d)$ but typically much better due to heuristic pruning |
| **Space complexity** | $O(b^d)$ |

### Implementation

```python
def astar(grid: WarehouseGrid, start_override=None) -> SearchResult:
    src = start_override or grid.start
    counter = 0
    frontier = []
    heapq.heappush(frontier, (manhattan(src, grid.goal), counter, src))

    g_cost = {src: 0.0}
    came_from = {src: None}
    nodes_expanded = 0

    while frontier:
        f, _, state = heapq.heappop(frontier)
        nodes_expanded += 1

        if grid.goal_test(state):
            ...  # reconstruct and return

        for nr, nc, _ in grid.actions(state):
            new_g = g_cost[state] + grid.entry_cost(nr, nc)
            if new_g < g_cost.get((nr, nc), inf):
                g_cost[(nr, nc)] = new_g
                came_from[(nr, nc)] = state
                h = manhattan((nr, nc), grid.goal)
                counter += 1
                heapq.heappush(frontier, (new_g + h, counter, (nr, nc)))
```

> [!IMPORTANT]
> A* uses a `start_override` parameter, allowing replanning from any position (used by Task 4 for trolley replanning). The `g_cost` dictionary tracks the **best known cost** to each node and is updated whenever a cheaper path is found, ensuring optimality.

### Results on Default Grid

```
   0 1 2 3 4 5 6 7 8 9
 0 S . # . . . . C . .
 1 * . # . . # . . . .
 2 * . # . . # . C . .
 3 * . . . C . . . . .
 4 * # # # # # . . . .
 5 * . C . . . . . . .
 6 * . . . # . . . . .
 7 * C . . # . . C . .
 8 * . . . # * * * . .
 9 * * * * * * # * * G
```

| Metric | Value |
|--------|-------|
| Moves | **20** |
| Path Cost | **20.0** ✓ (optimal) |
| Nodes Expanded | **51** |
| Time | 0.161 ms |

---

## 3.7 Manhattan Distance Heuristic — Admissibility Proof

### Definition

For a state $n = (r, c)$ and goal $G = (r_G, c_G)$:

$$h(n) = |r - r_G| + |c - c_G|$$

### Proof that $h$ is a lower bound (admissible)

**Claim:** $h(n) \le h^*(n)$ for all states $n$, where $h^*(n)$ is the true optimal cost from $n$ to $G$.

**Proof:**

1. In a 4-connected grid, the **minimum number of moves** to travel from $(r, c)$ to $(r_G, c_G)$ is exactly $|r - r_G| + |c - c_G|$ — the Manhattan distance. This is because each move changes either the row or the column by exactly 1, and reaching the goal requires at least $|\Delta r|$ vertical moves and $|\Delta c|$ horizontal moves.

2. Each move enters one cell, and the **minimum possible entry cost** for any traversable cell in the grid is $c_{\min} = 1$ (open aisle).

3. Therefore, the true remaining cost satisfies:

$$h^*(n) \ge c_{\min} \times (|r - r_G| + |c - c_G|) = 1 \times h(n) = h(n)$$

4. Since $h(n) \le h^*(n)$ for all $n$, the heuristic **never overestimates** and is therefore **admissible**. $\square$

### Consistency (Monotonicity)

The Manhattan heuristic is also **consistent**:

$$h(n) \le c(n, a, n') + h(n')$$

For any single orthogonal step, $|h(n) - h(n')| \le 1$, and $c(n, a, n') \ge 1$. Therefore, $h(n) - h(n') \le 1 \le c(n, a, n')$, confirming consistency. This means A* with graph search never re-expands a node.

---

## 3.8 Cost ≠ Moves Demonstration

The assignment requires testing "a short, crowded route against a longer, less costly route." This is demonstrated using a specialised 2-row grid:

```
    0 1 2 3 4 5 6 7 8 9 10
 0  S . C C C C . . . . G
 1  . . . . . . . . . . .
```

**BFS result** (fewest moves):
- Path: $(0,0) \to (0,1) \to (0,2) \to \ldots \to (0,10)$ — straight across row 0
- Moves: **10**
- Cost: **22.0** (passes through 4 crowded cells at cost 4 each: $6 \times 1 + 4 \times 4 = 22$)

**A* result** (lowest cost):
- Path: $(0,0) \to (0,1) \to (1,1) \to (1,2) \to \ldots \to (1,6) \to (0,6) \to \ldots \to (0,10)$ — detours via row 1
- Moves: **12** (2 extra moves)
- Cost: **12.0** (all cells entered are open aisles at cost 1)

> [!IMPORTANT]
> **Conclusion:** A* found a route that takes 2 more moves but costs 10 less than BFS's path. This proves that **the shortest route (fewest moves) is not always the cheapest (lowest total cost)** when cell traversal costs are non-uniform.

---

## 3.9 Summary Comparison Table

| Algorithm | Goal? | Moves | Path Cost | Nodes Expanded | Time (ms) |
|-----------|:-----:|:-----:|:---------:|:--------------:|:---------:|
| **BFS** | ✅ | 18 | 21.0 | 86 | 0.136 |
| **DFS** | ✅ | 24 | 30.0 | 43 | 0.063 |
| **Greedy** | ✅ | 20 | 20.0 | 21 | 0.047 |
| **A\*** | ✅ | 20 | **20.0** ✓ | 51 | 0.161 |

### Key Findings

- **Fewest moves:** BFS (18 moves) — guaranteed by FIFO level-by-level exploration.
- **Lowest cost:** Greedy and A* (20.0) — A* is guaranteed optimal; Greedy was lucky on this grid.
- **Fewest nodes expanded:** Greedy (21) — aggressively heuristic-guided but not generally optimal.
- **Most nodes expanded:** BFS (86) — exhaustively explores all reachable cells at each depth level.
- **Shortest-move path ≠ lowest-cost path:** BFS's 18-move path costs 21.0 (crosses a crowded cell), while A*'s 20-move path costs 20.0 (avoids it). This confirms the assignment's hypothesis.

---

## 3.10 Unit Tests

The following tests in [`tests.py`](../tests.py) verify the search algorithms:

| Test Class | Test | Verification |
|------------|------|-------------|
| `TestManhattan` | `test_zero` | $h(n) = 0$ when $n = \text{goal}$. |
| `TestManhattan` | `test_simple` | Manhattan$(0,0) \to (3,4) = 7$. |
| `TestManhattan` | `test_symmetry` | $h(a, b) = h(b, a)$. |
| `TestBFS` | `test_simple_reaches_goal` | BFS finds a path from S to G on a 3×3 grid. |
| `TestBFS` | `test_blocked_no_path` | BFS correctly reports no path on a fully blocked grid. |
| `TestBFS` | `test_path_is_connected` | Every consecutive pair of cells in the path differs by exactly 1 step. |
| `TestBFS` | `test_bfs_optimal_moves` | BFS path length = Manhattan distance (4 moves on 3×3). |
| `TestDFS` | `test_simple_reaches_goal` | DFS finds a path from S to G. |
| `TestDFS` | `test_blocked_no_path` | DFS reports no path when blocked. |
| `TestDFS` | `test_path_is_connected` | DFS path is step-connected. |
| `TestGreedy` | `test_simple_reaches_goal` | Greedy finds a path. |
| `TestGreedy` | `test_blocked_no_path` | Greedy reports no path when blocked. |
| `TestAStar` | `test_simple_reaches_goal` | A* finds a path with correct start/goal endpoints. |
| `TestAStar` | `test_blocked_no_path` | A* reports no path when blocked. |
| `TestAStar` | `test_optimal_cost` | A* cost $\le$ BFS cost (optimality guarantee). |
| `TestAStar` | `test_astar_beats_or_ties_greedy_cost` | A* cost $\le$ Greedy cost. |
| `TestAStar` | `test_detour_cheaper` | A* finds the cheap detour (cost $< 18$) around CCCC cells. |
| `TestAStar` | `test_start_override` | A* works correctly with a non-default start position (used for replanning). |
| `TestRunAll` | `test_returns_four_results` | `run_all()` returns exactly 4 SearchResults with correct names. |
| `TestRunAll` | `test_all_reach_goal_on_default_grid` | All 4 algorithms find a path on the default grid. |

---

## 3.11 Visual Output

The module generates a 2×2 comparison figure saved as [`screenshots/algorithm_routes.png`](../screenshots/algorithm_routes.png), showing each algorithm's route overlaid on the warehouse grid with per-algorithm metrics.
