# Task 5 — Performance Evaluation, Visualization & Documentation

**Assigned To:** Sayan Majhi (BT24CSD028)  
**Source Files:** [`metrics.py`](../metrics.py), [`visualization.py`](../visualization.py), [`tests.py`](../tests.py), [`gui.py`](../gui.py)  
**Objective:** Compare all four search algorithms on goal reached, path length (moves), total path cost, nodes expanded, and execution time. For the trolley experiment, record the number of replans. Produce labeled grid visualizations, a comparison table, and a brief explanation of whether the shortest route also had the lowest cost.

---

## 5.1 Overview

This task serves as the **evaluation and presentation layer** for the entire project. It encompasses:

1. **Performance metrics collection** — systematically recording quantitative measures for each algorithm.
2. **Comparison table** — formatted side-by-side display of algorithm performance.
3. **Findings analysis** — automated identification of best performers and cost-vs-moves analysis.
4. **Visualization** — high-resolution Matplotlib plots of grid routes, trolley replanning sequences, and performance bar charts.
5. **Interactive GUI** — a Tkinter-based graphical application with real-time animation.
6. **Testing** — comprehensive unit and integration test suite validating all components.

---

## 5.2 Metrics Collected

For each search algorithm, the following metrics are recorded:

| Metric | Description | How Measured |
|--------|-------------|-------------|
| **Goal Reached** | Whether the algorithm found a valid path from $S$ to $G$. | `SearchResult.goal_reached` |
| **Path Length (Moves)** | Number of edges traversed (steps taken). | `len(path) - 1` |
| **Total Path Cost** | Sum of entry costs for all cells entered (excluding start). | `grid.path_cost(path)` |
| **Nodes Expanded** | Number of states popped from the frontier. | Counter incremented on each `pop()`. |
| **Execution Time** | Wall-clock time in milliseconds. | `time.perf_counter()` delta. |
| **Number of Replans** | (Trolley experiment only) How many times A* was re-invoked due to obstacle detection. | `SimulationResult.replan_count` |

---

## 5.3 Performance Comparison Table

### Implementation

The `print_comparison_table()` function in [`metrics.py`](../metrics.py) renders a formatted ASCII table:

```python
def print_comparison_table(results: List[SearchResult], title: str = "..."):
    # Prints header, separator, one row per algorithm, separator, and findings
```

### Actual Output

```
══════════════════════════════════════════════════════════════════
  Algorithm Performance Comparison
══════════════════════════════════════════════════════════════════
Algorithm | Goal?   | Moves   | Path Cost  | Nodes    | Time(ms)
─────────────────────────────────────────────────────────────────────
BFS      | Yes     | 18      | 21.0       | 86       | 0.136
DFS      | Yes     | 24      | 30.0       | 43       | 0.063
Greedy   | Yes     | 20      | 20.0       | 21       | 0.047
A*       | Yes     | 20      | 20.0       | 51       | 0.161
─────────────────────────────────────────────────────────────────────

  Key findings:
    • Fewest moves   : BFS (18 moves)
    • Lowest cost    : Greedy, A* (cost 20.0)
    • Fewest nodes   : Greedy (21 nodes expanded)
    • → Shortest-move path ≠ lowest-cost path: crowded cells inflate cost
        along the fewest-move route.
```

### Results in Tabular Form

| Algorithm | Goal? | Moves | Path Cost | Nodes Expanded | Time (ms) |
|:---------:|:-----:|:-----:|:---------:|:--------------:|:---------:|
| **BFS** | ✅ Yes | 18 | 21.0 | 86 | 0.136 |
| **DFS** | ✅ Yes | 24 | 30.0 | 43 | 0.063 |
| **Greedy** | ✅ Yes | 20 | 20.0 | 21 | 0.047 |
| **A\*** | ✅ Yes | 20 | **20.0** | 51 | 0.161 |

---

## 5.4 Trolley Replanning Metrics

```
═══════════════════════════════════════════════════════
  TROLLEY REPLANNING METRICS
═══════════════════════════════════════════════════════
  Metric                         Value
───────────────────────────────────────────────────────
  Trolley position(s)            [(3, 0)]
  Initial path cost              20.0
  Initial path length            20 moves
  Number of replans              1
  Goal reached                   Yes ✓
    Replan #1 cost               19.0
    Replan #1 moves              16
    Replan #1 nodes              37
═══════════════════════════════════════════════════════
```

---

## 5.5 Key Findings & Analysis

### Finding 1: Shortest Route ≠ Lowest Cost

> **Does the shortest route also have the lowest cost?**
>
> **No.** On the default 10×10 grid:
> - **BFS** found the shortest path: **18 moves** with cost **21.0**.
> - **A\*** found the cheapest path: **20 moves** with cost **20.0**.
>
> BFS's 18-move path traverses a crowded cell (`C` at $(5,2)$ with entry cost 4), while A*'s 20-move path avoids all crowded cells entirely. The 2 extra moves save 1.0 in total cost.

This finding is further reinforced by the **Cost ≠ Moves Demo** on the 2-row corridor grid:

| Algorithm | Moves | Cost | Route Strategy |
|-----------|:-----:|:----:|----------------|
| BFS | 10 | 22.0 | Direct path through 4 crowded cells (cost 4 each) |
| A* | 12 | 12.0 | Detour via row 1, avoiding all crowded cells |

The 2 extra moves for A* save **10.0** in cost — a dramatic demonstration of the difference.

### Finding 2: Algorithm Efficiency Trade-offs

| Criterion | Best Algorithm | Explanation |
|-----------|---------------|-------------|
| **Cost optimality** | A* | Guaranteed by admissible heuristic ($f = g + h$). |
| **Move optimality** | BFS | FIFO exploration guarantees minimum depth. |
| **Fewest nodes expanded** | Greedy (21) | Aggressively follows heuristic, little exploration. |
| **Fastest execution** | Greedy (0.047 ms) | Fewest nodes expanded → fewest operations. |
| **Highest cost** | DFS (30.0) | Erratic deep-first exploration yields poor paths. |

### Finding 3: Replanning Resilience

The dynamic trolley experiment demonstrated that:
- The robot's one-step lookahead sensor successfully detected the obstacle before entering the blocked cell.
- A* replanning from the current position computed a viable bypass in under 1 ms.
- The replanned path (cost 19.0, 16 remaining moves) was actually *cheaper* than the remaining portion of the original plan would have been (cost 18.0 for 18 remaining moves, but that path was now blocked).
- The system gracefully handles the "no path" edge case when all routes are blocked.

---

## 5.6 Visualization Engine

### Implementation

The visualization module ([`visualization.py`](../visualization.py)) uses Matplotlib with the `Agg` (headless) backend to generate publication-quality figures. Key functions:

| Function | Output File | Description |
|----------|-------------|-------------|
| `save_grid_overview()` | `screenshots/grid_overview.png` | Colour-coded grid with cell type legend. |
| `save_algorithm_routes()` | `screenshots/algorithm_routes.png` | 2×2 subplot: each algorithm's route overlaid on the grid with metrics in the title. |
| `save_trolley_demo()` | `screenshots/trolley_replanning.png` | Multi-panel: initial plan → detection → replanned path → full trace. |
| `save_performance_chart()` | `screenshots/performance_chart.png` | 3-panel bar charts: Nodes Expanded, Cost vs. Moves, Execution Time. |

### Colour Scheme

| Cell Type | Colour | Hex Code |
|-----------|--------|----------|
| Open aisle | White | `#FFFFFF` |
| Crowded cell | Amber | `#FFD580` |
| Shelf | Dark slate | `#4A4A6A` |
| Restricted | Red | `#C0392B` |
| Start | Green | `#27AE60` |
| Goal | Blue | `#2980B9` |
| Trolley | Bright red | `#E74C3C` |
| Path overlay | Purple | `#8E44AD` |
| Replan path | Orange | `#E67E22` |
| Robot trace | Teal | `#16A085` |

### Generated Screenshots

1. **Grid Overview** — [`screenshots/grid_overview.png`](../screenshots/grid_overview.png)
   - Full warehouse floor plan with colour-coded cell types and legend.

2. **Algorithm Routes** — [`screenshots/algorithm_routes.png`](../screenshots/algorithm_routes.png)
   - 2×2 comparison showing BFS, DFS, Greedy, and A* routes with per-algorithm stats.

3. **Trolley Replanning** — [`screenshots/trolley_replanning.png`](../screenshots/trolley_replanning.png)
   - Multi-panel sequence: initial plan with hidden trolley marked → detection point → replanned bypass → full trace.

4. **Performance Chart** — [`screenshots/performance_chart.png`](../screenshots/performance_chart.png)
   - Three bar charts comparing Nodes Expanded, Path Cost vs. Move Count, and Execution Time across all four algorithms.

---

## 5.7 Interactive GUI Application

### Implementation

The [`gui.py`](../gui.py) module provides a full Tkinter-based graphical interface with the following features:

| Feature | Description |
|---------|-------------|
| **Algorithm Selector** | Dropdown to switch between A*, BFS, Greedy Best-First, and DFS. The planned path updates instantly. |
| **Robot Animation** | Click ▶ Run to watch the robot (🤖) traverse the grid cell by cell with smooth step-by-step animation. |
| **Speed Control** | Slider to adjust animation speed from 50 ms to 500 ms per step. |
| **Step Mode** | ⏭ Step button advances exactly one cell per click for detailed inspection. |
| **Pause/Resume** | ⏸ Pause/▶ Resume toggle during animation. |
| **Interactive Grid Editing** | Click any cell to cycle: Open `.` → Shelf `#` → Crowded `C` → Trolley `T`. |
| **Dynamic Trolley Simulation** | Checkbox to enable trolley mode: a hidden trolley is auto-placed on the route, and the robot detects and replans in real time. |
| **Live Metrics HUD** | Real-time display of Robot Position, Accumulated Cost, Plan Cost, Nodes Expanded, Replans Occurred. |
| **Comparison Dialog** | 📊 button opens a modal table comparing all 4 algorithms on the current grid. |
| **Reset & Default** | 🔄 Reset returns to initial position; 🧹 Default restores the standard assignment layout. |

### Launch

```bash
python gui.py
# or
python main.py --gui
```

---

## 5.8 Test Suite

### Implementation

The test suite ([`tests.py`](../tests.py)) uses Python's built-in `unittest` framework. It contains **39 tests** across 8 test classes:

| Test Class | Tests | Coverage |
|------------|:-----:|----------|
| `TestWarehouseGrid` | 14 | Grid construction, cell access, bounds checking, cost calculation, actions, goal test, cloning, dynamic marking. |
| `TestManhattan` | 3 | Heuristic correctness: zero distance, known distance, symmetry. |
| `TestBFS` | 4 | Goal reachability, blocked grid handling, path connectivity, optimal move count. |
| `TestDFS` | 3 | Goal reachability, blocked grid handling, path connectivity. |
| `TestGreedy` | 2 | Goal reachability, blocked grid handling. |
| `TestAStar` | 6 | Goal reachability, blocked grid, cost optimality vs. BFS, cost optimality vs. Greedy, detour cheaper than direct, start override. |
| `TestTrolleySimulator` | 5 | Simulation execution, replan count, goal reachability without trolleys, custom trolley placement, trace starts at S. |
| `TestRunAll` | 2 | Returns 4 results with correct algorithm names; all algorithms find paths on default grid. |

### Test Results

```
----------------------------------------------------------------------
Ran 39 tests in 0.005s

OK
```

All 39 tests pass successfully.

### How to Run

```bash
# Using unittest (built-in)
python -m unittest tests.py -v

# Using pytest (if installed)
pytest tests.py -v

# Direct execution
python tests.py
```

---

## 5.9 Summary of All Deliverables

| Deliverable | Location | Status |
|-------------|----------|:------:|
| Performance comparison table | Console output from `main.py` | ✅ |
| Trolley replanning metrics table | Console output from `main.py` | ✅ |
| Grid overview screenshot | `screenshots/grid_overview.png` | ✅ |
| Algorithm routes screenshot | `screenshots/algorithm_routes.png` | ✅ |
| Trolley replanning screenshot | `screenshots/trolley_replanning.png` | ✅ |
| Performance bar chart | `screenshots/performance_chart.png` | ✅ |
| Interactive GUI with animation | `gui.py` | ✅ |
| Automated test suite (39 tests) | `tests.py` | ✅ |
| Project README documentation | `README.md` | ✅ |
| Cost ≠ Moves explanation | Console output + Task 3 report | ✅ |
