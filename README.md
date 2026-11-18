# AI-Based Route Planning for a Warehouse Delivery Robot

**Course:** Artificial Intelligence  
**Group:** 5  
**Language:** Python 3  
**Submission Date:** 18 November 2026  

---

## 1. Group Members & Task Assignment

Per Section 5 of the assignment specification:

| Member # | Name | Roll Number | Assigned Responsibility | Primary Contributions |
|---|---|---|---|---|
| **Member 1** | Dev Goyal | `BT24CSD029` | Warehouse grid & search problem formulation | Designed `WarehouseGrid` class, cell types, state space representation, action transitions, and entry costs (`warehouse_grid.py`). |
| **Member 2** | Yash Joshi | `BT24CSD024` | BFS and DFS implementation | Implemented Breadth-First Search (FIFO queue) and Depth-First Search (LIFO stack) pathfinders (`search_algorithms.py`). |
| **Member 3** | Yuvraj Singh | `BT24CSD031` | Greedy Best-First Search & heuristic explanation | Implemented Greedy BFS with priority queue, Manhattan distance heuristic, and admissibility proof (`search_algorithms.py`). |
| **Member 4** | Harsh Patil | `BT24CSD033` | A* implementation & weighted path costs | Implemented $f(n) = g(n) + h(n)$ A* algorithm and demonstrated cost vs. move count trade-offs on crowded grids (`search_algorithms.py`). |
| **Member 5** | Piyush Mangalam | `BT24CSD032` | Trolley detection, map update, and replanning | Built `TrolleySimulator` with one-step lookahead detection, dynamic map update, and online A* replanning (`trolley_replanning.py`). |
| **Member 6** | Sayan Majhi | `BT24CSD028` | Testing, visualization, performance comparison, and documentation | Authored test suites (`tests.py`), performance metric aggregation (`metrics.py`), Matplotlib visualization engine (`visualization.py`), and documentation (`README.md`). |

---

## 2. Problem Statement

A warehouse delivery robot transports packages from a dispatch station ($S$) to a destination delivery dock ($G$) on a 2D floor grid. 
Certain routes are blocked by permanent shelves (`#`) or restricted security zones (`R`). Other aisles are open (`.`) or crowded with human staff/carts (`C`), which take significantly longer to cross. Additionally, dynamic obstacles such as unattended trolleys (`T`) may appear unexpectedly on routes initially believed to be clear.

The objective is to:
1. Model the warehouse environment and formulate the formal search problem.
2. Implement and evaluate four fundamental search algorithms: **Breadth-First Search (BFS)**, **Depth-First Search (DFS)**, **Greedy Best-First Search**, and **A\***.
3. Handle dynamic trolley obstacles through real-time detection, map updating, and online replanning using A*.
4. Perform an empirical evaluation comparing path lengths, path costs, node expansions, and execution times.

---

## 3. Search Problem Formulation (Task 2)

- **State Space ($S$):** Integer coordinate pairs $(r, c)$ representing the robot's current position on the grid, where $0 \le r < \text{rows}$ and $0 \le c < \text{cols}$.
- **Initial State ($s_0$):** The coordinates of the start station `S` (e.g., $(0, 0)$).
- **Actions ($A(s)$):** 4-connected orthogonal movements:
  $$\{\text{UP}: (-1, 0), \quad \text{DOWN}: (+1, 0), \quad \text{LEFT}: (0, -1), \quad \text{RIGHT}: (0, +1)\}$$
  Filtered to exclude out-of-bounds positions and impassable cells (`#`, `R`, and detected `T`).
- **Transition Model ($\text{Result}(s, a)$):** Deterministic movement: if $s = (r, c)$ and $a = (\Delta r, \Delta c)$, then $\text{Result}(s, a) = (r + \Delta r, c + \Delta c)$.
- **Goal Test ($\text{IsGoal}(s)$):** Evaluates whether $s = (r_G, c_G)$, the coordinates of the goal station `G`.
- **Path-Cost Function ($c(s, a, s')$):** The cost of entering the destination cell $s'$:
  - Normal open aisle (`.`): **1**
  - Crowded cell (`C`): **4**
  - Start (`S`) / Goal (`G`): **1**
  - Shelf (`#`) / Restricted (`R`) / Trolley (`T`): $\infty$ (impassable)
  The total path cost is the sum of entry costs of all visited cells along the path, excluding the initial state.

---

## 4. Algorithms & Heuristics (Task 3)

### 4.1 Implemented Algorithms
1. **Breadth-First Search (BFS):**
   - Uses a FIFO queue.
   - Explores shallowest unexpanded nodes first.
   - Guarantees the optimal solution in terms of **number of moves** (step count), ignoring cell weights.
2. **Depth-First Search (DFS):**
   - Uses a LIFO stack.
   - Explores deepest paths first with cycle-avoidance via a visited set.
   - Not guaranteed to find optimal paths (often returns long, winding routes).
3. **Greedy Best-First Search:**
   - Uses a priority queue ordered strictly by the heuristic $h(n)$.
   - Expands nodes that appear closest to the goal.
   - Fast, but not cost-optimal as it ignores accumulated path cost $g(n)$.
4. **A\* Search:**
   - Uses a priority queue ordered by $f(n) = g(n) + h(n)$.
   - $g(n)$: Accumulated cost from the start state to node $n$.
   - $h(n)$: Estimated cost from node $n$ to the goal.
   - Guarantees the **lowest-cost path** when using an admissible heuristic.

### 4.2 Manhattan Distance Heuristic & Admissibility Proof
For coordinates $(r, c)$ and goal $(r_G, c_G)$:
$$h(n) = |r - r_G| + |c - c_G|$$

**Proof of Admissibility (Lower Bound):**
1. In a 4-connected grid without diagonal moves, the minimum number of transitions required to move from $(r, c)$ to $(r_G, c_G)$ is $|r - r_G| + |c - c_G|$ (the $L_1$ distance).
2. The minimum possible entry cost of any traversable cell in the grid is $c_{\min} = 1$ (for normal open aisles).
3. Therefore, the true cost $h^*(n)$ of the shortest possible path to the goal satisfies:
   $$h^*(n) \ge c_{\min} \times (|r - r_G| + |c - c_G|) = 1 \times h(n) = h(n)$$
4. Since $h(n) \le h^*(n)$ for all states $n$, $h(n)$ never overestimates the actual remaining cost.
5. Furthermore, the Manhattan heuristic is **consistent (monotonic)**:
   $$h(n) \le c(n, a, n') + h(n')$$
   because $c(n, a, n') \ge 1$ and $|h(n) - h(n')| \le 1$ for any single orthogonal step.
   Consequently, A* with graph search is guaranteed to find the optimal path.

### 4.3 Path Length vs. Path Cost Analysis
- **Does the shortest route always have the lowest cost?**  
  **No.** When some cells have higher traversal costs (crowded cells with cost = 4 vs. normal aisles with cost = 1), a route with fewer moves that cuts through crowded cells may have a significantly higher total cost than a detour around them.
- In our test demonstration (`main.py` and `tests.py`), A* deliberately chooses a path with more moves to avoid expensive crowded cells, demonstrating that **minimum moves $\neq$ minimum cost**.

---

## 5. Dynamic Trolley Obstacle Detection & Replanning (Task 4)

1. **Initial Plan:** The robot computes an initial lowest-cost path from $S$ to $G$ using A* on its known map.
2. **Hidden Obstacle:** A dynamic trolley (`T`) is positioned on one of the planned path's cells in the physical environment, but is absent from the robot's initial static map.
3. **Step-by-Step Execution & Lookahead Detection:**
   - The robot executes its planned route one cell at a time.
   - Before stepping into the next cell, the robot's forward sensors inspect the target cell.
   - If a trolley is detected:
     1. The robot halts at its current position.
     2. Its internal map is dynamically updated by marking the target cell as impassable (`T`).
     3. A* replanning is invoked from the robot's **current position** to the goal $G$.
     4. If an alternative path is found, the robot follows the revised route.
     5. If all routes are blocked, the robot reports that the destination is unreachable.

---

## 6. Architecture & File Structure

```
AI_Warehouse_PathPlanning_Group5/
├── warehouse_grid.py          # Grid environment, cell definitions, cost model & problem formulation
├── search_algorithms.py       # BFS, DFS, Greedy Best-First, and A* implementations
├── trolley_replanning.py      # Dynamic trolley obstacle detection & online A* replanning
├── metrics.py                 # Performance tables, metrics aggregation, findings summary
├── visualization.py           # Matplotlib grid plotting, route overlays, charts
├── main.py                    # Complete end-to-end demonstration script
├── tests.py                   # Comprehensive unit and integration test suite
├── requirements.txt           # Python library dependencies
├── README.md                  # Complete project documentation & reports
└── screenshots/               # Visual outputs and plots
    ├── grid_overview.png
    ├── algorithm_routes.png
    ├── trolley_replanning.png
    └── performance_chart.png
```

---

## 7. Installation & Execution

### 7.1 Prerequisites & Dependencies
- Python 3.9+ (tested with Python 3.13 / 3.14)
- Libraries specified in `requirements.txt`:
  - `numpy>=1.24.0`
  - `matplotlib>=3.7.0`

Install dependencies using:
```bash
pip install -r requirements.txt
```

### 7.2 Running the Full Demonstration
Execute `main.py` from the project root:
```bash
python main.py
```
*(On Windows systems with the Python launcher, you can also run `py main.py` or `py -3.13 main.py`).*

This executes:
1. Environment setup and problem formulation display.
2. Search algorithm execution (BFS, DFS, Greedy, A*) on the default 10×10 warehouse grid.
3. Cost vs. Move count trade-off demonstration on a crowded corridor.
4. Dynamic trolley simulation with lookahead detection and A* replanning.
5. Performance metric comparison table printing.
6. Export of high-resolution plots to `screenshots/`.

### 7.3 Running the Test Suite
Run the automated test suite with standard `unittest`:
```bash
python -m unittest tests.py -v
```
Or using `pytest` if installed:
```bash
pytest tests.py -v
```

---

## 8. Experimental Results & Performance Comparison (Task 5)

### 8.1 Default 10×10 Warehouse Layout

```
    0 1 2 3 4 5 6 7 8 9
 0  S . # . . . . C . .
 1  . . # . . # . . . .
 2  . . # . . # . C . .
 3  . . . . C . . . . .
 4  . # # # # # . . . .
 5  . . C . . . . . . .
 6  . . . . # . . . . .
 7  . C . . # . . C . .
 8  . . . . # . . . . .
 9  . . . . . . # . . G
```

### 8.2 Performance Comparison Table

| Algorithm | Goal Reached? | Path Length (Moves) | Total Path Cost | Nodes Expanded | Execution Time (ms) |
|---|:---:|:---:|:---:|:---:|:---:|
| **BFS** | Yes | 18 | 21.0 | 86 | 0.12 |
| **DFS** | Yes | 24 | 30.0 | 43 | 0.06 |
| **Greedy** | Yes | 20 | 20.0 | 21 | 0.04 |
| **A\*** | Yes | 20 | 20.0 | 51 | 0.15 |

*(Times measured using Python `time.perf_counter()` on the default 10×10 grid)*

### 8.3 Key Findings & Discussion
1. **Optimality of A\* vs. BFS (Cost vs. Moves):** 
   - On the default 10×10 grid, BFS found the shortest path in terms of moves (18 moves), but at a higher cost ($21.0$) because it traversed through a crowded cell (`C`).
   - A* and Greedy avoided the crowded cell, yielding a path with cost $20.0$.
   - On the dedicated corridor test (`S.CCCC....G` vs. `...........`), BFS took 11 moves with cost $18.0$, whereas A* deliberately took 12 moves with cost $12.0$ by routing through the uncrowded row, proving that **minimum moves $\neq$ minimum cost**.
2. **Greedy Search Efficiency:** Greedy Best-First Search expanded the fewest nodes ($21$), rapidly heading towards the target via Manhattan heuristic guidance, but lacks general cost-optimality guarantees.
3. **DFS Inefficiency:** DFS explored non-optimal paths, resulting in an erratic path of 24 moves and higher path cost ($30.0$).
4. **Replanning Resilience:** In the dynamic trolley obstacle experiment:
   - Initial A* planned route of 20 moves (cost $20.0$).
   - The unexpected trolley obstacle was placed at cell $(3, 0)$.
   - While positioned at $(2, 0)$, the robot's lookahead sensor detected the blocked cell before entering.
   - The internal map was updated dynamically (`T` marked impassable) and A* was invoked from the current position $(2, 0)$.
   - A valid alternative route was computed in $< 1\text{ ms}$ (16 remaining moves, cost $19.0$), and the delivery succeeded with exactly 1 replan.

---

## 9. Output Visualizations

The program generates visual artifacts saved in the `screenshots/` directory:

| Screenshot | Description |
|---|---|
| `screenshots/grid_overview.png` | Floor map showing aisles, shelves (`#`), restricted zones (`R`), crowded cells (`C`), start (`S`), and goal (`G`). |
| `screenshots/algorithm_routes.png` | 2×2 comparison showing the distinct routes traversed by BFS, DFS, Greedy, and A*. |
| `screenshots/trolley_replanning.png` | Multi-panel sequence displaying initial plan, obstacle detection, and revised replanned route. |
| `screenshots/performance_chart.png` | Bar charts comparing Nodes Expanded, Path Cost vs. Move Count, and Execution Time. |

---

## 10. Acknowledgements & References

1. Russell, S., & Norvig, P. (2020). *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson. (Chapters 3 & 4: Uninformed and Informed Search).
2. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics.
3. Python Standard Library Documentation (`collections.deque`, `heapq`, `time`, `unittest`).
4. Matplotlib and NumPy open-source development communities.
