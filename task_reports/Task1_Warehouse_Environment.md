# Task 1 — Warehouse Environment

**Assigned To:** Dev Goyal (BT24CSD029)  
**Source File:** [`warehouse_grid.py`](../warehouse_grid.py)  
**Objective:** Build and display a 2D warehouse grid with differentiated cell types, movement rules, and cost model.

---

## 1.1 Overview

The warehouse is modelled as a rectangular $rows \times cols$ grid of cells. Each cell represents a physical region of the warehouse floor. The robot navigates this grid from a packing station to a dispatch area, subject to obstacles and variable traversal costs.

This module defines:
- The grid data structure and its visual representation.
- Cell types with associated traversal semantics (passable vs. impassable, cost).
- The movement model (4-directional connectivity).
- Helper methods for grid construction, cell queries, display, cloning, and dynamic updates.

---

## 1.2 Cell Types

The warehouse contains seven distinct cell types, each represented by a single character symbol:

| Symbol | Name | Traversable? | Entry Cost | Description |
|:------:|------|:------------:|:----------:|-------------|
| `.` | **Open Aisle** | ✅ Yes | 1 | Standard unobstructed corridor. |
| `C` | **Crowded Cell** | ✅ Yes | 4 | Aisle congested with staff or equipment; robot slows down. |
| `#` | **Shelf** | ❌ No | $\infty$ | Permanent storage rack; completely impassable. |
| `R` | **Restricted Zone** | ❌ No | $\infty$ | Security or hazard zone; entry forbidden. |
| `S` | **Start** | ✅ Yes | 1 | Robot's initial position (packing station). |
| `G` | **Goal** | ✅ Yes | 1 | Delivery destination (dispatch dock). |
| `T` | **Trolley** | ❌ No | $\infty$ | Dynamic obstacle; unknown to robot until detected by sensors (see Task 4). |

> [!IMPORTANT]
> The cost model distinguishes between **passable** and **impassable** cells. Shelves (`#`), restricted zones (`R`), and detected trolleys (`T`) are strictly impassable — the robot cannot enter these cells under any circumstances. Crowded cells are passable but penalised with a 4× cost multiplier compared to open aisles.

---

## 1.3 Movement Rules

The robot uses **4-connectivity** (orthogonal movement only):

| Action | Direction Vector $(\\Delta r, \\Delta c)$ |
|--------|:---:|
| `UP` | $(-1, 0)$ |
| `DOWN` | $(+1, 0)$ |
| `LEFT` | $(0, -1)$ |
| `RIGHT` | $(0, +1)$ |

An action is **valid** if and only if:
1. The resulting cell $(r + \Delta r, c + \Delta c)$ is within grid bounds: $0 \le r' < rows$ and $0 \le c' < cols$.
2. The target cell is **not** impassable (i.e., not `#`, `R`, or detected `T`).

Diagonal movement is **not** permitted.

---

## 1.4 Cost Model

The **path cost** is calculated as the sum of **entry costs** for every cell the robot enters along its route, **excluding the start cell**:

$$\text{PathCost}(p) = \sum_{i=1}^{n} \text{EntryCost}(p_i)$$

where $p = \langle p_0, p_1, \ldots, p_n \rangle$ is the sequence of cells visited from start $p_0$ to goal $p_n$.

**Example:** A path that enters 3 open aisles and 1 crowded cell costs $3 \times 1 + 1 \times 4 = 7$.

---

## 1.5 Default Grid Layout (10×10)

The project uses the following default warehouse floor plan for all demonstrations:

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

**Key features of this layout:**
- **Start** `S` is at cell $(0, 0)$ — top-left corner.
- **Goal** `G` is at cell $(9, 9)$ — bottom-right corner.
- A vertical wall of shelves `#` runs down column 2 (rows 0–2), creating a barrier the robot must navigate around.
- A horizontal wall `#####` spans row 4, columns 1–5, forcing the robot through one of two main corridors.
- Several crowded cells `C` are scattered to create cost trade-offs (at $(0,7)$, $(2,7)$, $(3,4)$, $(5,2)$, $(7,1)$, $(7,7)$).
- The layout ensures that the **shortest-move** path and the **lowest-cost** path are distinct, verifying that move count $\neq$ path cost.

---

## 1.6 Implementation Details

### `WarehouseGrid` Class

The core data structure is the `WarehouseGrid` class defined in [`warehouse_grid.py`](../warehouse_grid.py):

```python
class WarehouseGrid:
    def __init__(self, layout: List[str]) -> None:
        self.rows = len(layout)
        self.cols = len(layout[0])
        self._grid: List[List[str]] = [list(row) for row in layout]
        self.start: Tuple[int, int] = self._find(START)
        self.goal:  Tuple[int, int] = self._find(GOAL)
```

**Constructor:** Accepts a list of strings where each string represents one row. Each character maps to a cell type. The grid is stored as a mutable list-of-lists to support dynamic updates (trolley placement in Task 4). Start and goal positions are located automatically using `_find()`.

### Key Methods

| Method | Purpose |
|--------|---------|
| `cell(r, c)` | Returns the symbol at position $(r, c)$. |
| `in_bounds(r, c)` | Returns `True` if $(r, c)$ lies within the grid dimensions. |
| `is_passable(r, c)` | Returns `True` if the cell is both in-bounds and not impassable. |
| `entry_cost(r, c)` | Returns the numeric entry cost for the cell (1, 4, or $\infty$). |
| `actions(state)` | Returns all valid moves from `state` as `(new_r, new_c, name)` tuples. |
| `result(state, action)` | Deterministic transition: returns the new state after taking an action. |
| `goal_test(state)` | Returns `True` if `state` matches the goal coordinates. |
| `path_cost(path)` | Computes the total traversal cost of a given path (excludes start cell). |
| `mark_impassable(r, c)` | Marks a cell as `'T'` (trolley) — used for dynamic map updates in Task 4. |
| `clone()` | Returns a deep copy of the grid for use in replanning without corrupting the original map. |
| `display(path, label)` | Prints the grid to console with optional path overlay (marked with `*`). |

### Constants

```python
CELL_COST = {'.': 1, 'C': 4, 'S': 1, 'G': 1}
IMPASSABLE = {'#', 'R', 'T'}
ACTIONS = [(-1, 0, 'UP'), (1, 0, 'DOWN'), (0, -1, 'LEFT'), (0, 1, 'RIGHT')]
```

---

## 1.7 Console Output

When `main.py` is executed, Task 1 produces the following console output:

```
═════════════════════════════════════════════════════════════════
  TASK 1 & 2 – WAREHOUSE ENVIRONMENT & PROBLEM FORMULATION
═════════════════════════════════════════════════════════════════

─────────────────────
 Default 10×10 Warehouse Grid
─────────────────────
   0 1 2 3 4 5 6 7 8 9
 0 S . # . . . . C . .
 1 . . # . . # . . . .
 2 . . # . . # . C . .
 3 . . . . C . . . . .
 4 . # # # # # . . . .
 5 . . C . . . . . . .
 6 . . . . # . . . . .
 7 . C . . # . . C . .
 8 . . . . # . . . . .
 9 . . . . . . # . . G
```

---

## 1.8 Visual Output

The module generates a colour-coded grid overview saved as [`screenshots/grid_overview.png`](../screenshots/grid_overview.png), showing all cell types with a legend.

---

## 1.9 Unit Tests

The following tests in [`tests.py`](../tests.py) verify the warehouse environment:

| Test | Verification |
|------|-------------|
| `test_dimensions` | Grid is correctly sized (3×3 for test layout). |
| `test_start_goal_detection` | `S` found at $(0,0)$, `G` at $(2,2)$. |
| `test_in_bounds` | Rejects out-of-bounds coordinates. |
| `test_is_passable` | Correctly classifies traversable vs. impassable cells. |
| `test_impassable_shelf` | Shelves are impassable. |
| `test_entry_cost_open` | Open aisle cost = 1. |
| `test_entry_cost_crowded` | Crowded cell cost = 4. |
| `test_path_cost_simple` | Sum of entry costs along a known path matches expected value. |
| `test_path_cost_empty` | Empty/single-cell paths have cost 0. |
| `test_actions_centre` | Centre cell has 4 neighbours. |
| `test_actions_corner` | Corner cell has only 2 valid neighbours. |
| `test_goal_test` | Goal test returns `True` only at goal coordinates. |
| `test_mark_impassable` | Dynamic marking changes passability. |
| `test_clone_independence` | Cloned grids are independent of the original. |
