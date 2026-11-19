# Task 1 — Warehouse Environment

**Assigned To:** Dev Goyal (BT24CSD029)
**Source File:** [`warehouse_grid.py`](../warehouse_grid.py)
**Objective:** Build and display a 2D warehouse grid with distinct cell types, a weighted
traversal cost, 4-directional movement rules and a start/destination pair.

---

## 1.1 Overview

The warehouse is a rectangular grid of single-character cells. Each cell is one region of the
floor: an aisle, a rack, a security zone, the packing station or the dispatch dock. The robot
navigates from the start cell to the goal cell subject to obstacles and variable traversal
cost.

The module owns four things:

* the **cell vocabulary** and the cost of entering each cell type;
* the **movement model** (4-connected, bounds- and obstacle-filtered);
* an immutable-ish **map object** that can be cloned and updated dynamically;
* the **presentation** of the map on the console.

---

## 1.2 Cell types

| Symbol | Name | Traversable | Entry cost | Description |
|:---:|---|:---:|:---:|---|
| `.` | Open aisle | yes | 1 | Standard unobstructed corridor |
| `C` | Crowded aisle | yes | 4 | Staff or carts present; the robot slows down |
| `#` | Shelf | no | ∞ | Storage rack |
| `R` | Restricted zone | no | ∞ | Security or hazard area; entry forbidden |
| `S` | Start | yes | 1 | Packing station |
| `G` | Goal | yes | 1 | Dispatch dock |
| `T` | Trolley | no | ∞ | Dynamic obstacle, unknown until the sensor sees it |

The distinction is deliberate: **crowded aisles are passable but expensive**, which is what
makes the cheapest route differ from the shortest one. Shelves, restricted zones and detected
trolleys are hard obstacles — `is_passable()` returns `False` and no action can enter them.

---

## 1.3 Movement rules

| Action | Vector $(\Delta r, \Delta c)$ |
|---|:---:|
| `UP` | $(-1, 0)$ |
| `DOWN` | $(+1, 0)$ |
| `LEFT` | $(0, -1)$ |
| `RIGHT` | $(0, +1)$ |

Diagonal moves are not allowed. An action is legal when the target cell is inside the grid
**and** not in `IMPASSABLE = {'#', 'R', 'T'}`.

---

## 1.4 Cost model

$$\text{PathCost}(p) = \sum_{i=1}^{n} \text{EntryCost}(p_i)$$

The start cell is free; every cell that is *entered* contributes. A three-step path that ends
on a crowded cell costs $1 + 1 + 4 = 6$.

```python
CELL_COST  = {'.': 1, 'C': 4, 'S': 1, 'G': 1}
IMPASSABLE = {'#', 'R', 'T'}
```

---

## 1.5 Default 10×10 floor plan

```text
    0 1 2 3 4 5 6 7 8 9
 0  S . # . . . . C . R
 1  . . # . . # . . . .
 2  . . # . . # . C R .
 3  . . . . C . . . . .
 4  . # # # # # . . . .
 5  . . C . . . . . . .
 6  . . . . # . . . R .
 7  . C . . # . . C . .
 8  . . . . # . . . R .
 9  . . . . . . # . . G
```

| Cell type | Count |
|---|---:|
| Open aisles `.` | 74 |
| Crowded `C` | 6 — (0,7), (2,7), (3,4), (5,2), (7,1), (7,7) |
| Shelves `#` | 14 — column 2 rows 0–2, column 5 rows 1–2, row 4 columns 1–5, column 4 rows 6–9 |
| Restricted `R` | 4 — (0,9), (2,8), (6,8), (8,8) |
| Start `S` | (0,0) |
| Goal `G` | (9,9) |

Design intent:

* The vertical rack in column 2 and the wall in row 4 split the floor into a northern and a
  southern half, so a route has to commit to a corridor.
* The rack in column 4 (rows 6–9) creates two southern exits, which is what lets the robot
  detour around an obstacle.
* Crowded cells sit on the tempting direct routes, not on the corners, so avoiding them costs
  moves rather than being free.
* The four restricted zones remove the far-eastern aisle, forcing the routes seen in Task 3.

---

## 1.6 Class API

```python
class WarehouseGrid:
    def __init__(self, layout: List[str]) -> None
    def cell(self, r, c) -> str
    def in_bounds(self, r, c) -> bool
    def is_passable(self, r, c) -> bool
    def entry_cost(self, r, c) -> float
    def count(self, symbol) -> int
    def actions(self, state) -> List[Tuple[int, int, str]]
    def result(self, state, action) -> Tuple[int, int]
    def goal_test(self, state) -> bool
    def path_cost(self, path) -> float
    def mark_impassable(self, r, c) -> None
    def clone(self) -> 'WarehouseGrid'
    def display(self, path=None, label='') -> None
```

`mark_impassable()` and `clone()` exist for Task 4: the simulator updates the robot's belief
about the map and replans without corrupting the reference copy.

### Input validation

The constructor rejects malformed maps instead of failing later with an `IndexError`:

```python
WarehouseGrid(["S..", "..G.."])   # ValueError: All rows must have the same width
WarehouseGrid([])                 # ValueError: Layout must contain at least one row
WarehouseGrid(["S..", "..."])     # ValueError: Symbol 'G' not found in grid
```

---

## 1.7 Console output

```text
─────────────────────
 Default 10×10 warehouse grid
─────────────────────
   0 1 2 3 4 5 6 7 8 9
 0 S . # . . . . C . R
 1 . . # . . # . . . .
 2 . . # . . # . C R .
 3 . . . . C . . . . .
 4 . # # # # # . . . .
 5 . . C . . . . . . .
 6 . . . . # . . . R .
 7 . C . . # . . C . .
 8 . . . . # . . . R .
 9 . . . . . . # . . G
```

---

## 1.8 Visual output

[`screenshots/grid_overview.png`](../screenshots/grid_overview.png) — rounded tiles, column
and row rulers, a cell-type legend and the cell inventory.

---

## 1.9 Unit tests

From [`tests.py`](../tests.py):

| Test | Verifies |
|---|---|
| `test_dimensions` | Grid is sized from the layout |
| `test_start_goal_detection` | `S` found at (0,0), `G` at (2,2) in the 3×3 fixture |
| `test_in_bounds` | Out-of-range coordinates rejected |
| `test_is_passable` | Traversable vs impassable classification |
| `test_impassable_shelf` | Shelves block movement |
| `test_restricted_zone_is_impassable` | `R` blocks movement like `#` |
| `test_entry_cost_open` / `_crowded` | Costs 1 and 4 |
| `test_path_cost_simple` / `_empty` | Cost sums entered cells, start excluded |
| `test_actions_centre` / `_corner` | 4 neighbours inside, 2 at a corner |
| `test_goal_test` | Goal test only true at the goal |
| `test_mark_impassable` | Dynamic update changes passability |
| `test_clone_independence` | Clones do not share state |
| `test_ragged_layout_rejected` | Unequal row widths raise `ValueError` |
| `test_empty_layout_rejected` | Empty layout raises `ValueError` |
| `test_missing_goal_rejected` | Missing `G` raises `ValueError` |
| `test_default_grid_uses_every_cell_type` | Default map contains `.`, `C`, `#`, `R` |
