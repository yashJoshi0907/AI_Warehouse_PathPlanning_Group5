"""
warehouse_grid.py
=================
Task 1 & 2 – Warehouse Environment & Problem Formulation
Author   : Dev Goyal (BT24CSD029)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

The warehouse floor is a rectangular grid of single-character cells.  This
module owns the cell vocabulary, the traversal cost model, the 4-connected
movement rules, the display routine and the dynamic map update used by the
trolley replanning task.
"""

from __future__ import annotations
from typing import Dict, List, Optional, Tuple
import copy

# ---------------------------------------------------------------------------
# Cell-type constants
# ---------------------------------------------------------------------------
OPEN       = '.'   # Normal aisle            cost = 1
CROWDED    = 'C'   # Crowded cell            cost = 4
SHELF      = '#'   # Shelf (impassable)      cost = ∞
RESTRICTED = 'R'   # Restricted zone         cost = ∞
START      = 'S'   # Robot start position    cost = 1
GOAL       = 'G'   # Delivery destination    cost = 1
TROLLEY    = 'T'   # Dynamic obstacle        cost = ∞ (when detected)

# Entry cost for each traversable cell type
CELL_COST: Dict[str, float] = {
    OPEN:       1,
    CROWDED:    4,
    START:      1,
    GOAL:       1,
}

IMPASSABLE = {SHELF, RESTRICTED, TROLLEY}

# 4-connectivity action vectors  (dr, dc, name)
ACTIONS = [
    (-1,  0, 'UP'),
    ( 1,  0, 'DOWN'),
    ( 0, -1, 'LEFT'),
    ( 0,  1, 'RIGHT'),
]


class WarehouseGrid:
    """
    Represents the warehouse floor as a 2-D grid.

    Parameters
    ----------
    layout : list of str
        Each string is one row; every character is a cell symbol.
        All rows must have the same length.

    Task 2 – Problem Formulation
    ----------------------------
    State            : (row, col) – integer tuple
    Initial state    : coordinates of 'S'
    Actions          : {UP, DOWN, LEFT, RIGHT} – filtered by bounds & passability
    Transition model : deterministic; result(action, state) = adjacent cell
    Goal test        : state == coordinates of 'G'
    Path-cost fn     : sum of CELL_COST for every cell entered (start excluded)
    """

    def __init__(self, layout: List[str]) -> None:
        if not layout:
            raise ValueError("Layout must contain at least one row")
        widths = {len(row) for row in layout}
        if len(widths) != 1:
            raise ValueError(
                f"All rows must have the same width, got widths {sorted(widths)}"
            )
        self.rows = len(layout)
        self.cols = widths.pop()
        # Store as mutable list-of-lists for dynamic updates
        self._grid: List[List[str]] = [list(row) for row in layout]
        self.start: Tuple[int, int] = self._find(START)
        self.goal:  Tuple[int, int] = self._find(GOAL)

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    def _find(self, symbol: str) -> Tuple[int, int]:
        for r in range(self.rows):
            for c in range(self.cols):
                if self._grid[r][c] == symbol:
                    return (r, c)
        raise ValueError(f"Symbol '{symbol}' not found in grid")

    # ------------------------------------------------------------------
    # Cell access
    # ------------------------------------------------------------------

    def cell(self, r: int, c: int) -> str:
        return self._grid[r][c]

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_passable(self, r: int, c: int) -> bool:
        return self.in_bounds(r, c) and self._grid[r][c] not in IMPASSABLE

    def entry_cost(self, r: int, c: int) -> float:
        sym = self._grid[r][c]
        return CELL_COST.get(sym, float('inf'))

    def count(self, symbol: str) -> int:
        """Number of cells carrying *symbol* (used by the report and plots)."""
        return sum(row.count(symbol) for row in self._grid)

    # ------------------------------------------------------------------
    # Task 2 – actions & transition
    # ------------------------------------------------------------------

    def actions(self, state: Tuple[int, int]) -> List[Tuple[int, int, str]]:
        """Return list of (new_r, new_c, action_name) for valid moves."""
        r, c = state
        result = []
        for dr, dc, name in ACTIONS:
            nr, nc = r + dr, c + dc
            if self.is_passable(nr, nc):
                result.append((nr, nc, name))
        return result

    def result(self, state: Tuple[int, int], action: Tuple[int, int, str]) -> Tuple[int, int]:
        """Deterministic transition: returns new state."""
        nr, nc, _ = action
        return (nr, nc)

    def goal_test(self, state: Tuple[int, int]) -> bool:
        return state == self.goal

    def path_cost(self, path: List[Tuple[int, int]]) -> float:
        """Sum of entry costs for all cells in path EXCEPT the start cell."""
        if len(path) < 2:
            return 0.0
        return sum(self.entry_cost(r, c) for r, c in path[1:])

    # ------------------------------------------------------------------
    # Dynamic map update (Task 4)
    # ------------------------------------------------------------------

    def mark_impassable(self, r: int, c: int) -> None:
        """Mark a cell as a trolley obstacle (dynamic update)."""
        self._grid[r][c] = TROLLEY

    def clone(self) -> 'WarehouseGrid':
        """Return a deep copy of the grid (for replanning)."""
        new_grid = WarehouseGrid.__new__(WarehouseGrid)
        new_grid.rows  = self.rows
        new_grid.cols  = self.cols
        new_grid._grid = copy.deepcopy(self._grid)
        new_grid.start = self.start
        new_grid.goal  = self.goal
        return new_grid

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def display(self, path: Optional[List[Tuple[int, int]]] = None,
                label: str = '') -> None:
        """Print grid to console; optionally highlight a path with '*'."""
        path_set = set(path or [])
        width = self.cols * 2 + 1
        if label:
            print(f"\n{'─' * width}")
            print(f" {label}")
            print(f"{'─' * width}")
        # Column numbers (two characters wide, so column 10 is not ambiguous)
        header = "   " + " ".join(f"{c % 10}" for c in range(self.cols))
        print(header)
        for r in range(self.rows):
            row_str = f"{r:2} "
            for c in range(self.cols):
                sym = self._grid[r][c]
                if (r, c) in path_set and sym not in (START, GOAL):
                    row_str += '* '
                else:
                    row_str += sym + ' '
            print(row_str.rstrip())
        print()

    def __repr__(self) -> str:
        return f"WarehouseGrid({self.rows}×{self.cols}, S={self.start}, G={self.goal})"


# ---------------------------------------------------------------------------
# Default 10×10 warehouse layout used in all demos
#
#   S  start (packing station)      G  goal (dispatch dock)
#   #  shelf                        R  restricted zone
#   C  crowded aisle (cost 4)       .  open aisle (cost 1)
# ---------------------------------------------------------------------------
DEFAULT_LAYOUT = [
    "S.#....C.R",
    "..#..#....",
    "..#..#.CR.",
    "....C.....",
    ".#####....",
    "..C.......",
    "....#...R.",
    ".C..#..C..",
    "....#...R.",
    "......#..G",
]

# A second layout to illustrate the cost-vs-moves trade-off (Task 3 test case).
# Row 0 is the fewest-moves route but crosses four crowded cells (cost 4 each);
# row 1 is a 2-move-longer detour made entirely of open aisles (cost 1 each).
COST_TEST_LAYOUT = [
    "S.CCCC....G",
    "...........",
]

# A one-cell-wide serpentine corridor (Task 4 "no alternative route" case).
# Blocking any cell on the route disconnects the goal completely, so replanning
# must report that the destination is unreachable.
BOTTLENECK_LAYOUT = [
    "S.....#",
    "#####.#",
    "#.....#",
    "#.#####",
    "#.....#",
    "#####.#",
    "#....G#",
]


def make_default_grid() -> WarehouseGrid:
    return WarehouseGrid(DEFAULT_LAYOUT)
