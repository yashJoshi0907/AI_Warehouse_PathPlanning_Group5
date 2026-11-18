"""
main.py
=======
Orchestrates all tasks and runs the full demonstration.
Author   : All members (integration & demo)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

Usage:
    python main.py

Output:
    Console output with grids, tables, and summaries.
    Screenshots saved to screenshots/ directory.
"""

from __future__ import annotations
import os
import sys

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
from warehouse_grid import (
    WarehouseGrid, make_default_grid,
    DEFAULT_LAYOUT, OPEN, CROWDED, SHELF, RESTRICTED, START, GOAL, TROLLEY,
    CELL_COST
)
from search_algorithms import bfs, dfs, greedy, astar, run_all, manhattan
from trolley_replanning import TrolleySimulator
from metrics import print_comparison_table, print_trolley_metrics
from visualization import (
    save_grid_overview,
    save_algorithm_routes,
    save_trolley_demo,
    save_performance_chart,
)

# ---------------------------------------------------------------------------
# Change working directory so relative paths work from any launch location
# ---------------------------------------------------------------------------
os.chdir(os.path.dirname(os.path.abspath(__file__)))


# ===========================================================================
# SECTION 1 – Environment Overview
# ===========================================================================

def section1_environment() -> WarehouseGrid:
    print("\n" + "=" * 65)
    print("  TASK 1 & 2 – WAREHOUSE ENVIRONMENT & PROBLEM FORMULATION")
    print("=" * 65)

    grid = make_default_grid()
    grid.display(label="Default 10×10 Warehouse Grid")

    print("  Problem Formulation (Task 2)")
    print("  ─────────────────────────────")
    print(f"  State space      : (row, col) integer pairs")
    print(f"  Initial state    : {grid.start}  [S]")
    print(f"  Goal state       : {grid.goal}  [G]")
    print(f"  Actions          : UP / DOWN / LEFT / RIGHT (4-connectivity)")
    print(f"  Transition model : deterministic adjacent-cell move")
    print(f"  Goal test        : state == {grid.goal}")
    print(f"  Path-cost fn     : sum of entry costs (open=1, crowded=4)")
    print()
    print("  Cell type legend:")
    print("    '.' Open aisle (cost 1)   '#' Shelf (impassable)")
    print("    'C' Crowded   (cost 4)    'R' Restricted (impassable)")
    print("    'S' Start     (cost 1)    'T' Trolley (impassable when detected)")
    print("    'G' Goal      (cost 1)")
    print()
    return grid


# ===========================================================================
# SECTION 3 – Search Algorithms
# ===========================================================================

def section3_search(grid: WarehouseGrid) -> list:
    print("=" * 65)
    print("  TASK 3 – SEARCH ALGORITHMS")
    print("=" * 65)

    results = run_all(grid)

    for res in results:
        print(f"\n  ── {res.algorithm} ──")
        grid.display(path=res.path, label=f"{res.algorithm} path")
        if res.goal_reached:
            print(f"  Path   : {res.path}")
            print(f"  Moves  : {res.path_length}   Cost : {res.path_cost:.1f}")
            print(f"  Nodes  : {res.nodes_expanded}   Time : {res.elapsed_sec*1000:.3f} ms")
        else:
            print("  No path found.")

    # Heuristic explanation
    print("\n  Heuristic: Manhattan Distance Admissibility Proof")
    print("  " + "─" * 55)
    print("  h(n) = |row_n − row_goal| + |col_n − col_goal|")
    print()
    print("  In a 4-connected grid the minimum moves from (r,c) to (gr,gc)")
    print("  is exactly |r-gr|+|c-gc|.  Each move enters a cell with entry")
    print("  cost ≥ 1 (minimum possible).  Therefore:")
    print()
    print("      g*(n) ≥ |Δrow| + |Δcol| = h(n)")
    print()
    print("  h never over-estimates the true remaining cost → h is admissible.")
    print("  An admissible heuristic guarantees A* returns the optimal path.")
    print()

    # Cost-vs-moves demo
    print("  Cost ≠ Moves Demo")
    print("  " + "─" * 55)
    _cost_vs_moves_demo()

    return results


def _cost_vs_moves_demo() -> None:
    """Illustrate that the fewest-moves path is not always the cheapest."""
    demo_layout = [
        "S.CCCC....G",
        "...........",
    ]
    demo_grid = WarehouseGrid(demo_layout)
    demo_grid.display(label="Cost-vs-Moves demo grid (2 rows)")

    bfs_r  = bfs(demo_grid)
    astar_r = astar(demo_grid)

    print(f"  BFS  (fewest moves): path={bfs_r.path}")
    print(f"       moves={bfs_r.path_length}  cost={bfs_r.path_cost:.1f}")
    print(f"  A*   (lowest cost) : path={astar_r.path}")
    print(f"       moves={astar_r.path_length}  cost={astar_r.path_cost:.1f}")
    if bfs_r.path_cost > astar_r.path_cost:
        print("  → A* found a LOWER-COST route by going around crowded cells,")
        print("    even though it takes more moves. Shortest ≠ Cheapest.\n")
    else:
        print("  → On this grid fewest-moves happens to be cheapest too.\n")


# ===========================================================================
# SECTION 4 – Trolley Detection & Replanning
# ===========================================================================

def section4_trolley(grid: WarehouseGrid) -> TrolleySimulator:
    print("=" * 65)
    print("  TASK 4 – TROLLEY DETECTION & REPLANNING")
    print("=" * 65)

    # Build simulator; trolley auto-placed on initial A* route
    sim_obj = TrolleySimulator(grid)
    result  = sim_obj.run()
    result.print_summary()
    print_trolley_metrics(result)

    # Show grids
    base_grid = sim_obj.base_grid
    base_grid.display(path=sim_obj.initial_path,
                      label="Initial A* plan (before trolley detected)")

    real_grid = sim_obj.real_grid
    for ev in result.replan_events:
        real_grid.display(path=ev.new_path,
                          label=f"Replanned route (after detecting trolley at {ev.detected_at})")

    return sim_obj, result


# ===========================================================================
# SECTION 5 – Performance Metrics & Visualisation
# ===========================================================================

def section5_metrics_and_viz(grid: WarehouseGrid,
                              results: list,
                              sim_obj,
                              sim_result) -> None:
    print("=" * 65)
    print("  TASK 5 – PERFORMANCE METRICS & VISUALISATION")
    print("=" * 65)

    print_comparison_table(results)

    # Save all screenshots
    print("  Generating screenshots …")
    save_grid_overview(grid)
    save_algorithm_routes(grid, results)
    save_trolley_demo(sim_obj.base_grid, sim_result)
    save_performance_chart(results)
    print("  All screenshots saved to screenshots/\n")


# ===========================================================================
# MAIN
# ===========================================================================

def main() -> None:
    if "--gui" in sys.argv:
        try:
            import gui
            print("Launching Interactive GUI Application...")
            gui.main()
            return
        except Exception as e:
            print(f"Error launching GUI: {e}")
            sys.exit(1)

    print("\n" + "█" * 65)
    print("  AI-BASED ROUTE PLANNING FOR A WAREHOUSE DELIVERY ROBOT")
    print("  Group 5 — BT24CSD024 / 028 / 029 / 031 / 032 / 033")
    print("█" * 65)

    # Task 1 & 2
    grid = section1_environment()

    # Task 3
    results = section3_search(grid)

    # Task 4
    sim_obj, sim_result = section4_trolley(grid)

    # Task 5
    section5_metrics_and_viz(grid, results, sim_obj, sim_result)

    print("=" * 65)
    print("  Demo complete. See screenshots/ for visual output.")
    print("  Tip: Run 'python gui.py' or 'python main.py --gui' to launch the interactive window!")
    print("=" * 65 + "\n")


if __name__ == '__main__':
    main()
