"""
main.py
=======
Orchestrates all five tasks and runs the full demonstration.

Project  : AI-Based Route Planning for a Warehouse Delivery Robot
Group 5  : BT24CSD024 / 028 / 029 / 031 / 032 / 033

Usage:
    python main.py            full CLI report + all figures
    python main.py --gui      interactive Tkinter simulator

Output:
    Console report (grids, tables, findings)
    screenshots/*.png         labelled figures
    results/performance_results.md   measured numbers (auto-generated)
"""

from __future__ import annotations
import os
import sys
from typing import Tuple

# ---------------------------------------------------------------------------
# Make console output UTF-8 safe on Windows (cp1252 terminals / redirected
# pipes).  The report uses box-drawing characters and arrows, which would
# otherwise raise UnicodeEncodeError on a non-UTF-8 stdout.
# ---------------------------------------------------------------------------
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError, OSError):
        pass

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
from warehouse_grid import (
    WarehouseGrid, make_default_grid,
    COST_TEST_LAYOUT, BOTTLENECK_LAYOUT,
    OPEN, CROWDED, SHELF, RESTRICTED, TROLLEY,
)
from search_algorithms import bfs, astar
from trolley_replanning import TrolleySimulator, SimulationResult
from metrics import (
    REPEATS, benchmark, print_comparison_table, print_trolley_metrics,
    write_results_file,
)
from visualization import (
    save_grid_overview,
    save_algorithm_routes,
    save_cost_vs_moves,
    save_trolley_demo,
    save_no_alternative,
    save_performance_chart,
)

RESULTS_FILE = os.path.join('results', 'performance_results.md')

# ---------------------------------------------------------------------------
# Change working directory so relative paths work from any launch location
# ---------------------------------------------------------------------------
os.chdir(os.path.dirname(os.path.abspath(__file__)))


def _banner(title: str, char: str = '═') -> None:
    print(f"\n{char * 72}")
    print(f"  {title}")
    print(char * 72)


# ===========================================================================
# SECTION 1 & 2 – Environment and problem formulation
# ===========================================================================

def section_environment() -> WarehouseGrid:
    _banner("TASK 1 & 2 – WAREHOUSE ENVIRONMENT & PROBLEM FORMULATION")

    grid = make_default_grid()
    grid.display(label=f"Default {grid.rows}×{grid.cols} warehouse grid")

    print("  Cell inventory")
    print("  " + "─" * 40)
    for symbol, name in [
        (OPEN, 'open aisles      (cost 1)'),
        (CROWDED, 'crowded aisles   (cost 4)'),
        (SHELF, 'shelves          (impassable)'),
        (RESTRICTED, 'restricted zones (impassable)'),
        (TROLLEY, 'trolleys         (impassable once detected)'),
    ]:
        print(f"    {symbol}  {grid.count(symbol):2}  {name}")
    print()

    print("  Problem formulation (Task 2)")
    print("  " + "─" * 40)
    print("  State space      : (row, col) integer pairs inside the grid")
    print(f"  Initial state    : {grid.start}  [S]")
    print(f"  Goal state       : {grid.goal}  [G]")
    print("  Actions          : UP / DOWN / LEFT / RIGHT (4-connectivity)")
    print("  Transition model : deterministic adjacent-cell move")
    print(f"  Goal test        : state == {grid.goal}")
    print("  Path-cost fn     : sum of entry costs (open = 1, crowded = 4,")
    print("                     shelves / restricted zones = impassable)")
    print()
    return grid


# ===========================================================================
# TASK 3 – Search algorithms
# ===========================================================================

def section_search(grid: WarehouseGrid) -> list:
    _banner("TASK 3 – SEARCH ALGORITHMS (BFS, DFS, GREEDY, A*)")

    results = benchmark(grid, REPEATS)

    for res in results:
        print(f"\n  ── {res.algorithm} ──")
        grid.display(path=res.path, label=f"{res.algorithm} route")
        if res.goal_reached:
            print(f"  Moves : {res.path_length}    Cost : {res.path_cost:.1f}")
            print(f"  Nodes : {res.nodes_expanded}    Time : {res.elapsed_sec * 1000:.3f} ms")

    print("\n  Heuristic – why Manhattan distance is a lower bound")
    print("  " + "─" * 55)
    print("  h(n) = |row − row_goal| + |col − col_goal|")
    print("  A 4-connected grid needs at least h(n) moves to reach the goal and")
    print("  every move enters a cell of cost ≥ 1, so the true remaining cost")
    print("  satisfies g*(n) ≥ h(n).  h never over-estimates → admissible, and")
    print("  since a single step changes h by at most 1 it is also consistent,")
    print("  which guarantees A* returns a lowest-cost path.\n")

    _cost_vs_moves_demo()
    return results


def _cost_vs_moves_demo() -> None:
    """Illustrate that the fewest-moves path is not always the cheapest."""
    print("  Cost ≠ Moves demo (Task 3 requirement)")
    print("  " + "─" * 55)
    demo_grid = WarehouseGrid(COST_TEST_LAYOUT)
    demo_grid.display(label="Crowded top corridor vs. open lower detour")

    bfs_r = bfs(demo_grid)
    a_star_r = astar(demo_grid)

    print(f"  BFS (fewest moves): {bfs_r.path_length} moves, cost {bfs_r.path_cost:.1f}")
    print(f"  A*  (lowest cost) : {a_star_r.path_length} moves, cost {a_star_r.path_cost:.1f}")
    if a_star_r.path_cost < bfs_r.path_cost:
        print(f"  → A* spends {a_star_r.path_length - bfs_r.path_length} extra move(s) to save "
              f"{bfs_r.path_cost - a_star_r.path_cost:.1f} in cost:")
        print("    the shortest route is not the cheapest route.\n")
    else:
        print("  → On this grid the fewest-moves route is also the cheapest.\n")


# ===========================================================================
# TASK 4 – Trolley detection and replanning
# ===========================================================================

def section_trolley(grid: WarehouseGrid) -> Tuple[TrolleySimulator, SimulationResult, SimulationResult]:
    _banner("TASK 4 – TROLLEY DETECTION, MAP UPDATE & REPLANNING")

    sim = TrolleySimulator(grid)
    result = sim.run()
    result.print_summary()
    print_trolley_metrics(result)

    sim.base_grid.display(path=sim.initial_path,
                          label="Initial A* plan (trolley still unknown)")
    for ev in result.replan_events:
        sim.real_grid.display(path=ev.new_path,
                              label=f"Replanned route after detecting the trolley at {ev.detected_at}")

    bottleneck = _blocked_corridor_demo()
    return sim, result, bottleneck


def _blocked_corridor_demo() -> SimulationResult:
    """The Task 4 edge case: replanning fails because no alternative exists."""
    _banner("TASK 4 – EDGE CASE: NO ALTERNATIVE ROUTE", '─')

    grid = WarehouseGrid(BOTTLENECK_LAYOUT)
    grid.display(label="Single-corridor warehouse (goal has one approach)")

    sim = TrolleySimulator(grid)
    result = sim.run()
    result.print_summary()

    if not result.goal_reached and result.replan_events:
        ev = result.replan_events[-1]
        print(f"  Replan #{ev.replan_no} returned no path from {ev.current_pos}:")
        print("  the trolley at", ev.detected_at, "severs the only corridor to G.")
        print("  The robot stops and reports the destination as unreachable.\n")
    return result


# ===========================================================================
# TASK 5 – Metrics, figures and generated results
# ===========================================================================

def section_metrics(grid: WarehouseGrid,
                    results: list,
                    sim: TrolleySimulator,
                    sim_result: SimulationResult,
                    bottleneck: SimulationResult) -> None:
    _banner("TASK 5 – PERFORMANCE METRICS, VISUALISATION & RESULTS FILE")

    print_comparison_table(results, title="Algorithm Performance Comparison — default grid")

    print(f"  Generating figures ({REPEATS} timed runs per algorithm for the tables)…")
    save_grid_overview(grid)
    save_algorithm_routes(grid, results)
    save_cost_vs_moves(WarehouseGrid(COST_TEST_LAYOUT), bfs, astar)
    save_trolley_demo(sim.base_grid, sim_result)
    save_no_alternative(WarehouseGrid(BOTTLENECK_LAYOUT), bottleneck)
    save_performance_chart(results)
    print("  All figures saved to screenshots/\n")

    write_results_file(RESULTS_FILE, grid, results, sim_result, bottleneck, REPEATS)


# ===========================================================================
# MAIN
# ===========================================================================

def main(argv: list | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)

    if '--gui' in argv:
        try:
            import gui
            print("Launching interactive GUI…")
            gui.main()
            return
        except Exception as exc:                      # pragma: no cover - GUI dependent
            print(f"Could not launch the GUI: {exc}")
            sys.exit(1)

    print("\n" + "█" * 72)
    print("  AI-BASED ROUTE PLANNING FOR A WAREHOUSE DELIVERY ROBOT")
    print("  Group 5 — BT24CSD024 / 028 / 029 / 031 / 032 / 033")
    print("  BFS · DFS · Greedy Best-First · A* · online replanning")
    print("█" * 72)

    grid = section_environment()                        # Tasks 1 & 2
    results = section_search(grid)                      # Task 3
    sim, sim_result, bottleneck = section_trolley(grid)  # Task 4 (+ edge case)
    section_metrics(grid, results, sim, sim_result, bottleneck)   # Task 5

    _banner("DEMO COMPLETE")
    print("  Figures        : screenshots/")
    print("  Measured data  : results/performance_results.md")
    print("  Interactive app: python gui.py   or   python main.py --gui")
    print("█" * 72 + "\n")


if __name__ == '__main__':
    main()
