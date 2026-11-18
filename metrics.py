"""
metrics.py
==========
Task 5 – Performance Evaluation Metrics
Author   : Sayan Majhi (BT24CSD028)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot
"""

from __future__ import annotations
from typing import List, Optional
import textwrap

from search_algorithms import SearchResult
from trolley_replanning import SimulationResult


# ---------------------------------------------------------------------------
# Column widths for pretty-printing
# ---------------------------------------------------------------------------
_COL = {
    'algorithm': 8,
    'goal':      7,
    'moves':     7,
    'cost':      10,
    'nodes':     8,
    'time_ms':   10,
}


def _hr(char: str = '─') -> str:
    total = sum(_COL.values()) + len(_COL) * 3 + 1
    return char * total


def print_comparison_table(results: List[SearchResult],
                           title: str = "Algorithm Performance Comparison") -> None:
    """
    Print a formatted comparison table for the given search results.

    Columns
    -------
    Algorithm | Goal? | Moves | Path Cost | Nodes | Time(ms)
    """
    header = (
        f"{'Algorithm':<{_COL['algorithm']}} | "
        f"{'Goal?':<{_COL['goal']}} | "
        f"{'Moves':<{_COL['moves']}} | "
        f"{'Path Cost':<{_COL['cost']}} | "
        f"{'Nodes':<{_COL['nodes']}} | "
        f"{'Time(ms)':<{_COL['time_ms']}}"
    )

    print(f"\n{'═' * len(header)}")
    print(f"  {title}")
    print('═' * len(header))
    print(header)
    print(_hr())

    for r in results:
        goal_str  = "Yes" if r.goal_reached else "No"
        moves_str = str(r.path_length) if r.goal_reached else "—"
        cost_str  = f"{r.path_cost:.1f}" if r.goal_reached else "—"
        nodes_str = str(r.nodes_expanded)
        time_str  = f"{r.elapsed_sec * 1000:.3f}"

        print(
            f"{r.algorithm:<{_COL['algorithm']}} | "
            f"{goal_str:<{_COL['goal']}} | "
            f"{moves_str:<{_COL['moves']}} | "
            f"{cost_str:<{_COL['cost']}} | "
            f"{nodes_str:<{_COL['nodes']}} | "
            f"{time_str:<{_COL['time_ms']}}"
        )

    print(_hr())
    _print_findings(results)


def _print_findings(results: List[SearchResult]) -> None:
    """Print brief analysis under the table."""
    successful = [r for r in results if r.goal_reached]
    if not successful:
        print("\n  No algorithm found a path.\n")
        return

    min_moves = min(r.path_length for r in successful)
    min_cost  = min(r.path_cost   for r in successful)
    min_nodes = min(r.nodes_expanded for r in successful)

    best_moves = [r.algorithm for r in successful if r.path_length == min_moves]
    best_cost  = [r.algorithm for r in successful if r.path_cost == min_cost]
    best_nodes = [r.algorithm for r in successful if r.nodes_expanded == min_nodes]

    findings = [
        f"Fewest moves   : {', '.join(best_moves)} ({min_moves} moves)",
        f"Lowest cost    : {', '.join(best_cost)} (cost {min_cost:.1f})",
        f"Fewest nodes   : {', '.join(best_nodes)} ({min_nodes} nodes expanded)",
    ]

    if set(best_moves) != set(best_cost):
        findings.append(
            "→ Shortest-move path ≠ lowest-cost path: crowded cells inflate cost "
            "along the fewest-move route."
        )
    else:
        findings.append(
            "→ Shortest-move path = lowest-cost path on this grid."
        )

    print("\n  Key findings:")
    for f in findings:
        print(f"    • {f}")
    print()


def print_trolley_metrics(sim: SimulationResult) -> None:
    """Print a summary table for the trolley replanning experiment."""
    print(f"\n{'═' * 55}")
    print("  TROLLEY REPLANNING METRICS")
    print('═' * 55)
    print(f"  {'Metric':<30} {'Value'}")
    print('─' * 55)

    rows = [
        ("Trolley position(s)",    str(sim.trolley_positions)),
        ("Initial path cost",      f"{sim.initial_result.path_cost:.1f}"),
        ("Initial path length",    str(len(sim.initial_path) - 1) + " moves"),
        ("Number of replans",      str(sim.replan_count)),
        ("Goal reached",           "Yes ✓" if sim.goal_reached else "No ✗"),
    ]

    for ev in sim.replan_events:
        prefix = f"  Replan #{ev.replan_no}"
        if ev.new_path:
            rows.append((f"{prefix} cost",   f"{ev.search_result.path_cost:.1f}"))
            rows.append((f"{prefix} moves",  f"{len(ev.new_path) - 1}"))
            rows.append((f"{prefix} nodes",  str(ev.search_result.nodes_expanded)))
        else:
            rows.append((f"{prefix}", "NO PATH FOUND"))

    for label, value in rows:
        print(f"  {label:<30} {value}")

    print('═' * 55 + "\n")
