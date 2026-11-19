"""
metrics.py
==========
Task 5 – Performance Evaluation Metrics
Author   : Sayan Majhi (BT24CSD028)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

Timing note
-----------
A single search on these grids finishes in well under a millisecond, so one
`perf_counter()` measurement is dominated by OS scheduling noise.  Everything
reported here is therefore measured over REPEATS runs and the fastest run is
kept: the path, cost and node count are identical on every run (all algorithms
are deterministic), while the timing becomes stable enough to compare.
"""

from __future__ import annotations
import os
from typing import List, Sequence

from search_algorithms import ALGORITHMS, SearchResult
from trolley_replanning import SimulationResult

REPEATS = 25


# ---------------------------------------------------------------------------
# Column widths for pretty-printing
# ---------------------------------------------------------------------------
_COL = {
    'algorithm': 9,
    'goal':      6,
    'moves':     6,
    'cost':      9,
    'nodes':     6,
    'time_ms':   10,
}


def _hr(char: str = '─') -> str:
    total = sum(_COL.values()) + len(_COL) * 3 - 1
    return char * total


def _header() -> str:
    return (
        f"{'Algorithm':<{_COL['algorithm']}} | "
        f"{'Goal?':<{_COL['goal']}} | "
        f"{'Moves':<{_COL['moves']}} | "
        f"{'Cost':<{_COL['cost']}} | "
        f"{'Nodes':<{_COL['nodes']}} | "
        f"{'Time(ms)':<{_COL['time_ms']}}"
    )


def _row(r: SearchResult) -> str:
    goal_str  = "Yes" if r.goal_reached else "No"
    moves_str = str(r.path_length) if r.goal_reached else "—"
    cost_str  = f"{r.path_cost:.1f}" if r.goal_reached else "—"
    # Two decimals: the sub-millisecond timing jitters between runs, and this
    # precision stays stable across repeats on the same machine.
    time_str  = f"{r.elapsed_sec * 1000:.2f}"
    return (
        f"{r.algorithm:<{_COL['algorithm']}} | "
        f"{goal_str:<{_COL['goal']}} | "
        f"{moves_str:<{_COL['moves']}} | "
        f"{cost_str:<{_COL['cost']}} | "
        f"{r.nodes_expanded:<{_COL['nodes']}} | "
        f"{time_str:<{_COL['time_ms']}}"
    )


# ---------------------------------------------------------------------------
# Benchmarking
# ---------------------------------------------------------------------------

def benchmark(grid, repeats: int = REPEATS) -> List[SearchResult]:
    """
    Run every registered algorithm on *grid* and keep its fastest run.

    Paths, costs and node counts are deterministic, so only the wall-clock
    timing needs repeating.
    """
    results: List[SearchResult] = []
    for fn in ALGORITHMS.values():
        best: SearchResult = fn(grid)
        for _ in range(max(0, repeats - 1)):
            candidate = fn(grid)
            if candidate.elapsed_sec < best.elapsed_sec:
                best = candidate
        results.append(best)
    return results


# ---------------------------------------------------------------------------
# Console tables
# ---------------------------------------------------------------------------

def print_comparison_table(results: Sequence[SearchResult],
                           title: str = "Algorithm Performance Comparison") -> None:
    """
    Print a formatted comparison table for the given search results.

    Columns: Algorithm | Goal? | Moves | Cost | Nodes | Time(ms)
    """
    header = _header()
    print(f"\n{'═' * len(header)}")
    print(f"  {title}")
    print('═' * len(header))
    print(header)
    print(_hr())
    for r in results:
        print(_row(r))
    print(_hr())
    _print_findings(results)


def _print_findings(results: Sequence[SearchResult]) -> None:
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


# ---------------------------------------------------------------------------
# Machine-written results file (keeps the README honest)
# ---------------------------------------------------------------------------

def results_markdown(grid,
                     results: Sequence[SearchResult],
                     sim: SimulationResult,
                     bottleneck: SimulationResult,
                     repeats: int = REPEATS) -> str:
    """Render the measured results as Markdown."""
    lines = [
        "# Measured Results",
        "",
        "> Generated automatically by `python main.py` — do not edit by hand.",
        f"> Timings are the fastest of {repeats} runs on the machine that ran the demo;",
        "> paths, costs and node counts are identical on every run.",
        "",
        f"## Default {grid.rows}×{grid.cols} warehouse grid",
        "",
        "| Algorithm | Goal reached | Moves | Path cost | Nodes expanded | Time (ms) |",
        "|---|---|:---:|---:|---:|---:|",
    ]
    for r in results:
        lines.append(
            "| **{alg}** | {goal} | {moves} | {cost} | {nodes} | {t:.2f} |".format(
                alg=r.algorithm,
                goal="Yes" if r.goal_reached else "No",
                moves=r.path_length if r.goal_reached else "—",
                cost=f"{r.path_cost:.1f}" if r.goal_reached else "—",
                nodes=r.nodes_expanded,
                t=r.elapsed_sec * 1000,
            )
        )

    lines += [
        "",
        "## Trolley replanning",
        "",
        "| Metric | Value |",
        "|---|---|",
        f"| Trolley cell | `{sim.trolley_positions}` |",
        f"| Initial path | {len(sim.initial_path) - 1} moves, cost "
        f"{sim.initial_result.path_cost:.1f} |",
        f"| Replans | {sim.replan_count} |",
        f"| Goal reached | {'Yes' if sim.goal_reached else 'No'} |",
    ]
    for ev in sim.replan_events:
        if ev.new_path:
            lines.append(
                f"| Replan #{ev.replan_no} | detected at `{ev.detected_at}` from "
                f"`{ev.current_pos}` → {len(ev.new_path) - 1} moves, cost "
                f"{ev.search_result.path_cost:.1f}, "
                f"{ev.search_result.nodes_expanded} nodes |"
            )
        else:
            lines.append(f"| Replan #{ev.replan_no} | no alternative path |")

    lines += [
        "",
        "## Blocked-corridor case (no alternative route)",
        "",
        "| Metric | Value |",
        "|---|---|",
        f"| Trolley cell | `{bottleneck.trolley_positions}` |",
        f"| Initial path | {len(bottleneck.initial_path) - 1} moves, cost "
        f"{bottleneck.initial_result.path_cost:.1f} |",
        f"| Replans | {bottleneck.replan_count} |",
        f"| Goal reached | {'Yes' if bottleneck.goal_reached else 'No'} |",
        f"| Result | {'alternative found' if bottleneck.replan_events and bottleneck.replan_events[0].new_path else '**no alternative path — destination unreachable**'} |",
        "",
    ]
    return "\n".join(lines)


def write_results_file(path: str,
                       grid,
                       results: Sequence[SearchResult],
                       sim: SimulationResult,
                       bottleneck: SimulationResult,
                       repeats: int = REPEATS) -> None:
    """Write the measured results to *path* (creating the directory)."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(results_markdown(grid, results, sim, bottleneck, repeats))
    print(f"  [saved] {path}")
