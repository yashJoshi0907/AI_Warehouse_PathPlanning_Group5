"""
visualization.py
================
Task 5 – Grid rendering, route overlays and performance charts
Author   : Sayan Majhi (BT24CSD028)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

All figures use the Matplotlib 'Agg' backend so they can be written to disk
headlessly.  Each figure follows the same visual language: light tiles with
rounded corners, a muted palette, column/row rulers and a legend, so the
screenshots read as one set.
"""

from __future__ import annotations
import os
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use('Agg')          # headless backend for file output
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Patch
import numpy as np

from warehouse_grid import (
    WarehouseGrid, OPEN, CROWDED, SHELF, RESTRICTED, START, GOAL, TROLLEY,
)
from search_algorithms import SearchResult
from trolley_replanning import SimulationResult

# ---------------------------------------------------------------------------
# Palette
# ---------------------------------------------------------------------------
INK        = '#1B1F2A'
MUTED      = '#6B7385'
PAPER      = '#FFFFFF'
PANEL      = '#F7F8FC'
RULE       = '#D9DEE9'

CELL_COLOURS: Dict[str, str] = {
    OPEN:       '#F1F4FA',
    CROWDED:    '#FFD9A0',
    SHELF:      '#2E3550',
    RESTRICTED: '#F2A7B3',
    START:      '#1FA97F',
    GOAL:       '#2D6CDF',
    TROLLEY:    '#E0414A',
}
CELL_TEXT: Dict[str, str] = {
    CROWDED:    '#8A5200',
    SHELF:      '#FFFFFF',
    RESTRICTED: '#7A1F2B',
    START:      '#FFFFFF',
    GOAL:       '#FFFFFF',
    TROLLEY:    '#FFFFFF',
}

ALGO_COLOURS: Dict[str, str] = {
    'BFS':    '#4C6EF5',
    'DFS':    '#F08C00',
    'Greedy': '#12B886',
    'A*':     '#D6336C',
}
PATH_COLOUR   = '#6C5CE7'
REPLAN_COLOUR = '#E8590C'
TRACE_COLOUR  = '#0CA678'
ALERT         = '#E03131'

LEGEND_ITEMS = [
    (OPEN,       'Open aisle · cost 1'),
    (CROWDED,    'Crowded · cost 4'),
    (SHELF,      'Shelf · blocked'),
    (RESTRICTED, 'Restricted · blocked'),
    (START,      'Start (S)'),
    (GOAL,       'Goal (G)'),
    (TROLLEY,    'Trolley (T)'),
]


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _apply_style() -> None:
    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 9,
        'axes.edgecolor': RULE,
        'axes.labelcolor': INK,
        'text.color': INK,
        'figure.facecolor': PAPER,
        'axes.facecolor': PAPER,
        'savefig.facecolor': PAPER,
    })


def _cell_color(sym: str) -> str:
    return CELL_COLOURS.get(sym, '#E9ECF3')


def _cell_text(sym: str) -> str:
    if sym == START:
        return 'S'
    if sym == GOAL:
        return 'G'
    if sym == CROWDED:
        return '4'
    if sym == TROLLEY:
        return 'T'
    return ''


def _draw_grid(ax: plt.Axes,
               grid: WarehouseGrid,
               path: Optional[Sequence[Tuple[int, int]]] = None,
               path_color: str = PATH_COLOUR,
               extra_markers: Optional[Dict[Tuple[int, int], str]] = None,
               dim_tiles: bool = False) -> None:
    """
    Draw *grid* onto *ax* with rounded tiles and an optional route overlay.

    path          : cells to connect with a coloured route line.
    extra_markers : {(r, c): colour} for alert markers (e.g. detected trolley).
    dim_tiles     : mute the cell colours (used behind a highlighted route).
    """
    rows, cols = grid.rows, grid.cols
    extra_markers = extra_markers or {}
    gap = 0.07

    for r in range(rows):
        for c in range(cols):
            sym = grid.cell(r, c)
            colour = _cell_color(sym)
            if dim_tiles and sym not in (START, GOAL, SHELF, RESTRICTED, TROLLEY):
                colour = '#F7F8FC'
            tile = FancyBboxPatch(
                (c + gap, r + gap), 1 - 2 * gap, 1 - 2 * gap,
                boxstyle='round,pad=0,rounding_size=0.10',
                linewidth=0, facecolor=colour, zorder=1,
            )
            ax.add_patch(tile)

            label = _cell_text(sym)
            if label:
                ax.text(c + 0.5, r + 0.5, label,
                        ha='center', va='center', zorder=2,
                        fontsize=8.5 if label in ('S', 'G') else 7,
                        fontweight='bold',
                        color=CELL_TEXT.get(sym, INK))

    # Route overlay
    if path:
        xs = [c + 0.5 for _, c in path]
        ys = [r + 0.5 for r, _ in path]
        ax.plot(xs, ys, '-', color=path_color, linewidth=3.2, zorder=3,
                solid_capstyle='round', solid_joinstyle='round', alpha=0.95)
        ax.plot(xs, ys, 'o', color=PAPER, markersize=4.2, zorder=4)
        ax.plot(xs, ys, 'o', color=path_color, markersize=2.2, zorder=5)

    # Alert markers
    for (r, c), colour in extra_markers.items():
        ax.plot(c + 0.5, r + 0.5, marker='X', color=colour, markersize=13,
                markeredgecolor=PAPER, markeredgewidth=1.2, zorder=7)

    # Rulers
    ax.set_xlim(0, cols)
    ax.set_ylim(rows, 0)
    ax.set_aspect('equal')
    ax.set_xticks([c + 0.5 for c in range(cols)])
    ax.set_yticks([r + 0.5 for r in range(rows)])
    ax.set_xticklabels([str(c) for c in range(cols)], fontsize=6.5, color=MUTED)
    ax.set_yticklabels([str(r) for r in range(rows)], fontsize=6.5, color=MUTED)
    ax.xaxis.tick_top()
    ax.tick_params(length=0, pad=2)
    for spine in ax.spines.values():
        spine.set_visible(False)


def _legend(fig: plt.Figure,
            items: Sequence[Tuple[str, str]] = LEGEND_ITEMS,
            extra: Sequence[Tuple[str, str]] = (),
            ncol: int = 4, y: float = 0.0) -> None:
    """Draw the shared cell legend at the bottom of *fig*."""
    handles = [
        Patch(facecolor=CELL_COLOURS[k], edgecolor=RULE, label=label, linewidth=0.6)
        for k, label in items
    ]
    handles += [Patch(facecolor=c, edgecolor='none', label=t) for c, t in extra]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(0.5, y),
               ncol=ncol, frameon=False, fontsize=7.5, handlelength=1.1,
               columnspacing=1.6, labelcolor=INK)


def _panel_title(ax: plt.Axes, title: str, subtitle: str = '',
                 colour: str = INK) -> None:
    ax.set_title(title, fontsize=11, fontweight='bold', color=colour, pad=16)
    if subtitle:
        ax.set_xlabel(subtitle, fontsize=8, color=MUTED, labelpad=8)


def _footer(fig: plt.Figure, text: str, y: float = -0.03) -> None:
    fig.text(0.5, y, text, ha='center', fontsize=7, color=MUTED)


def _save(fig: plt.Figure, filepath: str, pad: float = 0.6) -> None:
    os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
    fig.savefig(filepath, dpi=170, bbox_inches='tight', pad_inches=pad,
                facecolor=PAPER)
    plt.close(fig)
    print(f"  [saved] {filepath}")


def _algo_colour(name: str) -> str:
    return ALGO_COLOURS.get(name, PATH_COLOUR)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def save_grid_overview(grid: WarehouseGrid,
                       filepath: str = 'screenshots/grid_overview.png') -> None:
    """The warehouse floor plan with its cell inventory."""
    _apply_style()
    fig, ax = plt.subplots(figsize=(6.4, 6.6))
    _draw_grid(ax, grid)

    title = f"Warehouse floor plan · {grid.rows}×{grid.cols}"
    stats = (f"{grid.count(OPEN)} open · {grid.count(CROWDED)} crowded · "
             f"{grid.count(SHELF)} shelves · {grid.count(RESTRICTED)} restricted")
    ax.set_title(title, fontsize=13, fontweight='bold', color=INK, pad=18)
    ax.set_xlabel(stats, fontsize=8, color=MUTED, labelpad=10)

    _legend(fig, y=0.0, ncol=4)
    _footer(fig, "Entering an open aisle costs 1, a crowded aisle costs 4; "
                 "shelves, restricted zones and detected trolleys are impassable.",
            y=-0.055)
    fig.subplots_adjust(bottom=0.20)
    _save(fig, filepath)


def save_algorithm_routes(grid: WarehouseGrid,
                          results: Sequence[SearchResult],
                          filepath: str = 'screenshots/algorithm_routes.png') -> None:
    """2×2 comparison of the four search algorithms on the same grid."""
    _apply_style()
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 11.8))
    axes = axes.flatten()

    for ax, res in zip(axes, results):
        colour = _algo_colour(res.algorithm)
        _draw_grid(ax, grid, path=res.path, path_color=colour, dim_tiles=True)
        if res.goal_reached:
            subtitle = (f"{res.path_length} moves · cost {res.path_cost:.1f} · "
                        f"{res.nodes_expanded} nodes · {res.elapsed_sec * 1000:.2f} ms")
        else:
            subtitle = "no path found"
        _panel_title(ax, res.algorithm, subtitle, colour if res.goal_reached else ALERT)

    fig.suptitle("Route comparison across the four search algorithms",
                 fontsize=14, fontweight='bold', color=INK, y=0.98)
    _legend(fig, y=0.0, ncol=4)
    _footer(fig, "Every route starts at S and ends at G; colour identifies the algorithm.",
            y=-0.035)
    fig.subplots_adjust(top=0.92, bottom=0.12, hspace=0.28, wspace=0.18)
    _save(fig, filepath)


def save_cost_vs_moves(grid: WarehouseGrid,
                       bfs_fn: Callable[[WarehouseGrid], SearchResult],
                       astar_fn: Callable[[WarehouseGrid], SearchResult],
                       filepath: str = 'screenshots/cost_vs_moves.png') -> None:
    """
    Side-by-side proof that the fewest-move route is not the cheapest:
    BFS takes the crowded top corridor, A* detours through the open row.
    """
    _apply_style()
    bfs_r = bfs_fn(grid)
    astar_r = astar_fn(grid)

    fig, axes = plt.subplots(1, 2, figsize=(10.6, 2.6))
    for ax, res, note in (
        (axes[0], bfs_r, "fewest moves, but crosses four cost-4 cells"),
        (axes[1], astar_r, "two more moves, yet avoids every crowded cell"),
    ):
        colour = _algo_colour(res.algorithm)
        _draw_grid(ax, grid, path=res.path, path_color=colour, dim_tiles=True)
        _panel_title(
            ax,
            f"{res.algorithm} · {res.path_length} moves · cost {res.path_cost:.1f}",
            note, colour,
        )

    fig.suptitle("Shortest route ≠ cheapest route\n"
                 "Same start, same goal: BFS minimises moves, A* minimises weighted cost",
                 fontsize=13, fontweight='bold', color=INK, y=1.14)
    _legend(fig, items=LEGEND_ITEMS[:2] + LEGEND_ITEMS[4:6], y=-0.26, ncol=4)
    fig.subplots_adjust(bottom=0.08, wspace=0.12)
    _save(fig, filepath)


def _grid_with_trolleys(base_grid: WarehouseGrid,
                        sim: SimulationResult) -> WarehouseGrid:
    world = base_grid.clone()
    for pos in sim.trolley_positions:
        world.mark_impassable(*pos)
    return world


def save_trolley_demo(base_grid: WarehouseGrid,
                      sim: SimulationResult,
                      filepath: str = 'screenshots/trolley_replanning.png') -> None:
    """
    Sequence figure: initial plan → detection & replan → final robot trace.
    """
    _apply_style()
    world = _grid_with_trolleys(base_grid, sim)
    panels = 2 + len(sim.replan_events)

    fig, axes = plt.subplots(1, panels, figsize=(5.2 * panels, 5.6))
    axes = np.atleast_1d(axes)

    trolley_markers = {pos: CELL_COLOURS[TROLLEY] for pos in sim.trolley_positions}

    # Panel 0 – initial plan
    _draw_grid(axes[0], base_grid, path=sim.initial_path,
               path_color=PATH_COLOUR, extra_markers=trolley_markers)
    _panel_title(
        axes[0], "1 · Initial A* plan",
        f"{len(sim.initial_path) - 1} moves · cost {sim.initial_result.path_cost:.1f}  "
        f"(✕ = hidden trolley)",
        PATH_COLOUR,
    )

    # Replan panels
    for i, ev in enumerate(sim.replan_events):
        ax = axes[i + 1]
        _draw_grid(ax, world, path=ev.new_path, path_color=REPLAN_COLOUR,
                   extra_markers={ev.detected_at: ALERT})
        if ev.new_path:
            _panel_title(
                ax, f"{i + 2} · Replan #{ev.replan_no}",
                f"detected at {ev.detected_at} from {ev.current_pos} → "
                f"{len(ev.new_path) - 1} moves · cost {ev.search_result.path_cost:.1f}",
                REPLAN_COLOUR,
            )
        else:
            _panel_title(ax, f"{i + 2} · Replan #{ev.replan_no}",
                         f"detected at {ev.detected_at} — no alternative path",
                         ALERT)

    # Final panel – executed trace
    _draw_grid(axes[-1], world, path=sim.final_trace, path_color=TRACE_COLOUR)
    _panel_title(
        axes[-1], "Executed route",
        f"{len(sim.final_trace) - 1} moves · {sim.replan_count} replan"
        f"{'s' if sim.replan_count != 1 else ''} · "
        f"{'goal reached' if sim.goal_reached else 'goal NOT reached'}",
        TRACE_COLOUR if sim.goal_reached else ALERT,
    )

    fig.suptitle("Dynamic trolley: lookahead detection, map update, online A* replanning",
                 fontsize=14, fontweight='bold', color=INK, y=1.0)
    _legend(fig, extra=[(ALERT, 'Detection point')], y=0.0, ncol=4)
    fig.subplots_adjust(bottom=0.15, wspace=0.10)
    _save(fig, filepath)


def save_no_alternative(grid: WarehouseGrid,
                        sim: SimulationResult,
                        filepath: str = 'screenshots/no_alternative_path.png') -> None:
    """
    The Task 4 failure case: the only corridor is blocked, so the replanned
    search returns no path and the robot reports the goal as unreachable.
    """
    _apply_style()
    world = _grid_with_trolleys(grid, sim)
    ev = sim.replan_events[-1] if sim.replan_events else None

    fig, axes = plt.subplots(1, 2, figsize=(10.4, 5.0))
    _draw_grid(axes[0], grid, path=sim.initial_path, path_color=PATH_COLOUR,
               extra_markers={p: CELL_COLOURS[TROLLEY] for p in sim.trolley_positions})
    _panel_title(axes[0], "Initial plan",
                 f"{len(sim.initial_path) - 1} moves · cost "
                 f"{sim.initial_result.path_cost:.1f} · single corridor",
                 PATH_COLOUR)

    _draw_grid(axes[1], world, path=sim.final_trace, path_color=TRACE_COLOUR,
               extra_markers={ev.detected_at: ALERT} if ev else None)
    _panel_title(
        axes[1], "Replan fails — destination unreachable",
        (f"detected at {ev.detected_at} from {ev.current_pos} · no alternative route"
         if ev else "no replan recorded"),
        ALERT,
    )

    fig.suptitle("Blocked corridor: the goal has no second approach",
                 fontsize=14, fontweight='bold', color=INK, y=1.02)
    _legend(fig, y=-0.03, ncol=4)
    _footer(fig, "When A* returns no path the robot halts and reports the failure "
                 "instead of entering the blocked cell.", y=-0.095)
    fig.subplots_adjust(bottom=0.18, wspace=0.10)
    _save(fig, filepath)


def save_performance_chart(results: Sequence[SearchResult],
                           filepath: str = 'screenshots/performance_chart.png') -> None:
    """Three clean bar charts: nodes expanded, cost vs moves, execution time."""
    _apply_style()

    names  = [r.algorithm for r in results]
    colours = [_algo_colour(n) for n in names]
    nodes  = [r.nodes_expanded for r in results]
    costs  = [r.path_cost if r.goal_reached else 0 for r in results]
    moves  = [r.path_length for r in results]
    times  = [r.elapsed_sec * 1000 for r in results]
    x = np.arange(len(names))

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))

    def _clean(ax, title: str, ylabel: str) -> None:
        ax.set_title(title, fontsize=11, fontweight='bold', color=INK, pad=10)
        ax.set_ylabel(ylabel, fontsize=8, color=MUTED)
        ax.set_xticks(x)
        ax.set_xticklabels(names, fontsize=9)
        ax.grid(axis='y', color=RULE, linewidth=0.8, alpha=0.9)
        ax.set_axisbelow(True)
        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)
        ax.spines['bottom'].set_color(RULE)
        ax.tick_params(length=0, colors=MUTED)
        ax.set_ylim(0, max(ax.get_ylim()[1], 1) * 1.18)

    # Nodes expanded
    bars = axes[0].bar(x, nodes, width=0.58, color=colours, zorder=3)
    _clean(axes[0], "Work done", "nodes expanded")
    for bar, value in zip(bars, nodes):
        axes[0].text(bar.get_x() + bar.get_width() / 2, value, str(value),
                     ha='center', va='bottom', fontsize=8.5, color=INK)

    # Cost vs moves
    width = 0.36
    b1 = axes[1].bar(x - width / 2, costs, width, label='Path cost', color='#2D6CDF', zorder=3)
    b2 = axes[1].bar(x + width / 2, moves, width, label='Moves', color='#F2A93B', zorder=3)
    _clean(axes[1], "Cost vs moves", "value")
    for bars_ in (b1, b2):
        for bar in bars_:
            axes[1].text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                         f"{bar.get_height():.0f}", ha='center', va='bottom',
                         fontsize=7.5, color=INK)
    axes[1].legend(frameon=False, fontsize=8, loc='upper left')

    # Execution time
    bars = axes[2].bar(x, times, width=0.58, color=colours, zorder=3)
    _clean(axes[2], "Execution time", "milliseconds")
    for bar, value in zip(bars, times):
        axes[2].text(bar.get_x() + bar.get_width() / 2, value, f"{value:.2f}",
                     ha='center', va='bottom', fontsize=8, color=INK)

    fig.suptitle("Performance comparison on the default warehouse grid",
                 fontsize=14, fontweight='bold', color=INK, y=1.03)
    _footer(fig, "Timings are the fastest of repeated runs; paths, costs and node "
                 "counts are identical on every run.")
    fig.subplots_adjust(bottom=0.18, wspace=0.28)
    _save(fig, filepath)
