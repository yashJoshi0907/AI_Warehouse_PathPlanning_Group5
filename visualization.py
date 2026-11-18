"""
visualization.py
================
Task 5/6 – Grid Rendering, Route Overlay & Performance Charts
Author   : Sayan Majhi (BT24CSD028)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot

Dependencies: matplotlib, numpy
"""

from __future__ import annotations
import os
from typing import List, Optional, Tuple, Dict

import matplotlib
matplotlib.use('Agg')          # headless backend for file output
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from warehouse_grid import WarehouseGrid, OPEN, CROWDED, SHELF, RESTRICTED, START, GOAL, TROLLEY
from search_algorithms import SearchResult
from trolley_replanning import SimulationResult

# ---------------------------------------------------------------------------
# Colour scheme
# ---------------------------------------------------------------------------
CELL_COLOURS: Dict[str, str] = {
    OPEN:       '#FFFFFF',   # white
    CROWDED:    '#FFD580',   # amber
    SHELF:      '#4A4A6A',   # dark slate (impassable)
    RESTRICTED: '#C0392B',   # red (impassable)
    START:      '#27AE60',   # green
    GOAL:       '#2980B9',   # blue
    TROLLEY:    '#E74C3C',   # bright red
}
PATH_COLOUR   = '#8E44AD'   # purple for path overlay
REPLAN_COLOUR = '#E67E22'   # orange for replanned path
TRACE_COLOUR  = '#16A085'   # teal for robot trace


def _cell_color(sym: str) -> str:
    return CELL_COLOURS.get(sym, '#EEEEEE')


def _draw_grid(ax: plt.Axes, grid: WarehouseGrid,
               path: Optional[List[Tuple[int, int]]] = None,
               path_color: str = PATH_COLOUR,
               extra_markers: Optional[Dict[Tuple[int, int], str]] = None) -> None:
    """
    Low-level grid drawing onto *ax*.
    path       : cells to overlay with coloured dots.
    extra_markers : {(r,c): color_hex} for special markers (e.g. trolley detection).
    """
    rows, cols = grid.rows, grid.cols
    path_set = set(path or [])
    extra_markers = extra_markers or {}

    for r in range(rows):
        for c in range(cols):
            sym = grid.cell(r, c)
            color = _cell_color(sym)
            rect = plt.Rectangle([c, rows - 1 - r], 1, 1,
                                  facecolor=color, edgecolor='#AAAAAA', linewidth=0.5)
            ax.add_patch(rect)

            # Cell label
            label = sym if sym not in ('.', ) else ''
            if label:
                ax.text(c + 0.5, rows - 1 - r + 0.5, label,
                        ha='center', va='center', fontsize=7,
                        fontweight='bold',
                        color='white' if sym in (SHELF, RESTRICTED) else '#333333')

    # Path overlay
    if path:
        for i, (r, c) in enumerate(path):
            if (r, c) == path[0]:    # start
                marker_color = CELL_COLOURS[START]
            elif (r, c) == path[-1]: # goal
                marker_color = CELL_COLOURS[GOAL]
            else:
                marker_color = path_color
            ax.plot(c + 0.5, rows - 1 - r + 0.5, 'o',
                    color=marker_color, markersize=5, alpha=0.8)
        # Draw connecting lines
        xs = [c + 0.5 for _, c in path]
        ys = [rows - 1 - r + 0.5 for r, _ in path]
        ax.plot(xs, ys, '-', color=path_color, linewidth=1.5, alpha=0.6)

    # Extra markers (e.g. trolley detection point)
    for (r, c), col in extra_markers.items():
        ax.plot(c + 0.5, rows - 1 - r + 0.5, 'X',
                color=col, markersize=10, markeredgecolor='black', markeredgewidth=0.5)

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.set_xticklabels(range(cols), fontsize=6)
    ax.set_yticklabels(range(rows - 1, -1, -1), fontsize=6)
    ax.tick_params(length=0)
    ax.grid(False)
    ax.set_aspect('equal')


# ---------------------------------------------------------------------------
# Public visualisation functions
# ---------------------------------------------------------------------------

def save_grid_overview(grid: WarehouseGrid,
                       filepath: str = 'screenshots/grid_overview.png') -> None:
    """Save a plain labelled grid with no route overlay."""
    os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    _draw_grid(ax, grid)
    ax.set_title('Warehouse Grid Overview', fontsize=12, fontweight='bold', pad=10)

    legend_patches = [
        mpatches.Patch(facecolor=CELL_COLOURS[k], edgecolor='grey', label=v)
        for k, v in [
            (OPEN, 'Open aisle (cost 1)'),
            (CROWDED, 'Crowded cell (cost 4)'),
            (SHELF, 'Shelf (impassable)'),
            (RESTRICTED, 'Restricted (impassable)'),
            (START, 'Start (S)'),
            (GOAL, 'Goal (G)'),
        ]
    ]
    ax.legend(handles=legend_patches, loc='upper right',
              fontsize=7, framealpha=0.9, ncol=2)

    fig.tight_layout()
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  [saved] {filepath}")


def save_algorithm_routes(grid: WarehouseGrid,
                          results: List[SearchResult],
                          filepath: str = 'screenshots/algorithm_routes.png') -> None:
    """
    Save a 2×2 subplot showing each algorithm's route on the grid.
    """
    os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(14, 14))
    axes = axes.flatten()

    colors = [PATH_COLOUR, '#E74C3C', '#27AE60', '#E67E22']

    for i, (res, ax) in enumerate(zip(results, axes)):
        _draw_grid(ax, grid, path=res.path, path_color=colors[i])
        status = (f"Cost={res.path_cost:.0f}  Moves={res.path_length}  "
                  f"Nodes={res.nodes_expanded}  {res.elapsed_sec*1000:.1f}ms")
        title  = f"{res.algorithm}  |  " + ("✓ " + status if res.goal_reached else "✗ No path")
        ax.set_title(title, fontsize=9, fontweight='bold')

    fig.suptitle('Algorithm Route Comparison', fontsize=14, fontweight='bold', y=1.01)
    fig.tight_layout()
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  [saved] {filepath}")


def save_trolley_demo(base_grid: WarehouseGrid,
                      sim: SimulationResult,
                      filepath: str = 'screenshots/trolley_replanning.png') -> None:
    """
    Save a 3-panel figure showing:
      Left  – initial A* plan (with trolley positions marked)
      Middle – moment of detection + replanned path
      Right  – full robot trace
    """
    os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
    n_panels = 2 + len(sim.replan_events)
    fig, axes = plt.subplots(1, n_panels, figsize=(7 * n_panels, 7))
    if n_panels == 1:
        axes = [axes]

    trolley_markers = {pos: '#E74C3C' for pos in sim.trolley_positions}

    # Panel 0 – initial plan
    ax0 = axes[0]
    _draw_grid(ax0, base_grid,
               path=sim.initial_path,
               path_color=PATH_COLOUR,
               extra_markers=trolley_markers)
    ax0.set_title(f"Initial A* Plan\nCost={sim.initial_result.path_cost:.1f}  "
                  f"Moves={len(sim.initial_path)-1}\n"
                  f"(✕ = hidden trolley)", fontsize=9, fontweight='bold')

    # Replan panels
    real_grid_display = base_grid.clone()
    for pos in sim.trolley_positions:
        real_grid_display.mark_impassable(*pos)

    for i, ev in enumerate(sim.replan_events):
        ax = axes[i + 1]
        detection_marker = {ev.detected_at: '#FF0000'}
        _draw_grid(ax, real_grid_display,
                   path=ev.new_path,
                   path_color=REPLAN_COLOUR,
                   extra_markers=detection_marker)
        if ev.new_path:
            ax.set_title(f"Replan #{ev.replan_no}\nDetected at {ev.detected_at}\n"
                         f"New cost={ev.search_result.path_cost:.1f}  "
                         f"Moves={len(ev.new_path)-1}", fontsize=9, fontweight='bold')
        else:
            ax.set_title(f"Replan #{ev.replan_no}\nDetected at {ev.detected_at}\n"
                         "NO ALTERNATIVE PATH", fontsize=9, fontweight='bold', color='red')

    # Final panel – robot trace
    ax_last = axes[-1]
    _draw_grid(ax_last, real_grid_display,
               path=sim.final_trace,
               path_color=TRACE_COLOUR)
    ax_last.set_title(
        f"Full Robot Trace\nGoal {'reached ✓' if sim.goal_reached else 'NOT reached ✗'}",
        fontsize=9, fontweight='bold',
        color='green' if sim.goal_reached else 'red'
    )

    fig.suptitle('Trolley Detection & Replanning Demo', fontsize=13,
                 fontweight='bold', y=1.02)
    fig.tight_layout()
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  [saved] {filepath}")


def save_performance_chart(results: List[SearchResult],
                           filepath: str = 'screenshots/performance_chart.png') -> None:
    """
    Bar chart comparing nodes-expanded and path cost across algorithms.
    """
    os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)

    algos   = [r.algorithm for r in results]
    nodes   = [r.nodes_expanded for r in results]
    costs   = [r.path_cost if r.goal_reached else 0 for r in results]
    moves   = [r.path_length for r in results]
    times   = [r.elapsed_sec * 1000 for r in results]

    x = np.arange(len(algos))
    bar_w = 0.2

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    # Nodes expanded
    axes[0].bar(x, nodes, width=0.5, color=['#8E44AD', '#E74C3C', '#27AE60', '#E67E22'])
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(algos)
    axes[0].set_title('Nodes Expanded', fontweight='bold')
    axes[0].set_ylabel('Count')
    for xi, v in zip(x, nodes):
        axes[0].text(xi, v + 0.3, str(v), ha='center', va='bottom', fontsize=9)

    # Path cost vs moves
    axes[1].bar(x - bar_w / 2, costs, bar_w * 2, label='Path Cost', color='#2980B9', alpha=0.85)
    axes[1].bar(x + bar_w / 2, moves, bar_w * 2, label='Moves', color='#F39C12', alpha=0.85)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(algos)
    axes[1].set_title('Path Cost vs Move Count', fontweight='bold')
    axes[1].legend(fontsize=8)

    # Execution time
    axes[2].bar(x, times, width=0.5, color='#1ABC9C')
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(algos)
    axes[2].set_title('Execution Time (ms)', fontweight='bold')
    axes[2].set_ylabel('ms')
    for xi, v in zip(x, times):
        axes[2].text(xi, v, f'{v:.2f}', ha='center', va='bottom', fontsize=8)

    fig.suptitle('Algorithm Performance Comparison', fontsize=13, fontweight='bold')
    fig.tight_layout()
    fig.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  [saved] {filepath}")
