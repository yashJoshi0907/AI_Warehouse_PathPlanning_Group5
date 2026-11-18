"""
trolley_replanning.py
=====================
Task 4 – Trolley Detection, Map Update & A* Replanning
Author   : Piyush Mangalam (BT24CSD033)
Project  : AI-Based Route Planning for a Warehouse Delivery Robot
"""

from __future__ import annotations
from typing import List, Tuple, Optional
import copy

from warehouse_grid import WarehouseGrid, TROLLEY
from search_algorithms import astar, SearchResult


# ---------------------------------------------------------------------------
# Data class for one replan event
# ---------------------------------------------------------------------------

class ReplanEvent:
    """Records what happened at a single replanning step."""

    def __init__(self,
                 replan_no: int,
                 detected_at: Tuple[int, int],
                 current_pos: Tuple[int, int],
                 new_path: Optional[List[Tuple[int, int]]],
                 search_result: SearchResult) -> None:
        self.replan_no    = replan_no
        self.detected_at  = detected_at   # cell that was found blocked
        self.current_pos  = current_pos   # robot's position when detected
        self.new_path     = new_path
        self.search_result = search_result


# ---------------------------------------------------------------------------
# TrolleySimulator
# ---------------------------------------------------------------------------

class TrolleySimulator:
    """
    Simulates step-by-step robot movement with dynamic trolley obstacles.

    Workflow (per Task 4 spec):
    1. Run initial A* on the base grid.
    2. Place one or more trolleys on cells that lie on that path
       (but not on S or G) – representing obstacles unknown at plan time.
    3. Robot advances along the planned route one step at a time.
    4. Before entering the next cell the robot looks one step ahead.
       If the next cell is now TROLLEY → detection event:
         a. Mark cell impassable in the robot's known map.
         b. Replan with A* from current position.
         c. Follow new plan until next trolley or goal.
    5. Report all replan events; flag "no path" if goal unreachable.
    """

    def __init__(self, base_grid: WarehouseGrid,
                 trolley_positions: Optional[List[Tuple[int, int]]] = None) -> None:
        """
        Parameters
        ----------
        base_grid : WarehouseGrid
            The original grid (without trolleys placed).
        trolley_positions : list of (r, c), optional
            Cells where trolleys will be secretly placed.
            If None, one trolley is auto-placed on the 3rd cell of the
            initial A* path (a sensible default for demos).
        """
        self.base_grid = base_grid.clone()

        # Robot's internal (known) map starts as a clean copy
        self.known_grid = base_grid.clone()

        # Build the "real world" grid with hidden trolleys
        self.real_grid = base_grid.clone()

        # Run initial A* to get the first planned route
        init_result = astar(self.known_grid)
        if init_result.path is None:
            raise ValueError("No initial path found — cannot run simulation.")
        self.initial_path: List[Tuple[int, int]] = init_result.path
        self.initial_result: SearchResult = init_result

        # Decide trolley positions
        if trolley_positions is None:
            # Default: place on 3rd step (index 2) if path is long enough
            if len(self.initial_path) > 3:
                trolley_positions = [self.initial_path[3]]
            elif len(self.initial_path) > 2:
                trolley_positions = [self.initial_path[2]]
            else:
                trolley_positions = []

        self.trolley_positions = [
            pos for pos in trolley_positions
            if pos not in (base_grid.start, base_grid.goal)
        ]

        # Place trolleys in the real world only
        for r, c in self.trolley_positions:
            self.real_grid.mark_impassable(r, c)

    # ------------------------------------------------------------------
    # Main simulation
    # ------------------------------------------------------------------

    def run(self) -> 'SimulationResult':
        """
        Execute the step-by-step simulation.

        Returns
        -------
        SimulationResult with full trace.
        """
        current_pos    = self.base_grid.start
        goal           = self.base_grid.goal
        current_plan   = list(self.initial_path)  # working copy
        replan_events: List[ReplanEvent]  = []
        full_trace:    List[Tuple[int, int]] = [current_pos]
        replan_count   = 0
        success        = False

        # Advance along the plan step by step
        plan_idx = 1  # next cell to visit in current_plan

        while current_pos != goal:
            if plan_idx >= len(current_plan):
                # Plan exhausted without reaching goal → shouldn't happen
                # but handle gracefully
                break

            next_cell = current_plan[plan_idx]
            nr, nc = next_cell

            # ---- One-step lookahead detection ----
            if self.real_grid.cell(nr, nc) == TROLLEY and \
               self.known_grid.cell(nr, nc) != TROLLEY:

                # Detection: update known map
                self.known_grid.mark_impassable(nr, nc)
                replan_count += 1

                # Replan from current position
                self.known_grid.start = current_pos
                new_result = astar(self.known_grid, start_override=current_pos)
                self.known_grid.start = self.base_grid.start  # restore

                new_path = new_result.path  # None if no route

                event = ReplanEvent(
                    replan_no=replan_count,
                    detected_at=next_cell,
                    current_pos=current_pos,
                    new_path=new_path,
                    search_result=new_result,
                )
                replan_events.append(event)

                if new_path is None:
                    # Dead end — no alternative route
                    break

                # Switch to new plan (skip the first cell = current_pos)
                current_plan = new_path
                plan_idx = 1
                continue  # re-evaluate next step

            # ---- Safe to move ----
            current_pos = next_cell
            full_trace.append(current_pos)
            plan_idx += 1

            if current_pos == goal:
                success = True
                break

        return SimulationResult(
            initial_path    = self.initial_path,
            initial_result  = self.initial_result,
            final_trace     = full_trace,
            replan_events   = replan_events,
            replan_count    = replan_count,
            goal_reached    = success,
            trolley_positions = self.trolley_positions,
        )


# ---------------------------------------------------------------------------
# SimulationResult
# ---------------------------------------------------------------------------

class SimulationResult:
    """Container for the full output of a TrolleySimulator run."""

    def __init__(self,
                 initial_path: List[Tuple[int, int]],
                 initial_result: SearchResult,
                 final_trace: List[Tuple[int, int]],
                 replan_events: List[ReplanEvent],
                 replan_count: int,
                 goal_reached: bool,
                 trolley_positions: List[Tuple[int, int]]) -> None:
        self.initial_path      = initial_path
        self.initial_result    = initial_result
        self.final_trace       = final_trace
        self.replan_events     = replan_events
        self.replan_count      = replan_count
        self.goal_reached      = goal_reached
        self.trolley_positions = trolley_positions

    def print_summary(self) -> None:
        print("\n" + "═" * 60)
        print("  TROLLEY DETECTION & REPLANNING SIMULATION")
        print("═" * 60)
        print(f"  Trolley(s) placed at : {self.trolley_positions}")
        print(f"  Initial A* path      : {self.initial_path}")
        print(f"  Initial path cost    : {self.initial_result.path_cost:.1f}")
        print(f"  Number of replans    : {self.replan_count}")

        for ev in self.replan_events:
            print(f"\n  ── Replan #{ev.replan_no} ──")
            print(f"     Trolley detected at : {ev.detected_at}")
            print(f"     Robot was at        : {ev.current_pos}")
            if ev.new_path:
                cost = ev.search_result.path_cost
                print(f"     New path            : {ev.new_path}")
                print(f"     New path cost       : {cost:.1f}")
                print(f"     Nodes expanded      : {ev.search_result.nodes_expanded}")
            else:
                print("     *** NO ALTERNATIVE PATH — destination unreachable ***")

        print(f"\n  Final robot trace    : {self.final_trace}")
        status = "✓ GOAL REACHED" if self.goal_reached else "✗ GOAL NOT REACHED"
        print(f"  Outcome             : {status}")
        print("═" * 60 + "\n")
