# Task Reports — Index

Detailed documentation for each task of **AI-Based Route Planning for a Warehouse Delivery
Robot** (Group 5). The top-level [`README.md`](../README.md) is the summary; these reports hold
the full derivations, code walk-throughs and measured results.

| File | Task | Assigned to | Contents |
|---|---|---|---|
| [Task1_Warehouse_Environment.md](Task1_Warehouse_Environment.md) | **Task 1 — Warehouse environment** | Dev Goyal (BT24CSD029) | Cell vocabulary (7 types), weighted cost model, 4-connectivity, default 10×10 plan with shelves, crowded aisles and restricted zones, `WarehouseGrid` API, input validation, console and figure output |
| [Task2_Problem_Formulation.md](Task2_Problem_Formulation.md) | **Task 2 — Problem formulation** | Dev Goyal (BT24CSD029) | Formal state space, initial state, actions, deterministic transition model, goal test, path-cost function, graph properties, and how the formulation constrains each algorithm |
| [Task3_Search_Algorithms.md](Task3_Search_Algorithms.md) | **Task 3 — Search algorithms** | Yash Joshi (BT24CSD024), Yuvraj Singh (BT24CSD031), Harsh Patil (BT24CSD033) | BFS (FIFO), DFS (LIFO), Greedy (h only), A\* (g + h); every route drawn on the map; Manhattan admissibility and consistency proof; cost ≠ moves corridor; comparison table |
| [Task4_Trolley_Replanning.md](Task4_Trolley_Replanning.md) | **Task 4 — Trolley detection and replanning** | Piyush Mangalam (BT24CSD032) | Three-grid knowledge model, one-step lookahead, dynamic map update, online A\* from the current cell, measured replan, and the blocked-corridor case where no alternative exists |
| [Task5_Performance_Evaluation.md](Task5_Performance_Evaluation.md) | **Task 5 — Performance evaluation and visualisation** | Sayan Majhi (BT24CSD028) | Benchmark methodology (fastest of 25 runs), comparison and trolley tables, findings, the six figures, the Tkinter simulator, the 51-test suite and the deliverables checklist |

## Where the numbers come from

`python main.py` measures every algorithm, writes `screenshots/*.png`, and regenerates
[`results/performance_results.md`](../results/performance_results.md). The reports and README
quote that output, so the documented results always match a fresh run. Moves, path costs and
node counts are deterministic; timings are the fastest of 25 runs and will differ slightly on
other hardware.

## Deliverables at a glance

* Source: `warehouse_grid.py`, `search_algorithms.py`, `trolley_replanning.py`, `metrics.py`,
  `visualization.py`, `gui.py`, `main.py`
* Report: `README.md` plus these five task reports
* Figures: `screenshots/` — floor plan, algorithm routes, cost vs moves, trolley replanning,
  blocked corridor, performance chart, simulator window
* Measured data: `results/performance_results.md`
* Tests: `tests.py` — 51 tests
