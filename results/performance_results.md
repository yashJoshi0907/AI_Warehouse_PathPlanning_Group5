# Measured Results

> Generated automatically by `python main.py` — do not edit by hand.
> Timings are the fastest of 25 runs on the machine that ran the demo;
> paths, costs and node counts are identical on every run.

## Default 10×10 warehouse grid

| Algorithm | Goal reached | Moves | Path cost | Nodes expanded | Time (ms) |
|---|---|:---:|---:|---:|---:|
| **BFS** | Yes | 18 | 21.0 | 82 | 0.11 |
| **DFS** | Yes | 24 | 30.0 | 43 | 0.06 |
| **Greedy** | Yes | 20 | 20.0 | 21 | 0.04 |
| **A*** | Yes | 20 | 20.0 | 49 | 0.15 |

## Trolley replanning

| Metric | Value |
|---|---|
| Trolley cell | `[(3, 0)]` |
| Initial path | 20 moves, cost 20.0 |
| Replans | 1 |
| Goal reached | Yes |
| Replan #1 | detected at `(3, 0)` from `(2, 0)` → 16 moves, cost 19.0, 34 nodes |

## Blocked-corridor case (no alternative route)

| Metric | Value |
|---|---|
| Trolley cell | `[(0, 3)]` |
| Initial path | 19 moves, cost 19.0 |
| Replans | 1 |
| Goal reached | No |
| Result | **no alternative path — destination unreachable** |
