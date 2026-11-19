# Task 2 — Search Problem Formulation

**Assigned To:** Dev Goyal (BT24CSD029)
**Source File:** [`warehouse_grid.py`](../warehouse_grid.py)
**Objective:** Define the warehouse navigation task as a formal search problem: state space,
initial state, actions, transition model, goal test and path-cost function.

---

## 2.1 Why formulate first

Search algorithms do not know about warehouses; they know about states, actions and costs.
Casting the floor plan into the Russell & Norvig framework (AIMA, ch. 3) fixes exactly what
BFS, DFS, Greedy and A\* are allowed to do, and makes "optimal" a precise claim: optimal in
**moves** for BFS, optimal in **cost** for A\*.

---

## 2.2 Formal definition

### State space

$$\mathcal{S} = \{(r,c) \mid 0 \le r < \text{rows},\ 0 \le c < \text{cols}\}$$

A state is the robot's cell. For the default 10×10 map there are 100 candidate states, of
which 82 are traversable (100 minus 14 shelves and 4 restricted zones, with `S` and `G`
included). Impassable cells are never generated as successors, so they never enter the
frontier.

### Initial state

$$s_0 = (0, 0) \quad \text{(cell } S\text{, the packing station)}$$

### Actions

$$A(s) = \{\,a \in \{\text{UP},\text{DOWN},\text{LEFT},\text{RIGHT}\} \mid \text{Result}(s,a)\ \text{is legal}\,\}$$

An action is legal when the destination is in bounds and passable:

```python
def actions(self, state):
    r, c = state
    return [(r + dr, c + dc, name)
            for dr, dc, name in ACTIONS
            if self.is_passable(r + dr, c + dc)]
```

### Transition model

$$\text{Result}(s, a) = (r + \Delta r,\ c + \Delta c)$$

Deterministic: the robot never slips, and the model is fully observable. Task 4 relaxes the
observability assumption — the robot only sees one cell ahead — but the underlying transition
model stays deterministic.

### Goal test

$$\text{IsGoal}(s) \iff s = (9, 9)$$

### Path-cost function

$$g(p) = \sum_{i=1}^{n} c(p_{i-1}, a_i, p_i), \qquad
c(s,a,s') = \begin{cases}
1 & s' \text{ is open} \\
4 & s' \text{ is crowded} \\
\infty & s' \text{ impassable (action not legal)}
\end{cases}$$

| Destination | Step cost |
|:---:|:---:|
| Open aisle `.` | 1 |
| Crowded aisle `C` | 4 |
| Start `S` / Goal `G` | 1 |
| Shelf `#` / Restricted `R` / Trolley `T` | ∞ — action filtered out |

The start cell is excluded. This is the convention that makes `g` additive along a path:
$g(\text{prefix} + [x]) = g(\text{prefix}) + \text{EntryCost}(x)$, which A\* relies on.

---

## 2.3 Graph properties

| Property | Value |
|---|---|
| Type | Finite, discrete, deterministic |
| Size | rows × cols (100 default) |
| Branching factor | ≤ 4, fewer at edges and next to obstacles |
| Edge weights | Positive, non-uniform (1 or 4) |
| Directed? | No — every move is reversible at the same cost |
| Observability | Full in Tasks 1–3; partial in Task 4 |
| Negative cycles | Impossible (all costs ≥ 1) |

Because all edge weights are positive, Dijkstra and A\* apply directly. Because weights are
non-uniform, BFS's unit-cost assumption is wrong on purpose: BFS minimises hops, not cost.

Since every move is reversible at equal cost, the state graph is undirected, so any connected
component is strongly connected — a blocked corridor (Task 4) disconnects the goal outright
rather than making it merely expensive.

---

## 2.4 How the formulation drives the algorithms

| Algorithm | Uses of the formulation |
|---|---|
| **BFS** | Treats every edge as cost 1 because it only counts depth; returns the fewest-move route, which may cross `C` cells |
| **DFS** | Ignores cost and depth optimality entirely; returns whatever depth-first dive reaches `G` |
| **Greedy** | Uses $h(n) = \text{Manhattan}(n, G)$, ignoring $g$; fast but unsound for cost |
| **A\*** | Combines $g$ (the path-cost function above) with an admissible $h$; returns the lowest-cost route |

The non-uniform costs are not decoration: they are what makes the assignment's required
"short, crowded route vs. longer, cheaper route" comparison possible.

---

## 2.5 Console output

```text
  Problem formulation (Task 2)
  ────────────────────────────────────────
  State space      : (row, col) integer pairs inside the grid
  Initial state    : (0, 0)  [S]
  Goal state       : (9, 9)  [G]
  Actions          : UP / DOWN / LEFT / RIGHT (4-connectivity)
  Transition model : deterministic adjacent-cell move
  Goal test        : state == (9, 9)
  Path-cost fn     : sum of entry costs (open = 1, crowded = 4,
                     shelves / restricted zones = impassable)
```

---

## 2.6 Relationship to the other tasks

| Task | Uses |
|---|---|
| **Task 3** | Every algorithm operates on exactly these states, actions, transitions, goal test and costs |
| **Task 4** | The map becomes partially observable: `mark_impassable()` changes the robot's model of `is_passable`, and `clone()` gives replanning a private copy |
| **Task 5** | `path_cost()` is the "Path cost" column; `nodes_expanded` counts states dequeued from the frontier |

---

## 2.7 Tests

| Test | Verifies |
|---|---|
| `test_actions_centre` / `test_actions_corner` | Action set is bounds- and obstacle-filtered |
| `test_goal_test` | Goal predicate |
| `test_path_cost_simple` / `test_path_cost_empty` | Cost function and start exclusion |
| `test_clone_independence` | Replanning copies do not corrupt the reference map |
| `test_mark_impassable` | Dynamic modification of the transition model |
| `test_restricted_zone_is_impassable` | `∞`-cost cells never become successors |
