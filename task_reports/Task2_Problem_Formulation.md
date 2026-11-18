# Task 2 — Search Problem Formulation

**Assigned To:** Dev Goyal (BT24CSD029)  
**Source File:** [`warehouse_grid.py`](../warehouse_grid.py) (documented in class docstring and methods)  
**Objective:** Formally define the warehouse navigation task as a search problem using the standard AI framework: state, initial state, actions, transition model, goal test, and path-cost function.

---

## 2.1 Overview

Before applying any search algorithm, the warehouse delivery task must be cast as a formal **search problem** following the framework established by Russell & Norvig (*Artificial Intelligence: A Modern Approach*, 4th ed., Chapter 3).

A search problem is defined by five components:

1. **State space** — the set of all possible configurations.
2. **Initial state** — where the agent starts.
3. **Actions** — what the agent can do in each state.
4. **Transition model** — how actions change the state.
5. **Goal test** — recognising when the task is complete.
6. **Path-cost function** — quantifying solution quality.

---

## 2.2 Formal Definition

### State Space

$$\mathcal{S} = \{ (r, c) \mid 0 \le r < \text{rows}, \; 0 \le c < \text{cols} \}$$

Each state is an **integer coordinate pair** $(r, c)$ representing the robot's row and column position on the warehouse grid. For the default $10 \times 10$ grid, the state space contains up to 100 possible positions (many of which are occupied by impassable obstacles).

The **reachable** state space is a subset of $\mathcal{S}$ — those cells that are traversable (not shelves, not restricted zones, and not known trolley obstacles).

### Initial State

$$s_0 = \text{coordinates of cell } S$$

On the default grid: $s_0 = (0, 0)$.

This is the robot's starting position at the packing station.

### Actions

$$A(s) = \{ a \in \{ \text{UP}, \text{DOWN}, \text{LEFT}, \text{RIGHT} \} \mid \text{Result}(s, a) \text{ is valid} \}$$

At each state $s = (r, c)$, the robot can attempt up to four orthogonal movements:

| Action | Direction Vector $(\\Delta r, \\Delta c)$ |
|--------|:---:|
| `UP` | $(-1, 0)$ |
| `DOWN` | $(+1, 0)$ |
| `LEFT` | $(0, -1)$ |
| `RIGHT` | $(0, +1)$ |

An action is **valid** (included in $A(s)$) if and only if:
1. The resulting position $(r + \Delta r, \; c + \Delta c)$ is **within grid bounds**.
2. The target cell is **not impassable** (i.e., not `#`, `R`, or detected `T`).

**Implementation in code:**
```python
def actions(self, state: Tuple[int, int]) -> List[Tuple[int, int, str]]:
    r, c = state
    result = []
    for dr, dc, name in ACTIONS:
        nr, nc = r + dr, c + dc
        if self.is_passable(nr, nc):
            result.append((nr, nc, name))
    return result
```

### Transition Model

$$\text{Result}(s, a) = (r + \Delta r, \; c + \Delta c)$$

The transition model is **deterministic**: given state $s = (r, c)$ and action $a = (\Delta r, \Delta c)$, the robot deterministically moves to the adjacent cell. There is no stochasticity, no slip probability, and no partial observability in the base model.

**Implementation in code:**
```python
def result(self, state, action):
    nr, nc, _ = action
    return (nr, nc)
```

### Goal Test

$$\text{IsGoal}(s) = \begin{cases} \text{True} & \text{if } s = (r_G, c_G) \\ \text{False} & \text{otherwise} \end{cases}$$

Where $(r_G, c_G)$ are the coordinates of cell `G`. On the default grid: $(r_G, c_G) = (9, 9)$.

**Implementation in code:**
```python
def goal_test(self, state):
    return state == self.goal
```

### Path-Cost Function

$$g(p) = \sum_{i=1}^{n} c(p_{i-1}, a_i, p_i)$$

Where the **step cost** $c(s, a, s')$ equals the **entry cost** of the destination cell $s'$:

| Destination Cell Type | Step Cost $c(s, a, s')$ |
|:-----:|:---:|
| Open aisle (`.`) | 1 |
| Crowded cell (`C`) | 4 |
| Start (`S`) | 1 |
| Goal (`G`) | 1 |
| Impassable (`#`, `R`, `T`) | $\infty$ (action not valid) |

> [!NOTE]
> The start cell's cost is **excluded** from the path cost. Only cells actually *entered* (moved into) contribute to the total cost. This follows the standard convention where the initial state incurs no entry cost.

**Implementation in code:**
```python
def path_cost(self, path: List[Tuple[int, int]]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(self.entry_cost(r, c) for r, c in path[1:])
```

---

## 2.3 State-Space Graph Properties

| Property | Value |
|----------|-------|
| **State space type** | Finite, discrete |
| **State space size** | $\text{rows} \times \text{cols}$ (100 for default grid) |
| **Reachable states** | Passable cells only (excludes `#`, `R`, undetected `T`) |
| **Branching factor** | At most 4 (fewer at boundaries and near obstacles) |
| **Deterministic?** | Yes — each action produces exactly one successor state |
| **Fully observable?** | Base model: Yes. With trolleys (Task 4): partially observable |
| **Edge weights** | Non-uniform positive weights (1 or 4), dependent on destination cell |
| **Graph structure** | Undirected (all moves are reversible at equal cost) |

---

## 2.4 Significance of the Formulation

This formulation directly influences algorithm selection and behaviour:

1. **BFS** treats all edges as unit-cost, so it finds the path with the fewest moves but ignores the entry-cost differences between `.` (cost 1) and `C` (cost 4). This means BFS is **not cost-optimal** when crowded cells are present.

2. **DFS** does not consider cost or depth optimality at all. It simply explores deeply and may return an arbitrary path.

3. **Greedy Best-First Search** uses $h(n) = \text{Manhattan}(n, \text{goal})$ to prioritise states that appear closest to the goal. It does not track accumulated cost $g(n)$, so it cannot guarantee cost-optimality.

4. **A\*** combines accumulated cost $g(n)$ with the admissible heuristic $h(n)$, producing $f(n) = g(n) + h(n)$. Because the heuristic never overestimates (see Task 3), A* is guaranteed to return the **minimum-cost** path.

> [!TIP]
> The non-uniform edge weights (crowded cells costing 4 instead of 1) are specifically designed so that the optimal-moves solution (BFS) differs from the optimal-cost solution (A*). This creates the opportunity for the **"cost ≠ moves"** demonstration required by the assignment.

---

## 2.5 Console Output

When the program runs, the problem formulation is displayed as:

```
  Problem Formulation (Task 2)
  ─────────────────────────────
  State space      : (row, col) integer pairs
  Initial state    : (0, 0)  [S]
  Goal state       : (9, 9)  [G]
  Actions          : UP / DOWN / LEFT / RIGHT (4-connectivity)
  Transition model : deterministic adjacent-cell move
  Goal test        : state == (9, 9)
  Path-cost fn     : sum of entry costs (open=1, crowded=4)

  Cell type legend:
    '.' Open aisle (cost 1)   '#' Shelf (impassable)
    'C' Crowded   (cost 4)    'R' Restricted (impassable)
    'S' Start     (cost 1)    'T' Trolley (impassable when detected)
    'G' Goal      (cost 1)
```

---

## 2.6 Relationship to Other Tasks

| Task | How it uses the formulation |
|------|-----------------------------|
| **Task 3** | All four search algorithms (BFS, DFS, Greedy, A*) operate on the state space, actions, transitions, goal test, and cost function defined here. |
| **Task 4** | The transition model is extended with partial observability: trolleys are unknown until the robot's sensor detects them one step ahead. The `mark_impassable()` method dynamically modifies the grid, and `clone()` supports creating independent copies for replanning. |
| **Task 5** | The path-cost function is the basis for the "Path Cost" column in the performance comparison table. The distinction between move count and cost is a direct consequence of the non-uniform cost model. |
