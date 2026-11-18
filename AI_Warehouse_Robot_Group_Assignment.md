## ARTIFICIAL INTELLIGENCE

Group Coding Assignment

*Title: AI-Based Route Planning for a Warehouse Delivery Robot*

| Activity | Group project | Group size | 5–6 students |
| --- | --- | --- | --- |
| Language | Python | Deadline | 18 November 2026 |

our group 5 is :
	Dev Goyal	BT24CSD029
	Yash Joshi	BT24CSD024
	Yuvraj Singh	BT24CSD031
	Harsh Patil	BT24CSD033
	Piyush Mangalam	BT24CSD032
	Sayan Majhi	BT24CSD028

## 1. Background

A warehouse robot carries packages from a packing station to a dispatch area. Shelves and restricted zones block some routes, and crowded aisles take longer to cross. A trolley may also block a route that the robot initially believes is clear.

## 2. Problem Statement

Develop a simulated delivery robot that searches for a route on a 2D warehouse grid. Compare four search algorithms and demonstrate how the robot reacts when it discovers an obstacle on its planned route.

## 3. Tasks to be Completed

- Task 1 – Warehouse environment: Build and display a grid with open aisles, crowded cells, shelves, restricted zones, a start, and a destination. Allow moves up, down, left, and right. Entering a normal cell costs 1; entering a crowded cell costs 4. Shelves and restricted zones are impassable.

- Task 2 – Problem formulation: Define the state, initial state, actions, transition model, goal test, and path-cost function.

- Task 3 – Search algorithms: Implement Breadth-First Search (BFS), Depth-First Search (DFS), Greedy Best-First Search, and A* on the same grid. Use Manhattan distance for Greedy and A*. Explain why it is a lower bound on remaining travel cost. Test a short, crowded route against a longer, less costly route.

- Task 4 – Obstacle detection and replanning: Place a trolley on the initial A* route, but omit it from the robot’s known map. Simulate detection before the robot enters the blocked cell, update its map, and replan with A* from the current position. Report if no alternative exists.

- Task 5 – Performance evaluation: Compare goal reached, path length (moves), total path cost, nodes expanded, and execution time. For the trolley experiment, also record the number of replans.

## 4. Expected Program Output

- A labeled grid and the route returned by each algorithm, or a clear message if no route is found.

- A demonstration showing the original route, trolley detection, and the revised route or failure to replan.

- A comparison table and a brief explanation of whether the shortest route also had the lowest cost.


## 5. Group Work

| member | work |
| --- | --- |
| 1 | Warehouse grid and search problem formulation |
| 2 | BFS and DFS implementation |
| 3 | Greedy Best-First Search and heuristic explanation |
| 4 | A* implementation and weighted path costs |
| 5 | Trolley detection, map update, and replanning |
| 6 | Testing, visualization, performance comparison, and documentation |

## 6. GitHub Repository and Submission

Create one GitHub repository per group and make it accessible to the instructor. Include:

- Complete Python source code.

- A README.md with the problem statement, algorithms, dependencies, and instructions to run the program.

- Screenshots of program outputs, including the trolley and replanning demonstration.

- A performance comparison table and a brief summary of your findings.

- Names, roll numbers, and contributions of all group members.

- Give some random task to the members according to the matrix above in the main readme file.

## 7. General Instructions

- Use Python. You may use libraries such as NumPy and Matplotlib.

- Acknowledge external references or code used. Each member should be prepared to explain how the program works during evaluation.

- Apply your creativity to make your work more presentable (GUI, animations, visualizations, etc.).
