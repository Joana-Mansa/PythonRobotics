# Dijkstra's Algorithm (grid based)

Dijkstra's algorithm finds the shortest path from a start cell to a goal cell on a grid, and it is guaranteed to find the shortest one for the movement costs you define. It is a natural first planner to study because it takes no shortcuts: it expands outward from the start, always exploring the cheapest-so-far cell next, until it reaches the goal.

This page documents the implementation in
`PathPlanning/Dijkstra/dijkstra.py`.

## When you would use it

Reach for Dijkstra when you need a provably shortest path and have no
reliable way to estimate the distance still to go. When you do have such an estimate (a heuristic), A* is usually better, because it searches towards the goal instead of in every direction. Dijkstra is in fact A* with the heuristic set to zero, which makes this page the foundation for the A* page that follows.

## How it works

The world is split into a grid whose cell spacing is set by `resolution`.
Every obstacle is "inflated" by the robot radius, so any cell within
`robot_radius` of an obstacle is marked blocked. That inflation is what keeps the planned path clear of the walls rather than clipping them.

The search keeps two collections: an open set of cells discovered but not
yet settled (the frontier), and a closed set of cells already settled with
their final lowest cost. The loop repeats one rule. Take the open cell
with the lowest accumulated cost, settle it, and examine its neighbours.
For each neighbour, the tentative cost is:

$$
g(n) = g(c) + w(c, n)
$$

where $g(c)$ is the cost already accumulated at the current cell and
$w(c, n)$ is the cost of the single step to the neighbour. Step costs come
from the motion model: orthogonal moves cost $1$ and diagonal moves cost
$\sqrt{2}$. If the neighbour is new, or if this route reaches it more
cheaply than any found before, its cost and parent are recorded. When the
settled cell is the goal, the search stops and the path is rebuilt by
following each cell's parent back to the start.

## Key components in the code

- `DijkstraPlanner(ox, oy, resolution, robot_radius)`: builds the inflated
  obstacle map and loads the motion model.
- `planning(sx, sy, gx, gy)`: runs the search and returns two lists, `rx`
  and `ry`, the coordinates of the final path.
- `Node`: one grid cell, holding its indices, accumulated `cost`, and the
  `parent_index` used to trace the path back.
- `get_motion_model()`: the eight permitted moves and their costs (four
  straight at cost 1, four diagonal at cost root two).
- `calc_final_path()`: walks the parent links from goal back to start.

## Running it

```bash
cd PathPlanning/Dijkstra
python dijkstra.py
```

The default scenario in `main()` puts the start at $(-5, -5)$ and the goal
at $(50, 50)$ on a bordered map with two internal walls. The grid
resolution is $2.0$ m and the robot radius is $1.0$ m.

## Reading the animation

- Black dots are obstacles, the green circle is the start, and the blue
  cross is the goal.
- Cyan crosses are the cells being expanded. They spread outward from the
  start in roughly all directions at once. That even, undirected flood is
  the visual signature of Dijkstra, and the thing A* improves on.
- The red line drawn at the end is the shortest path.
- Press Escape to stop the simulation early.

## Experiment

Change the values in `main()` and rerun to build intuition:

- Move the start and goal to plan between different points.
- Increase `grid_size` for a faster but coarser search, or decrease it for
  finer paths at more cost.
- Increase `robot_radius` and watch the path keep further from the walls
  as the inflated obstacles grow.
