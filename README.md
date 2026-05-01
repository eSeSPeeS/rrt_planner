# rrt_planner

RRT (**Rapidly-exploring Random Tree**) global planner plugin for **Nav2** (ROS 2).

Implements **Algorithm 3** verbatim:

```
V ← {x_init};  E ← ∅
for i = 1, …, n do
    x_rand    ← SampleFree_i
    x_nearest ← Nearest(G=(V,E), x_rand)
    x_new     ← Steer(x_nearest, x_rand)
    if ObstacleFree(x_nearest, x_new) then
        V ← V ∪ {x_new};  E ← E ∪ {(x_nearest, x_new)}
return G = (V, E)
```

---

## Package layout

```
rrt_planner/
├── include/rrt_planner/
│   └── rrt_planner.hpp      # Plugin class declaration
├── src/
│   └── rrt_planner.cpp      # Full RRT implementation
├── CMakeLists.txt
├── package.xml
├── plugins.xml               # pluginlib descriptor
└── nav2_params.yaml          # Example Nav2 parameter snippet
```

---

## Build

```bash
# Place the package in your ROS 2 workspace
cp -r rrt_planner ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select rrt_planner --symlink-install
source install/setup.bash
```

---

## Configure Nav2

Add the snippet from `nav2_params.yaml` to your Nav2 configuration:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "rrt_planner/RRTPlanner"
      max_iterations:       5000
      step_size:            0.05   # metres
      goal_tolerance:       0.15   # metres
      goal_bias:            0.10   # 10 % goal-biased sampling
      collision_check_step: 0.02   # metres
```

---

## Parameters

| Parameter              | Type   | Default | Description |
|------------------------|--------|---------|-------------|
| `max_iterations`       | int    | 5000    | Max RRT expansions before failure |
| `step_size`            | double | 0.05 m  | Max edge length (Steer distance) |
| `goal_tolerance`       | double | 0.15 m  | Distance to goal considered "reached" |
| `goal_bias`            | double | 0.10    | Probability of sampling the goal directly |
| `collision_check_step` | double | 0.02 m  | Sampling density along each new edge for collision checking |

---

## Algorithm details

| Step | Function | Implementation |
|------|----------|----------------|
| `SampleFree` | `sampleFree()` | Rejection-samples random `(x,y)` from costmap bounds; with probability `goal_bias` returns the goal |
| `Nearest` | `nearest()` | Linear scan of `tree_` – O(n); sufficient for typical maps |
| `Steer` | `steer()` | Moves toward `x_rand` by at most `step_size` |
| `ObstacleFree` | `obstacleFree()` | Dense walk at `collision_check_step` resolution; checks `nav2_costmap_2d::LETHAL_OBSTACLE` |
| Path extraction | `tracePath()` | Follows `parent_idx` pointers from goal node to root, then reverses |

---

## Limitations & extensions

- **No path smoothing** – the raw RRT tree edges are returned. Add a post-processing step (e.g. shortcutting) for smoother robot motion.
- **O(n) Nearest** – acceptable for thousands of nodes; replace with a kd-tree for very large maps.
- **No rewiring** – this is plain RRT, not RRT*. If asymptotic optimality is needed, upgrade to RRT*.
