# rrt_planner

RRT* (**Rapidly-exploring Random Tree\***) globalny planer ścieżki dla **Nav2** w ROS 2

Implementacja **algorytmu** opisanego pseudokodem:

```
1 V ← {xinit}; E ← ∅;
2 for i = 1, . . . , n do
3   xrand ← SampleFreei;
4   xnearest ← Nearest(G = (V, E), xrand);
5   xnew ← Steer(xnearest, xrand) ;
6   if ObtacleFree(xnearest, xnew) then
7      Xnear ← Near(G = (V, E), xnew, min{γRRT∗ (log(card (V ))/ card (V ))1/d, η}) ;
8      V ← V ∪ {xnew};
9      xmin ← xnearest; cmin ← Cost(xnearest) + c(Line(xnearest, xnew));
10     foreach xnear ∈ Xnear do // Connect along a minimum-cost path
11         if CollisionFree(xnear, xnew) ∧ Cost(xnear) + c(Line(xnear, xnew)) < cmin then
12         xmin ← xnear; cmin ← Cost(xnear) + c(Line(xnear, xnew))
13     E ← E ∪ {(xmin, xnew)};
14     foreach xnear ∈ Xnear do // Rewire the tree
15         if CollisionFree(xnew, xnear) ∧ Cost(xnew) + c(Line(xnew, xnear)) < Cost(xnear) then
           xparent ← Parent(xnear);
16         E ← (E \ {(xparent, xnear)}) ∪ {(xnew, xnear)}
17 return G = (V, E);
```

---

## Struktura paczki

```
rrt_planner/
├── include/rrt_planner/
│   └── rrt_planner.hpp      # Plik nagłówkowy klasy
├── src/
│   └── rrt_planner.cpp      # Pełna implementacja RRT*
├── CMakeLists.txt
├── package.xml
├── plugins.xml              
└── nav2_params.yaml         # Plik z parametrami planera
```

---

## Uruchomienie

```bash
# Umieść paczkę w raz z plikiem start.sh w swoim ROS 2 workspace
cd ~/ros2_ws
./start.sh # włączenie z budowaniem paczkie
./start.sh --no-build # włączenie bez budowania paczki

# W przypadku problemów z uruchomieniam paczki (nie włącza się RViZ/Gazebo) można spróbować przed wykonaniem pliki start.sh wykonać linię poleceń:
# pkill -9 -f gazebo; pkill -9 -f gz; pkill -9 -f ros2; pkill -9 -f rviz
```

---

## Konfiguracja parametrów Nav2

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

## Parametry

| Parametr               | Typ    | Domyślnie | Opis        |
|------------------------|--------|-----------|-------------|
| `max_iterations`       | int    | 5000      | Max RRT expansions before failure |
| `step_size`            | double | 0.05 m    | Max edge length (Steer distance) |
| `goal_tolerance`       | double | 0.15 m    | Distance to goal considered "reached" |
| `goal_bias`            | double | 0.10      | Probability of sampling the goal directly |
| `collision_check_step` | double | 0.02 m    | Sampling density along each new edge for collision checking |

---

## Szczegóły algorytmu

| Step | Function | Implementation |
|------|----------|----------------|
| `SampleFree` | `sampleFree()` | Rejection-samples random `(x,y)` from costmap bounds; with probability `goal_bias` returns the goal |
| `Nearest` | `nearest()` | Linear scan of `tree_` – O(n); sufficient for typical maps |
| `Steer` | `steer()` | Moves toward `x_rand` by at most `step_size` |
| `ObstacleFree` | `obstacleFree()` | Dense walk at `collision_check_step` resolution; checks `nav2_costmap_2d::LETHAL_OBSTACLE` |
| Path extraction | `tracePath()` | Follows `parent_idx` pointers from goal node to root, then reverses |

