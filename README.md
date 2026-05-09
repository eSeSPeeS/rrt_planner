# rrt_planner

RRT\* (**Rapidly-exploring Random Tree\***) globalny planer ścieżki dla **Nav2** w ROS 2

Implementacja **algorytmu** opisanego pseudokodem:

```
1  V ← {xinit}; E ← ∅;
2  for i = 1, . . . , n do
3    xrand    ← SampleFreei;
4    xnearest ← Nearest(G = (V, E), xrand);
5    xnew     ← Steer(xnearest, xrand);
6    if ObstacleFree(xnearest, xnew) then
7      Xnear ← Near(G = (V, E), xnew, min{γRRT∗ (log(card(V)) / card(V))^(1/d), η});
8      V ← V ∪ {xnew};
9      xmin ← xnearest; cmin ← Cost(xnearest) + c(Line(xnearest, xnew));
10     foreach xnear ∈ Xnear do  // Connect along a minimum-cost path
11       if CollisionFree(xnear, xnew) ∧ Cost(xnear) + c(Line(xnear, xnew)) < cmin then
12         xmin ← xnear; cmin ← Cost(xnear) + c(Line(xnear, xnew))
13     E ← E ∪ {(xmin, xnew)};
14     foreach xnear ∈ Xnear do  // Rewire the tree
15       if CollisionFree(xnew, xnear) ∧ Cost(xnew) + c(Line(xnew, xnear)) < Cost(xnear) then
16         xparent ← Parent(xnear);
17         E ← (E \ {(xparent, xnear)}) ∪ {(xnew, xnear)}
18 return G = (V, E);
```

---

## Struktura paczki

```
ros2_ws
├── start.sh                         # Skrypt uruchamiający symulację
└── src
    └── rrt_planner/
        ├── include/rrt_planner/
        │   └── rrt_planner.hpp      # Plik nagłówkowy klasy
        ├── src/
        │   └── rrt_planner.cpp      # Pełna implementacja RRT*
        ├── CMakeLists.txt
        ├── package.xml
        ├── plugins.xml
        └── nav2_params_rrt.yaml     # Plik z parametrami Nav2
```

---

## Uruchomienie

```bash
# Umieść paczkę wraz z plikiem start.sh w swoim ROS 2 workspace
cd ~/ros2_ws
./start.sh            # uruchomienie z budowaniem paczki
./start.sh --no-build # uruchomienie bez budowania paczki (brak zmian)

# W przypadku problemów z uruchomieniem (nie włącza się RViz/Gazebo)
# można spróbować przed wykonaniem start.sh uruchomić:
# pkill -9 -f gazebo; pkill -9 -f gz; pkill -9 -f ros2; pkill -9 -f rviz
```

Skrypt `start.sh` automatycznie wykrywa swoje położenie na dysku jako katalog workspace.

---

## Wizualizacja w RViz2

Po uruchomieniu można dodać topic z drzewem w RViZ:

1. Kliknij **Add → By topic**
2. Wybierz `/GridBased/rrt_tree` → **MarkerArray**
3. Kliknij **OK**

Drzewo pojawia się po zadaniu pierwszego celu.

| Kolor | Co pokazuje |
|-------|-------------|
| 🔵 Niebieski | Pełne drzewo RRT\* (wszystkie węzły i krawędzie) |
| 🔴 Czerwony | Aktualnie najlepsza ścieżka od startu do celu (aktualizuje się podczas rewiringu) |

---

## Konfiguracja parametrów Nav2

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "rrt_planner/RRTPlanner"
      max_iterations:        5000
      step_size:             0.07
      goal_tolerance:        0.15
      goal_bias:             0.10
      collision_check_step:  0.02
      gamma_rrt_star:        5.0
      max_near_radius:       0.40
      post_goal_iterations:  2000
      node_scale:            0.03
      edge_scale:            0.01
      viz_publish_every_n:   500000
      viz_step_delay_ms:     0
```

---

## Parametry

### Rdzeń RRT\*

| Parametr | Typ | Domyślnie | Opis |
|---|---|---|---|
| `max_iterations` | int | 5000 | Maksymalna liczba iteracji (rozszerzeń drzewa) w jednym wywołaniu `createPlan()`. Jeśli cel nie zostanie znaleziony, planowanie jest kontynuowane w kolejnym wywołaniu z tym samym drzewem |
| `step_size` | double | 0.07 m | Maksymalna długość krawędzi w jednej iteracji (długość kroku funkcji `Steer`) |
| `goal_tolerance` | double | 0.15 m | Odległość od celu uznawana za jego osiągnięcie |
| `goal_bias` | double | 0.10 | Prawdopodobieństwo [0–1] wylosowania punktu dokładnie w miejscu celu zamiast losowego punktu mapy. Przyspiesza zbieżność |
| `collision_check_step` | double | 0.02 m | Rozdzielczość próbkowania wzdłuż każdej nowej krawędzi przy sprawdzaniu kolizji. Musi być ≤ `step_size` |
| `gamma_rrt_star` | double | 5.0 | Stała γ we wzorze na promień sąsiedztwa `Near`: `r = γ · √(log(n)/n)`. Większa wartość = szersze sąsiedztwo = lepsza optymalizacja, ale wolniejsze iteracje. Zalecany zakres: 3.0 – 10.0 |
| `max_near_radius` | double | 0.40 m | Górne ograniczenie promienia sąsiedztwa `Near`. Musi być **wyraźnie większy niż `step_size`** – ustawienie równe `step_size` praktycznie blokuje rewiring, bo sąsiedztwo staje się za małe |
| `post_goal_iterations` | int | 2000 | Liczba dodatkowych iteracji RRT\* wykonywanych **po** pierwszym znalezieniu celu. W tym czasie algorytm kontynuuje rewiring, szukając lepszej ścieżki (widoczne jako zmiana czerwonej linii). Po ich zakończeniu plan zostaje zamrożony i robot rusza |

### Wizualizacja

| Parametr | Typ | Domyślnie | Opis |
|---|---|---|---|
| `node_scale` | double | 0.03 m | Rozmiar sfery reprezentującej węzeł drzewa w RViz |
| `edge_scale` | double | 0.01 m | Szerokość linii krawędzi drzewa i najlepszej ścieżki w RViz |
| `viz_publish_every_n` | int | 500000 | Publikuj drzewo do RViz co N dodanych węzłów. Małe wartości (np. 5–50) tworzą animację rozrostu drzewa. `0` = tylko na końcu planowania |
| `viz_step_delay_ms` | int | 0 | Czas oczekiwania między kolejnymi publikacjami animacji [ms]. Zwiększ, jeśli animacja jest za szybka by ją zobaczyć |

---

## Szczegóły algorytmu

### RRT\* vs zwykły RRT

| | RRT | RRT\* |
|---|---|---|
| Łączy z | najbliższym węzłem | sąsiadem o najniższym koszcie w promieniu `Near` |
| Rewiring | ✗ | ✓ – przepina sąsiadów jeśli ścieżka przez `x_new` jest tańsza |
| Asymptotyczna optymalność | ✗ | ✓ |
| Śledzenie kosztu | ✗ | ✓ – każdy węzeł przechowuje koszt od korzenia |
| Propagacja kosztów | ✗ | ✓ – BFS po potomkach przepiętego węzła |

### Mapowanie kroków algorytmu na kod

| Krok algorytmu | Funkcja | Szczegóły implementacji |
|---|---|---|
| `SampleFree` | `sampleFree()` | Losuje `(x,y)` z obszaru costmapy metodą odrzucania (pomija komórki z kosztem ≥ `LETHAL_OBSTACLE`). Z prawdopodobieństwem `goal_bias` zwraca bezpośrednio punkt celu |
| `Nearest` | `nearest()` | Liniowe przeszukiwanie O(n) po kwadracie odległości euklidesowej |
| `Steer` | `steer()` | Przesuwa się o co najwyżej `step_size` metrów w kierunku wylosowanego punktu |
| `ObstacleFree` | `obstacleFree()` | Gęsty spacer wzdłuż krawędzi z krokiem `collision_check_step`. Blokuje komórki z kosztem ≥ 180, co odpowiada środkowej części strefy inflacyjnej – ścieżki są wyznaczane z marginesem od przeszkód |
| `Near` | `near()` | Zwraca wszystkie węzły w promieniu `r = min(γ · √(log(n)/n), max_near_radius)` |
| Wybór rodzica | `createPlan()` linie 9–12 | Spośród węzłów `X_near` wybiera ten minimalizujący `cost(x_near) + dist(x_near, x_new)` |
| Rewiring | `createPlan()` linie 14–16 | Dla każdego sąsiada sprawdza czy połączenie przez `x_new` daje niższy koszt. Jeśli tak, przepina krawędź i wywołuje `propagateCosts()` |
| Propagacja kosztów | `propagateCosts()` | Iteracyjny BFS w dół drzewa od przepiętego węzła. Aktualizuje koszty wszystkich potomków, w tym węzła celu – bez tego rewiring głębszych węzłów nie wpływałby na koszt ścieżki do celu |
| Ekstrakcja ścieżki | `tracePath()` | Podąża po wskaźnikach `parent_idx` od węzła celu do korzenia, odwraca kolejność i ustawia orientację każdej pozy w kierunku następnego punktu |

### Cykl życia planowania

```
Kliknięcie celu → wywołanie createPlan() nr 1
  Faza 1: drzewo RRT* rośnie aż do znalezienia celu
  Faza 2: post_goal_iterations dodatkowych iteracji (sam rewiring)
          → czerwona ścieżka aktualizuje się przy każdej poprawie
  → planning_done = true → zwróć najlepszą ścieżkę → robot rusza

Wywołania 2, 3, … (robot jedzie, ten sam cel)
  → planning_done == true → natychmiastowy zwrot ścieżki z cache
  → drzewo NIE rośnie

Nowy cel → pełny reset → Faza 1 od nowa
```
