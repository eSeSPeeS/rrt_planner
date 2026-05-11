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
src/
└── rrt_planner/
    ├── include/rrt_planner/
    │   └── rrt_planner.hpp      # Plik nagłówkowy klasy
    ├── src/
    │   └── rrt_planner.cpp      # Pełna implementacja RRT*
    ├── CMakeLists.txt
    ├── package.xml
    ├── plugins.xml
    ├── nav2_params_rrt.yaml     # Plik z parametrami Nav2
    └── start.sh                 # Skrypt uruchamiający symulację
```

---

## Uruchomienie

```bash
# Umieść paczkę w swoim ROS 2 workspace
cd ~/ros2_ws/src/rrt_planner
./start.sh            # uruchomienie z budowaniem paczki
./start.sh --no-build # uruchomienie bez budowania paczki (brak zmian)

# W przypadku problemów z uruchomieniem (nie włącza się RViz/Gazebo)
# można spróbować przed wykonaniem start.sh uruchomić:
# pkill -9 -f gazebo; pkill -9 -f gz; pkill -9 -f ros2; pkill -9 -f rviz
```

Skrypt `start.sh` automatycznie wykrywa swoje położenie na dysku jako katalog workspace.

---

## Parametry

### Logika RRT\*

| Parametr | Typ | Domyślnie | Opis |
|---|---|---|---|
| `max_iterations` | int | 5000 | Maksymalna liczba iteracji (rozszerzeń drzewa) w jednym wywołaniu `createPlan()` |
| `step_size` | double | 0.07 m | Maksymalna długość krawędzi w jednej iteracji (długość kroku funkcji `Steer`) |
| `goal_tolerance` | double | 0.15 m | Odległość od celu uznawana za jego osiągnięcie |
| `goal_bias` | double | 0.10 | Prawdopodobieństwo [0–1] wylosowania punktu dokładnie w miejscu celu zamiast losowego punktu mapy. Przyspiesza zbieżność |
| `collision_check_step` | double | 0.02 m | Rozdzielczość próbkowania wzdłuż każdej nowej krawędzi przy sprawdzaniu kolizji. Musi być ≤ `step_size` |
| `gamma_rrt_star` | double | 5.0 | Stała γ we wzorze na promień sąsiedztwa `Near`: `r = γ · √(log(n)/n)`. Większa wartość = szersze sąsiedztwo = lepsza optymalizacja, ale wolniejsze iteracje. Ograniczenie miękkie na promień sąsiedztwa. |
| `max_near_radius` | double | 0.40 m | Twarde górne ograniczenie promienia sąsiedztwa `Near`.e |
| `post_goal_iterations` | int | 2000 | Liczba dodatkowych iteracji RRT\* wykonywanych **po** pierwszym znalezieniu celu. W tym czasie algorytm kontynuuje przepinanie drzewa, szukając lepszej ścieżki. Po ich zakończeniu ścieżka zostaje zamrożona i robot rusza. |

### Wizualizacja RRT\* w RViZ

| Parametr | Typ | Domyślnie | Opis |
|---|---|---|---|
| `node_scale` | double | 0.03 m | Rozmiar sfery reprezentującej węzeł drzewa w RViz |
| `edge_scale` | double | 0.01 m | Szerokość linii krawędzi drzewa i najlepszej ścieżki w RViz |
| `viz_publish_every_n` | int | 0 | Publikuj drzewo do RViz co N dodanych węzłów. Małe wartości (np. 5–50) tworzą animację rozrostu drzewa. `0` = tylko na końcu planowania |
| `viz_step_delay_ms` | int | 0 | Czas oczekiwania między kolejnymi publikacjami animacji [ms]. Zwiększ, jeśli animacja jest za szybka by ją zobaczyć |

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
| 🔴 Czerwony | Aktualnie najlepsza ścieżka od startu do celu (aktualizuje się podczas przepinania drzewa) |

---

## Szczegóły algorytmu

### RRT vs RRT\*

| | RRT | RRT\* |
|---|---|---|
| Łączy z | najbliższym węzłem | sąsiadem o najniższym koszcie w promieniu `Near` |
| Rewiring | Nie | Tak – przepina sąsiadów jeśli ścieżka przez `x_new` jest tańsza |
| Asymptotyczna optymalność | Nie | Tak |
| Śledzenie kosztu | Nie | Tak – każdy węzeł przechowuje koszt od korzenia |
| Propagacja kosztów | Nie | Tak – BFS po potomkach przepiętego węzła |

### Mapowanie kroków algorytmu na kod

| Pseudokod | Funkcja | Implementacja |
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

---

## Porównanie RRT\* z domyślnym planerem Nav2

Pomiary przeprowadzono na mapie TurtleBot3 w RViZ, dla tego samego punktu startowego i celu oddalonego o ~4–5 m. Parametry dla RRT\*: `max_iterations=20000`, `post_goal_iterations=10000`, `step_size=0.07`, `viz_publish_every_n=0`, `viz_step_delay_ms=0`.

### Wyniki pomiarów

#### RRT\* (`max_near_radius=0.40 m`) — 4 przebiegi

| | Pierwsza znaleziona ścieżka | Ostateczna ścieżka (~10000 iter po celu) |
|---|---|---|
| Średnia długość | 4.826 m | 3.861 m |
| Średni czas | 1.82 ms | 105.05 ms |
| Poprawa długości względem pierwszej | — | −20.0% |

#### RRT\* (`max_near_radius=0.20 m`) — 4 przebiegi

| | Pierwsza znaleziona ścieżka | Ostateczna ścieżka (~10000 iter po celu) |
|---|---|---|
| Średnia długość | 4.248 m | 3.906 m |
| Średni czas | 1.42 ms | 181.77 ms |
| Poprawa długości względem pierwszej | — | −8.1% |

#### Domyślny planer Nav2 (NavFn/Dijkstra) — 4 przebiegi

| Średni czas planowania | Średnia długość ścieżki |
|---|---|
| 1529.2 ms | 4.470 m |

### Porównanie czasów i długości

| Planer | Śr. czas (ostateczny) | Śr. długość (ostateczna) |
|---|---|---|
| RRT\* (r=0.40 m) | 105.1 ms | 3.861 m |
| RRT\* (r=0.20 m) | 181.8 ms | 3.906 m | 
| NavFn (domyślny) | 1529.2 ms | 4.470 m |

### Wpływ limitu iteracji na wynik

Poniżej porównanie jakości ścieżki przy ustawieniu ~2500 iteracji po znalezieniu celu (symulacja krótszego planowania):

| Planer | Śr. czas przy ~2500 iter | Śr. długość przy ~2500 iter |
|---|---|---|
| RRT\* (r=0.40 m) | 9.50 ms | 4.727 m |
| RRT\* (r=0.20 m) | 11.51 ms | 4.192 m |
| NavFn (domyślny) | 1529.2 ms | 4.470 m |

### Wnioski

**Czas planowania.** Oba warianty RRT\* są znacznie szybsze od NavFn — wariant z `r=0.40 m` jest około **15× szybszy** (105 ms vs 1529 ms), a wariant z `r=0.20 m` około **8× szybszy** (182 ms vs 1529 ms). Wynika to z losowej natury RRT\* — algorytm nie przeszukuje całej siatki kosztów, lecz eksploruje przestrzeń próbkując losowo, co pozwala na szybkie znalezienie i optymalizację ścieżki bez konieczności odwiedzenia każdej komórki mapy.

**Długość ścieżki.** Oba warianty RRT\* po pełnej fazie rewiringu generują ścieżki krótsze od NavFn — o 13.6% przy `r=0.40 m` i o 12.6% przy `r=0.20 m`. NavFn jako algorytm oparty na siatce jest ograniczony przez rozdzielczość costmapy i kierunki dozwolone przez siatkę, co powoduje powstawanie charakterystycznych "schodkowych" ścieżek dłuższych niż geometrycznie minimalna. RRT\*, operując w przestrzeni ciągłej, może wyznaczać krawędzie w dowolnym kierunku, co skutkuje krótszymi, bardziej naturalnymi ścieżkami.

**Wpływ promienia sąsiedztwa `max_near_radius` na zbieżność.** Przy `r=0.40 m` algorytm osiąga lepszą ścieżkę końcową (3.861 m vs 3.906 m) w krótszym czasie (105 ms vs 182 ms), bo każde przepięcie obejmuje szerszy obszar i może skrócić ścieżkę o większy odcinek. Przy `r=0.20 m` każde pojedyncze rewirowanie daje mniejszą poprawę, więc algorytm musi wykonać więcej operacji by osiągnąć podobny efekt — stąd wolniejsza zbieżność i dłuższy czas planowania. Widać to wyraźnie w danych: przy `r=0.20 m` przepinania trwają do iteracji ~9700, podczas gdy przy `r=0.40 m` zbieżność następuje wcześniej.

**Wpływ limitu iteracji.** Przy ograniczeniu do ~2500 iteracji obraz się odwraca: RRT\* z `r=0.20 m` daje krótszą ścieżkę (4.192 m) niż wariant z `r=0.40 m` (4.727 m), przy zbliżonym czasie (11.5 ms vs 9.5 ms). Wynika to z tego, że mniejszy promień sąsiedztwa powoduje większą gęstość przepinań na jednostkę czasu — w krótkim horyzoncie czasowym częste, drobne poprawy dają lepszy wynik niż rzadkie, ale duże przepięcia charakterystyczne dla `r=0.40 m`. Przy długim planowaniu przewagę odzyskuje większy promień.

**Powtarzalność.** NavFn jest deterministyczny — dla tego samego celu zawsze zwraca identyczną ścieżkę (4.467–4.474 m w pomiarach). RRT\* jest algorytmem losowym, więc długość pierwszej znalezionej ścieżki różni się między przebiegami (4.36–5.72 m przy `r=0.40 m`), natomiast po pełnej fazie rewiringu wyniki zbiegają do zbliżonych wartości w okolicach 3.85–3.99 m.
