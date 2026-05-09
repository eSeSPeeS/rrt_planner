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
ros2_ws/
├── start.sh                         # Skrypt uruchamiający symulację
└── src/
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

Pomiary przeprowadzono na mapie TurtleBot3 w Gazebo, dla tego samego punktu startowego i celu oddalonego o ~4–5 m. Parametry RRT\*: `max_iterations=5000`, `post_goal_iterations=2000`, `step_size=0.07`, `viz_publish_every_n=500000`, `viz_step_delay_ms=0`.

### Wyniki pomiarów

#### RRT\* (`max_near_radius=0.40 m`) — 5 przebiegów

| | Pierwsza znaleziona ścieżka | Ostateczna ścieżka (po rewiringu) |
|---|---|---|
| Średnia długość | 5.054 m | 4.463 m |
| Średni czas | 1.75 ms | 12.89 ms |

#### RRT\* (`max_near_radius=0.20 m`) — 2 przebiegi

| | Pierwsza znaleziona ścieżka | Ostateczna ścieżka (po rewiringu) |
|---|---|---|
| Średnia długość | 4.011 m | 3.845 m |
| Średni czas | 0.57 ms | 554.61 ms |

#### Domyślny planer Nav2 (NavFn/Dijkstra) — 3 przebiegi

| Średni czas planowania | Średnia długość ścieżki |
|---|---|
| 1362.3 ms | 4.469 m |

### Porównanie zbiorcze

| Planer | Śr. czas (ostateczny) | Śr. długość (ostateczna) | Uwagi |
|---|---|---|---|
| RRT\* (r=0.40 m) | ~13 ms | 4.463 m | Szybki, jakość zbliżona do NavFn |
| RRT\* (r=0.20 m) | ~555 ms | 3.845 m | Dłuższy czas, krótsza ścieżka o ~14% vs NavFn |
| NavFn (domyślny) | ~1362 ms | 4.469 m | Deterministyczny, wolny, powtarzalny |

### Wnioski

**Czas planowania.** RRT\* w wariancie `max_near_radius=0.40 m` jest około **100× szybszy** od domyślnego planera Nav2 (13 ms vs 1362 ms), osiągając przy tym porównywalną jakość ścieżki (4.463 m vs 4.469 m, różnica poniżej 0.1%). Wynika to z losowej natury RRT\* — algorytm nie przeszukuje całej siatki kosztów, lecz eksploruje przestrzeń próbkując losowo, co pozwala na bardzo szybkie znalezienie pierwszej akceptowalnej ścieżki.

**Jakość ścieżki.** Wariant z `max_near_radius=0.20 m` uzyskał krótszą ścieżkę końcową (3.845 m) niż NavFn (4.469 m) — o około 14%. Mniejszy promień sąsiedztwa oznacza, że każdy węzeł przepinany jest w węższym otoczeniu, więc pojedyncze rewirowanie daje mniejszą poprawę. Algorytm musi wykonać znacznie więcej operacji przepinania, by osiągnąć podobny efekt optymalizacji — stąd dłuższy czas, ale ostatecznie krótsza ścieżka.

**Wpływ promienia sąsiedztwa `max_near_radius` na liczbę przepinań.** Przy mniejszym promieniu (`r=0.20 m`) w tej samej liczbie iteracji zachodzi znacznie więcej zmian ścieżki (operacji rewiringu) niż przy `r=0.40 m`. Wynika to z tego, że małe sąsiedztwo oznacza gęstą siatkę węzłów w promieniu `Near` — każdy nowy węzeł trafia w obszar, gdzie jest wielu bliskich sąsiadów z podobnymi kosztami, co sprzyja częstemu przepinaniu. Przy dużym promieniu nowy węzeł ma mniej sąsiadów w zasięgu, ale za to każde przepięcie może skrócić ścieżkę o dłuższy odcinek. Efektem jest wyraźna różnica w kształcie krzywej zbieżności: przy `r=0.20 m` ścieżka jest poprawiana regularnie przez cały czas planowania (widoczne w danych — przepinania trwają do iteracji ~18000), podczas gdy przy `r=0.40 m` poprawa następuje szybciej i wygasa wcześniej.

**Powtarzalność.** NavFn jest deterministyczny — dla tego samego celu zawsze zwraca identyczną ścieżkę (4.467–4.474 m w pomiarach). RRT\* jest losowy, więc długość pierwszej znalezionej ścieżki różni się między przebiegami (3.93–6.21 m przy `r=0.40 m`), natomiast po fazie rewiringu wyniki zbiegają do zbliżonych wartości.
