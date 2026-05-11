#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# start.sh  –  uruchamia symulację TurtleBot3 z plannerem RRT*
#
# Użycie:
#   ./start.sh              – build + uruchom
#   ./start.sh --no-build   – pomiń build (kod się nie zmienił)
#
# Skrypt leży w ros2_ws/src/rrt_planner/ i automatycznie wykrywa
# katalog workspace jako dwa poziomy wyżej.
# ─────────────────────────────────────────────────────────────────────────────

set -e

# ── Wykryj workspace – skrypt jest w ros2_ws/src/rrt_planner/ ────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/../.." && pwd)"
PARAMS=$SCRIPT_DIR/nav2_params_rrt.yaml
BUILD=true

if [[ "$1" == "--no-build" ]]; then
  BUILD=false
fi

echo "╔══════════════════════════════════════════╗"
echo "║     TurtleBot3 + RRT* Planner Start      ║"
echo "╚══════════════════════════════════════════╝"
echo "  Workspace: $WS"
echo "  Params:    $PARAMS"
echo ""

# ── Sprawdź czy params istnieje ───────────────────────────────────────────────
if [ ! -f "$PARAMS" ]; then
  echo "✗ BŁĄD: nie znaleziono pliku params:"
  echo "  $PARAMS"
  exit 1
fi

# ── 1. Środowisko ROS 2 ───────────────────────────────────────────────────────
echo "[1/4] Ładowanie środowiska ROS 2 Humble..."
source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

cd "$WS"
source install/setup.bash 2>/dev/null || true

# ── 2. Build ──────────────────────────────────────────────────────────────────
if [ "$BUILD" = true ]; then
  echo "[2/4] Budowanie paczki rrt_planner..."
  colcon build --packages-select rrt_planner --symlink-install
  source install/setup.bash
  echo "      ✓ Build zakończony"
else
  echo "[2/4] Pomijam build (--no-build)"
fi

# ── 3. Sprawdź czy plugin jest widoczny ───────────────────────────────────────
echo "[3/4] Sprawdzanie pluginu..."
if ros2 pkg list 2>/dev/null | grep -q rrt_planner; then
  echo "      ✓ rrt_planner widoczny"
else
  echo "      ✗ BŁĄD: rrt_planner nie znaleziony – sprawdź błędy buildu"
  exit 1
fi

# ── 4. Uruchom symulację ──────────────────────────────────────────────────────
echo "[4/4] Uruchamiam symulację..."
echo ""

ros2 launch nav2_bringup tb3_simulation_launch.py \
  headless:=False \
  params_file:="$PARAMS"
