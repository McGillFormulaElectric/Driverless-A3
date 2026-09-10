#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# A3 — Neil's side launcher
# Starts the TF scenario publisher + grader. New members connect over Tailscale,
# play the input bag or run live, and you grade their TF transforms automatically.
#
# Usage:  bash scripts/launch_neil.sh [--build]
#         bash scripts/launch_neil.sh --record-inputs   # regenerate inputs.bag
# ---------------------------------------------------------------------------
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f $REPO_ROOT/docker/docker-compose.yml"

if [[ "${1:-}" == "--build" ]]; then
  echo "[neil] Building image..."
  $COMPOSE build neil
fi

if [[ "${1:-}" == "--record-inputs" ]]; then
  echo "[neil] Recording deterministic input bag → ros2_ws/src/a3_neil/data/inputs.bag"
  $COMPOSE run --rm neil bash -c "
    source /opt/ros/humble/setup.bash
    cd /workspace
    colcon build --packages-select a3_neil --symlink-install --quiet
    source install/setup.bash
    ros2 run a3_neil record_inputs
  "
  exit 0
fi

echo "[neil] Starting A3 TF scenario + grader..."
echo "[neil] Members subscribe to /neil/sensor_cones and /tf"
echo "[neil] Feedback → /neil/feedback"
echo ""

$COMPOSE run --rm neil bash -c "
  source /opt/ros/humble/setup.bash
  cd /workspace
  if [ -f install/setup.bash ]; then source install/setup.bash; fi
  colcon build --packages-select a3_neil --symlink-install --quiet
  source install/setup.bash
  echo '[neil] Launching tf_scenario_node + grader...'
  ros2 launch a3_neil neil.launch.py
"
