#!/usr/bin/env bash
##############################################################################
# install_and_run.sh
#
# Clone/use this repo, install missing deps (ROS 2 Humble, Gazebo Harmonic,
# PX4 SITL, MicroXRCEAgent, px4_msgs), then run the 6-drone farm mission.
#
# Usage:
#   git clone https://github.com/AbdurRouf230/Only_simulation_code_repo.git
#   cd Only_simulation_code_repo
#   bash install_and_run.sh
#
# Env:
#   SKIP_INSTALL=1     already have ROS2/PX4/gz/XRCE — only run
#   SKIP_RUN=1         only install, do not start the sim
#   NUM_DRONES=6
#   DURATION=200
#   PX4_DIR=~/PX4-Main/PX4-Autopilot
#   ROS2_WS=~/ros2_ws
##############################################################################
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Run this inside WSL Ubuntu 22.04 (or Ubuntu 22.04), not Windows."
  exit 1
fi

REPO_URL="${REPO_URL:-https://github.com/AbdurRouf230/Only_simulation_code_repo.git}"
REPO_DIR="${REPO_DIR:-$HOME/Only_simulation_code_repo}"
NUM_DRONES="${NUM_DRONES:-6}"
DURATION="${DURATION:-200}"

echo "============================================================="
echo " Multi-drone pollination sim (PX4 + Gazebo Harmonic)"
echo "============================================================="

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}" 2>/dev/null || realpath "${BASH_SOURCE[0]}" 2>/dev/null || echo "")"
if [[ -n "$SCRIPT_PATH" && -f "$(dirname "$SCRIPT_PATH")/intelleswarm-ai/genai_framework/px4_gazebo_sim/run_pollination_simulation.py" ]]; then
  REPO_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
  echo "Using local repo: $REPO_DIR"
elif [[ -f "$REPO_DIR/intelleswarm-ai/genai_framework/px4_gazebo_sim/run_pollination_simulation.py" ]]; then
  echo "Using existing clone: $REPO_DIR"
  git -C "$REPO_DIR" pull --ff-only || true
else
  echo "Cloning $REPO_URL -> $REPO_DIR"
  git clone "$REPO_URL" "$REPO_DIR"
fi

DEMO_DIR="$REPO_DIR/intelleswarm-ai/genai_framework/px4_gazebo_sim"
INSTALL_SH="$REPO_DIR/script/install_deps.sh"
test -f "$DEMO_DIR/run_pollination_simulation.py"
test -f "$DEMO_DIR/px4_multi_drone.sh"
test -f "$DEMO_DIR/agricultural_farm.world"
test -f "$DEMO_DIR/multi_drone_script/drone_controller.py"
test -f "$INSTALL_SH"

sed -i 's/\r$//' "$INSTALL_SH" "$DEMO_DIR"/*.sh "$DEMO_DIR"/*.py "$DEMO_DIR"/multi_drone_script/*.py 2>/dev/null || true
chmod +x "$INSTALL_SH" "$DEMO_DIR/px4_multi_drone.sh"

if [[ "${SKIP_INSTALL:-0}" != "1" ]]; then
  echo
  echo "=== Checking / installing dependencies ==="
  bash "$INSTALL_SH"
else
  echo "SKIP_INSTALL=1 — not installing dependencies"
fi

if [[ "${SKIP_RUN:-0}" == "1" ]]; then
  echo "SKIP_RUN=1 — install done. Later run:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  source \"\${ROS2_WS:-\$HOME/ros2_ws}/install/setup.bash\""
  echo "  cd \"$DEMO_DIR\""
  echo "  python3 run_pollination_simulation.py --num-drones $NUM_DRONES --duration $DURATION"
  exit 0
fi

echo
echo "=== Starting mission: $NUM_DRONES drones, duration ${DURATION}s ==="
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROS2_WS/install/setup.bash"
fi
set -u

cd "$DEMO_DIR"
export DISPLAY="${DISPLAY:-:0}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export HOME="${HOME:-$(eval echo ~)}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Main/PX4-Autopilot}"

python3 run_pollination_simulation.py \
  --num-drones "$NUM_DRONES" \
  --duration "$DURATION" \
  --px4-dir "$PX4_DIR"

echo "DONE."
