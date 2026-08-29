#!/usr/bin/env bash
##############################################################################
# install_and_run.sh
#
# Friend one-shot:
#   1) Clone this GitHub repo (if needed)
#   2) Install Gazebo Classic 11 + PX4-Classic BESIDE Harmonic (no Harmonic wipe)
#   3) Run the timed GUI pollination demo
#
# Usage (recommended):
#   git clone https://github.com/AbdurRouf230/Only_simulation_code_repo.git
#   cd Only_simulation_code_repo
#   bash install_and_run.sh
#
# Or:
#   curl -fsSL https://raw.githubusercontent.com/AbdurRouf230/Only_simulation_code_repo/main/install_and_run.sh | bash
#
# Env:
#   SKIP_INSTALL=1   # only clone/update + run demo (Classic already installed)
#   SKIP_RUN=1       # only install Classic, do not start demo
#   NUM_DRONES=1
#   DURATION=90
#   REPO_DIR=~/Only_simulation_code_repo
##############################################################################
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/AbdurRouf230/Only_simulation_code_repo.git}"
REPO_DIR="${REPO_DIR:-$HOME/Only_simulation_code_repo}"
NUM_DRONES="${NUM_DRONES:-1}"
DURATION="${DURATION:-90}"

echo "============================================================="
echo " Only_simulation_code_repo — install Classic + run demo"
echo "============================================================="

# ---- Locate or clone repo ----
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}" 2>/dev/null || realpath "${BASH_SOURCE[0]}" 2>/dev/null || echo "")"
if [[ -n "$SCRIPT_PATH" && -f "$(dirname "$SCRIPT_PATH")/intelleswarm-ai/genai_framework/px4_gazebo_sim/run_pollination_simulation.py" ]]; then
  REPO_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
  echo "Using local repo: $REPO_DIR"
elif [[ -f "$REPO_DIR/intelleswarm-ai/genai_framework/px4_gazebo_sim/run_pollination_simulation.py" ]]; then
  echo "Using existing clone: $REPO_DIR"
  git -C "$REPO_DIR" pull --ff-only || true
else
  echo "Cloning $REPO_URL -> $REPO_DIR"
  rm -rf "$REPO_DIR"
  git clone "$REPO_URL" "$REPO_DIR"
fi

DEMO_DIR="$REPO_DIR/intelleswarm-ai/genai_framework/px4_gazebo_sim"
INSTALL_SH="$REPO_DIR/script/install_classic_beside_harmonic.sh"
test -f "$DEMO_DIR/run_pollination_simulation.py"
test -f "$DEMO_DIR/px4_multi_drone.sh"
test -f "$DEMO_DIR/agricultural_farm.world"
test -f "$INSTALL_SH"

# Fix Windows CRLF if present
sed -i 's/\r$//' "$INSTALL_SH" "$DEMO_DIR"/*.sh "$DEMO_DIR"/*.py 2>/dev/null || true
chmod +x "$INSTALL_SH" "$DEMO_DIR/px4_multi_drone.sh"

# ---- Install Classic beside Harmonic ----
if [[ "${SKIP_INSTALL:-0}" != "1" ]]; then
  echo
  echo "=== Installing Classic (Harmonic left alone) ==="
  bash "$INSTALL_SH"
else
  echo "SKIP_INSTALL=1 — skipping Classic apt/PX4 build"
fi

# ---- Run timed GUI demo ----
if [[ "${SKIP_RUN:-0}" == "1" ]]; then
  echo "SKIP_RUN=1 — install done. Demo not started."
  echo "Later:"
  echo "  source ~/env_px4_classic.sh"
  echo "  cd \"$DEMO_DIR\""
  echo "  python3 run_pollination_simulation.py --num-drones $NUM_DRONES --duration $DURATION"
  exit 0
fi

echo
echo "=== Running timed pollination demo (GUI) ==="
# shellcheck disable=SC1090
source "$HOME/env_px4_classic.sh"
cd "$DEMO_DIR"
export DISPLAY="${DISPLAY:-:0}"
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
unset DONT_RUN HEADLESS
python3 run_pollination_simulation.py --num-drones "$NUM_DRONES" --duration "$DURATION"
echo "DONE. Results JSON in: $DEMO_DIR/pollination_simulation_results_*.json"
