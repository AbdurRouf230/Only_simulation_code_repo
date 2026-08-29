#!/usr/bin/env bash
##############################################################################
# install_classic_beside_harmonic.sh
#
# One-shot installer: Gazebo Classic 11 + PX4-Classic (iris SITL)
# NEXT TO an existing Gazebo Harmonic / PX4-Main setup.
#
# SAFE FOR HARMONIC:
#   - Does NOT uninstall Harmonic
#   - Does NOT remove / overwrite the Harmonic `gz` binary permanently
#   - Uses separate tree: ~/PX4-Classic/PX4-Autopilot
#   - Classic CLI: gazebo / gz11   |   Harmonic CLI: gz
#
# Target: Ubuntu 22.04 (Jammy) / WSL2
# Needs: sudo, internet, ~30–90+ min first build
#
# Usage:
#   cd "Only simulation run/script"
#   chmod +x install_classic_beside_harmonic.sh
#   bash install_classic_beside_harmonic.sh
#
# Optional env:
#   PX4_MAIN=~/PX4-Main/PX4-Autopilot   # copy from Harmonic tree if present
#   SKIP_APT=1                         # skip gazebo apt install
#   SKIP_BUILD=1                       # skip PX4 make (tree only)
##############################################################################
set -euo pipefail

echo "============================================================="
echo " Classic beside Harmonic — automatic install"
echo "============================================================="
echo "Harmonic (gz / PX4-Main) will be LEFT ALONE."
echo "Classic goes to: ~/PX4-Classic/PX4-Autopilot"
echo

PX4_MAIN="${PX4_MAIN:-$HOME/PX4-Main/PX4-Autopilot}"
if [[ ! -d "$PX4_MAIN" && -d "$HOME/PX4-Autopilot" ]]; then
  PX4_MAIN="$HOME/PX4-Autopilot"
fi
DST_ROOT="${PX4_CLASSIC_ROOT:-$HOME/PX4-Classic}"
DST="$DST_ROOT/PX4-Autopilot"
HARMONIC_GZ_BEFORE=""
command -v gz >/dev/null 2>&1 && HARMONIC_GZ_BEFORE="$(command -v gz)" || true

sudo apt-get update -y
sudo apt-get install -y \
  git rsync curl wget ca-certificates \
  build-essential cmake ninja-build \
  python3 python3-pip python3-venv \
  lsb-release gnupg software-properties-common

if [[ "${SKIP_APT:-0}" != "1" ]]; then
  echo
  echo "=== [1/4] Install Gazebo Classic 11 (keep Harmonic gz) ==="
  if ! grep -Rqs "gazebo11-gz-cli" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
    sudo add-apt-repository -y ppa:openrobotics/gazebo11-gz-cli || true
  fi
  sudo apt-get update -y
  if ! sudo apt-get install -y gazebo libgazebo-dev; then
    sudo apt-get install -y gazebo11 libgazebo11-dev || {
      echo "ERROR: could not install Gazebo Classic"
      exit 1
    }
  fi
else
  echo "[1/4] SKIP_APT=1"
fi

echo
echo "--- Classic / Harmonic CLI check ---"
command -v gazebo && gazebo --version | head -1 || echo "WARN: gazebo missing"
command -v gz11 && echo "gz11 OK (Classic)" || echo "WARN: gz11 missing"
if command -v gz >/dev/null 2>&1; then
  echo "Harmonic gz still present: $(command -v gz)"
else
  echo "WARN: Harmonic gz not on PATH"
fi
if [[ -n "$HARMONIC_GZ_BEFORE" && -x "$HARMONIC_GZ_BEFORE" ]]; then
  echo "Pre-install Harmonic gz path still exists: $HARMONIC_GZ_BEFORE"
fi

if command -v gz11 >/dev/null 2>&1; then
  WRAP="$HOME/.local/bin/classic-gz-wrap"
  mkdir -p "$WRAP"
  ln -sfn "$(command -v gz11)" "$WRAP/gz"
  echo "Classic gz wrap: $WRAP/gz -> $(readlink -f "$WRAP/gz")"
fi

echo
echo "=== [2/4] PX4-Classic tree at $DST ==="
mkdir -p "$DST_ROOT"

if [[ -f "$DST/CMakeLists.txt" ]]; then
  echo "PX4-Classic already present — skip clone/copy"
elif [[ -d "$PX4_MAIN" && -f "$PX4_MAIN/CMakeLists.txt" ]]; then
  echo "Copying from Harmonic tree (read-only source): $PX4_MAIN"
  rsync -a --info=progress2 \
    --exclude 'build/' \
    --exclude 'build_*/' \
    "$PX4_MAIN/" "$DST/" || {
      echo "rsync failed — cloning instead"
      rm -rf "$DST"
      git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$DST"
    }
else
  echo "No PX4-Main found — cloning fresh PX4-Autopilot into Classic tree"
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$DST"
fi

cd "$DST"
if [[ -f .gitmodules ]]; then
  git submodule update --init --recursive || true
fi

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  echo
  echo "=== [3/4] Build Classic SITL (gazebo-classic_iris) ==="
  echo "First build often takes 20–90 minutes..."
  export DONT_RUN=1
  unset HEADLESS || true
  if make px4_sitl gazebo-classic_iris; then
    echo "Build OK: gazebo-classic_iris"
  elif make px4_sitl gazebo-classic; then
    echo "Build OK: gazebo-classic (fallback)"
  else
    echo "ERROR: Classic SITL build failed"
    exit 1
  fi
  test -x "$DST/build/px4_sitl_default/bin/px4"
  echo "PX4 binary OK: $DST/build/px4_sitl_default/bin/px4"
else
  echo "[3/4] SKIP_BUILD=1"
fi

echo
echo "=== [4/4] Write ~/env_px4_classic.sh and ~/env_px4_harmonic.sh ==="

cat > "$HOME/env_px4_classic.sh" <<'EOF'
# source ~/env_px4_classic.sh   — Classic-only session
export PX4_DIR="$HOME/PX4-Classic/PX4-Autopilot"
export PX4_SIMULATOR=gazebo-classic
export PX4_SIM_MODEL=iris
export DISPLAY="${DISPLAY:-:0}"
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
unset DONT_RUN
unset HEADLESS
unset PX4_GZ_WORLD
if [[ -x "$HOME/.local/bin/classic-gz-wrap/gz" ]]; then
  export PATH="$HOME/.local/bin/classic-gz-wrap:/usr/bin:${PATH}"
fi
echo "Classic env: PX4_DIR=$PX4_DIR  gazebo=$(command -v gazebo)"
EOF

cat > "$HOME/env_px4_harmonic.sh" <<'EOF'
# source ~/env_px4_harmonic.sh  — Harmonic-only session
if [[ -d "$HOME/PX4-Main/PX4-Autopilot" ]]; then
  export PX4_DIR="$HOME/PX4-Main/PX4-Autopilot"
elif [[ -d "$HOME/PX4-Autopilot" ]]; then
  export PX4_DIR="$HOME/PX4-Autopilot"
fi
export PX4_SIMULATOR=gz
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500_mono_cam}"
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v classic-gz-wrap | paste -sd:)"
echo "Harmonic env: PX4_DIR=$PX4_DIR  gz=$(command -v gz)"
EOF

chmod +x "$HOME/env_px4_classic.sh" "$HOME/env_px4_harmonic.sh"

echo
echo "============================================================="
echo " DONE — Classic installed beside Harmonic"
echo "============================================================="
echo "Verify:"
echo "  command -v gazebo; command -v gz11; command -v gz"
echo "  test -x ~/PX4-Classic/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo PX4_CLASSIC_OK"
echo
echo "  source ~/env_px4_classic.sh"
echo "  cd /path/to/px4_gazebo_sim"
echo "  python3 run_pollination_simulation.py --num-drones 1 --duration 90"
echo "============================================================="
