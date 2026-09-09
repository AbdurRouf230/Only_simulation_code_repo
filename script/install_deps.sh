#!/usr/bin/env bash
##############################################################################
# Install ROS 2 Humble, Gazebo Harmonic, PX4 SITL, MicroXRCEAgent, px4_msgs.
# Safe to re-run. Does not remove an existing Harmonic gz install.
#
# Env:
#   PX4_DIR=~/PX4-Main/PX4-Autopilot
#   ROS2_WS=~/ros2_ws
#   SKIP_PX4_BUILD=1
##############################################################################
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This installer is for Ubuntu 22.04 / WSL2 Ubuntu."
  exit 1
fi

if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    echo "Warning: this stack is tested on Ubuntu 22.04 (found ${VERSION_ID:-unknown})."
  fi
fi

PX4_DIR="${PX4_DIR:-$HOME/PX4-Main/PX4-Autopilot}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
export DEBIAN_FRONTEND=noninteractive

need_cmd() { command -v "$1" >/dev/null 2>&1; }

sudo apt-get update
sudo apt-get install -y \
  git cmake build-essential python3 python3-pip python3-venv \
  wget curl lsb-release gnupg software-properties-common

# ---- ROS 2 Humble ----
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Installing ROS 2 Humble..."
  sudo apt-get install -y software-properties-common
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
  fi
  rosdep update || true
else
  echo "ROS 2 Humble already present."
fi

# ---- Gazebo Harmonic ----
if ! need_cmd gz; then
  echo "Installing Gazebo Harmonic..."
  sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y gz-harmonic
else
  echo "Gazebo already present: $(gz sim --versions 2>/dev/null | head -n1 || echo gz)"
fi

# python bits used by the controller
python3 -m pip install --user --upgrade pip || true
python3 -m pip install --user pyyaml numpy pymavlink || true

# ---- PX4 ----
if [[ -x "$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4" && ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  PX4_DIR="$HOME/PX4-Autopilot"
  echo "Using existing PX4 at $PX4_DIR"
fi
if [[ "${SKIP_PX4_BUILD:-0}" == "1" ]]; then
  echo "SKIP_PX4_BUILD=1"
elif [[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  echo "PX4 SITL already built at $PX4_DIR"
else
  echo "Building PX4 SITL at $PX4_DIR (this can take a long time)..."
  mkdir -p "$(dirname "$PX4_DIR")"
  if [[ ! -d "$PX4_DIR/.git" ]]; then
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive "$PX4_DIR"
  fi
  if [[ -f "$PX4_DIR/Tools/setup/ubuntu.sh" ]]; then
    bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-sim-tools || bash "$PX4_DIR/Tools/setup/ubuntu.sh" || true
  fi
  (
    cd "$PX4_DIR"
    make px4_sitl gz_x500_mono_cam
  )
fi

if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  echo "ERROR: PX4 binary not found at $PX4_DIR/build/px4_sitl_default/bin/px4"
  echo "Set PX4_DIR to your Autopilot tree or run: cd \"\$PX4_DIR\" && make px4_sitl gz_x500_mono_cam"
  exit 1
fi

# ---- Micro XRCE-DDS Agent ----
if ! need_cmd MicroXRCEAgent; then
  echo "Building MicroXRCEAgent..."
  AGENT_DIR="${MICRO_XRCE_DIR:-$HOME/Micro-XRCE-DDS-Agent}"
  if [[ ! -d "$AGENT_DIR/.git" ]]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
  fi
  sudo apt-get install -y cmake g++
  cmake -S "$AGENT_DIR" -B "$AGENT_DIR/build"
  cmake --build "$AGENT_DIR/build" -j"$(nproc)"
  sudo cmake --install "$AGENT_DIR/build"
  sudo ldconfig
else
  echo "MicroXRCEAgent already on PATH."
fi

if ! need_cmd MicroXRCEAgent; then
  echo "ERROR: MicroXRCEAgent not found after install."
  exit 1
fi

# ---- px4_msgs workspace ----
mkdir -p "$ROS2_WS/src"
if [[ ! -d "$ROS2_WS/src/px4_msgs/.git" ]]; then
  git clone https://github.com/PX4/px4_msgs.git "$ROS2_WS/src/px4_msgs"
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f "$ROS2_WS/install/px4_msgs/share/px4_msgs/package.xml" && ! -d "$ROS2_WS/install/px4_msgs" ]]; then
  echo "Building px4_msgs in $ROS2_WS ..."
  cd "$ROS2_WS"
  colcon build --packages-select px4_msgs --symlink-install
fi

echo
echo "Dependencies OK."
echo "  ROS 2:  /opt/ros/humble"
echo "  PX4:    $PX4_DIR"
echo "  ROS2_WS:$ROS2_WS"
echo "  Agent:  $(command -v MicroXRCEAgent)"
echo "  Gazebo: $(command -v gz)"
