#!/usr/bin/env bash
##############################################################################
# IntelleSwarm AI - PX4 Multi-Drone Pollination Swarm Launcher (Gazebo Classic)
#
# Converted from mixed gz-sim/iris to clean Classic:
#   - Gazebo 11 (gazebo / gzserver / gzclient) — NOT Harmonic `gz sim`
#   - Model: iris
#   - PX4 tree: ~/PX4-Classic/PX4-Autopilot (does not touch PX4-Main / Harmonic)
#
# Author: IntelleSwarm AI Team (Classic conversion)
##############################################################################
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Classic/PX4-Autopilot}"
GAZEBO_SITL_DIR="$PX4_DIR/Tools/simulation/gazebo-classic"
WORLDS_DIR="$GAZEBO_SITL_DIR/sitl_gazebo-classic/worlds"
# Same farm world as Zahid repo; override with WORLD_FILE=/path/to/other.world if needed
WORLD_FILE="${WORLD_FILE:-$SCRIPT_DIR/agricultural_farm.world}"
NUM_DRONES="${NUM_DRONES:-6}"
LOGDIR="${LOGDIR:-/tmp/px4_classic_only_sim}"
mkdir -p "$LOGDIR"

# Resolve world path (name or absolute). Install into PX4 Classic worlds/ so sitl_run can load it.
resolve_world() {
  local w="$1"
  if [[ -f "$w" ]]; then
    echo "$w"
    return
  fi
  if [[ -f "$SCRIPT_DIR/$w" ]]; then
    echo "$SCRIPT_DIR/$w"
    return
  fi
  if [[ -f "$SCRIPT_DIR/${w}.world" ]]; then
    echo "$SCRIPT_DIR/${w}.world"
    return
  fi
  if [[ -f "$WORLDS_DIR/${w}.world" ]]; then
    echo "$WORLDS_DIR/${w}.world"
    return
  fi
  if [[ -f "$WORLDS_DIR/$w" ]]; then
    echo "$WORLDS_DIR/$w"
    return
  fi
  echo ""
}

WORLD_PATH="$(resolve_world "$WORLD_FILE")"
if [[ -n "$WORLD_PATH" ]]; then
  WORLD_BASENAME="$(basename "$WORLD_PATH")"
  WORLD_STEM="${WORLD_BASENAME%.world}"
  mkdir -p "$WORLDS_DIR"
  cp -f "$WORLD_PATH" "$WORLDS_DIR/${WORLD_STEM}.world"
  export PX4_SITL_WORLD="$WORLD_STEM"
  export PX4_SIM_WORLD="$WORLD_STEM"
  echo "World: $WORLD_STEM -> $WORLDS_DIR/${WORLD_STEM}.world"
else
  echo "WARN: world not found ($WORLD_FILE) — falling back to Classic empty/iris default"
  unset PX4_SITL_WORLD || true
  WORLD_STEM=""
fi

# Pollination mission spawn positions (over different field sections)
# Format: x y z yaw  — used for logging / future multi-spawn; drone 0 uses PX4 default
SPAWN_POSITIONS=(
    "50 50 5 0"      # Sunflower patch 1 - Primary
    "-50 50 5 1.57"  # Sunflower patch 2 - Primary
    "0 0 5 0"        # Clover field - High priority
    "25 25 5 0.79"   # Between patches - Support
    "-25 25 5 2.36"  # Coverage support
    "0 75 5 1.57"    # Northern field coverage
)

MAVLINK_UDP_PORTS=(14560 14570 14580 14590 14600 14610)

export DISPLAY="${DISPLAY:-:0}"
export PX4_SIMULATOR=gazebo-classic
export PX4_SIM_MODEL=iris
# Respect HEADLESS=1 from caller; otherwise force GUI
if [[ "${HEADLESS:-}" == "1" ]]; then
  export HEADLESS=1
  echo "Mode: HEADLESS (no Gazebo window)"
else
  unset HEADLESS
  echo "Mode: GUI (DISPLAY=$DISPLAY)"
fi
# Critical: sitl_run.sh treats ANY non-empty DONT_RUN as "do not run"
unset DONT_RUN

# WSL: advertise localhost; Classic `gz model` must not hit Harmonic `gz`
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
if command -v gz11 >/dev/null 2>&1; then
  _GZWRAP="${HOME}/.local/bin/classic-gz-wrap"
  mkdir -p "$_GZWRAP"
  ln -sfn "$(command -v gz11)" "$_GZWRAP/gz"
  export PATH="$_GZWRAP:/usr/bin:${PATH}"
fi

echo "IntelleSwarm AI - Multi-Drone Pollination (Gazebo Classic)"
echo "============================================================="
echo "World/model: gazebo-classic_iris + ${WORLD_STEM:-default} (Classic 11)"
echo "PX4_DIR: $PX4_DIR"
echo "Drones: $NUM_DRONES"
echo "DISPLAY: $DISPLAY"
echo "============================================================="

if [[ ! -d "$PX4_DIR" ]]; then
    echo "ERROR: PX4-Classic not found at $PX4_DIR"
    echo "  Set PX4_DIR or build ~/PX4-Classic/PX4-Autopilot"
    exit 1
fi

if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
    echo "ERROR: PX4 binary missing. Build:"
    echo "  cd \$PX4_DIR && DONT_RUN=1 make px4_sitl gazebo-classic_iris"
    exit 1
fi

if ! command -v gazebo >/dev/null 2>&1; then
    echo "ERROR: gazebo (Classic 11) not installed"
    echo "  See run_classic.md / install_gazebo_classic_side_by_side.sh"
    exit 1
fi

cleanup() {
    # Prevent re-entry (pkill must NOT match this script path: .../px4_gazebo_sim/...)
    trap - SIGINT SIGTERM
    echo "Shutting down Classic simulation..."
    for pid in "${CHILD_PIDS[@]:-}"; do
      kill "$pid" 2>/dev/null || true
    done
    pkill -x px4 2>/dev/null || true
    pkill -f '/bin/px4' 2>/dev/null || true
    pkill -x gzserver 2>/dev/null || true
    pkill -x gzclient 2>/dev/null || true
    pkill -x gazebo 2>/dev/null || true
    # Do not use: pkill -f gazebo  — matches folder name px4_gazebo_sim and kills this launcher
    pkill -f 'gz sim' 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM
CHILD_PIDS=()

# Clear leftover sims (Classic session) — use -x / exact names only
pkill -f 'gz sim' 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x gazebo 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -f '/bin/px4' 2>/dev/null || true
sleep 2

# Strip ROS from library path — breaks Classic Gazebo plugins / gzclient
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  _clean=""
  IFS=':' read -ra _parts <<< "$LD_LIBRARY_PATH"
  for p in "${_parts[@]}"; do
    case "$p" in
      *ros*|*humble*|*ros2_ws*|*ament*|*colcon*) ;;
      *) _clean="${_clean:+$_clean:}$p" ;;
    esac
  done
  export LD_LIBRARY_PATH="$_clean"
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO || true

BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
JINJA="$GAZEBO_SITL_DIR/sitl_gazebo-classic/scripts/jinja_gen.py"
IRIS_JINJA="$GAZEBO_SITL_DIR/sitl_gazebo-classic/models/iris/iris.sdf.jinja"
PLUGIN_DIR="$GAZEBO_SITL_DIR/sitl_gazebo-classic"
WORLD_TO_LOAD="$WORLDS_DIR/empty.world"
if [[ -n "${WORLD_STEM:-}" ]] && [[ -f "$WORLDS_DIR/${WORLD_STEM}.world" ]]; then
  WORLD_TO_LOAD="$WORLDS_DIR/${WORLD_STEM}.world"
fi

cd "$PX4_DIR"
# shellcheck disable=SC1091
source "$GAZEBO_SITL_DIR/setup_gazebo.bash" "$PX4_DIR" "$BUILD_DIR"

# Clean ROS libs again after setup_gazebo
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  _clean=""
  IFS=':' read -ra _parts <<< "$LD_LIBRARY_PATH"
  for p in "${_parts[@]}"; do
    case "$p" in
      *ros*|*humble*|*ros2_ws*|*ament*|*colcon*) ;;
      *) _clean="${_clean:+$_clean:}$p" ;;
    esac
  done
  export LD_LIBRARY_PATH="$_clean"
fi

CHILD_PIDS=()

echo "Starting gzserver with world: $WORLD_TO_LOAD"
gzserver "$WORLD_TO_LOAD" --verbose >"$LOGDIR/gzserver.log" 2>&1 &
CHILD_PIDS+=($!)
# Farm world can be slow to load
for _ in $(seq 1 30); do
  sleep 1
  if pgrep -x gzserver >/dev/null; then
    # give physics a moment after process appears
    sleep 3
    break
  fi
done
if ! pgrep -x gzserver >/dev/null; then
  echo "ERROR: gzserver failed — see $LOGDIR/gzserver.log"
  tail -50 "$LOGDIR/gzserver.log" || true
  exit 1
fi
echo "gzserver OK"

if [[ "${HEADLESS:-}" != "1" ]]; then
  echo "Starting gzclient (GUI)..."
  gzclient --verbose >"$LOGDIR/gzclient.log" 2>&1 &
  CHILD_PIDS+=($!)
  sleep 3
  if ! pgrep -x gzclient >/dev/null; then
    echo "ERROR: gzclient failed — see $LOGDIR/gzclient.log"
    echo "HINT: export DISPLAY=:0 && gzclient"
    tail -40 "$LOGDIR/gzclient.log" || true
    exit 1
  fi
  echo "gzclient OK (look for Gazebo window)"
fi

# Spawn iris + start PX4 (same pattern as classic_real_flight multi-spawn)
if [[ ! -f "$JINJA" || ! -f "$IRIS_JINJA" ]]; then
  echo "ERROR: iris jinja assets missing"
  exit 1
fi

SDF_OUT="/tmp/iris_0_pollination_demo.sdf"
INST_DIR="$BUILD_DIR/rootfs_pollination_demo_0"
mkdir -p "$INST_DIR"
python3 "$JINJA" "$IRIS_JINJA" "$PLUGIN_DIR" \
  --mavlink_tcp_port 4560 \
  --mavlink_udp_port 14540 \
  --mavlink_id 1 \
  --gst_udp_port 5600 \
  --video_uri 5600 \
  --mavlink_cam_udp_port 14530 \
  --output-file "$SDF_OUT"

echo "Starting PX4 instance 0..."
(
  cd "$INST_DIR"
  export DISPLAY
  export GAZEBO_IP
  export GAZEBO_MASTER_URI
  export PATH
  export PX4_SIMULATOR=gazebo-classic
  export PX4_SIM_MODEL=gazebo-classic_iris
  export PX4_SYS_AUTOSTART=10016
  unset DONT_RUN
  unset LD_LIBRARY_PATH
  if [[ -d "$BUILD_DIR/etc" ]]; then
    "$BUILD_DIR/bin/px4" -i 0 -d "$BUILD_DIR/etc"
  else
    "$BUILD_DIR/bin/px4" -i 0 \
      -s "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS" \
      -t "$PX4_DIR/test_data" \
      -d "$PX4_DIR/platforms/posix/rootfs"
  fi
) >"$LOGDIR/drone0.log" 2>&1 &
CHILD_PIDS+=($!)
sleep 2

echo "Spawning iris_0 into farm world..."
if ! gz model --spawn-file="$SDF_OUT" --model-name="iris_0" \
    -x 0 -y 0 -z 0.83 >"$LOGDIR/spawn_0.log" 2>&1; then
  echo "ERROR: gz model spawn failed — see $LOGDIR/spawn_0.log"
  cat "$LOGDIR/spawn_0.log" || true
  exit 1
fi

# Wait for PX4 simulator link
ok=0
for _ in $(seq 1 24); do
  sleep 2
  if grep -q 'Simulator connected' "$LOGDIR/drone0.log" 2>/dev/null; then
    ok=1
    break
  fi
  if ! pgrep -f 'bin/px4' >/dev/null; then
    break
  fi
done

if [[ "$ok" -ne 1 ]]; then
  echo "ERROR: PX4 did not connect to Gazebo — see $LOGDIR/drone0.log"
  tr -cd '\11\12\15\40-\176' <"$LOGDIR/drone0.log" | tail -40 || true
  exit 1
fi
echo "Drone 1 OK (iris in $(basename "$WORLD_TO_LOAD") + GUI)"

# Keep first px4 pid as PID0 for compatibility with later checks
PID0=${CHILD_PIDS[-1]}

# --- Extra drones: additional PX4 instances joining Classic Gazebo ---
# Note: full multi-iris farm poses need Gazebo model insert; instances still start.
MAX=$NUM_DRONES
if [[ "$MAX" -gt 6 ]]; then
  MAX=6
fi
for i in $(seq 1 $((MAX - 1))); do
  drone_id=$((i + 1))
  spawn_pos=${SPAWN_POSITIONS[$i]:-"0 0 5 0"}
  IFS=' ' read -r spawn_x spawn_y spawn_z spawn_yaw <<< "$spawn_pos"
  mavlink_udp=${MAVLINK_UDP_PORTS[$i]:-$((14560 + i * 10))}

  echo "Launching Drone $drone_id (instance $i) pose~($spawn_x,$spawn_y,$spawn_z) UDP $mavlink_udp"

  (
    export DISPLAY
    export PX4_SIMULATOR=gazebo-classic
    export PX4_SIM_MODEL=iris
    export PX4_SYS_AUTOSTART=10016
    unset DONT_RUN
    cd "$PX4_DIR"
    "$PX4_DIR/build/px4_sitl_default/bin/px4" \
      -i "$i" \
      -s "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS" \
      -t "$PX4_DIR/test_data" \
      -d "$PX4_DIR/platforms/posix/rootfs"
  ) >"$LOGDIR/drone${i}.log" 2>&1 &

  sleep 3
  echo "  Drone $drone_id PX4 instance started"
done

echo ""
echo "Classic Pollination Swarm Ready"
echo "============================================================="
echo "Gazebo Classic GUI should show iris + farm world (if loaded)"
echo "World: ${PX4_SITL_WORLD:-default}"
echo "Logs: $LOGDIR"
echo "MAVLink (approx):"
for i in $(seq 0 $((MAX - 1))); do
  echo "  • Drone $((i + 1)): UDP ${MAVLINK_UDP_PORTS[$i]}"
done
echo "Ctrl+C to shutdown"
echo "============================================================="

# Keep script alive while PX4 runs
while true; do
  if ! pgrep -f "bin/px4" >/dev/null; then
    echo "No PX4 processes — exiting"
    break
  fi
  sleep 5
done

echo "PX4 Classic swarm processes completed"
