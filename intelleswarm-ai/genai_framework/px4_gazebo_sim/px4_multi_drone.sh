#!/bin/bash
##############################################################################
# IntelleSwarm AI - PX4 Multi-Drone Pollination Swarm Launcher
#
# This script launches multiple PX4 SITL instances for assistive pollination
# missions with coordinated takeoff positions over agricultural fields.
#
# Author: IntelleSwarm AI Team
# Date: May 2026
##############################################################################

# Configuration
WORLD_FILE="agricultural_farm.world"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR=${PX4_DIR:-"$HOME/PX4-Main/PX4-Autopilot"}
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_ETC="$PX4_DIR/build/px4_sitl_default/etc"
PX4_ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
GAZEBO_SITL_DIR="$PX4_DIR/Tools/simulation/gz"

# Number of drones (override with NUM_DRONES=3 python3 run_pollination_simulation.py --num-drones 3)
NUM_DRONES=${NUM_DRONES:-6}
if [ "$NUM_DRONES" -lt 1 ]; then
    NUM_DRONES=1
fi

# Pollination mission spawn positions (over different field sections)
# Format: x y z yaw — extras after index 5 are generated
SPAWN_POSITIONS=(
    "50 50 0.5 0"      # Sunflower patch 1
    "-50 50 0.5 1.57"  # Sunflower patch 2
    "0 0 0.5 0"        # Clover field
    "25 25 0.5 0.79"   # Support
    "-25 25 0.5 2.36"  # Coverage support
    "0 75 0.5 1.57"    # Northern field
)

echo "🚁 IntelleSwarm AI - Multi-Drone Pollination Simulation"
echo "============================================================="
echo "📍 World: $WORLD_FILE"
echo "🌾 Mission Type: Assistive Pollination"
echo "🤖 Drones: $NUM_DRONES"
echo "============================================================="

# Check if PX4-Autopilot exists
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ Error: PX4-Autopilot directory not found at $PX4_DIR"
    echo "   Please set PX4_DIR environment variable or install PX4-Autopilot"
    exit 1
fi

if [ ! -x "$PX4_BIN" ]; then
    echo "❌ Error: PX4 SITL binary not found at $PX4_BIN"
    echo "   Build PX4 with: cd $PX4_DIR && make px4_sitl gz_x500"
    exit 1
fi

if [ ! -d "$PX4_ETC" ]; then
    echo "❌ Error: PX4 rootfs/etc not found at $PX4_ETC"
    exit 1
fi

# Function to cleanup background processes
cleanup() {
    echo "🛑 Shutting down simulation..."
    pkill -f "px4"
    pkill -f "gz sim"
    exit 0
}

# Set cleanup trap
trap cleanup SIGINT SIGTERM

# PX4 gz_bridge waits on /world/${PX4_GZ_WORLD}/scene/info then
# spawns file://${PX4_GZ_MODELS}/x500/model.sdf. Those vars come from gz_env.sh.
# World name must match <world name="..."> in agricultural_farm.world.
if [ -f "$PX4_ROOTFS/gz_env.sh" ]; then
    # shellcheck disable=SC1091
    . "$PX4_ROOTFS/gz_env.sh"
fi

export PX4_SIM_MODEL=gz_x500_mono_cam
export PX4_SYS_AUTOSTART=4010
export PX4_SIMULATOR=gz
export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD=agricultural_farm
export PX4_SIM_WORLD="$SCRIPT_DIR/$WORLD_FILE"
export PX4_GZ_NO_FOLLOW=1
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export DISPLAY="${DISPLAY:-:0}"

wait_for_gz_world() {
    echo "⏳ Waiting for Gazebo world agricultural_farm..."
    local n=0
    while [ "$n" -lt 60 ]; do
        if gz service -i --service "/world/agricultural_farm/scene/info" 2>/dev/null | grep -q "Service providers"; then
            echo "✅ Gazebo world is ready"
            return 0
        fi
        n=$((n + 1))
        sleep 1
    done
    echo "❌ Gazebo world agricultural_farm is not ready"
    return 1
}

# Check if Gazebo is already running (from main simulation)
if pgrep -f "gz sim" > /dev/null; then
    echo "🌍 Gazebo already running, using existing instance..."
else
    echo "🌍 Starting Gazebo with agricultural farm world..."
    cd "$PX4_DIR"
    gz sim -r -s "$SCRIPT_DIR/$WORLD_FILE" &
    GAZEBO_PID=$!
    echo "⏳ Waiting for Gazebo to initialize..."
    sleep 8
fi

if ! wait_for_gz_world; then
    exit 1
fi

# Allow ROS offboard to arm in SITL without QGroundControl (datalink/RC checks).
SITL_PARAMS="$SCRIPT_DIR/multi_drone_script/px4-rc.params"
if [ -f "$SITL_PARAMS" ]; then
    mkdir -p "$PX4_ETC/init.d-posix"
    cp "$SITL_PARAMS" "$PX4_ETC/init.d-posix/px4-rc.params"
    AF="$PX4_ETC/init.d-posix/airframes/4010_gz_x500_mono_cam"
    if [ -f "$AF" ] && ! grep -q "pollination_sitl_arm" "$AF"; then
        {
            echo ""
            echo "# pollination_sitl_arm"
            grep -E '^param ' "$SITL_PARAMS" || true
        } >> "$AF"
        echo "✅ SITL arm params added to $AF"
    else
        echo "✅ SITL arm params: $PX4_ETC/init.d-posix/px4-rc.params"
    fi
fi

# Launch PX4 SITL instances for each drone (instance 0..N-1 matches /px4_0 .. /px4_N)
for i in $(seq 0 $((NUM_DRONES-1))); do
    drone_id=$i
    xrce_port=$((8888 + i))
    mavlink_udp=$((14560 + i * 10))
    mavlink_tcp=$((4560 + i * 10))

    spawn_pos=${SPAWN_POSITIONS[$i]:-}
    if [ -z "$spawn_pos" ]; then
        extra=$((i - 6))
        spawn_pos="$((extra * 10)) -50 0.5 0"
    fi
    IFS=' ' read -r spawn_x spawn_y spawn_z spawn_yaw <<< "$spawn_pos"

    echo "🚁 Launching Drone $drone_id at position ($spawn_x, $spawn_y, $spawn_z)"
    echo "   ROS ns: /px4_${drone_id}  XRCE UDP: $xrce_port"
    echo "   MAVLink UDP: $mavlink_udp, TCP: $mavlink_tcp"

    export PX4_SIM_MODEL=gz_x500_mono_cam
    export PX4_SYS_AUTOSTART=4010
    export PX4_SIM_HOSTNAME=localhost
    export PX4_SIMULATOR=gz
    export PX4_GZ_STANDALONE=1
    export PX4_GZ_WORLD=agricultural_farm
    export PX4_GZ_NO_FOLLOW=1
    export PX4_GZ_MODEL_POSE="${spawn_x},${spawn_y},${spawn_z},0,0,${spawn_yaw}"
    export PX4_UXRCE_DDS_NS="px4_${drone_id}"
    export PX4_UXRCE_DDS_PORT="$xrce_port"

    export PX4_HOME_LAT=37.7749
    export PX4_HOME_LON=-122.4194
    export PX4_HOME_ALT=30.0

    INSTANCE_DIR="$PX4_ROOTFS/instance_${drone_id}"
    mkdir -p "$INSTANCE_DIR"

    cd "$INSTANCE_DIR"
    "$PX4_BIN" \
        -i $drone_id \
        -d \
        -s "$PX4_ETC/init.d-posix/rcS" \
        -w "$INSTANCE_DIR" \
        "$PX4_ETC" \
        > "$INSTANCE_DIR/out.log" 2>&1 &

    echo "   ✅ Drone $drone_id PX4 instance started (log: $INSTANCE_DIR/out.log)"
    sleep 3
done

echo ""
echo "🎯 Pollination Swarm Ready!"
echo "============================================================="
echo "🌾 Agricultural Field Coverage:"
echo "   • Sunflower patches: Drones 1, 2"
echo "   • Clover field: Drone 3 (high priority)"
echo "   • Support coverage: Drones 4, 5, 6"
echo ""
echo "📡 MAVLink / ROS 2:"
for i in $(seq 0 $((NUM_DRONES-1))); do
    mavlink_udp=$((14560 + i * 10))
    echo "   • Drone $i: /px4_${i}  XRCE $((8888 + i))  MAVLink UDP $mavlink_udp"
done
echo ""
echo "🎮 Control with:"
echo "   • QGroundControl: Connect to UDP ports above"
echo "   • ROS 2: Use ros2_node_intelleswarm_pollination.py"
echo "   • MARL Training: Use run_pollination_simulation.py"
echo ""
echo "🛑 Press Ctrl+C to shutdown all drones"
echo "============================================================="

# Give processes time to fully initialize
sleep 5

# Keep script running to maintain processes
echo "🏃 Drones running, script active..."
# Monitor processes without blocking the simulation
while true; do
    if ! pgrep -x "px4" > /dev/null; then
        echo "⚠️  No PX4 processes detected, exiting..."
        break
    fi
    sleep 5
done

echo "🏁 PX4 swarm processes completed"
