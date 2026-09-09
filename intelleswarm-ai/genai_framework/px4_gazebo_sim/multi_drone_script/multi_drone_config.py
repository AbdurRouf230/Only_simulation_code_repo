"""Shared settings for multi-drone fleet."""

# Gazebo spawn: metres between drone home positions (X axis)
SPAWN_SEPARATION_M = 2.0

# Each drone occupies a 2x2x2 m safety box — no other drone may enter
SAFETY_BOX_M = 2.0

# If XY conflict: try raising altitude by this many metres (NED z more negative)
ALTITUDE_STEP_M = 2.0
MAX_ALTITUDE_STEPS = 4

# If altitude cannot resolve conflict: delay before same waypoint (seconds)
WAYPOINT_DELAY_BASE_S = 3.0

# Mission grid (same as single-drone drone0.py)
MISSION_GROUND_SIZE_M = 25.0
MISSION_GRID = 5
YAW_CAPTURE_STEP_DEG = 90.0

# XRCE / PX4
XRCE_BASE_PORT = 8888
# Use $HOME not ~ — bash does not expand ~ inside ${VAR:-default}
PX4_DIR_DEFAULT = '$HOME/PX4-Main/PX4-Autopilot'

# Gazebo world (must match on drone 0 and all STANDALONE instances)
GZ_WORLD_DEFAULT = 'baylands'

# Fleet state file (all drones read/write)
FLEET_STATE_PATH = '/tmp/multidrone_fleet.json'
