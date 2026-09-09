#!/usr/bin/env bash
# Print what is missing. Run inside WSL/Ubuntu after sourcing ROS 2.
set -u
echo "user=$(whoami)  home=$HOME  pwd=$(pwd)"
echo "uname=$(uname -a)"
echo "---"
echo -n "gz: "; command -v gz || echo MISSING
echo -n "MicroXRCEAgent: "; command -v MicroXRCEAgent || echo MISSING
echo -n "ros2: "; command -v ros2 || echo MISSING
echo "humble setup: $([[ -f /opt/ros/humble/setup.bash ]] && echo yes || echo NO)"
echo "ros2_ws overlay: $([[ -f $HOME/ros2_ws/install/setup.bash ]] && echo yes || echo NO)"
for d in "$HOME/PX4-Main/PX4-Autopilot" "$HOME/PX4-Autopilot" "${PX4_DIR:-}"; do
  [[ -z "$d" ]] && continue
  bin="$d/build/px4_sitl_default/bin/px4"
  if [[ -x "$bin" ]]; then echo "PX4 ok: $bin"; else echo "PX4 missing: $bin"; fi
done
echo "leftover gz: $(pgrep -af 'gz sim' || true)"
echo "leftover px4: $(pgrep -x px4 || true)"
python3 - <<'PY'
try:
    from px4_msgs.msg import VehicleCommand
    print("px4_msgs import: ok")
except Exception as e:
    print("px4_msgs import: FAIL", e)
PY
