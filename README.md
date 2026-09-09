# Multi-drone pollination simulation (PX4 + Gazebo Harmonic)

Clone this repo and run a 6-drone pollination mission in `agricultural_farm.world` using PX4 `x500_mono_cam`. Each drone covers a grid over its own spawn, then returns home and lands.

**Run inside WSL Ubuntu 22.04 (or Ubuntu 22.04), not Windows CMD.**

## Already have ROS 2, PX4, and Gazebo Harmonic?

```bash
git clone https://github.com/AbdurRouf230/Only_simulation_code_repo.git
cd Only_simulation_code_repo
bash install_and_run.sh
```

Default flight: 6 drones, 200 seconds max (`--duration`). Override:

```bash
NUM_DRONES=3 DURATION=120 bash install_and_run.sh
```

Or skip the dependency installer if everything is already built:

```bash
SKIP_INSTALL=1 NUM_DRONES=6 DURATION=200 bash install_and_run.sh
```

## What the installer checks / installs

`install_and_run.sh` calls `script/install_deps.sh`, which looks for:

1. ROS 2 Humble (`/opt/ros/humble`)
2. Gazebo Harmonic (`gz sim`)
3. PX4 SITL at `~/PX4-Main/PX4-Autopilot` (builds `gz_x500_mono_cam` if missing)
4. `MicroXRCEAgent`
5. `px4_msgs` in `~/ros2_ws`

The first PX4 build can take 30–60 minutes.

## Manual run (same as the installer)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/Only_simulation_code_repo/intelleswarm-ai/genai_framework/px4_gazebo_sim
python3 run_pollination_simulation.py --num-drones 6 --duration 200
```

If PX4 is not under `~/PX4-Main/PX4-Autopilot`:

```bash
python3 run_pollination_simulation.py --num-drones 6 --duration 200 --px4-dir ~/PX4-Autopilot
```

Headless (no Gazebo window):

```bash
python3 run_pollination_simulation.py --num-drones 6 --duration 200 --headless
```

## How it works

1. Gazebo Harmonic loads `agricultural_farm.world`.
2. Six PX4 SITL instances spawn (`gz_x500_mono_cam`) at different field poses.
3. Micro XRCE-DDS agents expose each drone as `/px4_0` … `/px4_5` in ROS 2.
4. The ROS 2 node flies each drone in offboard: takeoff, lawnmower grid over spawn, return home, land.
5. `--duration` is the max flight time. If the grid finishes earlier, they land then.

## Layout

```
install_and_run.sh
script/install_deps.sh
intelleswarm-ai/genai_framework/px4_gazebo_sim/
  run_pollination_simulation.py      # starts Gazebo, PX4, XRCE, mission
  ros2_node_intelleswarm_pollination.py
  px4_multi_drone.sh
  agricultural_farm.world
  pollination_drone_config.yaml
  multi_drone_script/
    drone_controller.py              # PX4 offboard + camera
    mission_logic.py                 # grid + land
    ...
```

## Notes

- `bash install_and_run.sh` already starts the mission. Do **not** also run `python3 run_pollination_simulation.py` in another terminal unless the installer finished or you used `SKIP_RUN=1`.
- Do not run two Gazebo / PX4 sessions at once.
- Stop a run with Ctrl+C. Leftovers: `pkill -f "gz sim"; pkill -x px4; pkill -f MicroXRCEAgent`.
- Camera stills (if captured) go under `multi_drone_script/captures/`.

## If it does not start

Clone as your normal user into home, not as root into `/`:

```bash
cd ~
git clone https://github.com/AbdurRouf230/Only_simulation_code_repo.git
cd Only_simulation_code_repo
git pull
```

Kill any leftover sim, then run **only one** of these:

```bash
pkill -f "gz sim"; pkill -x px4; pkill -f MicroXRCEAgent; pkill -f ros2_node_intelleswarm_pollination.py

# A) installer (installs missing deps, then flies)
SKIP_INSTALL=1 bash install_and_run.sh

# B) manual (same thing, after source)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/Only_simulation_code_repo/intelleswarm-ai/genai_framework/px4_gazebo_sim
python3 run_pollination_simulation.py --num-drones 6 --duration 200
```

Quick checks:

```bash
which gz
which MicroXRCEAgent
ls ~/PX4-Main/PX4-Autopilot/build/px4_sitl_default/bin/px4
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -c "from px4_msgs.msg import VehicleCommand; print('px4_msgs ok')"
```

If PX4 lives in `~/PX4-Autopilot`, pass `--px4-dir ~/PX4-Autopilot`.
If there is no GUI, add `--headless`.
