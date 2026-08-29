# Share with a friend who only has Harmonic

## Folder: `Only simulation run/script/`

| File | Purpose |
|------|---------|
| `install_classic_beside_harmonic.sh` | Auto-install Classic **beside** Harmonic |
| `README.md` (this file) | How to use |

Also send: `../intelleswarm-ai/genai_framework/px4_gazebo_sim/`  
(`run_pollination_simulation.py`, `px4_multi_drone.sh`, `agricultural_farm.world`)

## Friend runs (WSL Ubuntu 22.04)

```bash
cd /path/to/script
chmod +x install_classic_beside_harmonic.sh
bash install_classic_beside_harmonic.sh
# sudo + 30–90+ min first build

command -v gz && command -v gazebo && command -v gz11
test -x ~/PX4-Classic/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo PX4_CLASSIC_OK

source ~/env_px4_classic.sh
cd /path/to/px4_gazebo_sim
sed -i 's/\r$//' *.sh *.py
chmod +x px4_multi_drone.sh
unset DONT_RUN HEADLESS
python3 run_pollination_simulation.py --num-drones 1 --duration 90
```

## Protect Harmonic

- Do not remove Harmonic `gz`
- One stack per terminal: `env_px4_classic.sh` vs `env_px4_harmonic.sh`
- Do not run Classic + Harmonic at the same time
