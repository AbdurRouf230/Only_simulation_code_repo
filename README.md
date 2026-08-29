# Only_simulation_code_repo

Classic **timed pollination demo** (GUI) that can run **beside** Gazebo Harmonic without removing Harmonic.

> Mission scores are **timer-based / simulated**. Gazebo shows farm world + iris.

## For your friend (Harmonic already installed)

```bash
git clone https://github.com/AbdurRouf230/Only_simulation_code_repo.git
cd Only_simulation_code_repo
bash install_and_run.sh
```

This will:
1. Use/clone this repo  
2. Install Gazebo Classic 11 + `~/PX4-Classic` **without harming Harmonic `gz`**  
3. Run: `python3 run_pollination_simulation.py --num-drones 1 --duration 90`

Optional:
```bash
SKIP_INSTALL=1 bash install_and_run.sh   # Classic already installed — only run demo
SKIP_RUN=1 bash install_and_run.sh       # only install Classic
NUM_DRONES=1 DURATION=120 bash install_and_run.sh
```

## One-liner (after repo is public)

```bash
bash <(curl -fsSL https://raw.githubusercontent.com/AbdurRouf230/Only_simulation_code_repo/main/install_and_run.sh)
```

## Protect Harmonic

- Do **not** delete Harmonic `gz`
- Classic session: `source ~/env_px4_classic.sh`
- Harmonic session: `source ~/env_px4_harmonic.sh`
- Do not run both sims at once

## Layout

```
install_and_run.sh                 # clone + install Classic + run demo
script/install_classic_beside_harmonic.sh
intelleswarm-ai/genai_framework/px4_gazebo_sim/
  run_pollination_simulation.py
  px4_multi_drone.sh
  agricultural_farm.world
```
