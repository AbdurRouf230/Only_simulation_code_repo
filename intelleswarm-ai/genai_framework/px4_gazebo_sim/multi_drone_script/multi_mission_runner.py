#!/usr/bin/env python3
"""Run m or cm mission for multiple drones with separate field centers.

Usage:
  python3 multi_mission_runner.py m \\
    --centers "0 15 -10" "6 15 -10" "12 15 -10" "18 15 -10"

  python3 multi_mission_runner.py cm --file centers.txt

centers.txt (one x y z per line):
  0 15 -10
  6 15 -10
  12 15 -10
  18 15 -10

Each drone returns to its position before the mission started.
"""

from __future__ import annotations

import argparse
import threading
import time
import traceback
from pathlib import Path

import rclpy

from import_paths import setup_import_paths
from ros_thread_spin import install_thread_safe_spin

setup_import_paths(__file__)

from drone_controller import DroneController  # noqa: E402
from mission_logic import (  # noqa: E402
    FleetDrone,
    land_and_wait,
    mission_aborted,
    run_grid_mission_fleet,
    sleep_unless_abort,
)


def parse_center(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(f'Expected "x y z", got: {text!r}')
    return float(parts[0]), float(parts[1]), float(parts[2])


def load_centers(args) -> list[tuple[float, float, float]]:
    if args.file:
        lines = Path(args.file).read_text(encoding='utf-8').splitlines()
        centers = [parse_center(ln.strip()) for ln in lines if ln.strip() and not ln.startswith('#')]
    else:
        centers = [parse_center(c) for c in args.centers]
    return centers


def mission_thread(
    fleet: FleetDrone,
    center: tuple[float, float, float],
    home: tuple[float, float, float],
    mode: str,
    start_delay: float,
) -> None:
    try:
        if not sleep_unless_abort(fleet.drone, start_delay):
            land_and_wait(fleet.drone)
            fleet.close()
            return
        drone_name = f'drone{fleet.drone_id}'
        print(f'[Drone {fleet.drone_id}] starting {mode.upper()} after {start_delay:.0f}s delay')
        ground_xy = (home[0], home[1])
        if not fleet.drone.is_armed() or not fleet.drone.is_offboard():
            fleet.drone.takeoff(abs(center[2]))
            sleep_unless_abort(fleet.drone, 2.0)
            if mission_aborted(fleet.drone):
                land_and_wait(fleet.drone)
                fleet.close()
                return
            ground_xy = (fleet.drone.current_x, fleet.drone.current_y)
        # Return home XY at mission altitude, then land — do not keep airborne z as home.
        home_xy_alt = (ground_xy[0], ground_xy[1], center[2])
        run_grid_mission_fleet(fleet, center, home_xy_alt, mode, drone_name)
        fleet.close()
    except Exception as exc:
        print(f'[Drone {fleet.drone_id}] MISSION FAILED: {exc}')
        traceback.print_exc()
        try:
            land_and_wait(fleet.drone)
        except Exception:
            pass
        try:
            fleet.close()
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description='Multi-drone m/cm mission runner')
    parser.add_argument('mode', choices=['m', 'cm'], help='Mission type')
    parser.add_argument(
        '--centers', nargs='+', help='One "x y z" center per drone (quoted strings)',
    )
    parser.add_argument('--file', help='File with one x y z center per line')
    parser.add_argument(
        '--stagger', type=float, default=2.0,
        help='Seconds between each drone mission start (default 2)',
    )
    args = parser.parse_args()

    if not args.centers and not args.file:
        parser.error('Provide --centers or --file')

    centers = load_centers(args)
    n = len(centers)
    print(f'Running {args.mode.upper()} for {n} drone(s)')

    rclpy.init()
    install_thread_safe_spin()

    fleets: list[FleetDrone] = []
    homes: list[tuple[float, float, float]] = []

    base = Path(__file__).resolve().parent / 'captures'
    for i in range(n):
        cap_dir = base / f'drone{i}'
        ctrl = DroneController(
            namespace=f'/px4_{i}',
            target_sys=i + 1,
            drone_label=f'Drone {i}',
            instance_id=i,
            capture_dir=str(cap_dir),
        )
        fleet = FleetDrone(i, ctrl)
        fleets.append(fleet)
        for _ in range(30):
            rclpy.spin_once(ctrl, timeout_sec=0.1)
        homes.append((ctrl.current_x, ctrl.current_y, ctrl.current_z))

    threads: list[threading.Thread] = []
    for i, (center, home) in enumerate(zip(centers, homes)):
        delay = i * args.stagger
        t = threading.Thread(
            target=mission_thread,
            args=(fleets[i], center, home, args.mode, delay),
            name=f'mission_drone_{i}',
        )
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    for f in fleets:
        try:
            f.drone.destroy_node()
        except Exception:
            pass
    rclpy.shutdown()
    print('All missions finished.')


if __name__ == '__main__':
    main()
