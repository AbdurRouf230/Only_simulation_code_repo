"""Grid mission (m / cm) with fleet collision avoidance."""

from __future__ import annotations

import time
from datetime import datetime

import rclpy

from import_paths import setup_import_paths

setup_import_paths(__file__)

from drone_controller import DroneController  # noqa: E402

from collision_avoidance import FleetRegistry  # noqa: E402
from multi_drone_config import (  # noqa: E402
    MISSION_GROUND_SIZE_M,
    MISSION_GRID,
    YAW_CAPTURE_STEP_DEG,
)


def mission_aborted(drone) -> bool:
    return bool(getattr(drone, "_abort_mission", False))


def sleep_unless_abort(drone, seconds: float, step: float = 0.2) -> bool:
    """Sleep; return False if the mission was aborted."""
    end = time.time() + max(0.0, seconds)
    while time.time() < end:
        if mission_aborted(drone):
            return False
        remaining = end - time.time()
        time.sleep(min(step, remaining) if remaining > 0 else 0.0)
    return not mission_aborted(drone)


def wait_until_reached(drone, x, y, tolerance=0.5, timeout=120.0):
    end = time.time() + timeout
    while time.time() < end and rclpy.ok():
        if mission_aborted(drone):
            return False
        rclpy.spin_once(drone, timeout_sec=0.1)
        if drone.reached_position(x, y, tolerance):
            return True
    print(f'  WARN: timeout waiting for ({x:.2f}, {y:.2f})')
    return False


def land_and_wait(drone, timeout=50.0) -> None:
    """Leave offboard hover, land, and disarm. Call after the grid/RTH."""
    label = getattr(drone, "drone_label", "drone")
    print(f'[{label}] Landing...')
    try:
        if drone.is_armed() or abs(getattr(drone, "current_z", 0.0)) > 0.4:
            if abs(getattr(drone, "current_z", 0.0)) > 1.0:
                drone.go_to(drone.current_x, drone.current_y, -0.6)
                end = time.time() + 20.0
                while time.time() < end and rclpy.ok():
                    rclpy.spin_once(drone, timeout_sec=0.1)
                    if abs(drone.current_z) <= 0.9:
                        break
            drone.land()
            end = time.time() + timeout
            last_land = time.time()
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(drone, timeout_sec=0.1)
                if not drone.is_armed():
                    print(f'[{label}] On ground (disarmed)')
                    return
                if abs(drone.current_z) < 0.25:
                    drone.disarm(force=True)
                if time.time() - last_land >= 3.0 and drone.is_armed():
                    drone.land()
                    last_land = time.time()
            if drone.is_armed():
                drone.disarm(force=True)
                for _ in range(15):
                    rclpy.spin_once(drone, timeout_sec=0.1)
        print(f'[{label}] Land command finished armed={drone.is_armed()} z={drone.current_z:.2f}')
    finally:
        try:
            drone.stop_streaming()
        except Exception:
            pass


def square_from_center(cx: float, cy: float, z: float) -> dict:
    half = MISSION_GROUND_SIZE_M / 2.0
    return {
        'cx': cx,
        'cy': cy,
        'z': z,
        'left_x': cx - half,
        'right_x': cx + half,
        'top_y': cy + half,
        'bottom_y': cy - half,
        'mid_y': cy,
    }


def leg_endpoints(col_idx: int, cols: int, top_y: float, bottom_y: float):
    last = cols - 1
    if col_idx == last:
        return bottom_y, top_y, 'UP'
    if col_idx % 2 == 0:
        return top_y, bottom_y, 'DOWN'
    return bottom_y, top_y, 'UP'


def m_yaw_scan_filename(drone_name: str, leg_num: int, cap_label: str, step_idx: int) -> str:
    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return f'mission_{drone_name}_leg{leg_num}_{cap_label}_{step_idx + 1}_{ts}.png'


def cm_capture_filename(drone_name: str, cm_count: int) -> str:
    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return f'cm_{drone_name}_image_{cm_count}_{ts}.png'


class FleetDrone:
    """Wraps DroneController with fleet registry + safe go_to."""

    def __init__(self, drone_id: int, controller: DroneController):
        self.drone_id = drone_id
        self.drone = controller
        self.registry = FleetRegistry()
        self.registry.register(drone_id)

    def __getattr__(self, name):
        return getattr(self.drone, name)

    def sync_pose(self) -> None:
        self.registry.update_pose(
            self.drone_id,
            self.drone.current_x,
            self.drone.current_y,
            self.drone.current_z,
        )

    def go_to_safe(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        if mission_aborted(self.drone):
            return
        self.sync_pose()
        sx, sy, sz, delay = self.registry.resolve_waypoint(self.drone_id, x, y, z)
        if delay > 0:
            print(
                f'[Drone {self.drone_id}] collision wait {delay:.1f}s '
                f'before ({sx:.1f},{sy:.1f},{sz:.1f})'
            )
            if not sleep_unless_abort(self.drone, delay):
                return
        if sz != z:
            print(
                f'[Drone {self.drone_id}] altitude adjust z={z:.1f} -> {sz:.1f} '
                f'to avoid fleet conflict'
            )
        self.drone.go_to(sx, sy, sz, yaw)
        self.sync_pose()

    def close(self) -> None:
        self.registry.unregister(self.drone_id)


def m_scan_at_midpoint(fleet: FleetDrone, px, py, z, leg_num: int, drone_name: str):
    print(
        f'\n[Drone {fleet.drone_id}] Leg {leg_num} HOLD ({px:.2f}, {py:.2f}) — '
        f'yaw 360°, Cap1–Cap4'
    )

    def filename_fn(cap_label, offset_deg, step_idx):
        del offset_deg
        return m_yaw_scan_filename(drone_name, leg_num, cap_label, step_idx)

    return fleet.drone.yaw_360_capture_every_90(
        px, py, z,
        filename_fn=filename_fn,
        step_deg=YAW_CAPTURE_STEP_DEG,
        clockwise=True,
    )


def cm_capture_at_midpoint(fleet: FleetDrone, px, py, z, drone_name: str, cm_count: int):
    print(f'\n[Drone {fleet.drone_id}] Capture at ({px:.2f}, {py:.2f}) — level hold')
    fleet.drone.hold()
    filename = cm_capture_filename(drone_name, cm_count)
    return fleet.drone.capture_image(filename=filename)


def run_grid_mission_fleet(
    fleet: FleetDrone,
    center: tuple[float, float, float],
    return_location: tuple[float, float, float],
    mission_mode: str,
    drone_name: str,
) -> None:
    cx, cy, z = center
    square = square_from_center(cx, cy, z)
    left_x = square['left_x']
    right_x = square['right_x']
    top_y = square['top_y']
    bottom_y = square['bottom_y']
    mid_y = square['mid_y']
    cols = MISSION_GRID
    x_step = (right_x - left_x) / (cols - 1)

    label = 'M' if mission_mode == 'm' else 'CM'
    print(
        f'\n[Drone {fleet.drone_id}] {label} mission center ({cx:.1f},{cy:.1f}) '
        f'→ TOP-LEFT to TOP-RIGHT'
    )

    fleet.go_to_safe(left_x, top_y, z)
    wait_until_reached(fleet.drone, left_x, top_y)

    cm_count = 0
    for col_idx in range(cols):
        if mission_aborted(fleet.drone):
            break
        x_col = left_x + col_idx * x_step
        y_start, y_end, leg_dir = leg_endpoints(col_idx, cols, top_y, bottom_y)

        if col_idx > 0:
            fleet.go_to_safe(x_col, y_start, z)
            if mission_aborted(fleet.drone) or not wait_until_reached(fleet.drone, x_col, y_start):
                if mission_aborted(fleet.drone):
                    break

        print(f'[Drone {fleet.drone_id}] Leg {col_idx + 1}/{cols} {leg_dir}')
        fleet.go_to_safe(x_col, mid_y, z)
        wait_until_reached(fleet.drone, x_col, mid_y, tolerance=0.35)
        if mission_aborted(fleet.drone):
            break
        fleet.sync_pose()

        if mission_mode == 'm':
            m_scan_at_midpoint(fleet, x_col, mid_y, z, col_idx + 1, drone_name)
        else:
            cm_count += 1
            cm_capture_at_midpoint(fleet, x_col, mid_y, z, drone_name, cm_count)
        if mission_aborted(fleet.drone):
            break

        fleet.go_to_safe(x_col, y_end, z)
        wait_until_reached(fleet.drone, x_col, y_end)
        if mission_aborted(fleet.drone):
            break
        fleet.sync_pose()

    if not mission_aborted(fleet.drone):
        print(f'[Drone {fleet.drone_id}] Return home')
        fleet.go_to_safe(return_location[0], return_location[1], z)
        wait_until_reached(fleet.drone, return_location[0], return_location[1])
    land_and_wait(fleet.drone)
    print(f'[Drone {fleet.drone_id}] {label} mission complete')
