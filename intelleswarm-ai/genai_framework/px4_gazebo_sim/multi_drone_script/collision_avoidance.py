"""Simple fleet registry for 2x2x2 m volume collision avoidance."""

from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass

from multi_drone_config import (
    ALTITUDE_STEP_M,
    FLEET_STATE_PATH,
    MAX_ALTITUDE_STEPS,
    SAFETY_BOX_M,
    WAYPOINT_DELAY_BASE_S,
)


@dataclass
class FleetPose:
    x: float
    y: float
    z: float
    updated_at: float


_FLEET_FILE_LOCK = threading.Lock()


class FleetRegistry:
    """File-backed poses so separate Python processes can avoid each other."""

    def __init__(self, path: str = FLEET_STATE_PATH):
        self.path = path

    def _load(self) -> dict:
        if not os.path.isfile(self.path):
            return {}
        try:
            with open(self.path, 'r', encoding='utf-8') as fh:
                return json.load(fh)
        except (json.JSONDecodeError, OSError):
            return {}

    def _save(self, data: dict) -> None:
        directory = os.path.dirname(self.path) or '/tmp'
        os.makedirs(directory, exist_ok=True)
        tmp = f'{self.path}.{os.getpid()}.{threading.get_ident()}.tmp'
        with _FLEET_FILE_LOCK:
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(data, fh, indent=2)
                fh.flush()
                os.fsync(fh.fileno())
            try:
                os.replace(tmp, self.path)
            except FileNotFoundError:
                try:
                    os.remove(tmp)
                except OSError:
                    pass

    def register(self, drone_id: int) -> None:
        data = self._load()
        key = str(drone_id)
        if key not in data:
            data[key] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'updated_at': time.time()}
            self._save(data)

    def unregister(self, drone_id: int) -> None:
        data = self._load()
        data.pop(str(drone_id), None)
        self._save(data)

    def update_pose(self, drone_id: int, x: float, y: float, z: float) -> None:
        data = self._load()
        data[str(drone_id)] = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'updated_at': time.time(),
        }
        self._save(data)

    @staticmethod
    def _boxes_overlap(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
        box_size: float,
    ) -> bool:
        """True if two 2x2x2 cubes (side box_size) centered at a and b overlap."""
        return (
            abs(a[0] - b[0]) < box_size
            and abs(a[1] - b[1]) < box_size
            and abs(a[2] - b[2]) < box_size
        )

    def _conflict_at(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float,
        box_size: float = SAFETY_BOX_M,
    ) -> bool:
        data = self._load()
        target = (x, y, z)
        for key, pose in data.items():
            if int(key) == drone_id:
                continue
            other = (pose['x'], pose['y'], pose['z'])
            if self._boxes_overlap(target, other, box_size):
                return True
        return False

    def resolve_waypoint(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float,
    ) -> tuple[float, float, float, float]:
        """Return (x, y, z, delay_sec). Raises altitude or adds delay if blocked."""
        if not self._conflict_at(drone_id, x, y, z):
            return x, y, z, 0.0

        for step in range(1, MAX_ALTITUDE_STEPS + 1):
            z_up = z - step * ALTITUDE_STEP_M
            if not self._conflict_at(drone_id, x, y, z_up):
                return x, y, z_up, 0.0

        delay = WAYPOINT_DELAY_BASE_S * (1 + drone_id * 0.5)
        return x, y, z, delay
