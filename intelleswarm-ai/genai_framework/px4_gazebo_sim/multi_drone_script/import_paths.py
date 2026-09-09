"""Resolve drone_controller import paths for Windows and WSL layouts."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _known_controller_dirs() -> list[Path]:
    home = Path.home()
    return [
        home / "ros2_ws" / "src" / "px4_python" / "MultiDrone",
        Path("/home/rouf/ros2_ws/src/px4_python/MultiDrone"),
        Path("/mnt/e/Multi Drone project/Main setup/main_used_code_in_script"),
        Path("/mnt/e/Multi Drone project/Main setup/Main_codes"),
    ]


def _walk_up_controller_dirs(start: Path) -> list[Path]:
    found: list[Path] = []
    for parent in [start, *start.parents]:
        found.append(parent / "main_used_code_in_script")
        found.append(parent / "MultiDrone")
        found.append(parent / "px4_python" / "MultiDrone")
    return found


def _controller_already_on_path() -> Path | None:
    for entry in sys.path:
        if not entry:
            continue
        candidate = Path(entry) / "drone_controller.py"
        if candidate.is_file():
            return candidate.parent
    return None


def setup_import_paths(script_file: str | Path) -> tuple[Path, Path]:
    """Add multi_drone_script and drone_controller dirs to sys.path.

    Returns (multi_root, controller_root).
    """
    script = Path(script_file).resolve()
    multi_root = script.parent
    if multi_root.name == "generated":
        multi_root = multi_root.parent

    search_roots = [
        multi_root,  # vendored drone_controller.py in this repo
        multi_root / "MultiDrone",
        multi_root.parent / "main_used_code_in_script",
        multi_root.parent / "MultiDrone",
        multi_root / "main_used_code_in_script",
        *_known_controller_dirs(),
        *_walk_up_controller_dirs(multi_root),
    ]

    controller_root = None
    seen: set[Path] = set()
    for candidate in search_roots:
        try:
            resolved = candidate.resolve()
        except OSError:
            continue
        if resolved in seen:
            continue
        seen.add(resolved)
        if (resolved / "drone_controller.py").is_file():
            controller_root = resolved
            break

    if controller_root is None:
        controller_root = _controller_already_on_path()

    if controller_root is None:
        env_dir = os.environ.get("DRONE_CONTROLLER_DIR", "").strip()
        if env_dir and (Path(env_dir) / "drone_controller.py").is_file():
            controller_root = Path(env_dir)

    if controller_root is None:
        tried = ", ".join(str(p) for p in search_roots[:8])
        raise ImportError(
            "drone_controller.py not found. Tried: "
            f"{tried}. Set DRONE_CONTROLLER_DIR or keep it in "
            "~/ros2_ws/src/px4_python/MultiDrone or "
            "Main setup/main_used_code_in_script."
        )

    for path in (str(multi_root), str(controller_root)):
        if path not in sys.path:
            sys.path.insert(0, path)

    return multi_root, controller_root
